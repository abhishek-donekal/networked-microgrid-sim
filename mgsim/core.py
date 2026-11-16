"""Simulation core: DC network solver, profiles, simulator loop and results.

Sign convention used everywhere: power / current is POSITIVE when it flows
INTO a DC bus (a source discharging is positive, a load is negative).
"""
from __future__ import annotations

import copy
import csv
import math
import pickle
from bisect import insort

import numpy as np


# --------------------------------------------------------------------------
# Profiles
# --------------------------------------------------------------------------
class Profile:
    """Time profile built from (time, value) points.

    kind="step"   : value holds until the next point (load steps).
    kind="linear" : linear interpolation between points (irradiance, wind).
    """

    def __init__(self, points, kind="step"):
        pts = sorted((float(t), float(v)) for t, v in points)
        self.t = np.array([p[0] for p in pts])
        self.v = np.array([p[1] for p in pts])
        self.kind = kind

    def __call__(self, t):
        if self.kind == "linear":
            return float(np.interp(t, self.t, self.v))
        i = int(np.searchsorted(self.t, t, side="right")) - 1
        return float(self.v[max(i, 0)])


class Const:
    """Constant profile (a class rather than a lambda so it can be pickled)."""

    def __init__(self, value):
        self.value = float(value)

    def __call__(self, t):
        return self.value


def as_profile(x):
    """Accept a number, a list of (t, value) points, or any callable f(t)."""
    if callable(x):
        return x
    if isinstance(x, (int, float)):
        return Const(x)
    return Profile(x)


def lag(dt, tau):
    """Exact discretisation factor of a first-order lag (stable for any dt)."""
    return 1.0 if tau <= 0 else 1.0 - math.exp(-dt / tau)


# --------------------------------------------------------------------------
# DC network
# --------------------------------------------------------------------------
class DCNetwork:
    """DC buses (each with a capacitance) joined by R-L lines with breakers.

    Solved the way EMT programs do it: every line and capacitor is replaced by
    a companion model (a conductance plus a history current) and the node
    voltages come from one nodal solve per step.

    method="trapezoidal" : second-order accurate, the EMT standard. Use for
                           transient studies (dt around 1e-4 s or smaller).
    method="euler"       : backward Euler. First-order but heavily damped;
                           use for long energy-management runs with large dt.
    """

    def __init__(self, method="trapezoidal"):
        self.method = method
        self.names = []
        self.index = {}
        self._C = []
        self._v0 = []
        self.lines = {}          # name -> dict(a, b, R, L, closed, i)
        self.v = None
        self._dirty = True
        self._Ainv = None
        self._dt = None

    def add_node(self, name, C, v0):
        self.index[name] = len(self.names)
        self.names.append(name)
        self._C.append(C)
        self._v0.append(v0)
        self.v = np.array(self._v0, dtype=float)

    def add_line(self, name, a, b, R, L=0.0, closed=True):
        self.lines[name] = {"a": self.index[a], "b": self.index[b], "R": R, "L": L,
                            "closed": closed, "i": 0.0}
        self._dirty = True

    def set_breaker(self, name, closed):
        ln = self.lines[name]
        ln["closed"] = bool(closed)
        if not closed:
            ln["i"] = 0.0        # ideal breaker: current interrupted at once
        self._dirty = True

    def _build(self, dt):
        k = 2.0 if self.method == "trapezoidal" else 1.0
        C = np.array(self._C)
        A = np.diag(k * C / dt)
        for ln in self.lines.values():
            ln["g"] = 1.0 / (ln["R"] + k * ln["L"] / dt)
            if ln["closed"]:
                g, a, b = ln["g"], ln["a"], ln["b"]
                A[a, a] += g
                A[b, b] += g
                A[a, b] -= g
                A[b, a] -= g
        self._Ainv = np.linalg.inv(A)
        self._Cdt = k * C / dt
        self._dt = dt
        self._dirty = False

    def step(self, i_inj, dt):
        if self._dirty or dt != self._dt:
            self._build(dt)
        v = self.v
        trap = self.method == "trapezoidal"
        rhs = self._Cdt * v + (2.0 * i_inj if trap else i_inj)
        closed = [ln for ln in self.lines.values() if ln["closed"]]
        for ln in closed:
            a, b, g, i_old = ln["a"], ln["b"], ln["g"], ln["i"]
            if trap:
                h = g * ((v[a] - v[b]) + (2.0 * ln["L"] / dt - ln["R"]) * i_old)
                src = h + i_old
            else:
                h = g * (ln["L"] / dt) * i_old
                src = h
            ln["h"] = h
            rhs[a] -= src
            rhs[b] += src
        self.v = self._Ainv @ rhs
        np.maximum(self.v, 0.0, out=self.v)
        for ln in closed:
            ln["i"] = ln["g"] * (self.v[ln["a"]] - self.v[ln["b"]]) + ln["h"]

    def line_current(self, name):
        """Current from node a to node b (A)."""
        ln = self.lines[name]
        return ln["i"] if ln["closed"] else 0.0

    def line_loss(self, name):
        ln = self.lines[name]
        return self.line_current(name) ** 2 * ln["R"]


# --------------------------------------------------------------------------
# Results
# --------------------------------------------------------------------------
class Results:
    """Recorded signals. Access with res["v.MG1"]; time vector is res.t."""

    def __init__(self, t, signals):
        self.t = np.asarray(t)
        self.signals = {k: np.asarray(v) for k, v in signals.items()}

    def __getitem__(self, key):
        return self.signals[key]

    def keys(self, prefix=""):
        return [k for k in self.signals if k.startswith(prefix)]

    def at(self, key, t):
        """Value of a signal at time t (nearest recorded sample)."""
        return float(self.signals[key][int(np.argmin(np.abs(self.t - t)))])

    def to_csv(self, path):
        keys = list(self.signals)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + keys)
            for i, t in enumerate(self.t):
                w.writerow([f"{t:.6f}"] + [f"{self.signals[k][i]:.6g}" for k in keys])

    def plot(self, panels, path=None, title=None, scale=None):
        """Quick multi-panel plot.

        panels : list of (y-label, [signal names or prefixes ending in '*'])
        scale  : optional dict y-label -> multiplier (e.g. 1e-3 for kW)
        """
        import matplotlib
        if path:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(len(panels), 1, sharex=True,
                                 figsize=(11, 2.6 * len(panels)), squeeze=False)
        for ax, (ylabel, names) in zip(axes[:, 0], panels):
            k = (scale or {}).get(ylabel, 1.0)
            for name in names:
                matches = self.keys(name[:-1]) if name.endswith("*") else [name]
                for m in matches:
                    ax.plot(self.t, self.signals[m] * k, label=m, linewidth=1.3)
            ax.set_ylabel(ylabel)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7, ncol=4, loc="best")
        axes[-1, 0].set_xlabel("time (s)")
        if title:
            fig.suptitle(title)
        fig.tight_layout()
        if path:
            fig.savefig(path, dpi=130)
            plt.close(fig)
        return fig


# --------------------------------------------------------------------------
# Simulator
# --------------------------------------------------------------------------
class Simulator:
    """Fixed-step time-domain simulator.

    dt           : integration step (s). 1e-4 for transients; up to ~1e-2 for
                   long energy-management runs.
    record_every : store one sample every N steps.
    """

    def __init__(self, dc, ac=None, dt=1e-4, record_every=10):
        self.dc = dc
        self.ac = ac
        self.dt = dt
        self.record_every = record_every
        self.t = 0.0
        self.dev = {}             # name -> DC device
        self._events = []         # (t, seq, fn)
        self._controllers = []    # [period, fn, next_t]
        self._seq = 0
        self._step = 0
        self._log_t = []
        self._log = {}
        self._recording = True

    # ---- steady state and snapshots ---------------------------------------
    def initialize(self, t_settle=3.0):
        """Settle to steady state before t = 0, so runs start without a
        start-up transient. Runs the model for t_settle seconds of negative
        time with recording off, then restores every state of charge and
        hydrogen counter and clears the log. Events at t >= 0 do not fire.
        """
        keep = [(d, a, getattr(d, a)) for d in self.dev.values()
                for a in ("soc", "h2_kg") if hasattr(d, a)]
        self._recording = False
        self.t = -float(t_settle)
        for c in self._controllers:
            c[2] = self.t
        self.run(0.0)
        self.t, self._step = 0.0, 0
        for d, a, val in keep:
            setattr(d, a, val)
        self._recording = True
        self._log_t, self._log = [], {}
        return self

    def snapshot(self):
        """Independent copy of the whole simulation state. Settle once, then
        branch many what-if cases from the same starting point:
            base = build_system(); base.initialize()
            case = base.snapshot(); case.at(1.0, ...); res = case.run(5.0)
        """
        return copy.deepcopy(self)

    def save(self, path):
        """Write the state to disk. Controllers and events must be picklable
        (module-level functions or classes, not lambdas)."""
        with open(path, "wb") as f:
            pickle.dump(self, f)

    @staticmethod
    def load(path):
        with open(path, "rb") as f:
            return pickle.load(f)

    # ---- building -------------------------------------------------------
    def add(self, device):
        device.node_idx = self.dc.index[device.node]
        self.dev[device.name] = device
        return device

    def at(self, t, fn):
        """Schedule fn(sim) at time t (breaker operations, faults, setpoints)."""
        self._seq += 1
        insort(self._events, (float(t), self._seq, fn))

    def every(self, period, fn):
        """Run fn(sim) periodically: the hook for secondary control / EMS."""
        self._controllers.append([float(period), fn, self.t])

    def breaker(self, name, closed):
        """Open/close a breaker by name: DC line, AC line, device or IBR."""
        if name in self.dc.lines:
            self.dc.set_breaker(name, closed)
        elif self.ac is not None and name in self.ac.lines:
            self.ac.set_breaker(name, closed)
        elif name in self.dev:
            self.dev[name].connected = bool(closed)
        elif self.ac is not None and name in self.ac.ibr:
            self.ac.ibr[name].connected = bool(closed)
        elif self.ac is not None and name in self.ac.loads:
            self.ac.loads[name].connected = bool(closed)
        else:
            raise KeyError(f"no breaker / device named {name!r}")

    def v(self, node):
        return float(self.dc.v[self.dc.index[node]])

    # ---- running --------------------------------------------------------
    def _record(self):
        self._log_t.append(self.t)
        row = {}
        for name, i in self.dc.index.items():
            row[f"v.{name}"] = self.dc.v[i]
        for name in self.dc.lines:
            row[f"i.{name}"] = self.dc.line_current(name)
            row[f"closed.{name}"] = 1.0 if self.dc.lines[name]["closed"] else 0.0
        for d in self.dev.values():
            d.signals(row)
        if self.ac is not None:
            self.ac.signals(row)
        for k, val in row.items():
            self._log.setdefault(k, []).append(val)

    def run(self, t_end):
        dc, dt = self.dc, self.dt
        n = len(dc.names)
        devices = list(self.dev.values())
        steps = int(round((t_end - self.t) / dt))
        for _ in range(steps):
            t = self.t
            while self._events and self._events[0][0] <= t + 1e-12:
                self._events.pop(0)[2](self)
            for c in self._controllers:
                if t + 1e-12 >= c[2]:
                    c[1](self)
                    c[2] += c[0]

            v = dc.v
            inj = np.zeros(n)
            for d in devices:
                inj[d.node_idx] += d.step(t, v[d.node_idx], dt)
            if self.ac is not None:
                self.ac.step(t, dt, dc, inj)
            dc.step(inj, dt)

            self.t = t + dt
            self._step += 1
            if self._recording and self._step % self.record_every == 0:
                self._record()
        return self.results()

    def results(self):
        return Results(self._log_t, self._log)
