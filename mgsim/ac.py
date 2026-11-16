"""AC side: utility grid, AC buses, AC loads and the interlinking converters.

The AC side is an ACTIVE-POWER (frequency) model. AC buses joined by closed
breakers form an island that shares one frequency; AC line impedance, voltage
magnitude and reactive power are not modelled.

Each island is one of:
  grid-tied : frequency = grid frequency, the grid is the slack. IBRs run
              grid-following (GFL).
  islanded  : IBRs run grid-forming (GFM) and share the AC load by P-f droop.
  dead      : no grid and no IBR. Loads are unserved.

IBR power convention: p_dc > 0 means AC -> DC (rectifying, feeding the DC bus).
"""
from __future__ import annotations

from .core import as_profile, lag


class ACLoad:
    def __init__(self, name, node, p):
        self.name, self.node = name, node
        self.demand = as_profile(p)
        self.connected = True
        self.p = 0.0          # served power (W)
        self.unserved = 0.0


class IBR:
    """Bidirectional AC/DC interlinking converter.

    gfl_mode (used when its AC island is grid-tied):
        "vdc_droop" : supports the DC bus,  p_dc = (v_nom - v_dc)/r * v_dc + p_set
        "pq"        : fixed dispatch,       p_dc = p_set
    When its AC island loses the grid the IBR switches to grid-forming with
    P-f droop, provided auto_gfm is True. Otherwise it trips.
    """

    def __init__(self, name, ac_node, dc_node, s_rated, v_nom=750.0, f0=60.0,
                 eta=0.97, droop_v=0.02, droop_f=0.01, tau=0.01, tau_f=0.05,
                 gfl_mode="vdc_droop", auto_gfm=True):
        self.name, self.ac_node, self.dc_node = name, ac_node, dc_node
        self.s_rated = s_rated
        self.v_nom, self.f0 = v_nom, f0
        self.eta = eta
        self.r_droop = droop_v * v_nom ** 2 / s_rated
        self.m_f = droop_f * f0 / s_rated          # Hz per W
        self.tau, self.tau_f = tau, tau_f
        self.gfl_mode = gfl_mode
        self.auto_gfm = auto_gfm
        self.p_set = 0.0
        self.dv_sec = 0.0
        self.connected = True
        self.mode = "GFL"         # actual state: GFL / GFM / OFF
        self.p_dc = 0.0           # power into the DC bus (W)
        self.p_ac = 0.0           # power drawn from the AC bus (W)
        self.f = f0

    def ac_from_dc(self, p_dc):
        return p_dc / self.eta if p_dc >= 0 else p_dc * self.eta

    def dc_from_ac_out(self, p_out):
        """DC power needed (negative injection) to deliver p_out to the AC bus."""
        return -p_out / self.eta if p_out >= 0 else -p_out * self.eta

    def capability(self, v_dc):
        """Deliverable AC power; folds back as the DC bus collapses."""
        k = min(max((v_dc / self.v_nom - 0.70) / 0.10, 0.0), 1.0)
        return self.s_rated * k


class ACSide:
    def __init__(self, f_grid=60.0):
        self.f_grid = f_grid
        self.grid_node = None
        self.grid_available = True      # set False to model a utility outage
        self.nodes = []
        self.lines = {}                 # name -> dict(a, b, closed)
        self.loads = {}
        self.ibr = {}
        self.p_grid = 0.0
        self.island = {}                # node -> island id
        self.state = {}                 # node -> "grid" / "island" / "dead"
        self.f_node = {}
        self._dirty = True
        self._groups = []

    # ---- building -------------------------------------------------------
    def add_node(self, name):
        self.nodes.append(name)
        self.f_node[name] = self.f_grid

    def add_grid(self, node, breaker_name="grid"):
        """Grid connects to `node` through a breaker called breaker_name."""
        self.grid_node = node
        self.lines[breaker_name] = {"a": "__grid__", "b": node, "closed": True}

    def add_line(self, name, a, b, closed=True):
        self.lines[name] = {"a": a, "b": b, "closed": closed}
        self._dirty = True

    def add_load(self, load):
        self.loads[load.name] = load
        return load

    def add_ibr(self, ibr):
        self.ibr[ibr.name] = ibr
        return ibr

    def set_breaker(self, name, closed):
        self.lines[name]["closed"] = bool(closed)
        self._dirty = True

    # ---- topology -------------------------------------------------------
    def _find_islands(self):
        parent = {n: n for n in self.nodes + ["__grid__"]}

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for ln in self.lines.values():
            if ln["closed"]:
                parent[find(ln["a"])] = find(ln["b"])
        groups = {}
        for n in self.nodes:
            groups.setdefault(find(n), []).append(n)
        g_root = find("__grid__")
        self._groups = [(nodes, root == g_root) for root, nodes in groups.items()]
        self._dirty = False

    # ---- one time step --------------------------------------------------
    def step(self, t, dt, dc, inj):
        if self._dirty:
            self._find_islands()
        self.p_grid = 0.0
        for nodes, has_grid in self._groups:
            loads = [l for l in self.loads.values() if l.node in nodes]
            ibrs = [c for c in self.ibr.values() if c.ac_node in nodes]
            demand = 0.0
            for l in loads:
                l.p = l.demand(t) if l.connected else 0.0
                l.unserved = 0.0
                demand += l.p

            grid_tied = has_grid and self.grid_available
            if grid_tied:
                f = self.f_grid
                p_draw = demand
                for c in ibrs:
                    v = dc.v[dc.index[c.dc_node]]
                    if not c.connected or v < 1.0:
                        c.mode, target = "OFF", 0.0
                    else:
                        c.mode = "GFL"
                        target = c.p_set
                        if c.gfl_mode == "vdc_droop":
                            target += (c.v_nom + c.dv_sec - v) / c.r_droop * v
                        target = min(max(target, -c.s_rated), c.s_rated)
                    c.p_dc += lag(dt, c.tau) * (target - c.p_dc)
                    c.p_ac = c.ac_from_dc(c.p_dc)
                    p_draw += c.p_ac
                self.p_grid += p_draw
                state = "grid"
            else:
                formers = []
                for c in ibrs:
                    v = dc.v[dc.index[c.dc_node]]
                    cap = c.capability(v) if (c.connected and c.auto_gfm) else 0.0
                    if cap > 0.0:
                        c.mode = "GFM"
                        formers.append((c, cap))
                    else:
                        c.mode = "OFF"
                        c.p_dc += lag(dt, c.tau) * (0.0 - c.p_dc)
                        c.p_ac = c.ac_from_dc(c.p_dc)
                if formers:
                    state = "island"
                    f, out, unserved = self._share(demand, formers)
                    for c, _ in formers:
                        target = c.dc_from_ac_out(out[c.name])
                        c.p_dc += lag(dt, c.tau) * (target - c.p_dc)
                        c.p_ac = c.ac_from_dc(c.p_dc)
                    if unserved > 0 and demand > 0:
                        k = unserved / demand
                        for l in loads:
                            l.unserved = l.p * k
                            l.p -= l.unserved
                else:
                    state, f = "dead", 0.0
                    for l in loads:
                        l.unserved, l.p = l.p, 0.0

            for c in ibrs:
                target_f = f if c.mode != "OFF" else (f if state != "dead" else 0.0)
                c.f += lag(dt, c.tau_f) * (target_f - c.f)
                v = dc.v[dc.index[c.dc_node]]
                if v >= 1.0:
                    inj[dc.index[c.dc_node]] += c.p_dc / v
            for n in nodes:
                self.state[n] = state
                self.f_node[n] += lag(dt, 0.05) * (f - self.f_node[n])

    @staticmethod
    def _share(demand, formers):
        """Split `demand` between grid-forming IBRs by P-f droop with limits.

        Returns (frequency, {name: AC output W}, unserved W).
        """
        out, active, remaining = {}, list(formers), demand
        f0 = formers[0][0].f0
        f = f0
        while active:
            k_sum = sum(1.0 / c.m_f for c, _ in active)
            f = f0 - remaining / k_sum
            over = [(c, cap) for c, cap in active if abs((f0 - f) / c.m_f) > cap]
            if not over:
                for c, _ in active:
                    out[c.name] = (f0 - f) / c.m_f
                return f, out, 0.0
            for c, cap in over:
                out[c.name] = cap if remaining > 0 else -cap
                remaining -= out[c.name]
                active.remove((c, cap))
        # every converter is at its limit
        unserved = max(remaining, 0.0)
        worst = max(c.m_f * abs(out[c.name]) for c, _ in formers)
        return f0 - worst if demand > 0 else f0 + worst, out, unserved

    # ---- logging --------------------------------------------------------
    def signals(self, row):
        row["p.grid"] = self.p_grid
        for name, ln in self.lines.items():
            row[f"closed.{name}"] = 1.0 if ln["closed"] else 0.0
        for n in self.nodes:
            row[f"f.{n}"] = self.f_node[n]
            row[f"energized.{n}"] = 0.0 if self.state.get(n) == "dead" else 1.0
        for c in self.ibr.values():
            row[f"p.{c.name}"] = c.p_dc
            row[f"gfm.{c.name}"] = 1.0 if c.mode == "GFM" else 0.0
        for l in self.loads.values():
            row[f"p.{l.name}"] = -l.p
            row[f"unserved.{l.name}"] = l.unserved
