"""DC-side devices: droop-controlled sources, renewables and loads.

Every device is an averaged converter model: a controlled current source with
a first-order response. No switching ripple is modelled.

Each device exposes
    step(t, v, dt) -> current injected into its bus (A)
    signals(row)   -> adds its recorded quantities to the log row
"""
from __future__ import annotations

from .core import as_profile, lag

H2_LHV_WH_PER_KG = 33_330.0


class Device:
    def __init__(self, name, node):
        self.name = name
        self.node = node
        self.node_idx = None
        self.connected = True     # the device's own switch
        self.i = 0.0              # current into the bus (A)
        self.p = 0.0              # power into the bus (W)

    def signals(self, row):
        row[f"p.{self.name}"] = self.p


class DroopSource(Device):
    """Dispatchable source under V-I droop:  i_ref = (v_nom + dv_sec - v) / r_droop

    p_max / p_min : discharge / charge limits in W (p_min = 0 if unidirectional)
    droop         : per-unit voltage drop at rated power (0.05 = 5 %)
    tau           : converter current-loop time constant (s)
    ramp          : optional power ramp-rate limit (W/s)
    hp_tau        : if set, the droop response is high-pass filtered so the
                    source only supplies transients (supercapacitor behaviour)
    dv_sec        : secondary-control voltage shift (V), written by controllers
    p_bias        : extra power setpoint (W), written by an EMS
    """

    def __init__(self, name, node, p_max, p_min=0.0, v_nom=750.0, droop=0.05,
                 tau=0.005, ramp=None, hp_tau=None):
        super().__init__(name, node)
        self.p_max, self.p_min = p_max, p_min
        self.v_nom = v_nom
        self.r_droop = droop * v_nom ** 2 / p_max
        self.tau = tau
        self.ramp = ramp
        self.hp_tau = hp_tau
        self.dv_sec = 0.0
        self.p_bias = 0.0
        self._lp = 0.0

    def limits(self):
        """(p_min, p_max) available right now. Storage overrides this."""
        return self.p_min, self.p_max

    def extra_ref(self, v):
        return 0.0

    def step(self, t, v, dt):
        if not self.connected or v < 1.0:
            self.i = self.p = 0.0
            self._lp = 0.0
            self.integrate(dt)
            return 0.0
        i_ref = (self.v_nom + self.dv_sec - v) / self.r_droop
        if self.hp_tau:
            self._lp += lag(dt, self.hp_tau) * (i_ref - self._lp)
            i_ref -= self._lp
        i_ref += (self.p_bias + self.extra_ref(v)) / v
        p_lo, p_hi = self.limits()
        i_ref = min(max(i_ref, p_lo / v), p_hi / v)
        i_new = self.i + lag(dt, self.tau) * (i_ref - self.i)
        if self.ramp:
            di = self.ramp * dt / v
            i_new = min(max(i_new, self.i - di), self.i + di)
        self.i = i_new
        self.p = self.i * v
        self.integrate(dt)
        return self.i

    def integrate(self, dt):
        pass


class Battery(DroopSource):
    """Bidirectional storage with coulomb-counted state of charge."""

    def __init__(self, name, node, p_max, energy_wh, soc=0.6, p_charge_max=None,
                 soc_min=0.1, soc_max=0.95, eta=0.96, **kw):
        p_min = -(p_charge_max if p_charge_max is not None else p_max)
        super().__init__(name, node, p_max, p_min=p_min, **kw)
        self.energy_wh = energy_wh
        self.soc = soc
        self.soc_min, self.soc_max = soc_min, soc_max
        self.eta = eta

    def limits(self):
        lo = 0.0 if self.soc >= self.soc_max else self.p_min
        hi = 0.0 if self.soc <= self.soc_min else self.p_max
        return lo, hi

    def integrate(self, dt):
        p_cell = self.p / self.eta if self.p > 0 else self.p * self.eta
        self.soc -= p_cell * dt / (self.energy_wh * 3600.0)
        self.soc = min(max(self.soc, 0.0), 1.0)

    def signals(self, row):
        row[f"p.{self.name}"] = self.p
        row[f"soc.{self.name}"] = self.soc


class SuperCap(Battery):
    """Small, fast storage: supplies transients only, then recharges slowly.

    The droop term is high-pass filtered (hp_tau) and a slow loop pulls the
    state of charge back to soc_ref.
    """

    def __init__(self, name, node, p_max, energy_wh, soc=0.75, soc_ref=0.75,
                 k_soc=2.0, hp_tau=0.5, droop=0.01, tau=0.001, **kw):
        super().__init__(name, node, p_max, energy_wh, soc=soc, droop=droop,
                         tau=tau, hp_tau=hp_tau, soc_min=0.25, soc_max=1.0,
                         eta=0.98, **kw)
        self.soc_ref = soc_ref
        self.k_soc = k_soc

    def extra_ref(self, v):
        return -self.k_soc * (self.soc_ref - self.soc) * self.p_max


class FuelCell(DroopSource):
    """Unidirectional, slow source. Tracks hydrogen consumption."""

    def __init__(self, name, node, p_max, tau=0.3, ramp=None, efficiency=0.5, **kw):
        ramp = ramp if ramp is not None else 0.2 * p_max     # 20 % of rating / s
        super().__init__(name, node, p_max, p_min=0.0, tau=tau, ramp=ramp, **kw)
        self.efficiency = efficiency
        self.h2_kg = 0.0

    def integrate(self, dt):
        self.h2_kg += self.p * dt / 3600.0 / (self.efficiency * H2_LHV_WH_PER_KG)

    def signals(self, row):
        row[f"p.{self.name}"] = self.p
        row[f"h2_kg.{self.name}"] = self.h2_kg


class EV(Battery):
    """Electric vehicle.

    mode="charge" : constant-power charging at p_charge until full. Charging
                    folds back linearly when the bus sags below 0.90 pu and
                    stops at 0.80 pu.
    mode="v2g"    : behaves like a droop-controlled battery.
    mode="idle"   : plugged in, no power.
    """

    def __init__(self, name, node, p_max, energy_wh, p_charge=None, soc=0.4,
                 mode="charge", **kw):
        super().__init__(name, node, p_max, energy_wh, soc=soc, **kw)
        self.p_charge = p_charge if p_charge is not None else p_max
        self.mode = mode

    def step(self, t, v, dt):
        if self.mode == "v2g":
            return super().step(t, v, dt)
        if not self.connected or v < 1.0:
            self.i = self.p = 0.0
            return 0.0
        p_ref = 0.0
        if self.mode == "charge" and self.soc < self.soc_max:
            fold = min(max((v / self.v_nom - 0.80) / 0.10, 0.0), 1.0)
            p_ref = -self.p_charge * fold
        self.i += lag(dt, self.tau) * (p_ref / v - self.i)
        self.p = self.i * v
        self.integrate(dt)
        return self.i


class Renewable(Device):
    """PV or wind at maximum power point: injects the available power.

    p_avail : W, as a number, a list of (t, W) points, or a callable f(t).
    Curtails linearly between v_curt[0] and v_curt[1] (pu) on over-voltage.
    """

    def __init__(self, name, node, p_avail, v_nom=750.0, tau=0.01,
                 v_curt=(1.05, 1.10)):
        super().__init__(name, node)
        self.p_avail = as_profile(p_avail)
        self.v_nom = v_nom
        self.tau = tau
        self.v_curt = v_curt
        self.curtail = 1.0        # EMS can set < 1 to curtail deliberately
        self.avail = 0.0

    def step(self, t, v, dt):
        self.avail = self.p_avail(t)
        if not self.connected or v < 1.0:
            self.i = self.p = 0.0
            return 0.0
        lo, hi = self.v_curt
        k = min(max((hi - v / self.v_nom) / (hi - lo), 0.0), 1.0)
        p_ref = self.avail * k * self.curtail
        self.i += lag(dt, self.tau) * (p_ref / v - self.i)
        self.p = self.i * v
        return self.i

    def signals(self, row):
        row[f"p.{self.name}"] = self.p
        row[f"avail.{self.name}"] = self.avail


class Load(Device):
    """Constant-power load. Below v_cp (pu) it behaves as a resistance so a
    collapsing bus decays to zero instead of diverging."""

    def __init__(self, name, node, p, v_nom=750.0, v_cp=0.7):
        super().__init__(name, node)
        self.demand = as_profile(p)
        self.v_min = v_cp * v_nom

    def step(self, t, v, dt):
        if not self.connected:
            self.i = self.p = 0.0
            return 0.0
        p = self.demand(t)
        self.i = -p / v if v >= self.v_min else -p * v / self.v_min ** 2
        self.p = self.i * v
        return self.i
