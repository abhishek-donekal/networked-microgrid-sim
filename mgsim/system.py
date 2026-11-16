"""The hybrid AC/DC networked microgrid from microgrid-architecture.jpg.

    Grid --[grid]-- MAIN ----[feeder_AC1]---- AC1 --IBR01-- DC1 --[feeder_MG1]-- MG1
                     |                                       |
                     |                                     [tie]
                     |                                       |
                     +-------[feeder_AC2]---- AC2 --IBR02-- DCC --[feeder_MG2]-- MG2
                                                             +----[feeder_MG3]-- MG3

Names in [brackets] are breakers: sim.breaker("tie", False) opens the tie-line.

ALL NUMBERS IN `DEFAULTS` ARE PLACEHOLDERS, not values from the lab testbed.
Replace them with the real ratings before drawing conclusions.
"""
from __future__ import annotations

import copy

from .ac import ACLoad, ACSide, IBR
from .core import DCNetwork, Simulator
from .devices import EV, Battery, FuelCell, Load, Renewable, SuperCap

kW = 1e3
kWh = 1e3

DEFAULTS = {
    "v_dc": 750.0,            # V, nominal DC bus voltage
    "f": 60.0,                # Hz
    "bus_C": 10e-3,           # F, lumped capacitance of each DC bus
    "solver": "trapezoidal",  # "trapezoidal" (EMT-style) or "euler" (long runs)
    "lines": {                # R in ohm, L in henry (cable inductance)
        "feeder_MG1": {"R": 0.02, "L": 60e-6},
        "feeder_MG2": {"R": 0.02, "L": 60e-6},
        "feeder_MG3": {"R": 0.03, "L": 90e-6},
        "tie":        {"R": 0.05, "L": 150e-6},
    },
    "ibr": {
        "IBR01": {"s_rated": 100 * kW},
        "IBR02": {"s_rated": 100 * kW},
    },
    "ac_loads": {             # W
        "ACLoad.common": 30 * kW, "ACLoad.1": 20 * kW, "ACLoad.2": 20 * kW,
    },
    "dc_common_load": 20 * kW,
    "MG1": {
        "load": 40 * kW,
        "Battery": {"p_max": 50 * kW, "energy_wh": 100 * kWh, "soc": 0.6},
        "FuelCell": {"p_max": 30 * kW},
        "EV": {"p_max": 11 * kW, "energy_wh": 60 * kWh, "soc": 0.4},
        "SolarPV": {"p_avail": 30 * kW},
        "Wind": {"p_avail": 15 * kW},
    },
    "MG2": {
        "load": 35 * kW,
        "SuperCap": {"p_max": 50 * kW, "energy_wh": 0.2 * kWh},
        "Battery": {"p_max": 40 * kW, "energy_wh": 80 * kWh, "soc": 0.6},
        "FuelCell": {"p_max": 30 * kW},
        "EV": {"p_max": 11 * kW, "energy_wh": 60 * kWh, "soc": 0.4},
    },
    "MG3": {
        "load": 30 * kW,
        "Battery": {"p_max": 40 * kW, "energy_wh": 80 * kWh, "soc": 0.6},
        "FuelCell": {"p_max": 30 * kW},
        "EV": {"p_max": 11 * kW, "energy_wh": 60 * kWh, "soc": 0.4},
    },
}

_KINDS = {"Battery": Battery, "FuelCell": FuelCell, "EV": EV,
          "SuperCap": SuperCap, "SolarPV": Renewable, "Wind": Renewable}


def default_params():
    """A deep copy of DEFAULTS that is safe to edit."""
    return copy.deepcopy(DEFAULTS)


def build_system(params=None, dt=1e-4, record_every=10):
    """Build the full system and return a ready-to-run Simulator."""
    p = params or default_params()
    v = p["v_dc"]

    dc = DCNetwork(method=p.get("solver", "trapezoidal"))
    for node in ("DC1", "DCC", "MG1", "MG2", "MG3"):
        dc.add_node(node, p["bus_C"], v)

    def line(name, a, b):
        ln = p["lines"][name]
        if isinstance(ln, dict):
            dc.add_line(name, a, b, ln["R"], ln.get("L", 0.0))
        else:                     # plain number = resistance only
            dc.add_line(name, a, b, float(ln))

    line("feeder_MG1", "DC1", "MG1")
    line("feeder_MG2", "DCC", "MG2")
    line("feeder_MG3", "DCC", "MG3")
    line("tie", "DC1", "DCC")

    ac = ACSide(f_grid=p["f"])
    for node in ("MAIN", "AC1", "AC2"):
        ac.add_node(node)
    ac.add_grid("MAIN", "grid")
    ac.add_line("feeder_AC1", "MAIN", "AC1")
    ac.add_line("feeder_AC2", "MAIN", "AC2")
    ac.add_load(ACLoad("ACLoad.common", "MAIN", p["ac_loads"]["ACLoad.common"]))
    ac.add_load(ACLoad("ACLoad.1", "AC1", p["ac_loads"]["ACLoad.1"]))
    ac.add_load(ACLoad("ACLoad.2", "AC2", p["ac_loads"]["ACLoad.2"]))
    ac.add_ibr(IBR("IBR01", "AC1", "DC1", v_nom=v, f0=p["f"], **p["ibr"]["IBR01"]))
    ac.add_ibr(IBR("IBR02", "AC2", "DCC", v_nom=v, f0=p["f"], **p["ibr"]["IBR02"]))

    sim = Simulator(dc, ac, dt=dt, record_every=record_every)
    sim.add(Load("DCLoad.common", "DCC", p["dc_common_load"], v_nom=v))
    for mg in ("MG1", "MG2", "MG3"):
        cfg = p[mg]
        sim.add(Load(f"{mg}.Load", mg, cfg["load"], v_nom=v))
        for kind, cls in _KINDS.items():
            if kind in cfg:
                sim.add(cls(f"{mg}.{kind}", mg, v_nom=v, **cfg[kind]))
    return sim
