"""Physical sanity checks. Run:  .venv\\Scripts\\python -m pytest -q"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mgsim import attach_voltage_restoration, build_system, default_params  # noqa: E402

DT = 2e-4


def dc_power_sum(sim):
    """Net power into the DC network minus line losses (W). ~0 in steady state."""
    total = sum(d.p for d in sim.dev.values())
    total += sum(c.p_dc for c in sim.ac.ibr.values())
    total -= sum(sim.dc.line_loss(n) for n in sim.dc.lines)
    return total


def test_dc_power_balance_grid_tied():
    sim = build_system(dt=DT)
    sim.run(4.0)
    throughput = sum(abs(d.p) for d in sim.dev.values())
    assert abs(dc_power_sum(sim)) < 0.005 * throughput


def test_ac_power_balance_grid_tied():
    sim = build_system(dt=DT)
    sim.run(1.0)
    ac = sim.ac
    expected = sum(l.p for l in ac.loads.values()) + sum(c.p_ac for c in ac.ibr.values())
    assert ac.p_grid == pytest.approx(expected, rel=1e-9)
    assert all(c.mode == "GFL" for c in ac.ibr.values())


def test_islanding_switches_to_grid_forming():
    sim = build_system(dt=DT)
    sim.at(1.0, lambda s: s.breaker("grid", False))
    res = sim.run(2.0)
    ac = sim.ac
    assert all(c.mode == "GFM" for c in ac.ibr.values())
    assert ac.p_grid == 0.0
    assert res.at("f.MAIN", 0.9) == pytest.approx(60.0)
    assert 59.0 < res.at("f.MAIN", 1.9) < 60.0
    # 70 kW of AC load, fully served, split equally between two equal IBRs
    assert sum(l.unserved for l in ac.loads.values()) == 0.0
    out = [-c.p_ac for c in ac.ibr.values()]
    assert sum(out) == pytest.approx(70e3, rel=1e-3)
    assert out[0] == pytest.approx(out[1], rel=1e-3)


def test_ac_bus_without_any_source_is_dead():
    sim = build_system(dt=DT)
    sim.at(0.5, lambda s: (s.breaker("grid", False), s.breaker("IBR01", False),
                           s.breaker("IBR02", False)))
    sim.run(1.0)
    assert sim.ac.state["MAIN"] == "dead"
    assert sim.ac.loads["ACLoad.common"].unserved == pytest.approx(30e3)


def test_isolated_microgrid_holds_voltage_and_restores():
    sim = build_system(dt=DT)
    sim.at(0.5, lambda s: s.breaker("feeder_MG3", False))
    sim.run(3.0)
    v_droop = sim.v("MG3")
    assert 0.90 * 750 < v_droop < 750          # droop alone: sags but holds

    sim2 = build_system(dt=DT)
    attach_voltage_restoration(sim2, nodes=("MG3",))
    sim2.at(0.5, lambda s: s.breaker("feeder_MG3", False))
    sim2.run(6.0)
    assert abs(sim2.v("MG3") - 750) < abs(v_droop - 750) / 4


def test_open_tie_line_carries_no_current():
    sim = build_system(dt=DT)
    sim.at(0.5, lambda s: s.breaker("tie", False))
    res = sim.run(1.0)
    assert abs(res.at("i.tie", 0.4)) > 1.0
    assert res.at("i.tie", 0.9) == 0.0


def test_battery_soc_follows_power():
    p = default_params()
    sim = build_system(p, dt=DT)
    sim.at(0.2, lambda s: s.breaker("feeder_MG3", False))   # MG3 battery must discharge
    soc0 = sim.dev["MG3.Battery"].soc
    sim.run(2.0)
    assert sim.dev["MG3.Battery"].p > 0
    assert sim.dev["MG3.Battery"].soc < soc0
    assert sim.dev["MG3.FuelCell"].h2_kg > 0


def test_initialize_removes_startup_transient():
    sim = build_system(dt=DT)
    sim.initialize()
    res = sim.run(0.5)
    v = res["v.MG2"]
    assert abs(v.max() - v.min()) < 2.0          # no start-up swing after settling


def test_snapshot_branches_are_independent():
    base = build_system(dt=DT)
    base.initialize()
    case_a = base.snapshot()
    case_b = base.snapshot()
    case_a.at(0.1, lambda s: s.breaker("grid", False))
    case_a.run(1.0)
    case_b.run(1.0)
    assert case_a.ac.p_grid == 0.0
    assert case_b.ac.p_grid > 1e3                # untouched branch stays grid-tied
    assert base.t == 0.0


def test_trapezoidal_line_current_matches_ohms_law():
    sim = build_system(dt=DT)
    sim.initialize()
    dc = sim.dc
    for name, ln in dc.lines.items():
        dv = dc.v[ln["a"]] - dc.v[ln["b"]]
        assert dc.line_current(name) == pytest.approx(dv / ln["R"], abs=0.5)
