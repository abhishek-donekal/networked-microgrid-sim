"""Baseline scenario: a 12 s run that exercises the main operating modes.

  t = 2 s   load step in Microgrid 02 (+25 kW)
  t = 4 s   cloud passes over the PV in Microgrid 01 (30 kW -> 5 kW over 1 s)
  t = 6 s   utility grid lost: both IBRs switch from grid-following to grid-forming
  t = 8 s   DC tie-line opens: Microgrid 01 is separated from 02 and 03
  t = 10 s  grid restored and tie-line reclosed

Run from the research folder:
    .venv\\Scripts\\python examples\\run_baseline.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mgsim import Profile, build_system, default_params, kW  # noqa: E402
from mgsim.dashboard import write_dashboard  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results"
OUT.mkdir(exist_ok=True)

# ---- parameters: start from the defaults and override what the scenario needs
p = default_params()
p["MG2"]["load"] = [(0, 35 * kW), (2, 60 * kW)]
p["MG1"]["SolarPV"]["p_avail"] = Profile(
    [(0, 30 * kW), (4, 30 * kW), (5, 5 * kW)], kind="linear")

sim = build_system(p, dt=1e-4, record_every=20)
sim.initialize()        # settle to steady state so t = 0 starts clean

# ---- events
sim.at(6.0, lambda s: s.breaker("grid", False))
sim.at(8.0, lambda s: s.breaker("tie", False))
sim.at(10.0, lambda s: (s.breaker("grid", True), s.breaker("tie", True)))

res = sim.run(12.0)

# ---- interactive playback dashboard (open results\dashboard.html in a browser)
EVENTS = [(2.0, "Load step in Microgrid 02"), (4.0, "Cloud over solar PV"),
          (6.0, "Grid lost"), (8.0, "Tie-line opens"),
          (10.0, "Grid and tie-line restored")]
META = dict(
    events=EVENTS,
    title="Microgrid Simulation Playback",
    subtitle="Baseline scenario: three DC microgrids, two interlinking converters, 12 s run.",
    note="Averaged converter models; AC side is active power and frequency only. "
         "All ratings and control gains are placeholder values, not testbed data.",
)
write_dashboard(res, OUT / "dashboard.html", **META)
if len(sys.argv) > 1:      # optional: also write the body-only fragment to this path
    write_dashboard(res, sys.argv[1], standalone=False, **META)

# ---- outputs
res.to_csv(OUT / "baseline.csv")
res.plot(
    [
        ("DC bus voltage (V)", ["v.*"]),
        ("AC frequency (Hz)", ["f.*"]),
        ("IBR / grid / tie (kW)", ["p.IBR01", "p.IBR02", "p.grid"]),
        ("MG1 sources (kW)", ["p.MG1.Battery", "p.MG1.FuelCell", "p.MG1.SolarPV",
                              "p.MG1.Wind", "p.MG1.EV"]),
        ("MG2 sources (kW)", ["p.MG2.SuperCap", "p.MG2.Battery", "p.MG2.FuelCell",
                              "p.MG2.EV"]),
        ("MG3 sources (kW)", ["p.MG3.Battery", "p.MG3.FuelCell", "p.MG3.EV"]),
    ],
    path=OUT / "baseline.png",
    title="Baseline scenario",
    scale={k: 1e-3 for k in ("IBR / grid / tie (kW)", "MG1 sources (kW)",
                             "MG2 sources (kW)", "MG3 sources (kW)")},
)

print(f"{'t (s)':>6} {'v.MG1':>7} {'v.MG2':>7} {'v.MG3':>7} {'f.AC1':>7} "
      f"{'IBR01 kW':>9} {'IBR02 kW':>9} {'grid kW':>8} {'tie A':>7}")
for t in (1.9, 3.9, 5.9, 7.9, 9.9, 11.9):
    print(f"{t:6.1f} {res.at('v.MG1', t):7.1f} {res.at('v.MG2', t):7.1f} "
          f"{res.at('v.MG3', t):7.1f} {res.at('f.AC1', t):7.3f} "
          f"{res.at('p.IBR01', t) / 1e3:9.1f} {res.at('p.IBR02', t) / 1e3:9.1f} "
          f"{res.at('p.grid', t) / 1e3:8.1f} {res.at('i.tie', t):7.1f}")
print(f"\nSaved {OUT / 'baseline.csv'} and {OUT / 'baseline.png'}")
