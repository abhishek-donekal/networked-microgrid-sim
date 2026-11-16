# mgsim: hybrid AC/DC networked microgrid simulator

Python time-domain simulator of a three-microgrid hybrid AC/DC system
(two interlinking converters, batteries, supercapacitor, fuel cells, EVs,
solar and wind), for control and energy-management experiments.

**Live demo:** <https://abhishek-donekal.github.io/networked-microgrid-sim/> —
the same Python model running in your browser via Pyodide: trip breakers and
move load/renewable sliders while it runs in real time.
There is also a [recorded baseline scenario](https://abhishek-donekal.github.io/networked-microgrid-sim/playback.html)
and the [system architecture](https://abhishek-donekal.github.io/networked-microgrid-sim/architecture.svg).

## Run

From this folder (Windows):

    .venv\Scripts\python examples\run_baseline.py     # writes results\baseline.csv and .png
    .venv\Scripts\python -m pytest -q                 # sanity tests

To recreate the environment: `python -m venv .venv` then
`.venv\Scripts\python -m pip install numpy matplotlib pytest`.

## Layout

| File | Contents |
|---|---|
| `mgsim/system.py` | The architecture and every parameter (`DEFAULTS`). Start here. |
| `mgsim/devices.py` | Battery, SuperCap, FuelCell, EV, Renewable (PV / wind), Load |
| `mgsim/ac.py` | Grid, AC buses, AC loads, interlinking converters (IBR 01 / 02) |
| `mgsim/control.py` | Supervisory controllers (secondary voltage restoration) |
| `mgsim/core.py` | DC network solver, simulator loop, results, plotting |
| `examples/run_baseline.py` | Example scenario to copy for new experiments |

## Writing an experiment

    from mgsim import build_system, default_params, kW

    p = default_params()
    p["MG3"]["load"] = [(0, 30*kW), (2, 50*kW)]        # load step at t = 2 s
    sim = build_system(p)
    sim.initialize()                                   # settle to steady state first

    sim.at(3.0, lambda s: s.breaker("grid", False))    # island at t = 3 s
    sim.every(0.1, my_ems)                             # your controller, every 100 ms

    res = sim.run(6.0)
    res["v.MG3"]; res.to_csv("out.csv")

To branch several contingencies from one settled state (ParaEMT-style snapshots):

    base = build_system(); base.initialize()
    base.save("settled.pkl")                 # lambdas aren't picklable; use
    case = base.snapshot()                   # named functions with save()
    case.at(1.0, lambda s: s.breaker("tie", False))
    res = case.run(5.0)

Breakers: `grid`, `feeder_AC1`, `feeder_AC2`, `tie`, `feeder_MG1/2/3`, any
device name (`MG1.SolarPV`), `IBR01`, `IBR02`, any load name.

A controller is a function `f(sim)`. It can read `sim.v("MG1")`,
`sim.dev["MG1.Battery"].soc` / `.p`, `sim.ac.ibr["IBR01"].p_dc`, and write
`.dv_sec`, `.p_bias`, `.mode` (EV), `.curtail` (PV / wind), and for an IBR
`.p_set` and `.gfl_mode` (`"vdc_droop"` or `"pq"`).

Recorded signals: `v.<bus>`, `i.<line>`, `p.<device>`, `soc.<storage>`,
`avail.<renewable>`, `h2_kg.<fuelcell>`, `f.<ac bus>`, `p.grid`,
`gfm.<ibr>`, `unserved.<ac load>`. Power is positive into a DC bus; IBR power
is positive AC to DC; `p.grid` is positive when importing.

## What the model is, and is not

- Averaged models: each converter is a controlled current source with a
  first-order response. No switching, harmonics or inner-loop detail.
- DC side: bus capacitances and R-L lines solved by the trapezoidal-rule
  companion-model (nodal) method, the standard EMT formulation; set
  `p["solver"] = "euler"` for heavily damped long runs with large steps.
- AC side: active power and frequency only. No AC voltage magnitude, reactive
  power, line impedance or synchronisation transients. Reconnection to the
  grid is instantaneous.
- Grid-forming IBRs sharing an island split load by their droop gains
  instantly; only the frequency is filtered.
- Each AC feeder and each converter is one breaker (the drawing shows two in
  series).
- All ratings, impedances and control gains in `DEFAULTS` are placeholders.
