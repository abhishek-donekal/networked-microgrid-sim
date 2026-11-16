"""Write a self-contained HTML playback dashboard for a simulation run.

    from mgsim.dashboard import write_dashboard
    write_dashboard(res, "results/dashboard.html",
                    events=[(6.0, "Grid lost")], title="My experiment")

Open the file in any browser. It needs no server and no internet connection.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

_TEMPLATE = Path(__file__).with_name("dashboard_template.html")
HEAD = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n')
TAIL = "\n</html>\n"


def _convert(key, arr):
    if key.startswith(("p.", "avail.", "unserved.")):
        return np.round(arr / 1e3, 2)                 # W -> kW
    if key.startswith("soc."):
        return np.round(arr, 4)
    if key.startswith("f."):
        return np.round(arr, 3)
    if key.startswith("h2_kg."):
        return np.round(arr, 5)
    return np.round(arr, 1)


def write_dashboard(res, path, events=(), title="Microgrid Simulation Playback",
                    subtitle="", note="", v_nom=750.0, f0=60.0, max_frames=800,
                    autoplay=True, standalone=True):
    """Export `res` (a Results object) as an interactive playback page.

    events     : list of (time_s, label) shown as jump buttons and chart markers
    max_frames : the run is thinned to at most this many animation frames
    standalone : True writes a complete HTML file; False writes only the body
                 fragment (for embedding or publishing).
    """
    step = max(1, math.ceil(len(res.t) / max_frames))
    data = {
        "t": np.round(res.t[::step], 4).tolist(),
        "s": {k: _convert(k, v[::step]).tolist() for k, v in res.signals.items()},
        "events": [{"t": float(t), "label": label} for t, label in sorted(events)],
        "meta": {"title": title, "subtitle": subtitle, "note": note,
                 "v_nom": v_nom, "f0": f0, "autoplay": autoplay},
    }
    page = _TEMPLATE.read_text(encoding="utf-8").replace(
        "/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    if standalone:
        page = HEAD + page + TAIL
    Path(path).write_text(page, encoding="utf-8")
    return Path(path)
