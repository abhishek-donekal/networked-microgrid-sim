"""Supervisory controllers. Attach with  sim.every(period, controller).

A controller is any callable taking the simulator. It can read voltages,
powers and states of charge and write setpoints (dv_sec, p_bias, p_set, mode,
curtail) or operate breakers. Write your own experiments in the same shape.
"""
from __future__ import annotations

from .devices import DroopSource


class VoltageRestoration:
    """Secondary control: slow integral action that shifts the droop curves of
    one microgrid's sources so its local bus returns to nominal voltage.
    Power sharing between the sources is unchanged (all get the same shift).
    """

    def __init__(self, node, v_nom=750.0, ki=5.0, period=0.01, limit=0.08,
                 sources=None):
        self.node, self.v_nom = node, v_nom
        self.ki, self.period = ki, period
        self.limit = limit * v_nom
        self.sources = sources
        self.dv = 0.0

    def __call__(self, sim):
        if self.sources is None:
            self.sources = [d for d in sim.dev.values()
                            if d.node == self.node and isinstance(d, DroopSource)
                            and not d.hp_tau]
        self.dv += self.ki * (self.v_nom - sim.v(self.node)) * self.period
        self.dv = min(max(self.dv, -self.limit), self.limit)
        for d in self.sources:
            d.dv_sec = self.dv


def attach_voltage_restoration(sim, nodes=("MG1", "MG2", "MG3"), **kw):
    """Add a VoltageRestoration loop to each microgrid. Returns the controllers."""
    ctrls = []
    for node in nodes:
        c = VoltageRestoration(node, **kw)
        sim.every(c.period, c)
        ctrls.append(c)
    return ctrls
