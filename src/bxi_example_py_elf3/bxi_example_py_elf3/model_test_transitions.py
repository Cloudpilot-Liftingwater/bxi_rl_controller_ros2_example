"""Reuse BXI's running blend with faster model entry in simulation only."""
import math

from bxi_example_py_elf3.framework.transitions.running_blend import RunningBlendTransition


class ModelEntryTransition(RunningBlendTransition):
    type_name = "execution_suite_model_entry"

    @classmethod
    def from_config(cls, name, raw):
        options = dict(raw)
        duration = options.pop("simulation_duration", 0.3)
        if (isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(duration) or duration <= 0):
            raise ValueError("model simulation_duration must be a finite positive number")
        plan = super().from_config(name, options)
        plan._simulation_duration = float(duration)
        return plan

    def on_start(self, ctx, from_state, to_state):
        # on_start runs on the session copy, preserving the hardware plan.
        if getattr(getattr(ctx, "ros_node", None), "topic_prefix", "") == "simulation/":
            self._duration = self._simulation_duration
        super().on_start(ctx, from_state, to_state)

    def config_snapshot(self):
        return {**super().config_snapshot(), "simulation_duration": self._simulation_duration}


class CxkEntryTransition(ModelEntryTransition):
    """Keep the already-used CXK transition type compatible."""
    type_name = "execution_suite_cxk_entry"
