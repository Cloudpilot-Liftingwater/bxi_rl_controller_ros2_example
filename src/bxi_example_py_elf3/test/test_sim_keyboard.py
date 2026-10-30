import numpy as np
from bxi_example_py_elf3.sim_keyboard import StartupGate,PD,NORMAL
from elf3_eval.contract import BODY


def state(name,transition=False):
    return dict(current=dict(name=name,elapsed=4),in_transition=transition)


def test_suspension_never_releases_from_pd_or_state_heartbeat_alone():
    gate=StartupGate();gate.state_update(state(PD),0)
    for t in np.arange(0,2,.02):gate.command_update(BODY,[0]*29,[100]*29,t)
    assert not gate.ready_to_release(2)
    gate.state_update(state(NORMAL),2)
    gate.state_update(state(NORMAL),3)
    assert not gate.ready_to_release(3)


def test_only_fresh_complete_normal_output_opens_release_gate():
    gate=StartupGate();gate.state_update(state(NORMAL),0)
    for t in np.arange(0,.82,.02):
        gate.state_update(state(NORMAL),t)
        gate.command_update(tuple(reversed(BODY)),[0]*29,[100]*29,t)
    assert gate.ready_to_release(.81)
    assert not gate.ready_to_release(1.2) # stale output
    gate.state_update(state(NORMAL,transition=True),1.21)
    assert not gate.ready_to_release(1.21)


def test_missing_joints_nonfinite_targets_and_zero_gains_do_not_count():
    gate=StartupGate();gate.state_update(state(NORMAL),0)
    for t in np.arange(0,1,.02):
        gate.state_update(state(NORMAL),t)
        gate.command_update(BODY[:-1],[0]*28,[100]*28,t)
        gate.command_update(BODY,[float('nan')]+[0]*28,[100]*29,t)
        gate.command_update(BODY,[0]*29,[0]*29,t)
    assert gate.normal_commands==0
    assert not gate.ready_to_release(.99)
