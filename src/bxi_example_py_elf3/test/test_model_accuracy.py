"""Causal pairing, short/stale/duplicate recordings and exact known errors."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from bxi_example_py_elf3.control.elf3 import JOINT_NAMES
from bxi_example_py_elf3.framework.joints import JointLayout, JointStateBuffer
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.model_test_accuracy import TrackingRun
from test_dance_execution import Logger


def publish(run, t, q=0., *, blend=False):
    names=tuple(reversed(JOINT_NAMES))
    frame=MotorFrame.create(JointLayout(names), np.full(29, q), np.ones(29), np.ones(29))
    run.stage(frame)
    ns=int(round(t*1e9))
    msg=NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9)))
    run.command_published(msg, run.state, blend)


def feedback(t, q):
    names=('head_y_joint', *reversed(JOINT_NAMES), 'head_z_joint')
    state=JointStateBuffer(JointLayout(names))
    state.position[:]=q
    state.view.timestamp_ns=int(round(t*1e9))
    return state.view


def summarize(run, completed=True):
    run.finish(completed)
    run.worker.join(5)
    assert not run.worker.is_alive()
    return run.report


@pytest.mark.parametrize('degrees,status', [(2.,'PASS'),(4.,'FAIL')])
def test_known_error_and_all_29_named_joints(degrees,status):
    run=TrackingRun('test/dance',.02,Logger())
    for i in range(160):
        t=100+i*.02
        publish(run,t, q=t)
        run.sample(feedback(t+.01,t+np.radians(degrees)))
    report=summarize(run)
    assert report['status']==status and report['coverage']>=.95
    assert report['independent_targets']==160
    for row in report['joints'].values():
        for metric in ('mae','rmse','p95','maximum','bias'):
            assert row[metric]==pytest.approx(degrees, abs=3e-4)


def test_feedback_never_uses_future_or_unpublished_model():
    run=TrackingRun('test/dance',.02,Logger())
    publish(run,1.,q=.1)
    publish(run,1.02,q=.9)
    run.sample(feedback(1.01,.1))
    assert run.samples==1 and np.max(np.abs(run.errors[0]))<1e-5
    # Merely staging a target must not count as successful publication.
    frame=MotorFrame.create(JointLayout(JOINT_NAMES),np.ones(29),np.ones(29),np.ones(29))
    run.stage(frame)
    run.sample(feedback(1.025,.9))
    assert run.samples==2 and np.max(np.abs(run.errors[1]))<1e-5


def test_duplicates_do_not_become_independent_samples_and_abort_cannot_pass():
    run=TrackingRun('test/dance',.02,Logger())
    publish(run,1.)
    for _ in range(160):
        run.sample(feedback(1.01,0.))
    assert run.samples==1
    assert summarize(run)['status']=='INSUFFICIENT_DATA'
    other=TrackingRun('test/dance',.02,Logger())
    assert summarize(other,False)['status']=='ABORTED'


def test_gap_blend_clock_jump_and_capacity_cannot_pass():
    run=TrackingRun('test/dance',.02,Logger())
    publish(run,1.,blend=True)
    run.sample(feedback(1.01,0.))
    assert run.samples==0
    publish(run,2.)
    run.sample(feedback(2.01,0.))
    run.sample(feedback(3.,0.))  # Target is stale.
    run.sample(feedback(2.5,0.))  # Backwards receipt clock.
    report=summarize(run)
    assert report['status']=='INSUFFICIENT_DATA'
    assert report['coverage']<.95
    assert 'feedback_clock_discontinuity' in report['reasons']
    other=TrackingRun('test/dance',.02,Logger())
    other.max_samples=1
    publish(other,1.)
    other.sample(feedback(1.01,0.))
    other.sample(feedback(1.02,0.))
    assert 'sample_capacity_exceeded' in summarize(other)['reasons']


def test_large_error_identifies_joint_and_listener_works_without_offline_eval(monkeypatch):
    from bxi_example_py_elf3.framework import execution_observer as bridge
    monkeypatch.setattr(bridge,'_host',None)
    run=TrackingRun('test/dance',.02,Logger())
    bridge.add_command_listener(run)
    try:
        frame=MotorFrame.create(JointLayout(JOINT_NAMES),np.zeros(29),np.ones(29),np.ones(29))
        run.stage(frame)
        bridge.observe('command_published',NS(header=NS(stamp=NS(sec=1,nanosec=0))),run.state,False)
        joints=feedback(1.01,0.)
        joints.position[joints.layout.index('l_knee_y_joint')]=np.radians(25.)
        run.sample(joints)
        assert run.fault['joint']=='l_knee_y_joint'
        assert run.fault['observed_deg']==pytest.approx(25.)
    finally:
        bridge.remove_command_listener(run)
