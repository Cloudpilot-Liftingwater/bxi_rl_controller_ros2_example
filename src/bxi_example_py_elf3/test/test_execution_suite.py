"""Full-suite handoff, release and actual SONIC file adapter; no ROS sockets."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import time
import sys
from types import ModuleType
from threading import RLock
from functools import partial

import numpy as np
import pytest

from bxi_example_py_elf3.simulation_release import SimulationRelease,NORMAL
from bxi_example_py_elf3.framework.inference import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointStateBuffer
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS

ROOT=Path(__file__).resolve().parents[1]


def module(relative,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/relative)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


class Logger:
    def info(self,message):pass
    def warning(self,message):pass
    def error(self,message):pass


class NativeResetService:
    """BXI simulator contract: 0 + step1 -> 1, 1 + step2 -> 2.

    A repeated/out-of-order step returns false and resets the protocol to 0,
    as confirmed by the installed simulator callback and the failed run log.
    """
    def __init__(self):self.stage=0;self.requests=[];self.released=False
    def wait_for_service(self,timeout_sec):return True
    def service_is_ready(self):return True
    def call_async(self,request):
        self.requests.append((request.reset_step,request.release))
        success=request.reset_step==self.stage+1 and self.stage in (0,1)
        if success:
            self.stage+=1
            self.released=self.released or request.release
        else:self.stage=0
        return NS(done=lambda:True,result=lambda:NS(is_success=success))


@pytest.mark.parametrize('deferred',[True,False])
def test_real_startup_and_release_never_repeat_native_step2(deferred):
    from bxi_example_py_elf3.bxi_example_demo import BxiExample
    service=NativeResetService()
    fw=NS(current_state_name='com.bxi.basic_actions/pd_brake',state_machine=NS(in_transition=False),loop_count=1)
    node=NS(imu_required=False,_imu_received=True,simulation_release_on_normal=deferred,
        release_suspension=False,topic_prefix='simulation/',step=0,_second_reset_at=0.,
        _reset_future=None,_reset_pending_step=0,_reset_sent_at=0.,_reset_retry_after_at=0.,
        rest_srv=service,_joint_received=True,lock_in=RLock(),get_logger=lambda:Logger(),
        runtime=NS(framework=fw,request_state=lambda *a,**kw:True),_simulation_release=SimulationRelease())
    node.robot_reset=partial(BxiExample.robot_reset,node)
    node._reset_request_completed=partial(BxiExample._reset_request_completed,node)
    def tick(t):return BxiExample.startup_step(node,t)
    tick(100.);tick(100.01);tick(101.1);tick(101.2)
    assert node.step==2 and tick(102.)
    if deferred:
        assert service.requests==[(1,False)]
        assert service.stage==1 and not service.released
        for t in (103.,200.,500.):assert tick(t)
        fw.current_state_name=NORMAL
        tick(501.);tick(501.6);tick(501.7)
        assert node._simulation_release.confirmed
        assert service.requests==[(1,False),(2,True)] and service.released
        for t in (502.,503.,504.):assert tick(t)
        assert len(service.requests)==2
    else:
        assert service.requests==[(1,False),(2,False)]
        assert service.stage==2 and not service.released


def test_controller_releases_only_after_normal_and_service_ack():
    calls=[];logger=Logger();framework=NS(current_state_name='com.bxi.basic_actions/pd_brake',
        state_machine=NS(in_transition=False),loop_count=20)
    future=NS(done=lambda:False,result=lambda:NS(is_success=True))
    node=NS(topic_prefix='simulation/',runtime=NS(framework=framework),_imu_received=True,
        get_logger=lambda:logger,rest_srv=NS(service_is_ready=lambda:True,call_async=lambda request:calls.append(request) or future))
    release=SimulationRelease();request=lambda:NS(header=NS())
    release.tick(node,0,request);release.tick(node,10,request);assert not calls
    framework.current_state_name=NORMAL
    release.tick(node,10,request);release.tick(node,10.6,request)
    assert len(calls)==1 and calls[0].release and calls[0].reset_step==2
    assert not release.confirmed
    future.done=lambda:True;release.tick(node,10.7,request);assert release.confirmed


@pytest.mark.parametrize('clip',['06','11'])
def test_real_sonic_adapter_keeps_exact_named_decoder_output_and_timeline(clip):
    from bxi_example_py_elf3.sonic_file import SonicFilePolicy
    from elf3_eval import host
    events=[]
    class Capture:
        errors=0
        def capture(self,factory):events.append(factory())
    original=host._enabled,host._tap;host._enabled=True;host._tap=Capture()
    policy=SonicFilePolicy(ROOT/'mods/com.bxi.execution_suite/assets'/('sonic'+clip+'.npz'),clip_id=clip,logger=Logger())
    joints=JointStateBuffer(ELF3_POLICY_JOINTS);joints.position[:]=policy.inner.default_dof_pos;joints.velocity.fill(0)
    frame=InferenceFrame(joints=joints.view,quat_wxyz=np.array([1.,0,0,0]),angular_velocity=np.zeros(3))
    try:
        policy.reset(frame);policy.inner.source_blend_duration_s=0
        policy.step(frame,.02,advance=False);assert policy.cursor==0 and not events
        for i in range(5):
            host.begin_cycle();output=policy.step(frame,.02)
            expected=policy.inner.default_dof_pos+policy.inner.last_action*policy.inner.action_scale
            np.testing.assert_allclose(output.joints.position,expected,atol=1e-6)
            np.testing.assert_array_equal(events[-1]['q'],output.joints.position)
            assert events[-1]['reference_frame']==i
            assert events[-1]['names']==list(ELF3_POLICY_JOINTS.names)
        assert policy.cursor==5 and host._tap.errors==0
        policy.cursor=policy.frames;last=policy.output.joints.position.copy()
        policy.step(frame,.02);np.testing.assert_array_equal(policy.output.joints.position,last)
    finally:
        policy.close();host._enabled,host._tap=original
