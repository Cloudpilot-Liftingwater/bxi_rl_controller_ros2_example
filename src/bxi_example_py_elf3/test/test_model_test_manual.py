"""Exercise manual input through the real state machine, plus original thermal rules."""
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework.joints import JointTargetBuffer
from bxi_example_py_elf3.framework.mod_api import MotorFrame, PoseState
from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine
from bxi_example_py_elf3.framework.runtime.transition import register_transition_plugin
from bxi_example_py_elf3.model_test_safety import ModelTemperatureGuard
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS
from test_dance_execution import Logger, context
from test_execution_suite import module

ROOT=Path(__file__).resolve().parents[1]
NORMAL='com.bxi.basic_actions/normal'
PREFIX='com.bxi.execution_suite/'


class Policy:
    def __init__(self):
        self.buffer=JointTargetBuffer(ELF3_POLICY_JOINTS)
        self.buffer.update(np.zeros(29),np.full(29,10.),np.ones(29))
        self.output=NS(joints=self.buffer.view)
        self.asset_info={'motion_names_observed':True}
        self.inner=NS(bind_logger=lambda logger:None)
        self.cursor=0;self.steps=0;self.frames=50
    def reset(self,frame):self.cursor=0
    def step(self,frame,dt,*,advance=True):
        if advance:self.cursor+=1;self.steps+=1
        return self.output
    def finished(self,trim=0):return self.cursor>=self.frames-trim


def setup_suite(monkeypatch,topic='simulation/'):
    monkeypatch.setenv('BXI_EXECUTION_SUITE','1')
    monkeypatch.setenv('BXI_EXECUTION_SUITE_MODE','manual')
    plugin=module('mods/com.bxi.execution_suite/plugin.py','manual_suite')
    handles={}
    class Resources:
        mod_root=ROOT/'mods/com.bxi.execution_suite'
        def register_resource(self,*args,**kwargs):pass
        def resource(self,key):
            if key.id not in handles:
                policy=Policy();handles[key.id]=NS(get=lambda:policy,status='ready',key=key,error=None)
            return handles[key.id]
    definition=plugin.create_mod(Resources())
    states={key:factory(NS(name=PREFIX+key,state_id=i+10))
            for i,(key,factory) in enumerate(definition.state_factories.items())}
    ctx,frames,requests=context()
    ctx.ros_node=NS(topic_prefix=topic)
    names=list(ELF3_POLICY_JOINTS.names)+['head_y_joint','head_z_joint']
    ctx.actuator_temperatures=NS(names=names,motor_c=[20.]*31,driver_c=[22.]*31,
                                received_at=__import__('time').monotonic())
    for state in states.values():state._bind_logger(Logger())
    return plugin,definition,states,ctx,frames,requests


def machine(monkeypatch, *, include_zero=False, initial_state=NORMAL):
    plugin,definition,states,ctx,frames,requests=setup_suite(monkeypatch)
    for transition in definition.transition_plugins.values():
        register_transition_plugin(transition)
    class Normal(PoseState):
        def target_position(self,ctx):return np.zeros(29,np.float32)
        def gains(self,ctx):return np.full(29,10,np.float32),np.ones(29,np.float32)
    normal=Normal(NORMAL,1);normal._bind_logger(Logger())
    forward=Normal('com.bxi.basic_actions/forward_back',3);forward._bind_logger(Logger())
    factories={NORMAL:normal,forward.name:forward,**{s.name:s for s in states.values()}}
    if include_zero:
        zero_module=module('mods/com.bxi.basic_actions/zero_torque_state.py','manual_real_zero')
        zero=zero_module.ZeroTorqueState('com.bxi.basic_actions/zero_torque',2)
        zero._bind_logger(Logger());factories[zero.name]=zero
    config=dict(initial_state=initial_state,graph={'validate':False},default_transition='instant',
                transition_profiles={'instant':{'type':'instant'},
                                     'soft_switch':{'type':'hold','duration':.02}},
                states={n:{} for n in factories})
    for mod in ('basic_actions','execution_suite'):
        prefix='com.bxi.'+mod+'/'
        manifest=yaml.safe_load((ROOT/f'mods/com.bxi.{mod}/mod.yaml').read_text())
        for route in manifest['routes']:
            src=route['from'] if '/' in route['from'] else prefix+route['from']
            dst=route['to'] if '/' in route['to'] else prefix+route['to']
            if src not in factories or dst not in factories:continue
            event=route['event'] if '/' in route['event'] else prefix+route['event']
            config['states'][src].setdefault('transitions',{}).setdefault('on_event',{})[event]=dict(to=dst,transition=route.get('transition','instant'))
        for action in manifest.get('actions',[]):
            src=action['from'] if '/' in action['from'] else prefix+action['from']
            if src not in factories:continue
            event=action['event'] if '/' in action['event'] else prefix+action['event']
            config['states'][src].setdefault('transitions',{}).setdefault('on_event',{})[event]={'action':action['action']}
    ctx.last_motor_frame=MotorFrame.create(ctx.robot_layout,np.zeros(29),np.full(29,10),np.ones(29))
    def resolve(frame,out):return out.update(frame.qpos,frame.kp,frame.kd,vel=frame.vel,torque=frame.torque)
    ctx.resolve_motor_frame=resolve
    def publish(frame):
        # ROS ActuatorCmds serializes values at publication. Retaining policy
        # views here would retroactively mutate historical "sent" commands.
        frames.append(MotorFrame.create(frame.layout,frame.qpos.copy(),frame.kp.copy(),
            frame.kd.copy(),vel=frame.vel.copy(),torque=frame.torque.copy()))
        resolve(frame,ctx.last_motor_frame)
    ctx.set_motor_target=publish
    ctx.preheat_model=lambda policy:(policy.reset(ctx.inference_frame),policy.step(ctx.inference_frame,.02,advance=False))
    sm=RobotStateMachine(ctx,config,factories,logger=Logger())
    ctx.state_machine=sm;ctx.request_state=lambda target,**kwargs:sm.request_transition(target,**kwargs)
    def tick(event=None):
        ctx.robot_joints.timestamp_ns+=1
        active=sm.update(.02,[] if event is None else [event])
        if not active:sm.update_current_state(.02)
        if definition.state_observer is not None:
            definition.state_observer(sm.current_state_name)
    return sm,states,ctx,frames,tick


def reach_motion_phase(sm, states, tick, stage, phase):
    state = states[stage]
    if phase == "entry":
        assert sm.in_transition and state.result == "preparing"
        return
    for _ in range(400):
        if phase == "active" and (sm.current_state_name == PREFIX + stage
                and not sm.in_transition and state.result == "running" and state.policy.steps > 0):
            return
        if phase == "exit" and state.result == "playback_complete" and sm.in_transition:
            return
        tick()
    raise AssertionError("did not reach " + stage + " " + phase)


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
def test_normal_preempts_entry_without_advancing_model(monkeypatch,stage):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    tick(PREFIX+stage)
    for _ in range(5):tick()
    assert sm.in_transition and sm.current_state_name==NORMAL
    tick(NORMAL)
    assert sm.current_state_name==NORMAL and not sm.in_transition
    assert states[stage].result=='cancelled'
    assert states[stage].policy.steps==0
    assert all(np.all(f.kp>0) for f in frames)


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
def test_normal_cancels_resource_preparation_before_policy_entry(monkeypatch,stage):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    resource=states[stage].required_resources[0]
    resource.status='loading'
    tick(PREFIX+stage)
    assert sm.snapshot()['preparing'] is not None
    tick(NORMAL)
    assert sm.snapshot()['preparing'] is None
    resource.status='ready'
    for _ in range(20):tick()
    assert sm.current_state_name==NORMAL and not sm.in_transition
    assert states[stage].policy.steps==0 and states[stage].result=='not_started'


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
def test_normal_preempts_active_motion_and_does_not_restart(monkeypatch,stage):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    tick(PREFIX+stage)
    reach_motion_phase(sm, states, tick, stage, "active")
    assert sm.current_state_name==PREFIX+stage and not sm.in_transition
    steps=states[stage].policy.steps
    tick(NORMAL)
    for _ in range(7):tick()
    assert sm.current_state_name==NORMAL and not sm.in_transition
    assert states[stage].result=='cancelled' and states[stage].policy.steps==steps
    for _ in range(200):tick()
    assert sm.current_state_name==NORMAL and all(np.all(f.kp>0) for f in frames)


def test_each_completed_clip_waits_for_operator_and_can_be_repeated(monkeypatch):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    for stage in ['sonic06','dance','sonic11','sonic06']:
        tick(PREFIX+stage)
        for _ in range(240):tick()
        assert states[stage].result=='playback_complete'
        assert sm.current_state_name==NORMAL and not sm.in_transition
        for _ in range(50):tick()
        assert sm.current_state_name==NORMAL


@pytest.mark.parametrize('topic', ['simulation/', 'hardware/'])
@pytest.mark.parametrize('stage', ['dance', 'sonic06', 'sonic11'])
def test_overlimit_error_is_scored_after_full_playback(monkeypatch, topic, stage):
    _, _, states, ctx, frames, requests = setup_suite(monkeypatch, topic)
    state = states[stage]
    state.on_enter(ctx)
    try:
        state.policy.frames = 121
        joint = 'r_ankle_y_joint'
        ctx.robot_joints.position[ctx.robot_layout.index(joint)] = np.radians(25.)
        run = state._accuracy
        for i in range(state.policy.frames):
            stamp = 1_000_000_000 + i * 20_000_000
            ctx.robot_joints.timestamp_ns = stamp
            state.on_update(ctx, .02)
            if i < state.policy.frames - 1:
                assert state.result == 'running' and not requests
                sec, nanosec = divmod(stamp, 1_000_000_000)
                run.command_published(NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec))), state.name)
        assert len(frames) == state.policy.frames
        assert state.result == 'playback_complete'
        assert [(target, kw['trigger']) for target, kw in requests] == [(NORMAL, 'dance_playback_complete')]
        run.worker.join(5)
        assert run.report['paired'] == 120
        assert run.report['status'] == 'FAIL'
        assert run.report['joints'][joint]['maximum'] == pytest.approx(25.)
    finally:
        state.on_exit(ctx)


def test_normal_shortens_exit_blend_without_reclassifying_completion(monkeypatch):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    tick(PREFIX+'dance')
    reach_motion_phase(sm, states, tick, "dance", "exit")
    assert states['dance'].result=='playback_complete' and sm.in_transition
    tick(NORMAL)
    for _ in range(7):tick()
    assert sm.current_state_name==NORMAL and not sm.in_transition
    assert states['dance'].result=='playback_complete'


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
@pytest.mark.parametrize('fault',['missing','stale','nan','overheat'])
def test_all_models_use_hardware_thermal_protection(monkeypatch,stage,fault):
    plugin,definition,states,ctx,frames,requests=setup_suite(monkeypatch,'hardware/')
    state=states[stage];state.on_enter(ctx)
    sample=ctx.actuator_temperatures
    if fault=='missing':ctx.actuator_temperatures=None
    elif fault=='stale':sample.received_at-=1
    elif fault=='nan':sample.motor_c[0]=np.nan
    else:
        _,limit,_,_=state.temperature_guard.policy.limits_for(sample.names[0])
        sample.motor_c[0]=limit-15
    state.on_update(ctx,.02)
    assert state.result=='failed' and not frames
    assert requests[-1][0]==NORMAL
    assert any('failed' in s for s in state.logger.messages)


def test_named31_temperatures_preserve_exact_original_thresholds():
    guard=ModelTemperatureGuard(ROOT/'mods/com.bxi.suspended_tests/config')
    names=['head_z_joint',*reversed(ELF3_POLICY_JOINTS.names),'head_y_joint']
    sample=NS(names=names,motor_c=[20.]*31,driver_c=[30.]*31,received_at=__import__('time').monotonic())
    ctx=NS(ros_node=NS(topic_prefix='hardware/'),actuator_temperatures=sample)
    assert guard.check(ctx) is None
    index=names.index('l_knee_y_joint');_,limit,_,_=guard.policy.limits_for(names[index])
    sample.motor_c[index]=limit-15
    fault=guard.check(ctx)
    assert fault['code']=='MOTOR_OVER_TEMPERATURE'
    assert fault['details']['joint']=='l_knee_y_joint' and fault['details']['threshold_c']==limit-15


def test_hardware_rejects_automatic_sequence(monkeypatch):
    plugin,definition,states,ctx,*_=setup_suite(monkeypatch,'hardware/')
    assert plugin.enabled(ctx)
    monkeypatch.setenv('BXI_EXECUTION_SUITE_MODE','auto')
    assert not plugin.enabled(ctx)
    assert not states['dance'].is_available(ctx)


def test_temperature_fault_uses_error_and_emits_exact_threshold(monkeypatch):
    from bxi_example_py_elf3.framework import execution_observer
    events=[];levels=[]
    monkeypatch.setattr(execution_observer,'_host',NS(test_event=lambda *a,**kw:events.append((a,kw))))
    _,_,states,ctx,*_=setup_suite(monkeypatch,'hardware/')
    state=states['sonic06']
    state._bind_logger(NS(info=lambda s:levels.append('info'),warning=lambda s:levels.append('warning'),error=lambda s:levels.append('error')))
    state.on_enter(ctx)
    _,limit,_,_=state.temperature_guard.policy.limits_for(ctx.actuator_temperatures.names[0])
    ctx.actuator_temperatures.motor_c[0]=limit-15
    state.on_update(ctx,.02)
    assert 'error' in levels
    args,details=next((a,k) for a,k in events if k.get('code')=='MOTOR_OVER_TEMPERATURE')
    assert args[1]=='failed' and details['severity']=='error'
    assert details['details']['observed_c']==limit-15
    assert details['details']['threshold_c']==limit-15


def test_imu_timeout_keeps_original_lock_and_logs_once(monkeypatch):
    from threading import RLock
    import time
    from bxi_example_py_elf3.bxi_example_demo import BxiExample
    from bxi_example_py_elf3.framework import execution_observer
    events=[];requests=[]
    monkeypatch.setattr(execution_observer,'_host',NS(test_event=lambda *a,**kw:events.append((a,kw))))
    node=NS(imu_required=True,_imu_last_received_at=time.monotonic()-1,_imu_received=True,
        _imu_protection_latched=False,imu_runtime_timeout_sec=.1,lock_in=RLock(),get_logger=lambda:Logger(),
        runtime=NS(current_state_name=NORMAL,request_state=lambda *a,**kw:requests.append((a,kw)) or True))
    BxiExample._check_imu_runtime_safety(node);BxiExample._check_imu_runtime_safety(node)
    assert node._imu_protection_latched and len(events)==1
    assert events[0][1]['code']=='IMU_FEEDBACK_STALE'
    assert requests[-1][0]==('com.bxi.basic_actions/imu_protection',)


def test_timing_event_preserves_warning_throttle_and_observer_isolation(monkeypatch):
    from queue import SimpleQueue
    from bxi_example_py_elf3.framework.platform import runtime
    from bxi_example_py_elf3.framework import execution_observer
    events=[]
    monkeypatch.setattr(execution_observer,'_host',NS(test_event=lambda *a,**kw:events.append((a,kw))))
    summary=dict(state=PREFIX+'dance',events=3,wake_events=3,finish_events=0,
        max_finish_late_ms=0.,max_wake_late_ms=2.,max_cycle_ms=1.,period_ms=20.,latest_count=3)
    owner=NS(config=NS(deadline_tolerance_sec=.001,deadline_warning_interval_sec=1.,compute_budget_sec=.005),
        _deadline_miss_queue=SimpleQueue(),_pending_deadline_summary=summary,_last_deadline_warning_at=0.,_log=lambda *a:True)
    method=next(getattr(cls,'_drain_deadline_misses') for cls in vars(runtime).values()
                if isinstance(cls,type) and '_drain_deadline_misses' in vars(cls))
    method(owner)
    assert owner._pending_deadline_summary is None and events[0][1]['severity']=='warning'
    assert events[0][1]['details']['wake_tolerance_ms']==1.
    owner._pending_deadline_summary=dict(summary);owner._last_deadline_warning_at=0.
    def broken(*a,**kw):raise RuntimeError('observer unavailable')
    monkeypatch.setattr(execution_observer,'_host',NS(test_event=broken))
    method(owner)
    assert owner._pending_deadline_summary is None
