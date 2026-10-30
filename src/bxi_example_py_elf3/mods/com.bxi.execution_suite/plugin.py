"""Append model tests to the existing B sequence; keep one BXI publisher."""
import importlib
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import ModuleType

import numpy as np

from bxi_example_py_elf3.framework.execution_observer import observe, log_message
from bxi_example_py_elf3.framework.mod_api import ModDefinition,ResourceKey,RobotControlState
from bxi_example_py_elf3.sonic_file import SonicFilePolicy
from bxi_example_py_elf3.model_test_safety import ModelTemperatureGuard
from bxi_example_py_elf3.model_test_transitions import CxkEntryTransition, ModelEntryTransition

PREFIX='com.bxi.execution_suite/'
NORMAL='com.bxi.basic_actions/normal'
ZERO='com.bxi.basic_actions/zero_torque'
ORDER=('dance','sonic06','sonic11')


def enabled(ctx):
    return (os.environ.get('BXI_EXECUTION_SUITE')=='1'
            and getattr(ctx.ros_node,'topic_prefix','') in ('simulation/','hardware/')
            and not (getattr(ctx.ros_node,'topic_prefix','')=='hardware/'
                     and os.environ.get('BXI_EXECUTION_SUITE_MODE','manual')=='auto'))


class Session:
    def __init__(self):
        self.active=False;self.ctx=None;self.completed=[];self.pending=None
        self.requested_at=None;self.normal_since=None;self.logger=None
        self.automatic=False;self.faulted=False;self.base_completed=False

    def mark(self,status,reason='',**details):
        severity='error' if status=='failed' else 'warning' if status in ('cancelled','interrupted') else 'info'
        log_message(self.logger,severity,'EXECUTION_SUITE '+status+': '+reason)
        observe('test_event',PREFIX+'suite',status,reason,severity=severity,**details)

    def begin(self,ctx,logger,*,base_completed=True,automatic=None):
        self.active=True;self.ctx=ctx;self.logger=logger;self.completed=[]
        self.automatic=(os.environ.get('BXI_EXECUTION_SUITE_MODE','manual')=='auto') if automatic is None else automatic
        if self.automatic and getattr(ctx.ros_node,'topic_prefix','')!='simulation/':
            raise ValueError('automatic model sequencing is simulation-only')
        self.base_completed=base_completed
        self.pending='dance' if self.automatic else None
        self.requested_at=None;self.normal_since=None
        self.mark('started',('base stages completed; ' if base_completed else 'individual model test; ')
                  +('automatic CXK -> SONIC06 -> SONIC11' if self.automatic else 'manual model selection; wait in normal'))

    def complete(self,stage):
        if not self.active:return
        if self.automatic:
            expected=ORDER[len(self.completed)]
            if expected!=stage:self.abort('unexpected completion '+stage);return
        self.completed.append(stage)
        self.pending=ORDER[len(self.completed)] if self.automatic and len(self.completed)<3 else None
        self.requested_at=None;self.normal_since=None
        self.mark('stage_complete',stage)

    def abort(self,reason):
        if self.active:self.mark('failed',reason)
        self.active=False;self.pending=None;self.faulted=True

    def cancel(self,reason):
        if self.active:self.mark('cancelled',reason,code='OPERATOR_RETURN_NORMAL',category='operator')
        self.active=False;self.pending=None;self.automatic=False

    def observe(self,state):
        if not self.active:return
        now=time.monotonic();ctx=self.ctx
        if state==NORMAL:
            if ctx.state_machine.in_transition:return
            if not self.automatic:
                if self.base_completed and set(ORDER)<=set(self.completed):
                    self.mark('playback_complete','base stages and all three model stages completed; accuracy assessed separately')
                    self.active=False
                return
            release=ctx.ros_node._simulation_release
            if release.failed:self.abort('simulation suspension release failed');return
            if not release.confirmed:return
            if self.normal_since is None:self.normal_since=now
            if now-self.normal_since<1.:return
            if self.pending is None:
                self.mark('playback_complete','all six stages finished and returned to normal; accuracy assessed by eval')
                self.active=False;return
            if self.requested_at is not None:
                if now-self.requested_at>30:self.abort('model state preparation timeout')
                return
            self.requested_at=now
            accepted=ctx.request_state(PREFIX+self.pending,trigger='execution_suite_next',transition={
                'type':CxkEntryTransition.type_name if self.pending=='dance' else ModelEntryTransition.type_name,
                'duration':2.,'curve':'smootherstep',
                'sample_from':True,'sample_to':True,'advance_from':True,'advance_to':False})
            if accepted is False:self.abort('next state rejected: '+self.pending)
        elif state not in {PREFIX+'ground_prepare',*(PREFIX+s for s in ORDER)}:
            self.cancel('interrupted in '+state)


class GroundPrepare(RobotControlState):
    def __init__(self,name,state_id,normal,all_models,session):
        # Returning to normal must not wait for unrelated dance resources.
        super().__init__(name,state_id,resources=(normal,))
        self.normal=normal;self.session=session

    def is_available(self,ctx):
        return enabled(ctx) and bool(getattr(ctx.ros_node,'execution_suite_handoff',None))

    def on_enter(self,ctx):
        self.session.begin(ctx,self.logger)
        ctx.ros_node.execution_suite_handoff=None
        self.elapsed=0.;self.requested=False
        self.last_stamp=ctx.robot_joints.timestamp_ns;self.feedback_at=time.monotonic()
        names=self.normal.get().default_target.layout.names
        self.start=np.array([ctx.last_motor_frame.qpos[ctx.last_motor_frame.layout.index(n)] for n in names])

    def on_update(self,ctx,dt):
        if not enabled(ctx):raise RuntimeError('execution suite is not enabled for this control source')
        if ctx.robot_joints.timestamp_ns!=self.last_stamp:
            self.last_stamp=ctx.robot_joints.timestamp_ns;self.feedback_at=time.monotonic()
        if time.monotonic()-self.feedback_at>.2:
            self.session.abort('ground preparation feedback stale')
            ctx.request_state(ZERO,trigger='suite_fault',force=True);return
        target=self.normal.get().default_target
        self.elapsed+=dt
        a=np.clip(self.elapsed/3.,0,1);a=a*a*a*(10-15*a+6*a*a)
        frame=self._motor_frame_from_target(ctx,target)
        frame.qpos[:]=self.start+a*(frame.qpos-self.start)
        self._apply_frame(ctx,frame)
        measured=np.array([ctx.robot_joints.position[ctx.robot_layout.index(n)] for n in frame.layout.names])
        if not np.isfinite(measured).all():
            self.session.abort('ground preparation feedback nonfinite')
            ctx.request_state(ZERO,trigger='suite_fault',force=True);return
        if self.elapsed>=3 and np.max(np.abs(measured-frame.qpos))<=np.deg2rad(5) and not self.requested:
            self.requested=True
            if ctx.request_state(NORMAL,trigger='suite_ground_normal',transition='soft_switch') is False:
                self.session.abort('normal rejected')
        if self.elapsed>12 and not self.requested:
            self.session.abort('ground preparation did not settle');ctx.request_state(ZERO,trigger='suite_fault',force=True)


def create_mod(context):
    mode=os.environ.get('BXI_EXECUTION_SUITE_MODE','manual')
    if mode not in ('manual','auto'):raise ValueError('BXI_EXECUTION_SUITE_MODE must be manual or auto')
    # Reuse the existing dance test decoder/state with no source duplication.
    package=ModuleType('_execution_suite_dance');package.__path__=[str(context.mod_root.parent/'com.bxi.dance_test')]
    sys.modules[package.__name__]=package
    dance_policy=importlib.import_module(package.__name__+'.policy')
    dance_state=importlib.import_module(package.__name__+'.state')
    guard_spec=importlib.util.spec_from_file_location(
        '_execution_suite_remote_guard',context.mod_root/'remote_guard.py')
    guard_module=importlib.util.module_from_spec(guard_spec)
    guard_spec.loader.exec_module(guard_module)
    thermal=ModelTemperatureGuard(context.mod_root.parent/'com.bxi.suspended_tests/config')
    session=Session()
    class SuiteMotion(dance_state.DanceTestState):
        def __init__(self,name,state_id,policy,stage):
            super().__init__(name,state_id,policy,temperature_guard=thermal)
            self.stage=stage;self.reported=False
        def is_available(self,ctx):
            return (enabled(ctx) and not session.faulted
                    and (not session.automatic or not session.active or session.pending==self.stage))
        def on_prepare(self,ctx,from_state):
            if not session.active:
                session.begin(ctx,self.logger,base_completed=False,automatic=False)
            if self.stage.startswith('sonic'):self.policy.inner.bind_logger(self.logger)
            super().on_prepare(ctx,from_state)
        def on_prepare_cancel(self,ctx,from_state):
            super().on_prepare_cancel(ctx,from_state)
            if self.result=='cancelled':session.cancel(self.stage+' entry cancelled')
        def on_enter(self,ctx):
            self.reported=False;super().on_enter(ctx)
        def _fail(self,ctx,reason,**details):
            session.abort(self.stage+': '+reason)
            super()._fail(ctx,reason,**details)
        def _validate_frame(self,frame):
            if session.ctx is None or not enabled(session.ctx):return super()._validate_frame(frame)
            # Both original decoders use physical PD targets, which need not
            # lie inside mechanical travel. Preserve them and measure error.
            from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS
            if set(frame.layout.names)!=set(ELF3_POLICY_JOINTS.names):raise ValueError('model body29 missing')
            if not all(np.isfinite(a).all() for a in (frame.qpos,frame.kp,frame.kd)):raise ValueError('invalid model output')
            if np.any(frame.kp < 0) or np.any(frame.kd < 0):raise ValueError('negative model gains')
            return frame
        def on_update(self,ctx,dt):
            super().on_update(ctx,dt)
            if self.result=='playback_complete' and not self.reported:
                self.reported=True;session.complete(self.stage)
            elif self.result=='failed':session.abort(self.stage+' failed')
        def on_exit(self,ctx):
            super().on_exit(ctx)
            if self.result=='failed':session.abort(self.stage+' failed')
            elif self.result!='playback_complete':session.cancel(self.stage+' '+self.result)
        def on_action(self,ctx,action_name):
            if action_name=='cancel':
                session.cancel(self.stage+' operator returned to normal')
            return super().on_action(ctx,action_name)
    handles={}
    def load_cxk(resource):
        root=Path(os.environ.get('BXI_CXK_ROOT','/data/ros2_ws/备份/com.bxi.cxk'))/'assets'
        return dance_policy.load_dance_policy(root/'cxkv5.npz',root/'cxkv5_15k.onnx',
                                             variant='gravity_isaaclab_v3',start_frame=0,fixed_pos=True)
    startup='startup' if os.environ.get('BXI_EXECUTION_SUITE')=='1' else 'on_demand'
    key=ResourceKey(PREFIX+'dance_policy');context.register_resource(key,load_cxk,policy=startup);handles['dance']=context.resource(key)
    for clip in ('06','11'):
        key=ResourceKey(PREFIX+'sonic'+clip+'_policy')
        context.register_resource(key,lambda resource,clip=clip:SonicFilePolicy(
            resource.asset('assets/sonic'+clip+'.npz'),clip_id=clip,logger=_SilentLogger()),policy=startup)
        handles['sonic'+clip]=context.resource(key)
    normal=context.resource(ResourceKey('com.bxi.basic_actions/normal_policy'))
    return ModDefinition(remote_event_filter=guard_module.SuiteRemoteGuard(
        isolate=os.environ.get('BXI_EXECUTION_SUITE')=='1'),
        transition_plugins={CxkEntryTransition.type_name:CxkEntryTransition,
                            ModelEntryTransition.type_name:ModelEntryTransition},
        state_observer=session.observe,state_factories={
        'ground_prepare':lambda state:GroundPrepare(state.name,state.state_id,normal,tuple(handles.values()),session),
        **{stage:lambda state,stage=stage:SuiteMotion(state.name,state.state_id,handles[stage],stage) for stage in ORDER}})


class _SilentLogger:
    def info(self,message):pass
    def warning(self,message):pass
    def error(self,message):pass
