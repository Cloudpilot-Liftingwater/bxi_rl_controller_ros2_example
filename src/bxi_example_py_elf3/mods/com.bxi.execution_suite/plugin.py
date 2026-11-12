"""Manual model checks from existing walking states; no suspended-test handoff."""
import importlib
import importlib.util
import os
from pathlib import Path
import sys
from types import ModuleType

import numpy as np

from bxi_example_py_elf3.framework.execution_observer import observe
from bxi_example_py_elf3.framework.mod_api import ModDefinition,ResourceKey
from bxi_example_py_elf3.sonic_file import SonicFilePolicy
from bxi_example_py_elf3.model_test_safety import ModelTemperatureGuard
from bxi_example_py_elf3.model_test_transitions import CxkEntryTransition, ModelEntryTransition

PREFIX='com.bxi.execution_suite/'
NORMAL='com.bxi.basic_actions/normal'
ZERO='com.bxi.basic_actions/zero_torque'
ORDER=('dance','sonic06','sonic11')


def enabled(ctx):
    return (getattr(ctx.ros_node,'topic_prefix','') in ('simulation/','hardware/')
            and os.environ.get('BXI_EXECUTION_SUITE_MODE','manual')=='manual')


class Session:
    def __init__(self):
        self.active=False;self.ctx=None;self.completed=[];self.logger=None;self.faulted=False

    def mark(self,status,reason='',**details):
        severity='error' if status=='failed' else 'warning' if status in ('cancelled','interrupted') else 'info'
        # The model state writes the single concise human-readable result.
        observe('test_event',PREFIX+'suite',status,reason,severity=severity,**details)

    def begin(self,ctx,logger):
        self.active=True;self.ctx=ctx;self.logger=logger;self.faulted=False
        self.mark('started','standalone walking-mode model test; select one action manually')

    def complete(self,stage):
        if not self.active:return
        self.completed.append(stage)
        self.mark('stage_complete',stage)
        self.active=False

    def abort(self,reason):
        if self.active:self.mark('failed',reason)
        self.active=False;self.faulted=True

    def cancel(self,reason):
        if self.active:self.mark('cancelled',reason,code='OPERATOR_RETURN_NORMAL',category='operator')
        self.active=False


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
            return enabled(ctx)
        def on_prepare(self,ctx,from_state):
            if not session.active:
                session.begin(ctx,self.logger)
            if self.stage.startswith('sonic'):self.policy.inner.bind_logger(self.logger)
            super().on_prepare(ctx,from_state)
        def on_prepare_cancel(self,ctx,from_state):
            super().on_prepare_cancel(ctx,from_state)
            if self.result=='cancelled':session.cancel(self.stage+' entry cancelled')
        def on_enter(self,ctx):
            self.reported=False;super().on_enter(ctx)
            # Score tracking error over the full clip, including samples above 20 degrees.
            self._accuracy.stop_on_maximum = False
        def _fail(self,ctx,reason,**details):
            session.abort(self.stage+': '+reason)
            super()._fail(ctx,reason,**details)
        def sample_running_frame(self,ctx,dt,*,advance):
            try:
                return super().sample_running_frame(ctx,dt,advance=advance)
            except Exception:
                if self.result!='failed':
                    raise
                # A blend previews the target outside on_update. Keep its
                # last valid command for this tick while the queued fault
                # transition returns to normal/zero; never return bad output
                # or let the diagnostic terminate the controller thread.
                from bxi_example_py_elf3.framework.mod_api import MotorFrame
                last=ctx.last_motor_frame
                return MotorFrame.create(last.layout,last.qpos.copy(),last.kp.copy(),
                    last.kd.copy(),vel=last.vel.copy(),torque=last.torque.copy())
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
        root=Path(os.environ.get('BXI_CXK_ROOT') or '/data/ros2_ws/备份/com.bxi.cxk')/'assets'
        return dance_policy.load_dance_policy(root/'cxkv5.npz',root/'cxkv5_15k.onnx',
                                             variant='gravity_isaaclab_v3',start_frame=0,fixed_pos=True)
    standalone=os.environ.get('BXI_EXECUTION_SUITE')=='1'
    startup='startup' if standalone else 'on_demand'
    key=ResourceKey(PREFIX+'dance_policy');context.register_resource(key,load_cxk,policy=startup);handles['dance']=context.resource(key)
    for clip in ('06','11'):
        key=ResourceKey(PREFIX+'sonic'+clip+'_policy')
        context.register_resource(key,lambda resource,clip=clip:SonicFilePolicy(
            resource.asset('assets/sonic'+clip+'.npz'),clip_id=clip,logger=_SilentLogger()),policy=startup)
        handles['sonic'+clip]=context.resource(key)
    return ModDefinition(remote_event_filter=guard_module.SuiteRemoteGuard(isolate=standalone),
        transition_plugins={CxkEntryTransition.type_name:CxkEntryTransition,
                            ModelEntryTransition.type_name:ModelEntryTransition},
        state_factories={stage:lambda state,stage=stage:SuiteMotion(
            state.name,state.state_id,handles[stage],stage) for stage in ORDER})


class _SilentLogger:
    def info(self,message):pass
    def warning(self,message):pass
    def error(self,message):pass
