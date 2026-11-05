"""No hardware: real dance decoder, ONNX Runtime, and evaluator end to end.

The generated model/motion are validation fixtures, never deployable dance assets.
Set PYTHONPATH to this package and the evaluator source before running.
"""
import ast
import importlib
import json
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import pytest
import yaml

from bxi_example_py_elf3.control.elf3 import JOINT_NAMES
from bxi_example_py_elf3.framework.inference import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointLayout, JointStateBuffer
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from elf3_eval import host
from elf3_eval.analysis import analyze, Options
from elf3_eval.contract import BODY, angle_event
from elf3_eval.storage import import_recording

ROOT = Path(__file__).resolve().parents[1]
MOD = ROOT/'mods/com.bxi.dance_test'


@pytest.fixture
def modules(monkeypatch):
    pkg = ModuleType('_dance_execution_test')
    pkg.__path__ = [str(MOD)]
    monkeypatch.setitem(sys.modules,pkg.__name__,pkg)
    return NS(policy=importlib.import_module(pkg.__name__+'.policy'),
              state=importlib.import_module(pkg.__name__+'.state'),
              plugin=importlib.import_module(pkg.__name__+'.plugin'))


@pytest.fixture
def assets(tmp_path):
    model_path=tmp_path/'fixture.onnx';motion_path=tmp_path/'fixture.npz'
    weights=np.linspace(.002,.008,29,dtype=np.float32).reshape(1,29)
    graph=helper.make_graph([
        helper.make_node('MatMul',['time_step','weights'],['actions']),
    ],'validation_only',[
        helper.make_tensor_value_info('obs',TensorProto.FLOAT,[1,154]),
        helper.make_tensor_value_info('time_step',TensorProto.FLOAT,[1,1]),
    ],[helper.make_tensor_value_info('actions',TensorProto.FLOAT,[1,29])],
      [numpy_helper.from_array(weights,name='weights')])
    model=helper.make_model(graph,opset_imports=[helper.make_opsetid('',13)])
    model.ir_version=8
    scale=np.linspace(.1,.3,29,dtype=np.float32)
    for key,value in dict(joint_names=','.join(JOINT_NAMES),default_joint_pos=','.join(['0']*29),
                          joint_stiffness=','.join(['10']*29),joint_damping=','.join(['1']*29),
                          action_scale=','.join(map(str,scale))).items():
        prop=model.metadata_props.add();prop.key=key;prop.value=value
    onnx.save(model,str(model_path))
    n=24
    quat=np.zeros((n,1,4),np.float32);quat[:,:,0]=1
    np.savez(motion_path,joint_pos=np.zeros((n,29),np.float32),joint_vel=np.zeros((n,29),np.float32),
             body_pos_w=np.zeros((n,1,3),np.float32),body_quat_w=quat,joint_names=np.array(JOINT_NAMES))
    return NS(model=model_path,motion=motion_path,scale=scale,weights=weights.reshape(-1),frames=n)


class Logger:
    def __init__(self):self.messages=[]
    def debug(self,s):self.messages.append(s)
    def info(self,s):self.messages.append(s)
    def warning(self,s):self.messages.append(s)
    def error(self,s):self.messages.append(s)


def context():
    joints=JointStateBuffer(JointLayout(JOINT_NAMES))
    joints.position.fill(0);joints.velocity.fill(0);joints.view.timestamp_ns=1
    inference=InferenceFrame(joints=joints.view,quat_wxyz=np.array([1.,0,0,0]),angular_velocity=np.zeros(3))
    frames=[];requests=[]
    ctx=NS(robot_layout=joints.layout,robot_joints=joints.view,inference_frame=inference,
           current_quat_xyzw=np.array([0.,0,0,1.]),is_orientation_unsafe=lambda q:False,
           set_motor_target=frames.append,request_state=lambda target,**kwargs:requests.append((target,kwargs)) or True)
    return ctx,frames,requests


def state_with_policy(modules,assets):
    policy=modules.policy.load_dance_policy(assets.motion,assets.model)
    state=modules.state.DanceTestState('com.bxi.dance_test/dance',99,NS(get=lambda:policy))
    state._bind_logger(Logger())
    return state,policy


def test_dance_routes_are_separate_and_have_cancel_and_safety():
    config=yaml.safe_load((MOD/'mod.yaml').read_text())
    assert config['events']['dance']==dict(slot='btn_6',value=2)
    assert [r['from'] for r in config['routes'] if r.get('to')=='dance']==['com.bxi.basic_actions/normal']
    assert any(r.get('action')=='cancel' for r in config['actions'])
    assert any(r.get('to')=='com.bxi.basic_actions/zero_torque' for r in config['routes'])
    remote=yaml.safe_load((ROOT.parent/'remote_controller/config/xbox_default.yaml').read_text())
    assert {r['output'] for r in remote['outputs']['level']} >= {'btn_10=31','btn_10=33','btn_10=34'}


def test_all_builtin_mod_manifests_compile_together_without_input_conflicts(monkeypatch):
    # Only manifest composition is under test; no installed ROS package lookup.
    ament=ModuleType('ament_index_python.packages')
    ament.PackageNotFoundError=type('PackageNotFoundError',(LookupError,),{})
    def no_lookup(name):raise AssertionError('manifest composition queried ROS installation')
    ament.get_package_prefix=no_lookup
    monkeypatch.setitem(sys.modules,'ament_index_python',ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules,'ament_index_python.packages',ament)
    from bxi_example_py_elf3.framework.runtime.mod_loader import _discover_mods, _compose_config
    from bxi_example_py_elf3.framework.mod_api import ModDefinition
    mods=_discover_mods((ROOT/'mods',))
    # Startup assets/ROS packages are deliberately absent here. This checks the
    # real manifest composer with placeholder factories, not robot execution.
    definitions={key:ModDefinition(state_factories={name:lambda context:None
                 for name in mod.manifest['states']}) for key,mod in mods.items()}
    config,factories=_compose_config(yaml.safe_load((ROOT/'config/elf3_state_machine.yaml').read_text()),
                                     tuple(mods.values()),definitions)
    assert 'com.bxi.dance_test/dance' in factories
    assert config['remote_events']['com.bxi.dance_test/dance']==dict(slot='btn_6',value=2)


def test_real_onnx_decoder_scale_and_preview(modules,assets):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context();policy.reset(ctx.inference_frame)
        policy.step(ctx.inference_frame,.02,advance=True)
        output=policy.step(ctx.inference_frame,.02,advance=False)
        np.testing.assert_allclose(output.joints.position,assets.weights*assets.scale,rtol=2e-6)
        assert policy.inner._frame==1
        assert policy.asset_info['backend']=='onnxruntime'
    finally:policy.close()


def test_start_frame_and_heading_setting_reach_real_decoder(modules,assets):
    policy=modules.policy.load_dance_policy(assets.motion,assets.model,start_frame=3,fixed_pos=True)
    try:
        ctx,_,_=context();policy.reset(ctx.inference_frame)
        assert policy.inner.fixed_pos is True
        assert policy.inner._frame==3
        output=policy.step(ctx.inference_frame,.02)
        np.testing.assert_allclose(output.joints.position,3*assets.weights*assets.scale,rtol=2e-6)
        assert policy.asset_info['start_frame']==3
        assert policy.asset_info['fixed_pos'] is True
        assert policy.asset_info['playback_seconds']==pytest.approx((assets.frames-3)/50)
    finally:policy.close()


@pytest.mark.parametrize('kwargs',[{'start_frame':-1},{'start_frame':24},{'start_frame':1.2},
                                    {'start_frame':True},{'fixed_pos':'false'}])
def test_invalid_playback_settings_rejected_before_backend(modules,assets,kwargs):
    with pytest.raises(ValueError):modules.policy.load_dance_policy(assets.motion,assets.model,**kwargs)


def test_plugin_passes_explicit_load2_settings(modules,monkeypatch):
    captured={}
    def load(*args,**kwargs):captured.update(args=args,**kwargs);return captured
    monkeypatch.setattr(modules.plugin,'load_dance_policy',load)
    for key,value in dict(BXI_DANCE_MOTION='/selected/cxkv5.npz',BXI_DANCE_MODEL='/selected/cxkv5_15k.onnx',
                          BXI_DANCE_VARIANT='gravity_isaaclab_v3',BXI_DANCE_START_FRAME='0',BXI_DANCE_FIXED_POS='true').items():
        monkeypatch.setenv(key,value)
    modules.plugin._load_policy(NS(mod_root=MOD))
    assert captured==dict(args=('/selected/cxkv5.npz','/selected/cxkv5_15k.onnx'),
                         variant='gravity_isaaclab_v3',start_frame=0,fixed_pos=True)
    monkeypatch.setenv('BXI_DANCE_FIXED_POS','typo')
    with pytest.raises(ValueError):modules.plugin._load_policy(NS(mod_root=MOD))


def test_playback_finishes_once_without_accuracy_pass(modules,assets):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context();policy.reset(ctx.inference_frame);state.on_enter(ctx)
        for _ in range(assets.frames+2):state.on_update(ctx,.02)
        assert len(frames)==assets.frames
        assert len(requests)==1
        assert requests[0][0]==modules.state.NORMAL
        assert state.result=='playback_complete'
        assert not any('SUCCESS' in s for s in state.logger.messages)
    finally:policy.close()


def test_cancel_and_interruption_are_not_completion(modules,assets):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context();state.on_enter(ctx)
        assert state.on_action(ctx,'cancel');assert state.result=='cancelled'
        state.on_exit(ctx);assert state.result=='cancelled'
        state.on_enter(ctx);state.on_exit(ctx);assert state.result=='interrupted'
    finally:policy.close()


def test_exit_blend_holds_last_frame_without_out_of_range_inference(modules,assets,monkeypatch):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context();state.on_enter(ctx)
        for _ in range(assets.frames):state.on_update(ctx,.02)
        expected=policy.output.joints.position.copy()
        def fail_if_called(inputs):raise AssertionError('inference beyond final frame')
        monkeypatch.setattr(policy.inner._backend,'run',fail_if_called)
        held=state.sample_running_frame(ctx,.02,advance=False)
        np.testing.assert_array_equal(held.qpos,expected)
        assert policy.inner._frame==assets.frames
    finally:policy.close()


def test_real_state_machine_enters_dance_and_returns_after_final_frame(modules,assets):
    from bxi_example_py_elf3.framework.mod_api import PoseState
    from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine
    class NormalPose(PoseState):
        def target_position(self,ctx):return np.zeros(29,np.float32)
        def gains(self,ctx):return np.full(29,10,np.float32),np.ones(29,np.float32)
    state,policy=state_with_policy(modules,assets)
    state._policy_handle.status='ready'
    ctx,frames,requests=context()
    ctx.last_motor_frame=MotorFrame.create(ctx.robot_layout,np.zeros(29),np.full(29,10),np.ones(29))
    def resolve(frame,output):
        output.update(frame.qpos,frame.kp,frame.kd,vel=frame.vel,torque=frame.torque)
        return output
    ctx.resolve_motor_frame=resolve
    def publish(frame):
        frames.append(frame)
        resolve(frame,ctx.last_motor_frame)
    ctx.set_motor_target=publish
    def preheat(model):
        model.reset(ctx.inference_frame);model.step(ctx.inference_frame,.02,advance=False)
    ctx.preheat_model=preheat
    normal=NormalPose(modules.state.NORMAL,1)
    normal._bind_logger(Logger())
    entry=yaml.safe_load((MOD/'mod.yaml').read_text())['routes'][0]['transition']
    config=dict(initial_state=normal.name,graph=dict(validate=False),default_transition='instant',
        transition_profiles={'instant':{'type':'instant'}},
        states={normal.name:{'transitions':{'on_event':{'dance':{'to':state.name,'transition':entry}}}},
                state.name:{}},remote_events={'dance':dict(slot='btn_6',value=2)})
    machine=RobotStateMachine(ctx,config,{normal.name:normal,state.name:state},logger=Logger())
    ctx.request_state=lambda target,**kwargs:machine.request_transition(target,**kwargs)
    try:
        entered=False
        for i in range(240):
            ctx.robot_joints.timestamp_ns+=1
            host.begin_cycle();host.set_transition(machine.in_transition)
            active=machine.update(.02,['dance'] if i==0 else [])
            host.set_transition(active)
            if not active:machine.update_current_state(.02)
            entered=entered or machine.current_state_name==state.name
            if entered and machine.current_state_name==normal.name and not machine.in_transition:
                break
        assert entered
        assert machine.current_state_name==normal.name
        assert not machine.in_transition
        assert state.result=='playback_complete'
        assert policy.inner._frame==assets.frames
    finally:policy.close()


@pytest.mark.parametrize('fault',['feedback','orientation','nonfinite','limit'])
def test_control_faults_do_not_complete_playback(modules,assets,fault):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context();state.on_enter(ctx)
        if fault=='feedback':state._last_feedback_at-=.3
        elif fault=='orientation':ctx.is_orientation_unsafe=lambda q:True
        else:
            value=np.nan if fault=='nonfinite' else 10000.
            policy.inner._backend.run=lambda inputs:dict(actions=np.full((1,29),value,np.float32))
        state.on_update(ctx,.02)
        assert state.result=='failed';assert not frames
        assert requests[-1][0]==(modules.state.ZERO if fault in ("feedback", "orientation") else modules.state.NORMAL)
    finally:policy.close()


def test_missing_and_wrong_assets_fail_before_control(modules,assets,tmp_path):
    with pytest.raises(FileNotFoundError):modules.policy.inspect_assets(tmp_path/'missing',assets.model)
    with pytest.raises(ValueError):modules.policy.inspect_assets(assets.motion,assets.model,variant='unknown')
    model=onnx.load(str(assets.model))
    next(p for p in model.metadata_props if p.key=='joint_names').value=','.join(reversed(JOINT_NAMES))
    onnx.save(model,str(assets.model))
    with pytest.raises(ValueError):modules.policy.inspect_assets(assets.motion,assets.model,variant='mjlab')


def test_invalid_preview_requests_fault_and_never_returns_command(modules,assets,monkeypatch):
    state,policy=state_with_policy(modules,assets)
    try:
        ctx,frames,requests=context()
        monkeypatch.setattr(policy.inner._backend,'run',lambda inputs:dict(actions=np.full((1,29),np.nan,np.float32)))
        with pytest.raises(ValueError):state.sample_running_frame(ctx,.02,advance=False)
        assert state.result=='failed';assert not frames
        assert requests[-1][0]==modules.state.NORMAL
    finally:policy.close()


def test_observer_failure_leaves_policy_output_unchanged(modules,assets,monkeypatch):
    state,policy=state_with_policy(modules,assets)
    from bxi_example_py_elf3.framework import execution_observer
    def broken(*args,**kwargs):raise RuntimeError('injected observer failure')
    monkeypatch.setattr(execution_observer,'_host',NS(policy_output=broken))
    try:
        ctx,frames,requests=context();policy.step(ctx.inference_frame,.02)
        assert np.isfinite(policy.output.joints.position).all()
        assert policy.inner._frame==1
    finally:policy.close()


def publish_function():
    # Execute the actual final publisher method with transport-only substitutes.
    tree=ast.parse((ROOT/'bxi_example_py_elf3/bxi_example_demo.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='BxiExample')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_motor_frame')
    from bxi_example_py_elf3.framework.execution_observer import observe
    class Msg:
        def __init__(self):self.header=NS()
    namespace=dict(MotorFrame=MotorFrame,bxiMsg=NS(ActuatorCmds=Msg),robot_name='elf3',time=time,observe=observe)
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<real publish_motor_frame>','exec'),namespace)
    return namespace['publish_motor_frame']


def test_real_decoder_final_publisher_and_eval_known_error(modules,assets,tmp_path,monkeypatch):
    events=[]
    class Capture:
        errors=0
        def capture(self,factory):events.append(factory())
    monkeypatch.setattr(host,'_enabled',True);monkeypatch.setattr(host,'_tap',Capture())
    state,policy=state_with_policy(modules,assets)
    ctx,frames,requests=context()
    command_messages=[]
    names=('head_y_joint',)+tuple(reversed(BODY))+('head_z_joint',)
    def override(frame,**kwargs):
        q=[9.]+(frame.qpos[::-1]+.02).tolist()+[-9.]
        return MotorFrame.create(JointLayout(names),q,[10.]*31,[1.]*31)
    def stamp():
        now=time.monotonic_ns()+10**12
        return NS(sec=now//10**9,nanosec=now%10**9)
    publisher=NS(_actuator_publisher_conflict=False,motor_override_allow_in_zero_torque=False,
                 runtime=NS(framework=NS(current_state_name=state.name,state_machine=NS(in_transition=False))),
                 _motor_override=NS(apply=override),get_clock=lambda:NS(now=lambda:NS(to_msg=stamp)),
                 act_pub=NS(publish=command_messages.append))
    publish=publish_function()
    try:
        policy.reset(ctx.inference_frame);state.on_enter(ctx)
        for i in range(assets.frames):
            ctx.robot_joints.timestamp_ns+=1
            host.begin_cycle();state.on_update(ctx,.02)
            model_target=policy.output.joints.position.copy()
            publish(publisher,frames[-1])
            msg=command_messages[-1];rx=time.monotonic()
            source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            events.append(dict(kind='command',received_monotonic=rx,ros_now_ns=int((rx+1000)*1e9),
                value=dict(stamp_ns=source_ns,names=list(names),position=msg.pos,kp=msg.kp)))
            events.append(angle_event('actual',time.monotonic(),[9.]+(model_target[::-1]+.05).tolist()+[-9.],names=names))
            # Real time, to avoid making 50 Hz inference look like a 1 kHz run.
            time.sleep(.02)
        source=tmp_path/'events.jsonl'
        source.write_text(''.join(json.dumps(e)+'\n' for e in events))
        meta=import_recording(source,tmp_path/'run.sqlite')
        result=analyze(tmp_path/'run.sqlite',tmp_path/'report',Options(command_age_ms=100,model_age_ms=100))
        assert not any(k.startswith('rejected:') for k in meta['counts'])
        assert result['summary']['model_valid_body29_frames']==assets.frames
        for segment in result['segments']:
            comparisons=segment['comparisons']
            if comparisons['actual_minus_model']['body29']['n']:
                assert comparisons['actual_minus_model']['body29']['mae_deg']==pytest.approx(np.degrees(.05),abs=1e-5)
                assert comparisons['actual_minus_command']['body29']['mae_deg']==pytest.approx(np.degrees(.03),abs=1e-5)
                assert comparisons['command_minus_model']['body29']['mae_deg']==pytest.approx(np.degrees(.02),abs=1e-5)
        assert any(e['status']=='playback_complete' for e in result['test_events'])
        assert result['accuracy_acceptance']=='not_assessed_no_user_threshold'
        arrays=np.load(tmp_path/'report/aligned.npz',allow_pickle=False)
        assert any(arrays[k].all() for k in arrays.files if k.endswith('_valid_model'))
    finally:policy.close()
