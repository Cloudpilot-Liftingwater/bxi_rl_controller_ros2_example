"""Compile original native mapper and drive actual controller state routes."""
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace as NS
import subprocess

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_prefix

from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter
from test_execution_suite import module
from test_model_test_manual import machine, reach_motion_phase, NORMAL, PREFIX, ROOT

REMOTE=ROOT.parent/'remote_controller'


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    out=tmp_path_factory.mktemp('native-remote')/'probe'
    includes=[]
    for pkg in ('communication','builtin_interfaces','rosidl_runtime_c','rosidl_runtime_cpp',
                'std_msgs','geometry_msgs','sensor_msgs','rosidl_typesupport_interface','rcutils'):
        includes.extend(['-I',str(Path(get_package_prefix(pkg))/'include'/pkg)])
    cmd=['c++','-std=c++14','-O2','-I',str(REMOTE/'include'),*includes,
         str(Path(__file__).with_name('native_remote_probe.cpp')),
         *(str(REMOTE/'src'/f) for f in ('config.cpp','input_mapper.cpp','motion_commands_adapter.cpp')),
         '-lyaml-cpp','-o',str(out)]
    result=subprocess.run(cmd,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    def read(snapshots):
        result=subprocess.run([str(out),str(REMOTE/'config/xbox_default.yaml')],
            input='\n'.join(' '.join(str(int(k in snapshot)) for k in ('lb','rb','rt','a','b','x','y','view'))
                            for snapshot in snapshots)+'\n',check=True,capture_output=True,text=True)
        return [NS(**{f'btn_{i+1}':int(v) for i,v in enumerate(line.split()[1:])})
                for line in result.stdout.splitlines() if line.startswith('MSG ')]
    return read


def adapter():
    events={}
    for name in ('basic_actions','suspended_tests','execution_suite'):
        config=yaml.safe_load((ROOT/f'mods/com.bxi.{name}/mod.yaml').read_text())
        events.update({f'com.bxi.{name}/'+key:binding for key,binding in config['events'].items()})
    return RemoteEventAdapter(events)


@pytest.mark.parametrize('face,stage',[('a','dance'),('x','sonic06'),('y','sonic11')])
def test_all_native_press_release_orders_select_once_and_never_trigger_release_actions(probe,face,stage):
    for press in permutations(('lb','rt',face)):
        for release in permutations(('lb','rt',face)):
            snapshots=[set()]; held=set()
            for key in press:
                held.add(key);snapshots.append(set(held))
            for key in release:
                held.remove(key);snapshots.append(set(held))
            guard=module('mods/com.bxi.execution_suite/remote_guard.py','native_guard').SuiteRemoteGuard()
            a=adapter(); selected=[];requested=False
            for msg in probe(snapshots):
                # Include the case where a short motion finished before the
                # initiating keys were released: guard must protect normal.
                events=guard(msg,a.extract_events(msg),NORMAL)
                if PREFIX+stage in events:
                    requested=True;selected.append(PREFIX+stage)
                if requested:
                    assert not set(events)&{'com.bxi.basic_actions/recover','com.bxi.basic_actions/lie_down'}
            assert selected==[PREFIX+stage]


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
@pytest.mark.parametrize('phase',['entry','active','exit'])
@pytest.mark.parametrize('escape',['normal','zero'])
def test_native_escape_reaches_actual_normal_or_zero_frame(monkeypatch,probe,stage,phase,escape):
    sm,states,ctx,frames,tick=machine(monkeypatch,include_zero=True)
    tick(PREFIX+stage)
    reach_motion_phase(sm,states,tick,stage,phase)
    a=adapter();guard=module('mods/com.bxi.execution_suite/remote_guard.py','escape_guard').SuiteRemoteGuard()
    buttons={'rb','x'} if escape=='normal' else {'view'}
    for msg in probe([set(),buttons,set()]):
        for event in guard(msg,a.extract_events(msg),sm.current_state_name):
            tick(event)
    for _ in range(8):tick()
    assert sm.current_state_name==(NORMAL if escape=='normal' else 'com.bxi.basic_actions/zero_torque')
    if escape=='zero':
        assert np.all(frames[-1].kp==0) and np.all(frames[-1].kd==0)
        assert np.all(frames[-1].torque==0)


def test_suspended_commands_keep_original_mapping_and_view_exit(probe):
    for key,field,value in [('a','btn_7',1),('b','btn_9',2),('x','btn_9',1),('y','btn_10',1)]:
        msg=probe([set(),{key}])[-1]
        assert getattr(msg,field)==value
    msg=probe([{'lb','rt','a','view'}])[-1]
    assert msg.btn_8==2 and msg.btn_2==1


@pytest.mark.parametrize('face,stage',[('a','dance'),('x','sonic06'),('y','sonic11')])
def test_start_from_forward_back_and_wait_in_normal_after_one_motion(monkeypatch,probe,face,stage):
    sm,states,ctx,frames,tick=machine(monkeypatch,initial_state='com.bxi.basic_actions/forward_back')
    a=adapter();guard=module('mods/com.bxi.execution_suite/remote_guard.py','walk_guard').SuiteRemoteGuard()
    for msg in probe([set(),{'lb','rt',face},set()]):
        for event in guard(msg,a.extract_events(msg),sm.current_state_name):tick(event)
    for _ in range(240):tick()
    assert states[stage].result=='playback_complete'
    assert sm.current_state_name==NORMAL and not sm.in_transition
    for _ in range(100):tick()
    assert sm.current_state_name==NORMAL
    assert all(states[other].policy.steps==0 for other in states if other!=stage)


@pytest.mark.parametrize('stage',['dance','sonic06','sonic11'])
@pytest.mark.parametrize('phase',['entry','active'])
def test_invalid_model_returns_to_normal_without_publishing_nan(monkeypatch,stage,phase):
    sm,states,ctx,frames,tick=machine(monkeypatch,include_zero=True)
    tick(PREFIX+stage)
    reach_motion_phase(sm,states,tick,stage,phase)
    states[stage].policy.buffer.position[:]=np.nan
    for _ in range(12):tick()
    assert sm.current_state_name==NORMAL and not sm.in_transition
    assert states[stage].result=='failed'
    assert all(np.isfinite(frame.qpos).all() for frame in frames)


def test_one_failed_model_does_not_start_or_lock_the_next_manual_test(monkeypatch):
    sm,states,ctx,frames,tick=machine(monkeypatch)
    tick(PREFIX+'dance')
    states['dance'].policy.buffer.position[:]=np.nan
    for _ in range(20):tick()
    assert sm.current_state_name==NORMAL
    assert states['sonic06'].policy.steps==0
    assert states['sonic06'].is_available(ctx)
    tick(PREFIX+'sonic06')
    for _ in range(240):tick()
    assert states['sonic06'].result=='playback_complete' and sm.current_state_name==NORMAL


def test_original_mods_and_remote_cpp_are_byte_identical_to_official():
    import hashlib
    workspace=ROOT.parents[1]
    tracked=subprocess.check_output(['git','ls-tree','-r','--name-only','e1d412f',
        'src/bxi_example_py_elf3/mods','src/remote_controller'],cwd=workspace,text=True).splitlines()
    for filename in tracked:
        if filename=='src/remote_controller/config/xbox_default.yaml':continue
        if Path(filename).suffix not in ('.py','.yaml','.cpp','.hpp','.txt','.md','.sh','.xml'):
            continue  # Partial clones can omit unchanged external model blobs.
        expected=subprocess.check_output(['git','show','e1d412f:'+filename],cwd=workspace)
        path=workspace/filename
        actual=str(path.readlink()).encode() if path.is_symlink() else path.read_bytes()
        assert hashlib.sha256(actual).digest()==hashlib.sha256(expected).digest(),filename
    assert not list((REMOTE/'config').glob('execution_suite_*.yaml'))
