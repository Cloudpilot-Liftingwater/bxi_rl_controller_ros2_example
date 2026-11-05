"""State-aware keyboard for the task's isolated BXI simulation only."""
import argparse
from dataclasses import dataclass
import json
import math
import os
import select
import sys
import termios
import time
import tty

PD='com.bxi.basic_actions/pd_brake'
NORMAL='com.bxi.basic_actions/normal'
DANCE='com.bxi.dance_test/dance'


@dataclass
class StartupGate:
    state: str = ''
    transition: bool = True
    elapsed: float = 0.
    state_rx: float = -math.inf
    command_rx: float = -math.inf
    normal_since: float | None = None
    normal_commands: int = 0

    def state_update(self, info, now):
        self.state=str(info.get('current',{}).get('name',''))
        self.elapsed=float(info.get('current',{}).get('elapsed',0))
        self.transition=bool(info.get('in_transition',True))
        self.state_rx=now
        if self.state==NORMAL and not self.transition:
            if self.normal_since is None:self.normal_since=now
        else:
            self.normal_since=None;self.normal_commands=0

    def command_update(self, names, position, kp, now):
        from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS
        BODY=ELF3_POLICY_JOINTS.names
        # A real complete body command, not a timer/state heartbeat.
        if len(set(names))!=len(names):return
        indices={n:i for i,n in enumerate(names)}
        if not set(BODY)<=indices.keys():return
        try:
            for name in BODY:
                i=indices[name]
                if not math.isfinite(position[i]) or not math.isfinite(kp[i]) or kp[i]<=0:return
        except (IndexError,TypeError):return
        self.command_rx=now
        if self.normal_since is not None:self.normal_commands+=1

    def ready_to_release(self, now):
        return (self.state==NORMAL and not self.transition and self.normal_since is not None
                and now-self.normal_since>=.6 and now-self.state_rx<.5
                and now-self.command_rx<.2 and self.normal_commands>=15)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--event',choices=['brake','walk','dance'])
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!=os.environ.get('ELF3_SIM_DOMAIN_ID','73'):
        raise SystemExit('Use mujoco_dance.sh; isolated local simulation domain required.')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from communication.msg import MotionCommands,ActuatorCmds
    from communication.srv import RobotReset
    rclpy.init();node=Node('elf3_eval_sim_keyboard');gate=StartupGate()
    pub=node.create_publisher(MotionCommands,'/motion_commands',10)
    def state(msg):
        try:gate.state_update(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):pass
    node.create_subscription(String,'/simulation/state_machine_info',state,10)
    node.create_subscription(ActuatorCmds,'/simulation/actuators_cmds',
        lambda m:gate.command_update(m.actuators_name,m.pos,m.kp,time.monotonic()),qos_profile_sensor_data)
    reset=node.create_client(RobotReset,'/simulation/robot_reset')

    def wait_for(predicate, seconds, message):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            if predicate():return
        raise RuntimeError(message)

    def emit(slot,value):
        wait_for(lambda:pub.get_subscription_count()>0,5,'No simulation controller subscribes to motion commands.')
        message=MotionCommands();setattr(message,slot,value);pub.publish(message)
        start=time.monotonic()
        while time.monotonic()-start<.15:rclpy.spin_once(node,timeout_sec=.02)
        pub.publish(MotionCommands())

    def release():
        wait_for(lambda:gate.ready_to_release(time.monotonic()),12,
            'Suspension kept: normal did not produce fresh, complete PD targets for 0.6 seconds.')
        if not reset.service_is_ready():raise RuntimeError('Suspension kept: simulation reset service unavailable.')
        request=RobotReset.Request();request.header.frame_id='elf3';request.reset_step=2;request.release=True
        future=reset.call_async(request)
        wait_for(future.done,5,'Simulation release response timed out; inspect simulator before continuing.')
        if future.result() is None or not future.result().is_success:raise RuntimeError('Simulator rejected release.')
        print('normal confirmed; simulation suspension released.',flush=True)

    def event(which):
        wait_for(lambda:time.monotonic()-gate.state_rx<.5,12,'No fresh simulation state. Start run first.')
        if which=='brake':
            if gate.state!=PD:emit('btn_3',1)
            wait_for(lambda:gate.state==PD and not gate.transition,5,'pd_brake was not reached.')
            print('pd_brake confirmed. Initial suspension stays on; press N after startup ramp.',flush=True)
        elif which=='walk':
            if gate.state!=PD:raise RuntimeError(f'Press B first; normal is entered from pd_brake (current={gate.state}).')
            wait_for(lambda:gate.state==PD and gate.elapsed>=3.2 and not gate.transition,8,'PD startup ramp not ready.')
            emit('btn_1',1)
            wait_for(lambda:gate.state==NORMAL and not gate.transition,8,'normal transition did not finish; suspension kept.')
            release()
        else:
            if gate.state not in (NORMAL,DANCE) or gate.transition:
                raise RuntimeError(f'Dance needs settled normal, or active dance to cancel (current={gate.state}).')
            emit('btn_6',2)
            print('Dance event sent.',flush=True)

    saved=None
    try:
        if args.event:event(args.event);return
        if not sys.stdin.isatty():raise RuntimeError('Interactive terminal required.')
        print('BXI simulation: B=pd_brake, N=normal + confirmed release, D=dance/cancel, Q=quit keyboard.',flush=True)
        print('Initially suspended. Click this terminal; press B, then N, then D.',flush=True)
        saved=termios.tcgetattr(sys.stdin);tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if key=='q':break
                which={'b':'brake','n':'walk','d':'dance'}.get(key)
                if which:
                    try:event(which)
                    except RuntimeError as e:print(str(e),flush=True)
    finally:
        if saved is not None:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
