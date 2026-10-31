"""Separate ROS processes: sensor-only guide, independent object, audit observer."""
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import random
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from .core import (Pose, PoseHistory, RangeTracker, FrozenPolicy, RouteFollower, ACTIONS,
                   ROBOT_GOAL, OBJECT_GOAL, progress, limited_speed)

ROOT = Path(os.environ.get('RANGE_ROOT', '/workspace/range'))
OUT = Path(os.environ.get('RANGE_OUTPUT', '/workspace/output'))


class SimNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.set_parameters([Parameter('use_sim_time', value=True)])

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return Pose(p.x, p.y, yaw, msg.twist.twist.linear.x)

    @staticmethod
    def velocity(pub, command):
        msg = Twist();msg.linear.x=float(command[0]);msg.angular.z=float(command[1])
        pub.publish(msg)


class Guide(SimNode):
    """No subscription to the object's odometry, status, or command topics."""
    def __init__(self):
        super().__init__('range_only_guide')
        self.robot = None;self.robot_stamp = None
        self.robot_history=PoseHistory()
        self.tracker=RangeTracker();self.follower=RouteFollower(ROBOT_GOAL)
        self.policy=FrozenPolicy(ROOT/'models/q_final.json')
        self.started=None;self.boot=None;self.previous=None;self.next_action=0.
        self.action=0;self.state='READY';self.command=(0.,0.)
        self.last_distance=None;self.last_distance_time=None;self.closing=0.
        self.cmd=self.create_publisher(Twist,'/guide/cmd_vel',1)
        self.status=self.create_publisher(String,'/range/status',10)
        self.start_pub=self.create_publisher(Bool,'/range/start',10)
        self.done_pub=self.create_publisher(Bool,'/range/done',10)
        self.create_subscription(Odometry,'/guide/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/guide/scan',self.scan,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    def odometry(self,msg):
        self.robot=self.pose(msg)
        self.robot_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.robot_history.add(self.robot_stamp,self.robot)

    def scan(self,msg):
        if self.robot is None:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        robot=self.robot_history.at(stamp)
        if robot is None:return
        observed=self.tracker.update(stamp,robot,msg.angle_min,msg.angle_increment,
                                     msg.ranges,msg.range_min,msg.range_max)
        if observed:
            distance=math.hypot(observed.x-robot.x,observed.y-robot.y)
            if self.last_distance_time is not None and stamp>self.last_distance_time:
                self.closing=(self.last_distance-distance)/(stamp-self.last_distance_time)
            self.last_distance=distance;self.last_distance_time=stamp

    def finish(self,reason):
        self.state=reason;self.command=(0.,0.);self.follower.previous_v=0.
        self.velocity(self.cmd,self.command)

    def tick(self):
        now=self.now()
        if self.boot is None:self.boot=now
        if self.robot is None:return
        dt=.05 if self.previous is None else min(.1,max(0.,now-self.previous))
        self.previous=now
        terminal=self.state in ('ARRIVED','TOO_CLOSE','TOO_FAR','TIMEOUT','ODOM_LOST')
        fresh=self.tracker.fresh(now)
        if self.started is None and fresh and self.tracker.detected_scans>=20 and now-self.boot>=3.:
            self.started=now;self.state='GUIDING'
            self.get_logger().info('Starting fixed Q policy with LaserScan-only object observations')
        elapsed=0. if self.started is None else now-self.started
        if now-self.robot_stamp>.5:
            self.finish('ODOM_LOST');terminal=True
        elif self.started is None or terminal:
            self.command=(0.,0.)
        elif elapsed>100:
            self.finish('TIMEOUT');terminal=True
        elif not fresh:
            self.state='TARGET_LOST';self.command=(0.,0.);self.follower.previous_v=0.
        else:
            target=self.tracker.observation
            distance=math.hypot(self.robot.x-target.x,self.robot.y-target.y)
            gap=progress(self.robot)-progress(target.pose())
            if distance<.85:
                self.finish('TOO_CLOSE');terminal=True
            elif gap>3.:
                self.finish('TOO_FAR');terminal=True
            elif (math.hypot(self.robot.x-ROBOT_GOAL[0],self.robot.y-ROBOT_GOAL[1])<.15 and
                  math.hypot(target.x-OBJECT_GOAL[0],target.y-OBJECT_GOAL[1])<.15 and
                  abs(self.robot.speed)<.04 and target.speed<.05):
                self.finish('ARRIVED');terminal=True
            else:
                self.state='GUIDING'
                if elapsed>=self.next_action:
                    self.action=self.policy.act(self.policy.state(self.robot,target,self.follower.phase))
                    self.next_action=elapsed+.2
                requested=limited_speed(ACTIONS[self.action],target.speed,gap)
                self.command=self.follower.command(self.robot,requested,dt)
        self.velocity(self.cmd,self.command)
        self.start_pub.publish(Bool(data=self.started is not None))
        self.done_pub.publish(Bool(data=terminal))
        observed=self.tracker.observation
        data={'time':round(now,3),'elapsed':round(elapsed,3),'state':self.state,
              'robot':self.robot.json(),'observation':asdict(observed) if observed else None,
              'observation_fresh':fresh,'distance':math.hypot(self.robot.x-observed.x,self.robot.y-observed.y) if observed else None,
              'target_motion':'STOPPED' if observed and observed.speed<.06 else 'APPROACHING' if self.closing>.08 else 'MOVING',
              'command':self.command,'action_index':self.action,'scans':self.tracker.received_scans,
              'detections':self.tracker.detected_scans,'target_truth_used_for_control':False,
              'policy_sha256':self.policy.sha256,'policy_training_updates':self.policy.updates,
              'policy_decisions':self.policy.decisions,'policy_unknown_decisions':self.policy.unknown_decisions}
        self.status.publish(String(data=json.dumps(data)))


class ObjectActor(SimNode):
    """Uses its OWN pose to follow a programmed trajectory; no robot feedback."""
    def __init__(self):
        super().__init__('independent_test_object')
        self.object=None;self.started=None;self.finished=False
        self.follower=RouteFollower(OBJECT_GOAL,.8)
        rng=random.Random(9000)
        self.scenario={'seed':9000,'base_speed':rng.uniform(.38,.62),'slow_speed':rng.uniform(.22,.38),
                       'brisk_speed':rng.uniform(.55,.70),'start_delay':rng.uniform(0,.8),
                       'stop_start':rng.uniform(4,9),'stop_duration':rng.uniform(1,4.5)}
        self.cmd=self.create_publisher(Twist,'/object/cmd_vel',1)
        self.create_subscription(Odometry,'/object/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Bool,'/range/start',self.start,10)
        self.create_subscription(Bool,'/range/done',self.done,10)
        self.previous=None;self.create_timer(.05,self.tick)

    def odometry(self,msg):self.object=self.pose(msg)
    def start(self,msg):
        if msg.data and self.started is None:self.started=self.now()
    def done(self,msg):self.finished=msg.data

    def tick(self):
        now=self.now();dt=.05 if self.previous is None else min(.1,max(0.,now-self.previous));self.previous=now
        if self.object is None or self.started is None or self.finished:
            self.velocity(self.cmd,(0.,0.));return
        t=now-self.started;s=self.scenario
        stopped=(t<s['start_delay'] or s['stop_start']<=t<s['stop_start']+s['stop_duration'])
        speed=0. if stopped else s['slow_speed'] if 10<=t<17 else s['brisk_speed'] if 17<=t<25 else s['base_speed']
        self.velocity(self.cmd,self.follower.command(self.object,speed,dt))


class Audit(SimNode):
    """Ground truth is only observed here, not sent back to the guide."""
    def __init__(self):
        super().__init__('range_audit_observer')
        OUT.mkdir(parents=True,exist_ok=True)
        self.path=OUT/'trace.jsonl'
        if self.path.exists():raise RuntimeError('Existing trace preserved; choose a new output directory')
        self.truth=None;self.robot_truth=None;self.truth_history=PoseHistory()
        self.rows=0;self.squared_error=[];self.speed_squared_error=[]
        self.min_distance=float('inf');self.max_gap=0.;self.lost=0;self.terminal=None
        self.result=None;self.post_min_distance=float('inf')
        self.create_subscription(Odometry,'/object/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/guide/odom',self.robot_odometry,qos_profile_sensor_data)
        self.create_subscription(String,'/range/status',self.status,10)

    def odometry(self,msg):
        self.truth=self.pose(msg)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.truth_history.add(stamp,self.truth)
    def robot_odometry(self,msg):self.robot_truth=self.pose(msg)

    def status(self,msg):
        d=json.loads(msg.data)
        if self.truth is not None:
            d['audit_object_truth']=self.truth.json()
            if d['elapsed']>0 and self.terminal is None:
                robot=Pose(**d['robot'])
                actual_distance=math.hypot(robot.x-self.truth.x,robot.y-self.truth.y)
                d['audit_true_distance']=actual_distance
                self.min_distance=min(self.min_distance,actual_distance)
                self.max_gap=max(self.max_gap,progress(robot)-progress(self.truth))
                if not d['observation_fresh']:self.lost+=1
                elif d['observation']:
                    o=d['observation'];reference=self.truth_history.at(o['stamp'])
                    if reference:
                        self.squared_error.append((o['x']-reference.x)**2+(o['y']-reference.y)**2)
                        self.speed_squared_error.append((o['speed']-reference.speed)**2)
        with self.path.open('a') as stream:stream.write(json.dumps(d)+'\n')
        self.rows+=1
        if d['state'] in ('ARRIVED','TOO_CLOSE','TOO_FAR','TIMEOUT','ODOM_LOST'):
            if self.terminal is None:
                self.terminal=d
                source_hash=hashlib.sha256((ROOT/'models/q_final.json').read_bytes()).hexdigest()
                self.result={'success':False,'controller_reported_arrived':d['state']=='ARRIVED',
                    'audit_goal_reached':False,'physical_arrival_sim_seconds':None,
                    'outcome':d['state'],'elapsed_sim_seconds':d['elapsed'],
                    'scenario_seed':9000,'scans_received':d['scans'],'scans_detected':d['detections'],
                    'control_target_truth':False,'control_inputs':['/guide/scan','/guide/odom'],
                    'ground_truth_role':'Independent actor and read-only audit only',
                    'policy_unchanged':source_hash==d['policy_sha256'],'policy_sha256':source_hash,
                    'additional_learning_updates':0,'rows':self.rows,'lost_control_ticks':self.lost,
                    'policy_decisions':d['policy_decisions'],'policy_unknown_decisions':d['policy_unknown_decisions'],
                    'min_true_center_distance':self.min_distance,'max_true_route_gap':self.max_gap,
                    'position_rmse_m':math.sqrt(sum(self.squared_error)/len(self.squared_error)) if self.squared_error else None,
                    'speed_rmse_mps':math.sqrt(sum(self.speed_squared_error)/len(self.speed_squared_error)) if self.speed_squared_error else None,
                    'final_robot':d['robot'],'final_object':self.truth.json() if self.truth else None}
            if self.truth is not None and self.robot_truth is not None:
                robot=self.robot_truth;target=self.truth
                distance=math.hypot(robot.x-target.x,robot.y-target.y)
                self.post_min_distance=min(self.post_min_distance,distance)
                reached=(math.hypot(robot.x-ROBOT_GOAL[0],robot.y-ROBOT_GOAL[1])<.15 and
                         math.hypot(target.x-OBJECT_GOAL[0],target.y-OBJECT_GOAL[1])<.15 and
                         abs(robot.speed)<.04 and abs(target.speed)<.04)
                if reached and not self.result['audit_goal_reached']:
                    self.result['audit_goal_reached']=True
                    self.result['physical_arrival_sim_seconds']=d['elapsed']
                self.result['success']=self.result['controller_reported_arrived'] and self.result['audit_goal_reached']
                self.result['post_terminal_min_distance']=self.post_min_distance
                self.result['post_terminal_last_robot']=robot.json()
                self.result['post_terminal_last_object']=target.json()
            temp=OUT/'result.tmp';temp.write_text(json.dumps(self.result,indent=2));temp.replace(OUT/'result.json')


def main():
    roles={'guide':Guide,'object':ObjectActor,'audit':Audit}
    rclpy.init();node=roles[sys.argv[1]]()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok() and hasattr(node,'cmd'):node.velocity(node.cmd,(0.,0.))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
