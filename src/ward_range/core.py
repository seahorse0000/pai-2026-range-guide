"""Range-only target estimation and frozen speed policy, independent of ROS.

The controlled test has ONE cylindrical target on a known straight route.
This is geometric detection, not semantic person recognition or general tracking.
Target ground-truth position/velocity is not an input to these classes.
"""
import bisect
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import random
import numpy as np
from .base_core import Pose, steer, clamp

ACTIONS = (0., .2, .4, .6, .8)
LASER_X = -.28
ROBOT_GOAL = (10., 0.)
OBJECT_GOAL = (8.5, 0.)


def progress(pose):
    return pose.x


def path_error(pose):
    return abs(pose.y)


def limited_speed(policy_speed, object_speed, gap):
    """Cap the learned action so the guide waits for a stopped/lagging object.

    This explicit distance rule is separate from the frozen Q policy.
    All inputs are estimated from LaserScan and the guide's own odometry.
    """
    upper = max(0., object_speed + .8*(1.5-gap))
    return min(policy_speed, upper)


class PoseHistory:
    def __init__(self):self.samples=deque(maxlen=600)

    def add(self, stamp, pose):
        if not self.samples or stamp>self.samples[-1][0]:self.samples.append((stamp,pose))

    def at(self, stamp):
        if not self.samples:return None
        times=[t for t,_ in self.samples]
        index=bisect.bisect_left(times,stamp)
        if 0<index<len(times):
            t0,p0=self.samples[index-1];t1,p1=self.samples[index]
            fraction=(stamp-t0)/(t1-t0)
            yaw_delta=math.atan2(math.sin(p1.yaw-p0.yaw),math.cos(p1.yaw-p0.yaw))
            return Pose(p0.x+fraction*(p1.x-p0.x),p0.y+fraction*(p1.y-p0.y),
                        p0.yaw+fraction*yaw_delta,p0.speed+fraction*(p1.speed-p0.speed))
        time,pose=self.samples[0 if index==0 else -1]
        dt=stamp-time
        if abs(dt)>.08:return None
        return Pose(pose.x+pose.speed*math.cos(pose.yaw)*dt,
                    pose.y+pose.speed*math.sin(pose.yaw)*dt,pose.yaw,pose.speed)


@dataclass
class Observation:
    stamp: float
    x: float
    y: float
    speed: float
    radius: float
    points: int
    residual: float

    def pose(self):
        return Pose(self.x, self.y, speed=self.speed)


def fit_circle(points):
    """Fit circle to returns, rejecting straight walls and badly fitted arcs."""
    if len(points) < 6:
        return None
    xy = np.asarray(points, dtype=float)
    # Center the coordinates to keep the least-squares system well conditioned.
    origin = xy.mean(axis=0)
    local = xy-origin
    design = np.column_stack((2*local[:, 0], 2*local[:, 1], np.ones(len(xy))))
    coefficients, _, rank, _ = np.linalg.lstsq(design, (local*local).sum(axis=1), rcond=None)
    if rank < 3:
        return None
    center = coefficients[:2]+origin
    radius_squared = coefficients[2]+float(coefficients[:2]@coefficients[:2])
    if radius_squared <= 0:
        return None
    radius = math.sqrt(radius_squared)
    error = float(np.sqrt(np.mean((np.linalg.norm(xy-center, axis=1)-radius)**2)))
    if not .12 <= radius <= .26 or error > .018:
        return None
    return float(center[0]), float(center[1]), radius, error


class RangeTracker:
    def __init__(self):
        self.observation = None
        self.history = deque(maxlen=7)
        self.received_scans = 0
        self.detected_scans = 0

    def update(self, stamp, robot, angle_min, angle_step, ranges, range_min=.08, range_max=8.):
        self.received_scans += 1
        if self.observation and stamp <= self.observation.stamp:
            return self.observation
        cosine, sine = math.cos(robot.yaw), math.sin(robot.yaw)
        origin_x, origin_y = robot.x+LASER_X*cosine, robot.y+LASER_X*sine
        clusters, group = [], []
        previous = None
        for index, distance in enumerate(ranges):
            point = None
            if math.isfinite(distance) and max(.4, range_min) < distance < range_max:
                angle = robot.yaw+angle_min+index*angle_step
                x, y = origin_x+distance*math.cos(angle), origin_y+distance*math.sin(angle)
                # The route is known. Walls are outside its narrow center band.
                if path_error(Pose(x, y)) < .5:
                    point = (x, y)
            if point is None or (previous and math.dist(previous, point) > .13):
                if group:
                    clusters.append(group)
                group = []
            if point is not None:
                group.append(point)
            previous = point
        if group:
            clusters.append(group)
        if len(clusters)>1 and math.dist(clusters[0][0], clusters[-1][-1]) < .13:
            clusters[0] = clusters[-1]+clusters[0]
            clusters.pop()
        candidates = []
        for points in clusters:
            fitted = fit_circle(points)
            if fitted is None:
                continue
            x, y, radius, error = fitted
            pose = Pose(x, y)
            if path_error(pose) > .35 or progress(pose) > progress(robot)+.35:
                continue
            if self.observation and stamp-self.observation.stamp < .5:
                separation = math.hypot(x-self.observation.x, y-self.observation.y)
                if separation > .6:
                    continue
            else:
                separation = math.hypot(x-robot.x, y-robot.y)
            candidates.append((separation, x, y, radius, len(points), error))
        if not candidates:
            return None
        _, x, y, radius, count, error = min(candidates)
        if not self.observation or stamp-self.observation.stamp > .5:
            self.history.clear()
        self.history.append((stamp, progress(Pose(x, y))))
        speed = 0.
        if len(self.history) >= 3:
            times = np.array([t for t, _ in self.history])
            positions = np.array([p for _, p in self.history])
            times -= times.mean()
            denominator = float(times@times)
            if denominator > 1e-8:
                speed = clamp(float(times@(positions-positions.mean()))/denominator, 0., .9)
        self.observation = Observation(stamp, x, y, speed, radius, count, error)
        self.detected_scans += 1
        return self.observation

    def fresh(self, now, maximum_age=.3):
        return self.observation is not None and 0 <= now-self.observation.stamp <= maximum_age


class RouteFollower:
    def __init__(self, goal, acceleration=.35):
        self.goal = goal
        self.acceleration = acceleration
        self.phase = 0
        self.previous_v = 0.

    def command(self, pose, target_speed, dt):
        target = self.goal
        if math.hypot(pose.x-target[0], pose.y-target[1]) < .12:
            self.phase = 2
        v, w = (0., 0.) if self.phase == 2 else steer(pose, target, target_speed)
        self.previous_v = clamp(v, self.previous_v-.5*dt, self.previous_v+self.acceleration*dt)
        if self.phase == 2 and self.previous_v < .005:
            self.previous_v = 0.
        return self.previous_v, w


class FrozenPolicy:
    def __init__(self, path, seed=59000):
        data = Path(path).read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        model = json.loads(data)
        assert tuple(model['actions_mps']) == ACTIONS
        self.q = model['q']
        self.updates = model['updates']
        self.rng = random.Random(seed)
        self.decisions=0
        self.unknown_decisions=0

    @staticmethod
    def state(robot, target, phase):
        gap = progress(robot)-progress(target.pose())
        closing = target.speed-robot.speed
        return (bisect.bisect_right([.9,1.2,1.6,2.,2.5,3.],gap),
                bisect.bisect_right([.08,.48],target.speed),
                bisect.bisect_right([-.12,.12],closing),
                3 if phase==2 else 0)

    def act(self, state):
        key=','.join(map(str,state));self.decisions+=1
        if key not in self.q:self.unknown_decisions+=1
        values = self.q.get(key, [0.]*5)
        self.rng.random()  # Preserve the original epsilon=0 evaluation RNG sequence.
        best = max(values)
        return self.rng.choice([i for i,v in enumerate(values) if abs(v-best)<1e-12])
