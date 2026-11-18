"""Carry a long flexible profile from translating supports without bending it."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_moving_flexible_profile_transfer.geometry import Solid
from dynamic_task_moving_flexible_profile_transfer.mechanisms import linear_stage
from dynamic_task_moving_flexible_profile_transfer.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
AMPLITUDE = .027
PERIOD = 7.5
PROFILE_X, RECEIVER_X = .269, .386
PROFILE_Z = TOP+.092
SEGMENT_Y = .123
GRIP_Y = .142
PROFILE_HEIGHT = .007
FLEX_LIMIT = math.radians(8.)
SHAPE = (.020, .113, PROFILE_HEIGHT)
STAGE = Box('profile_carriage', pos=(PROFILE_X, 0., TOP+.015), size=(.094, .382, .030),
            mass=3.7, color=(.18, .29, .42))
CENTER = Box('profile_center', pos=(PROFILE_X, 0., PROFILE_Z), size=(.020, .131, PROFILE_HEIGHT),
             mass=.018, color=(.39, .55, .62))
OUTER = {
    'left':Box('profile_positive_end', pos=(PROFILE_X, SEGMENT_Y, PROFILE_Z), size=SHAPE,
               mass=.012, color=(.38, .54, .61)),
    'right':Box('profile_negative_end', pos=(PROFILE_X, -SEGMENT_Y, PROFILE_Z), size=SHAPE,
                mass=.012, color=(.38, .54, .61)),
}
STAND = Box('receiving_saddles', pos=(RECEIVER_X, 0., TOP+.034), size=(.067, .322, .068),
            mass=2.0, color=(.27, .35, .39))
MIDDLE = Box('spring_middle_saddle', pos=(RECEIVER_X,0.,PROFILE_Z-PROFILE_HEIGHT/2+.00045-.0035),
             size=(.045,.024,.007), mass=.014, color=(.56,.61,.64))
SOURCE_SUPPORTS = (-.086, 0., .086)
RECEIVE_SUPPORTS = (-.095, 0., .095)
SIDE = {'left':1., 'right':-1.}

hardware, BODY_PARTS = linear_stage(STAGE, AMPLITUDE)
OBJECTS = [STAGE, CENTER, *OUTER.values(), STAND, MIDDLE, *hardware]
for body in (CENTER, *OUTER.values()):
    BODY_PARTS[body.name] = (
        Solid('box', (.020, body.size[1], .0040), (0., 0., -.0015)),
        *(Solid('box', (.004, body.size[1], .003), (sign*.008, 0., .002), color=(.32, .48, .56))
          for sign in (-1, 1)),
    )


def seats(body, y_positions, rail_top):
    for y in y_positions:
        above = PROFILE_Z-PROFILE_HEIGHT/2
        x_rel, y_rel = 0., y-body.pos[1]
        yield Solid('box', (.045, .024, above-rail_top),
                    (x_rel,y_rel,(above+rail_top)/2-body.pos[2]), color=(.24,.33,.40))
        for side in (-1,1):
            yield Solid('box', (.006,.023,.010),
                        (side*.020,y_rel,above+.004-body.pos[2]), color=(.67,.57,.36))


BODY_PARTS[STAGE.name] += tuple(seats(STAGE, SOURCE_SUPPORTS, TOP+.030))
BODY_PARTS[STAND.name] = (
    Solid('box', (.067,.322,.012), (0.,0.,TOP+.006-STAND.pos[2])),
    *seats(STAND, (y for y in RECEIVE_SUPPORTS if y), TOP+.012),
    Solid('box', (.045,.024,.062),(0.,0.,TOP+.043-STAND.pos[2]),color=(.24,.33,.40)),
)
OBJECT_JOINTS = {
    body.name:{'kind':'revolute','parent':CENTER.name,'axis':'X',
               'anchor':(PROFILE_X,SIDE[arm]*.066,PROFILE_Z),
               'lower':-30.,'upper':30.,
               'drive':{'stiffness':.18,'damping':.013,'max_force':.10}}
    for arm,body in OUTER.items()
}
OBJECT_JOINTS[MIDDLE.name] = {'kind':'prismatic','parent':STAND.name,'axis':'Z','anchor':MIDDLE.pos,
                             'lower':-.002,'upper':0.,
                             'drive':{'stiffness':750.,'damping':2.0,'max_force':3.}}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,AMPLITUDE,0.),PERIOD)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.039,'upper':.039},STAND.name:{'kind':'fixed'}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':AMPLITUDE}}
PAYLOAD_SUPPORTS = {
    body.name:dict(body=STAGE.name,top_local_z=PROFILE_Z-PROFILE_HEIGHT/2-STAGE.pos[2],
                   penetration_tolerance_m=.0015)
    for body in (CENTER,*OUTER.values())
}
CONTACT_OFFSETS_M = {body.name:.00025 for body in (CENTER,*OUTER.values(),STAGE,STAND,MIDDLE)}
RIGID_SOLVER_ITERATIONS = {body.name:(96,4) for body in (CENTER,*OUTER.values(),STAGE,STAND,MIDDLE)}
CONTACT_PAIRS = {f'{arm}_saddle':(body.name,STAND.name) for arm,body in OUTER.items()}
CONTACT_PAIRS['center_saddle'] = (CENTER.name,MIDDLE.name)
SUCCESS_HOLD_S = .60
MAX_PARTIAL_SCORE = .55
LIVE_CONTEXT = {}
_acquired = _lifted = _failure = _placed = False


def on_reset():
    global _acquired, _lifted, _failure, _placed
    _acquired = _lifted = _failure = _placed = False


def hand_points(origins, frames):
    return {arm:origins[body.name]+frames[body.name]@np.array((0.,SIDE[arm]*(GRIP_Y-SEGMENT_Y),0.))
            for arm,body in OUTER.items()}


def observe(objects, context, dt):
    global _acquired, _lifted, _failure, _placed
    origins = context['body_origins']
    segments = (CENTER,*OUTER.values())
    frames = {body.name:quat_xyzw_to_matrix(np.asarray(objects[body.name].quat_xyzw))
              for body in segments}
    ends = hand_points(origins,frames)
    bending = max(math.acos(np.clip(frames[CENTER.name][:,1]@frames[body.name][:,1],-1.,1.))
                  for body in OUTER.values())
    grasping = all(math.dist(context['tcp'][arm],ends[arm])<.037 and context['gripper'][arm]<.75
                   for arm in OUTER)
    _acquired |= bool(grasping and abs(objects[STAGE.name].lin_vel[1])>.012
                      and abs(origins[CENTER.name][0]-PROFILE_X)<.018)
    _lifted |= bool(_acquired and grasping and all(origins[body.name][2]>PROFILE_Z+.024 for body in segments))
    _failure |= bool(bending>FLEX_LIMIT or any(objects[body.name].low[2]<TOP+.030 for body in segments))
    hands_clear = all(math.dist(tcp,end)>.060 for tcp in context['tcp'].values() for end in ends.values())
    aligned = all(abs(origins[body.name][0]-RECEIVER_X)<.009
                  and abs(origins[body.name][1]-body.pos[1])<.013
                  and math.hypot(*objects[body.name].lin_vel)<.022 for body in segments)
    supported = all(context['contact_forces'][f'{name}_saddle'][2]>.026 for name in ('left','center','right'))
    _placed = bool(_lifted and not _failure and hands_clear and aligned and supported
                   and bending<math.radians(2.5))


def success(objects):
    return _placed


def score(objects):
    if _failure:
        return 0.
    return 1. if _placed else .55 if _lifted else .20 if _acquired else 0.


def demo():
    result = {}
    pick_z = PROFILE_Z+PAD-.0035
    for arm,sign in SIDE.items():
        grip_y = sign*GRIP_Y
        yaw = -sign*math.pi/2
        ready = (PROFILE_X,grip_y,pick_z+.060)
        close = (PROFILE_X,grip_y,pick_z)
        overhead = (RECEIVER_X,grip_y,pick_z+.052)
        seat = (RECEIVER_X,grip_y,pick_z-.002)
        clear = (.260,sign*.220,pick_z+.070)
        result[arm] = [
            Waypoint(1.95,ready,OPEN,'track the moving profile',yaw),
            Waypoint(.95,close,OPEN,'approach both ends',yaw),
            Waypoint(.75,close,CLOSED,'grasp both ends',yaw),
            Waypoint(.85,(PROFILE_X,grip_y,pick_z+.052),CLOSED,'lift the profile level',yaw),
            Waypoint(1.60,overhead,CLOSED,'transfer horizontally',yaw),
            Waypoint(1.05,seat,CLOSED,'lower into the receiving saddles',yaw),
            Waypoint(.50,seat,OPEN,'release both ends',yaw),
            Waypoint(.60,overhead,OPEN,'lift clear of the profile',yaw),
            Waypoint(1.25,clear,OPEN,'clear the receiving saddles',yaw),
            Waypoint(2.00,clear,OPEN,'observe the supported profile',yaw),
        ]
    return result


def demo_target(arm,label,t,pos,rotation,finger):
    if not LIVE_CONTEXT:
        return pos,rotation,finger
    if label in {'track the moving profile','approach both ends','grasp both ends','lift the profile level'}:
        origins = LIVE_CONTEXT['body_origins']
        carried_y = float(origins[CENTER.name][1])
        lead = .066*LIVE_OBJECTS[CENTER.name].lin_vel[1]
        return pos+np.array((0.,carried_y+lead,0.)),rotation,finger
    if label=='transfer horizontally':
        fraction = (RECEIVER_X-pos[0])/(RECEIVER_X-PROFILE_X)
        y = float(LIVE_CONTEXT['body_origins'][CENTER.name][1])
        return pos+np.array((0.,fraction*y,0.)),rotation,finger
    return pos,rotation,finger
