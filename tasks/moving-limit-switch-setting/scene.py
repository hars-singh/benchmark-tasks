"""Index the cam dog on a translating slide and prove its switch trip point."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_moving_limit_switch_setting.geometry import Solid
from dynamic_task_moving_limit_switch_setting.mechanisms import linear_stage
from dynamic_task_moving_limit_switch_setting.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRAVEL_M = .043
PERIOD_S = 9.
INDEX_M = .022
RELEASE_M = .013
TRIP_M = .0024
RESET_M = .0013
TRIP_WINDOW_M = (-.012, .008)
STAGE = Box('switch_test_carriage', pos=(.315, .025, TOP+.015), size=(.205, .247, .030),
            mass=3.1, color=(.16, .29, .44))
DOG = Box('indexed_cam_dog', pos=(.350, -.018, TOP+.117), size=(.130, .094, .035),
          mass=.095, color=(.77, .49, .19))
LOCK = Box('spring_index_pin', pos=(.344, .004, TOP+.092), size=(.148, .127, .030),
           mass=.044, color=(.71, .55, .25))
SWITCH = Box('roller_switch_enclosure', pos=(.468, .022, TOP+.034), size=(.058, .048, .068),
             mass=2.4, color=(.20, .31, .38))
ROLLER = Box('switch_roller_plunger', pos=(.444, .022, TOP+.094), size=(.020, .012, .037),
             mass=.019, color=(.59, .67, .70))
WHEEL = Box('switch_roller_wheel', pos=ROLLER.pos, size=(.018, .012, .012),
            mass=.007, color=(.64, .71, .74))
RELEASE_PAD = (.276, .091, TOP+.113)
PUSH_TAB = (.319, DOG.pos[1]-.055, TOP+.130)
RELEASE_LOCAL = tuple(a-b for a,b in zip(RELEASE_PAD, LOCK.pos, strict=True))
PUSH_LOCAL = tuple(a-b for a,b in zip(PUSH_TAB, DOG.pos, strict=True))

hardware, BODY_PARTS = linear_stage(STAGE, TRAVEL_M)
OBJECTS = [STAGE, DOG, LOCK, SWITCH, ROLLER, WHEEL, *hardware]


def positioned(body, size, center, color):
    return Solid('box', size, tuple(a-b for a,b in zip(center, body.pos, strict=True)), color=color)


BODY_PARTS[STAGE.name] += tuple(
    positioned(STAGE, size, center, color) for size,center,color in (
        ((.039, .145, .073), (.375, -.014, TOP+.0655), (.27, .36, .41)),
        ((.017, .138, .070), (.299, -.014, TOP+.064), (.27, .36, .41)),
        ((.012, .100, .003), (.297, -.018, TOP+.102), (.66, .71, .74)),
    )
)
BODY_PARTS[DOG.name] = (
    Solid('box', (.092, .090, .010)),
    *(Solid('box', (.021, .014, .012), (-.006, y, -.011), color=(.61, .66, .70))
      for y in (-.010, .011, .033)),
    Solid('box', (.030, .013, .030), PUSH_LOCAL, color=(.89, .62, .23)),
    Solid('box', (.101, .025, .009), (.055, 0., -.008)),
    Solid('box', (.025, .016, .008), (.094, 0., -.017), color=(.60, .65, .68)),
    *(Solid('box', (.025, .018, .007), (.094, side*.016, -.0135),
            roll=side*math.atan(.44), color=(.60, .65, .68)) for side in (-1, 1)),
)
BODY_PARTS[LOCK.name] = (
    Solid('box', (.011, .005, .029), (0., 0., .0025), color=(.68, .72, .76)),
    Solid('box', (.067, .011, .008), (-.0335, 0., -.008)),
    Solid('box', (.011, .091, .008), (-.068, .0435, -.008)),
    Solid('box', (.011, .011, .029), (-.068, .087, .007)),
    Solid('box', (.027, .025, .005), RELEASE_LOCAL, color=(.93, .70, .28)),
)
BODY_PARTS[SWITCH.name] = (
    Solid('box', SWITCH.size[:2]+(.016,), (0., 0., -.026)),
    Solid('box', (.038, .032, .054), (-.023, 0., .009)),
    Solid('box', (.023, .021, .008), (-.024, 0., .036), color=(.52, .57, .60)),
)
BODY_PARTS[ROLLER.name] = (
    Solid('box', (.003, .007, .023), (.0095, 0., -.012), color=(.49, .55, .61)),
    Solid('box', (.008, .007, .016), (.005, 0., -.021), color=(.49, .55, .61)),
)
BODY_PARTS[WHEEL.name] = (Solid('cylinder', (.006, .018), axis='X'),)
OBJECT_JOINTS = {
    DOG.name: {'kind':'prismatic', 'parent':STAGE.name, 'axis':'Y', 'anchor':DOG.pos,
               'lower':-.0005, 'upper':INDEX_M,
               'drive':{'stiffness':0., 'damping':2., 'max_force':1.5}},
    LOCK.name: {'kind':'prismatic', 'parent':STAGE.name, 'axis':'Z', 'anchor':LOCK.pos,
                'lower':-RELEASE_M, 'upper':0.,
                'drive':{'stiffness':300., 'damping':2., 'max_force':6.}},
    ROLLER.name: {'kind':'prismatic', 'parent':SWITCH.name, 'axis':'Z', 'anchor':ROLLER.pos,
                  'lower':-.0065, 'upper':0.,
                  'drive':{'stiffness':320., 'damping':1., 'max_force':3.}},
    WHEEL.name: {'kind':'revolute', 'parent':ROLLER.name, 'axis':'X', 'anchor':WHEEL.pos},
}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos, (0., TRAVEL_M, 0.), PERIOD_S)}
SUPPORT_JOINTS = {
    STAGE.name:{'axis':'Y', 'lower':-.056, 'upper':.056},
    SWITCH.name:{'kind':'fixed'},
}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage', 'amplitude':TRAVEL_M}}
CONTACT_PAIRS = {'roller_cam_load':(WHEEL.name, DOG.name)}
CONTACT_OFFSETS_M = {obj.name:.00015 for obj in (DOG, LOCK, ROLLER, WHEEL)}
RIGID_SOLVER_ITERATIONS = {obj.name:(96, 4) for obj in (STAGE, DOG, LOCK, SWITCH, ROLLER, WHEEL)}
SUCCESS_HOLD_S = .35
MAX_PARTIAL_SCORE = .65
LIVE_CONTEXT = {}
_released = _indexed = _closed = _armed = _verified = False
_trip_started = None


def on_reset():
    global _released, _indexed, _closed, _armed, _verified, _trip_started
    _released = _indexed = _closed = _armed = _verified = False
    _trip_started = None


def delta(origins, body, frame, axis):
    return origins[body.name][axis]-origins[frame.name][axis]-(body.pos[axis]-frame.pos[axis])


def observe(objects, context, dt):
    global _released, _indexed, _closed, _armed, _verified, _trip_started
    origins = context['body_origins']
    stage_y = origins[STAGE.name][1]-STAGE.pos[1]
    speed = objects[STAGE.name].lin_vel[1]
    release = -delta(origins, LOCK, STAGE, 2)
    setting = delta(origins, DOG, STAGE, 1)
    depth = -delta(origins, ROLLER, SWITCH, 2)
    release_pad = origins[LOCK.name]+RELEASE_LOCAL
    push_tab = origins[DOG.name]+PUSH_LOCAL
    left, right = context['tcp']['left'], context['tcp']['right']
    _released |= bool(abs(speed)>.011 and release>.0095 and math.dist(left, release_pad)<.032)
    _indexed |= bool(_released and abs(speed)>.010 and release>.0095 and setting>INDEX_M-.002
                     and math.dist(right, push_tab)<.048)
    hands_clear = all(math.dist(tcp, point)>.055 for tcp in (left, right)
                      for point in (push_tab, release_pad, origins[ROLLER.name]))
    retained = _indexed and release<.0022 and setting>INDEX_M-.003 and hands_clear
    load = -context['contact_forces']['roller_cam_load'][2]
    fixed_switch = math.dist(origins[SWITCH.name], SWITCH.pos)<.002
    before_window = stage_y<TRIP_WINDOW_M[0]
    _armed |= bool(retained and fixed_switch and before_window and depth<RESET_M)
    if not retained or not fixed_switch:
        _armed = _verified = False
    closed = depth>=TRIP_M if not _closed else depth>RESET_M
    if closed and not _closed:
        correct_point = TRIP_WINDOW_M[0]<=stage_y<=TRIP_WINDOW_M[1]
        _trip_started = stage_y if _armed and correct_point and speed>.009 and load>.22 else None
    if closed and _trip_started is not None:
        _verified |= bool(retained and fixed_switch and stage_y>=_trip_started+.006 and load>.22)
    if not closed:
        _trip_started = None
    _closed = closed


def success(objects):
    return _verified


def score(objects):
    return 1. if success(objects) else .65 if _indexed else .20 if _released else 0.


def demo():
    release_hover = (*RELEASE_PAD[:2], TOP+.164)
    release_down = (*RELEASE_PAD[:2], RELEASE_PAD[2]+.0025+PAD-.020)
    push_hover = (PUSH_TAB[0], PUSH_TAB[1]-.028, TOP+.166)
    push_ready = (*push_hover[:2], PUSH_TAB[2]+PAD-.010)
    push_end = (push_ready[0], PUSH_TAB[1]+INDEX_M+.007, push_ready[2])
    push_exit = (push_end[0]-.020, push_end[1], TOP+.156)
    left_clear = (.246, .194, TOP+.169)
    right_clear = (.218, -.148, TOP+.169)
    return {
        'left':[
            Waypoint(2.25, release_hover, CLOSED, 'approach the release paddle'),
            Waypoint(1.20, release_down, CLOSED, 'depress the index pin'),
            Waypoint(4.10, release_down, CLOSED, 'hold the index pin clear'),
            Waypoint(.80, release_hover, CLOSED, 'reengage the index pin'),
            Waypoint(1.20, left_clear, OPEN, 'move clear of the carriage'),
            Waypoint(12., left_clear, OPEN, 'observe the carried cam trip the switch'),
        ],
        'right':[
            Waypoint(2.15, push_hover, CLOSED, 'approach the cam dog'),
            Waypoint(1.60, push_ready, CLOSED, 'lower behind the adjustment tab'),
            Waypoint(2.20, push_end, CLOSED, 'advance the dog to the early detent'),
            Waypoint(2.45, push_end, CLOSED, 'hold the new setting'),
            Waypoint(.70, push_exit, CLOSED, 'lift off the adjustment tab'),
            Waypoint(1.15, right_clear, OPEN, 'move clear of the carriage'),
            Waypoint(12., right_clear, OPEN, 'observe the carried cam trip the switch'),
        ],
    }


def demo_target(arm, label, t, pos, rotation, finger):
    if not LIVE_CONTEXT or label in {'move clear of the carriage', 'observe the carried cam trip the switch', 'done'}:
        return pos, rotation, finger
    shift = LIVE_CONTEXT['body_origins'][STAGE.name]-np.asarray(STAGE.pos)
    shift[1] += .078*LIVE_OBJECTS[STAGE.name].lin_vel[1]
    return pos+shift, rotation, finger
