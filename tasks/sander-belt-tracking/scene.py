"""Center a drifting abrasive belt using two start-stop checks and its tracking knob."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_sander_belt_tracking.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
X, Y = .401, .017
HALF_STRAIGHT, DRUM_R, BELT_WIDTH = .1765, .0331, .1016
BELT_Z = TOP + .191
BELT_LENGTH = 4. * HALF_STRAIGHT + math.tau * DRUM_R
RESET_LATERAL = .0015
STEERING_LEAD = .00125
INITIAL_SKEW = .000157
MOTOR_DEG_S = 1420.
DARK = (.18, .21, .22)
IRON = (.45, .50, .52)
ABRASIVE = (.51, .30, .19)
SEAM = (.69, .46, .34)
RED = (.70, .11, .084)
GREEN = (.23, .58, .35)

FRAME = Box('wen_6502t_sander_and_stock_guards', pos=(X, Y, TOP + .076),
            size=(.381, .455, .152), mass=17.9, color=DARK)
DRIVE = Box('powered_sanding_drum', pos=(X, Y - HALF_STRAIGHT, BELT_Z),
            size=(.125, 2. * DRUM_R, 2. * DRUM_R), mass=.35, color=IRON)
IDLER = Box('tracking_idler_drum', pos=(X, Y + HALF_STRAIGHT, BELT_Z),
            size=DRIVE.size, mass=.24, color=IRON)
BELT = Box('four_inch_by_thirty_six_inch_abrasive_loop', pos=(X + RESET_LATERAL, Y, BELT_Z),
           size=(BELT_WIDTH, 2. * (HALF_STRAIGHT + DRUM_R), 2. * DRUM_R), mass=.070, color=ABRASIVE)
KNOB = Box('stock_belt_tracking_knob', pos=(X - .082, Y + HALF_STRAIGHT - .021, BELT_Z - .011),
           size=(.019, .035, .035), mass=.024, color=(.14, .15, .15))
SHOE = Box('idler_tracking_screw_carriage', pos=(X - .076, Y + HALF_STRAIGHT, BELT_Z - .059),
           size=(.012, .024, .020), mass=.041, color=IRON)
SWITCH = Box('stock_keyed_on_off_rocker', pos=(X - .166, Y - .113, TOP + .102),
             size=(.010, .036, .043), mass=.012, color=(.23, .26, .26))
OBJECTS = [FRAME, DRIVE, IDLER, BELT, KNOB, SHOE, SWITCH]


def loop_pose(distance):
    """Return Y, Z and X-axis tilt on the closed belt path."""
    distance %= BELT_LENGTH
    straight = 2. * HALF_STRAIGHT
    curve = math.pi * DRUM_R
    if distance < straight:
        return -HALF_STRAIGHT + distance, DRUM_R, 0.
    distance -= straight
    if distance < curve:
        angle = distance / DRUM_R
        return HALF_STRAIGHT + DRUM_R * math.sin(angle), DRUM_R * math.cos(angle), -angle
    distance -= curve
    if distance < straight:
        return HALF_STRAIGHT - distance, -DRUM_R, -math.pi
    angle = (distance - straight) / DRUM_R
    return -HALF_STRAIGHT - DRUM_R * math.sin(angle), -DRUM_R * math.cos(angle), -math.pi - angle


def local(size, point, color):
    return Solid('box', size, tuple(float(a - b) for a, b in zip(point, FRAME.pos)), color=color)


def section(distance, depth, color):
    y, z, tilt = loop_pose(distance)
    return Solid('box', (BELT_WIDTH, depth, .00155), (0., y, z), roll=tilt, color=color)


BODY_PARTS = {
    FRAME.name: (
        Solid('box', (.372, .390, .025), (0., -.015, -.063), color=DARK),
        Solid('box', (.278, .243, .120), (.007, -.036, -.013), color=IRON),
        local((.101, .337, .018), (X, Y, BELT_Z + .008), IRON),
        *(local((.013, .357, .073), (X + sign * .069, Y, BELT_Z - .018), DARK)
          for sign in (-1., 1.)),
        local((.028, .157, .084), (X - .135, Y - .124, TOP + .095), DARK),
        Solid('cylinder', (.0762, .018), (-.127, -.174, -.000), axis='X', color=DARK),
        Solid('cylinder', (.060, .008), (-.136, -.174, 0.), axis='X', color=(.36, .29, .25)),
        local((.086, .134, .010), (X - .165, Y - .175, TOP + .066), IRON),
        local((.151, .016, .034), (X, Y - HALF_STRAIGHT - .021, BELT_Z + .036), DARK),
        local((.012, .048, .052), (X - .155, SWITCH.pos[1], SWITCH.pos[2]), DARK),
    ),
    DRIVE.name: (Solid('cylinder', (DRUM_R - .0026, .124), axis='X', color=IRON),),
    IDLER.name: (Solid('cylinder', (DRUM_R - .0026, .124), axis='X', color=IRON),),
    BELT.name: tuple(section((i + .5) * BELT_LENGTH / 70., BELT_LENGTH / 70. + .00025, ABRASIVE)
                     for i in range(70)),
    KNOB.name: (
        Solid('cylinder', (.015, .020), axis='X', color=(.12, .14, .15)),
        *(Solid('box', (.017, .007, .007),
                (0., .014 * math.sin(a), .014 * math.cos(a)), roll=-a, color=(.20, .22, .23))
          for a in np.linspace(0., math.tau, 6, endpoint=False)),
    ),
    SWITCH.name: (
        Solid('box', (.008, .034, .039), color=(.19, .22, .22)),
        Solid('box', (.008, .028, .014), (-.005, 0., .010), color=GREEN),
        Solid('box', (.008, .028, .015), (-.005, 0., -.010), color=RED),
        Solid('box', (.007, .009, .011), (-.0095, 0., 0.), color=(.88, .64, .13)),
    ),
}
SUPPORT_JOINTS = {FRAME.name: {'kind': 'fixed'}}
OBJECT_JOINTS = {
    DRIVE.name: {'kind': 'revolute', 'parent': FRAME.name, 'axis': 'X', 'anchor': DRIVE.pos,
                 'drive': {'damping': .18, 'max_force': .65}},
    IDLER.name: {'kind': 'revolute', 'parent': FRAME.name, 'axis': 'X', 'anchor': IDLER.pos,
                 'drive': {'damping': .13, 'max_force': .48}},
    BELT.name: {'kind': 'prismatic', 'parent': FRAME.name, 'axis': 'X', 'anchor': BELT.pos,
                'lower': -.015, 'upper': .012,
                'drive': {'stiffness': 29., 'damping': 3.2, 'max_force': 2.6}},
    SHOE.name: {'kind': 'prismatic', 'parent': FRAME.name, 'axis': 'Y', 'anchor': SHOE.pos,
                'lower': -.00055, 'upper': .00065, 'drive': {'damping': 12., 'max_force': 3.}},
    KNOB.name: {'kind': 'revolute', 'parent': FRAME.name, 'axis': 'X', 'anchor': KNOB.pos,
                'lower': -100., 'upper': 170., 'drive': {'damping': .006, 'max_force': .038},
                'screw_coupling': {'carriage': SHOE.name, 'travel_per_turn_m': STEERING_LEAD}},
    SWITCH.name: {'kind': 'revolute', 'parent': FRAME.name, 'axis': 'Y', 'anchor': SWITCH.pos,
                  'lower': -15., 'upper': 15.,
                  'drive': {'stiffness': .051, 'damping': .010, 'max_force': .044}},
}
SURFACE_FRICTION = {KNOB.name: (.95, .76), SWITCH.name: (.69, .52)}
CONTACT_OFFSETS_M = {body.name: .00045 for body in (FRAME, BELT, KNOB, SWITCH, DRIVE, IDLER)}
OBJECT_MATERIALS = {
    FRAME.name: 'WEN 6502T cast-iron body with factory bed, stop and disc guard',
    KNOB.name: 'factory knurled tracking screw',
    SHOE.name: 'native idler screw travel and shoe',
    BELT.name: 'full-size 4 x 36 in abrasive belt',
    SWITCH.name: 'manufacturer keyed rocker with its normal key in place',
    DRIVE.name: 'native drive drum', IDLER.name: 'native idler drum',
}

camera = np.array((-.262, -.066, .365))
forward = np.asarray((-.027, .008, .098)) - camera
forward /= np.linalg.norm(forward)
side = np.cross(forward, (0., 0., 1.))
side /= np.linalg.norm(side)
camera_q = matrix_to_quat_xyzw(np.column_stack((side, np.cross(side, forward), -forward)))
TASK_RECORD_CAMERA = (FRAME.name, Camera(tuple(camera), tuple(camera_q), 'opengl',
                                       Lens.from_focal(20., 640, 360)))

LIVE_CONTEXT = {}
SUCCESS_HOLD_S = 1.
MAX_PARTIAL_SCORE = .5
_powered = False
_cycles = 0
_first_drift = 0.
_first_start = None
_stable_s = 0.
_verified_second = False
_unsafe = False
_running = False
_last_knob = None
_working_adjustment = 0.
_lateral_target = 0.
_loop_distance = 0.
_rocker_direction = 0
_rocker_time = 0.
_drives = None
_seam = None


def on_reset():
    global _powered, _cycles, _first_drift, _first_start, _stable_s, _verified_second, _unsafe, _running
    global _last_knob, _working_adjustment, _lateral_target, _loop_distance, _rocker_direction, _rocker_time
    _powered = _verified_second = _unsafe = _running = False
    _cycles = 0
    _first_drift = _stable_s = _working_adjustment = _lateral_target = _loop_distance = 0.
    _last_knob = None
    _first_start = None
    _rocker_direction = 0
    _rocker_time = 0.


def observe(objects, context, dt):
    global _powered, _cycles, _first_drift, _first_start, _stable_s, _verified_second, _unsafe, _running
    global _last_knob, _working_adjustment, _lateral_target, _loop_distance, _rocker_direction, _rocker_time
    pos = context['body_origins']
    rocker = quat_xyzw_to_matrix(objects[SWITCH.name].quat_xyzw)
    was_powered = _powered
    deflected = float(rocker[0, 2])
    direction = 1 if deflected > .130 else -1 if deflected < -.130 else 0
    _rocker_time = _rocker_time + dt if direction == _rocker_direction else dt
    _rocker_direction = direction
    if direction and _rocker_time >= .13:
        _powered = direction > 0
    _cycles += int(_powered and not was_powered)
    rotor_speed = float(objects[DRIVE.name].ang_vel[0])
    _running = abs(rotor_speed) > .50
    _loop_distance += abs(rotor_speed) * DRUM_R * dt
    wrapped_knob = quat_xyzw_to_matrix(objects[KNOB.name].quat_xyzw)
    knob_angle = math.atan2(wrapped_knob[2, 1], wrapped_knob[1, 1])
    knob_change = 0. if _last_knob is None else knob_angle - _last_knob
    _last_knob = knob_angle
    _working_adjustment = pos[SHOE.name][1] - SHOE.pos[1]
    lateral = pos[BELT.name][0] - X
    if _running:
        destination = float(np.clip(52. * (INITIAL_SKEW - _working_adjustment), -.012, .012))
        current_command = _lateral_target + RESET_LATERAL
        _lateral_target += float(np.clip(destination - current_command, -.011 * dt, .011 * dt))
        if _cycles == 1:
            _first_start = lateral if _first_start is None else _first_start
            _first_drift = max(_first_drift, abs(lateral - _first_start))
        if _cycles >= 2 and abs(lateral) <= .0025 and abs(objects[BELT.name].lin_vel[0]) < .0014:
            _stable_s += dt
            _verified_second |= _stable_s + 1.e-9 >= 1.5
        else:
            _stable_s = 0.
        _unsafe |= abs(knob_change) > .016
        for tip in context['tcp'].values():
            in_belt_x = abs(tip[0] - pos[BELT.name][0]) < BELT_WIDTH / 2. + .010
            in_belt_y = abs(tip[1] - Y) < HALF_STRAIGHT + DRUM_R + .009
            _unsafe |= bool(in_belt_x and in_belt_y and abs(tip[2] - BELT_Z) < DRUM_R + .017)
    _unsafe |= abs(lateral) > .0117


def update_actuators():
    global _drives
    if _drives is None:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        joints = {p.GetName(): p for p in stage.Traverse() if p.GetName().endswith('_joint')}
        settings = ((DRIVE, 'angular'), (IDLER, 'angular'), (BELT, 'linear'), (SWITCH, 'angular'))
        _drives = [UsdPhysics.DriveAPI(joints[b.name + '_joint'], axis) for b, axis in settings]
    speed = -MOTOR_DEG_S if _powered else 0.
    for drive in _drives[:2]:
        drive.GetTargetVelocityAttr().Set(speed)
    _drives[2].GetTargetPositionAttr().Set(_lateral_target)
    _drives[3].GetTargetPositionAttr().Set(10. if _powered else -10.)


def update_visuals():
    global _seam
    from pxr import Gf, UsdGeom

    if _seam is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        belt = next(prim for prim in stage.Traverse() if prim.GetName() == BELT.name)
        witness = UsdGeom.Cube.Define(stage, str(belt.GetPath()) + '/factory_splice')
        witness.CreateSizeAttr(1.)
        shift = witness.AddTranslateOp()
        roll = witness.AddRotateXOp()
        witness.AddScaleOp().Set(Gf.Vec3d(.1005, .0090, .00030))
        witness.CreateDisplayColorAttr([Gf.Vec3f(*SEAM)])
        _seam = shift, roll
    y, z, tilt = loop_pose(_loop_distance)
    offset_y = -.0010 * math.sin(tilt)
    offset_z = .0010 * math.cos(tilt)
    _seam[0].Set(Gf.Vec3d(0., y + offset_y, z + offset_z))
    _seam[1].Set(math.degrees(tilt))


def success(objects):
    belt = LIVE_CONTEXT['body_origins'][BELT.name]
    safe_hands = all(math.dist((X-.060, Y, BELT_Z), tcp) > .14 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(not _unsafe and not _powered and not _running and _cycles >= 2
                and _first_drift >= .0035 and _verified_second and abs(belt[0] - X) <= .0025 and safe_hands)


def score(objects):
    if _unsafe:
        return 0.
    return 1. if success(objects) else .5 if _verified_second and _first_drift >= .0035 else 0.


ON = (SWITCH.pos[0] + .008, SWITCH.pos[1], SWITCH.pos[2] + .011)
OFF = (SWITCH.pos[0] + .014, SWITCH.pos[1], SWITCH.pos[2] - .019)
NEAR = (SWITCH.pos[0] - .046, SWITCH.pos[1], SWITCH.pos[2] + .027)
KNOB_GRIP = (KNOB.pos[0] - .014, KNOB.pos[1], KNOB.pos[2] + .004)
KNOB_HOVER = (KNOB_GRIP[0] - .073, KNOB_GRIP[1], KNOB_GRIP[2] + .033)
PARK_RIGHT = (.177, -.285, TOP + .188)
PARK_LEFT = (.188, .273, TOP + .191)


def demo():
    return {
        'right': [
            Waypoint(1.3, NEAR, -.012, 'approach the keyed switch'),
            Waypoint(.9, ON, -.012, 'press green to check the original tracking'),
            Waypoint(.62, NEAR, -.012, 'clear the control while the band runs'),
            Waypoint(1.28, NEAR, -.012, 'observe the abrasive belt drift'),
            Waypoint(.9, OFF, -.012, 'press red and wait for the drums to stop'),
            Waypoint(1.0, PARK_RIGHT, FINGER_OPEN, 'clear the tracking adjustment'),
            Waypoint(6.7, PARK_RIGHT, FINGER_OPEN, 'wait for the external knob adjustment'),
            Waypoint(1.0, NEAR, -.012, 'return to the keyed switch'),
            Waypoint(.92, ON, -.012, 'press green to check the new tracking'),
            Waypoint(.64, NEAR, -.012, 'withdraw to watch the band'),
            Waypoint(4.6, NEAR, -.012, 'verify running belt centering'),
            Waypoint(.94, OFF, -.012, 'switch the centered sander off'),
            Waypoint(.61, (OFF[0] - .057, OFF[1], OFF[2]), -.012, 'withdraw straight from the red rocker'),
            Waypoint(.92, PARK_RIGHT, FINGER_OPEN, 'withdraw after the test'),
            Waypoint(3.5, PARK_RIGHT, FINGER_OPEN, 'verify the stopped belt and clear grippers'),
        ],
        'left': [
            Waypoint(1.20, PARK_LEFT, FINGER_OPEN, 'stay clear of the first tracking check'),
            Waypoint(5.26, PARK_LEFT, FINGER_OPEN, 'wait for the drums to stop'),
            Waypoint(1.25, KNOB_HOVER, FINGER_OPEN, 'approach the original tracking knob'),
            Waypoint(.96, KNOB_GRIP, FINGER_OPEN, 'reach the tracking knob'),
            Waypoint(.76, KNOB_GRIP, 0., 'grip the stationary tracking knob'),
            Waypoint(1.28, KNOB_GRIP, 0., 'turn the tracking knob'),
            Waypoint(.64, KNOB_GRIP, FINGER_OPEN, 'release the adjusted knob'),
            Waypoint(.84, KNOB_HOVER, FINGER_OPEN, 'withdraw from the adjustment'),
            Waypoint(1.04, PARK_LEFT, FINGER_OPEN, 'park before the second run'),
            Waypoint(13., PARK_LEFT, FINGER_OPEN, 'stand clear while checking the belt'),
        ],
    }


def demo_target(arm, phase, time_s, position, rotation, finger):
    if arm == 'right':
        pitch = axis_angle_matrix(np.array((0., 1., 0.)), -.93)
        return position, pitch @ grip_rotation(), finger
    pitch = axis_angle_matrix(np.array((0., 1., 0.)), -1.27)
    turn = float(np.clip((time_s - 9.43) / 1.28, 0., 1.))
    if time_s >= 11.35:
        turn *= 1. - float(np.clip((time_s - 11.35) / 1.88, 0., 1.))
    turn_rotation = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(44.) * turn)
    return position, turn_rotation @ pitch @ grip_rotation(), finger
