"""Take one empty paper cup from a horizontal spring-fed counter dispenser."""

import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

from dynamic_task_single_cup_dispenser.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
CUP_DISPENSER_MODEL = 'San Jamar EZ-Fit C2410C'
TABLE = TABLE_TOP[SCENE]
FACE_X = .421
CENTER_Y = -.164
CENTER_Z = TABLE + .129
TUBE_LENGTH = .597
RIM_RADIUS = .0385
BASE_RADIUS = .026
CUP_LENGTH = .095
NEST_PITCH = .008
MOUNT_RADIUS = .073
LEAF_OUTER = .061
LEAF_INNER = RIM_RADIUS - .00145
LEAF_X = FACE_X + .002
REST_RIM_X = FACE_X + .007
MAT = (.420, -.420)
MAT_SIZE = .138
RISER_HEIGHT = .127
MAT_TOP = TABLE + RISER_HEIGHT + .0016
CUP_COUNT = 4
CUPS = tuple(f'cup_{i}' for i in range(1, CUP_COUNT + 1))
LEAVES = tuple(f'gasket_leaf_{i}' for i in range(1, 7))
FRAME_COLOR = (.10, .12, .14)
TRIM_COLOR = (.23, .26, .29)
GASKET_COLOR = (.078, .080, .087)
PAPER_COLOR = (.95, .91, .83)
MARK_COLOR = (.12, .32, .39)

# The fixture's depth is behind the service face. The near tabletop provides a cup landing area.
TASK_TABLE = Box('counter', size=(1.16, 1.18, .038), pos=(.440, 0., TABLE-.019))
FRAME = Box('dispenser', pos=(FACE_X, CENTER_Y, CENTER_Z), size=(TUBE_LENGTH, .156, .156),
            mass=1.7, color=FRAME_COLOR)
PUSHER_X = REST_RIM_X + (CUP_COUNT - 1) * NEST_PITCH + .0048
PUSHER = Box('cup_pusher', pos=(PUSHER_X, CENTER_Y, CENTER_Z), size=(.004, .083, .083),
             mass=.055, color=(.39, .43, .47))
MAT_OBJECT = Box('cup_mat', pos=(*MAT, MAT_TOP-.0008), size=(MAT_SIZE, MAT_SIZE, .0016),
                 color=MARK_COLOR)
RISER = Box('serving_riser', pos=(*MAT, TABLE+RISER_HEIGHT/2), size=(.171,.171,RISER_HEIGHT),
            color=TRIM_COLOR)


def circle_angles(count):
    return tuple(math.tau * i / count for i in range(count))


def ring_segments(x, inner_radius, outer_radius, depth, segments, color):
    radius = (inner_radius + outer_radius) / 2
    width = 2 * outer_radius * math.tan(math.pi / segments)
    return tuple(Solid('box', (depth, outer_radius-inner_radius, width),
                       (x, radius*math.cos(a), radius*math.sin(a)), roll=a, color=color)
                 for a in circle_angles(segments))


def cup_solids():
    # Shared edges give adjacent sectors a conformal taper for physical nesting.
    sectors = 24
    span = math.tau / sectors
    thickness = .00045
    top_inner = RIM_RADIUS - .00125 - thickness
    wall = tuple(Solid('tapered_panel', (BASE_RADIUS-thickness, top_inner, thickness,
                                        CUP_LENGTH-.002, span), roll=a)
                 for a in circle_angles(sectors))
    base = Solid('cylinder', (BASE_RADIUS, .0018), (-CUP_LENGTH/2+.0009,0.,0.), axis='X')
    rim = tuple(Solid('tapered_panel', (top_inner, top_inner, RIM_RADIUS-top_inner,
                                       .0034, span), (CUP_LENGTH/2-.0017,0.,0.), roll=a)
                for a in circle_angles(sectors))
    return (base, *wall, *rim)


# The six real hinges are a lumped approximation of the flexible hexagonal gasket.
# The native pusher is always interacting with the stack by contact.
OBJECTS = [FRAME, PUSHER, RISER, MAT_OBJECT]
BODY_PARTS = {
    FRAME.name: (
        *ring_segments((.010+TUBE_LENGTH)/2, .056, .0665, TUBE_LENGTH-.010, 20, FRAME_COLOR),
        *ring_segments(.007, .055, .082, .016, 24, TRIM_COLOR),
        Solid('cylinder', (.0665, .009), (TUBE_LENGTH-.0045,0.,0.), axis='X'),
        Solid('box', (.113, .186, .015), (.072,0.,TABLE-CENTER_Z+.0075), color=TRIM_COLOR),
        Solid('box', (.110, .186, .015), (.476,0.,TABLE-CENTER_Z+.0075), color=TRIM_COLOR),
        Solid('box', (.113, .148, .057), (.072,0.,-.093), color=FRAME_COLOR),
        Solid('box', (.110, .148, .057), (.476,0.,-.093), color=FRAME_COLOR),
        # Inner guide ribs belong to the tube and end behind the flexible outlet.
        Solid('box', (.525, .023, .004), (.285,0.,-.0425), color=GASKET_COLOR),
        Solid('box', (.525, .004, .027), (.285,-.0425,0.), color=GASKET_COLOR),
        Solid('box', (.525, .004, .027), (.285,.0425,0.), color=GASKET_COLOR),
    ),
    PUSHER.name: (Solid('cylinder', (.0415,.004), axis='X'),),
    RISER.name: (
        Solid('box', (.171,.171,.008), (0.,0.,RISER_HEIGHT/2-.004)),
        Solid('box', (.171,.009,RISER_HEIGHT-.008), (0.,-.081,-.004)),
        Solid('box', (.171,.009,RISER_HEIGHT-.008), (0.,.081,-.004)),
    ),
}
SUPPORT_JOINTS = {FRAME.name: {'kind': 'fixed'}}
OBJECT_JOINTS = {
    PUSHER.name: {'kind':'prismatic','parent':FRAME.name,'anchor':PUSHER.pos,
                  'axis':'X','lower':-.032,'upper':.250,
                  'drive':{'stiffness':85.,'damping':2.6,'target':-.024,'max_force':1.10}},
}
SURFACE_FRICTION = {FRAME.name: (.20,.16), PUSHER.name: (.20,.16)}
CONTACT_OFFSETS_M = {FRAME.name:.00040, PUSHER.name:.00027}
RIGID_SOLVER_ITERATIONS = {PUSHER.name: (48, 4)}
OBJECT_MATERIALS = {
    FRAME.name: 'high-impact plastic on a fixed counter mount',
    PUSHER.name: 'molded plastic driven by the captive steel spring',
    RISER.name: 'stainless steel counter riser',
    MAT_OBJECT.name: 'blue nonslip silicone',
    **dict.fromkeys(CUPS, 'single-wall paperboard with rolled rims'),
    **dict.fromkeys(LEAVES, 'flexible elastomer gasket, approximated by sprung leaves'),
}
for index, name in enumerate(CUPS):
    x = REST_RIM_X - CUP_LENGTH/2 + index * NEST_PITCH
    OBJECTS.append(Box(name, pos=(x,CENTER_Y,CENTER_Z), size=(CUP_LENGTH,2*RIM_RADIUS,2*RIM_RADIUS),
                       mass=.008, color=PAPER_COLOR))
    BODY_PARTS[name] = cup_solids()
    CONTACT_OFFSETS_M[name] = .00012
    SURFACE_FRICTION[name] = (.62,.48)
    RIGID_SOLVER_ITERATIONS[name] = (64, 4)
for a, name in zip(circle_angles(6), LEAVES, strict=True):
    y, z = math.cos(a), math.sin(a)
    center = (LEAF_INNER+LEAF_OUTER)/2
    leaf = Box(name, pos=(LEAF_X,CENTER_Y+center*y,CENTER_Z+center*z), size=(.003,.025,.047),
               mass=.009, color=GASKET_COLOR)
    OBJECTS.append(leaf)
    BODY_PARTS[name] = (Solid('box', (.003,LEAF_OUTER-LEAF_INNER,.0415), roll=a),)
    OBJECT_JOINTS[name] = {
        'kind':'revolute','parent':FRAME.name,'axis':'Y',
        'anchor':(LEAF_X, CENTER_Y+LEAF_OUTER*y, CENTER_Z+LEAF_OUTER*z),
        'frame_roll_rad':a+math.pi/2,'lower':-57.,'upper':58.,
        'drive':{'stiffness':.022,'damping':.0012,'max_force':.045,'target':0.},
    }
    CONTACT_OFFSETS_M[name] = .00022
    SURFACE_FRICTION[name] = (.24,.19)
    RIGID_SOLVER_ITERATIONS[name] = (48, 4)

SUCCESS_HOLD_S = 1.0
MAX_PARTIAL_SCORE = .50
LIVE_CONTEXT = {}
_retained_start = None
_breached = False
_lead_exited = False
_grasp_delta = np.zeros(3)


def on_reset():
    global _retained_start, _breached, _lead_exited, _grasp_delta
    _retained_start = None
    _breached = False
    _lead_exited = False
    _grasp_delta = np.zeros(3)


def axial_direction(body):
    x, y, z, w = body.quat_xyzw
    return np.asarray((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)))


def rim_center(name, objects, origins):
    return origins[name]+(CUP_LENGTH/2-.0017)*axial_direction(objects[name])


def observe(objects, context, dt):
    global _retained_start, _breached, _lead_exited
    origins = context['body_origins']
    if _retained_start is None:
        _retained_start = float(rim_center(CUPS[1], objects, origins)[0])
    if any(objects[name].low[2] < TABLE-.035 for name in CUPS):
        _breached = True
    if any(objects[name].high[0] < FACE_X-.003 for name in CUPS[1:]):
        _breached = True
    if objects[CUPS[0]].high[0] < FACE_X-.003:
        _lead_exited = True


def retained(objects):
    if _retained_start is None:
        return False
    origins = LIVE_CONTEXT['body_origins']
    next_rim = rim_center(CUPS[1], objects, origins)
    return bool(
        _retained_start-next_rim[0] >= .0025
        and FACE_X-.004 <= next_rim[0] <= FACE_X+.016
        and all(axial_direction(objects[n])[0] >= .92 for n in CUPS[1:])
        and all(FACE_X-.004 <= rim_center(n, objects, origins)[0] <= FACE_X+.078 for n in CUPS[1:])
        and all(abs(origins[n][1]-CENTER_Y) <= .012 and abs(origins[n][2]-CENTER_Z) <= .016
                for n in CUPS[1:])
    )


def success(objects):
    lead = objects[CUPS[0]]
    center = LIVE_CONTEXT['body_origins'][CUPS[0]]
    return bool(
        not _breached and _lead_exited and retained(objects)
        and axial_direction(lead)[2] >= math.cos(math.radians(12))
        and abs(lead.low[2]-MAT_TOP) <= .004
        and np.all(lead.low[:2] >= np.array(MAT)-MAT_SIZE/2)
        and np.all(lead.high[:2] <= np.array(MAT)+MAT_SIZE/2)
        and np.linalg.norm(lead.lin_vel) <= .015 and np.linalg.norm(lead.ang_vel) <= .35
        and all(math.dist(center, p) >= .083 for p in LIVE_CONTEXT['tcp'].values())
    )


def score(objects):
    if _breached:
        return 0.0
    if success(objects):
        return 1.0
    lead = objects[CUPS[0]]
    outside = (lead.high[0] < FACE_X-.003
               or lead.high[1] < CENTER_Y-MOUNT_RADIUS
               or lead.low[1] > CENTER_Y+MOUNT_RADIUS)
    return .50 if _lead_exited and retained(objects) and outside else 0.0


PINCH = -.0265
PICK_DEPTH = .029
PICK = (REST_RIM_X-CUP_LENGTH+PICK_DEPTH, CENTER_Y, CENTER_Z+.006)
PULL_X = .223
PULL_AT = 3.35
PULL_S = 1.75
TILT_AT = 7.0
TILT_SECONDS = 2.0
TURN_Y = -.285
CARRY_Z = MAT_TOP + .100
RELEASE_Z = MAT_TOP + PICK_DEPTH + .003


def demo():
    return {'right': [
        Waypoint(1.45, (PICK[0],PICK[1],CENTER_Z+.064), FINGER_OPEN, 'approach'),
        Waypoint(1.10, PICK, FINGER_OPEN, 'descend'),
        Waypoint(.80, PICK, PINCH, 'grip cup wall'),
        Waypoint(PULL_S, (PULL_X,PICK[1],PICK[2]), PINCH, 'pull through gasket'),
        Waypoint(.85, (PULL_X,PICK[1],CARRY_Z), PINCH, 'raise the extracted cup'),
        Waypoint(1.05, (.290,TURN_Y,CARRY_Z), PINCH, 'carry forward of the riser'),
        Waypoint(TILT_SECONDS, (.300,TURN_Y,CARRY_Z), PINCH, 'turn upright'),
        Waypoint(.65, (.300,TURN_Y,CARRY_Z), PINCH, 'settle above the counter'),
        Waypoint(1.60, (MAT[0]-.005,MAT[1],CARRY_Z), PINCH, 'move above mat'),
        Waypoint(1.45, (MAT[0]-.005,MAT[1],RELEASE_Z), PINCH, 'lower to mat'),
        Waypoint(.95, (MAT[0]-.005,MAT[1],RELEASE_Z), FINGER_OPEN, 'fully open over mat'),
        Waypoint(1.25, (MAT[0]-.117,MAT[1],RELEASE_Z), FINGER_OPEN, 'withdraw along the grip axis'),
        Waypoint(1.30, (MAT[0]-.117,MAT[1],RELEASE_Z), FINGER_OPEN, 'hold clear'),
    ]}


def demo_target(arm, phase, time_s, position, rotation, finger):
    global _grasp_delta
    if phase in ('approach', 'descend'):
        nominal = np.array((REST_RIM_X-CUP_LENGTH/2, CENTER_Y, CENTER_Z))
        offset = LIVE_CONTEXT['body_origins'][CUPS[0]] - nominal
        _grasp_delta = np.clip(offset, (-.012, -.009, -.012), (.028, .009, .012))
    if phase in ('approach', 'descend', 'grip cup wall'):
        weight = ik.smoothstep(min(1., time_s/.5))
        position = position + weight*_grasp_delta
    if phase == 'pull through gasket':
        weight = 1.-ik.smoothstep(float(np.clip((time_s-PULL_AT)/PULL_S, 0., 1.)))
        position = position + weight*_grasp_delta
    return position, rotation, finger


def demo_rotation(arm, phase, time_s, rotation):
    if time_s < TILT_AT:
        return rotation
    fraction = ik.smoothstep(float(np.clip((time_s-TILT_AT)/TILT_SECONDS, 0, 1)))
    return axis_angle_matrix(np.array((0.,1.,0.)), -math.pi*fraction/2) @ grip_rotation()
