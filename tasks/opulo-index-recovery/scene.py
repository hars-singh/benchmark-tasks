"""Recover an over-advanced feeder and check three loaded component positions."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
from dynamic_task_opulo_index_recovery.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
Y = .036
X = .436
LANE_Z = TOP+.095
FILM_Z = LANE_Z+.00415
PICK_X = .399
START_POCKET_X = .395
HOME = np.array((.496,Y,LANE_Z))
GEAR_X = .561
PEEL_Y = Y+.029
PEEL_Z = FILM_Z+.020
POCKET_PITCH = .008
TAP_STEP = .002
PIECE_COUNT = 3
RADIUS = .0112
DRIVER_LEAD = .0704
FWD_X = .325
FWD_Y, BACK_Y = Y-.033,Y+.008
BUTTON_Z = TOP+.086
BUTTON_TRAVEL = .0035
FRAME_COLOR = (.18,.30,.42)
GUIDE_COLOR = (.38,.42,.44)
COVER_COLOR = (.67,.72,.64)
FILM_COLOR = (.72,.78,.59)
OPEN_LED = (.20,.58,.91)
PACKAGE = (.080,.089,.090)

CAGE = Box('opulo_12mm_feeder_on_loading_station',pos=(X,Y,TOP+.048),
           size=(.248,.104,.096),mass=.76,color=FRAME_COLOR)
CARRIER = Box('loaded_12mm_pocket_tape',pos=tuple(map(float,HOME)),size=(.218,.012,.0032),
              mass=.009,color=(.23,.24,.27))
DRIVE = Box('carrier_sprocket',pos=(.435,Y-.004,LANE_Z-.013),size=(.023,.011,.023),mass=.026,color=GUIDE_COLOR)
PEEL = Box('powered_cover_peel_gear',pos=(GEAR_X,PEEL_Y,PEEL_Z-.0055),
           size=(.011,.014,.011),mass=.007,color=GUIDE_COLOR)
IDLER = Box('cover_peel_idler',pos=(GEAR_X,PEEL_Y,PEEL_Z+.0055),
            size=(.011,.014,.011),mass=.006,color=GUIDE_COLOR)
FWD = Box('feeder_forward_button',pos=(FWD_X,FWD_Y,BUTTON_Z),
          size=(.013,.015,.0064),mass=.005,color=(.24,.55,.71))
BACK = Box('feeder_reverse_button',pos=(FWD_X,BACK_Y,BUTTON_Z),
           size=FWD.size,mass=.005,color=(.29,.48,.62))


def component(i):
    return Box(f'package_in_pocket_{i+1}',pos=(START_POCKET_X+i*POCKET_PITCH,Y+.00015,LANE_Z+.00137),
               size=(.0051,.0058,.00175),mass=.00012,color=PACKAGE)


COMPONENTS=tuple(component(i) for i in range(PIECE_COUNT))
OBJECTS=[CAGE,CARRIER,DRIVE,PEEL,IDLER,FWD,BACK,*COMPONENTS]


def local_box(size,world_pos,color):
    return Solid('box',size,tuple(float(v-h) for v,h in zip(world_pos,CAGE.pos)),color=color)


def gear(radius,width,color):
    parts=[Solid('cylinder',(radius,width),axis='Y',color=color)]
    for a in np.linspace(0.,math.tau,12,endpoint=False):
        parts.append(Solid('box',(.0018,width,.002),
                           (radius*math.sin(a),0.,radius*math.cos(a)),pitch=float(a),color=color))
    return tuple(parts)


hold_rails=[
    local_box((.181,.0025,.0064),(.480,Y+s*.0074,LANE_Z-.00055),GUIDE_COLOR)
    for s in (-1,1)
]
BODY_PARTS={
    CAGE.name:(
        local_box((.161,.079,.020),(.428,Y,TOP+.010),FRAME_COLOR),
        local_box((.193,.023,.058),(.460,Y+.007,TOP+.050),FRAME_COLOR),
        local_box((.041,.078,.026),(FWD_X,Y-.012,TOP+.067),FRAME_COLOR),
        local_box((.053,.030,.012),(.537,PEEL_Y,PEEL_Z-.017),GUIDE_COLOR),
        local_box((.039,.0031,.009),(.548,PEEL_Y-.0066,PEEL_Z-.002),GUIDE_COLOR),
        local_box((.039,.0031,.009),(.548,PEEL_Y+.0066,PEEL_Z-.002),GUIDE_COLOR),
        local_box((.083,.025,.004),(.499,Y+.043,FILM_Z+.0135),FRAME_COLOR),
        local_box((.021,.019,.004),(.449,Y+.010,FILM_Z+.0075),FRAME_COLOR),
        local_box((.213,.026,.010),(.457,Y,LANE_Z-.0063),FRAME_COLOR),
        *hold_rails,
    ),
    DRIVE.name:gear(RADIUS,.005,GUIDE_COLOR),
    PEEL.name:gear(.0052,.012, (.71,.53,.20)),
    IDLER.name:gear(.0052,.012, (.71,.53,.20)),
}
carrier_parts=[Solid('box',(.218,.012,.001),(0.,0.,-.00225))]
for sign in (-1,1):
    carrier_parts.append(Solid('box',(.218,.0028,.0021),(0.,sign*.0046,-.00071)))
for i in range(PIECE_COUNT+1):
    pocket_x=START_POCKET_X+(i-.5)*POCKET_PITCH-HOME[0]
    carrier_parts.append(Solid('box',(.0022,.0072,.0025),(float(pocket_x),0.,-.00045)))
BODY_PARTS[CARRIER.name]=tuple(carrier_parts)
VISUAL_PARTS={
    CARRIER.name:tuple(Solid('box',(.00165,.00135,.0002),(x,-.0048,.0004),color=(.53,.57,.56))
                       for x in np.linspace(-.101,.102,51)),
    CAGE.name:(
        local_box((.016,.003,.0004),(PICK_X,Y-.011,LANE_Z+.0028),(.74,.69,.28)),
        local_box((.00135,.007,.00035),(PICK_X,Y-.010,LANE_Z+.0032),(.90,.84,.38)),
        local_box((.00135,.007,.00035),(PICK_X,Y+.010,LANE_Z+.0032),(.90,.84,.38)),
        local_box((.019,.010,.0006),(PICK_X+.016,Y,LANE_Z+.0085),(.32,.38,.35)),
    ),
}
SUPPORT_JOINTS={CAGE.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    CARRIER.name:{'kind':'prismatic','parent':CAGE.name,'axis':'X','anchor':CARRIER.pos,
                  'lower':-.027,'upper':.009,'drive':{'damping':65.,'max_force':4.}},
    DRIVE.name:{'kind':'revolute','parent':CAGE.name,'axis':'Y','anchor':DRIVE.pos,
                'drive':{'stiffness':.075,'damping':.039,'max_force':.18},
                'screw_coupling':{'carriage':CARRIER.name,'travel_per_turn_m':-DRIVER_LEAD}},
    PEEL.name:{'kind':'revolute','parent':CAGE.name,'axis':'Y','anchor':PEEL.pos,
               'drive':{'damping':.015,'max_force':.055}},
    IDLER.name:{'kind':'revolute','parent':CAGE.name,'axis':'Y','anchor':IDLER.pos,
                'drive':{'damping':.0007,'max_force':.025}},
}
for button in (FWD,BACK):
    OBJECT_JOINTS[button.name]={'kind':'prismatic','parent':CAGE.name,'axis':'Z','anchor':button.pos,
        'lower':-BUTTON_TRAVEL,'upper':0.,'drive':{'stiffness':1040.,'damping':3.,'max_force':5.0}}
SURFACE_FRICTION={CARRIER.name:(.39,.32), **{p.name:(.72,.55) for p in COMPONENTS}}
CONTACT_OFFSETS_M={o.name:.000105 for o in (CARRIER,FWD,BACK,*COMPONENTS)}
RIGID_SOLVER_ITERATIONS={o.name:(72,10) for o in (DRIVE,PEEL,IDLER,CARRIER,*COMPONENTS)}
OBJECT_MATERIALS={CAGE.name:'stock-style Opulo 12 mm feeder and aluminium loading rail',
                  CARRIER.name:'12 mm embossed antistatic carrier tape',
                  DRIVE.name:'native stock-style index sprocket',PEEL.name:'powered cover peel gear',
                  IDLER.name:'opposed free peel gear',FWD.name:'blue forward button',BACK.name:'reverse button',
                  **{p.name:'dummy SOIC-8 package seated in embossed tape' for p in COMPONENTS}}

cam=np.asarray((-.024,-.183,.247))
view=np.array((-.024,0.,.064))-cam
view/=np.linalg.norm(view)
side=np.cross(view,(0.,0.,1.));side/=np.linalg.norm(side)
cam_q=matrix_to_quat_xyzw(np.column_stack((side,np.cross(side,view),-view)))
TASK_RECORD_CAMERA=(CAGE.name,Camera(tuple(cam),tuple(cam_q),'opengl',Lens.from_focal(34.,640,360)))


LIVE_CONTEXT = {}
SUCCESS_HOLD_S = 1.
MAX_PARTIAL_SCORE = .5
_requested = 0.
_pressed = 0
_requested_before_check = 0.
_release_s = .18
_stage = 0
_settle_s = 0.
_bad_tape = False
_drive = None


def on_reset():
    global _requested, _pressed, _requested_before_check, _stage, _settle_s, _bad_tape, _release_s
    _requested = _requested_before_check = _settle_s = 0.
    _pressed = _stage = 0
    _release_s = .18
    _bad_tape = False


def observe(objects, context, dt):
    global _requested, _pressed, _requested_before_check, _stage, _settle_s, _bad_tape, _release_s
    origins = context['body_origins']
    forward = FWD.pos[2] - origins[FWD.name][2] > .0021
    reverse = BACK.pos[2] - origins[BACK.name][2] > .0021
    button = int(forward) - int(reverse)
    if button and _release_s >= .13:
        _requested = float(np.clip(_requested + TAP_STEP * button, -.008, .026))
    _release_s = 0. if button else _release_s + dt
    _pressed = button
    carrier = np.asarray(origins[CARRIER.name])
    _bad_tape |= (abs(carrier[2] - LANE_Z) > .0025 or abs(carrier[1] - Y) > .0030)
    for i, part in enumerate(COMPONENTS):
        pocket_x = START_POCKET_X + i * POCKET_PITCH - HOME[0]
        local = np.asarray(origins[part.name]) - carrier
        _bad_tape |= not (abs(local[0] - pocket_x) < .0035 and abs(local[1]) < .003
                          and -.0015 < local[2] < .0045)
    if _stage >= PIECE_COUNT:
        return
    part = COMPONENTS[_stage]
    stopped = np.linalg.norm(objects[CARRIER.name].lin_vel) < .0021 and not forward and not reverse
    reached_requested_position = abs(carrier[0] - HOME[0] + _requested) < .0011
    centered = abs(origins[part.name][0] - PICK_X) <= .00115
    in_sequence = _requested <= -.0032 if _stage == 0 else _requested > _requested_before_check + .0058
    if stopped and reached_requested_position and centered and in_sequence:
        _settle_s += dt
        if _settle_s >= .40:
            _stage += 1
            _requested_before_check = _requested
            _settle_s = 0.
    else:
        _settle_s = 0.


def update_actuators():
    global _drive
    if _drive is None:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        joint = next(prim for prim in stage.Traverse() if prim.GetName() == DRIVE.name + '_joint')
        _drive = UsdPhysics.DriveAPI(joint, 'angular')
    _drive.GetTargetPositionAttr().Set(float(360. * _requested / DRIVER_LEAD))


def success(objects):
    origins = LIVE_CONTEXT['body_origins']
    centered = abs(origins[COMPONENTS[-1].name][0] - PICK_X) <= .00115
    clear = all(math.dist((FWD_X, Y, BUTTON_Z), tcp) > .081 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(not _bad_tape and _stage == PIECE_COUNT and centered and clear
                and np.linalg.norm(objects[CARRIER.name].lin_vel) < .0021 and not _pressed)


def score(objects):
    if _bad_tape:
        return 0.
    return 1. if success(objects) else .5 if _stage >= 2 else 0.


def button_tap(button):
    x, y, z = button.pos
    touch = (x, y - .0016, z - .0018)
    hover = (x, y, z + .031)
    label = 'reverse' if button == BACK else 'advance'
    return [
        Waypoint(.43, touch, -.008, 'tap ' + label),
        Waypoint(.17, touch, -.008, 'hold briefly'),
        Waypoint(.38, hover, -.008, 'release the index button'),
        Waypoint(.17, hover, -.008, 'let the switch rise'),
    ]


def demo():
    rear = (FWD_X, BACK_Y, BUTTON_Z + .061)
    forward = (FWD_X, FWD_Y, BUTTON_Z + .044)
    park = (.242, -.229, TOP + .186)
    steps = [Waypoint(1.36, rear, -.008, 'approach the reverse button')]
    for _ in range(2):
        steps.extend(button_tap(BACK))
    steps.extend((Waypoint(2.55, (FWD_X, BACK_Y, BUTTON_Z + .049), -.008, 'verify the recovered first package'),
                  Waypoint(.72, forward, -.008, 'move to the forward button')))
    for package in (2, 3):
        for _ in range(4):
            steps.extend(button_tap(FWD))
        steps.append(Waypoint(2.55, forward, -.008, f'let package {package} settle on the center mark'))
    steps.extend((Waypoint(1.02, park, FINGER_OPEN, 'clear the pick window'),
                  Waypoint(2.1, park, FINGER_OPEN, 'show the third package aligned')))
    return {'right': steps}


def demo_target(arm, phase, time_s, position, rotation, finger):
    pitch = axis_angle_matrix(np.array((0., 1., 0.)), -.61)
    return position, pitch @ grip_rotation(), finger
