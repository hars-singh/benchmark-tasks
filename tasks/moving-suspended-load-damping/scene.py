"""Settle sway in a suspended carrier while its overhead trolley keeps moving."""

import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_moving_suspended_load_damping.geometry import Solid
from dynamic_task_moving_suspended_load_damping.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRAVEL_M = .052
PERIOD_S = 8.
RELEASE_S = .35
SWAY_START = math.radians(16.)
FREE_HINGE_DAMPING = .000035
ACTIVE_SWAY_ANGLE = math.radians(4.)
ACTIVE_SWAY_RATE = .38
QUIET_GRASP_S = .25
LINK_M = .142
PIVOT = np.array((.272, 0., TOP+.226))
GANTRY_X = .433
SIDES = {'left':1, 'right':-1}
TAB = np.array((-.017, .086, .010))


def roll(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array(((1., 0., 0.), (0., c, -s), (0., s, c)))


SKEW = roll(SWAY_START)
NEUTRAL = PIVOT-np.array((0., 0., LINK_M))
RAIL = Box('gantry_rail', pos=(GANTRY_X, 0., TOP+.275), size=(.105, .483, .018),
           color=(.35, .39, .44))
TROLLEY = Box('beam_trolley', pos=(GANTRY_X, 0., TOP+.247), size=(.066, .042, .026),
             mass=2.8, color=(.16, .30, .48))
LOAD = Box('suspended_fixture', pos=tuple(map(float, PIVOT+SKEW@np.array((0.,0.,-LINK_M)))),
           size=(.055, .148, .052), mass=.27, color=(.64, .37, .17))
OBJECTS = [RAIL, TROLLEY, LOAD]


def loaded_solid(kind, size, offset=(0.,0.,0.), *, color=None):
    return Solid(kind, size, tuple(map(float,SKEW@offset)), roll=SWAY_START, color=color)


BODY_PARTS = {
    RAIL.name:(
        Solid('box', RAIL.size),
        *(Solid('box', (.032,.029,.266), (.033,y,-.134)) for y in (-.219,.219)),
        *(Solid('box', (.091,.060,.012), (.025,y,-.269)) for y in (-.219,.219)),
        *(Solid('cylinder', (.005,.460), (x,0.,-.013), axis='Y', color=(.65,.70,.73))
          for x in (-.046,.046)),
    ),
    TROLLEY.name:(
        Solid('box', TROLLEY.size),
        Solid('box', (.202,.036,.012), (-.087,0.,-.010), color=(.35,.54,.66)),
        Solid('cylinder', (.008,.029), (PIVOT[0]-GANTRY_X,0.,-.021), axis='X', color=(.68,.71,.72)),
    ),
    LOAD.name:(
        loaded_solid('box', LOAD.size),
        loaded_solid('box', (.010,.011,.122), (0.,0.,.078), color=(.64,.67,.70)),
        loaded_solid('box', (.029,.162,.010), (0.,0.,.025), color=(.40,.47,.49)),
        *(loaded_solid('box', (.018,.036,.017), TAB*(1.,sign,1.), color=(.82,.61,.25))
          for sign in SIDES.values()),
    ),
}
OBJECT_JOINTS = {LOAD.name:{'kind':'revolute','parent':TROLLEY.name,'anchor':tuple(map(float,PIVOT)),
                          'axis':'X','lower':-49.,'upper':18.,
                          'drive':{'stiffness':20.,'damping':.23,'max_force':1.}}}
MOTIONS = {TROLLEY.name:OscillatingPlatform(TROLLEY.pos,(0.,TRAVEL_M,0.),PERIOD_S)}
SUPPORT_JOINTS = {TROLLEY.name:{'axis':'Y','lower':-.069,'upper':.069}}
CONTACT_OFFSETS_M = {LOAD.name:.00040,TROLLEY.name:.00040}
RIGID_SOLVER_ITERATIONS = {LOAD.name:(96,4),TROLLEY.name:(96,4)}
SUCCESS_HOLD_S = .80
MAX_PARTIAL_SCORE = .65
LIVE_CONTEXT = {}
_drive = None
_baseline_sway = 0.
_quiet_grasp_s = 0.
_grasped = _held_steady = _settled = False
_proof_y = None
_centering_offset = {}


def on_reset():
    global _baseline_sway, _quiet_grasp_s, _grasped, _held_steady, _settled, _proof_y
    _baseline_sway = _quiet_grasp_s = 0.
    _grasped = _held_steady = _settled = False
    _proof_y = None
    _centering_offset.clear()
    if _drive is not None:
        _drive.GetStiffnessAttr().Set(20.)
        _drive.GetDampingAttr().Set(.23)


def orientation(obj):
    return quat_xyzw_to_matrix(np.asarray(obj.quat_xyzw))@SKEW


def fixture_position(obj, origin, side):
    return origin+orientation(obj)@(TAB*(1.,SIDES[side],1.))


def observe(objects, context, dt):
    global _baseline_sway, _quiet_grasp_s, _grasped, _held_steady, _settled, _proof_y
    load = objects[LOAD.name]
    trolley = objects[TROLLEY.name]
    origin = context['body_origins'][LOAD.name]
    measured = orientation(load)
    theta = math.atan2(measured[2,1],measured[1,1])
    angular_speed = abs(load.ang_vel[0])
    quiet = abs(theta)<math.radians(2.4) and angular_speed<.17
    active_sway = abs(theta)>ACTIVE_SWAY_ANGLE or angular_speed>ACTIVE_SWAY_RATE
    grips = {side:fixture_position(load,origin,side) for side in SIDES}
    hands_clear = all(math.dist(tcp,grip)>.048 for tcp in context['tcp'].values() for grip in grips.values())
    held = all(.04<context['gripper'][side]<.65 and math.dist(tcp,grips[side])<.026
               for side,tcp in context['tcp'].items())
    if RELEASE_S+.02<context['time_s']<1.50 and hands_clear:
        _baseline_sway=max(_baseline_sway,abs(theta))
    moving = abs(trolley.lin_vel[1])>.012
    _grasped |= bool(_baseline_sway>math.radians(10.) and moving and held and active_sway)
    _quiet_grasp_s = _quiet_grasp_s+dt if _grasped and moving and held and quiet else 0.
    _held_steady |= _quiet_grasp_s>=QUIET_GRASP_S-1e-8
    measured_pivot = origin+measured@np.array((0.,0.,LINK_M))
    trolley_shift = context['body_origins'][TROLLEY.name]-np.asarray(TROLLEY.pos)
    if _held_steady and quiet and moving and hands_clear and math.dist(measured_pivot,PIVOT+trolley_shift)<.002:
        y = float(trolley_shift[1])
        if _proof_y is None:
            _proof_y=y
        _settled=abs(y-_proof_y)>.010
    else:
        _proof_y=None
        _settled=False


def update_materials():
    global _drive
    if _drive is None:
        import omni.usd
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        joint, = (p for p in stage.Traverse() if p.GetName()==LOAD.name+'_joint')
        _drive = UsdPhysics.DriveAPI(joint,'angular')
    if LIVE_CONTEXT['time_s']>=RELEASE_S and _drive.GetStiffnessAttr().Get()!=0.:
        _drive.GetStiffnessAttr().Set(0.)
        _drive.GetDampingAttr().Set(FREE_HINGE_DAMPING)


def success(objects):
    return _settled


def score(objects):
    return 1. if success(objects) else .65 if _held_steady else .25 if _grasped else 0.


def demo():
    sequence = {}
    for side, sign in SIDES.items():
        contact = NEUTRAL+TAB*(1.,sign,1.)+np.array((0.,0.,PAD-.001))
        approach = contact+np.array((0.,0.,.036))
        clear = (.246,sign*.193,TOP+.168)
        yaw=-sign*math.pi/2
        sequence[side]=[
            Waypoint(1.70,tuple(approach),OPEN,'approach the swinging tabs',yaw),
            Waypoint(.82,tuple(contact),OPEN,'track the swinging tabs',yaw),
            Waypoint(.76,tuple(contact),CLOSED,'hold the two tabs',yaw),
            Waypoint(1.40,tuple(contact),CLOSED,'bring the load upright',yaw),
            Waypoint(.95,tuple(contact),CLOSED,'settle on the running guide',yaw),
            Waypoint(.55,tuple(contact),OPEN,'release the settled load',yaw),
            Waypoint(.50,tuple(approach),OPEN,'lift clear of the tabs',yaw),
            Waypoint(1.05,clear,OPEN,'withdraw from the moving load',yaw),
            Waypoint(8.0,clear,OPEN,'observe hands-off travel',yaw),
        ]
    return sequence


def demo_target(arm,label,t,pos,rotation,finger):
    if not LIVE_CONTEXT or label in {'withdraw from the moving load','observe hands-off travel','done'}:
        return pos,rotation,finger
    origins=LIVE_CONTEXT['body_origins']
    load=LIVE_OBJECTS[LOAD.name]
    origin=origins[LOAD.name]
    grip=fixture_position(load,origin,arm)
    neutral=NEUTRAL+TAB*(1.,SIDES[arm],1.)
    shift=origins[TROLLEY.name]-np.asarray(TROLLEY.pos)
    if label in {'approach the swinging tabs','track the swinging tabs','hold the two tabs'}:
        lever=grip-origin
        velocity=load.lin_vel+np.cross(load.ang_vel,lever)
        gain=ik.smoothstep(float(np.clip(t/1.7,0.,1.)))
        pos=pos+gain*(grip-neutral+.050*velocity)
    elif label=='bring the load upright':
        if arm not in _centering_offset:
            _centering_offset[arm]=grip-neutral-shift
        portion=ik.smoothstep(float(np.clip((t-3.28)/1.40,0.,1.)))
        pos=pos+shift+(1.-portion)*_centering_offset[arm]
    else:
        pos=pos+shift
    return pos,rotation,finger
