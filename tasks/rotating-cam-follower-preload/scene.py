"""Set the preload of a rolling follower on a low-speed eccentric test cam."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_rotating_cam_follower_preload.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
CAM_AXIS = (.324,.103,TOP+.064)
CAM_RADIUS = .027
ECCENTRICITY = .0036
FOLLOW_RADIUS = .011
ADJUST_AXIS = (*CAM_AXIS[:2],TOP+.191)
ADJUST_RADIUS = .017
PRELOAD_OFFSET = .0135
HANDLE_RADIUS = .048
HANDLE_X = .231
HANDLE_PHASE = .18
TOGGLE_ANGLE = math.radians(53.)
UNLATCHED_STIFFNESS = .00048
LATCHED_STIFFNESS = .085
DRIVE_DEG_S = 72.
TURN_START_S = 5.70
TURN_DURATION_S = 2.45
PRESS_HEIGHT = PAD+.002
BASE = Box('cam_test_stand',pos=(*CAM_AXIS[:2],TOP+.0075),size=(.208,.162,.015),
           mass=3.,color=(.15,.28,.36))
CAM = Box('eccentric_test_cam',pos=CAM_AXIS,size=(.044,.061,.061),mass=.122,
          color=(.27,.32,.36))
SHOE = Box('preload_shoe',pos=(*CAM_AXIS[:2],TOP+.168),size=(.043,.041,.012),
           mass=.033,color=(.68,.70,.72))
YOKE = Box('follower_stem',pos=(*CAM_AXIS[:2],TOP+.116),size=(.047,.034,.066),
           mass=.022,color=(.56,.61,.65))
WHEEL = Box('preloaded_roller',pos=YOKE.pos,size=(.023,.022,.022),mass=.010,
            color=(.28,.42,.48))
ADJUSTER = Box('preload_toggle',pos=ADJUST_AXIS,size=(.187,.064,.057),mass=.078,
               color=(.61,.48,.26))
OBJECTS = [BASE,CAM,SHOE,YOKE,WHEEL,ADJUSTER]


def toggle_pad(angle):
    x,y,z = ADJUST_AXIS
    phase = angle+HANDLE_PHASE
    return np.array((HANDLE_X,y+HANDLE_RADIUS*math.cos(phase),z+HANDLE_RADIUS*math.sin(phase)))


BODY_PARTS = {
    BASE.name:(
        Solid('box',BASE.size),
        *(Solid('box',(.028,.039,.177),(x,-.048,.096),color=(.27,.41,.48)) for x in (-.065,.065)),
        *(Solid('box',(.021,.071,.051),(x,0.,.030),color=(.29,.35,.39)) for x in (-.064,.064)),
        *(Solid('cylinder',(.009,.014),(x,0.,.0565),axis='X') for x in (-.064,.064)),
        Solid('box',(.156,.024,.012),(0.,-.048,.1785)),
    ),
    CAM.name:(
        Solid('cylinder',(CAM_RADIUS,.036),(0.,0.,ECCENTRICITY),axis='X'),
        Solid('cylinder',(.005,.149),axis='X',color=(.60,.63,.67)),
        Solid('box',(.001,.004,.019),(-.0186,0.,.013),color=(.80,.59,.24)),
    ),
    SHOE.name:(Solid('box',SHOE.size),),
    YOKE.name:(
        # Leave room for a full follower rise without touching the preload eccentric.
        Solid('box',(.009,.013,.043),(0.,0.,.028)),
        Solid('box',(.047,.014,.008),(0.,0.,.015)),
        *(Solid('box',(.004,.014,.022),(x,0.,.006)) for x in (-.021,.021)),
    ),
    WHEEL.name:(Solid('cylinder',(FOLLOW_RADIUS,.023),axis='X'),),
    ADJUSTER.name:(
        Solid('cylinder',(ADJUST_RADIUS,.031),(0.,PRELOAD_OFFSET,0.),axis='X'),
        Solid('cylinder',(.0045,.161),(-.018,0.,0.),axis='X',color=(.54,.59,.63)),
        Solid('box',(.010,.050,.009),(HANDLE_X-ADJUST_AXIS[0],.023*math.cos(HANDLE_PHASE),.023*math.sin(HANDLE_PHASE)),
              roll=HANDLE_PHASE),
        Solid('cylinder',(.010,.022),tuple(toggle_pad(0.)-ADJUST_AXIS),axis='X',color=(.28,.64,.38)),
    ),
}
OBJECT_JOINTS = {
    CAM.name:{'kind':'revolute','parent':BASE.name,'anchor':CAM_AXIS,'axis':'X',
              'drive':{'damping':.02,'max_force':.16}},
    SHOE.name:{'kind':'prismatic','parent':BASE.name,'anchor':SHOE.pos,'axis':'Z',
               'lower':-.016,'upper':0.,
               'drive':{'stiffness':290.,'damping':2.,'max_force':4.6}},
    YOKE.name:{'kind':'prismatic','parent':SHOE.name,'anchor':YOKE.pos,'axis':'Z',
               'lower':-.005,'upper':.018,
               'drive':{'stiffness':225.,'damping':1.5,'max_force':5.2,'target':-.005}},
    WHEEL.name:{'kind':'revolute','parent':YOKE.name,'anchor':WHEEL.pos,'axis':'X',
                'drive':{'damping':.00001,'max_force':.009}},
    ADJUSTER.name:{'kind':'revolute','parent':BASE.name,'anchor':ADJUST_AXIS,'axis':'X',
                   'lower':-92.,'upper':2.,
                   'drive':{'stiffness':UNLATCHED_STIFFNESS,'damping':.0007,'max_force':.19}},
}
SUPPORT_JOINTS = {BASE.name:{'kind':'fixed'}}
CONTACT_PAIRS = {'follower_on_eccentric':(WHEEL.name,CAM.name),'adjuster_on_shoe':(ADJUSTER.name,SHOE.name)}
CONTACT_OFFSETS_M = {o.name:.00015 for o in (CAM,SHOE,YOKE,WHEEL,ADJUSTER)}
RIGID_SOLVER_ITERATIONS = {o.name:(96,4) for o in OBJECTS}
OBJECT_MATERIALS = {o.name:'steel bearing and roller assembly' for o in OBJECTS}
OBJECT_MATERIALS[BASE.name] = 'aluminum base, stand and bearing mount'
OBJECT_MATERIALS[ADJUSTER.name] = 'steel eccentric with polymer knob'
SUCCESS_HOLD_S = .60
MAX_PARTIAL_SCORE = .62
LIVE_CONTEXT = {}
_drives = None
_turned = 0.
_previous = None
_engaged = False
_start = None
_proof = 0.
_baseline_gap = False


def roll_of(body):
    x, _, _, w = body.quat_xyzw
    return 2.*math.atan2(x,w)


def on_reset():
    global _turned,_previous,_engaged,_start,_proof,_baseline_gap
    _turned = _proof = 0.
    _previous = _start = None
    _engaged = _baseline_gap = False
    if _drives:
        for drive in _drives:
            drive.GetTargetPositionAttr().Set(0.)
        _drives[0].GetTargetVelocityAttr().Set(0.)
        _drives[1].GetStiffnessAttr().Set(UNLATCHED_STIFFNESS)


def observe(objects, context, dt):
    global _turned,_previous,_engaged,_start,_proof,_baseline_gap
    angle = roll_of(objects[CAM.name])
    if _previous is not None:
        _turned += math.remainder(angle-_previous,math.tau)
    _previous = angle
    lever = roll_of(objects[ADJUSTER.name])
    tcp = context['tcp']['left']
    pad = toggle_pad(lever)
    _engaged |= bool(lever<-TOGGLE_ANGLE and math.dist(tcp,pad)<.044)
    pressure = context['contact_forces']['follower_on_eccentric'][2]
    if .75<context['time_s']<4.80 and pressure<.04:
        _baseline_gap = True
    clear = all(math.dist(hand,pad)>.065 for hand in context['tcp'].values())
    loaded = -.36 < lever+math.pi/2 < .16 and .28<=pressure<=4.8
    if _engaged and _baseline_gap and clear and loaded and objects[CAM.name].ang_vel[0]>.35:
        if _start is None:
            _start = _turned
        _proof = _turned-_start
    else:
        _start = None
        _proof = 0.


def update_materials():
    global _drives
    if _drives is None:
        import omni.usd
        from pxr import UsdPhysics
        stage=omni.usd.get_context().get_stage()
        joints={prim.GetName():prim for prim in stage.Traverse()}
        _drives=tuple(UsdPhysics.DriveAPI(joints[o.name+'_joint'],'angular') for o in (CAM,ADJUSTER))
    cam, adjust = _drives
    ramp = min(max((LIVE_CONTEXT['time_s']-.25)/.65,0.),1.)
    cam.GetTargetVelocityAttr().Set(DRIVE_DEG_S*ramp)
    target = -90. if _engaged else 0.
    if adjust.GetTargetPositionAttr().Get()!=target:
        adjust.GetTargetPositionAttr().Set(target)
        adjust.GetStiffnessAttr().Set(LATCHED_STIFFNESS if _engaged else UNLATCHED_STIFFNESS)


def success(objects):
    return _proof>=math.tau


def score(objects):
    return 1. if success(objects) else .62 if _proof>math.pi else .22 if _engaged else 0.


def demo_target(arm,label,t,pos,rotation,finger):
    if arm!='left':
        return pos,rotation,finger
    if label=='set the preload toggle':
        phase=np.clip((t-TURN_START_S)/TURN_DURATION_S,0.,1.)
        angle=-1.21*phase*phase*(3.-2.*phase)
        measured=toggle_pad(roll_of(LIVE_OBJECTS[ADJUSTER.name]))
        pos=np.array((measured[0],measured[1]+.003,toggle_pad(angle)[2]+PRESS_HEIGHT))
    return pos,rotation,finger


def demo():
    pad = toggle_pad(0.)
    hover = tuple(pad+(0.,0.,.021))
    press = tuple(pad+(0.,.003,PRESS_HEIGHT))
    end = tuple(toggle_pad(-1.21)+(0.,.003,PRESS_HEIGHT))
    clear = (.214,.179,TOP+.184)
    right = (.233,-.209,TOP+.172)
    return {'left':[
        Waypoint(2.25,hover,CLOSED,'reach the preload toggle'),
        Waypoint(2.35,hover,CLOSED,'observe the unadjusted cam'),
        Waypoint(1.10,press,CLOSED,'approach the green knob'),
        Waypoint(TURN_DURATION_S,end,CLOSED,'set the preload toggle'),
        Waypoint(.72,clear,OPEN,'withdraw from the knob'),
        Waypoint(12.,clear,OPEN,'observe the loaded follower'),
    ],'right':[Waypoint(1.7,right,OPEN,'park the other wrist'),
                 Waypoint(20.,right,OPEN,'keep the cam fixture clear')]}
