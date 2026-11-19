"""Close the cable guide and hold the motor switch to rewind a field cable."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_bench_cable_reel_winding.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
AXIS = (.570,-.278,TOP+.196)
DRUM_RADIUS = .076
FLANGE_RADIUS = .178
DRUM_WIDTH = .122
CABLE_RADIUS = .003
LINK_LENGTH = .019
LINK_COUNT = 43
CABLE_MASS_PER_METRE = .052
SWING_LIMIT_DEGREES = 36.
INITIAL_HEIGHT = AXIS[2]+DRUM_RADIUS+CABLE_RADIUS+.0006
SUPPORT = Box('reel_mount',pos=(AXIS[0],AXIS[1],TOP+.008),size=(.224,.382,.016),
              mass=6.4,color=(.18,.23,.27))
REEL = Box('drum',pos=AXIS,size=(.132,2*FLANGE_RADIUS,2*FLANGE_RADIUS),
           mass=2.2,color=(.76,.37,.10))
GUIDE = Box('cable_guide',pos=(AXIS[0],AXIS[1]+.174,TOP+.107),
            size=(.104,.100,.214),mass=2.,color=(.26,.33,.37))
CABLE = [Box(f'cable_{i:02}',pos=(AXIS[0],AXIS[1]+(i+.5)*LINK_LENGTH,INITIAL_HEIGHT),
             size=(LINK_LENGTH,2*CABLE_RADIUS,2*CABLE_RADIUS),yaw=math.pi/2,
             mass=.047 if i==LINK_COUNT-1 else CABLE_MASS_PER_METRE*LINK_LENGTH,color=(.15,.22,.28))
         for i in range(LINK_COUNT)]
POWERED_REEL = True
GUIDE_CLOSED_RAD = math.radians(11.5)
GUIDE_PAD_LOCAL = (-.325,.252,-.114)
BUTTON_DEPTH = .0048
MOTOR_TARGET_DEGREES = 540.
MOTOR_RATE_DEG_S = 48.
RETAINER = Box('guide_retainer',pos=(.618,GUIDE.pos[1],TOP+.284),size=(.348,.270,.110),
               mass=.082,color=(.76,.56,.24))
SWITCH_BOX = Box('reel_switch_box',pos=(.265,-.164,TOP+.011),size=(.062,.064,.022),
                 mass=.35,color=(.20,.27,.33))
SWITCH = Box('motor_start_button',pos=(*SWITCH_BOX.pos[:2],TOP+.038),size=(.026,.026,.012),
             mass=.022,color=(.83,.29,.14))
OBJECTS = [SUPPORT,REEL,GUIDE,RETAINER,SWITCH_BOX,SWITCH,*CABLE]

BODY_PARTS = {
    SUPPORT.name: (
        Solid('box',SUPPORT.size),
        *(Solid('box',(.022,.078,.194),(x,0.,.103)) for x in (-.091,.091)),
        *(Solid('cylinder',(.021,.024),(x,0.,AXIS[2]-SUPPORT.pos[2]),axis='X')
          for x in (-.089,.089)),
    ),
    REEL.name: (
        Solid('cylinder',(DRUM_RADIUS,DRUM_WIDTH),axis='X'),
        *(Solid('cylinder',(FLANGE_RADIUS,.005),(x,0.,0.),axis='X',color=(.25,.30,.34))
          for x in (-.0635,.0635)),
        Solid('cylinder',(.012,.220),(-.025,0.,0.),axis='X',color=(.51,.55,.60)),
    ),
    RETAINER.name:(
        Solid('cylinder',(.009,.044),(-.048,0.,.005),axis='X',color=(.58,.64,.69)),
        Solid('cylinder',(.007,.071),axis='Y'),
        Solid('box',(.190,.017,.012),(-.090,0.,.024)),
        Solid('box',(.015,.264,.012),(-.165,.125,.024)),
        Solid('box',(.014,.021,.138),(-.165,.252,-.045)),
        Solid('box',(.170,.025,.010),(-.245,.252,GUIDE_PAD_LOCAL[2])),
        Solid('box',(.022,.025,.021),GUIDE_PAD_LOCAL),
    ),
    GUIDE.name: (
        Solid('box',(.104,.100,.012),(0.,0.,-.101)),
        *(Solid('box',(.014,.060,.208),(x,0.,.004)) for x in (-.039,.039)),
        Solid('cylinder',(.030,.065),(0.,0.,.115),axis='X',color=(.53,.58,.63)),
        *(Solid('box',(.008,.038,.056),(x,0.,.137)) for x in (-.032,.032)),
    ),
    **{link.name:(Solid('capsule_x',(CABLE_RADIUS,LINK_LENGTH-2*CABLE_RADIUS)),) for link in CABLE},
}
BODY_PARTS[CABLE[-1].name] += (Solid('box',(.019,.015,.012),color=(.24,.33,.39)),)
OBJECT_JOINTS = {
    REEL.name:{'kind':'revolute','parent':SUPPORT.name,'anchor':AXIS,'axis':'X',
               'drive':{'stiffness':.18,'damping':.009,'max_force':1.0}},
    RETAINER.name:{'kind':'revolute','parent':GUIDE.name,'anchor':RETAINER.pos,'axis':'Y',
                   'lower':-15.,'upper':0.,
                   'drive':{'stiffness':6.,'damping':.035,'max_force':.55,'target':8.}},
    SWITCH.name:{'kind':'prismatic','parent':SWITCH_BOX.name,'anchor':SWITCH.pos,'axis':'Z',
                 'lower':-.008,'upper':0.,
                 'drive':{'stiffness':180.,'damping':.8,'max_force':4.,'target':.002}},
    **{link.name:{'kind':'spherical','parent':REEL.name if i==0 else CABLE[i-1].name,
                 'anchor':(AXIS[0],AXIS[1]+i*LINK_LENGTH,INITIAL_HEIGHT),
                 'axis':'Y' if i==0 else 'X','swing_limit_degrees':SWING_LIMIT_DEGREES,
                 'collision_enabled':i==0}
       for i,link in enumerate(CABLE)},
}
SUPPORT_JOINTS = {body.name:{'kind':'fixed'} for body in (GUIDE,SWITCH_BOX)}
CONTACT_OFFSETS_M = {body.name:.00055 for body in (REEL,GUIDE,RETAINER,SWITCH,*CABLE)}
RIGID_SOLVER_ITERATIONS = {body.name:(80,4) for body in (SUPPORT,REEL,RETAINER,SWITCH,*CABLE)}
OBJECT_MATERIALS = {SUPPORT.name:'steel reel stand',REEL.name:'aluminum drum and steel spindle',
                    GUIDE.name:'aluminum cable guide with steel roller',RETAINER.name:'steel hinged guide with polymer press pad',
                    SWITCH_BOX.name:'polymer switch housing',SWITCH.name:'polymer cap and steel switch spring',
                    **{link.name:'polymer-insulated cable' for link in CABLE}}
OBJECT_MATERIALS[CABLE[-1].name] = 'polymer-insulated cable with molded metal-weighted end plug'
LIVE_CONTEXT = {}
SUCCESS_HOLD_S = .60
MAX_PARTIAL_SCORE = .55
_previous_angle = None
_drum_travel = 0.
_maximum_stretch = 0.
_under_guide = 0.
_score = 0.
_opened_at_reset = False
_motor_active = False
_motor_demand = 0.
_motor_drive = None


def on_reset():
    global _previous_angle, _drum_travel, _maximum_stretch, _under_guide, _score, _opened_at_reset
    global _motor_active, _motor_demand, _motor_drive
    _previous_angle = None
    _drum_travel = _maximum_stretch = _under_guide = _score = 0.
    _opened_at_reset = False
    _motor_active = False
    _motor_demand = 0.
    _motor_drive = None


def cable_state(origins):
    offsets = np.asarray([origins[link.name] for link in CABLE])-origins[REEL.name]
    radial = np.linalg.norm(offsets[:,1:],axis=1)
    near_drum = ((abs(offsets[:,0])<DRUM_WIDTH/2-CABLE_RADIUS)
                 & (radial>DRUM_RADIUS+.001) & (radial<DRUM_RADIUS+.015))
    outside = np.flatnonzero(~near_drum)
    count = int(outside[0]) if len(outside) else len(offsets)
    angles = np.unwrap(np.arctan2(offsets[:count,2],offsets[:count,1]))
    turns = 0. if count<2 else abs(float(angles[-1]-angles[0]))/math.tau
    stretch = float(np.max(np.linalg.norm(np.diff(offsets,axis=0),axis=1)))-LINK_LENGTH
    return turns,stretch,count



def observe(objects, context, dt):
    global _previous_angle, _drum_travel, _maximum_stretch, _under_guide, _score, _opened_at_reset
    global _motor_active, _motor_demand
    q=objects[REEL.name].quat_xyzw
    angle=2*math.atan2(q[0],q[3])
    if _previous_angle is not None:
        delta=angle-_previous_angle
        _drum_travel+=math.atan2(math.sin(delta),math.cos(delta))
    _previous_angle=angle
    origins=context['body_origins']
    wrapped,stretch,count=cable_state(origins)
    _maximum_stretch=max(_maximum_stretch,stretch)
    frame=quat_xyzw_to_matrix(np.asarray(objects[RETAINER.name].quat_xyzw))
    guide_angle=math.atan2(frame[0,2],frame[0,0])
    catch=origins[RETAINER.name]+frame@np.asarray(GUIDE_PAD_LOCAL)
    button=origins[SWITCH.name]
    depth=SWITCH.pos[2]-button[2]
    left,right=(context['tcp'][side] for side in ('left','right'))
    left_distance=math.dist(left,catch)
    _opened_at_reset |= context['time_s']<.8 and abs(guide_angle)<.035 and left_distance>.070
    guide_closed=_opened_at_reset and guide_angle<-GUIDE_CLOSED_RAD and left_distance<.042
    pressed=depth>BUTTON_DEPTH and math.dist(right,button)<.032
    _motor_active=bool(guide_closed and pressed and _maximum_stretch<=.004)
    angular_speed=objects[REEL.name].ang_vel[0]
    if _motor_active:
        _motor_demand=min(MOTOR_TARGET_DEGREES,_motor_demand+MOTOR_RATE_DEG_S*dt)
    if _motor_active and angular_speed>.10:
        _under_guide=max(_under_guide,wrapped)
    if _maximum_stretch>.004:
        _score=0.
        return
    released=(depth<.0015 and abs(guide_angle)<.035
              and all(math.dist(tcp,point)>.055 for tcp in (right,left) for point in (button,catch)))
    finished=(_under_guide>=1.10 and wrapped>=1.10 and count>=32 and released
              and abs(angular_speed)<.16)
    _score=1. if finished else .55 if _under_guide>=.75 else .20 if _under_guide>=.21 else 0.


def update_materials():
    global _motor_drive
    if _motor_drive is None:
        import omni.usd
        from pxr import UsdPhysics
        stage=omni.usd.get_context().get_stage()
        joint,=(prim for prim in stage.Traverse() if prim.GetName()==REEL.name+'_joint')
        _motor_drive=UsdPhysics.DriveAPI(joint,'angular')
    current=math.degrees(_previous_angle) if _previous_angle is not None else 0.
    error=_motor_demand-math.degrees(_drum_travel)
    lead=min(max(error,-24.),24.) if _motor_active else 0.
    _motor_drive.GetTargetPositionAttr().Set(current+lead)


def success(objects):
    return _score==1.


def score(objects):
    return _score


def demo():
    opened=np.asarray(RETAINER.pos)+GUIDE_PAD_LOCAL
    angle=-.250
    sine,cosine=math.sin(angle),math.cos(angle)
    x,y,z=GUIDE_PAD_LOCAL
    closed=np.asarray(RETAINER.pos)+np.array((x*cosine+z*sine,y,z*cosine-x*sine))
    left_hover=tuple(opened+(0.,0.,.029))
    left_touch=tuple(opened+(0.,0.,PAD+.009))
    press=tuple(closed+(0.,0.,PAD+.003))
    left_clear=(.224,.219,TOP+.174)
    right_hover=(*SWITCH.pos[:2],SWITCH.pos[2]+.046)
    right_press=(*SWITCH.pos[:2],SWITCH.pos[2]+.006)
    right_clear=(.222,-.226,TOP+.151)
    return {
        'right':[
            Waypoint(2.25,right_hover,CLOSED,'approach the motor switch'),
            Waypoint(2.70,right_hover,CLOSED,'wait for the cable guide'),
            Waypoint(.80,right_press,CLOSED,'press the motor switch'),
            Waypoint(15.20,right_press,CLOSED,'hold to wind the cable'),
            Waypoint(.85,right_hover,CLOSED,'release the motor switch'),
            Waypoint(1.30,right_clear,OPEN,'clear the motor switch'),
            Waypoint(5.,right_clear,OPEN,'watch the reel settle'),
        ],
        'left':[
            Waypoint(2.25,left_hover,CLOSED,'approach the hinged guide'),
            Waypoint(1.15,left_touch,CLOSED,'touch the guide handle'),
            Waypoint(1.50,press,CLOSED,'close the guide onto the cable'),
            Waypoint(17.05,press,CLOSED,'keep the cable guided while winding'),
            Waypoint(1.,left_hover,CLOSED,'release the cable guide'),
            Waypoint(1.70,left_clear,OPEN,'clear the cable guide'),
            Waypoint(5.,left_clear,OPEN,'watch the reel settle'),
        ],
    }
