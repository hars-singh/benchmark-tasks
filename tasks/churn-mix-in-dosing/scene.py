"""Add solid mix-ins through the lid while a freezer bowl is turning."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw, slerp_matrix
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_churn_mix_in_dosing.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.385,-.025
R=.083
BED_Z=TOP+.111
LID_Z=TOP+.179
OPENING=.048
CHIP_COUNT=18
CHIP_RADIUS=.0048
ICE=(.891,.838,.715)
WHITE=(.84,.86,.83)
LID=(.80,.88,.91)
CHOCOLATE=(.245,.112,.078)
CUP_COLOR=(.22,.45,.59)
BASE=Box('cuisinart_maker_with_locked_lid',pos=(X,Y,TOP+.078),size=(.224,.224,.161),
         mass=2.35,color=WHITE)
BOWL=Box('turning_one_point_five_quart_freezer_bowl',pos=(X,Y,TOP+.073),size=(.176,.176,.132),
         mass=1.37,color=WHITE)
SCOOP=Box('quarter_cup_measure',pos=(.276,-.237,TOP+.031),size=(.151,.067,.036),
          mass=.031,color=CUP_COLOR)
MAT=Box('measuring_cup_rest',pos=(.273,-.237,TOP+.0025),size=(.189,.084,.005),
        color=(.21,.29,.31))
SCOOP_BASIN=np.array((.031,0.,-.003))
SCOOP_GRIP=np.array((-.035,0.,.004))

CHIPS=tuple(Box(f'chocolate_chip_{i:02d}',pos=(SCOOP.pos[0]+SCOOP_BASIN[0]+.010*(i%4-1.5),
                  SCOOP.pos[1]+.010*((i//4)%4-1.5),SCOOP.pos[2]-.012+.010*(i//16)),
                size=(2.*CHIP_RADIUS,)*3,mass=.00066,color=CHOCOLATE)
            for i in range(CHIP_COUNT))
OBJECTS=[BASE,BOWL,SCOOP,MAT,*CHIPS]


def wall(inner,outer,low_z,high_z,count,color,opacity=None):
    height=high_z-low_z
    return tuple(Solid('tapered_panel',(inner,inner,outer-inner,height,math.tau/count),
                       (0.,0.,(low_z+high_z)/2.),roll=math.tau*i/count,pitch=-math.pi/2.,
                       color=color,opacity=opacity) for i in range(count))


body_z=BASE.pos[2]
bowl_z=BOWL.pos[2]
BODY_PARTS={
    BASE.name:(
        Solid('cylinder',(.105,.040),(0.,0.,TOP+.020-body_z),color=WHITE),
        *wall(.093,.106,TOP+.038-body_z,TOP+.142-body_z,24,WHITE),
        *wall(.087,.106,TOP+.142-body_z,TOP+.163-body_z,24,LID,.07),
        *wall(OPENING,.105,TOP+.162-body_z,TOP+.166-body_z,24,LID,.06),
        *wall(OPENING,OPENING+.0035,TOP+.164-body_z,LID_Z-body_z,24,LID,.12),
        Solid('box',(.018,.009,.021),(-.108,0.,-.009),color=(.072,.11,.13)),
        Solid('capsule_x',(.008,.055),(0.,.064,TOP+.145-body_z),color=WHITE),
        Solid('box',(.011,.012,.030),(.003,.057,BED_Z+.0195-body_z),yaw=-.45,color=WHITE),
        Solid('box',(.013,.030,.023),(.015,.045,BED_Z+.0155-body_z),yaw=.48,color=WHITE),
    ),
    BOWL.name:(
        Solid('cylinder',(.087,.024),(0.,0.,TOP+.054-bowl_z),color=(.65,.69,.70)),
        *wall(R,.088,TOP+.060-bowl_z,TOP+.139-bowl_z,24,(.62,.68,.70)),
        Solid('cylinder',(R-.001,BED_Z-TOP-.063),(0.,0.,(TOP+.063+BED_Z)/2.-bowl_z),color=ICE),
    ),
    SCOOP.name:(
        *(Solid(part.kind,part.size,tuple(np.asarray(part.offset)+SCOOP_BASIN),
                 roll=part.roll,pitch=part.pitch,color=CUP_COLOR)
          for part in wall(.030,.032,-.016,.018,16,CUP_COLOR)),
        Solid('cylinder',(.031,.004),(.031,0.,-.020),color=CUP_COLOR),
        Solid('capsule_x',(.0097,.062),(-.037,0.,.0),color=CUP_COLOR),
    ),
    **{chip.name:(Solid('sphere',(CHIP_RADIUS,),color=CHOCOLATE),) for chip in CHIPS},
}
VISUAL_PARTS={
    BOWL.name:(Solid('box',(.018,.006,.004),(.084,0.,TOP+.141-bowl_z),color=(.82,.29,.08)),),
    BASE.name:(Solid('box',(.001,.012,.009),(-.109,0.,-.025),color=(.38,.62,.46)),),
}
SUPPORT_JOINTS={BASE.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    BOWL.name:{'kind':'revolute','parent':BASE.name,'axis':'Z','anchor':BOWL.pos,
               'drive':{'damping':.13,'max_force':.39,'target_velocity':198.}},
}
SURFACE_FRICTION={BOWL.name:(.84,.69),SCOOP.name:(.76,.56),
                  **{o.name:(.50,.37) for o in CHIPS}}
CONTACT_OFFSETS_M={o.name:.00038 for o in (BASE,BOWL,SCOOP,*CHIPS)}
RIGID_SOLVER_ITERATIONS={o.name:(48,8) for o in (BOWL,SCOOP,*CHIPS)}
OBJECT_MATERIALS={BASE.name:'original-size ABS base, clear lid and plastic dasher',
                  BOWL.name:'aluminium freezer bowl holding a thick vanilla base',
                  SCOOP.name:'ordinary quarter-cup measure with molded handle',
                  **{o.name:'food-safe chocolate mix-in' for o in CHIPS}}

camera=np.array((-.125,.107,.337))
look=np.array((-.020,-.009,.085))-camera
look/=np.linalg.norm(look)
right=np.cross(look,(0.,0.,1.))
right/=np.linalg.norm(right)
rotation=matrix_to_quat_xyzw(np.column_stack((right,np.cross(right,look),-look)))
TASK_RECORD_CAMERA=(BASE.name,Camera(tuple(camera),tuple(rotation),'opengl',Lens.from_focal(25.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
_delivered=set()
_turn_since_dose=0.
_previous=None
_invalid=False
_parked=False
_sectors=0
_present=0
_grasp_offset=None
_grasp_rotation=None


def on_reset():
    global _turn_since_dose,_previous,_invalid,_parked,_sectors,_present
    global _grasp_offset,_grasp_rotation
    _delivered.clear()
    _turn_since_dose=0.
    _previous=None
    _invalid=_parked=False
    _sectors=0
    _present=0
    _grasp_offset=_grasp_rotation=None


def observe(objects,context,dt):
    global _turn_since_dose,_previous,_invalid,_parked,_sectors,_present
    origins=context['body_origins']
    q=objects[BOWL.name].quat_xyzw
    angle=2.*math.atan2(q[2],q[3])
    delta=0. if _previous is None else math.atan2(math.sin(angle-_previous),math.cos(angle-_previous))
    _previous=angle
    points=np.asarray([origins[o.name] for o in CHIPS])
    relative=points-np.array((X,Y,BED_Z))
    radial=np.linalg.norm(relative[:,:2],axis=1)
    as_mixed=(radial<R-.004) & (relative[:,2]>.001) & (relative[:,2]<.032)
    _present=np.count_nonzero(as_mixed)
    for i in range(len(points)):
        if as_mixed[i] and abs(objects[BOWL.name].ang_vel[2])>1.1:
            _delivered.add(i)
    scoop=origins[SCOOP.name]
    active_radius=np.linalg.norm(np.asarray(scoop[:2])-(X,Y))
    inserted=active_radius<.084 and objects[SCOOP.name].low[2]<LID_Z-.002
    _invalid |= bool(inserted or any(relative[i,2]<-.045 for i in _delivered))
    angles=np.arctan2(relative[as_mixed,1],relative[as_mixed,0])
    _sectors=len(set(np.floor((angles+math.pi)/(math.pi/2.)).astype(int)))
    on_rest=(math.dist(scoop[:2],MAT.pos[:2])<.030 and TOP+.013<scoop[2]<TOP+.037
             and quat_xyzw_to_matrix(objects[SCOOP.name].quat_xyzw)[2,2]>.940)
    _parked=on_rest and all(math.dist((X,Y,LID_Z),tcp)>.115 for tcp in context['tcp'].values())
    if _parked and _present==CHIP_COUNT:
        _turn_since_dose+=max(0.,delta)
    else:
        _turn_since_dose=0.


def success(objects):
    return bool(not _invalid and len(_delivered)==CHIP_COUNT and _present==CHIP_COUNT and _sectors>=3
                and _parked and _turn_since_dose>=2.*math.tau)


def score(objects):
    return 0. if _invalid else 1. if success(objects) else .5 if len(_delivered)>=CHIP_COUNT//2 else 0.


PICK=np.asarray(SCOOP.pos)+SCOOP_GRIP+(0.,0.,.001)
HOVER=PICK+(0.,0.,.070)
ABOVE=(X-.078,Y,LID_Z+.080)
HOLD=(X-.068,Y,LID_Z+.078)
POUR_START=5.30
PINCH=0.


def demo():
    return {'right':[
        Waypoint(1.16,tuple(HOVER),FINGER_OPEN,'approach the measure handle'),
        Waypoint(.76,tuple(PICK),FINGER_OPEN,'reach the loaded measure'),
        Waypoint(.78,tuple(PICK),PINCH,'grip the level measure'),
        Waypoint(.91,tuple(PICK+(0.,0.,.135)),PINCH,'lift without spilling the chips'),
        Waypoint(1.69,ABOVE,PINCH,'bring the cup over the locked lid'),
        Waypoint(1.90,HOLD,PINCH,'tip the mix-ins through the spout'),
        Waypoint(1.20,HOLD,PINCH,'finish dispensing the measured chips'),
        Waypoint(1.45,ABOVE,PINCH,'return the empty measure upright'),
        Waypoint(1.32,tuple(HOVER),PINCH,'withdraw the measure while the bowl turns'),
        Waypoint(.73,tuple(PICK),PINCH,'set the measure on its rest'),
        Waypoint(.68,tuple(PICK),FINGER_OPEN,'release the empty measure'),
        Waypoint(1.0,(.237,-.264,TOP+.151),FINGER_OPEN,'let the bowl mix with grippers clear'),
        Waypoint(5.,(.237,-.264,TOP+.151),FINGER_OPEN,'show the turning bowl'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    global _grasp_offset,_grasp_rotation
    pitch=axis_angle_matrix(np.array((0.,1.,0.)),-.47)
    ambient=pitch@grip_rotation()
    if 3.65<t<5.1 and 'tcp_rotation' in LIVE_CONTEXT:
        body=np.asarray(LIVE_CONTEXT['body_origins'][SCOOP.name])
        if body[2]>TOP+.097:
            body_rotation=quat_xyzw_to_matrix(LIVE_OBJECTS[SCOOP.name].quat_xyzw)
            _grasp_offset=body_rotation.T@(np.asarray(LIVE_CONTEXT['tcp']['right'])-body)
            _grasp_rotation=body_rotation.T@LIVE_CONTEXT['tcp_rotation']['right']
    if phase in {'reach the loaded measure','grip the level measure'} and 'body_origins' in LIVE_CONTEXT:
        actual=np.asarray(LIVE_CONTEXT['body_origins'][SCOOP.name])+SCOOP_GRIP+(0.,0.,.0035)
        weight=1. if phase=='grip the level measure' else min(1.,max(0.,(t-1.16)/.76))
        position=np.asarray(position)*(1.-weight)+actual*weight
    if arm=='right' and POUR_START<=t<=POUR_START+4.55:
        up=min(1.,max(0.,(t-POUR_START)/1.90))
        down=min(1.,max(0.,(t-POUR_START-3.10)/1.45))
        down=down*down*(3.-2.*down)
        tilt=axis_angle_matrix(np.array((1.,0.,0.)),1.84*(up-down))
        if _grasp_offset is not None:
            lip=np.array((SCOOP_BASIN[0],-.031,.0140))
            ideal=np.array((X+.002,Y+.006,LID_Z+.063))+tilt@(_grasp_offset-lip)
            weight=min(1.,max(0.,(t-POUR_START)/.58))
            weight*=1.-down
            orientation=tilt@_grasp_rotation
            if down>0.:
                # Rotate back gradually before moving the empty cup away from the lid.
                orientation=tilt@slerp_matrix(_grasp_rotation,ambient,down)
            return np.asarray(position)*(1.-weight)+ideal*weight,orientation,finger
        return position,tilt@ambient,finger
    return position,ambient,finger
