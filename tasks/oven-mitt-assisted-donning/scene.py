"""Open and align a loose mitt for an advancing hand and verify that it stays worn."""

from dataclasses import dataclass
import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_oven_mitt_assisted_donning.geometry import Solid
from dynamic_task_oven_mitt_assisted_donning.motion import PoseVelocity

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
RESET_X=.242
PRESENT_X=.271
MY=.012
START_MITT_Y=.035
YAW=-math.pi/2.
HAND_FRAME=axis_angle_matrix(np.array((0.,0.,1.)),YAW)
MITT_REST=TOP+.029
HAND_Z=TOP+.032
HAND_REST=.674
HAND_SEATED=.306
WITHDRAW=.128
SKIN=(.70,.455,.318)
CUFF=(.423,.337,.249)
SILICONE=(.733,.638,.476)
SEAM=(.511,.422,.330)
SHIRT=(.22,.34,.47)
LIP_EDGE_X=.056
LIP_Y=.111

MITT=Box('loose_left_silicone_oven_mitt',pos=(RESET_X,START_MITT_Y,MITT_REST),yaw=YAW,
         size=(.152,.274,.058),mass=.096,color=SILICONE)
HAND=Box('reaching_left_hand_and_forearm',pos=(HAND_REST,MY,HAND_Z),yaw=YAW,
         size=(.136,.298,.060),mass=.39,color=SKIN)
THUMB_PIVOT=np.array((.023,.003,-.008))
thumb_home=tuple(np.asarray(HAND.pos)+HAND_FRAME@THUMB_PIVOT)
THUMB=Box('articulated_left_thumb',pos=thumb_home,yaw=YAW,size=(.054,.068,.022),mass=.009,color=SKIN)
THUMB_TIP=np.array((.044,-.052,.001))
LIPS=tuple(Box(name,pos=(RESET_X+LIP_Y,START_MITT_Y-side*LIP_EDGE_X,MITT_REST+.002),yaw=YAW,
               size=(.027,.038,.051),mass=.006,color=CUFF)
           for side,name in ((-1,'cloth_cuff_near_edge'),(1,'cloth_cuff_far_edge')))
OBJECTS=[MITT,HAND,THUMB,*LIPS]


def link(a,b,r,color):
    a,b=np.asarray(a),np.asarray(b)
    dx,dy,dz=b-a
    return Solid('capsule_x',(r,max(.001,math.dist(a,b)-2*r)),tuple(map(float,(a+b)/2)),
                 yaw=math.atan2(dy,dx),pitch=-math.atan2(dz,math.hypot(dx,dy)),color=color)


def hand_mesh():
    parts=[link((0.,.031,0.),(0.,.211,-.006),.023,SHIRT),
           Solid('box',(.064,.075,.030),(0.,-.004,0.),color=SKIN)]
    for x,length in zip((-.027,-.010,.009,.027),(.077,.097,.103,.090)):
        parts.append(link((x,-.027,0.),(x,-length,.002),.0082,SKIN))
    return tuple(parts)


def mitt_shell():
    slabs=[]
    for s in (-1,1):
        z=s*.025
        slabs.extend((
          Solid('box',(.117,.162,.006),(0.,-.036,z),color=SILICONE),
          Solid('box',(.096,.037,.006),(0.,-.125,z),color=SILICONE),
          Solid('box',(.116,.050,.004),(0.,.070,z),color=CUFF),
          Solid('box',(.042,.087,.006),(.063,-.023,z),yaw=.27,color=SILICONE),
        ))
    slabs.extend((
      Solid('box',(.006,.204,.047),(-.057,-.015,0.),color=SEAM),
      Solid('box',(.006,.081,.047),(.057,-.088,0.),color=SEAM),
      Solid('box',(.006,.086,.047),(.086,-.026,0.),yaw=.27,color=SEAM),
      Solid('box',(.043,.007,.047),(.070,-.066,0.),yaw=.15,color=SEAM),
      Solid('box',(.101,.008,.050),(0.,-.144,0.),color=SEAM),
      Solid('box',(.043,.007,.047),(.070,.018,0.),color=SEAM),
      link((-.054,-.113,0.),(-.047,-.144,0.),.0085,SEAM),
      link((.053,-.112,0.),(.047,-.144,0.),.0085,SEAM),
    ))
    return tuple(slabs)


BODY_PARTS={
    MITT.name:mitt_shell(),
    HAND.name:hand_mesh(),
    THUMB.name:(link((0.,0.,0.),(.028,-.028,0.),.010,SKIN),link((.027,-.028,0.),THUMB_TIP,.0088,SKIN)),
    **{lip.name:(Solid('box',(.006,.036,.045),color=CUFF),
                 Solid('box',(.036,.013,.020),(0.,.012,.010),color=SEAM))
       for lip in LIPS},
}
VISUAL_PARTS={
    MITT.name:tuple(Solid('box',(.102,.0018,.0006),(0.,y,.0282),color=SEAM)
                    for y in np.linspace(-.113,.038,22)),
}
OBJECT_JOINTS={THUMB.name:{'kind':'revolute','parent':HAND.name,'axis':'Z','anchor':THUMB.pos,
                          'lower':-37.,'upper':3.,'drive':{'stiffness':.026,'damping':.0018,'target':-34.,'max_force':.066}}}
for side,lip in zip((-1,1),LIPS):
    lo,hi=(-.014,.002) if side==-1 else (-.002,.014)
    OBJECT_JOINTS[lip.name]={'kind':'prismatic','parent':MITT.name,'axis':'X','anchor':lip.pos,
                            'lower':lo,'upper':hi,'drive':{'stiffness':37.,'damping':1.0,'max_force':1.6}}
SURFACE_FRICTION={HAND.name:(.86,.72),MITT.name:(.95,.83),**{lip.name:(.98,.87) for lip in LIPS}}
CONTACT_OFFSETS_M={obj.name:.0006 for obj in OBJECTS}
RIGID_SOLVER_ITERATIONS={obj.name:(72,8) for obj in OBJECTS}
OBJECT_MATERIALS={MITT.name:'OXO Good Grips silicone oven mitt with sewn fabric lining',
                  HAND.name:'articulated-hand training surrogate with a sleeve', THUMB.name:'silicone mannequin thumb with a flexion joint',
                  **{lip.name:'flexible cloth cuff edge approximated as spring-supported strip' for lip in LIPS}}


def eased(u):
    u=min(1.,max(0.,u))
    return u*u*(3.-2.*u)


@dataclass(frozen=True)
class Reach:
    def at(self, seconds):
        insertion,forward=self.phase(seconds,4.7,8.85)
        withdrawal,back=self.phase(seconds,10.70,13.70)
        distance=HAND_REST-HAND_SEATED
        return PoseVelocity((HAND_REST-distance*insertion+WITHDRAW*withdrawal,MY,HAND_Z+.058*withdrawal),YAW,
                            (-distance*forward+WITHDRAW*back,0.,.058*back),(0.,0.,0.))

    @staticmethod
    def phase(seconds,start,end):
        u=min(1.,max(0.,(seconds-start)/(end-start)))
        return eased(u),6.*u*(1.-u)/(end-start)


MOTIONS={HAND.name:Reach()}
# The station puts a real collision mast 70 mm behind this camera's X position.
overhead=np.array((.520,-.375,1.82))
look=np.array((.419,.011,TOP+.047))-overhead
look/=np.linalg.norm(look)
right=np.cross(look,(0.,0.,1.));right/=np.linalg.norm(right)
TASK_TOP_CAMERA=Camera(tuple(overhead),tuple(matrix_to_quat_xyzw(np.column_stack((right,np.cross(right,look),-look)))),
                       'opengl',Lens.from_focal(19.5,640,352))
cuff_camera=np.array((.386,-.146,.540))
cuff_look=np.array((.066,.009,.014))-cuff_camera
cuff_look/=np.linalg.norm(cuff_look)
cuff_right=np.cross(cuff_look,(0.,0.,1.));cuff_right/=np.linalg.norm(cuff_right)
cuff_view=np.column_stack((cuff_right,np.cross(cuff_right,cuff_look),-cuff_look))
TASK_RECORD_CAMERA=(MITT.name,Camera(tuple(HAND_FRAME.T@cuff_camera),
                    tuple(matrix_to_quat_xyzw(HAND_FRAME.T@cuff_view)),
                    'opengl',Lens.from_focal(28.,640,352)))

LIVE_CONTEXT={}
LIVE_OBJECTS={}
SUCCESS_HOLD_S=.85
MAX_PARTIAL_SCORE=.50
_entered=False
_worn_from=None
_lost=False
_thumb_drive=None


def on_reset():
    global _entered,_worn_from,_lost
    _entered=False
    _worn_from=None
    _lost=False


def wearing(objects,context):
    mitt=np.asarray(context['body_origins'][MITT.name])
    hand=np.asarray(context['body_origins'][HAND.name])
    rotation=quat_xyzw_to_matrix(objects[MITT.name].quat_xyzw)
    fingers=rotation.T@(hand+HAND_FRAME@np.array((.006,-.098,.001))-mitt)
    thumb_body=np.asarray(context['body_origins'][THUMB.name])
    thumb_r=quat_xyzw_to_matrix(objects[THUMB.name].quat_xyzw)
    thumb=rotation.T@(thumb_body+thumb_r@THUMB_TIP-mitt)
    cuff=rotation.T@(hand+HAND_FRAME@np.array((0.,.086,0.))-mitt)
    entered=(abs(fingers[0])<.047 and -.144<fingers[1]<-.051 and abs(fingers[2])<.044)
    thumb_inside=.040<thumb[0]<.091 and -.078<thumb[1]<.033 and abs(thumb[2])<.045
    wrist_near=abs(cuff[0])<.042 and .064<cuff[1]<.164 and abs(cuff[2])<.046
    return entered,bool(entered and thumb_inside and wrist_near and rotation[2,2]>.90)


def grippers_clear(context):
    positions=context['body_origins']
    return all(math.dist(tcp,positions[MITT.name])>.125
               and all(math.dist(tcp,positions[lip.name])>.085 for lip in LIPS)
               for tcp in context['tcp'].values())


def observe(objects,context,dt):
    global _entered,_worn_from,_lost
    hand=context['body_origins'][HAND.name]
    mitt=context['body_origins'][MITT.name]
    entered,worn=wearing(objects,context)
    _entered |= entered and hand[0]<HAND_SEATED+.107
    clear=grippers_clear(context)
    if worn and clear and hand[0]<HAND_SEATED+.064 and mitt[2]>=TOP+.023 and _worn_from is None:
        _worn_from=hand[0]
    if _worn_from is not None and not (worn and clear):
        _lost=True


def update_actuators():
    global _thumb_drive
    if _thumb_drive is None:
        import omni.usd
        from pxr import UsdPhysics

        stage=omni.usd.get_context().get_stage()
        joint=next(prim for prim in stage.Traverse() if prim.GetName()==THUMB.name+'_joint')
        _thumb_drive=UsdPhysics.DriveAPI(joint,'angular')
    if 'body_origins' not in LIVE_CONTEXT:
        return
    positions=LIVE_CONTEXT['body_origins']
    penetration=positions[HAND.name][0]-positions[MITT.name][0]
    open_fraction=eased((.108-penetration)/.068)
    angle=-34.*(1.-open_fraction)
    _thumb_drive.GetTargetPositionAttr().Set(float(angle))


def success(objects):
    if _worn_from is None or _lost:
        return False
    hand=LIVE_CONTEXT['body_origins'][HAND.name]
    mitt=LIVE_CONTEXT['body_origins'][MITT.name]
    clear=grippers_clear(LIVE_CONTEXT)
    return bool(_entered and wearing(objects,LIVE_CONTEXT)[1] and clear
                and hand[0]-_worn_from>.080 and mitt[2]>TOP+.052)


def score(objects):
    _,worn=wearing(objects,LIVE_CONTEXT)
    return 1. if success(objects) else .50 if _entered and worn and not _lost else 0.


PRESENT_Z=MITT_REST-.003
PINCH=-.006
PITCH=axis_angle_matrix(np.array((0.,1.,0.)),-.30)
WRISTS={arm:PITCH@grip_rotation(yaw) for arm,yaw in (('left',-math.pi/2.),('right',math.pi/2.))}
PARK={'left':(.225,.287,TOP+.268),'right':(.203,-.253,TOP+.233)}


def demo():
    plans={}
    for i,arm in enumerate(('left','right')):
        lip=LIPS[i]
        pickup=np.asarray(lip.pos)+(.010,-.012 if i==0 else .012,.010)
        mouth=np.array((PRESENT_X+LIP_Y+.011,pickup[1]+MY-START_MITT_Y+(.008 if i==0 else -.018),PRESENT_Z+.014))
        plans[arm]=[
            Waypoint(1.65,tuple(pickup+(0.,0.,.104)),FINGER_OPEN,'approach the two cloth edges'),
            Waypoint(.91,tuple(pickup),-.023,'lower over the two cuff edges'),
            Waypoint(.73,tuple(pickup),PINCH,'grip both sides of the cuff'),
            Waypoint(1.62,tuple(mouth),PINCH,'slide the mitt square to the moving hand'),
            Waypoint(4.45,tuple(mouth),PINCH,'align the thumb and hold for insertion'),
            Waypoint(.48,tuple(mouth),FINGER_OPEN,'release the cuff when the hand is seated'),
            Waypoint(.86,PARK[arm],FINGER_OPEN,'clear the moving wrist'),
            Waypoint(4.4,PARK[arm],FINGER_OPEN,'observe the worn mitt on the retreating hand'),
        ]
    return plans


def demo_target(arm,phase,time,position,rotation,finger):
    if phase in {'lower over the two cuff edges','grip both sides of the cuff'} and LIVE_CONTEXT:
        lip=LIPS[0] if arm=='left' else LIPS[1]
        exact=np.asarray(LIVE_CONTEXT['body_origins'][lip.name])+(.009,-.019 if arm=='left' else .019,.0085)
        blend=eased((time-1.65)/.91)
        position=np.asarray(position)+blend*(exact-(np.asarray(lip.pos)+(.010,-.012 if arm=='left' else .012,.010)))
    if phase=='align the thumb and hold for insertion' and LIVE_CONTEXT:
        body=np.asarray(LIVE_CONTEXT['body_origins'][MITT.name])
        t=eased((time-4.85)/1.20)
        error=np.asarray((PRESENT_X,MY,MITT_REST))-body
        correction=np.clip(error,(-.024,-.023,-.006),(.039,.023,.006))
        position=np.asarray(position)+t*.79*correction
    return position,WRISTS[arm],finger
