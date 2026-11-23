"""Seat a Dynamixel horn at its index and watch both unloaded test swings."""

from dataclasses import dataclass
import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw,slerp_matrix
from yam_arena.rig import Camera,Lens
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from dynamic_task_servo_horn_neutral_check.geometry import Solid
from dynamic_task_servo_horn_neutral_check.motion import PoseVelocity

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.360,-.174
SURFACE=TOP+.047
HORN_SEATED_Z=SURFACE+.0075
HORN_R=.0128
MALE_ROOT=.00405
TEETH=24
PITCH=math.tau/TEETH
HORN_RESET_YAW=math.radians(24.)
START_TEST=13.2
END_TEST=19.2
ANGLE=math.radians(25.)
BLACK=(.075,.082,.095)
METAL=(.52,.55,.58)
LIGHT=(.78,.79,.76)
MARK=(.85,.45,.10)
BASE=Box('mx28ar_servo_case',pos=(X+.008,Y,TOP+.027),size=(.0506,.0356,.0355),mass=.078,color=BLACK)
SHAFT=Box('indexed_output_wheel_gear',pos=(X,Y,SURFACE+.0017),size=(.026,.026,.0065),mass=.018,color=METAL)
MAT=Box('parts_mat',pos=(.250,-.265,TOP+.004),size=(.086,.084,.008),color=(.13,.32,.28))
HORN=Box('hn07_n101_output_horn',pos=(MAT.pos[0],MAT.pos[1],TOP+.008+.0084),size=(.026,.026,.011),
         mass=.015,yaw=HORN_RESET_YAW,color=LIGHT)
WITNESS=Box('witness_angle_card',pos=(X,Y,TOP+.009),size=(.083,.078,.004),color=(.78,.81,.77))
OBJECTS=[BASE,SHAFT,HORN,MAT,WITNESS]


def sector(inner,outer,span,angle,height,z,color):
    return Solid('tapered_panel',(inner,inner,outer-inner,height,span),(0.,0.,z),
                 roll=angle,pitch=-math.pi/2,color=color)


def annulus(inner,outer,height,z,color):
    return tuple(sector(inner,outer,PITCH,i*PITCH,height,z,color) for i in range(TEETH))


def tooth_pattern(inner,outer,height,z,phase,color):
    return tuple(sector(inner,outer,PITCH*.34,i*PITCH+phase,height,z,color) for i in range(TEETH))


BODY_PARTS={
    BASE.name:(
        Solid('box',(.068,.056,.015),(0.,0.,TOP+.0075-BASE.pos[2]),color=METAL),
        Solid('box',(.0506,.0356,.028),(0.,0.,.002),color=BLACK),
        Solid('box',(.015,.0356,.007),(.0178,0.,.0148),color=BLACK),
        Solid('box',(.0044,.0356,.007),(-.0231,0.,.0148),color=BLACK),
        *(Solid('box',(.027,.0036,.007),(-.008,s*.016,.0148),color=BLACK) for s in (-1,1)),
        Solid('box',(.003,.018,.010),(.0258,0.,-.007),color=(.23,.26,.20)),
    ),
    SHAFT.name:(
        *annulus(.00498,.0110,.0042,-.0013,METAL),
        *annulus(.00527,.0110,.0012,.00140,METAL),
        *tooth_pattern(.00421,.00492,.0041,-.0013,PITCH/2.0,METAL),
        *annulus(.0092,.0128,.0010,.0036,BLACK),
    ),
    HORN.name:(
        *annulus(.00190,HORN_R,.0036,0.,LIGHT),
        *annulus(.00190,MALE_ROOT,.0049,-.00425,METAL),
        *annulus(.00190,.00372,.00115,-.00725,METAL),
        *tooth_pattern(MALE_ROOT,.00456,.0038,-.00382,0.,METAL),
    ),
}
VISUAL_PARTS={
    BASE.name:(Solid('box',(.020,.012,.0003),(.013,0.,.019),color=(.28,.30,.31)),),
    SHAFT.name:(Solid('cylinder',(.00075,.00015),(.0108,0.,.00418),color=MARK),),
    HORN.name:(
        Solid('cylinder',(.00134,.00024),(.0106,0.,.00194),color=MARK),
        Solid('box',(.0040,.0011,.00024),(.0079,0.,.00194),color=MARK),
        *(Solid('cylinder',(.00086,.00015),(.0093*math.cos(a),.0093*math.sin(a),.00190),color=BLACK)
          for a in (i*math.tau/8. for i in range(8)) if abs(a)>.1),
    ),
    WITNESS.name:tuple(Solid('box',(.011,.0012,.00024),(.033*math.cos(a),.033*math.sin(a),.0022),
                             yaw=a,color=MARK) for a in (-ANGLE,0.,ANGLE)),
}
OBJECT_MATERIALS={BASE.name:'ROBOTIS MX-28AR aluminium-front servo housing and mounting foot',
                  SHAFT.name:'native steel female wheel-gear serration and stock washer',
                  HORN.name:'ROBOTIS HN07-N101 horn and male serrated shaft',
                  MAT.name:'parts mat',WITNESS.name:'three-angle visual witness card'}
SUPPORT_JOINTS={BASE.name:{'kind':'fixed'}}
OBJECT_JOINTS={SHAFT.name:{'kind':'revolute','parent':BASE.name,'axis':'Z','anchor':SHAFT.pos,
                           'lower':-31.,'upper':31.,
                           'drive':{'stiffness':.029,'damping':.008,'max_force':.036}}}
SURFACE_FRICTION={HORN.name:(.64,.51),SHAFT.name:(.42,.31)}
CONTACT_OFFSETS_M={SHAFT.name:.00010,HORN.name:.000074}
RIGID_SOLVER_ITERATIONS={SHAFT.name:(72,8),HORN.name:(72,8)}


@dataclass(frozen=True)
class UnloadedAxisCheck:
    center: tuple[float,float,float]

    def at(self,t):
        targets=((START_TEST,0.),(START_TEST+1.5,ANGLE),(START_TEST+2.4,ANGLE),
                 (START_TEST+4.3,-ANGLE),(START_TEST+4.8,-ANGLE),(END_TEST,0.))
        angle=speed=0.
        for (begin,a),(end,b) in zip(targets,targets[1:]):
            if begin<=t<=end:
                u=(t-begin)/(end-begin)
                angle=a+(b-a)*u*u*(3.-2.*u)
                speed=(b-a)*6.*u*(1.-u)/(end-begin)
                break
        return PoseVelocity(self.center,angle,(0.,0.,0.),(0.,0.,speed))


TEST_SWING=UnloadedAxisCheck(SHAFT.pos)
camera_position=np.array((-.108,.112,.194))
camera_forward=np.array((-.006,0.,.014))-camera_position
camera_forward/=np.linalg.norm(camera_forward)
camera_right=np.cross(camera_forward,(0.,0.,1.));camera_right/=np.linalg.norm(camera_right)
camera_rot=np.column_stack((camera_right,np.cross(camera_right,camera_forward),-camera_forward))
TASK_RECORD_CAMERA=(BASE.name,Camera(tuple(camera_position),tuple(matrix_to_quat_xyzw(camera_rot)),
                                      'opengl',Lens.from_focal(38.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
_installed=False
_missed=False
_positive=_negative=False
_grasp_local=np.array((0.,0.,.009))
_grasp_rotation=None
_fit_trim=np.zeros(2)
_shaft_drive=None
_retainer_joint=None
_retained=False


def on_reset():
    global _installed,_missed,_positive,_negative,_grasp_rotation,_retained,_retainer_joint
    if _retainer_joint is not None:
        _retainer_joint.GetPrim().GetStage().RemovePrim(_retainer_joint.GetPath())
        _retainer_joint=None
    _installed=_missed=_positive=_negative=_retained=False
    _grasp_local[:]=(0.,0.,.009)
    _grasp_rotation=None
    _fit_trim.fill(0.)


def yaw_from_state(o):
    r=quat_xyzw_to_matrix(o.quat_xyzw)
    return math.atan2(r[1,0],r[0,0])


def fit(objects,context):
    horn=context['body_origins'][HORN.name]
    diff=yaw_from_state(objects[HORN.name])-yaw_from_state(objects[SHAFT.name])
    error=math.atan2(math.sin(diff),math.cos(diff))
    return bool(math.dist(horn[:2],(X,Y))<.00075 and abs(horn[2]-HORN_SEATED_Z)<.00065
                and abs(error)<=math.radians(5.)
                and quat_xyzw_to_matrix(objects[HORN.name].quat_xyzw)[2,2]>.993)


def observe(objects,context,dt):
    global _installed,_missed,_positive,_negative
    time=context.get('time_s',0.)
    if time<START_TEST:
        _installed=_installed or fit(objects,context)
    if START_TEST<=time<=END_TEST+.2:
        safe_distance=min(math.dist(SHAFT.pos,p) for p in context['tcp'].values())
        _missed=_missed or not fit(objects,context) or safe_distance<.085
        actual=yaw_from_state(objects[SHAFT.name])
        _positive=_positive or actual>=math.radians(22.)
        _negative=_negative or actual<=-math.radians(22.)


def update_actuators():
    global _shaft_drive,_retainer_joint,_retained
    from pxr import Gf, UsdPhysics

    if _shaft_drive is None:
        import omni.usd
        stage=omni.usd.get_context().get_stage()
        joints={prim.GetName():prim for prim in stage.Traverse() if prim.GetName().endswith('_joint')}
        _shaft_drive=UsdPhysics.DriveAPI(joints[SHAFT.name+'_joint'],'angular')
    time=LIVE_CONTEXT['time_s']
    hand_clear=all(math.dist(SHAFT.pos,p)>.085 for p in LIVE_CONTEXT['tcp'].values())
    if (START_TEST-.55<=time<START_TEST and not _retained and _installed and hand_clear
            and fit(LIVE_OBJECTS,LIVE_CONTEXT)):
        # The examiner installs the stock retainer before the unloaded check.
        # Lock the measured fit without moving or reindexing the horn.
        pos=LIVE_CONTEXT['body_origins']
        horn_r=quat_xyzw_to_matrix(LIVE_OBJECTS[HORN.name].quat_xyzw)
        gear_r=quat_xyzw_to_matrix(LIVE_OBJECTS[SHAFT.name].quat_xyzw)
        offset=horn_r.T@(np.asarray(pos[SHAFT.name])-pos[HORN.name])
        q=matrix_to_quat_xyzw(horn_r.T@gear_r)
        stage=_shaft_drive.GetPrim().GetStage()
        bodies={prim.GetName():prim for prim in stage.Traverse() if prim.GetName() in {SHAFT.name,HORN.name}}
        horn_prim=bodies[HORN.name].GetPath()
        _retainer_joint=UsdPhysics.FixedJoint.Define(stage,str(horn_prim)+'_observer_retainer')
        _retainer_joint.CreateBody0Rel().SetTargets([bodies[SHAFT.name].GetPath()])
        _retainer_joint.CreateBody1Rel().SetTargets([horn_prim])
        _retainer_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.,0.,0.))
        _retainer_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.))
        _retainer_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float,offset)))
        _retainer_joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(q[3]),*map(float,q[:3])))
        _retainer_joint.CreateCollisionEnabledAttr().Set(False)
        _retainer_joint.CreateBreakForceAttr().Set(20.)
        _retainer_joint.CreateBreakTorqueAttr().Set(.082)
        _retained=True
    target=TEST_SWING.at(time).yaw
    _shaft_drive.GetTargetPositionAttr().Set(math.degrees(target))


def success(objects):
    return bool(_installed and not _missed and _positive and _negative and fit(objects,LIVE_CONTEXT)
                and LIVE_CONTEXT.get('time_s',0.)>=END_TEST+.24
                and abs(yaw_from_state(objects[SHAFT.name]))<=math.radians(2.5))


def score(objects):
    return 1. if success(objects) else .50 if _installed and fit(objects,LIVE_CONTEXT) else 0.


START_GRIP=(HORN.pos[0],HORN.pos[1],HORN.pos[2]+.009)
OVER_SEAT=(X,Y,HORN_SEATED_Z+.034)
GRIP_HEIGHT=.009
SEAT_GOAL=(X,Y,HORN_SEATED_Z+GRIP_HEIGHT-.001)
BACK=(.246,-.286,TOP+.204)
GRIPPED=-.008


def demo():
    return {'right':[
        Waypoint(1.22,(START_GRIP[0],START_GRIP[1],TOP+.11),FINGER_OPEN,'move to the output horn'),
        Waypoint(.76,START_GRIP,FINGER_OPEN,'reach the horn rim'),
        Waypoint(.72,START_GRIP,GRIPPED,'grip the disc'),
        Waypoint(.9,(START_GRIP[0],START_GRIP[1],TOP+.128),GRIPPED,'lift the horn clear of its splines'),
        Waypoint(1.27,(X,Y,TOP+.131),GRIPPED,'move over the output wheel gear'),
        Waypoint(1.78,OVER_SEAT,GRIPPED,'align the two factory witnesses'),
        Waypoint(2.30,SEAT_GOAL,GRIPPED,'seat the horn onto the washer'),
        Waypoint(1.29,SEAT_GOAL,GRIPPED,'hold it at the seated depth'),
        Waypoint(.60,SEAT_GOAL,FINGER_OPEN,'let go of the indexed horn'),
        Waypoint(1.05,BACK,FINGER_OPEN,'clear the servo for the unloaded test'),
        Waypoint(11.,BACK,FINGER_OPEN,'watch both swings and the return to neutral'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    global _grasp_rotation
    if arm!='right':
        return position,rotation,finger
    u=min(1.,max(0.,(t-3.60)/1.27));yaw=-HORN_RESET_YAW*u*u*(3.-2.*u)
    aligning=phase=='align the two factory witnesses'
    fitting=phase in {'seat the horn onto the washer','hold it at the seated depth','let go of the indexed horn'}
    if 3.35<=t<=4.88 and 'tcp_rotation' in LIVE_CONTEXT:
        current=LIVE_CONTEXT['body_origins'][HORN.name]
        tcp=LIVE_CONTEXT['tcp']['right']
        horn_r=quat_xyzw_to_matrix(LIVE_OBJECTS[HORN.name].quat_xyzw)
        _grasp_local[:]=horn_r.T@(np.asarray(tcp)-np.asarray(current))
        _grasp_rotation=horn_r.T@LIVE_CONTEXT['tcp_rotation']['right']
    tilt=axis_angle_matrix(np.array((0.,1.,0.)),-.26*min(1.,t/.8))
    wrist=tilt@grip_rotation(yaw)
    if (aligning or fitting) and _grasp_rotation is not None:
        position=np.asarray(position).copy()
        fraction=min(1.,max(0.,(t-4.87)/.78))
        position+=fraction*(_grasp_local-np.array((0.,0.,GRIP_HEIGHT)))
        wrist=slerp_matrix(wrist,_grasp_rotation,fraction)
        if t>5.68 and 'body_origins' in LIVE_CONTEXT:
            error=np.array((X,Y))-LIVE_CONTEXT['body_origins'][HORN.name][:2]
            if np.linalg.norm(error)<.007:
                _fit_trim[:]=np.clip(_fit_trim+.22*error,-.008,.008)
            position[:2]+=_fit_trim+.52*error
            if (fitting and t<10.1 and
                    LIVE_CONTEXT['body_origins'][HORN.name][2]>HORN_SEATED_Z+.0008):
                dither=math.radians(3.)*math.sin((t-7.0)*6.)
                wrist=axis_angle_matrix(np.array((0.,0.,1.)),dither)@wrist
                position[2]-=.0016
    return position,wrist,finger
