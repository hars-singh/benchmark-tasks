"""Pass both handles of a loaded paper bag onto a reaching recipient's fingers."""

from dataclasses import dataclass
import math

import numpy as np
from yam_arena import station
from yam_arena.constants import FINGER_OPEN,READY
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw,slerp_matrix,tcp_pose
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix

from dynamic_task_paper_bag_recipient_handover.geometry import Solid
from dynamic_task_paper_bag_recipient_handover.motion import PoseVelocity

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
BAG_W, BAG_D, BAG_H = .180, .080, .220
BAG_X, BAG_Y = .339, -.215
GIVE_X, GIVE_Y = .388, -.112
YAW=math.pi/2.
RIM = TOP+BAG_H
HANDLE_R = .0032
ANCHOR_Z = RIM-.014
APEX_HEIGHT = .075
HANDLE_SPAN = .056
GIVE_LIFT = .085
PAPER = (.63,.39,.187)
PAPER_INSIDE = (.77,.55,.31)
CORD = (.48,.26,.105)
SKIN = (.65,.40,.284)
CUFF = (.174,.29,.52)


def capsule_between(start, end, radius, color):
    start, end = np.asarray(start), np.asarray(end)
    dx, dy, dz = end-start
    return Solid('capsule_x',(radius,max(.001,math.dist(start,end)-2*radius)),
                 tuple(map(float,(start+end)/2)),yaw=math.atan2(dy,dx),
                 pitch=-math.atan2(dz,math.hypot(dx,dy)),color=color)


BAG = Box('paper_carrier_bag',pos=(BAG_X,BAG_Y,TOP+BAG_H/2),size=(BAG_D,BAG_W,BAG_H),
          yaw=YAW,mass=.033,color=PAPER)
HAND_X = GIVE_X-.022
HAND_Y = GIVE_Y+.089
HAND_Z = ANCHOR_Z+APEX_HEIGHT+GIVE_LIFT-.013
REACH = .283
HAND = Box('recipient_hand',pos=(HAND_X,HAND_Y+REACH,HAND_Z),size=(.29,.10,.08),
           yaw=YAW,mass=.72,color=SKIN)
HANDLES = tuple(Box(name,pos=(BAG_X,BAG_Y+side*.038,ANCHOR_Z),size=(.032,.124,.082),
                    yaw=YAW,mass=.0034,color=CORD)
                for side,name in ((-1,'near_twisted_paper_handle'),(1,'far_twisted_paper_handle')))
CONTENTS = (Box('blue_carton',pos=(BAG_X+.037,BAG_Y,TOP+.072),size=(.053,.061,.136),
                yaw=YAW,mass=.100,color=(.19,.46,.61)),
            Box('green_carton',pos=(BAG_X-.037,BAG_Y,TOP+.060),size=(.056,.060,.112),
                yaw=YAW,mass=.085,color=(.29,.48,.26)))
OBJECTS = [BAG,HAND,*HANDLES,*CONTENTS]


def handle_solids(side):
    points = [(0.,-.059,-.005),(-side*.009,-.056,.029),(-side*.020,-.044,.062),
              (-side*.026,-.028,APEX_HEIGHT),(-side*.026,.028,APEX_HEIGHT),
              (-side*.020,.044,.062),(-side*.009,.056,.029),(0.,.059,-.005)]
    return tuple(capsule_between(a,b,HANDLE_R,CORD) for a,b in zip(points,points[1:]))


def recipient_solids():
    parts = [Solid('capsule_x',(.025,.143),(.110,0.,-.014),color=CUFF),
             Solid('capsule_x',(.022,.049),(.050,0.,-.010),color=SKIN)]
    parts.extend(Solid('capsule_x',(.013,.050),(0.,y,0.),color=SKIN)
                 for y in (-.023,-.007,.010,.027))
    for y, short in ((-.024,.003),(-.007,0.),(.010,.003),(.026,.014)):
        points=[(-.024,y,.004),(-.053,y,.006),(-.078+short,y,.008),
                (-.097+short,y,.016),(-.101+short,y,.029)]
        parts.extend(capsule_between(a,b,.008-i*.00065,SKIN)
                     for i,(a,b) in enumerate(zip(points,points[1:])))
    parts.extend((capsule_between((.021,.025,-.001),(-.006,.047,.006),.011,SKIN),
                  capsule_between((-.006,.047,.006),(-.026,.045,.015),.0088,SKIN)))
    return tuple(parts)


BODY_PARTS = {
    BAG.name:(Solid('box',(BAG_D,BAG_W,.002),(0.,0.,-BAG_H/2+.001),color=PAPER_INSIDE),
              *(Solid('box',(.002,BAG_W,BAG_H-.002),(side*(BAG_D/2-.001),0.,.001),color=PAPER)
                for side in (-1,1)),
              *(Solid('box',(BAG_D-.004,.002,BAG_H-.002),(0.,side*(BAG_W/2-.001),.001),color=PAPER_INSIDE)
                for side in (-1,1))),
    HAND.name:recipient_solids(),
    **{handle.name:handle_solids(side) for side,handle in zip((-1,1),HANDLES)},
}
VISUAL_PARTS = {
    BAG.name:tuple(Solid('box',(.00035,.025,.019),(side*.0386,y,BAG_H/2-.015),color=PAPER_INSIDE)
                   for side in (-1,1) for y in (-.057,.057)),
    **{carton.name:(Solid('box',(carton.size[0]-.008,.009,.0004),
                         (0.,0.,carton.size[2]/2+.0002),color=(.89,.91,.85)),)
       for carton in CONTENTS},
}
OBJECT_MATERIALS = {BAG.name:'brown kraft paper bag with 80 mm gusset',
                    HAND.name:'recipient hand with four supporting fingers and a shirt sleeve',
                    **{handle.name:'round twisted-paper cord' for handle in HANDLES},
                    **{carton.name:'sealed grocery sample carton' for carton in CONTENTS}}
OBJECT_JOINTS = {
    h.name:{'kind':'revolute','parent':BAG.name,'axis':'Y','anchor':h.pos,
            'lower':-21.,'upper':21.,'drive':{'stiffness':.145,'damping':.007,'max_force':.05}}
    for h in HANDLES
}
SUPPORT_JOINTS = {HAND.name:{'kind':'prismatic','axis':'X','lower':-REACH-.008,'upper':.008}}
SURFACE_FRICTION = {HAND.name:(.77,.63),BAG.name:(.56,.45),
                    **{h.name:(.90,.75) for h in HANDLES}}
CONTACT_OFFSETS_M = {BAG.name:.00055,HAND.name:.00065,
                      **{obj.name:.00045 for obj in (*HANDLES,*CONTENTS)}}
RIGID_SOLVER_ITERATIONS = {obj.name:(64,8) for obj in OBJECTS}


@dataclass(frozen=True)
class RecipientReach:
    reset: tuple[float, float, float]

    def at(self, seconds):
        approach, approach_v = self.travel(seconds,4.5,7.15)
        retreat, retreat_v = self.travel(seconds,10.05,13.10)
        x,y,z = self.reset
        return PoseVelocity((x,y-REACH*(approach-retreat),z),YAW,
                            (0.,-REACH*(approach_v-retreat_v),0.),(0.,0.,0.))

    @staticmethod
    def travel(t, start, end):
        u = min(1.,max(0.,(t-start)/(end-start)))
        return u*u*(3.-2.*u), 6.*u*(1.-u)/(end-start)


MOTIONS = {HAND.name:RecipientReach(HAND.pos)}
camera_pos = np.array((-.114,.381,1.62))
camera_forward = np.array((.387,-.012,.929))-camera_pos
camera_forward /= np.linalg.norm(camera_forward)
camera_right = np.cross(camera_forward,(0.,0.,1.))
camera_right /= np.linalg.norm(camera_right)
camera_rotation = np.column_stack((camera_right,np.cross(camera_right,camera_forward),-camera_forward))
TASK_TOP_CAMERA = Camera(tuple(camera_pos),tuple(matrix_to_quat_xyzw(camera_rotation)),
                         'opengl',Lens.from_focal(18.8,640,360))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.0
MAX_PARTIAL_SCORE=.50
_presented=False
_invalid=False


def on_reset():
    global _presented,_invalid
    _presented=_invalid=False


def apex_points(objects, origins):
    return tuple(origins[h.name]+quat_xyzw_to_matrix(objects[h.name].quat_xyzw)
                 @np.array((-side*.026,0.,APEX_HEIGHT)) for side,h in zip((-1,1),HANDLES))


def observe(objects, context, dt):
    global _presented,_invalid
    origins=context['body_origins']
    bag=objects[BAG.name]
    bag_origin=origins[BAG.name]
    bag_rotation=quat_xyzw_to_matrix(bag.quat_xyzw)
    local_payloads=[bag_rotation.T@(origins[o.name]-bag_origin) for o in CONTENTS]
    if bag_rotation[2,2]<.84 or any(abs(x)>.049 or abs(y)>.091 or not -.133<z<.13
                                   for x,y,z in local_payloads):
        _invalid=True
    near,far=apex_points(objects,origins)
    if (not _invalid and bag.low[2]>TOP+.048 and math.dist(near,far)<.033
        and abs(bag_origin[0]-GIVE_X)<.027 and abs(bag_origin[1]-GIVE_Y)<.026):
        _presented=True


def success(objects):
    origins=LIVE_CONTEXT['body_origins']
    hand=np.asarray(origins[HAND.name])
    hand_rotation=quat_xyzw_to_matrix(objects[HAND.name].quat_xyzw)
    near,far=apex_points(objects,origins)
    def supported(p):
        x,y,z=hand_rotation.T@(p-hand)
        return -.112<x<-.036 and -.051<y<.006 and -.004<z<.043
    clear=all(min(math.dist(near,tcp),math.dist(far,tcp))>.103 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(_presented and not _invalid and hand[1]>HAND_Y+.160 and supported(near)
                and supported(far) and clear and objects[BAG.name].low[2]>TOP+.050)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .50 if _presented else 0.


APEX=ANCHOR_Z+APEX_HEIGHT
START=(BAG_X-.003,BAG_Y,APEX+.003)
HOVER=(START[0],START[1],APEX+.075)
LIFT=(START[0],START[1],APEX+GIVE_LIFT+.022)
OFFER=(GIVE_X-.003,GIVE_Y+.043,APEX+GIVE_LIFT+.047)
LOWER=(OFFER[0],OFFER[1],APEX+GIVE_LIFT+.005)
EXIT=(.294,-.264,APEX+GIVE_LIFT+.049)
PINCH=-.0018


def demo():
    return {'right':[
        Waypoint(1.45,HOVER,FINGER_OPEN,'approach the two paper handles'),
        Waypoint(.9,START,FINGER_OPEN,'reach both handles'),
        Waypoint(.85,START,PINCH,'gather the handles'),
        Waypoint(.32,START,PINCH,'settle the hooked paper handles in the jaws'),
        Waypoint(.9,LIFT,PINCH,'lift the loaded bag'),
        Waypoint(1.5,OFFER,PINCH,'present the bag to the recipient'),
        Waypoint(1.33,OFFER,PINCH,'wait for fingers through both loops'),
        Waypoint(.7,LOWER,PINCH,'lower both handles onto the fingers'),
        Waypoint(.68,LOWER,FINGER_OPEN,'release onto the recipient'),
        Waypoint(1.22,EXIT,FINGER_OPEN,'retract above the paper handles'),
        Waypoint(5.,EXIT,FINGER_OPEN,'watch the recipient carry the bag away'),
    ]}


def demo_rotation(arm,phase,t,rotation):
    q=tcp_pose(np.asarray(READY),station.BASES[arm])[1]
    down=axis_angle_matrix(np.array((0.,1.,0.)),-1.05)@grip_rotation()
    return slerp_matrix(q,down,min(1.,t/.90))
