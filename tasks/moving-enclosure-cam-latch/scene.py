"""Compress a moving enclosure lid and secure it with a native over-center cam."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_moving_enclosure_cam_latch.geometry import Solid
from dynamic_task_moving_enclosure_cam_latch.mechanisms import linear_stage
from dynamic_task_moving_enclosure_cam_latch.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRAVEL_M = .039
PERIOD_S = 8.
OPEN_ANGLE = math.radians(-65.)
CLOSED_ANGLE = math.radians(23.)
CAM_ECCENTRICITY = .0115
CAM_RADIUS = .009
PADDLE_RADIUS = .010
HANDLE_LENGTH = .055
PADDLE_OFFSET_Y = -.145
PRELOAD_M = .0052
MINIMUM_SEAL_M = .0033
STAGE = Box('enclosure_carriage',pos=(.326,.040,TOP+.015),size=(.200,.226,.030),
            mass=2.1,color=(.13,.23,.36))
LID = Box('spring_lid',pos=(.333,.044,TOP+.085),size=(.116,.110,.012),mass=.135,
          color=(.55,.64,.69))
PIVOT = (.295,.021,TOP+.106)
CAM = Box('eccentric_latch',pos=PIVOT,size=(.139,.167,.090),mass=.047,color=(.76,.47,.16))
PRESS_POINT = (.367,.071)
hardware, BODY_PARTS = linear_stage(STAGE,TRAVEL_M)
OBJECTS = [STAGE,LID,CAM,*hardware]


def y_rotation(vector, angle):
    x,y,z = vector
    c,s = math.cos(angle),math.sin(angle)
    return x*c+z*s,y,-x*s+z*c


def paddle_local(angle):
    return y_rotation((HANDLE_LENGTH,PADDLE_OFFSET_Y,0.),angle)


BODY_PARTS[STAGE.name] += tuple(
    Solid('box',size,(x-STAGE.pos[0],y-STAGE.pos[1],z-STAGE.pos[2]),color=color)
    for size,(x,y,z),color in (
        ((.126,.120,.034),(.333,.044,TOP+.048),(.26,.32,.37)),
        ((.121,.115,.006),(.333,.044,TOP+.068),(.12,.15,.17)),
        ((.118,.112,.003),(.333,.044,TOP+.073),(.23,.27,.29)),
        ((.020,.018,.079),(PIVOT[0],-.020,TOP+.0695),(.29,.34,.37)),
        ((.016,.014,.018),(PIVOT[0],-.020,PIVOT[2]),(.62,.66,.69)),
    )
)
BODY_PARTS[LID.name] = (
    Solid('box',LID.size),
    Solid('box',(.036,.029,.0018),(PRESS_POINT[0]-LID.pos[0],PRESS_POINT[1]-LID.pos[1],.006),
          color=(.18,.31,.40)),
    Solid('box',(.029,.031,.002),(PIVOT[0]-LID.pos[0],PIVOT[1]-LID.pos[1],.006),
          color=(.66,.70,.72)),
)
BODY_PARTS[CAM.name] = (
    Solid('cylinder',(.0045,.162),(0.,-.073,0.),axis='Y',color=(.61,.66,.70)),
    Solid('cylinder',(CAM_RADIUS,.018),y_rotation((0.,0.,-CAM_ECCENTRICITY),OPEN_ANGLE),
          axis='Y',color=(.62,.67,.71)),
    Solid('box',(.054,.013,.010),y_rotation((.026,PADDLE_OFFSET_Y,0.),OPEN_ANGLE),pitch=OPEN_ANGLE),
    Solid('sphere',(PADDLE_RADIUS,),paddle_local(OPEN_ANGLE),color=(.91,.59,.21)),
)
OBJECT_JOINTS = {
    LID.name:{'kind':'prismatic','parent':STAGE.name,'anchor':LID.pos,
              'axis':'Z','lower':-.0092,'upper':0.,
              'drive':{'stiffness':1150.,'damping':5.,'max_force':16.}},
    CAM.name:{'kind':'revolute','parent':STAGE.name,'anchor':PIVOT,
              'axis':'Y','lower':0.,'upper':math.degrees(CLOSED_ANGLE-OPEN_ANGLE),
              'drive':{'stiffness':.003,'damping':.016,'max_force':.10}},
}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,TRAVEL_M,0.),PERIOD_S)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.054,'upper':.054}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':TRAVEL_M}}
CONTACT_PAIRS = {'cam_compression':(LID.name,CAM.name)}
CONTACT_OFFSETS_M = {LID.name:.00020,CAM.name:.00020}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,4) for obj in (STAGE,LID,CAM)}
SUCCESS_HOLD_S = .75
MAX_PARTIAL_SCORE = .60
LIVE_CONTEXT = {}
_preloaded = False
_engaged = False
_proof_start = None
_proof_travel = False


def on_reset():
    global _preloaded,_engaged,_proof_start,_proof_travel
    _preloaded = _engaged = _proof_travel = False
    _proof_start = None


def closure_angle(cam):
    q = cam.quat_xyzw
    return OPEN_ANGLE+2.*math.atan2(q[1],q[3])


def lid_depth(context):
    base = context['body_origins'][STAGE.name]
    cap = context['body_origins'][LID.name]
    return LID.pos[2]-STAGE.pos[2]-(cap[2]-base[2])


def contact_points(objects,context):
    origins = context['body_origins']
    cap = origins[LID.name]
    lid_patch = cap+np.array((PRESS_POINT[0]-LID.pos[0],PRESS_POINT[1]-LID.pos[1],.007))
    frame = quat_xyzw_to_matrix(np.asarray(objects[CAM.name].quat_xyzw))
    knob = origins[CAM.name]+frame@paddle_local(OPEN_ANGLE)
    return lid_patch,knob


def observe(objects,context,dt):
    global _preloaded,_engaged,_proof_start,_proof_travel
    lid_patch,knob = contact_points(objects,context)
    angle = closure_angle(objects[CAM.name])
    depth = lid_depth(context)
    left = context['tcp']['left']
    right = context['tcp']['right']
    fixture = objects[STAGE.name]
    reaction = -context['contact_forces']['cam_compression'][2]
    _preloaded |= bool(depth>=PRELOAD_M and abs(fixture.lin_vel[1])>=.012
                       and math.dist(left,lid_patch)<.030 and angle<math.radians(-10.))
    _engaged |= bool(_preloaded and angle>=math.radians(11.) and reaction>.5
                     and math.dist(right,knob)<.040)
    hands_clear = all(math.dist(tcp,lid_patch)>.062 and math.dist(tcp,knob)>.062
                      for tcp in context['tcp'].values())
    stable = (_engaged and hands_clear and angle>=math.radians(15.) and depth>=MINIMUM_SEAL_M
              and reaction>.85 and abs(objects[CAM.name].ang_vel[1])<.32)
    carriage_y = float(context['body_origins'][STAGE.name][1])
    if not stable:
        _proof_start = None
        _proof_travel = False
    elif _proof_start is None:
        _proof_start = carriage_y
    elif abs(carriage_y-_proof_start)>=.008:
        _proof_travel = True


def success(objects):
    return _proof_travel


def score(objects):
    return 1. if success(objects) else .60 if _engaged else .20 if _preloaded else 0.


def press_pad(angle, below=0.):
    x,y,z = paddle_local(angle)
    return PIVOT[0]+x,PIVOT[1]+y,PIVOT[2]+z+PADDLE_RADIUS+PAD-below


def demo():
    seal_above = (*PRESS_POINT,TOP+.143)
    seal_down = (*PRESS_POINT,LID.pos[2]+.007+PAD-.021)
    seal_clear = (.272,.180,TOP+.179)
    paddle = press_pad(OPEN_ANGLE)
    latch_above = (*paddle[:2],paddle[2]+.006)
    latch_clear = (.206,-.151,TOP+.164)
    left = [
        Waypoint(2.1,seal_above,CLOSED,'approach seal pad'),
        Waypoint(1.25,seal_down,CLOSED,'preload the lid'),
        Waypoint(5.55,seal_down,CLOSED,'hold lid compression'),
        Waypoint(.7,seal_above,CLOSED,'retract from lid'),
        Waypoint(1.15,seal_clear,OPEN,'move clear of enclosure'),
        Waypoint(3.,seal_clear,OPEN,'observe the locked lid'),
    ]
    right = [
        Waypoint(2.3,latch_above,CLOSED,'approach latch paddle'),
        Waypoint(1.2,latch_above,CLOSED,'wait for lid preload'),
        Waypoint(.55,press_pad(OPEN_ANGLE,.002),CLOSED,'approach latch paddle'),
    ]
    for angle in np.linspace(OPEN_ANGLE,CLOSED_ANGLE,13)[1:]:
        right.append(Waypoint(.25,press_pad(float(angle),.004),CLOSED,'push cam past center'))
    right.extend([
        Waypoint(3.25,press_pad(CLOSED_ANGLE,.006),CLOSED,'hold cam at its stop'),
        Waypoint(.65,press_pad(CLOSED_ANGLE)+np.array((0.,0.,.032)),CLOSED,'retract from paddle'),
        Waypoint(1.15,latch_clear,OPEN,'move clear of enclosure'),
        Waypoint(4.,latch_clear,OPEN,'observe the locked lid'),
    ])
    return {'left':left,'right':right}


def demo_target(arm,label,t,pos,rotation,finger):
    if not LIVE_CONTEXT or label in {'move clear of enclosure','observe the locked lid','done'}:
        return pos,rotation,finger
    shift = LIVE_CONTEXT['body_origins'][STAGE.name]-np.asarray(STAGE.pos)
    shift[1]+=.090*LIVE_OBJECTS[STAGE.name].lin_vel[1]
    if arm=='right' and label in {'push cam past center','hold cam at its stop'}:
        current = closure_angle(LIVE_OBJECTS[CAM.name])
        lead = min(current+math.radians(10.),CLOSED_ANGLE)
        pos = np.asarray(press_pad(lead,.007))
    return pos+shift,rotation,finger
