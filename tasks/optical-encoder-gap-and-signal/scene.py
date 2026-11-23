"""Seat a push-on encoder disk with its spacer, then verify direction and counts."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_optical_encoder_gap_and_signal.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.354,.042
MOTOR_TOP=TOP+.115
PCB_Z=MOTOR_TOP+.0035
DISK_TARGET=PCB_Z+.00425
PRESS_TRAVEL=.00355
HUB_START=DISK_TARGET+PRESS_TRAVEL
TOOL_Z=HUB_START+.0063
WHEEL_Z=TOP+.035
CPR=720
COUNTS_PER_REV=4*CPR
MIN_TEST_EDGES=336
MAX_TEST_EDGES=384
GRAY=(.37,.42,.48)
STEEL=(.52,.56,.61)
DARK=(.086,.095,.106)
OPTIC=(.095,.23,.17)
TOOL_COLOR=(.23,.42,.56)
HUB_COLOR=(.65,.64,.59)
HANDLE=np.array((-.022,-.085,.026))
SHAFT_CENTER=np.array((X,Y,WHEEL_Z))
FRAME=Box('mounted_motor_and_encoder_sensor',pos=(X,Y,TOP+.087),size=(.064,.059,.056),
          mass=1.08,color=DARK)
SHAFT=Box('motor_shaft_and_test_handwheel',pos=tuple(SHAFT_CENTER),size=(.177,.177,.102),
          mass=.16,color=GRAY)
DISK=Box('push_on_codewheel',pos=(X,Y,HUB_START),size=(.013,.013,.0104),mass=.008,color=HUB_COLOR)
SPACER=Box('manufacturer_gapping_tool',pos=(X,Y,TOOL_Z),size=(.032,.017,.018),mass=.010,color=TOOL_COLOR)
PARK=Box('spacer_tool_mat',pos=(.263,.142,TOP+.002),size=(.071,.068,.004),color=(.11,.29,.23))
OBJECTS=[FRAME,SHAFT,DISK,SPACER,PARK]


def ring(inner,outer,height,z,color):
    n=18
    return tuple(Solid('tapered_panel',(inner,inner,outer-inner,height,math.tau/n),
                       (0.,0.,z),roll=i*math.tau/n,pitch=-math.pi/2,color=color)
                 for i in range(n))


def board_with_bore(width,length,z,height,color):
    hole=.0106
    return (
        *(Solid('box',((width-hole)/2.,length,height),(s*(width+hole)/4.,0.,z),color=color)
          for s in (-1,1)),
        *(Solid('box',(hole,(length-hole)/2.,height),(0.,s*(length+hole)/4.,z),color=color)
          for s in (-1,1)),
    )


BODY_PARTS={
    FRAME.name:(
        Solid('box',(.060,.057,.051),(0.,0.,.0025),color=DARK),
        Solid('box',(.047,.025,.064),(.011,.042,-.052),color=GRAY),
        Solid('box',(.057,.027,.010),(.011,.042,TOP-FRAME.pos[2]+.005),color=GRAY),
        *board_with_bore(.027,.020,MOTOR_TOP-FRAME.pos[2]+.0015,.003,DARK),
        *board_with_bore(.024,.017,PCB_Z-FRAME.pos[2]-.0006,.0012,OPTIC),
        Solid('box',(.0048,.005,.005),(.0095,-.001,PCB_Z-FRAME.pos[2]+.0025),color=DARK),
        Solid('box',(.004,.007,.003),(-.010,.0007,PCB_Z-FRAME.pos[2]+.0015),color=(.84,.84,.76)),
        *(Solid('box',(.0025,.006,.0055),(x,0.,PCB_Z-FRAME.pos[2]+.00275),color=DARK)
          for x in (-.013,.013)),
    ),
    SHAFT.name:(
        Solid('cylinder',(.003175,MOTOR_TOP+.011-WHEEL_Z),(0.,0.,(MOTOR_TOP+.011-WHEEL_Z)/2.),color=STEEL),
        Solid('cylinder',(.017,.013),color=DARK),
        *ring(.078,.088,.009,0.,DARK),
        *(Solid('box',(.069,.012,.009),(.045*math.cos(a),.045*math.sin(a),0.),yaw=a,color=GRAY)
          for a in (0.,math.tau/3.,2*math.tau/3.)),
        Solid('box',(.018,.021,.010),(HANDLE[0],HANDLE[1],.004),color=GRAY),
        Solid('cylinder',(.010,.034),tuple(HANDLE),color=DARK),
    ),
    DISK.name:(Solid('cylinder',(.0064,.00065),(0.,0.,0.),color=(.77,.78,.74)),
               *ring(.00331,.00465,.0075,-.0016,HUB_COLOR)),
    SPACER.name:(
        Solid('box',(.0117,.018,.008),(-.0099,0.,0.),color=TOOL_COLOR),
        Solid('box',(.0117,.018,.008),(.0099,0.,0.),color=TOOL_COLOR),
        Solid('box',(.0081,.005,.008),(0.,-.00655,0.),color=TOOL_COLOR),
        Solid('box',(.0081,.005,.008),(0.,.00655,0.),color=TOOL_COLOR),
        *(Solid('box',(.0043,.0028,.0052),(x,y,-.0068),color=TOOL_COLOR)
          for x in (-.0089,.0089) for y in (-.0065,.0065)),
    ),
}
VISUAL_PARTS={
    DISK.name:tuple(Solid('box',(.00155,.00034,.00008),(.0054*math.cos(a),.0054*math.sin(a),.00037),
                         yaw=a,color=DARK) for a in (i*math.tau/32 for i in range(32))),
    FRAME.name:tuple(Solid('box',(.0032,.00057,.0003),(.030*math.cos(a),.030*math.sin(a),.0272),
                          yaw=a,color=(.85,.54,.12)) for a in (-1.10,-.76,-.42)),
}
SUPPORT_JOINTS={FRAME.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    SHAFT.name:{'kind':'revolute','parent':FRAME.name,'axis':'Z','anchor':SHAFT.pos,
                'lower':-6.,'upper':49.,'drive':{'stiffness':0.,'damping':.014,'max_force':.16}},
    DISK.name:{'kind':'prismatic','parent':SHAFT.name,'axis':'Z','anchor':DISK.pos,
               'lower':-PRESS_TRAVEL,'upper':.0004,'drive':{'damping':2600.,'max_force':33.}},
}
SURFACE_FRICTION={SPACER.name:(.90,.72),SHAFT.name:(.77,.65)}
CONTACT_OFFSETS_M={FRAME.name:.00021,SHAFT.name:.00035,DISK.name:.00012,SPACER.name:.00018}
RIGID_SOLVER_ITERATIONS={o.name:(72,8) for o in OBJECTS}
OBJECT_MATERIALS={FRAME.name:'supported motor, stock encoder base and photodiode PCB',
                  SHAFT.name:'6.35 mm steel shaft and unpowered bench handwheel',
                  DISK.name:'optical film disk on a press-on aluminium hub',
                  SPACER.name:'US Digital SPACER-E8T tool',PARK.name:'rubber parts mat'}

optical_pos=np.array((.148,-.084,.233))
optical_axis=np.array((-.023,-.008,.030))-optical_pos
optical_axis/=np.linalg.norm(optical_axis)
optical_right=np.cross(optical_axis,(0.,0.,1.))
optical_right/=np.linalg.norm(optical_right)
optical_rotation=np.column_stack((optical_right,np.cross(optical_right,optical_axis),-optical_axis))
TASK_RECORD_CAMERA=(FRAME.name,Camera(tuple(optical_pos),tuple(matrix_to_quat_xyzw(optical_rotation)),
                                      'opengl',Lens.from_focal(41.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.0
MAX_PARTIAL_SCORE=.50
_pressed=False
_forward_edges=0
_reverse_edges=0
_first_angle=None
_furthest=0.
_invalid=False


def on_reset():
    global _pressed,_forward_edges,_reverse_edges,_first_angle,_furthest,_invalid
    _pressed=False
    _forward_edges=_reverse_edges=0
    _first_angle=None
    _furthest=0.
    _invalid=False


def disk_is_seated(objects,context):
    p=context['body_origins'][DISK.name]
    normal=quat_xyzw_to_matrix(objects[DISK.name].quat_xyzw)[2,2]
    return abs(p[2]-DISK_TARGET)<=.00021 and math.hypot(p[0]-X,p[1]-Y)<=.00010 and normal>.999


def tool_is_clear(objects,context):
    p=context['body_origins'][SPACER.name]
    return abs(p[0]-PARK.pos[0])<.022 and abs(p[1]-PARK.pos[1])<.020 and objects[SPACER.name].high[2]<TOP+.027


def observe(objects,context,dt):
    global _pressed,_forward_edges,_reverse_edges,_first_angle,_furthest,_invalid
    origins=context['body_origins']
    _invalid=_invalid or math.dist(origins[FRAME.name],FRAME.pos)>.002
    seated=disk_is_seated(objects,context)
    touching=math.hypot(origins[SPACER.name][0]-X,origins[SPACER.name][1]-Y)<.0015
    if seated and touching and abs(origins[SPACER.name][2]-(TOOL_Z-PRESS_TRAVEL))<.00065:
        _pressed=True
    if not _pressed or not seated or not tool_is_clear(objects,context):
        return
    rotation=quat_xyzw_to_matrix(objects[DISK.name].quat_xyzw)
    angle=math.atan2(rotation[1,0],rotation[0,0])
    if _first_angle is None:
        _first_angle=angle
        _furthest=angle
    _furthest=max(_furthest,angle)
    _forward_edges=max(_forward_edges,round((_furthest-_first_angle)*COUNTS_PER_REV/math.tau))
    if _forward_edges>=MIN_TEST_EDGES:
        _reverse_edges=round((_furthest-angle)*COUNTS_PER_REV/math.tau)


def success(objects):
    correct_count=lambda n:MIN_TEST_EDGES<=n<=MAX_TEST_EDGES
    return bool(not _invalid and _pressed and disk_is_seated(objects,LIVE_CONTEXT)
                and tool_is_clear(objects,LIVE_CONTEXT) and correct_count(_forward_edges)
                and correct_count(_reverse_edges) and abs(_forward_edges-_reverse_edges)<=14
                and abs(objects[DISK.name].ang_vel[2])<.12)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .50 if _pressed and tool_is_clear(objects,LIVE_CONTEXT) else 0.


TOOL_GRIP=(X,Y,TOOL_Z+.006)
PRESS_TARGET=(X,Y,TOOL_Z-.0135)
SEATED_GRIP=(X,Y,TOOL_Z-PRESS_TRAVEL+.0045)
TOOL_LIFT=(X,Y,TOOL_Z+.078)
PARK_Z=TOP+.004+.0102+.006
WHEEL_GRIP=HANDLE+np.array((0.,0.,.003))
WHEEL_START=tuple(SHAFT_CENTER+WHEEL_GRIP)
WHEEL_APPROACH=tuple(SHAFT_CENTER+WHEEL_GRIP+(0.,0.,.063))
TURN_START=13.45
TURN_DURATION=1.65
TURN_BACK=TURN_START+TURN_DURATION+.63
TURN_END=TURN_BACK+TURN_DURATION
OPEN_GRIP=-.006
TOOL_PINCH=-.0024


def demo():
    return {
        'left':[
            Waypoint(1.3,(X,Y,TOOL_Z+.075),FINGER_OPEN,'move above the stock spacer'),
            Waypoint(.85,TOOL_GRIP,FINGER_OPEN,'reach the spacer'),
            Waypoint(.84,TOOL_GRIP,OPEN_GRIP,'grip the spacer sides'),
            Waypoint(1.0,PRESS_TARGET,OPEN_GRIP,'press to the installed gap'),
            Waypoint(.65,PRESS_TARGET,OPEN_GRIP,'hold against the spacer feet'),
            Waypoint(.84,(X,Y,TOOL_Z+.045),FINGER_OPEN,'lift and turn the open jaws'),
            Waypoint(1.28,SEATED_GRIP,FINGER_OPEN,'lower the open jaws around the seated spacer'),
            Waypoint(1.12,SEATED_GRIP,TOOL_PINCH,'grasp the spacer at seated height'),
            Waypoint(1.21,TOOL_LIFT,TOOL_PINCH,'lift the spacer off the shaft'),
            Waypoint(.85,(PARK.pos[0],PARK.pos[1],TOOL_LIFT[2]),TOOL_PINCH,'carry the spacer over its mat'),
            Waypoint(.80,(PARK.pos[0],PARK.pos[1],PARK_Z),TOOL_PINCH,'set it down'),
            Waypoint(.70,(PARK.pos[0],PARK.pos[1],PARK_Z),FINGER_OPEN,'release spacer'),
            Waypoint(.90,(.203,.211,PARK_Z+.085),FINGER_OPEN,'move away from the motor'),
        ],
        'right':[
            Waypoint(1.4,WHEEL_APPROACH,FINGER_OPEN,'approach the test wheel'),
            Waypoint(TURN_START-3.,WHEEL_APPROACH,-.019,'wait until the spacer is clear'),
            Waypoint(.90,WHEEL_START,-.019,'reach the handwheel knob'),
            Waypoint(.70,WHEEL_START,-.006,'grasp the knob'),
            Waypoint(TURN_DURATION,WHEEL_START,-.006,'verify forward count'),
            Waypoint(.63,WHEEL_START,-.006,'inspect the forward stop'),
            Waypoint(TURN_DURATION,WHEEL_START,-.006,'verify reverse count'),
            Waypoint(1.5,WHEEL_START,-.006,'stop at the starting mark'),
            Waypoint(.65,WHEEL_START,FINGER_OPEN,'release handwheel'),
            Waypoint(1.2,WHEEL_APPROACH,FINGER_OPEN,'clear the test wheel'),
        ],
    }


def demo_target(arm,phase,t,position,rotation,finger):
    pitch=-.35 if arm=='left' else -.49
    fraction=min(1.,t/.90)
    wrist=axis_angle_matrix(np.array((0.,1.,0.)),pitch*fraction)
    if arm=='left' and t>=4.64:
        turn=min(1.,max(0.,(t-4.64)/.84))
        yaw=axis_angle_matrix(np.array((0.,0.,1.)),math.pi/2.*turn)
        wrist=axis_angle_matrix(np.array((0.,1.,0.)),pitch*(1.-.65*turn))
        target=np.asarray(position)
        if (phase in {'lower the open jaws around the seated spacer','grasp the spacer at seated height'}
                and 'body_origins' in LIVE_CONTEXT):
            pick=np.asarray(LIVE_CONTEXT['body_origins'][SPACER.name])+(-.013,.004,.0056)
            weight=1. if phase=='grasp the spacer at seated height' else min(1.,max(0.,(t-5.48)/1.28))
            target=(1.-weight)*target+weight*pick
        return target,yaw@wrist@grip_rotation(),finger
    if arm!='right' or t<TURN_START or t>TURN_END+1.50:
        return position,wrist@rotation,finger
    interval=lambda t0:min(1.,max(0.,(t-t0)/TURN_DURATION))
    ease=lambda value:value*value*(3.-2.*value)
    desired=math.radians(46.)*ease(interval(TURN_START))-math.radians(48.2)*ease(interval(TURN_BACK))
    delta=desired
    if 'body_origins' in LIVE_CONTEXT:
        shaft_r=quat_xyzw_to_matrix(LIVE_OBJECTS[SHAFT.name].quat_xyzw)
        actual=math.atan2(shaft_r[1,0],shaft_r[0,0])
        delta+=float(np.clip((desired-actual)*.93,-.105,.11))
        if t>=TURN_END:
            delta-=.038
    yaw=axis_angle_matrix(np.array((0.,0.,1.)),delta)
    steering=axis_angle_matrix(np.array((0.,0.,1.)),.29*delta)
    return SHAFT_CENTER+yaw@WHEEL_GRIP,steering@wrist@grip_rotation(),finger
