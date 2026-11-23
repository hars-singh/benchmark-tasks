"""Support a spring-hinged door while placing a loose friction wedge beneath it."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_spring_door_wedge.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
HINGE = (.492, .221, TOP + .144)
DOOR_WIDTH, DOOR_HEIGHT, GAP = .265, .244, .017
LENGTH, WIDTH, HIGH = .12065, .03175, .03810
TOE = .0026
SURFACE_SLOPE = (HIGH - TOE) / LENGTH
HOLD_ANG = -math.radians(72.)
MARK_ANG = -math.radians(56.)
HANDLE = np.asarray((-.049, -.223, .055))
RIM = .246
FOUNDATION = (.132,.168,.184)
WOOD = (.571,.36,.224)
PANEL = (.655,.456,.304)
STEEL = (.49,.52,.55)
WEDGE_COLOR = (.824,.797,.704)

POST = Box('stationary_door_jamb', pos=(HINGE[0],HINGE[1]+.026,TOP+.136),
           size=(.051,.041,.272), mass=1.15, color=FOUNDATION)
DOOR = Box('spring_hinged_door', pos=HINGE, size=(.029,DOOR_WIDTH,DOOR_HEIGHT), mass=.37, color=WOOD)
WEDGE = Box('shepherd_9163_rubber_wedge', pos=(.235,-.282,TOP+HIGH/2+.0002),
            size=(LENGTH,WIDTH,HIGH), mass=.065, color=WEDGE_COLOR)
OBJECTS = [POST, DOOR, WEDGE]
SUPPORT_JOINTS = {POST.name:{'kind':'fixed'}}
OBJECT_JOINTS = {DOOR.name:{'kind':'revolute','parent':POST.name,'axis':'Z','anchor':HINGE,
                           'lower':-84.,'upper':.6,
                           'drive':{'stiffness':.034,'damping':.011,'target':0.,'max_force':.36}}}
SURFACE_FRICTION = {WEDGE.name:(1.0,.87), DOOR.name:(.84,.70)}
CONTACT_OFFSETS_M = {WEDGE.name:.00034,DOOR.name:.00045}
RIGID_SOLVER_ITERATIONS = {WEDGE.name:(72,8),DOOR.name:(72,8)}


ramp_pitch = -math.atan(SURFACE_SLOPE)
ramp_length = math.hypot(LENGTH,HIGH-TOE)
BODY_PARTS = {
    POST.name: (
        Solid('box',POST.size),
        Solid('box',(.070,.122,.016),(0.,.023,TOP-POST.pos[2]+.008),color=FOUNDATION),
        *(Solid('cylinder',(.009,.048),(0.,-.023,z),color=STEEL) for z in (-.079,.073)),
    ),
    DOOR.name: (
        Solid('box',(.027,DOOR_WIDTH,DOOR_HEIGHT),(0.,-DOOR_WIDTH/2,TOP+GAP+DOOR_HEIGHT/2-HINGE[2]),color=WOOD),
        Solid('box',(.012,.220,.182),(-.011,-.136,TOP+.139-HINGE[2]),color=PANEL),
        *(Solid('box',(.057,.013,.012),(-.032,-.223+dy,HANDLE[2]),color=STEEL)
          for dy in (-.035,.035)),
        Solid('capsule_x',(.0075,.059),tuple(HANDLE),yaw=math.pi/2,color=STEEL),
    ),
    WEDGE.name: (
        Solid('box',(LENGTH,WIDTH,.0022),(0.,0.,.0011-HIGH/2)),
        Solid('box',(ramp_length,WIDTH,.0028),(0.,0.,TOE/2-.0010),pitch=ramp_pitch),
        Solid('box',(.0105,WIDTH,.032),((LENGTH-.011)/2,0.,.016-HIGH/2)),
        *(Solid('box',(.015,WIDTH,height),(x,0.,height/2-HIGH/2))
          for x,height in ((-.041,.006),(-.023,.012),(-.005,.016),(.013,.022),(.030,.026),(.045,.030))),
    ),
}
VISUAL_PARTS = {
    WEDGE.name:tuple(Solid('box',(.0025,WIDTH-.0006,.00043),
                           (x,0.,TOE+SURFACE_SLOPE*(x+LENGTH/2)-HIGH/2+.0008),
                           pitch=ramp_pitch,color=(.68,.655,.59))
                     for x in np.linspace(-.050,.053,18)),
    DOOR.name:tuple(Solid('box',(.00038,.186,.0013),(-.0171,-.134,float(z)),color=(.61,.41,.27))
                    for z in np.linspace(-.089,.095,10)),
}
OBJECT_MATERIALS = {POST.name:'fixed residential jamb and hinge plates',
                    DOOR.name:'scaled wooden non-rated door on an adjustable spring hinge',
                    WEDGE.name:'Shepherd 9163 antiskid rubber wedge'}

cam=np.array((.135,-.117,TOP+.108))-np.asarray(POST.pos)
view=np.array((.308,.063,TOP+.046))-np.asarray(POST.pos)-cam
view/=np.linalg.norm(view)
side=np.cross(view,(0.,0.,1.))
side/=np.linalg.norm(side)
TASK_RECORD_CAMERA=(POST.name,Camera(tuple(cam),tuple(matrix_to_quat_xyzw(np.column_stack((side,np.cross(side,view),-view)))),
                                   'opengl',Lens.from_focal(23.,640,352)))


LIVE_CONTEXT = {}
LIVE_OBJECTS = {}
SUCCESS_HOLD_S = 2.
MAX_PARTIAL_SCORE = .50
_opened_to = 0.
_first_load = None


def yaw(orientation):
    return math.atan2(orientation[1,0], orientation[0,0])


def door_rotation(angle):
    return axis_angle_matrix(np.array((0.,0.,1.)),angle)


def door_sill(angle,radius):
    return np.asarray(HINGE)+door_rotation(angle)@np.array((0.,-radius,TOP+GAP-HINGE[2]))


tangent = np.array((math.cos(MARK_ANG),math.sin(MARK_ANG),0.))
nominal_crossing = door_sill(MARK_ANG,RIM)
WEDGE_YAW = MARK_ANG
WEDGE_TARGET = nominal_crossing + .014*tangent + np.array((0.,0.,HIGH/2-GAP+.001))


def on_reset():
    global _opened_to,_first_load
    _opened_to=0.
    _first_load=None


def wedge_contact(objects, context, angle):
    body = objects[WEDGE.name]
    r = quat_xyzw_to_matrix(body.quat_xyzw)
    origin = np.asarray(context['body_origins'][WEDGE.name])
    upright = r[2,2]>.98 and abs(body.low[2]-TOP)<.0032
    if not upright:
        return False
    # The thick bottom rail can contact the wedge at its closing face and free end.
    rail = np.array([(lateral, -radial, TOP + GAP - HINGE[2])
                     for lateral in (-.0135, -.00675, 0., .00675, .0135)
                     for radial in np.linspace(RIM - .064, DOOR_WIDTH, 35)])
    world = np.asarray(HINGE) + rail @ door_rotation(angle).T
    local = (world - origin) @ r
    level = TOE + SURFACE_SLOPE * (local[:, 0] + LENGTH / 2) - HIGH / 2
    return bool(np.any((abs(local[:, 1]) <= WIDTH / 2 + .0020)
                       & (abs(local[:, 0]) <= LENGTH / 2)
                       & (abs(local[:, 2] - level) < .003)))


def observe(objects,context,dt):
    global _opened_to,_first_load
    rotation=quat_xyzw_to_matrix(objects[DOOR.name].quat_xyzw)
    angle=yaw(rotation)
    _opened_to=min(_opened_to,angle)
    if _first_load is None and _opened_to<=math.radians(-66.) and wedge_contact(objects,context,angle):
        _first_load=context['time_s']


def supported(objects):
    door=objects[DOOR.name]
    angle=yaw(quat_xyzw_to_matrix(door.quat_xyzw))
    if _first_load is None or abs(door.ang_vel[2])>.075:
        return False,angle
    handle=np.asarray(HINGE)+door_rotation(angle)@HANDLE
    wedge=LIVE_CONTEXT['body_origins'][WEDGE.name]
    clear=all(min(math.dist(tcp,handle),math.dist(tcp,wedge))>.096 for tcp in LIVE_CONTEXT['tcp'].values())
    return (clear and wedge_contact(objects,LIVE_CONTEXT,angle) and
            np.linalg.norm(objects[WEDGE.name].lin_vel)<.013),angle


def success(objects):
    stable,angle=supported(objects)
    return bool(stable and math.radians(-61.)<=angle<=math.radians(-50.))


def score(objects):
    stable,angle=supported(objects)
    if stable and math.radians(-63.)<=angle<=math.radians(-27.):
        return 1. if success(objects) else .50
    return 0.


PULL_PICK=np.asarray(HINGE)+HANDLE
PULL_ABOVE=PULL_PICK+(-.044,0.,.074)
PULL_PITCH=axis_angle_matrix(np.array((0.,1.,0.)),math.radians(-45.))
PULL_GRIP=PULL_PITCH@grip_rotation(math.pi/2.)
WEDGE_GRIP=np.asarray(WEDGE.pos)+(.035,0.,.011)
WEDGE_UP=WEDGE_GRIP+(0.,0.,.143)
WEDGE_PLACE=WEDGE_TARGET+door_rotation(WEDGE_YAW)@(.035,0.,.011)
OPEN_START=3.02
OPEN_END=5.48
WEDGE_ALIGN=5.62
WEDGE_APPROACH=1.47
RELEASE_DOOR=10.45
PULL_PARK=(.171,.245,TOP+.240)
WEDGE_PARK=(.151,-.255,TOP+.189)


def demo():
    return {
        'left':[
            Waypoint(1.31,tuple(PULL_ABOVE),FINGER_OPEN,'approach the pull handle'),
            Waypoint(.96,tuple(PULL_PICK),-.026,'reach around the handle'),
            Waypoint(.75,tuple(PULL_PICK),-.004,'grip the round pull'),
            Waypoint(OPEN_END-OPEN_START,tuple(PULL_PICK),-.004,'open against the spring'),
            Waypoint(RELEASE_DOOR-OPEN_END,tuple(PULL_PICK),-.004,'hold while the wedge is set'),
            Waypoint(.49,tuple(PULL_PICK),FINGER_OPEN,'release the spring-loaded door'),
            Waypoint(1.22,PULL_PARK,FINGER_OPEN,'withdraw clear of the spring door'),
            Waypoint(4.,PULL_PARK,FINGER_OPEN,'observe the spring-loaded hold'),
        ],
        'right':[
            Waypoint(1.40,tuple(WEDGE_UP),FINGER_OPEN,'approach the loose door wedge'),
            Waypoint(.78,tuple(WEDGE_GRIP),FINGER_OPEN,'reach the raised rubber heel'),
            Waypoint(.68,tuple(WEDGE_GRIP),-.009,'grip the wedge heel'),
            Waypoint(.77,tuple(WEDGE_UP),-.009,'lift off the floor'),
            Waypoint(WEDGE_ALIGN-3.63,tuple(WEDGE_UP),-.009,'wait for door clearance'),
            Waypoint(WEDGE_APPROACH,tuple(WEDGE_PLACE+(0.,0.,.139)),-.009,'align the wedge toe'),
            Waypoint(1.08,tuple(WEDGE_PLACE),-.009,'set the wedge across the closing path'),
            Waypoint(.52,tuple(WEDGE_PLACE),FINGER_OPEN,'release the rubber wedge'),
            Waypoint(.70,tuple(WEDGE_PLACE+(0.,0.,.123)),FINGER_OPEN,'raise clear of the wedge'),
            Waypoint(1.05,WEDGE_PARK,FINGER_OPEN,'clear the closing door'),
            Waypoint(6.30,WEDGE_PARK,FINGER_OPEN,'leave the wedge supporting the door'),
        ]}


def eased(value):
    t=min(1.,max(0.,value))
    return t*t*(3.-2.*t)


def demo_target(arm,phase,time,position,rotation,finger):
    if arm=='right':
        turn=eased((time-WEDGE_ALIGN)/WEDGE_APPROACH)
        return position,grip_rotation(WEDGE_YAW*turn),finger
    if time<OPEN_START:
        return position,PULL_GRIP,finger
    if time>RELEASE_DOOR+.49:
        fraction=eased((time-RELEASE_DOOR-.49)/1.22)
        open_position=np.asarray(HINGE)+door_rotation(HOLD_ANG)@HANDLE
        destination=np.asarray(PULL_PARK)
        return open_position+fraction*(destination-open_position),PULL_PITCH@grip_rotation(math.pi/2+HOLD_ANG),finger
    angle=HOLD_ANG*eased((time-OPEN_START)/(OPEN_END-OPEN_START))
    lead=-.026 if OPEN_START+.22<time<OPEN_END else 0.
    target=np.asarray(HINGE)+door_rotation(angle+lead)@HANDLE
    return target,PULL_PITCH@grip_rotation(math.pi/2+angle),finger
