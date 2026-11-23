"""Square a twenty-way ribbon and seat a preassembled IDC socket using a bench press."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_idc_ribbon_termination.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
SOCKET_X, SOCKET_Y = .463, .047
BED_Z = TOP+.077
BASE_H, CAP_H = .0072, .0033
RIBBON_W, RIBBON_T = .0254, .0012
PITCH = .00127
WIRE_Z = BED_Z+BASE_H+RIBBON_T/2
STROKE = .0045
CAP_CLOSE_Z = WIRE_Z+RIBBON_T/2+CAP_H/2
PRESS_PIVOT = np.array((.530,SOCKET_Y,TOP+.211))
RAM_TOP = float(PRESS_PIVOT[2]-.019)
RAM_BOTTOM = CAP_CLOSE_Z+STROKE+CAP_H/2+.006
RAM_CENTER = (RAM_TOP+RAM_BOTTOM)/2
HANDLE_Y = -.144
HANDLE_LOCAL = np.array((-.199,HANDLE_Y,0.))
ANCHOR = np.array((.165,SOCKET_Y+.025,WIRE_Z))
LEN_SECOND = .157
FINISHED_TIP = np.array((SOCKET_X+.006,SOCKET_Y,WIRE_Z))
FINISHED_MID = FINISHED_TIP-np.array((LEN_SECOND,0.,0.))
FINISHED_FIRST = math.atan2(FINISHED_MID[1]-ANCHOR[1],FINISHED_MID[0]-ANCHOR[0])
LEN_FIRST = float(np.linalg.norm(FINISHED_MID-ANCHOR))
YAW_FIRST,YAW_SECOND = .55,-.51
RED=(.70,.11,.075)
STEEL=(.27,.31,.36)
BLACK=(.058,.063,.074)
SILVER=(.43,.48,.54)
PVC=(.54,.58,.62)
BRASS=(.64,.45,.17)


def rot(axis, angle):
    return axis_angle_matrix(np.asarray(axis,dtype=float),angle)


def axial(angle, length):
    return np.array((length*math.cos(angle),length*math.sin(angle),0.))


MID = ANCHOR+axial(YAW_FIRST,LEN_FIRST)
FRONT = MID+axial(YAW_SECOND,LEN_SECOND)
FRAME = Box('press_frame', pos=(SOCKET_X,SOCKET_Y,TOP+.040),size=(.285,.190,.080),
            mass=2.8,color=STEEL)
HANDLE=Box('press_handle',pos=tuple(map(float,PRESS_PIVOT)),size=(.262,.133,.044),mass=.21,color=STEEL)
RAM=Box('press_platen',pos=(SOCKET_X,SOCKET_Y,RAM_CENTER),size=(.039,.049,RAM_TOP-RAM_BOTTOM),
        mass=.115,color=SILVER)
CAP=Box('socket_cover',pos=(SOCKET_X,SOCKET_Y,CAP_CLOSE_Z+STROKE),size=(.006,.030,CAP_H),
        mass=.011,color=BLACK)
TESTER=Box('continuity_tester',pos=(.134,SOCKET_Y+.025,TOP+.041),size=(.070,.082,.082),
           mass=.33,color=(.15,.23,.29))
CABLE1=Box('ribbon_slack',pos=tuple(map(float,ANCHOR+axial(YAW_FIRST,LEN_FIRST/2.))),
           size=(LEN_FIRST,RIBBON_W,RIBBON_T),yaw=YAW_FIRST,mass=.012,color=PVC)
CABLE2=Box('ribbon_end',pos=tuple(map(float,MID+axial(YAW_SECOND,LEN_SECOND/2.))),
           size=(LEN_SECOND,RIBBON_W,RIBBON_T),yaw=YAW_SECOND,mass=.013,color=PVC)
DECK_TOP=BED_Z-.001
DECK=Box('bench_mat',pos=(.269,SOCKET_Y+.101,(TOP+DECK_TOP)/2.),size=(.186,.226,DECK_TOP-TOP),
         color=(.16,.29,.25))
OBJECTS=[FRAME,HANDLE,RAM,CAP,TESTER,CABLE1,CABLE2,DECK]
BODY_PARTS={
    FRAME.name:(
        Solid('box',(.248,.173,.019),(.028,0.,-.029)),
        Solid('box',(.102,.115,.058),(0.,0.,.004),color=BLACK),
        Solid('box',(.072,.064,.010),(0.,0.,BED_Z-FRAME.pos[2]-.005),color=SILVER),
        Solid('box',(.006,.030,BASE_H),(0.,0.,BED_Z-FRAME.pos[2]+BASE_H/2.),color=BLACK),
        Solid('box',(.050,.039,.174),(.098,0.,.051),color=STEEL),
        Solid('box',(.074,.046,.025),(.067,0.,PRESS_PIVOT[2]-FRAME.pos[2]),color=STEEL),
        Solid('cylinder',(.011,.101),(.067,-.035,PRESS_PIVOT[2]-FRAME.pos[2]),axis='Y',color=SILVER),
    ),
    HANDLE.name:(
        Solid('cylinder',(.010,.192),(-.096,HANDLE_Y,0.),axis='X',color=SILVER),
        Solid('cylinder',(.015,.064),tuple(HANDLE_LOCAL),axis='X',color=BLACK),
        Solid('cylinder',(.009,abs(HANDLE_Y)),(0.,HANDLE_Y/2.,0.),axis='Y',color=STEEL),
        Solid('box',(.068,.025,.022),(-.034,0.,-.003),color=STEEL),
        Solid('cylinder',(.008,.033),(-.065,0.,-.0095),axis='Y',color=SILVER),
    ),
    RAM.name:(
        Solid('cylinder',(.0105,RAM_TOP-RAM_BOTTOM-.013),(0.,0.,.0065)),
        Solid('box',(.038,.043,.010),(0.,0.,(RAM_BOTTOM-RAM_TOP)/2.+.005),color=STEEL),
        Solid('box',(.029,.056,.012),(0.,0.,(RAM_TOP-RAM_BOTTOM)/2.-.006),color=SILVER),
    ),
}
VISUAL_PARTS={
    cable.name:(Solid('box',(length-.001,.0016,.00012),(0.,RIBBON_W/2.-.001,.00066),color=RED),)
    for cable,length in ((CABLE1,LEN_FIRST),(CABLE2,LEN_SECOND))
}
VISUAL_PARTS[FRAME.name]=(Solid('box',(.003,.003,.0004),(0.,.013,BED_Z-FRAME.pos[2]+BASE_H+.00021),color=BRASS),)
SUPPORT_JOINTS={FRAME.name:{'kind':'fixed'},TESTER.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    HANDLE.name:{'kind':'revolute','parent':FRAME.name,'axis':'Y','anchor':tuple(PRESS_PIVOT),
                 'lower':-14.3,'upper':3.,'drive':{'stiffness':1.2,'target':0.,'damping':.046,'max_force':.65}},
    RAM.name:{'kind':'prismatic','parent':FRAME.name,'axis':'Z','anchor':RAM.pos,'collision_enabled':False,
              'lower':-.012,'upper':.0002,'drive':{'stiffness':820.,'target':0.,'damping':7.,'max_force':15.}},
    CAP.name:{'kind':'prismatic','parent':FRAME.name,'axis':'Z','anchor':CAP.pos,'collision_enabled':False,
              'lower':-STROKE,'upper':.00015,'drive':{'stiffness':8500.,'target':0.,'damping':8.,'max_force':42.}},
    CABLE1.name:{'kind':'revolute','parent':TESTER.name,'axis':'Z','anchor':tuple(ANCHOR),
                 'lower':-75.,'upper':75.,'drive':{'stiffness':.0008,'damping':.00064,'max_force':.012}},
    CABLE2.name:{'kind':'revolute','parent':CABLE1.name,'axis':'Z','anchor':tuple(MID),
                 'lower':-79.,'upper':79.,'drive':{'stiffness':.0008,'damping':.00064,'max_force':.012}},
}
CONTACT_OFFSETS_M={c.name:.00016 for c in (CABLE1,CABLE2,CAP,RAM)}
RIGID_SOLVER_ITERATIONS={c.name:(64,8) for c in (CABLE1,CABLE2,CAP,RAM,HANDLE)}
SURFACE_FRICTION={CABLE2.name:(.68,.54),HANDLE.name:(.84,.74),RAM.name:(.20,.15)}
OBJECT_MATERIALS={FRAME.name:'steel press frame and stock socket locator',
                  HANDLE.name:'steel lever with rubber sleeve', RAM.name:'steel ram and flat platen',
                  CAP.name:'glass-filled IDC housing',TESTER.name:'low-voltage tester housing',
                  CABLE1.name:'PVC-insulated twenty-way flat ribbon',CABLE2.name:'PVC-insulated twenty-way flat ribbon',
                  DECK.name:'steel harness bench riser with a rubber top'}

capture_position=np.array((.130,.245,.383))
capture_forward=np.array((-.038,-.049,.074))-capture_position
capture_forward/=np.linalg.norm(capture_forward)
capture_right=np.cross(capture_forward,(0.,0.,1.))
capture_right/=np.linalg.norm(capture_right)
capture_rotation=np.column_stack((capture_right,np.cross(capture_right,capture_forward),-capture_forward))
TASK_RECORD_CAMERA=(FRAME.name,Camera(tuple(capture_position),
                                       tuple(matrix_to_quat_xyzw(capture_rotation)),
                                       'opengl',Lens.from_focal(30.,640,360)))
LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
CONTACT_Y=PITCH*(np.arange(20)-9.5)
RESET_ROCKER_Z=PRESS_PIVOT[2]
_registration_trim=np.zeros(2)
_heading_trim=0.
_push_began=None
_hold_position=None


def on_reset():
    global _heading_trim,_push_began,_hold_position
    _registration_trim.fill(0.)
    _heading_trim=0.
    _push_began=None
    _hold_position=None


def local_cable(objects):
    origin=LIVE_CONTEXT['body_origins'][CABLE2.name]
    rotation=quat_xyzw_to_matrix(objects[CABLE2.name].quat_xyzw)
    return origin,rotation


def registered(objects):
    origin,rotation=local_cable(objects)
    tip=origin+rotation@np.array((LEN_SECOND/2.,0.,0.))
    forward=rotation[:,0]
    if forward[0] < .995 or rotation[2,2]<.985 or not SOCKET_X+.001<tip[0]<SOCKET_X+.011:
        return False
    x=(SOCKET_X-origin[0])/forward[0]
    conductors=origin+np.column_stack((np.full(20,x),CONTACT_Y,np.zeros(20)))@rotation.T
    return bool(np.max(np.abs(conductors[:,1]-SOCKET_Y-CONTACT_Y))<=.00062
                and np.max(np.abs(conductors[:,2]-WIRE_Z))<=.0011)


def success(objects):
    origins=LIVE_CONTEXT['body_origins']
    cap=origins[CAP.name]
    ram=origins[RAM.name]
    normal=quat_xyzw_to_matrix(objects[CAP.name].quat_xyzw)[2,2]
    return bool(registered(objects) and abs(cap[2]-CAP_CLOSE_Z)<=.00045
                and math.dist(cap[:2],(SOCKET_X,SOCKET_Y))<=.00025
                and normal>=math.cos(math.radians(2.)) and RAM_CENTER-ram[2]>=.0094)


def score(objects):
    if success(objects):
        return 1.
    return .5 if registered(objects) else 0.


RIBBON_PINCH=-.008
HANDLE_PINCH=-.011
RIBBON_GRIP_BEHIND_TIP=.072
START_CABLE_GRIP=FRONT-axial(YAW_SECOND,RIBBON_GRIP_BEHIND_TIP)+(0.,0.,.007)
END_CABLE_GRIP=np.array((SOCKET_X+.006-RIBBON_GRIP_BEHIND_TIP,SOCKET_Y,WIRE_Z+.007))
ALIGN_START, ALIGN_DURATION = 3.75, 2.2
START_HANDLE=PRESS_PIVOT+HANDLE_LOCAL+(0.,0.,.003)
PUSH_START=6.6
PUSH_S=1.35


def demo():
    return {
        'left':[
            Waypoint(1.25,tuple(START_CABLE_GRIP+(0.,0.,.067)),FINGER_OPEN,'approach ribbon'),
            Waypoint(1.0,tuple(START_CABLE_GRIP),FINGER_OPEN,'descend to ribbon'),
            Waypoint(.70,tuple(START_CABLE_GRIP),RIBBON_PINCH,'grip ribbon'),
            Waypoint(.80,tuple(START_CABLE_GRIP),RIBBON_PINCH,'settle the grip'),
            Waypoint(1.20,tuple(END_CABLE_GRIP+(-.018,.009,0.)),RIBBON_PINCH,'square the stripe'),
            Waypoint(1.0,tuple(END_CABLE_GRIP),RIBBON_PINCH,'feed under the cover'),
            Waypoint(3.4,tuple(END_CABLE_GRIP),RIBBON_PINCH,'hold ribbon position'),
        ],
        'right':[
            Waypoint(1.6,tuple(START_HANDLE+(0.,0.,.082)),FINGER_OPEN,'approach press'),
            Waypoint(3.45,tuple(START_HANDLE+(0.,0.,.082)),FINGER_OPEN,'wait above press'),
            Waypoint(.90,tuple(START_HANDLE),FINGER_OPEN,'descend to bar'),
            Waypoint(.65,tuple(START_HANDLE),HANDLE_PINCH,'grip bar'),
            Waypoint(PUSH_S,tuple(START_HANDLE),HANDLE_PINCH,'press to shut height'),
            Waypoint(2.4,tuple(START_HANDLE),HANDLE_PINCH,'hold press seated'),
        ],
    }


def demo_target(arm, phase, time_s, position, rotation, finger):
    global _heading_trim,_push_began,_hold_position
    if arm=='left':
        fraction=min(1.,max(0.,(time_s-ALIGN_START)/ALIGN_DURATION))
        registration=fraction*fraction*(3.-2.*fraction)
        yaw=YAW_SECOND*(1.-registration)
        if time_s>=ALIGN_START:
            first_angle=YAW_FIRST+(FINISHED_FIRST-YAW_FIRST)*registration
            position=(ANCHOR+axial(first_angle,LEN_FIRST)
                      +axial(yaw,LEN_SECOND-RIBBON_GRIP_BEHIND_TIP)+(0.,0.,.007))
        if fraction>=1. and 'body_origins' in LIVE_CONTEXT:
            origin,cable_rotation=local_cable(LIVE_OBJECTS)
            tip=origin+cable_rotation[:,0]*LEN_SECOND/2.
            ahead=cable_rotation[:,0]
            center_at_socket=origin+ahead*(SOCKET_X-origin[0])/max(.1,ahead[0])
            error=np.array((FINISHED_TIP[0]-tip[0],SOCKET_Y-center_at_socket[1]))
            if _hold_position is None:
                _registration_trim[:]=np.clip(_registration_trim+.09*error,-.014,.014)
                heading_error=math.atan2(ahead[1],ahead[0])
                _heading_trim=float(np.clip(_heading_trim-.13*heading_error,-.095,.095))
                position[:2]+=_registration_trim
                if time_s>=PUSH_START and abs(error[1])<.00038 and registered(LIVE_OBJECTS):
                    _hold_position=np.asarray(LIVE_CONTEXT['tcp']['left'],dtype=float).copy()
                    _hold_position[:2]+=error
            else:
                position=_hold_position.copy()
                position[:2]+=.55*error
            yaw+=_heading_trim
        return position,rot((0.,0.,1.),yaw)@grip_rotation(),finger
    approach=min(1.,max(0.,(time_s-4.8)/1.0))
    wrist_bias=-.20*approach*approach*(3.-2.*approach)
    if time_s>=PUSH_START:
        if _push_began is None and _hold_position is not None:
            _push_began=time_s
        phase=0. if _push_began is None else min(1.,(time_s-_push_began)/PUSH_S)
        angle=-.233*phase*phase*(3.-2.*phase)
        pitch=rot((0.,1.,0.),angle)
        # Finite position error under the press spring produces the handle load
        # through the standard YAM servo gains.
        wrist=rot((0.,1.,0.),wrist_bias+.55*angle)
        return PRESS_PIVOT+pitch@HANDLE_LOCAL+(.009*phase,0.,.003-.039*phase),wrist@grip_rotation(),finger
    return position,rot((0.,1.,0.),wrist_bias)@rotation,finger
