"""Lift a metal part over the retaining collar on a moving overhead hanger."""

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_overhead_hanger_unloading.geometry import Solid
from dynamic_task_overhead_hanger_unloading.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRACK_X, TRACK_Y = .367, .095
PIN_X, PIN_Z = .308, TOP+.180
PIN_END_X = .249
PIN_RADIUS = .006
COLLAR_RADIUS = .010
FORK_OFFSET = np.array((-.043,0.,.023))
EYE_OFFSET_Z = .069
NOMINAL_GRIP_OFFSET = FORK_OFFSET+np.array((0.,0.,PAD_TIP_PAST_TCP-.004))
RAIL = Box('rail_frame',pos=(TRACK_X,TRACK_Y,TOP+.265),size=(.066,.362,.020),
           color=(.30,.36,.43))
TROLLEY = Box('trolley',pos=(TRACK_X,TRACK_Y,TOP+.230),size=(.038,.042,.027),
              mass=1.5,color=(.18,.30,.44))
PART = Box('workpiece',pos=(.282,TRACK_Y,TOP+.117),size=(.014,.064,.078),
            mass=.088,color=(.84,.35,.13))
CRADLE = Box('unload_cradle',pos=(.202,.235,TOP+.006),size=(.110,.095,.012),
             mass=1.4,color=(.18,.39,.34))
OBJECTS = [RAIL,TROLLEY,PART,CRADLE]
BODY_PARTS = {
    RAIL.name: (
        Solid('box',RAIL.size),
        *(Solid('box',(.032,.029,.265),(.015,y,-.1325)) for y in (-.158,.158)),
        *(Solid('box',(.104,.054,.012),(.010,y,-.259)) for y in (-.158,.158)),
        *(Solid('cylinder',(.0045,.353),(x,0.,-.014),axis='Y',color=(.62,.67,.72))
          for x in (-.021,.021)),
    ),
    TROLLEY.name: (
        Solid('box',TROLLEY.size),
        Solid('box',(.016,.024,.050),(-.003,0.,-.033)),
        Solid('cylinder',(PIN_RADIUS,.118),(PIN_X-TRACK_X,0.,PIN_Z-TROLLEY.pos[2]),axis='X',color=(.62,.66,.71)),
        Solid('cylinder',(COLLAR_RADIUS,.004),(PIN_END_X+.002-TRACK_X,0.,PIN_Z-TROLLEY.pos[2]),axis='X',color=(.77,.78,.79)),
    ),
    PART.name: (
        Solid('box',PART.size),
        Solid('box',(.014,.046,.008),(0.,0.,EYE_OFFSET_Z+.004)),
        *(Solid('box',(.014,.007,.033),(0.,y,.052)) for y in (-.0195,.0195)),
        Solid('box',(.034,.015,.016),(-.019,0.,.023)),
        Solid('box',(.027,.028,.017),tuple(FORK_OFFSET)),
    ),
    CRADLE.name: (
        Solid('box',CRADLE.size),
        *(Solid('box',(.008,.078,.046),(x,0.,.029),color=(.25,.31,.32)) for x in (-.014,.014)),
    ),
}
MOTIONS = {TROLLEY.name: OscillatingPlatform(TROLLEY.pos,(0.,.055,0.),8.)}
SUPPORT_JOINTS = {TROLLEY.name: {'axis':'Y','lower':-.066,'upper':.066}}
CONTACT_OFFSETS_M = {PART.name:.0008,TROLLEY.name:.0008}
CONTACT_PAIRS = {'hanger_contact':(PART.name,TROLLEY.name),'cradle_contact':(PART.name,CRADLE.name)}
SUSPENDED_PAYLOAD_SUPPORTS = {PART.name: dict(body=TROLLEY.name,contact='hanger_contact',
    eye_local_z=EYE_OFFSET_Z,pin_top_local_z=PIN_Z+PIN_RADIUS-TROLLEY.pos[2])}
SUCCESS_HOLD_S = .55
MAX_PARTIAL_SCORE = .5
LIVE_CONTEXT = {}
_caught = False
_lifted = False
_unhooked = False


def on_reset():
    global _caught,_lifted,_unhooked
    _caught = _lifted = _unhooked = False


def handle(state, origin):
    rotation=quat_xyzw_to_matrix(np.asarray(state.quat_xyzw))
    return origin+rotation@FORK_OFFSET


def observe(objects, context, dt):
    global _caught,_lifted,_unhooked
    part=objects[PART.name]
    origin=context['body_origins'][PART.name]
    carrier=context['body_origins'][TROLLEY.name]
    grip=handle(part,origin)
    held=any(context['gripper'][side]<.65 and np.linalg.norm(grip-tcp)<.035
             for side,tcp in context['tcp'].items())
    if held and abs(origin[1]-carrier[1])<.012 and abs(origin[0]-PART.pos[0])<.012:
        if (abs(objects[TROLLEY.name].lin_vel[1])>.020
                and context['contact_forces']['hanger_contact'][2]>.07):
            _caught=True
        if _caught and origin[2]>=PART.pos[2]+.006:
            _lifted=True
    if held and _lifted and part.high[0]<PIN_END_X-.003:
        _unhooked=True


def success(objects):
    part=objects[PART.name]
    origin=LIVE_CONTEXT['body_origins'][PART.name]
    rotation=quat_xyzw_to_matrix(np.asarray(part.quat_xyzw))
    return bool(_unhooked and abs(origin[0]-CRADLE.pos[0])<.0085
                and abs(origin[1]-CRADLE.pos[1])<.011 and abs(part.low[2]-(TOP+.012))<.003
                and rotation[2,2]>.970 and part.resting()
                and LIVE_CONTEXT['contact_forces']['cradle_contact'][2]>.15
                and all(np.linalg.norm(handle(part,origin)-tcp)>.055 for tcp in LIVE_CONTEXT['tcp'].values()))


def score(objects):
    if objects[PART.name].low[2]<TOP-.020:
        return 0.
    if success(objects):
        return 1.
    return .5 if _unhooked else .25 if _caught else 0.


def demo():
    x,y,z=np.asarray(PART.pos)+NOMINAL_GRIP_OFFSET
    above=(x,y,z+.034)
    grip=(x,y,z)
    lifted=(x,y,z+.009)
    pulled=(.181,y,lifted[2])
    dock_x=CRADLE.pos[0]+FORK_OFFSET[0]
    dock_z=TOP+.012+PART.size[2]/2+FORK_OFFSET[2]+PAD_TIP_PAST_TCP-.002
    docking=(dock_x,CRADLE.pos[1],dock_z+.040)
    seat=(*docking[:2],dock_z)
    return {'left':[
        Waypoint(2.05,above,OPEN,'approach moving workpiece'),
        Waypoint(1.15,grip,OPEN,'track the grip flange'),
        Waypoint(.65,grip,CLOSED,'grasp on the moving hanger'),
        Waypoint(.8,lifted,CLOSED,'lift clear of the collar'),
        Waypoint(1.45,pulled,CLOSED,'withdraw from the pin'),
        Waypoint(1.75,docking,CLOSED,'carry to the unload cradle'),
        Waypoint(1.1,seat,CLOSED,'seat upright'),
        Waypoint(.55,seat,OPEN,'release in the cradle'),
        Waypoint(1.05,(dock_x,CRADLE.pos[1],dock_z+.093),OPEN,'move clear'),
    ]}


GRASP_PHASES = {'approach moving workpiece','track the grip flange','grasp on the moving hanger'}
HELD_PHASES = {'lift clear of the collar','withdraw from the pin','carry to the unload cradle','seat upright'}


def demo_rotation(arm,label,t,default):
    if label in HELD_PHASES and LIVE_CONTEXT and LIVE_CONTEXT['gripper'][arm]<.65:
        part=LIVE_OBJECTS[PART.name]
        part_rotation=quat_xyzw_to_matrix(np.asarray(part.quat_xyzw))
        target=part_rotation.T@LIVE_CONTEXT['tcp_rotation'][arm]
        return ik.slerp_matrix(default,target,.70)
    return default


def demo_target(arm,label,t,pos,rotation,finger):
    origin=LIVE_CONTEXT['body_origins'][PART.name]
    part=LIVE_OBJECTS[PART.name]
    if label in GRASP_PHASES:
        scale=ik.smoothstep(float(np.clip(t/1.9,0.,1.)))
        motion=handle(part,origin)-np.asarray(PART.pos)-FORK_OFFSET
        pos=pos+scale*motion
    elif label in HELD_PHASES:
        offset=LIVE_CONTEXT['body_origins'][TROLLEY.name][1]-TRACK_Y
        scale=1.
        if label=='carry to the unload cradle':
            scale=1.-ik.smoothstep(float(np.clip((t-6.10)/1.75,0.,1.)))
        elif label=='seat upright':
            scale=0.
        pos=pos+np.array((0.,scale*offset,0.))
        if LIVE_CONTEXT['gripper'][arm]<.65:
            tcp=LIVE_CONTEXT['tcp'][arm]
            current=LIVE_CONTEXT['tcp_rotation'][arm]
            correction=rotation@current.T@(tcp-origin)-NOMINAL_GRIP_OFFSET
            pos=pos+np.clip(correction,-.012,.012)
    return pos,rotation,finger
