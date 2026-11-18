"""Use a loose minimum-size plug gage to inspect two moving through-bores."""

import math

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_moving_bore_minimum_gage.geometry import Solid
from dynamic_task_moving_bore_minimum_gage.mechanisms import linear_stage
from dynamic_task_moving_bore_minimum_gage.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
GAGE_RADIUS = .006
TIP_OFFSET = .065
TIP_TO_TCP = TIP_OFFSET+PAD_TIP_PAST_TCP-.005
BORE_LENGTH = .019
BORE_TOP = TOP+.068
STAGE = Box('inspection_carriage',pos=(.323,.084,TOP+.015),size=(.120,.157,.030),
            mass=1.8,color=(.20,.30,.43))
GAGE = Box('plug_gage',pos=(.184,.233,TOP+.073),size=(.026,.026,.085),
           mass=.095,color=(.26,.31,.35))
REST = Box('gage_stand',pos=(*GAGE.pos[:2],TOP+.004),size=(.058,.062,.008),color=(.25,.30,.37))
PANEL = Box('result_panel',pos=(.205,-.185,TOP+.017),size=(.099,.160,.034),
            mass=1.8,color=(.17,.20,.24))
RINGS = {
    'ivory':Box('ivory_bore',pos=(STAGE.pos[0],STAGE.pos[1]+.047,BORE_TOP-BORE_LENGTH/2),
                size=(.031,.031,BORE_LENGTH),mass=.043,color=(.72,.67,.54)),
    'blue':Box('blue_bore',pos=(STAGE.pos[0],STAGE.pos[1]-.047,BORE_TOP-BORE_LENGTH/2),
               size=(.031,.031,BORE_LENGTH),mass=.047,color=(.16,.42,.68)),
}
BUTTONS = {name:Box(f'{name}_button',pos=(PANEL.pos[0],PANEL.pos[1]+side*.041,TOP+.040),
                   size=(.033,.031,.018),mass=.032,color=ring.color)
           for side,(name,ring) in zip((1,-1),RINGS.items())}
hard, BODY_PARTS = linear_stage(STAGE,.038)
OBJECTS = [STAGE,GAGE,REST,PANEL,*RINGS.values(),*BUTTONS.values(),*hard]


def annulus(inner_radius,thickness,length,segments=24,z=0.):
    radius=inner_radius+thickness/2
    width=2*(inner_radius+thickness)*math.tan(math.pi/segments)+.0004
    for step in range(segments):
        angle=math.tau*step/segments
        yield Solid('box',(thickness,width,length),
                    (radius*math.cos(angle),radius*math.sin(angle),z),yaw=angle)


BODY_PARTS[STAGE.name] += tuple(
    Solid('box',(.012,.027,.019),(side*.019,ring.pos[1]-STAGE.pos[1],.0245))
    for ring in RINGS.values() for side in (-1,1)
)
BODY_PARTS.update({
    GAGE.name:(Solid('cylinder',(.0125,.040)),
               Solid('cylinder',(GAGE_RADIUS,.050),(0.,0.,-.040),color=(.70,.74,.78))),
    REST.name:(Solid('box',REST.size),
               *(Solid('box',(.008,.026,.039),(sign*.012,0.,.0235)) for sign in (-1,1)),
               *(Solid('box',(.016,.008,.039),(0.,sign*.012,.0235)) for sign in (-1,1)),
               *annulus(.0080,.0085,.008,24,.0454)),
    PANEL.name:(Solid('box',PANEL.size),
                Solid('box',(.024,.160,.010),(.037,0.,.0195),color=(.28,.32,.36))),
    RINGS['ivory'].name:tuple(annulus(.0078,.0060,BORE_LENGTH)),
    RINGS['blue'].name:tuple(annulus(.0051,.0087,BORE_LENGTH)),
})
OBJECT_JOINTS = {ring.name:{'kind':'fixed','parent':STAGE.name,'anchor':ring.pos} for ring in RINGS.values()}
OBJECT_JOINTS.update({button.name:{'kind':'prismatic','parent':PANEL.name,'anchor':button.pos,
    'axis':'Z','lower':-.008,'upper':0.,'drive':{'stiffness':750.,'damping':2.,'max_force':8.}}
    for button in BUTTONS.values()})
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,.038,0.),7.5)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.053,'upper':.053}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':.038}}
CONTACT_PAIRS = {name:(GAGE.name,ring.name) for name,ring in RINGS.items()}
CONTACT_PAIRS['returned_gage']=(GAGE.name,REST.name)
CONTACT_OFFSETS_M = {name:.00035 for name in (GAGE.name,*(ring.name for ring in RINGS.values()),REST.name)}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,4) for obj in OBJECTS if obj.free}
SUCCESS_HOLD_S = .50
MAX_PARTIAL_SCORE = .60
LIVE_CONTEXT = {}
_hold = {}
_proved = set()
_decision = None
_demo_return_seated = False
IVORY_APPROACH_S = 4.65
BLUE_APPROACH_S = 8.65


def on_reset():
    global _decision, _demo_return_seated
    _hold.clear()
    _proved.clear()
    _decision=None
    _demo_return_seated=False


def observe(objects,context,dt):
    global _decision
    origins=context['body_origins']
    gage=objects[GAGE.name]
    origin=origins[GAGE.name]
    axis=quat_xyzw_to_matrix(np.asarray(gage.quat_xyzw))[:,2]
    tip=origin-TIP_OFFSET*axis
    held=any(context['gripper'][arm]<.60 and math.dist(origin,tcp)<.040 for arm,tcp in context['tcp'].items())
    for name,ring in RINGS.items():
        center=origins[ring.name]
        depth=center[2]+BORE_LENGTH/2-tip[2]
        force=context['contact_forces'][name][2]
        aligned=held and axis[2]>.996 and math.dist(tip[:2],center[:2])<.0016
        moving=abs(objects[STAGE.name].lin_vel[1])>=.010
        passed=(depth>=BORE_LENGTH and abs(force)<.55 if name=='ivory'
                else -.0012<=depth<=.0014 and .7<=force<=12.)
        _hold[name]=_hold.get(name,0.)+dt if aligned and moving and passed else 0.
        if _hold[name]>=.20:
            _proved.add(name)
    if len(_proved)!=len(RINGS):
        return
    panel_drop=PANEL.pos[2]-origins[PANEL.name][2]
    selection=[name for name,button in BUTTONS.items() if button.pos[2]-origins[button.name][2]-panel_drop>.004]
    if selection:
        _decision=selection[0] if len(selection)==1 else None


def success(objects):
    if _decision!='blue':
        return False
    context=LIVE_CONTEXT
    gage=objects[GAGE.name]
    origin=context['body_origins'][GAGE.name]
    rotation=quat_xyzw_to_matrix(np.asarray(gage.quat_xyzw))
    return bool(math.dist(origin[:2],REST.pos[:2])<.004 and abs(gage.low[2]-(TOP+.008))<.004
                and rotation[2,2]>.99 and gage.resting() and context['contact_forces']['returned_gage'][2]>.12
                and all(math.dist(origin,tcp)>.060 for tcp in context['tcp'].values()))


def score(objects):
    if success(objects):
        return 1.
    return .30*len(_proved)


def demo():
    travel_z=TOP+.172
    gage_pick=(*GAGE.pos[:2],GAGE.pos[2]+.010)
    gage_high=(*GAGE.pos[:2],travel_z)
    left=[Waypoint(2.,gage_high,OPEN,'approach gage stand'),
          Waypoint(1.15,gage_pick,OPEN,'align around gage'),
          Waypoint(.65,gage_pick,CLOSED,'grip gage'),
          Waypoint(.85,gage_high,CLOSED,'lift gage clear')]
    for name,depth in (('ivory',.0225),('blue',.0035)):
        ring=RINGS[name]
        top=(*ring.pos[:2],travel_z if name=='ivory' else TOP+.151)
        seat=(*top[:2],BORE_TOP+TIP_TO_TCP-depth)
        left.extend([Waypoint(1.1,top,CLOSED,f'approach {name} bore'),
                     Waypoint(.45,top,CLOSED,f'align over {name} bore'),
                     Waypoint(1.0,seat,CLOSED,f'test {name} bore'),
                     Waypoint(.75,seat,CLOSED,f'hold at {name} bore'),
                     Waypoint(.7,top,CLOSED,f'withdraw from {name} bore')])
    stand_seat=(*gage_pick[:2],TOP+.0084+TIP_TO_TCP)
    left.extend([Waypoint(1.15,gage_high,CLOSED,'return to gage stand'),
                 Waypoint(.70,gage_high,CLOSED,'align above gage stand'),
                 Waypoint(1.10,stand_seat,CLOSED,'seat gage in stand'),
                 Waypoint(.70,stand_seat,CLOSED,'settle gage in stand'),
                 Waypoint(.80,stand_seat,OPEN,'release gage'),
                 Waypoint(1.15,gage_high,OPEN,'move left hand clear')])
    button=BUTTONS['blue']
    above=(*button.pos[:2],TOP+.145)
    down=(*button.pos[:2],button.pos[2]+button.size[2]/2+PAD_TIP_PAST_TCP-.010)
    right=[Waypoint(2.5,above,CLOSED,'approach panel'),Waypoint(10.6,above,CLOSED,'wait for bore checks'),
           Waypoint(1.,down,CLOSED,'press blue button'),Waypoint(.5,down,CLOSED,'hold blue button'),
           Waypoint(1.15,above,CLOSED,'move right hand clear')]
    return {'left':left,'right':right}


def demo_rotation(arm,label,t,default):
    if arm!='left' or not LIVE_CONTEXT or LIVE_CONTEXT['gripper'][arm]>=.65:
        return default
    axis=quat_xyzw_to_matrix(np.asarray(LIVE_OBJECTS[GAGE.name].quat_xyzw))[:,2]
    if axis[2]<.85:
        return default
    x,y,_=np.cross(axis,(0.,0.,1.))
    skew=np.array(((0.,0.,y),(0.,0.,-x),(-y,x,0.)))
    upright=np.eye(3)+skew+(skew@skew)/(1.+axis[2])
    return ik.slerp_matrix(default,upright@LIVE_CONTEXT['tcp_rotation'][arm],.76)


def demo_target(arm,label,t,pos,rotation,finger):
    global _demo_return_seated
    if arm!='left' or not LIVE_CONTEXT or LIVE_CONTEXT['gripper'][arm]>=.65:
        return pos,rotation,finger
    tracking=next((key for key in RINGS if label.endswith(f'{key} bore')),None)
    stand_descent=label in {'seat gage in stand','settle gage in stand','release gage'}
    returning=stand_descent or label in {'return to gage stand','align above gage stand'}
    if tracking is None and not returning:
        return pos,rotation,finger
    gage=LIVE_CONTEXT['body_origins'][GAGE.name]
    axis=quat_xyzw_to_matrix(np.asarray(LIVE_OBJECTS[GAGE.name].quat_xyzw))[:,2]
    tip=gage-TIP_OFFSET*axis
    wrist_from_tip=LIVE_CONTEXT['tcp'][arm]-tip
    correction=rotation@LIVE_CONTEXT['tcp_rotation'][arm].T@wrist_from_tip-np.array((0.,0.,TIP_TO_TCP))
    adjusted=pos.copy()
    adjusted+=np.clip(correction,-.018,.018)
    if tracking is not None:
        ring=RINGS[tracking]
        sleeve=LIVE_CONTEXT['body_origins'][ring.name]
        phase=IVORY_APPROACH_S if tracking=='ivory' else BLUE_APPROACH_S
        blend=ik.smoothstep(float(np.clip((t-phase)/1.10,0.,1.))) if label.startswith('approach') else 1.
        velocity=LIVE_OBJECTS[STAGE.name].lin_vel[1]
        error=sleeve[:2]-tip[:2]
        adjusted+=blend*(sleeve-np.asarray(ring.pos)+np.array((0.,.095*velocity,0.)))
        if not label.startswith('approach'):
            adjusted[:2]+=.70*np.clip(error,-.004,.004)
        mouth=sleeve[2]+BORE_LENGTH/2
        needs_alignment=np.linalg.norm(error)>.00145 or axis[2]<.996
        if (label.startswith(('test','hold')) and needs_alignment and tip[2]>mouth-.001):
            adjusted[2]=max(adjusted[2],mouth+TIP_TO_TCP+correction[2]+.0018)
    elif stand_descent:
        if _demo_return_seated and label=='release gage':
            return adjusted,rotation,finger
        error=np.asarray(REST.pos[:2])-tip[:2]
        adjusted[:2]+=.80*np.clip(error,-.005,.005)
        mouth=TOP+.0534
        needs_alignment=np.linalg.norm(error)>.0014 or axis[2]<.998
        if needs_alignment and tip[2]>mouth-.001:
            adjusted[2]=max(adjusted[2],mouth+TIP_TO_TCP+correction[2]+.004)
        if label=='release gage':
            _demo_return_seated=bool(
                abs(tip[2]-(TOP+.008))<.002
                and np.linalg.norm(error)<.0025
                and LIVE_CONTEXT['contact_forces']['returned_gage'][2]>.12
            )
            if not _demo_return_seated:
                finger=CLOSED
    return adjusted,rotation,finger
