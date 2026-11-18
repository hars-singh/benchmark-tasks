"""Test a guided two-prong pusher on the native camera cable latch.

The bench clip performs a repeatable insertion and pull. The robot's right
gripper presses the passive fork to move the unpowered slider.
"""

import json

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_moving_ffc_connector.ffc_connector import SLOT_DEPTH, connector
from dynamic_task_moving_ffc_connector.geometry import Solid
from dynamic_task_moving_ffc_connector.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
ROBOT_SEATS = True
TOP = TABLE_TOP[SCENE]
BOARD_Z = TOP+.085
MOUTH_X = .282
EDGE_GAP = .0008
TIP_WIDTH = .016
TIP_LENGTH = .007
TIP_Z = BOARD_Z+.00150
TIP_X = MOUTH_X-EDGE_GAP-TIP_LENGTH/2
CLIP_X = TIP_X-.005
PULL_N = .65
PULL_START_S = 6.70
PAD_GAP = .195*FINGER_OPEN
SIDE = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])

BASE = Box('fixture',pos=(.286,0.,TOP+.006),size=(.126,.115,.012),mass=3.,color=(.22,.30,.35))
BOARD = Box('board',pos=(MOUTH_X+.020,0.,BOARD_Z-.0008),size=(.039,.042,.0016),
            mass=.0042,color=(.19,.40,.33))
parts, BODY_PARTS, OBJECT_JOINTS = connector(BOARD,MOUTH_X,BOARD_Z)
HOUSING,GUIDE,WEDGE,FOLLOWER,LEAF = parts
TIP = Box('conductor_tip',pos=(TIP_X,0.,TIP_Z),size=(TIP_LENGTH,TIP_WIDTH,.00030),mass=.000060,
          color=(.72,.55,.31))
CLIP = Box('pull_clip',pos=(CLIP_X,0.,TIP_Z+.0015),size=(.003,.017,.0015),mass=.007,
           color=(.25,.42,.58))
JAW = Box('clip_jaw',pos=(CLIP_X+.002,0.,TIP_Z),size=(.002,.018,.00096),mass=.00020,
          color=(.29,.52,.69))
PUSHER = Box('latch_pusher',pos=(MOUTH_X-.035,0.,WEDGE.pos[2]+.00067),
             size=(.010,.033,.014),mass=.014,color=(.72,.53,.23))
OBJECTS = [BASE,BOARD,*parts,TIP,CLIP,JAW,PUSHER]
BODY_PARTS[BASE.name] = (
    Solid('box',BASE.size),
    *(Solid('cylinder',(.004,.071),(x,y,.0415),color=(.58,.62,.66))
      for x in (.008,.028) for y in (-.015,.015)),
    Solid('box',(.027,.052,.058),(-.025,0.,.035),color=(.25,.29,.31)),
)
BODY_PARTS[JAW.name] = tuple(Solid('box',(.002,.018,.00030),(0.,0.,z)) for z in (-.00033,.00033))
BODY_PARTS[PUSHER.name] = (Solid('box',PUSHER.size),
    *(Solid('box',(.028,.00105,.00105),(.0185,y,0.),color=(.70,.72,.74))
      for y in (-.00955,.00955)))
OBJECT_JOINTS[WEDGE.name]['drive'] = {'stiffness':0.,'damping':1.2,'max_force':1.0}
OBJECT_JOINTS.update({
    BOARD.name:{'kind':'fixed','parent':BASE.name,'anchor':BOARD.pos},
    CLIP.name:{'kind':'prismatic','parent':BASE.name,'anchor':CLIP.pos,'axis':'X',
               'lower':-.009,'upper':.0045,'drive':{'stiffness':12000.,'damping':4.,'max_force':.28}},
    JAW.name:{'kind':'prismatic','parent':CLIP.name,'anchor':JAW.pos,'axis':'Z',
              'lower':-.00018,'upper':.00033,'drive':{'stiffness':100.,'damping':.15,'max_force':.032}},
    TIP.name:{'kind':'fixed','parent':JAW.name,'anchor':TIP.pos},
    PUSHER.name:{'kind':'prismatic','parent':BASE.name,'anchor':PUSHER.pos,'axis':'X',
                 'lower':0.,'upper':.00225,'drive':{'stiffness':70.,'damping':.4,'max_force':.30}},
})
SEATER = Box('cable_seater',pos=(.232,.065,TOP+.119),size=(.012,.028,.014),
             mass=.038,color=(.15,.48,.48))
RAIL = Box('carriage_rails',pos=(BASE.pos[0],0.,TOP+.004),size=(.158,.183,.008),
           color=(.34,.40,.46))
if ROBOT_SEATS:
    OBJECTS.extend((SEATER,RAIL))
    BODY_PARTS[SEATER.name] = (
        Solid('box',SEATER.size),
        Solid('box',(.010,.012,.041),(0.,0.,-.023)),
        Solid('box',(.033,.009,.007),(.015,0.,-.044)),
        Solid('box',(.009,.067,.006),(.024,-.0325,-.044)),
        Solid('box',(.021,.009,.006),(.032,-.065,-.044)),
    )
    BODY_PARTS[RAIL.name] = tuple(
        Solid('box',(.009,RAIL.size[1],.008),(side*.072,0.,0.)) for side in (-1,1)
    )
    OBJECT_JOINTS[SEATER.name] = {'kind':'prismatic','parent':BASE.name,'anchor':SEATER.pos,'axis':'X',
        'lower':0.,'upper':.00420,'drive':{'stiffness':0.,'damping':1.2,'max_force':.30}}
    OBJECT_JOINTS[CLIP.name].update(parent=SEATER.name,lower=-.012,upper=.0005)
    MOTIONS = {BASE.name:OscillatingPlatform(BASE.pos,(0.,.016,0.),10.)}
    SUPPORT_JOINTS = {BASE.name:{'axis':'Y','lower':-.022,'upper':.022}}
CONTACT_OFFSETS_M = {o.name:.000045 for o in OBJECTS if o not in (BASE,BOARD)}
RIGID_SOLVER_ITERATIONS = {o.name:(128,4) for o in OBJECTS}
CONTACT_PAIRS = {'tip_contact':(TIP.name,LEAF.name),'tip_follower':(TIP.name,FOLLOWER.name),
                 'follower_wedge':(FOLLOWER.name,WEDGE.name),'cam_on_guide':(WEDGE.name,GUIDE.name),
                 'fork_to_slider':(PUSHER.name,WEDGE.name)}
LIVE_CONTEXT = {}
SUCCESS_HOLD_S = .45
MAX_PARTIAL_SCORE = .70
_clip_drive = None
_seated = False
_closed = False
_proof_started = None


def on_reset():
    global _clip_drive,_seated,_closed,_proof_started
    _clip_drive = None
    _seated = _closed = False
    _proof_started = None if ROBOT_SEATS else PULL_START_S


def relative_stroke(piece, context):
    origins = context['body_origins']
    return float((origins[piece.name][0]-origins[BOARD.name][0])-(piece.pos[0]-BOARD.pos[0]))


def hands_clear(context):
    return all(np.linalg.norm(tcp-context['body_origins'][piece.name])>.057
               for tcp in context['tcp'].values() for piece in (SEATER,PUSHER))


def observe(objects, context, dt):
    global _seated,_closed,_proof_started
    insertion = relative_stroke(TIP,context)-EDGE_GAP
    stroke = relative_stroke(WEDGE,context)
    seated_by_robot = (ROBOT_SEATS and relative_stroke(SEATER,context)>.0036
        and abs(objects[BASE.name].lin_vel[1])>.006
        and np.linalg.norm(context['tcp']['left']-context['body_origins'][SEATER.name])<.040)
    if insertion>=.003 and (not ROBOT_SEATS or seated_by_robot):
        _seated = True
    pressing = not ROBOT_SEATS or abs(context['contact_forces']['fork_to_slider'][0])>.05
    if (_seated and pressing and stroke>=.0013 and context['contact_forces']['tip_follower'][2]>.25
            and context['contact_forces']['tip_contact'][2]<-.25):
        _closed = True
    if ROBOT_SEATS and _closed and _proof_started is None and hands_clear(context):
        _proof_started = context['time_s']
        print('FFC_PROOF_START '+json.dumps({'time_s':_proof_started,'pull_limit_N':PULL_N}),flush=True)
    if round(context['time_s']*20)%10==0:
        print('FFC_MANUAL_LATCH '+json.dumps({'time_s':context['time_s'],'insertion_mm':insertion*1000,
              'slider_stroke_mm':stroke*1000,'pusher_stroke_mm':1000*relative_stroke(PUSHER,context),
              'pusher_force_N':float(np.linalg.norm(context['contact_forces']['fork_to_slider'])),
              'closed_while_loaded':_closed}),flush=True)


def update_materials():
    global _clip_drive
    if _clip_drive is None:
        import omni.usd
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        candidates = [p for p in stage.Traverse() if p.GetName()=='pull_clip_joint']
        if len(candidates)!=1:
            raise RuntimeError('expected one insertion test clip')
        _clip_drive = UsdPhysics.DriveAPI(candidates[0],'linear')
    time = LIVE_CONTEXT['time_s']
    stroke = 0. if ROBOT_SEATS else ik.smoothstep(min(max(time-1.,0.),1.))*(SLOT_DEPTH+.00074)
    if _proof_started is not None and time>=_proof_started:
        _clip_drive.GetMaxForceAttr().Set(PULL_N)
        stroke-=ik.smoothstep(min(time-_proof_started,1.))*.011
    _clip_drive.GetTargetPositionAttr().Set(stroke)


def success(objects):
    c = LIVE_CONTEXT
    clear = hands_clear(c) if ROBOT_SEATS else np.linalg.norm(c['tcp']['right']-c['body_origins'][WEDGE.name])>.055
    return bool(_closed and _proof_started is not None and c['time_s']>=_proof_started+1.6
                and relative_stroke(TIP,c)-EDGE_GAP>=.0028 and relative_stroke(WEDGE,c)>=.0013 and clear)


def score(objects):
    return 1. if success(objects) else .70 if _closed else 0.


def demo_rotation(arm,label,t,default):
    initial = ik.tcp_pose(np.asarray(READY),station.BASES[arm])[1]
    return ik.slerp_matrix(initial,SIDE,ik.smoothstep(min(t/2.,1.)))


def demo():
    face = PUSHER.pos[0]-PUSHER.size[0]/2
    z = PUSHER.pos[2]
    wait = (face-PAD_TIP_PAST_TCP-.020,0.,z+.028)
    ready = (face-PAD_TIP_PAST_TCP-.004,0.,z)
    overtravel=.007 if ROBOT_SEATS else .0038
    press = (face-PAD_TIP_PAST_TCP+overtravel,0.,z)
    clear = (face-.075,0.,z+.028)
    right = [Waypoint(2.35,wait,PAD_GAP,'approach camera')]
    if ROBOT_SEATS:
        right.append(Waypoint(1.4,wait,PAD_GAP,'wait for insertion'))
    right.extend([Waypoint(.9,ready,PAD_GAP,'line up both tabs'),
                  Waypoint(1.35,press,PAD_GAP,'push both tabs'),
                  Waypoint(.60 if ROBOT_SEATS else .55,press,PAD_GAP,'seat latch'),
                  Waypoint(1.30,clear,PAD_GAP,'release latch'),
                  Waypoint(3.,clear,PAD_GAP,'wait for proof pull')])
    if not ROBOT_SEATS:
        return {'right':right}
    x,y,z=SEATER.pos
    face=x-SEATER.size[0]/2-PAD_TIP_PAST_TCP
    high=(face-.020,y,z+.031)
    start=(face-.004,y,z)
    forward=(face+.008,y,z)
    clear_left=(face-.045,y+.088,z+.029)
    left=[Waypoint(2.4,high,PAD_GAP,'approach cable slider'),
          Waypoint(.8,start,PAD_GAP,'line up cable slider'),
          Waypoint(.9,forward,PAD_GAP,'insert from slider'),
          Waypoint(2.55,forward,PAD_GAP,'support the inserted cable'),
          Waypoint(1.30,clear_left,PAD_GAP,'release cable slider'),
          Waypoint(3.,clear_left,PAD_GAP,'wait for proof pull')]
    return {'left':left,'right':right}


def demo_target(arm,label,t,pos,rotation,finger):
    if not ROBOT_SEATS or label in {'wait for proof pull','done'}:
        return pos,rotation,finger
    offset=LIVE_CONTEXT['body_origins'][BASE.name][1]-BASE.pos[1]
    weight=ik.smoothstep(min(t/2.2,1.))
    return pos+np.array((0.,weight*offset,0.)),rotation,finger
