"""Arm a spring-loaded caster lock, then prove that the wheel still rolls."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_caster_directional_lock.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
RUNNING_SURFACE = TOP+.008
WHEEL_RADIUS = .018
TRACK_X = .297
START_Y = -.073
FINISH_Y = .129
INITIAL_SWIVEL = .58
TRAIL = .021
INDEX_RADIUS = .021
ARM_STROKE = .004
PIN_STROKE = .0155
SEEK_FORCE_N = .47
TOP_OF_INDEX = RUNNING_SURFACE+.044
PIN_GAP = .007
PUSH_TAB = (0.,-.077,.026)
LOCK_PAD = (-.039,0.,.019)
REQUIRE_PARK_HANDLE_CLEAR = True
RUNWAY = Box('test_runway',pos=(TRACK_X,.009,TOP+.004),size=(.227,.427,.008),
             color=(.30,.37,.41))
CART = Box('service_cart',pos=(TRACK_X,START_Y,RUNNING_SURFACE+.056),size=(.119,.137,.017),
           mass=.160,color=(.31,.47,.56))
PIVOT = (TRACK_X,START_Y+.056,RUNNING_SURFACE+.036)
SLEEVE = Box('sprung_caster_mount',pos=PIVOT,size=(.022,.022,.022),mass=.015,
             color=(.44,.48,.51))
FORK = Box('swivel_fork',pos=PIVOT,size=(.052,.057,.041),mass=.036,
           yaw=INITIAL_SWIVEL,color=(.68,.70,.73))
FRONT = Box('caster_wheel',pos=(TRACK_X+math.sin(INITIAL_SWIVEL)*TRAIL,
                              PIVOT[1]-math.cos(INITIAL_SWIVEL)*TRAIL,RUNNING_SURFACE+WHEEL_RADIUS),
            size=(.015,.036,.036),mass=.025,yaw=INITIAL_SWIVEL,color=(.20,.23,.24))
REAR = Box('fixed_axle',pos=(TRACK_X,START_Y-.050,RUNNING_SURFACE+WHEEL_RADIUS),
           size=(.113,.036,.036),mass=.035,color=(.20,.23,.24))
PIN = Box('direction_pin',pos=(TRACK_X,PIVOT[1]+INDEX_RADIUS,TOP_OF_INDEX+PIN_GAP+.010),
          size=(.044,.021,.042),mass=.017,color=(.27,.63,.34))
OBJECTS = [RUNWAY,CART,SLEEVE,FORK,FRONT,REAR,PIN]


def index_ring():
    count = 32
    for index in range(2,count-1):
        angle = math.tau*index/count
        yield Solid('box',(.00416,.008,.009),
                    (INDEX_RADIUS*math.sin(angle),INDEX_RADIUS*math.cos(angle),.0035),
                    yaw=-angle,color=(.51,.55,.59))


BODY_PARTS = {
    CART.name:(
        Solid('box',(.119,.113,.017),(0.,-.012,0.)),
        Solid('box',(.014,.042,.011),(0.,.052,-.005)),
        Solid('cylinder',(.009,.008),(0.,.056,-.014),color=(.55,.57,.61)),
        Solid('box',(.030,.050,.014),(0.,-.065,.005)),
        Solid('box',(.024,.021,.022),(*PUSH_TAB[:2],.015),color=(.75,.55,.26)),
    ),
    SLEEVE.name:(Solid('cylinder',(.011,.010),(0.,0.,.011)),),
    FORK.name:(
        Solid('cylinder',(.015,.010),(0.,0.,-.003)),
        *(Solid('box',(.005,.025,.027),(x,-.016,-.011)) for x in (-.011,.011)),
        *index_ring(),
    ),
    FRONT.name:(Solid('cylinder',(WHEEL_RADIUS,.015),axis='X'),),
    REAR.name:(
        Solid('cylinder',(.004,.098),axis='X'),
        *(Solid('cylinder',(WHEEL_RADIUS,.016),(x,0.,0.),axis='X') for x in (-.047,.047)),
    ),
    PIN.name:(
        Solid('cylinder',(.0031,.020),color=(.62,.66,.69)),
        Solid('box',(.046,.009,.006),(-.017,0.,.008)),
        Solid('box',(.020,.019,.007),(-.039,0.,.0155)),
    ),
}
VISUAL_PARTS = {RUNWAY.name:tuple(
    Solid('box',(.0011,.327,.00014),(x,0.,.00415),color=(.88,.75,.30))
    for x in (-.077,.077)
)}
OBJECT_JOINTS = {
    SLEEVE.name:{'kind':'prismatic','parent':CART.name,'anchor':PIVOT,'axis':'Z',
                 'lower':-.005,'upper':.003,
                 'drive':{'stiffness':470.,'damping':1.8,'max_force':3.2,'target':-.004}},
    FORK.name:{'kind':'revolute','parent':SLEEVE.name,'anchor':PIVOT,'axis':'Z',
               'drive':{'damping':.000018,'max_force':.008}},
    FRONT.name:{'kind':'revolute','parent':FORK.name,'anchor':FRONT.pos,'axis':'X',
                'drive':{'damping':.000012,'max_force':.008}},
    REAR.name:{'kind':'revolute','parent':CART.name,'anchor':REAR.pos,'axis':'X',
               'drive':{'damping':.000012,'max_force':.008}},
    PIN.name:{'kind':'prismatic','parent':CART.name,'anchor':PIN.pos,'axis':'Z',
              'lower':-PIN_STROKE,'upper':0.,
              'drive':{'stiffness':160.,'damping':.9,'max_force':4.,'target':.004}},
}
CONTACT_PAIRS = {'roller_on_runway':(FRONT.name,RUNWAY.name),
                 'rear_on_runway':(REAR.name,RUNWAY.name),
                 'pin_on_index_ring':(PIN.name,FORK.name)}
CONTACT_OFFSETS_M = {body.name:.00023 for body in (CART,SLEEVE,FORK,FRONT,REAR,PIN)}
RIGID_SOLVER_ITERATIONS = {body.name:(96,4) for body in (CART,SLEEVE,FORK,FRONT,REAR,PIN)}
OBJECT_MATERIALS = {RUNWAY.name:'flat aluminum test runway',CART.name:'aluminum cart with polymer grips',
                    SLEEVE.name:'steel spring-loaded caster mount',
                    FORK.name:'steel caster fork and index ring',FRONT.name:'elastomer caster wheel',
                    REAR.name:'steel axle with two elastomer wheels',PIN.name:'steel locking pin and green polymer lever'}
SUCCESS_HOLD_S = .45
MAX_PARTIAL_SCORE = .58
LIVE_CONTEXT = {}
_arm_spring = None
_armed = False
_lock_reference = None
_rolling_travel = 0.


def on_reset():
    global _armed,_lock_reference,_rolling_travel
    _armed = False
    _lock_reference = None
    _rolling_travel = 0.
    if _arm_spring is not None:
        _arm_spring.GetTargetPositionAttr().Set(.004)
        _arm_spring.GetMaxForceAttr().Set(4.)


def observe(objects, context, dt):
    global _armed,_lock_reference,_rolling_travel
    origins = context['body_origins']
    cart, pin = (origins[obj.name] for obj in (CART,PIN))
    depth = (PIN.pos[2]-CART.pos[2])-(pin[2]-cart[2])
    cart_rotation = quat_xyzw_to_matrix(np.asarray(objects[CART.name].quat_xyzw))
    pad = pin+cart_rotation@LOCK_PAD
    _armed |= bool(depth>ARM_STROKE and math.dist(context['tcp']['left'],pad)<.036)
    fork_rotation = quat_xyzw_to_matrix(np.asarray(objects[FORK.name].quat_xyzw))
    relative_yaw = (cart_rotation.T@fork_rotation)[:,1]
    aligned = math.atan2(abs(relative_yaw[0]),relative_yaw[1])<math.radians(11.)
    supported = context['contact_forces']['roller_on_runway'][2]>.15
    wheel = objects[FRONT.name]
    rolling = abs(wheel.ang_vel[0])>.3 and objects[CART.name].lin_vel[1]>.013
    left_clear = math.dist(context['tcp']['left'],pad)>.065
    if _armed and aligned and depth>PIN_GAP+.004 and supported and left_clear:
        if rolling:
            if _lock_reference is None:
                _lock_reference = float(cart[1])
            _rolling_travel = max(_rolling_travel,float(cart[1])-_lock_reference)
    else:
        _lock_reference = None
        _rolling_travel = 0.


def update_materials():
    global _arm_spring
    if _arm_spring is None:
        import omni.usd
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        prim, = (p for p in stage.Traverse() if p.GetName()==PIN.name+'_joint')
        _arm_spring = UsdPhysics.DriveAPI(prim,'linear')
    target = -.018 if _armed else .004
    if _arm_spring.GetTargetPositionAttr().Get()!=target:
        _arm_spring.GetTargetPositionAttr().Set(target)
        _arm_spring.GetMaxForceAttr().Set(SEEK_FORCE_N if _armed else 4.)


def success(objects):
    if _rolling_travel<.089:
        return False
    cart = LIVE_CONTEXT['body_origins'][CART.name]
    forces = LIVE_CONTEXT['contact_forces']
    frame = quat_xyzw_to_matrix(np.asarray(objects[CART.name].quat_xyzw))
    handle = cart+frame@np.asarray(PUSH_TAB)
    return bool(cart[1]>.102 and abs(cart[0]-TRACK_X)<.020
                and math.hypot(*objects[CART.name].lin_vel)<.016
                and forces['rear_on_runway'][2]>.20
                and all(math.dist(tcp,cart)>.070 and math.dist(tcp,handle)>.054
                        for tcp in LIVE_CONTEXT['tcp'].values()))


def score(objects):
    return 1. if success(objects) else .58 if _rolling_travel>.025 else .20 if _armed else 0.


def demo():
    pad = np.asarray(PIN.pos)+LOCK_PAD
    hover = tuple(pad+(0.,0.,.043))
    press = tuple(pad+(0.,0.,PAD-.012))
    left_clear = (.217,.170,TOP+.159)
    hand = tuple(a+b for a,b in zip(CART.pos,PUSH_TAB,strict=True))
    start = (hand[0],hand[1],hand[2]+.030)
    end = (hand[0],hand[1]+FINISH_Y-START_Y,hand[2])
    right_clear = (.226,-.169,TOP+.161)
    return {'left':[
        Waypoint(2.,hover,CLOSED,'approach directional lock'),
        Waypoint(1.7,hover,CLOSED,'wait for cart grip'),
        Waypoint(.75,press,CLOSED,'press the green lock'),
        Waypoint(.55,press,CLOSED,'arm the lock spring'),
        Waypoint(.60,hover,CLOSED,'raise the wrist'),
        Waypoint(.95,left_clear,OPEN,'clear the cart path'),
        Waypoint(12.,left_clear,OPEN,'observe direct travel'),
    ],'right':[
        Waypoint(1.85,start,OPEN,'approach service cart'),
        Waypoint(.91,hand,OPEN,'straddle cart handle'),
        Waypoint(.72,hand,CLOSED,'grasp the cart handle'),
        Waypoint(3.42,hand,CLOSED,'wait for the caster lock'),
        Waypoint(6.,end,CLOSED,'roll the cart down the lane'),
        Waypoint(.40,end,CLOSED,'let the cart settle'),
        Waypoint(.85,end,OPEN,'release the cart'),
        Waypoint(.95,(end[0],end[1],end[2]+.024),OPEN,'lift straight above the cart handle'),
        Waypoint(1.,right_clear,OPEN,'withdraw from lane'),
        Waypoint(4.,right_clear,OPEN,'observe caster locked'),
    ]}
