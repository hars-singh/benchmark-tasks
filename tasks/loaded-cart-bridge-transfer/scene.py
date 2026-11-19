"""Seat a bridge plate and roll a loaded cart across a gap in its support."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_loaded_cart_bridge_transfer.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRACK_X = .296
DECK = TOP+.066
GAP = .058
LEDGE_OUTER = .057
PLATE_LENGTH = .094
WHEEL_RADIUS = .019
AXLE_HALF_SPAN = .040
WHEEL_TRACK = .037
PARK_Y = -.151
FINISH_Y = .112
PLATE_THICKNESS = .007
TOOL_REST = (.120,.142,TOP+.0295)
PLATE_HOME = (*TOOL_REST[:2],TOOL_REST[2]+PLATE_THICKNESS/2)
PLATE_CENTER = (TRACK_X,0.,DECK-PLATE_THICKNESS/2)
PLATE_GRIP_TCP = .022
CART_KNOB_TCP = (0.,-.077,.005)
REQUIRE_HANDLE_CLEARANCE = True
DECK_WIDTH = .196
DECKS = tuple(Box(name,pos=(TRACK_X,sign*.140,TOP+.030),size=(DECK_WIDTH,.223,.060),
                  color=(.25,.33,.40)) for name,sign in (('entry_deck',-1.),('exit_deck',1.)))
STOP = Box('cart_stop',pos=(TRACK_X,.169,DECK+.020),size=(.158,.012,.040),color=(.23,.41,.44))
REST = Box('bridge_rest',pos=(*TOOL_REST[:2],TOP+.007),size=(.131,.104,.014),color=(.18,.25,.33))
PLATE = Box('bridge_plate',pos=PLATE_HOME,size=(.124,PLATE_LENGTH,PLATE_THICKNESS),
            mass=.078,color=(.72,.52,.23))
CART = Box('loaded_cart',pos=(TRACK_X,PARK_Y,DECK+.044),size=(.104,.107,.015),
           mass=.177,color=(.25,.43,.54))
AXLES = tuple(Box(name,pos=(TRACK_X,PARK_Y+offset,DECK+WHEEL_RADIUS),
                  size=(.096,.038,.038),mass=.027,color=(.17,.19,.22))
             for name,offset in (('front_axle',AXLE_HALF_SPAN),('rear_axle',-AXLE_HALF_SPAN)))
CARGO = Box('cart_load',pos=(TRACK_X,PARK_Y,DECK+.052+.020),size=(.058,.055,.040),
            mass=.092,color=(.65,.47,.31))
OBJECTS = [*DECKS,STOP,REST,PLATE,CART,*AXLES,CARGO]


def deck_solids(body, side):
    ledge_length = LEDGE_OUTER-GAP/2
    high_length = .223-ledge_length
    yield Solid('box',(DECK_WIDTH,.223,.058),color=(.15,.24,.31))
    yield Solid('box',(DECK_WIDTH,high_length,.007),
                (0.,side*(LEDGE_OUTER+high_length/2)-body.pos[1],DECK-body.pos[2]-.0035))
    yield Solid('box',(DECK_WIDTH,ledge_length,.008),
                (0.,side*(GAP/2+ledge_length/2)-body.pos[1],DECK-body.pos[2]-PLATE_THICKNESS-.004))
    for sign in (-1.,1.):
        yield Solid('box',(.008,.190,.009),(sign*.088,0.,DECK-body.pos[2]+.003),
                    color=(.55,.59,.63))


BODY_PARTS = {body.name:tuple(deck_solids(body,side)) for body,side in zip(DECKS,(-1.,1.),strict=True)}
BODY_PARTS.update({
    REST.name:(Solid('box',REST.size),
               *(Solid('box',(.020,.095,.015),(x,0.,.015)) for x in (-.051,.051))),
    PLATE.name:(
        Solid('box',PLATE.size),
        *(Solid('box',(.014,.048,.011),(x,0.,-.009)) for x in (-.030,.030)),
        Solid('box',(.016,.023,.014),(0.,0.,.009),color=(.83,.65,.34)),
    ),
    CART.name:(
        Solid('box',CART.size),
        *(Solid('box',(.008,.078,.017),(x,0.,.010)) for x in (-.044,.044)),
        *(Solid('box',(.080,.008,.017),(0.,y,.010)) for y in (-.047,.047)),
        Solid('box',(.025,.050,.014),(0.,-.065,-.006)),
        Solid('box',(.022,.022,.023),(CART_KNOB_TCP[0],CART_KNOB_TCP[1],-.006),
              color=(.78,.53,.23)),
    ),
    **{axle.name:(
        Solid('cylinder',(.005,.086),axis='X'),
        *(Solid('cylinder',(WHEEL_RADIUS,.016),(x,0.,0.),axis='X') for x in (-WHEEL_TRACK,WHEEL_TRACK)),
      ) for axle in AXLES},
})
OBJECT_JOINTS = {axle.name:{'kind':'revolute','parent':CART.name,'anchor':axle.pos,'axis':'X',
                            'drive':{'stiffness':0.,'damping':.000045,'max_force':.012}}
                 for axle in AXLES}
CONTACT_OFFSETS_M = {body.name:.00023 for body in (PLATE,CART,*AXLES,*DECKS,CARGO)}
CONTACT_PAIRS = {'bridge_entry':(PLATE.name,DECKS[0].name),'bridge_exit':(PLATE.name,DECKS[1].name),
                 'load_on_cart':(CARGO.name,CART.name),
                 **{axle.name+'_on_entry':(axle.name,DECKS[0].name) for axle in AXLES},
                 **{axle.name+'_on_bridge':(axle.name,PLATE.name) for axle in AXLES},
                 **{axle.name+'_on_exit':(axle.name,DECKS[1].name) for axle in AXLES}}
RIGID_SOLVER_ITERATIONS = {body.name:(96,4) for body in (PLATE,CART,*AXLES,CARGO)}
OBJECT_MATERIALS = {body.name:'aluminum work surface with steel side guides' for body in DECKS}
OBJECT_MATERIALS.update({STOP.name:'elastomer end stop',REST.name:'aluminum plate rest',
                         PLATE.name:'aluminum bridge plate with steel locating tabs',
                         CART.name:'aluminum cart frame',CARGO.name:'dry polymer test load',
                         **{a.name:'steel axle and elastomer wheels' for a in AXLES}})
SUCCESS_HOLD_S = .55
MAX_PARTIAL_SCORE = .60
LIVE_CONTEXT = {}
_bridged = set()
_seated = False
_failed = False
_seat_offset = np.zeros(3)


def on_reset():
    global _seated,_failed
    _bridged.clear()
    _seated = _failed = False
    _seat_offset[:] = 0.


def observe(objects, context, dt):
    global _seated,_failed
    origin = context['body_origins']
    forces = context['contact_forces']
    plate = origin[PLATE.name]
    cart = origin[CART.name]
    hands = tuple(context['tcp'].values())
    seated = (math.dist(plate,PLATE_CENTER)<.007 and forces['bridge_entry'][2]>.12
              and forces['bridge_exit'][2]>.12 and all(math.dist(hand,plate)>.052 for hand in hands))
    _seated |= bool(seated)
    for axle in AXLES:
        center = origin[axle.name]
        if abs(center[1])<=GAP/2 and abs(center[0]-TRACK_X)<.020:
            if seated and forces[axle.name+'_on_bridge'][2]>.12 and abs(center[2]-DECK-WHEEL_RADIUS)<.0045:
                _bridged.add(axle.name)
            if center[2]<DECK+WHEEL_RADIUS-.010:
                _failed = True
    load = origin[CARGO.name]
    if load[2]<cart[2]-.010 or abs(load[0]-cart[0])>.055 or abs(load[1]-cart[1])>.056:
        _failed = True


def success(objects):
    if _failed or len(_bridged)!=len(AXLES):
        return False
    context = LIVE_CONTEXT
    origin, force = (context[key] for key in ('body_origins','contact_forces'))
    cart, plate = origin[CART.name], origin[PLATE.name]
    frame = quat_xyzw_to_matrix(np.asarray(objects[CART.name].quat_xyzw))
    handle = cart+frame@np.asarray(CART_KNOB_TCP)
    clear = all(math.dist(hand,cart)>.070 and math.dist(hand,plate)>.055
                and math.dist(hand,handle)>.055 for hand in context['tcp'].values())
    return bool(cart[1]>=.090 and abs(cart[0]-TRACK_X)<.018 and math.dist(plate,PLATE_CENTER)<.007
                and min(force[name][2] for name in ('bridge_entry','bridge_exit'))>.12
                and force['load_on_cart'][2]>.15 and all(force[a.name+'_on_exit'][2]>.15 for a in AXLES)
                and math.hypot(*objects[CART.name].lin_vel)<.020 and clear)


def score(objects):
    return 1. if success(objects) else 0. if _failed else .60 if _bridged else .25 if _seated else 0.


def demo_target(arm,label,t,pos,rotation,finger):
    if arm=='left' and label in {'seat both bridge lips','let the bridge settle'}:
        plate = LIVE_CONTEXT['body_origins'][PLATE.name]
        offset = LIVE_CONTEXT['tcp']['left']-plate
        _seat_offset[:2] = np.clip(offset[:2],-.012,.012)
    if arm=='left' and label in {'seat both bridge lips','let the bridge settle','release plate'}:
        pos = pos+_seat_offset
    return pos,rotation,finger


def demo():
    lift = (*PLATE_HOME[:2],TOP+.126)
    pick = (*PLATE_HOME[:2],PLATE_HOME[2]+PLATE_GRIP_TCP)
    over_gap = (TRACK_X,0.,TOP+.126)
    seat = (*PLATE_CENTER[:2],PLATE_CENTER[2]+PLATE_GRIP_TCP)
    left_clear = (.218,.192,TOP+.150)
    held = tuple(a+b for a,b in zip(CART.pos,CART_KNOB_TCP,strict=True))
    hover = (held[0],held[1],held[2]+.030)
    destination = (held[0],held[1]+FINISH_Y-PARK_Y,held[2])
    right_clear = (.233,-.197,TOP+.152)
    return {'left':[
        Waypoint(1.5,lift,OPEN,'approach bridge plate'),
        Waypoint(1.,pick,OPEN,'place fingers on plate tab'),
        Waypoint(.65,pick,CLOSED,'grasp plate tab'),
        Waypoint(.85,lift,CLOSED,'lift bridge plate'),
        Waypoint(1.65,over_gap,CLOSED,'carry plate above gap'),
        Waypoint(1.,seat,CLOSED,'seat both bridge lips'),
        Waypoint(.40,seat,CLOSED,'let the bridge settle'),
        Waypoint(.55,seat,OPEN,'release plate'),
        Waypoint(.7,over_gap,OPEN,'clear plate vertically'),
        Waypoint(1.,left_clear,OPEN,'clear rolling path'),
        Waypoint(11.,left_clear,OPEN,'observe cart crossing'),
    ],'right':[
        Waypoint(2.,hover,OPEN,'approach the cart grip'),
        Waypoint(.85,held,OPEN,'align cart grip'),
        Waypoint(.75,held,CLOSED,'grasp the cart grip'),
        Waypoint(5.85,held,CLOSED,'hold cart for the bridge'),
        Waypoint(8.4,destination,CLOSED,'roll both axles over bridge'),
        Waypoint(.42,destination,CLOSED,'let the cart settle'),
        Waypoint(.85,destination,OPEN,'release cart in destination bay'),
        Waypoint(.9,(.244,destination[1],destination[2]+.044),OPEN,'lift outside the cart lane'),
        Waypoint(.65,(.214,destination[1],destination[2]+.057),OPEN,'clear the cart laterally'),
        Waypoint(1.2,right_clear,OPEN,'withdraw from exit bay'),
        Waypoint(4.,right_clear,OPEN,'observe the loaded cart'),
    ]}
