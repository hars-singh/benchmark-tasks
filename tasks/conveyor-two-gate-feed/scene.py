"""Release a queue of conveyor pucks with independently sprung stop levers."""

import math

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_conveyor_two_gate_feed.geometry import Solid
from dynamic_task_conveyor_two_gate_feed.mechanisms import conveyor_frame


SCENE = "station"
RENDER_ANTIALIASING = "DLAA"
TOP = TABLE_TOP[SCENE]
DECK = TOP+.030
TRACK_X = .248
LEAD_Y = -.062
QUEUE_Y = LEAD_Y+.044
OUTLET_Y = -.152
COLLECTION_END = -.307
SPEED = .070
PUCK_RADIUS = .022
PUCK_HEIGHT = .014
PIVOT_Z = DECK+.012
HANDLE_LENGTH = .055
HANDLE_RISE = .039
SUCCESS_HOLD_S = .50
BELT = Box("belt", pos=(TRACK_X,-.039,TOP+.015), size=(.118,.568,.030), mass=5., color=(.07,.10,.12))
CONVEYORS = {BELT.name: (0.,-SPEED,0.)}
MACHINE_VISUALS = {BELT.name: {"kind":"conveyor"}}
_hardware, BODY_PARTS = conveyor_frame(BELT)

GUIDES = [Box(f"puck_guide_{edge}", pos=(TRACK_X+side*.030,-.039,DECK+.005),
              size=(.008,.536,.010), color=(.58,.63,.67))
          for side,edge in ((-1,"west"),(1,"east"))]
END_STOP = Box("collector_stop", pos=(TRACK_X,COLLECTION_END-.006,DECK+.010),
               size=(.065,.012,.020), color=(.24,.32,.38))
PUCKS = [Box(f"puck_{n}", pos=(TRACK_X,QUEUE_Y+.063+n*.051,DECK+PUCK_HEIGHT/2),
             size=(2*PUCK_RADIUS,2*PUCK_RADIUS,PUCK_HEIGHT), mass=.048, color=color)
         for n,color in enumerate(((.86,.46,.13),(.22,.58,.80),(.85,.69,.21)),1)]
CONVEYOR_PAYLOADS = {BELT.name: tuple(puck.name for puck in PUCKS)}
for puck in PUCKS:
    BODY_PARTS[puck.name] = (
        Solid("cylinder", (PUCK_RADIUS,.010), (0.,0.,-.002)),
        Solid("cylinder", (.016,.004), (0.,0.,.005), color=(.48,.52,.56)),
    )


def stop_assembly(name, side, y, handle_color):
    """A paddle rises as the outside of its balance lever is depressed."""
    pivot_x = TRACK_X+side*.058
    base = Box(name+"_base", pos=(pivot_x+side*.040,y,TOP+.005),
               size=(.044,.118,.010), mass=2., color=(.22,.32,.36))
    lever = Box(name, pos=(pivot_x,y,PIVOT_Z), size=(.112,.026,.044), mass=.089, color=(.40,.44,.48))
    BODY_PARTS[base.name] = (
        Solid("box",base.size),
        *(Solid("box",(.021,.014,.040),(side*.011,dy,.025)) for dy in (-.047,.047)),
        Solid("cylinder",(.007,.108),(-side*.040,0.,PIVOT_Z-base.pos[2]),axis="Y"),
        *(Solid("box",(.063,.012,.015),(-side*.017,dy,PIVOT_Z-base.pos[2])) for dy in (-.047,.047)),
    )
    BODY_PARTS[name] = (
        Solid("box",(.054,.011,.008),(side*.027,0.,.003)),
        Solid("box",(.050,.011,.008),(-side*.022,0.,.015)),
        Solid("box",(.012,.011,.022),(0.,0.,.008)),
        Solid("box",(.011,.004,.022),(-side*.041,0.,.0065)),
        Solid("box",(.010,.004,.016),(-side*.041,0.,-.003),color=(.75,.40,.12)),
        Solid("box",(.012,.011,.040),(side*HANDLE_LENGTH,0.,.020)),
        Solid("box",(.024,.024,.008),(side*HANDLE_LENGTH,0.,HANDLE_RISE),color=handle_color),
    )
    joint = {"kind":"revolute","parent":base.name,"anchor":lever.pos,"axis":"Y",
             "lower":0. if side==1 else -43.,"upper":43. if side==1 else 0.,
             "drive":{"stiffness":.007,"damping":.001,"max_force":.55}}
    return base,lever,joint


LEAD_BASE, LEAD, lead_joint = stop_assembly("exit_lever",1,LEAD_Y,(.91,.48,.13))
QUEUE_BASE, QUEUE, queue_joint = stop_assembly("queue_lever",-1,QUEUE_Y,(.20,.59,.79))
OBJECTS = [BELT,*_hardware,*GUIDES,END_STOP,*PUCKS,LEAD_BASE,LEAD,QUEUE_BASE,QUEUE]
OBJECT_JOINTS = {LEAD.name:lead_joint, QUEUE.name:queue_joint}
LIVE_CONTEXT = {}
_pocket = set()
_released = set()
_collected = set()
_last_release = None
_invalid = False


def on_reset():
    global _last_release, _invalid
    _pocket.clear()
    _released.clear()
    _collected.clear()
    _last_release = None
    _invalid = False


def lever_angle(lever, side):
    q = lever.quat_xyzw
    return side*math.degrees(2*math.atan2(q[1],q[3]))


def paddle_lower_edge(pivot_height, opening_degrees):
    angle = math.radians(opening_degrees)
    # Lowest corner of the stop face as its inward end rises about the spindle.
    return pivot_height+.036*math.sin(angle)-.011*math.cos(angle)


def gate_limits(objects, origins):
    return tuple(paddle_lower_edge(origins[gate.name][2],lever_angle(objects[gate.name],side))
                 for gate,side in ((LEAD,1),(QUEUE,-1)))


def observe(objects, context, dt):
    global _last_release, _invalid
    origins = context["body_origins"]
    lead_edge,queue_edge = gate_limits(objects,origins)
    open_edge = DECK+PUCK_HEIGHT+.001
    stop_edge = DECK+PUCK_HEIGHT-.003
    for puck in PUCKS:
        state = objects[puck.name]
        position = origins[puck.name]
        if abs(position[0]-TRACK_X)>.011 or abs(state.low[2]-DECK)>.007:
            _invalid = True
        if puck.name not in _pocket and position[1]<QUEUE_Y-.005:
            if _pocket != _released or queue_edge<open_edge or lead_edge>stop_edge:
                _invalid = True
            _pocket.add(puck.name)
        if puck.name not in _released and position[1]<LEAD_Y-.009:
            time = context["time_s"]
            if (puck.name not in _pocket or lead_edge<open_edge or queue_edge>stop_edge
                    or (_last_release is not None and time-_last_release<1.)):
                _invalid = True
            _released.add(puck.name)
            _last_release = time
        if puck.name in _released and state.high[1]<OUTLET_Y:
            _collected.add(puck.name)


def success(objects):
    both_closed = max(gate_limits(objects,LIVE_CONTEXT['body_origins']))<=DECK+PUCK_HEIGHT-.003
    return bool(not _invalid and len(_collected)==len(PUCKS)
                and both_closed
                and all(objects[p.name].high[1]<OUTLET_Y and objects[p.name].resting() for p in PUCKS))


def score(objects):
    if _invalid:
        return 0.
    if success(objects):
        return 1.
    return .22*len(_collected)


def demo_lever(lever, side, initial_wait):
    x,y,z = lever.pos
    angle = math.radians(40.)
    high = (x+side*HANDLE_LENGTH,y,z+HANDLE_RISE+.004+PAD_TIP_PAST_TCP+.040)
    low = (x+side*(HANDLE_LENGTH*math.cos(angle)+HANDLE_RISE*math.sin(angle)),
           y,z-HANDLE_LENGTH*math.sin(angle)+HANDLE_RISE*math.cos(angle)+PAD_TIP_PAST_TCP-.005)
    waypoints = [Waypoint(2.4,high,FINGER_CLOSED,"approach paddle"),
                 Waypoint(initial_wait,high,FINGER_CLOSED,"wait for queued puck")]
    for _ in PUCKS:
        waypoints.extend((Waypoint(.75,low,FINGER_CLOSED,"press paddle"),
                          Waypoint(.80,low,FINGER_CLOSED,"hold paddle open"),
                          Waypoint(.65,high,FINGER_CLOSED,"allow paddle return"),
                          Waypoint(5.30,high,FINGER_CLOSED,"pause for other gate")))
    return waypoints


def demo():
    return {"left":demo_lever(QUEUE,-1,.7),"right":demo_lever(LEAD,1,4.45)}
