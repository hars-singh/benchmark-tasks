"""Roll a legible batch code onto a moving carton using a held print wheel."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_moving_carton_roller_coding.geometry import Solid
from dynamic_task_moving_carton_roller_coding.mechanisms import linear_stage
from dynamic_task_moving_carton_roller_coding.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRAVEL = .068
PERIOD = 10.
WHEEL_RADIUS = .0254
WHEEL_WIDTH = .042
HUB = (0.,0.,-.025)
HANDLE_TCP = .027
PRINT_X = .337
PRINT_Y = .091
PITCH = .0040
DOT = .0031
DOT_X = .0040
DOT_COLORS = (.10,.12,.17)
MARK_CENTER = .014
# Seven successive columns read as B7 from the overhead camera.
COLUMNS = ((1,1,1,1,1),(1,0,1,0,1),(0,1,0,1,0),(0,0,0,0,0),
           (1,0,0,0,0),(1,0,1,1,1),(1,1,0,0,0))
NATIVE_PHASES = tuple((i+.5)*PITCH/WHEEL_RADIUS for i in range(len(COLUMNS)))
POSITION_TRACKED_ROLL = True
CRADLE_REACTION_RANGE_N = (.58,1.7)

STAGE = Box('carton_carriage',pos=(.363,.060,TOP+.015),size=(.155,.160,.030),
            mass=2.7,color=(.14,.28,.42))
CARTON = Box('carton',pos=(.359,.060,TOP+.057),size=(.133,.131,.054),
             mass=.13,color=(.76,.58,.35))
REST = Box('coder_rest',pos=(.181,.201,TOP+.0065),size=(.107,.075,.013),
           color=(.18,.27,.32))
TOOL = Box('coder_handle',pos=(*REST.pos[:2],TOP+.081),size=(.086,.049,.061),
           mass=.062,color=(.26,.47,.56))
WHEEL = Box('print_wheel',pos=(TOOL.pos[0],TOOL.pos[1],TOOL.pos[2]+HUB[2]),
            size=(WHEEL_WIDTH,2*WHEEL_RADIUS,2*WHEEL_RADIUS),mass=.026,
            color=(.29,.28,.27))
stock_top = CARTON.pos[2]+CARTON.size[2]/2
ink_height = CARTON.size[2]/2+.00020
mark_x = PRINT_X-CARTON.pos[0]
target_y = tuple(MARK_CENTER+(3-i)*PITCH for i in range(len(COLUMNS)))

hardware, BODY_PARTS = linear_stage(STAGE,TRAVEL)
OBJECTS = [STAGE,CARTON,REST,TOOL,WHEEL,*hardware]
BODY_PARTS.update({
    CARTON.name:(Solid('box',CARTON.size),),
    REST.name:(
        Solid('box',REST.size),
        # Keep the roller clear of the rails when the handle is slightly off center.
        *(Solid('box',(.017,.056,.0305),(side*.039,0.,.02375)) for side in (-1.,1.)),
    ),
    TOOL.name:(
        Solid('box',(.023,.026,.027),(0.,0.,.017)),
        Solid('box',(.086,.030,.008),(0.,0.,-.001)),
        *(Solid('box',(.009,.023,.037),(side*.038,0.,-.017)) for side in (-1.,1.)),
        *(Solid('cylinder',(.005,.008),(side*.038,0.,HUB[2]),axis='X') for side in (-1.,1.)),
    ),
    WHEEL.name:(Solid('cylinder',(WHEEL_RADIUS,WHEEL_WIDTH),axis='X'),),
})


def type_dots():
    for col, cells in enumerate(COLUMNS):
        for row, active in enumerate(cells):
            if active:
                yield col,(row-2)*DOT_X


def carton_visuals():
    top = CARTON.size[2]/2+.00010
    blue = (.18,.31,.37)
    for edge in (-1.,1.):
        yield Solid('box',(.0008,.044,.00012),(mark_x+edge*.014,MARK_CENTER,top),color=blue)
        yield Solid('box',(.028,.0008,.00012),(mark_x,MARK_CENTER+edge*.022,top),color=blue)
    for col, x in type_dots():
        yield Solid('box',(DOT,DOT,.00013),(mark_x+x,target_y[col],top-.001),color=DOT_COLORS)


def wheel_visuals():
    for col, x in type_dots():
        phase = -NATIVE_PHASES[col]
        radius = WHEEL_RADIUS+.00025
        yield Solid('box',(DOT,DOT,.00065),(x,radius*math.sin(phase),-radius*math.cos(phase)),
                    roll=phase,color=DOT_COLORS)


VISUAL_PARTS = {CARTON.name:tuple(carton_visuals()),WHEEL.name:tuple(wheel_visuals())}
OBJECT_JOINTS = {
    CARTON.name:{'kind':'fixed','parent':STAGE.name,'anchor':CARTON.pos},
    WHEEL.name:{'kind':'revolute','parent':TOOL.name,'anchor':WHEEL.pos,'axis':'X',
                'lower':-4.,'upper':135.,
                'drive':{'stiffness':.00017,'damping':.000025,'max_force':.035}},
}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,TRAVEL,0.),PERIOD)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.083,'upper':.083}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':TRAVEL}}
CONTACT_PAIRS = {'type_on_carton':(WHEEL.name,CARTON.name),'coder_on_cradle':(TOOL.name,REST.name)}
CONTACT_OFFSETS_M = {o.name:.00027 for o in (TOOL,WHEEL,CARTON)}
RIGID_SOLVER_ITERATIONS = {o.name:(96,4) for o in (TOOL,WHEEL,CARTON,STAGE)}
OBJECT_MATERIALS = {o.name:'aluminum linear stage hardware' for o in (*hardware,STAGE)}
OBJECT_MATERIALS.update({CARTON.name:'corrugated kraft carton with a printed registration box',
                         TOOL.name:'aluminum frame with polymer grip',REST.name:'polymer cradle',
                         WHEEL.name:'elastomer type on an aluminum print wheel'})
SUCCESS_HOLD_S = .45
MAX_PARTIAL_SCORE = .78
LIVE_CONTEXT = {}
_print_y = {}
_ink = None
_prior_patch = None
_lifted = False
_spoiled = False
_cradle_offset = np.zeros(3)


def on_reset():
    global _ink,_prior_patch,_lifted,_spoiled
    if _ink:
        for _, prim, _ in _ink:
            prim.MakeInvisible()
    _print_y.clear()
    _ink = _prior_patch = None
    _lifted = _spoiled = False
    _cradle_offset[:] = 0.


def observe(objects, context, dt):
    global _prior_patch,_lifted,_spoiled
    origins = context['body_origins']
    tool, wheel, carton = (objects[o.name] for o in (TOOL,WHEEL,CARTON))
    tool_origin, axle, stock = (origins[o.name] for o in (TOOL,WHEEL,CARTON))
    hand = context['tcp']['left']
    held = .04<context['gripper']['left']<.65 and math.dist(hand,tool_origin)<.047
    _lifted |= bool(held and tool_origin[2]>TOOL.pos[2]+.030)
    tool_rotation = quat_xyzw_to_matrix(np.asarray(tool.quat_xyzw))
    wheel_rotation = quat_xyzw_to_matrix(np.asarray(wheel.quat_xyzw))
    relative = tool_rotation.T@wheel_rotation
    phase = math.atan2(relative[2,1],relative[1,1])
    load = context['contact_forces']['type_on_carton'][2]
    patch_y = float(axle[1]-stock[1])
    patch_x = float(axle[0]-stock[0])
    sampling = _lifted and held and load>.60
    valid = (sampling and load<14. and carton.lin_vel[1]>.022
             and min(tool_rotation[2,2],tool_rotation[0,0])>.982 and abs(patch_x-mark_x)<.005)
    if sampling and _prior_patch is not None:
        last_phase, last_y, last_valid = _prior_patch
        travel = last_y-patch_y
        slip = abs(travel-WHEEL_RADIUS*(phase-last_phase))
        rolled = travel>.015*dt and slip<.019*dt
        for col, die in enumerate(NATIVE_PHASES):
            if last_phase<die<=phase:
                alpha = (die-last_phase)/(phase-last_phase)
                location = last_y+alpha*(patch_y-last_y)
                if not (valid and last_valid and rolled) or col in _print_y:
                    _spoiled = True
                else:
                    _print_y[col] = location
            elif phase<=die<last_phase:
                _spoiled = True
    _prior_patch = (phase,patch_y,valid) if sampling else None


def print_coverage():
    return sum(abs(location-target_y[col])<=.0045 for col,location in _print_y.items())/len(COLUMNS)


def success(objects):
    if _spoiled or print_coverage()<1.:
        return False
    origin = LIVE_CONTEXT['body_origins'][TOOL.name]
    upright = quat_xyzw_to_matrix(np.asarray(objects[TOOL.name].quat_xyzw))[2,2]>.986
    low, high = CRADLE_REACTION_RANGE_N
    return bool(math.dist(origin,TOOL.pos)<.009
                and abs(origin[2]-TOOL.pos[2])<.003 and upright
                and low<LIVE_CONTEXT['contact_forces']['coder_on_cradle'][2]<high
                and math.hypot(*objects[TOOL.name].lin_vel)<.018
                and all(math.dist(tcp,origin)>.068 for tcp in LIVE_CONTEXT['tcp'].values()))


def score(objects):
    return 1. if success(objects) else .10+.68*print_coverage() if _lifted and not _spoiled else 0.


def update_materials():
    global _ink
    from pxr import Gf, UsdGeom
    if _ink is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        carton, = (prim for prim in stage.Traverse() if prim.GetName()==CARTON.name)
        _ink = []
        for i, (col, _) in enumerate(type_dots(),start=4):
            prim = UsdGeom.Imageable(stage.GetPrimAtPath(f'{carton.GetPath()}/geometry/visual_{i}'))
            if not prim:
                raise RuntimeError('missing carton ink geometry')
            transform = prim.GetPrim().GetAttribute('xformOp:translate')
            _ink.append((col,prim,transform))
    for col, prim, transform in _ink:
        y = _print_y.get(col)
        visibility = UsdGeom.Tokens.invisible if y is None else UsdGeom.Tokens.inherited
        if prim.GetVisibilityAttr().Get()!=visibility:
            prim.CreateVisibilityAttr().Set(visibility)
        if y is not None:
            x, previous_y, z = transform.Get()
            if previous_y!=y or z!=ink_height:
                transform.Set(Gf.Vec3d(x,y,ink_height))


def demo_target(arm,label,t,pos,rotation,finger):
    if arm=='left' and label in {'seat coder','settle coder in cradle'}:
        held_center = LIVE_CONTEXT['body_origins'][TOOL.name]
        grasp_offset = LIVE_CONTEXT['tcp']['left']-held_center
        _cradle_offset[:2] = np.clip(grasp_offset[:2],-.008,.008)
    if arm=='left' and label in {'seat coder','settle coder in cradle','release coder'}:
        pos = pos+_cradle_offset
    return pos,rotation,finger


def demo():
    pick = (TOOL.pos[0],TOOL.pos[1],TOOL.pos[2]+HANDLE_TCP)
    above_tool = (*TOOL.pos[:2],TOP+.169)
    stage_hold = (PRINT_X,PRINT_Y,TOP+.171)
    contact = (PRINT_X,PRINT_Y,stock_top+WHEEL_RADIUS-HUB[2]+HANDLE_TCP-.004)
    park = (.203,.239,TOP+.180)
    away = (.250,-.242,TOP+.171)
    return {'left':[
        Waypoint(1.55,above_tool,OPEN,'approach coder'),
        Waypoint(.95,pick,OPEN,'seat gripper on coder'),
        Waypoint(.65,pick,CLOSED,'grasp coder'),
        Waypoint(.90,above_tool,CLOSED,'lift coder'),
        Waypoint(1.30,stage_hold,CLOSED,'move to print station'),
        Waypoint(4.33,stage_hold,CLOSED,'wait for forward carton'),
        Waypoint(.70,contact,CLOSED,'lower print wheel'),
        Waypoint(.91,contact,CLOSED,'roll batch code'),
        Waypoint(.64,stage_hold,CLOSED,'lift wheel clear'),
        Waypoint(1.30,above_tool,CLOSED,'return coder to cradle'),
        Waypoint(.91,pick,CLOSED,'seat coder'),
        Waypoint(.35,pick,CLOSED,'settle coder in cradle'),
        Waypoint(.63,pick,OPEN,'release coder'),
        Waypoint(.90,park,OPEN,'move hand clear'),
        Waypoint(1.80,park,OPEN,'observe printed carton'),
    ],'right':[Waypoint(1.60,away,OPEN,'park unused wrist'),
                 Waypoint(19.,away,OPEN,'keep the carton path clear')]}
