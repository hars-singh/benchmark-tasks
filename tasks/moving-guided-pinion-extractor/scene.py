"""Remove an unpowered motor's press-fit pinion with a guided wedge extractor."""

import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_moving_guided_pinion_extractor.geometry import Solid
from dynamic_task_moving_guided_pinion_extractor.mechanisms import linear_stage
from dynamic_task_moving_guided_pinion_extractor.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
CARRIER_TRAVEL = .030
CARRIER_PERIOD = 8.
AXIS_Y = .085
AXIS_Z = TOP+.096
FACE_X = .372
TIP_X = FACE_X-.022
GEAR_RADIUS = .009
GEAR_WIDTH = .008
SHAFT_RADIUS = .003
FORK_STROKE = .023
WEDGE_STROKE = .047
DRAW_STROKE = .0168
WEDGE_SLOPE = .35
WEDGE_PITCH = -math.atan(WEDGE_SLOPE)
WEDGE_RISE = .016
FOLLOWER_RADIUS = .010
FOLLOWER_REST_GAP = .00245
PINION_BREAK_FORCE_N = 10.
STAGE = Box('extractor_carriage',pos=(.335,.025,TOP+.015),size=(.260,.244,.030),
            mass=3.3,color=(.16,.27,.39))
MOTOR = Box('motor',pos=(FACE_X+.033,AXIS_Y,AXIS_Z),size=(.066,.037,.037),
            mass=.220,color=(.38,.43,.47))
GEAR = Box('pinion',pos=(.357,AXIS_Y,AXIS_Z),size=(GEAR_WIDTH,.018,.018),
           mass=.009,color=(.66,.71,.74))
DRAW = Box('puller_crosshead',pos=(.364,-.025,AXIS_Z-.014),size=(.038,.163,.038),
           mass=.095,color=(.22,.26,.30))
FORK = Box('pinion_fork',pos=(.364,AXIS_Y,AXIS_Z+.032),size=(.052,.055,.052),
           mass=.080,color=(.71,.53,.28))
FORK_PRESS = (.364,AXIS_Y+.033,AXIS_Z+.033)
FORK_WHEEL = Box('fork_press_roller',pos=(FORK_PRESS[0],FORK_PRESS[1],FORK_PRESS[2]-.0045),
                 size=(.014,.022,.014),mass=.007,color=(.86,.65,.29))
FOLLOWER = (.326,-.103,AXIS_Z-.004)
WEDGE_X = FOLLOWER[0]+(FOLLOWER_RADIUS+.004+FOLLOWER_REST_GAP)/math.cos(WEDGE_PITCH)-WEDGE_SLOPE*WEDGE_RISE
WEDGE = Box('wedge_ram',pos=(WEDGE_X,FOLLOWER[1],FOLLOWER[2]+WEDGE_RISE),size=(.045,.046,.085),
            mass=.075,color=(.78,.46,.20))
TRAY = Box('pinion_tray',pos=(.325,AXIS_Y,TOP+.034),size=(.072,.061,.008),mass=.02,
           color=(.14,.42,.47))
hardware, BODY_PARTS = linear_stage(STAGE,CARRIER_TRAVEL)
OBJECTS = [STAGE,MOTOR,GEAR,DRAW,FORK,FORK_WHEEL,WEDGE,TRAY,*hardware]


def local(position,body):
    return tuple(a-b for a,b in zip(position,body.pos,strict=True))


def pinion_profile():
    """Approximate the bore and teeth by finite native segments."""
    for number,radius,radial,tangent in ((24,.00538,.0040,.0021),(16,.00810,.0018,.0021)):
        for step in range(number):
            a=math.tau*step/number
            yield Solid('box',(GEAR_WIDTH,tangent,radial),
                        (0.,radius*math.sin(a),radius*math.cos(a)),roll=-a)


BODY_PARTS[STAGE.name]+=tuple(
    Solid('box',size,local(position,STAGE),color=color)
    for size,position,color in (
        ((.076,.051,.040),(.404,AXIS_Y,TOP+.046),(.24,.33,.39)),
        # The bearing housing must clear the pressing fingers at full usable stroke.
        ((.037,.043,.055),(WEDGE_X+.022,FOLLOWER[1],TOP+.0565),(.26,.34,.39)),
        ((.031,.155,.019),(.380,-.017,TOP+.047),(.27,.32,.35)),
    )
)
BODY_PARTS.update({
    MOTOR.name:(
        Solid('cylinder',(.0185,.066),axis='X'),
        Solid('cylinder',(SHAFT_RADIUS,.022),(-.044,0.,0.),axis='X',color=(.60,.65,.69)),
    ),
    GEAR.name:tuple(pinion_profile()),
    DRAW.name:(
        Solid('box',(.094,.153,.012),(-.005,.019,-.023)),
        Solid('box',(.022,.062,.012),local((.396,AXIS_Y+.001,AXIS_Z-.037),DRAW)),
        Solid('box',(.017,.013,.065),local((.396,AXIS_Y+.026,AXIS_Z+.005),DRAW)),
        Solid('box',(.018,.018,.023),local((FOLLOWER[0],FOLLOWER[1],FOLLOWER[2]-.010),DRAW)),
        Solid('sphere',(FOLLOWER_RADIUS,),local(FOLLOWER,DRAW),color=(.68,.72,.75)),
    ),
    FORK.name:(
        Solid('box',(.0035,.023,.007),(0.,0.,.002)),
        *(Solid('box',(.0035,.006,.019),(0.,side*.0065,-.0095),color=(.68,.73,.76))
          for side in (-1,1)),
        Solid('box',(.014,.036,.008),(0.,.016,.002)),
        *(Solid('box',(.017,.004,.011),local((FORK_PRESS[0],FORK_PRESS[1]+side*.013,
                                            FORK_WHEEL.pos[2]-.0035),FORK),color=(.86,.65,.29))
          for side in (-1,1)),
    ),
    FORK_WHEEL.name:(Solid('cylinder',(.007,.022),axis='Y'),),
    WEDGE.name:(
        Solid('box',(.008,.025,.076),(.009*math.sin(WEDGE_PITCH),0.,.009*math.cos(WEDGE_PITCH)),
              pitch=WEDGE_PITCH,color=(.64,.68,.71)),
        Solid('box',(.036,.035,.008),(.010,0.,.027)),
    ),
    TRAY.name:(
        Solid('box',TRAY.size),
        *(Solid('box',(.004,.061,.016),(side*.034,0.,.010)) for side in (-1,1)),
        *(Solid('box',(.064,.004,.016),(0.,side*.0285,.010)) for side in (-1,1)),
    ),
})
OBJECT_JOINTS = {
    MOTOR.name:{'kind':'fixed','parent':STAGE.name,'anchor':MOTOR.pos},
    GEAR.name:{'kind':'fixed','parent':MOTOR.name,'anchor':GEAR.pos,
               'collision_enabled':True,'break_force_N':PINION_BREAK_FORCE_N},
    DRAW.name:{'kind':'prismatic','parent':STAGE.name,'anchor':DRAW.pos,
              'axis':'X','lower':-DRAW_STROKE,'upper':0.,
              'drive':{'stiffness':105.,'damping':1.4,'max_force':3.}},
    FORK.name:{'kind':'prismatic','parent':DRAW.name,'anchor':FORK.pos,
              'axis':'Z','lower':-FORK_STROKE,'upper':0.,
              'drive':{'stiffness':205.,'damping':2.0,'max_force':6.5}},
    FORK_WHEEL.name:{'kind':'revolute','parent':FORK.name,'anchor':FORK_WHEEL.pos,'axis':'Y'},
    WEDGE.name:{'kind':'prismatic','parent':STAGE.name,'anchor':WEDGE.pos,
               'axis':'Z','lower':-WEDGE_STROKE,'upper':0.,
               'drive':{'stiffness':96.,'damping':2.5,'max_force':5.}},
    TRAY.name:{'kind':'fixed','parent':STAGE.name,'anchor':TRAY.pos},
}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,CARRIER_TRAVEL,0.),CARRIER_PERIOD)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.046,'upper':.046}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':CARRIER_TRAVEL}}
CONTACT_PAIRS = {'pinion_fork_load':(GEAR.name,FORK.name),
                 'wedge_draw_load':(DRAW.name,WEDGE.name),
                 'crosshead_motor_contact':(DRAW.name,MOTOR.name),
                 'fork_motor_contact':(FORK.name,MOTOR.name),
                 'pinion_tray_support':(GEAR.name,TRAY.name)}
CONTACT_OFFSETS_M = {obj.name:.000075 for obj in (MOTOR,GEAR,DRAW,FORK,FORK_WHEEL,WEDGE)}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,4) for obj in (STAGE,MOTOR,GEAR,DRAW,FORK,FORK_WHEEL,WEDGE,TRAY)}
SUCCESS_HOLD_S = .65
MAX_PARTIAL_SCORE = .60
LIVE_CONTEXT = {}
_seated = False
_loaded = False
_removed = False
_proof_start = None
_proof_travel = False


def on_reset():
    global _seated,_loaded,_removed,_proof_start,_proof_travel
    _seated = _loaded = _removed = _proof_travel = False
    _proof_start = None


def displacement(context,body,parent,axis):
    o=context['body_origins']
    return body.pos[axis]-parent.pos[axis]-o[body.name][axis]+o[parent.name][axis]


def observe(objects,context,dt):
    global _seated,_loaded,_removed,_proof_start,_proof_travel
    o=context['body_origins']
    gear=objects[GEAR.name]
    carrier=objects[STAGE.name]
    tray=o[TRAY.name]
    fork_point=o[FORK.name]+np.array(local(FORK_PRESS,FORK))
    wedge_point=o[WEDGE.name]+np.array((.010,0.,.031))
    fork_depth=displacement(context,FORK,DRAW,2)
    left,right=context['tcp']['left'],context['tcp']['right']
    _seated |= bool(fork_depth>=FORK_STROKE-.003 and math.dist(left,fork_point)<.034
                   and abs(carrier.lin_vel[1])>=.011)
    force=context['contact_forces']
    _loaded |= bool(_seated and fork_depth>=FORK_STROKE-.003 and math.dist(right,wedge_point)<.035
                   and -force['pinion_fork_load'][0]>1.1 and -force['wedge_draw_load'][0]>1.)
    shaft_end=TIP_X+o[MOTOR.name][0]-MOTOR.pos[0]
    _removed |= bool(_loaded and gear.high[0]<shaft_end-.0005)
    settled=(_removed and abs(gear.pos[0]-tray[0])<.029 and abs(gear.pos[1]-tray[1])<.023
             and abs(gear.low[2]-(tray[2]+TRAY.size[2]/2))<.0025
             and np.linalg.norm(gear.lin_vel-carrier.lin_vel)<.012
             and np.linalg.norm(gear.ang_vel)<.5 and force['pinion_tray_support'][2]>.025
             and all(math.dist(tcp,point)>.062 for tcp in (left,right) for point in (fork_point,wedge_point)))
    if not settled:
        _proof_start=None
        _proof_travel=False
    elif _proof_start is None:
        _proof_start=tray[1]
    elif abs(tray[1]-_proof_start)>=.007:
        _proof_travel=True


def success(objects):
    return _proof_travel


def score(objects):
    return 1. if success(objects) else .60 if _removed else .20 if _seated else 0.


def demo():
    fork_hover=(*FORK_PRESS[:2],TOP+.146)
    fork_down=(*FORK_PRESS[:2],FORK_PRESS[2]+.0025+PAD-.030)
    ramp_tip=(WEDGE.pos[0]+.010,WEDGE.pos[1],WEDGE.pos[2]+.031)
    wedge_hover=(*ramp_tip[:2],TOP+.168)
    wedge_down=(*ramp_tip[:2],ramp_tip[2]+PAD-.081)
    left_clear=(.245,.207,TOP+.166)
    right_clear=(.238,-.176,TOP+.159)
    return {
        'left':[
            Waypoint(2.1,fork_hover,CLOSED,'approach the extraction fork'),
            Waypoint(1.35,fork_down,CLOSED,'seat the fork behind the pinion'),
            Waypoint(4.50,fork_down,CLOSED,'hold the fork while pulling'),
            Waypoint(.65,fork_hover,CLOSED,'unload the fork'),
            Waypoint(1.15,left_clear,OPEN,'clear the carriage'),
            Waypoint(5.,left_clear,OPEN,'observe the recovered pinion'),
        ],
        'right':[
            Waypoint(2.1,wedge_hover,CLOSED,'approach the wedge ram'),
            Waypoint(1.75,wedge_hover,CLOSED,'wait above the wedge ram'),
            Waypoint(2.15,wedge_down,CLOSED,'depress the wedge ram'),
            Waypoint(1.4,wedge_down,CLOSED,'hold the axial stroke'),
            Waypoint(.75,wedge_hover,CLOSED,'release the wedge ram'),
            Waypoint(1.25,right_clear,OPEN,'clear the carriage'),
            Waypoint(5.,right_clear,OPEN,'observe the recovered pinion'),
        ],
    }


def demo_target(arm,label,t,pos,rotation,finger):
    if not LIVE_CONTEXT or label in {'clear the carriage','observe the recovered pinion','done'}:
        return pos,rotation,finger
    origins=LIVE_CONTEXT['body_origins']
    offset=origins[STAGE.name]-np.asarray(STAGE.pos)
    offset[1]+=.082*LIVE_OBJECTS[STAGE.name].lin_vel[1]
    if arm=='left':
        offset[0]=origins[FORK.name][0]-FORK.pos[0]
    return pos+offset,rotation,finger
