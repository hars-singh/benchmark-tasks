"""Register a framed stencil to an actual two-axis, screw-adjusted PCB bed."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_pcb_stencil_xy_registration.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.443,-.018
X_ERROR,Y_ERROR=-.00095,.00070
Z_FRAME=TOP+.061
Z_FOIL=TOP+.0495
FRAME_HINGE=np.array((X+.169,Y,Z_FRAME))
FRAME_FRONT=X-.149
HOLE_W=.0042
GRID_X=(-.014,.014)
GRID_Y=(-.0375,-.0125,.0125,.0375)
PITCH=.0018
BLUE=(.08,.26,.49)
ALUM=(.60,.68,.73)
PLATE=(.73,.76,.75)
BLACK=(.068,.082,.093)
GREEN=(.071,.34,.24)
COPPER=(.82,.51,.141)
FOIL=(.44,.48,.50)
BASE=Box('printer_base',pos=(X,Y,TOP+.014),size=(.350,.430,.028),mass=8.3,color=BLUE)
X_BED=Box('longitudinal_carriage',pos=(X+X_ERROR,Y,TOP+.031),size=(.290,.371,.006),
          mass=.92,color=ALUM)
XY_BED=Box('board_support_table',pos=(X+X_ERROR,Y+Y_ERROR,TOP+.040),size=(.292,.406,.012),
           mass=1.84,color=PLATE)
PCB=Box('controller_board',pos=(X+X_ERROR,Y+Y_ERROR,TOP+.047),size=(.090,.112,.0016),
        mass=.032,color=GREEN)
FRAME=Box('aluminium_stencil_frame',pos=tuple(map(float,FRAME_HINGE)),size=(.345,.340,.022),
          mass=.506,color=ALUM)
SCREW_X=Box('longitudinal_adjustment_knob',pos=(X-.194,Y-.137,TOP+.034),
            size=(.024,.029,.029),mass=.060,color=BLACK)
SCREW_Y=Box('transverse_adjustment_knob',pos=(X-.066,Y+.234,TOP+.036),
            size=(.029,.025,.029),mass=.061,color=BLACK)
OBJECTS=[BASE,X_BED,XY_BED,PCB,FRAME,SCREW_X,SCREW_Y]


def frame_box(size, pos, color):
    return Solid('box',size,tuple(map(float,np.asarray(pos)-FRAME_HINGE)),color=color)


def foil_mesh():
    """Disjoint foil rectangles leave eight actual square apertures."""
    shape_x,shape_y=.300,.318
    lower_x,upper_x=X-shape_x/2.,X+shape_x/2.
    lower_y,upper_y=Y-shape_y/2.,Y+shape_y/2.
    holes=tuple((X+x,Y+y) for x in GRID_X for y in GRID_Y)
    xs=sorted([lower_x,upper_x]+[X+x+s*HOLE_W/2 for x in GRID_X for s in (-1,1)])
    ys=sorted([lower_y,upper_y]+[Y+y+s*HOLE_W/2 for y in GRID_Y for s in (-1,1)])
    result=[]
    for xa,xb in zip(xs,xs[1:]):
        for ya,yb in zip(ys,ys[1:]):
            cx,cy=.5*(xa+xb),.5*(ya+yb)
            if all(abs(cx-hx)>HOLE_W*.48 or abs(cy-hy)>HOLE_W*.48 for hx,hy in holes):
                result.append(frame_box((xb-xa,yb-ya,.00025),(cx,cy,Z_FOIL),FOIL))
    return result


def screw_knob(axis):
    parts=[Solid('cylinder',(.014,.020),axis=axis,color=BLACK)]
    for n in range(10):
        angle=n*math.tau/10
        a,b=.014*math.cos(angle),.014*math.sin(angle)
        if axis=='X':
            parts.append(Solid('box',(.017,.003,.004),(0.,a,b),roll=angle,color=BLACK))
        else:
            parts.append(Solid('box',(.003,.017,.004),(a,0.,b),pitch=-angle,color=BLACK))
    return tuple(parts)


BODY_PARTS={
    BASE.name:(Solid('box',BASE.size,color=BLUE),
               *(Solid('box',(.028,.027,.050),(.163,y,.037),color=BLACK) for y in (-.189,.189)),
               *(Solid('cylinder',(.009,.048),(.169,y,Z_FRAME-BASE.pos[2]),axis='Y',color=BLACK)
                 for y in (-.180,.180))),
    FRAME.name:(
        *(frame_box((.020,.345,.019),(x,Y,Z_FRAME+.003),ALUM) for x in (FRAME_FRONT,X+.156)),
        *(frame_box((.300,.018,.019),(X+.003,y,Z_FRAME+.003),ALUM)
          for y in (Y-.164,Y+.164)),
        *foil_mesh(),
        *(frame_box((.023,.038,.027),(X+.163,y,Z_FRAME),BLACK) for y in (Y-.155,Y+.155)),
    ),
    SCREW_X.name:screw_knob('X'),
    SCREW_Y.name:screw_knob('Y'),
}
VISUAL_PARTS={
    PCB.name:(
        *(Solid('box',(.0055,.0055,.0002),(x,y,.0009),color=COPPER) for x in GRID_X for y in GRID_Y),
        *(Solid('box',(.022,.00055,.00005),(0.,y,.00084),color=(.64,.70,.49)) for y in GRID_Y),
    ),
    XY_BED.name:tuple(Solid('box',(.007,.016,.004),(x,y,.0077),color=BLACK)
                     for x in (-.048,.048) for y in (-.042,.042)),
    SCREW_X.name:(Solid('box',(.014,.0015,.0004),(0.,0.,.0158),color=ALUM),),
    SCREW_Y.name:(Solid('box',(.0015,.013,.0004),(0.,0.,.0158),color=ALUM),),
}
SUPPORT_JOINTS={BASE.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    X_BED.name:{'kind':'prismatic','parent':BASE.name,'axis':'X','anchor':X_BED.pos,
                'lower':-.010,'upper':.010,'drive':{'damping':135.,'max_force':25.}},
    XY_BED.name:{'kind':'prismatic','parent':X_BED.name,'axis':'Y','anchor':XY_BED.pos,
                 'lower':-.040,'upper':.040,'drive':{'damping':116.,'max_force':25.}},
    PCB.name:{'kind':'fixed','parent':XY_BED.name,'anchor':PCB.pos},
    FRAME.name:{'kind':'revolute','parent':BASE.name,'axis':'Y','anchor':FRAME.pos,
                'lower':0.0,'upper':45.,'drive':{'stiffness':.015,'target':24.,'damping':.035,'max_force':.95}},
    SCREW_X.name:{'kind':'revolute','parent':BASE.name,'axis':'X','anchor':SCREW_X.pos,
                  'drive':{'damping':.0015,'max_force':.025},
                  'screw_coupling':{'carriage':X_BED.name,'travel_per_turn_m':PITCH}},
    SCREW_Y.name:{'kind':'revolute','parent':X_BED.name,'axis':'Y','anchor':SCREW_Y.pos,
                  'drive':{'damping':.024,'max_force':.058},
                  'screw_coupling':{'carriage':XY_BED.name,'travel_per_turn_m':PITCH}},
}
SURFACE_FRICTION={FRAME.name:(.50,.42),SCREW_X.name:(.92,.79),SCREW_Y.name:(.92,.79)}
CONTACT_OFFSETS_M={o.name:.00024 for o in OBJECTS}
RIGID_SOLVER_ITERATIONS={o.name:(64,8) for o in OBJECTS}
OBJECT_MATERIALS={BASE.name:'blue steel YSJ-250 base',X_BED.name:'aluminium X carriage',
                  XY_BED.name:'aluminium Y support table with locating shoes',PCB.name:'FR-4 PCB with copper pads',
                  FRAME.name:'aluminium frame with a 0.25 mm stainless stencil',
                  SCREW_X.name:'steel adjustment screw with knurled polymer knob',
                  SCREW_Y.name:'steel adjustment screw with knurled polymer knob'}

down=np.column_stack(((0.,-1.,0.),(1.,0.,0.),(0.,0.,1.)))
TASK_RECORD_CAMERA=(BASE.name,Camera((0.,0.,.168),tuple(matrix_to_quat_xyzw(down)),
                                      'opengl',Lens.from_focal(28.,640,360)))
LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.50
_aligned=False
_opened=False
_invalid=False
_left_swing={}


def on_reset():
    global _aligned,_opened,_invalid
    _aligned=_opened=_invalid=False
    _left_swing.clear()


def aperture_registration(objects, context):
    o=context['body_origins']
    frame_rotation=quat_xyzw_to_matrix(objects[FRAME.name].quat_xyzw)
    board_rotation=quat_xyzw_to_matrix(objects[PCB.name].quat_xyzw)
    offsets=[]
    for x in GRID_X:
        for y in GRID_Y:
            opening=o[FRAME.name]+frame_rotation@(np.array((X+x,Y+y,Z_FOIL))-FRAME_HINGE)
            pad=o[PCB.name]+board_rotation@np.array((x,y,.0008))
            offsets.append(opening-pad)
    d=np.array(offsets)
    return float(np.max(np.linalg.norm(d[:,:2],axis=1))),float(np.max(abs(d[:,2])))


def observe(objects, context, dt):
    global _aligned,_opened,_invalid
    planar, vertical=aperture_registration(objects,context)
    _invalid=_invalid or math.dist(context['body_origins'][BASE.name],BASE.pos)>.002
    _aligned=_aligned or planar<=.00038 and vertical<=.0020
    lift=quat_xyzw_to_matrix(objects[FRAME.name].quat_xyzw)[0,2]
    _opened=_opened or _aligned and lift>=math.sin(math.radians(21.))


def success(objects):
    xy,z=aperture_registration(objects,LIVE_CONTEXT)
    free=all(math.dist((X-.090,Y-.164,Z_FRAME),tcp)>.080 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(not _invalid and _opened and xy<=.00038 and z<=.0020 and free)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .50 if _aligned else 0.


GRIP_H=.010
PINCH=-.0095
SWING=.82
TURN_REPEATS=4
SCREW_CYCLE=.70+.55+.61+SWING+.55+.63
FRAME_GRIP=np.array((X-.090,Y-.164,Z_FRAME+.009))-FRAME_HINGE
FRAME_START=TURN_REPEATS*SCREW_CYCLE
FRAME_LIFT_START=FRAME_START+1.1+.72+.60
FRAME_LIFT_END=FRAME_LIFT_START+1.35
FRAME_LOWER_START=FRAME_LIFT_END+.55
FRAME_LOWER_END=FRAME_LOWER_START+1.24
FRAME_RELEASE_END=FRAME_LOWER_END+.65
AXES={'left':np.array((0.,1.,0.)), 'right':np.array((1.,0.,0.))}
SCREWS={'left':SCREW_Y,'right':SCREW_X}
TURNS={'left':-math.tau*Y_ERROR/(PITCH*TURN_REPEATS),
       'right':-math.tau*X_ERROR/(PITCH*TURN_REPEATS)}


def smooth(v):
    v=min(1.,max(0.,v))
    return v*v*(3.-2.*v)


def wrist(arm):
    return grip_rotation(math.pi/2. if arm=='left' else 0.)


def demo():
    plan={}
    for arm,screw in SCREWS.items():
        p=np.asarray(screw.pos)+(0.,0.,GRIP_H)
        hover=tuple(p+(0.,0.,.061))
        plan[arm]=[]
        for turn_no in range(TURN_REPEATS):
            plan[arm].extend((
                Waypoint(.70,hover,FINGER_OPEN,'approach adjuster'),
                Waypoint(.55,tuple(p),FINGER_OPEN,'lower onto adjuster'),
                Waypoint(.61,tuple(p),PINCH,'grip adjuster'),
                Waypoint(SWING,tuple(p),PINCH,'turn adjuster'),
                Waypoint(.55,tuple(p),FINGER_OPEN,'release adjuster'),
                Waypoint(.63,hover,FINGER_OPEN,'lift clear of adjuster'),
            ))
    near=tuple(FRAME_HINGE+FRAME_GRIP)
    hover=tuple(FRAME_HINGE+FRAME_GRIP+(0.,0.,.083))
    plan['right'].extend((
        Waypoint(1.1,hover,FINGER_OPEN,'move over front stencil frame'),
        Waypoint(.72,near,FINGER_OPEN,'reach the frame rail'),
        Waypoint(.60,near,-.006,'grip frame rail'),
        Waypoint(1.35,near,-.006,'open stencil frame'),
        Waypoint(.55,near,-.006,'hold stencil open'),
        Waypoint(1.24,near,-.006,'lower stencil to confirm alignment'),
        Waypoint(.65,near,FINGER_OPEN,'release closed stencil'),
        Waypoint(1.0,hover,FINGER_OPEN,'clear the inspection view'),
        Waypoint(2.0,hover,FINGER_OPEN,'verify registration after closure'),
    ))
    return plan


def demo_target(arm, phase, time_s, position, rotation, finger):
    if arm=='right' and time_s>=FRAME_START:
        turn=.460*(smooth((time_s-FRAME_LIFT_START)/1.35)-smooth((time_s-FRAME_LOWER_START)/1.24))
        tilt=axis_angle_matrix(np.array((0.,1.,0.)),turn)
        if FRAME_LIFT_START<=time_s<=FRAME_RELEASE_END:
            position=FRAME_HINGE+tilt@FRAME_GRIP
        wrist_tilt=axis_angle_matrix(np.array((0.,1.,0.)),turn*.45-.34)
        return position,wrist_tilt@grip_rotation(),finger
    clock=time_s%SCREW_CYCLE
    outward=smooth((clock-1.86)/SWING)
    swing=TURNS[arm]
    if arm=='left' and 'body_origins' in LIVE_CONTEXT:
        cycle=int(time_s/SCREW_CYCLE)
        if phase=='turn adjuster' and cycle not in _left_swing:
            error=Y-LIVE_CONTEXT['body_origins'][PCB.name][1]
            _left_swing[cycle]=float(np.clip(1.12*math.tau*error/PITCH,-.86,.86))
        swing=_left_swing.get(cycle,swing)
    turn=swing*outward
    center=np.asarray(SCREWS[arm].pos)
    if 'body_origins' in LIVE_CONTEXT:
        center=np.asarray(LIVE_CONTEXT['body_origins'][SCREWS[arm].name])
    if phase in {'turn adjuster','release adjuster','lift clear of adjuster'}:
        tilt=axis_angle_matrix(AXES[arm],turn)
        position=center+tilt@np.array((0.,0.,GRIP_H))+(0.,0.,position[2]-SCREWS[arm].pos[2]-GRIP_H)
        return position,tilt@wrist(arm),finger
    return position,wrist(arm),finger
