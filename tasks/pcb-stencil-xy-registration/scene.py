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
# ROBOTIS OpenCR RevH U21 land centers and D228 centerline lengths.
# Bottom-up view, transformed from the official PCB outline and artwork.
U21_LANDS=((0.02770881, 0.02076516, 0.00150038),
 (0.02770881, 0.01949516, 0.00150038),
 (0.02770881, 0.01822516, 0.00150038),
 (0.02770881, 0.01695516, 0.00150038),
 (0.03260885, 0.02076516, 0.00150012),
 (0.03260885, 0.01949516, 0.00150012),
 (0.03260885, 0.01822516, 0.00150012),
 (0.03260885, 0.01695516, 0.00150012))
COPPER_WIDTH=0.000599948
APERTURE_X=.00170
APERTURE_Y=.00040
FOIL_THICKNESS=.00015
REGISTRATION_LIMIT=.00015
# The larger exposed copper landmarks come from the same bottom mask artwork.
OTHER_LANDS_MM=(('C', -51.0651, -3.105, 2.84988, 2.84988),
 ('C', -51.0651, 10.10503, 2.84988, 2.84988),
 ('C', -49.5251, -25.95002, 1.40005, 1.40005),
 ('C', -49.23732, -17.00007, 1.50012, 1.50012),
 ('C', -49.0999, -7.65008, 1.54991, 1.54991),
 ('C', -49.0999, 16.69989, 1.54991, 1.54991),
 ('C', -41.53502, -1.2, 2.84988, 2.84988),
 ('C', -41.53502, 3.50001, 2.84988, 2.84988),
 ('C', -38.2, -35.10011, 1.40005, 1.40005),
 ('C', -36.2, -35.10011, 1.40005, 1.40005),
 ('C', -34.20001, -35.10011, 1.40005, 1.40005),
 ('C', -22.20003, 34.60003, 1.59995, 1.59995),
 ('C', -19.69991, 34.60003, 1.59995, 1.59995),
 ('C', -11.50003, 34.60003, 1.59995, 1.59995),
 ('C', -8.99991, 34.60003, 1.59995, 1.59995),
 ('C', -8.30522, -22.58985, 1.59995, 1.59995),
 ('C', -5.76522, -22.58985, 1.59995, 1.59995),
 ('C', -3.22522, -22.58985, 1.59995, 1.59995),
 ('C', -1.20007, -35.19993, 1.45009, 1.45009),
 ('C', -0.80002, 34.60003, 1.59995, 1.59995),
 ('C', -0.68522, -22.58985, 1.59995, 1.59995),
 ('C', 0.83878, 25.67015, 1.59995, 1.59995),
 ('C', 1.33993, -35.19993, 1.45009, 1.45009),
 ('C', 1.33993, -32.65993, 1.45009, 1.45009),
 ('C', 1.7001, 34.60003, 1.59995, 1.59995),
 ('C', 1.85478, -22.58985, 1.59995, 1.59995),
 ('C', 3.37878, 25.67015, 1.59995, 1.59995),
 ('C', 3.87993, -35.19993, 1.45009, 1.45009),
 ('C', 3.87993, -32.65993, 1.45009, 1.45009),
 ('C', 4.39478, -22.58985, 1.59995, 1.59995),
 ('C', 5.91878, 25.67015, 1.59995, 1.59995),
 ('C', 6.41993, -35.19993, 1.45009, 1.45009),
 ('C', 6.41993, -32.65993, 1.45009, 1.45009),
 ('C', 6.93478, -22.58985, 1.59995, 1.59995),
 ('C', 8.45878, 25.67015, 1.59995, 1.59995),
 ('C', 8.95993, -35.19993, 1.45009, 1.45009),
 ('C', 8.95993, -32.65993, 1.45009, 1.45009),
 ('C', 9.47478, -22.58985, 1.59995, 1.59995),
 ('C', 9.95002, 34.6013, 1.59995, 1.59995),
 ('C', 10.99878, 25.67015, 1.59995, 1.59995),
 ('C', 11.49993, -35.19993, 1.45009, 1.45009),
 ('C', 11.49993, -32.65993, 1.45009, 1.45009),
 ('C', 12.01478, -22.58985, 1.59995, 1.59995),
 ('C', 12.45014, 34.6013, 1.59995, 1.59995),
 ('C', 13.53878, 25.67015, 1.59995, 1.59995),
 ('C', 14.03993, -35.19993, 1.45009, 1.45009),
 ('C', 14.03993, -32.65993, 1.45009, 1.45009),
 ('C', 14.95001, 34.6013, 1.59995, 1.59995),
 ('C', 16.07878, 25.67015, 1.59995, 1.59995),
 ('C', 16.57993, -35.19993, 1.45009, 1.45009),
 ('C', 16.57993, -32.65993, 1.45009, 1.45009),
 ('C', 18.61878, -22.58985, 1.59995, 1.59995),
 ('C', 19.11993, -35.19993, 1.45009, 1.45009),
 ('C', 19.11993, -32.65993, 1.45009, 1.45009),
 ('C', 21.15878, -22.58985, 1.59995, 1.59995),
 ('C', 21.65993, -35.19993, 1.45009, 1.45009),
 ('C', 21.65993, -32.65993, 1.45009, 1.45009),
 ('C', 23.15014, 34.6013, 1.59995, 1.59995),
 ('C', 23.69878, -22.58985, 1.59995, 1.59995),
 ('C', 23.69878, 25.67015, 1.59995, 1.59995),
 ('C', 25.65001, 34.6013, 1.59995, 1.59995),
 ('C', 26.23878, -22.58985, 1.59995, 1.59995),
 ('C', 26.23878, 25.67015, 1.59995, 1.59995),
 ('C', 28.15013, 34.6013, 1.59995, 1.59995),
 ('C', 28.77878, -22.58985, 1.59995, 1.59995),
 ('C', 28.77878, 25.67015, 1.59995, 1.59995),
 ('C', 31.31878, -22.58985, 1.59995, 1.59995),
 ('C', 31.31878, 25.67015, 1.59995, 1.59995),
 ('C', 33.85878, -22.58985, 1.59995, 1.59995),
 ('C', 33.85878, 25.67015, 1.59995, 1.59995),
 ('C', 33.99874, 0.32019, 1.59995, 1.59995),
 ('C', 33.99874, 2.86019, 1.59995, 1.59995),
 ('C', 34.40006, -35.40009, 1.40005, 1.40005),
 ('C', 34.40006, -33.40009, 1.40005, 1.40005),
 ('C', 34.40006, -31.40009, 1.40005, 1.40005),
 ('C', 34.40006, -29.4001, 1.40005, 1.40005),
 ('C', 36.35002, 34.6013, 1.59995, 1.59995),
 ('C', 36.40005, -35.40009, 1.40005, 1.40005),
 ('C', 36.40005, -33.40009, 1.40005, 1.40005),
 ('C', 36.40005, -31.40009, 1.40005, 1.40005),
 ('C', 36.40005, -29.4001, 1.40005, 1.40005),
 ('C', 36.53874, -2.21981, 1.59995, 1.59995),
 ('C', 36.53874, 0.32019, 1.59995, 1.59995),
 ('C', 36.53874, 2.86019, 1.59995, 1.59995),
 ('C', 38.40005, -35.40009, 1.40005, 1.40005),
 ('C', 38.40005, -33.40009, 1.40005, 1.40005),
 ('C', 38.40005, -31.40009, 1.40005, 1.40005),
 ('C', 38.40005, -29.4001, 1.40005, 1.40005),
 ('C', 38.85014, 34.6013, 1.59995, 1.59995),
 ('C', 40.40005, -35.40009, 1.40005, 1.40005),
 ('C', 40.40005, -33.40009, 1.40005, 1.40005),
 ('C', 40.40005, -31.40009, 1.40005, 1.40005),
 ('C', 40.40005, -29.4001, 1.40005, 1.40005),
 ('C', 41.15011, 27.09992, 1.50012, 1.50012),
 ('C', 41.35001, 34.6013, 1.59995, 1.59995),
 ('C', 42.40004, -35.40009, 1.40005, 1.40005),
 ('C', 42.40004, -31.40009, 1.40005, 1.40005),
 ('C', 47.34999, -0.3773, 1.24993, 1.24993),
 ('C', 47.34999, 4.47258, 1.24993, 1.24993),
 ('C', 50.29995, -25.19995, 1.40005, 1.40005),
 ('C', 50.29995, -23.19996, 1.40005, 1.40005),
 ('C', 50.29995, -21.19996, 1.40005, 1.40005),
 ('C', 50.29995, -14.39987, 1.40005, 1.40005),
 ('C', 50.29995, -12.39988, 1.40005, 1.40005),
 ('C', 50.29995, -10.39988, 1.40005, 1.40005),
 ('R', -49.5251, -27.95001, 1.40005, 1.40005),
 ('R', -49.23732, -19.50019, 1.50012, 1.50012),
 ('R', -49.0999, -10.14995, 1.54991, 1.54991),
 ('R', -49.0999, 14.20002, 1.54991, 1.54991),
 ('R', -41.53502, 8.20003, 2.84988, 2.84988),
 ('R', -40.2, -35.10011, 1.40005, 1.40005),
 ('R', -24.6999, 34.60003, 1.59995, 1.59995),
 ('R', -13.9999, 34.60003, 1.59995, 1.59995),
 ('R', -10.84522, -22.58985, 1.59995, 1.59995),
 ('R', -3.29989, 34.60003, 1.59995, 1.59995),
 ('R', -1.70122, 25.67015, 1.59995, 1.59995),
 ('R', -1.20007, -32.65993, 1.45009, 1.45009),
 ('R', 7.45015, 34.6013, 1.59995, 1.59995),
 ('R', 16.07878, -22.58985, 1.59995, 1.59995),
 ('R', 20.65002, 34.6013, 1.59995, 1.59995),
 ('R', 21.15878, 25.67015, 1.59995, 1.59995),
 ('R', 33.85015, 34.6013, 1.59995, 1.59995),
 ('R', 33.99874, -2.21981, 1.59995, 1.59995),
 ('R', 41.15011, 25.09992, 1.50012, 1.50012),
 ('R', 42.40004, -33.40009, 1.40005, 1.40005),
 ('R', 42.40004, -29.4001, 1.40005, 1.40005),
 ('R', 50.29995, -27.19995, 1.40005, 1.40005),
 ('R', 50.29995, -16.40012, 1.40005, 1.40005))
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
PCB=Box('robotis_opencr_revh_bare_controller_pcb',pos=(X+X_ERROR,Y+Y_ERROR,TOP+.047),
        size=(.105,.075,.0016),mass=.025,color=GREEN)
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
    """Build the 0.15 mm foil around the eight actual U21 openings."""
    lower_x,upper_x=X-.150,X+.150
    lower_y,upper_y=Y-.159,Y+.159
    holes=tuple((X+x,Y+y) for x,y,_ in U21_LANDS)
    xs=sorted({lower_x,upper_x}|{x+s*APERTURE_X/2. for x,_ in holes for s in (-1,1)})
    ys=sorted({lower_y,upper_y}|{y+s*APERTURE_Y/2. for _,y in holes for s in (-1,1)})
    for xa,xb in zip(xs,xs[1:]):
        for ya,yb in zip(ys,ys[1:]):
            cx,cy=.5*(xa+xb),.5*(ya+yb)
            occupied=any(abs(cx-x)<APERTURE_X/2. and abs(cy-y)<APERTURE_Y/2. for x,y in holes)
            if not occupied:
                yield frame_box((xb-xa,yb-ya,FOIL_THICKNESS),(cx,cy,Z_FOIL),FOIL)


def copper_witnesses():
    for x,y,length in U21_LANDS:
        yield Solid('box',(length,COPPER_WIDTH,.00006),(x,y,.00083),color=COPPER)
        for side in (-1.,1.):
            yield Solid('cylinder',(COPPER_WIDTH/2.,.00006),(x+side*length/2.,y,.00083),color=COPPER)
    for kind,x,y,width,height in OTHER_LANDS_MM:
        geometry=('cylinder',(width*.0005,.00004)) if kind=='C' else ('box',(width*.001,height*.001,.00004))
        yield Solid(*geometry,(x*.001,y*.001,.00082),color=COPPER)


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
    PCB.name:tuple(copper_witnesses()),
    XY_BED.name:tuple(Solid('box',(.007,.016,.004),(x,y,.0077),color=BLACK)
                     for x in (-.055,.055) for y in (-.026,.026)),
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
                  XY_BED.name:'aluminium Y support table with locating shoes',PCB.name:'ROBOTIS OpenCR 1.0 RevH FR-4 bare controller board with U21 copper lands',
                  FRAME.name:'aluminium frame with a 0.15 mm stainless bottom-side stencil for OpenCR U21',
                  SCREW_X.name:'steel adjustment screw with knurled polymer knob',
                  SCREW_Y.name:'steel adjustment screw with knurled polymer knob'}

down=np.column_stack(((0.,-1.,0.),(1.,0.,0.),(0.,0.,1.)))
U21_CENTER=tuple(float(sum(land[i] for land in U21_LANDS)/len(U21_LANDS)) for i in (0,1))
TASK_RECORD_CAMERA=(BASE.name,Camera((*U21_CENTER,.177),tuple(matrix_to_quat_xyzw(down)),
                                      'opengl',Lens.from_focal(164.,640,352)))
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
    for x,y,_ in U21_LANDS:
        opening=o[FRAME.name]+frame_rotation@(np.array((X+x,Y+y,Z_FOIL))-FRAME_HINGE)
        pad=o[PCB.name]+board_rotation@np.array((x,y,.00083))
        offsets.append(opening-pad)
    d=np.array(offsets)
    return float(np.max(np.linalg.norm(d[:,:2],axis=1))),float(np.max(abs(d[:,2])))


def observe(objects, context, dt):
    global _aligned,_opened,_invalid
    planar, vertical=aperture_registration(objects,context)
    _invalid=_invalid or math.dist(context['body_origins'][BASE.name],BASE.pos)>.002
    _aligned=_aligned or planar<=REGISTRATION_LIMIT and vertical<=.0020
    lift=quat_xyzw_to_matrix(objects[FRAME.name].quat_xyzw)[0,2]
    _opened=_opened or _aligned and lift>=math.sin(math.radians(21.))


def success(objects):
    xy,z=aperture_registration(objects,LIVE_CONTEXT)
    free=all(math.dist((X-.090,Y-.164,Z_FRAME),tcp)>.080 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(not _invalid and _opened and xy<=REGISTRATION_LIMIT and z<=.0020 and free)


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
