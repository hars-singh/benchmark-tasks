"""Eject a paper remnant from an off LX-350 through native platen contact."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera,Lens
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from dynamic_task_printer_manual_jam_eject.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.451,-.096
Z_NIP=TOP+.106
SLOT_Y=Y+.008
PAPER_W,PAPER_H,PAPER_T=.174,.078,.00085
PLATEN_R=.0147
KNOB_R=.019
KNOB_LOCAL=np.array((-.179,0.,0.))
HINGE=np.array((X,Y+.106,TOP+.142))
RESET_OPEN=math.radians(105.)
LID_LENGTH=.133
IVORY=(.82,.83,.80)
DARK=(.102,.111,.118)
RUBBER=(.115,.129,.136)
PAPER_COLOR=(.93,.91,.79)
INK=(.22,.30,.42)
SMOKE=(.37,.44,.49)
CASE=Box('epson_lx350_housing',pos=(X,Y,TOP+.048),size=(.348,.275,.096),mass=4.1,color=IVORY)
PLATEN=Box('manual_feed_platen',pos=(X,SLOT_Y+PLATEN_R+PAPER_T/2,Z_NIP),
            size=(.382,.038,.038),mass=.211,color=RUBBER)
IDLER=Box('paper_pinch_roller',pos=(X,SLOT_Y-.0078-PAPER_T/2+.00065,Z_NIP),
           size=(.224,.016,.016),mass=.082,color=RUBBER)
COVER=Box('tinted_printer_cover',pos=tuple(map(float,HINGE)),size=(.319,.133,.008),mass=.095,color=SMOKE)
PAPER=Box('folded_paper_remnant',pos=(X-.012,SLOT_Y,Z_NIP+.010),size=(PAPER_W,PAPER_T,PAPER_H),mass=.0043,color=PAPER_COLOR)
TRAY=Box('removed_paper_tray',pos=(.448,-.350,TOP+.0045),size=(.224,.141,.009),color=(.13,.29,.42))
OBJECTS=[CASE,PLATEN,IDLER,COVER,PAPER,TRAY]


def cover_part(size,position,color,opacity=None):
    rotation=axis_angle_matrix(np.array((1.,0.,0.)),-RESET_OPEN)
    return Solid('box',size,tuple(map(float,rotation@position)),roll=-RESET_OPEN,color=color,opacity=opacity)


BODY_PARTS={
    CASE.name:(
        Solid('box',(.348,.275,.042),(0.,0.,-.027),color=IVORY),
        *(Solid('box',(.025,.258,.096),(i*.161,0.,.030),color=IVORY) for i in (-1,1)),
        Solid('box',(.292,.075,.068),(0.,-.091,.020),color=IVORY),
        Solid('box',(.297,.048,.076),(0.,.102,.020),color=IVORY),
        Solid('box',(.294,.019,.010),(0.,-.024,.076),color=DARK),
        Solid('box',(.185,.010,.040),(0.,SLOT_Y-Y+.017,.028),color=IVORY),
        *(Solid('box',(.029,.022,.028),(i*.144,.106,.085),color=DARK) for i in (-1,1)),
        Solid('box',(.096,.053,.026),(-.097,-.082,.052),color=DARK),
    ),
    PLATEN.name:(
        Solid('cylinder',(PLATEN_R,.282),axis='X',color=RUBBER),
        Solid('cylinder',(.006,.352),axis='X',color=DARK),
        Solid('cylinder',(KNOB_R,.021),tuple(KNOB_LOCAL),axis='X',color=DARK),
        *(Solid('box',(.019,.0026,.004),(-.179,KNOB_R*math.cos(a),KNOB_R*math.sin(a)),
                roll=a,color=DARK) for a in (i*math.tau/12 for i in range(12))),
    ),
    IDLER.name:(Solid('cylinder',(.0078,.230),axis='X',color=RUBBER),),
    COVER.name:(
        cover_part((.306,.114,.0034),(0.,-.065,.001),SMOKE,.36),
        *(cover_part((.015,.125,.008),(.151*s,-.067,.001),DARK) for s in (-1,1)),
        cover_part((.316,.014,.009),(0.,-.124,.003),DARK),
        cover_part((.307,.013,.009),(0.,-.012,.003),DARK),
    ),
    PAPER.name:(
        Solid('box',PAPER.size,color=PAPER_COLOR),
        Solid('box',(.153,.0034,.008),(0.,-.0010,PAPER_H/2.-.0035),
              roll=.24,color=PAPER_COLOR),
    ),
    TRAY.name:(
        Solid('box',TRAY.size),
        *(Solid('box',(.004,.141,.018),(.110*s,0.,.013)) for s in (-1,1)),
        *(Solid('box',(.216,.004,.018),(0.,.0685*s,.013)) for s in (-1,1)),
    ),
}
VISUAL_PARTS={
    CASE.name:(
        *(Solid('box',(.011,.006,.0004),(x,-.089,.0653),color=(.32,.43,.48))
          for x in (-.123,-.106,-.089,-.072)),
        *(Solid('box',(.105,.0014,.0004),(.065,y,.0579),color=DARK) for y in (.081,.086,.091)),
    ),
    PAPER.name:(
        *(Solid('box',(.143,.00015,.0006),(0.,-.00049,z),color=INK)
          for z in (-.004,.003,.010,.017,.024)),
        Solid('box',(.164,.00016,.0015),(0.,-.00049,.030),color=INK),
    ),
    PLATEN.name:(Solid('box',(.004,.010,.00035),(-.180,0.,KNOB_R+.0003),color=(.63,.70,.75)),),
}
SUPPORT_JOINTS={CASE.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    PLATEN.name:{'kind':'revolute','parent':CASE.name,'axis':'X','anchor':PLATEN.pos,
                 'drive':{'stiffness':0.,'damping':.014,'max_force':.10}},
    IDLER.name:{'kind':'prismatic','parent':CASE.name,'axis':'Y','anchor':IDLER.pos,
                'lower':-.004,'upper':.0018,'drive':{'stiffness':660.,'damping':8.,'max_force':5.}},
    COVER.name:{'kind':'revolute','parent':CASE.name,'axis':'X','anchor':COVER.pos,
                'lower':-1.5,'upper':105.,'drive':{'damping':.048,'max_force':.32}},
}
SURFACE_FRICTION={PLATEN.name:(1.,.87),IDLER.name:(.24,.19),PAPER.name:(.82,.66),COVER.name:(.63,.54)}
CONTACT_OFFSETS_M={PAPER.name:.00023,IDLER.name:.00023,PLATEN.name:.00026,COVER.name:.00031}
RIGID_SOLVER_ITERATIONS={o.name:(64,8) for o in (PAPER,PLATEN,IDLER,COVER)}
OBJECT_MATERIALS={CASE.name:'stock LX-350 ABS housing and paper guides',
                  PLATEN.name:'rubber platen with its manual feed knob',IDLER.name:'paper pinch roller',
                  COVER.name:'original-style smoked printer cover',PAPER.name:'creased paper remnant',
                  TRAY.name:'shallow plastic paper tray'}

cam_pos=np.array((-.205,.242,.367))
cam_f=np.array((-.045,-.005,.088))-cam_pos
cam_f/=np.linalg.norm(cam_f)
cam_r=np.cross(cam_f,(0.,0.,1.));cam_r/=np.linalg.norm(cam_r)
cam_q=matrix_to_quat_xyzw(np.column_stack((cam_r,np.cross(cam_r,cam_f),-cam_f)))
TASK_RECORD_CAMERA=(CASE.name,Camera(tuple(cam_pos),tuple(cam_q),'opengl',Lens.from_focal(24.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
_exposed=False
_unwrapped=0.
_previous=None
_invalid=False


def on_reset():
    global _exposed,_unwrapped,_previous,_invalid
    _exposed=False
    _unwrapped=0.
    _previous=None
    _invalid=False


def roll_angle(state):
    x,y,z,w=state.quat_xyzw
    return 2.*math.atan2(x,w)


def observe(objects,context,dt):
    global _exposed,_unwrapped,_previous,_invalid
    angle=roll_angle(objects[PLATEN.name])
    if _previous is not None:
        change=angle-_previous
        _unwrapped+=math.atan2(math.sin(change),math.cos(change))
    _previous=angle
    paper=context['body_origins'][PAPER.name]
    _invalid=_invalid or math.dist(context['body_origins'][CASE.name],CASE.pos)>.002
    unheld=all(math.dist((paper[0],paper[1],objects[PAPER.name].high[2]),p)>.056
               for p in context['tcp'].values())
    if -_unwrapped>=1.42 and paper[2]-PAPER.pos[2]>=.023 and unheld:
        _exposed=True


def parked(objects):
    p=objects[PAPER.name]
    return bool(p.low[0]>TRAY.pos[0]-.110 and p.high[0]<TRAY.pos[0]+.110
                and p.low[1]>TRAY.pos[1]-.068 and p.high[1]<TRAY.pos[1]+.068
                and TOP+.004<p.low[2]<TOP+.021 and np.linalg.norm(p.lin_vel)<.015)


def success(objects):
    return bool(not _invalid and _exposed and parked(objects)
                and abs(roll_angle(objects[COVER.name])-RESET_OPEN)<=math.radians(3.)
                and abs(objects[COVER.name].ang_vel[0])<.03)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .5 if _exposed else 0.


SWING=.78
PREP=.76+.58+.62
REGRIP=PREP+SWING+.53+.56
REPEATS=5
COLLECT_START=REPEATS*REGRIP+.60
PAPER_GRIP_X=PAPER.pos[0]-.036
PAPER_GUESSED_Z=PAPER.pos[2]+PAPER_H/2+.023
KNOB_CENTER=np.asarray(PLATEN.pos)+KNOB_LOCAL
KNOB_GRIP=tuple(KNOB_CENTER+(0.,0.,.008))
CLOSE_PUSH=(X-.054,HINGE[1]+.056,HINGE[2]+.146)
CLOSE_END=(X-.054,HINGE[1]-.086,HINGE[2]+.099)
LEFT_PITCH=axis_angle_matrix(np.array((0.,1.,0.)),-.62)
RIGHT_PITCH=axis_angle_matrix(np.array((0.,1.,0.)),-.67)


def demo():
    knob_hover=tuple(KNOB_CENTER+(0.,0.,.077))
    right=[]
    for _ in range(REPEATS):
        right.extend((Waypoint(.76,knob_hover,FINGER_OPEN,'approach the feed knob'),
                      Waypoint(.58,KNOB_GRIP,FINGER_OPEN,'reach the knurled rim'),
                      Waypoint(.62,KNOB_GRIP,-.010,'grip the feed knob'),
                      Waypoint(SWING,KNOB_GRIP,-.010,'roll the paper outward'),
                      Waypoint(.53,KNOB_GRIP,FINGER_OPEN,'release for another grip'),
                      Waypoint(.56,knob_hover,FINGER_OPEN,'withdraw from the knob')))
    recovery=(PAPER_GRIP_X,SLOT_Y,PAPER_GUESSED_Z)
    front_clear=(PAPER_GRIP_X,SLOT_Y-.068,TOP+.264)
    hold=(PAPER_GRIP_X,SLOT_Y,TOP+.315)
    tray_pick_x=TRAY.pos[0]-.010
    tray_pick_y=TRAY.pos[1]-.006
    right += [
        Waypoint(.60,knob_hover,FINGER_OPEN,'leave the knob clear'),
        Waypoint(1.33,front_clear,FINGER_OPEN,'clear the open cover on the way to the paper'),
        Waypoint(.70,(recovery[0],recovery[1],recovery[2]+.041),FINGER_OPEN,'approach the sheet from the front'),
        Waypoint(.85,recovery,FINGER_OPEN,'reach exposed paper edge'),
        Waypoint(.68,recovery,0.,'grip exposed paper'),
        Waypoint(1.22,hold,0.,'lift paper straight up out of the nip'),
        Waypoint(1.13,(tray_pick_x,tray_pick_y,TOP+.167),0.,'carry paper to the tray'),
        Waypoint(.65,(tray_pick_x,tray_pick_y,TOP+.084),0.,'lower paper inside the tray'),
        Waypoint(.66,(tray_pick_x,tray_pick_y,TOP+.084),FINGER_OPEN,'release paper to the tray'),
        Waypoint(1.12,(.268,-.300,TOP+.220),FINGER_OPEN,'clear the tray'),
    ]
    return {'right':right,'left':[
        Waypoint(2.,(.259,.197,1.036),FINGER_OPEN,'stay clear of the sheet'),
        Waypoint(COLLECT_START+6.8-2.,(.259,.197,1.036),FINGER_OPEN,'wait until the paper clears'),
        Waypoint(1.25,CLOSE_PUSH,FINGER_OPEN,'reach above the open cover lip'),
        Waypoint(.76,(X-.054,HINGE[1]+.040,HINGE[2]+.122),-.022,'reach behind the open cover lip'),
        Waypoint(1.35,CLOSE_END,-.022,'push the cover past the balance point'),
        Waypoint(4.5,(X-.054,HINGE[1]-.120,HINGE[2]+.016),-.020,'guide the cover down onto the printer'),
        Waypoint(1.1,(.268,.207,TOP+.252),FINGER_OPEN,'clear the closed printer'),
        Waypoint(6.3,(.268,.207,TOP+.252),FINGER_OPEN,'observe the cover and removed paper'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    if arm=='left':
        if phase=='guide the cover down onto the printer' and 'body_origins' in LIVE_CONTEXT:
            closed=roll_angle(LIVE_OBJECTS[COVER.name])
            opening=max(0.,RESET_OPEN-closed-.16)
            edge=np.asarray(HINGE)+(-.054,-.120*math.cos(opening),.120*math.sin(opening))
            contact=edge+(0.,.009*math.sin(opening),.011+.006*math.cos(opening))
            return contact,LEFT_PITCH@rotation,finger
        return position,LEFT_PITCH@rotation,finger
    if t>=REPEATS*REGRIP:
        if phase in {'approach the sheet from the front','reach exposed paper edge','grip exposed paper'} and 'body_origins' in LIVE_CONTEXT:
            p=LIVE_CONTEXT['body_origins'][PAPER.name]
            position=np.asarray(position).copy()
            position[1]=p[1]-.006
            high=LIVE_OBJECTS[PAPER.name].high[2]
            if phase=='approach the sheet from the front':
                position[2]=max(position[2],high+.026)
            else:
                position[2]=high-.0105 if phase=='grip exposed paper' else min(position[2],high-.0105)
        base_approach=min(1.,max(0.,(t-REPEATS*REGRIP)/.85))
        pitch=axis_angle_matrix(np.array((0.,1.,0.)),-.67+.57*base_approach)
        return position,pitch@grip_rotation(),finger
    if phase in {'roll the paper outward','release for another grip','withdraw from the knob'}:
        f=min(1.,max(0.,(t%REGRIP-PREP)/SWING));a=-.63*f*f*(3.-2.*f)
        tilt=axis_angle_matrix(np.array((1.,0.,0.)),a)
        point=KNOB_CENTER+tilt@np.array((0.,0.,.008))+(0.,0.,position[2]-KNOB_GRIP[2])
        return point,RIGHT_PITCH@tilt@grip_rotation(),finger
    return position,RIGHT_PITCH@rotation,finger
