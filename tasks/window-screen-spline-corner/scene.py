"""Roll a connected screen spline through both legs of a real frame corner."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_window_screen_spline_corner.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
X0, X1 = .279, .486
Y0, Y1 = -.143, .092
BASE = TOP+.0105
THROAT = TOP+.0165
SEATED = BASE+.001778
RUBBER_R = .001778
WIDTH = .019
GROOVE = .00346
ROLL_R = .0175
BEND = .006
ALUMINIUM = (.62,.67,.69)
BLACK = (.070,.076,.070)
MESH = (.30,.36,.35)
WOOD = (.68,.46,.262)
FRAME = Box('aluminium_screen_frame',pos=((X0+X1)/2.,(Y0+Y1)/2.,TOP+.0073),
            size=(X1-X0+WIDTH,Y1-Y0+WIDTH,.016),mass=.40,color=ALUMINIUM)
HANDLE = Box('phifer_wood_spline_roller',pos=(.233,-.262,TOP+.021),
             size=(.153,.023,.029),mass=.052,color=WOOD)
AXLE = np.array((.081,0.,-.002))
ROLL = Box('concave_spline_roller_wheel',pos=tuple(map(float,np.asarray(HANDLE.pos)+AXLE)),
           size=(2.*ROLL_R,.0030,2.*ROLL_R),mass=.014,color=ALUMINIUM)


def points():
    front = [(x,Y0) for x in np.linspace(X0-.003,X1-BEND,11)]
    bend = [(X1-BEND+BEND*math.cos(a),Y0+BEND+BEND*math.sin(a))
            for a in (-math.pi/3.,-math.pi/6.,0.)]
    far = [(X1,y) for y in np.linspace(Y0+BEND,Y1-.003,13)][1:]
    return tuple(np.asarray(p,dtype=float) for p in front+bend+far)


PATH = points()
centres = tuple((a+b)/2. for a,b in zip(PATH,PATH[1:]))
lengths = [float(np.linalg.norm(b-a)) for a,b in zip(PATH,PATH[1:])]
angles = [math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(PATH,PATH[1:])]
FRONT_SEGMENTS = 10
OPEN_END = 13
SPLINES = tuple(Box(f'vinyl_spline_{i:02d}',pos=(float(p[0]-.00060*math.sin(yaw)*(0<i<OPEN_END)),
                    float(p[1]+.00060*math.cos(yaw)*(0<i<OPEN_END)),
                       SEATED if i==0 or i>=OPEN_END else THROAT+.0010),
                    size=(length,2.*RUBBER_R,2.*RUBBER_R),yaw=yaw,mass=max(.000035,length*.031),color=BLACK)
                for i,(p,length,yaw) in enumerate(zip(centres,lengths,angles)))
OBJECTS = [FRAME,HANDLE,ROLL,*SPLINES]


def bar(a,b,width,height,z,color):
    centre = (np.array(a)+b)/2.-np.array(FRAME.pos[:2])
    yaw=math.atan2(b[1]-a[1],b[0]-a[0])
    return Solid('box',(math.dist(a,b),width,height),(float(centre[0]),float(centre[1]),z-FRAME.pos[2]),
                 yaw=yaw,color=color)


outer = [(X0,Y0),(X1,Y0),(X1,Y1),(X0,Y1),(X0,Y0)]
frame_parts = []
for a,b in zip(outer,outer[1:]):
    frame_parts.append(bar(a,b,WIDTH,BASE-TOP,(BASE+TOP)/2.,ALUMINIUM))
    tangent=(np.asarray(b)-a)/math.dist(a,b)
    inward=np.array((-tangent[1],tangent[0]))
    for direction in (-1,1):
        offset=direction*inward*(WIDTH+GROOVE)/4.
        trim=.0085 if direction==1 else -.0075
        start=np.asarray(a)+offset+trim*tangent
        end=np.asarray(b)+offset-trim*tangent
        frame_parts.append(bar(start,end,(WIDTH-GROOVE)/2.,THROAT-BASE,
                               (THROAT+BASE)/2.,ALUMINIUM))
        outer_line=offset+direction*inward*.0026
        frame_parts.append(bar(np.asarray(a)+outer_line+trim*tangent,np.asarray(b)+outer_line-trim*tangent,
                               .0012,.00023,THROAT+.00012,(.72,.74,.74)))

BODY_PARTS = {
    FRAME.name: tuple(frame_parts),
    HANDLE.name: (
        Solid('box',(.085,.019,.018),(-.013,0.,0.),color=WOOD),
        Solid('capsule_x',(.009,.018),(-.053,0.,0.),color=WOOD),
        Solid('cylinder',(.008,.022),(.043,0.,0.),axis='X',color=(.41,.45,.44)),
        *(Solid('box',(.035,.0018,.008),(.068,s*.0048,-.002),color=(.42,.44,.46))
          for s in (-1,1)),
    ),
    ROLL.name: (
        Solid('cylinder',(ROLL_R,.0017),axis='Y',color=(.48,.52,.54)),
        *(Solid('cylinder',(ROLL_R+.0007,.00065),(0.,side*.00114,0.),axis='Y',color=ALUMINIUM)
          for side in (-1,1)),
    ),
}
for item,length in zip(SPLINES,lengths):
    if length>2.*RUBBER_R:
        BODY_PARTS[item.name]=(Solid('capsule_x',(RUBBER_R,length-2.*RUBBER_R),color=BLACK),)
    else:
        BODY_PARTS[item.name]=(Solid('sphere',(RUBBER_R,),color=BLACK),)

frame_x,frame_y,frame_z=FRAME.pos
mesh_z=BASE+.0011-frame_z
VISUAL_PARTS = {
    FRAME.name: (
        *(Solid('box',(X1-X0-.010,.00029,.00028),(0.,float(y-frame_y),mesh_z),color=MESH)
          for y in np.linspace(Y0+.004,Y1-.004,64)),
        *(Solid('box',(.00029,Y1-Y0-.010,.00027),(float(x-frame_x),0.,mesh_z),color=MESH)
          for x in np.linspace(X0+.004,X1-.004,57)),
        bar((X0,Y0+.002),(X0,Y1),2.*RUBBER_R,.002,THROAT-.0015,BLACK),
        bar((X0,Y1),(X1,Y1),2.*RUBBER_R,.002,THROAT-.0015,BLACK),
    ),
    ROLL.name:(Solid('box',(.006,.00021,.0012),(.011,.0025,0.),color=(.14,.25,.46)),),
}
SUPPORT_JOINTS={FRAME.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    ROLL.name:{'kind':'revolute','parent':HANDLE.name,'axis':'Y','anchor':ROLL.pos,
               'drive':{'damping':.00024,'max_force':.015}},
    SPLINES[0].name:{'kind':'fixed','parent':FRAME.name,'anchor':SPLINES[0].pos},
}
for i,item in enumerate(SPLINES[1:],1):
    z=.5*(SPLINES[i-1].pos[2]+item.pos[2])
    OBJECT_JOINTS[item.name]={'kind':'spherical','parent':SPLINES[i-1].name,
        'anchor':(float(PATH[i][0]),float(PATH[i][1]),z),'axis':'X','swing_limit_degrees':59.}
SURFACE_FRICTION={HANDLE.name:(.98,.80),ROLL.name:(.86,.71),
                  **{s.name:(.82,.63) for s in SPLINES}}
SURFACE_COMPLIANCE={s.name:(5250.,15.) for s in SPLINES}
CONTACT_OFFSETS_M={FRAME.name:.00016,ROLL.name:.00018,**{s.name:.000115 for s in SPLINES}}
RIGID_SOLVER_ITERATIONS={o.name:(72,8) for o in (ROLL,*SPLINES)}
OBJECT_MATERIALS={FRAME.name:'stock-size aluminium screen rails and fiberglass weave',
                  HANDLE.name:'wooden Phifer screen roller handle',ROLL.name:'stock metal concave spline wheel',
                  **{s.name:'linked sections of 0.140-inch vinyl screen spline' for s in SPLINES}}

cam=np.asarray((.129,.277,.410))
look=np.asarray((.016,-.043,.013))-cam
look/=np.linalg.norm(look)
side=np.cross(look,(0.,0.,1.));side/=np.linalg.norm(side)
in_view=matrix_to_quat_xyzw(np.column_stack((side,np.cross(side,look),-look)))
TASK_RECORD_CAMERA=(FRAME.name,Camera(tuple(cam),tuple(in_view),'opengl',Lens.from_focal(25.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
_rolled=np.zeros(len(SPLINES),dtype=bool)
_guide_y=0.
_guide_z=0.


def on_reset():
    global _guide_y,_guide_z,_corner_pass
    _rolled.fill(False)
    _corner_pass=False
    _guide_y=_guide_z=0.


def observe(objects,context,dt):
    global _corner_pass
    wheel=np.asarray(context['body_origins'][ROLL.name])
    axle=quat_xyzw_to_matrix(objects[ROLL.name].quat_xyzw)[:,1]
    spin=abs(np.dot(axle,objects[ROLL.name].ang_vel))
    if not BASE-.003<=wheel[2]-ROLL_R<=THROAT+.004 or spin<.42:
        return
    _rolled[:] |= np.linalg.norm(np.asarray(centres)-wheel[:2],axis=1)<.019
    if abs(wheel[0]-X1)<.012 and wheel[1]>Y0+.016:
        _corner_pass=True


def seated(context):
    states=np.asarray([context['body_origins'][s.name] for s in SPLINES])
    lateral=np.linalg.norm(states[:,:2]-np.asarray(centres),axis=1)
    return (states[:,2]<THROAT-.0010) & (states[:,2]>BASE-.001) & (lateral<.0031)


def success(objects):
    clear=all(math.dist(tcp,((X0+X1)/2.,-.080,TOP+.015))>.119 for tcp in LIVE_CONTEXT['tcp'].values())
    wheel=LIVE_CONTEXT['body_origins'][ROLL.name]
    withdrawn=wheel[1]<Y0-.042 and wheel[2]>TOP+.051
    return bool(clear and withdrawn and _corner_pass and seated(LIVE_CONTEXT).all() and np.all(_rolled[1:OPEN_END]))


def score(objects):
    if success(objects):
        return 1.
    return .50 if np.all(seated(LIVE_CONTEXT)[:FRONT_SEGMENTS]) and np.all(_rolled[1:FRONT_SEGMENTS]) else 0.


GRIP=np.asarray(HANDLE.pos)+(0.,0.,.008)
A=(float(X0+.008),Y0,THROAT+ROLL_R-.0034)
C=(X1-BEND,Y0,A[2])
D=(X1,Y0+BEND,A[2])
E=(X1,Y0+.089,A[2])
ARM_TILT=-.55
FRONT_INCLINATION=.18
START_ROLL=6.25
END_EDGE=START_ROLL+6.55
END_CORNER=END_EDGE+3.4
END_ROLL=END_CORNER+4.90


def tool_point(wheel,yaw,inclination=0.):
    lever=axis_angle_matrix(np.array((0.,1.,0.)),inclination)@np.array((AXLE[0],0.,AXLE[2]-.008))
    return tuple(np.asarray(wheel)-axis_angle_matrix(np.array((0.,0.,1.)),yaw)@lever)


def demo():
    high=GRIP+(0.,0.,.085)
    tool_entry=tool_point(A,0.,FRONT_INCLINATION)
    tool_entry=(tool_entry[0],tool_entry[1]+.004,tool_entry[2])
    tool_hover=(tool_entry[0],tool_entry[1],tool_entry[2]+.061)
    park=(.253,-.256,TOP+.106)
    finish=np.asarray(tool_point(E,math.pi/2.,.44))+(0.,0.,.105)
    return {'right':[
        Waypoint(1.38,tuple(high),FINGER_OPEN,'approach the wood handle'),
        Waypoint(.78,tuple(GRIP),FINGER_OPEN,'reach the spline roller'),
        Waypoint(.73,tuple(GRIP),-.009,'grip the roller handle'),
        Waypoint(.95,tuple(high),-.009,'lift the roller'),
        Waypoint(1.29,tool_hover,-.009,'position the concave wheel over the started end'),
        Waypoint(1.12,tool_entry,-.009,'set the wheel onto the spline'),
        Waypoint(END_ROLL-START_ROLL,tool_entry,-.009,'roll both edges and the corner'),
        Waypoint(.93,tuple(finish),-.009,'lift off the installed spline'),
        Waypoint(1.32,park,-.009,'carry the roller off the frame'),
        Waypoint(2.5,park,-.009,'hold the tool clear'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    global _guide_y,_guide_z
    lift_angle=FRONT_INCLINATION*min(1.,max(0.,(t-3.84)/1.29))
    pitch=axis_angle_matrix(np.array((0.,1.,0.)),ARM_TILT*min(1.,t/.9)+lift_angle)
    if phase in {'reach the spline roller','grip the roller handle'} and 'body_origins' in LIVE_CONTEXT:
        handle=np.asarray(LIVE_CONTEXT['body_origins'][HANDLE.name])+(-.008,0.,.008)
        fraction=1. if phase=='grip the roller handle' else min(1.,max(0.,(t-1.38)/.78))
        position=(1.-fraction)*np.asarray(position)+fraction*handle
    if phase=='lift off the installed spline':
        u=min(1.,max(0.,(t-END_ROLL)/.93))
        finish=np.asarray(tool_point(E,math.pi/2.,.44))
        clear=finish+(0.,0.,.105)
        turned=axis_angle_matrix(np.array((0.,0.,1.)),math.pi/2.)
        pitch=axis_angle_matrix(np.array((0.,1.,0.)),ARM_TILT+.44)
        return finish+u*(clear-finish),turned@pitch@grip_rotation(),finger
    if phase=='carry the roller off the frame':
        fraction=min(1.,max(0.,(t-END_ROLL-.93)/1.32))
        turned=axis_angle_matrix(np.array((0.,0.,1.)),math.pi/2.*(1.-fraction))
        pitch=axis_angle_matrix(np.array((0.,1.,0.)),ARM_TILT+.44*(1.-fraction)+FRONT_INCLINATION*fraction)
        return position,turned@pitch@grip_rotation(),finger
    if phase!='roll both edges and the corner':
        return position,pitch@grip_rotation(),finger
    if t<END_EDGE:
        fraction=(t-START_ROLL)/(END_EDGE-START_ROLL)
        travel=float(np.interp(fraction,(0.,.39,.51,1.),(A[0],A[0]+.089,A[0]+.057,C[0])))
        point=np.asarray((travel,Y0,A[2]))
        yaw=0.
        incline=FRONT_INCLINATION
    elif t<END_CORNER:
        fraction=(t-END_EDGE)/(END_CORNER-END_EDGE)
        turn=min(1.,max(0.,(fraction-.21)/.58))
        yaw=math.pi/2.*turn
        incline=FRONT_INCLINATION+(.44-FRONT_INCLINATION)*turn
        if fraction<.21:
            height=.032*fraction/.21
        elif fraction>.79:
            height=.032*(1.-fraction)/.21
        else:
            height=.032
        point=(X1-BEND+BEND*math.sin(yaw),Y0+BEND-BEND*math.cos(yaw),A[2]+height)
    else:
        fraction=min(1.,(t-END_CORNER)/(END_ROLL-END_CORNER))
        progress=float(np.interp(fraction,(0.,.29,.48,1.),(0.,.35,.07,1.)))
        point=np.asarray(D)+progress*(np.asarray(E)-D)
        point[2]-=.0019
        yaw=math.pi/2.
        incline=.44
    target=np.asarray(tool_point(point,yaw,incline))
    if t<END_EDGE and 'body_origins' in LIVE_CONTEXT:
        error=Y0-LIVE_CONTEXT['body_origins'][ROLL.name][1]
        _guide_y=float(np.clip(_guide_y+.18*error,-.013,.013))
        target[1]+=_guide_y+.61*float(np.clip(error,-.010,.010))
    if t>=END_EDGE and 'body_origins' in LIVE_CONTEXT:
        wheel=np.asarray(LIVE_CONTEXT['body_origins'][ROLL.name])
        relative=wheel-np.asarray(LIVE_CONTEXT['tcp']['right'])
        expected=np.asarray(point)-target
        correction=np.clip(expected-relative,(-.013,-.021,-.010),(.013,.021,.013))
        target+=.72*correction
        if t>=END_CORNER:
            target[0]+=.78*float(np.clip(X1-wheel[0],-.019,.019))
    if (t<END_EDGE or t>=END_CORNER) and 'body_origins' in LIVE_CONTEXT:
        error=point[2]-LIVE_CONTEXT['body_origins'][ROLL.name][2]
        _guide_z=float(np.clip(_guide_z+.14*error,-.008,.002))
        target[2]+=_guide_z+.37*float(np.clip(error,-.009,.009))
    else:
        _guide_z=0.
    pitch=axis_angle_matrix(np.array((0.,1.,0.)),ARM_TILT+incline)
    heading=axis_angle_matrix(np.array((0.,0.,1.)),yaw)
    return target,heading@pitch@grip_rotation(),finger
