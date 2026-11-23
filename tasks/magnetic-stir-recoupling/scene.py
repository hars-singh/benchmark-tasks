"""Recover magnetic stirring by recentering a loose beaker on a running stirrer."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_magnetic_stir_recoupling.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.424,-.182
OFFSET=-.044
PLATE=TOP+.045
GLASS_H=.080
GLASS_R=.030
BAR_R=.003
BAR_LENGTH=.025
RPM=200.
OMEGA=RPM*math.tau/60.
MOTOR_BLUE=(.08,.22,.45)
WHITE=(.86,.90,.92)
DARK=(.10,.13,.16)
WATER=(.19,.40,.58)
CLEAR=(.60,.75,.82)
GLASS_CENTER=PLATE+GLASS_H/2.
HULL=Box('stirrer_housing',pos=(X-.010,Y,TOP+.0225),size=(.205,.185,.045),
         mass=.69,color=MOTOR_BLUE)
ROTOR=Box('field_rotor',pos=(X,Y,TOP+.025),size=(.054,.012,.006),
          mass=.034,color=DARK)
VESSEL=Box('glass_beaker',pos=(X,Y+OFFSET,GLASS_CENTER),size=(.06,.06,.08),
           mass=.181,color=CLEAR)
BAR=Box('ptfe_stir_bar',pos=(X,Y+OFFSET,PLATE+.0028+BAR_R+.0005),
        size=(BAR_LENGTH,2*BAR_R,2*BAR_R),mass=.0038,color=WHITE)
OBJECTS=[HULL,ROTOR,VESSEL,BAR]
span=math.tau/20.
BODY_PARTS={
    HULL.name:(
        Solid('cylinder',(.0925,.016),(0.,0.,-.0145),color=MOTOR_BLUE),
        Solid('cylinder',(.080,.005),(.010,0.,.020),color=WHITE),
        *(Solid('box',(.013,.035,.023),(.085*math.cos(a),.085*math.sin(a),.0055),yaw=a)
          for a in (i*math.tau/16 for i in range(16))),
        Solid('box',(.033,.103,.029),(-.082,0.,-.006),color=MOTOR_BLUE),
        Solid('box',(.003,.043,.014),(-.100,.019,-.001),color=DARK),
        Solid('cylinder',(.009,.012),(-.105,-.021,-.001),axis='X',color=DARK),
    ),
    ROTOR.name:(Solid('box',(.054,.012,.006)),),
    VESSEL.name:(
        Solid('cylinder',(GLASS_R,.0028),(0.,0.,-GLASS_H/2.+.0014),color=CLEAR,opacity=.31),
        *(Solid('tapered_panel',(.0286,.0286,.0014,.0776,span),(0.,0.,.0012),
                roll=i*span,pitch=-math.pi/2,color=CLEAR,opacity=.20)
          for i in range(20)),
    ),
    BAR.name:(
        Solid('capsule_x',(BAR_R,BAR_LENGTH-2.*BAR_R),color=WHITE),
    ),
}
VISUAL_PARTS={
    VESSEL.name:(
        Solid('cylinder',(.0278,.0005),(0.,0.,-GLASS_H/2.+.00305),color=WATER),
        *(Solid('tapered_panel',(.0266,.0266,.0017,.0012,span),(0.,0.,.0077),
                roll=i*span,pitch=-math.pi/2,color=WATER) for i in range(20)),
        *(Solid('box',(.0005,.006,.001),(-.0302,.004,z),color=WHITE) for z in (-.010,.003,.016)),
    ),
    HULL.name:(
        Solid('box',(.0006,.023,.002),(-.102,.021,-.001),color=(.91,.11,.05)),
        *(Solid('box',(.018,.001,.0003),(.010,y,.0226),color=(.19,.27,.36))
          for y in (-.007,.007)),
        *(Solid('box',(.001,.018,.0003),(x,0.,.0226),color=(.19,.27,.36))
          for x in (.003,.017)),
    ),
}
OBJECT_MATERIALS={HULL.name:'molded IKA housing and glass setup plate',
                  ROTOR.name:'steel-supported rotating magnet',
                  VESSEL.name:'DURAN borosilicate beaker and 115 mL of water',
                  BAR.name:'PTFE-coated magnetic stir bar'}
SUPPORT_JOINTS={HULL.name:{'kind':'fixed'}}
OBJECT_JOINTS={ROTOR.name:{'kind':'revolute','parent':HULL.name,'axis':'Z','anchor':ROTOR.pos,
                         'drive':{'stiffness':0.,'damping':.0012,'max_force':.024,
                                  'target_velocity':math.degrees(OMEGA)}}}
SURFACE_FRICTION={VESSEL.name:(.29,.23),BAR.name:(.044,.033),HULL.name:(.42,.35)}
CONTACT_OFFSETS_M={BAR.name:.00022,VESSEL.name:.00032,ROTOR.name:.00035}
RIGID_SOLVER_ITERATIONS={BAR.name:(64,8),VESSEL.name:(48,4)}

capture_position=np.array((.010,-.022,.389))
capture_forward=np.array((0.,0.,-1.))
capture_right=np.array((0.,-1.,0.))
capture_rotation=np.column_stack((capture_right,np.cross(capture_right,capture_forward),-capture_forward))
TASK_RECORD_CAMERA=(HULL.name,Camera(tuple(capture_position),
                                     tuple(matrix_to_quat_xyzw(capture_rotation)),
                                     'opengl',Lens.from_focal(49.,640,360)))


def yaw_of(q):
    x,y,z,w=q
    return math.atan2(2.*(w*z+x*y),1.-2.*(y*y+z*z))


@dataclass(frozen=True)
class StirBarCoupling:
    width_m: float=.020
    centering_N_m: float=9.
    magnetic_torque_Nm: float=.00034
    liquid_drag_N_s_m: float=.013
    spin_drag_Nm_s: float=.000006

    def evaluate(self, rotor_pose, bar_pose, bar_twist):
        lateral=bar_pose[:2]-rotor_pose[:2]
        envelope=math.exp(-float(np.dot(lateral,lateral))/(self.width_m**2))
        force=-self.centering_N_m*envelope*lateral-self.liquid_drag_N_s_m*bar_twist[:2]
        lag=yaw_of(rotor_pose[3:])-yaw_of(bar_pose[3:])
        torque=self.magnetic_torque_Nm*envelope*math.sin(lag)-self.spin_drag_Nm_s*bar_twist[5]
        return (force[0],force[1],0.),(0.,0.,torque)


MAGNETIC_STIR=(ROTOR.name,BAR.name,StirBarCoupling())
SUCCESS_HOLD_S=10.
MAX_PARTIAL_SCORE=.5
LIVE_CONTEXT={}
_spin_history=deque(maxlen=5)
_failed=False
_turns=0.
_partial=False


def on_reset():
    global _failed,_turns,_partial
    _spin_history.clear()
    _failed=False
    _turns=0.
    _partial=False


def coupled(objects, context):
    origins=context['body_origins']
    rotor, bar, vessel=(origins[o.name] for o in (ROTOR,BAR,VESSEL))
    frequency=np.mean(_spin_history)*60./math.tau if _spin_history else 0.
    return bool(len(_spin_history)==5 and 170.<=frequency<=230.
                and math.dist(bar[:2],rotor[:2]) <= .010
                and math.dist(vessel[:2],rotor[:2]) <= .007
                and abs(objects[VESSEL.name].low[2]-PLATE)<.003)


def observe(objects, context, dt):
    global _failed,_turns,_partial
    origins=context['body_origins']
    center=origins[VESSEL.name]
    rotation=quat_xyzw_to_matrix(objects[VESSEL.name].quat_xyzw)
    bar=origins[BAR.name]
    if (rotation[2,2]<math.cos(math.radians(12.)) or math.dist(bar[:2],center[:2])>.031
        or bar[2]<center[2]-GLASS_H/2.-.004 or bar[2]>center[2]+GLASS_H/2.):
        _failed=True
    for side,hand in context['tcp'].items():
        jaw_half_open = -FINGER_OPEN*context['gripper'][side]
        if (math.dist(hand[:2],center[:2])+jaw_half_open < .024
            and hand[2]<center[2]+GLASS_H/2.-.006):
            _failed=True
    spin=float(objects[BAR.name].ang_vel[2])
    _spin_history.append(spin)
    _turns=_turns + max(0.,spin)*dt/math.tau if coupled(objects,context) else 0.
    _partial=_partial or _turns>=3.


def success(objects):
    center=LIVE_CONTEXT['body_origins'][VESSEL.name]
    return bool(not _failed and coupled(objects,LIVE_CONTEXT)
                and math.fsum(objects[VESSEL.name].lin_vel**2) < .006**2
                and min(math.dist(tuple(center), hand) for hand in LIVE_CONTEXT['tcp'].values())>=.094)


def score(objects):
    if _failed:
        return 0.
    if success(objects):
        return 1.
    return .5 if _partial else 0.


GRIP_Z=GLASS_CENTER+.011
PINCH=-.027
HOVER=GRIP_Z+.080
CLEAR_POSE=(X-.164,Y-.102,HOVER+.020)


def demo():
    return {'right':[
        Waypoint(1.25,(X,Y+OFFSET,HOVER),FINGER_OPEN,'approach the glass vessel'),
        Waypoint(1.05,(X,Y+OFFSET,GRIP_Z),FINGER_OPEN,'reach the vessel'),
        Waypoint(.85,(X,Y+OFFSET,GRIP_Z),PINCH,'grip the vessel wall'),
        Waypoint(1.8,(X,Y,GRIP_Z),PINCH,'centre on the glass plate'),
        Waypoint(.75,(X,Y,GRIP_Z),FINGER_OPEN,'release the vessel'),
        Waypoint(.85,(X,Y,HOVER),FINGER_OPEN,'lift clear of the glass'),
        Waypoint(.95,CLEAR_POSE,FINGER_OPEN,'withdraw from the stirrer'),
        Waypoint(10.90,CLEAR_POSE,FINGER_OPEN,'observe the running stir bar'),
    ]}
