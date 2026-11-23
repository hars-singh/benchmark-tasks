"""Exercise the retained-stack and second-cup penalties without rendering."""

from pathlib import Path
from types import SimpleNamespace
import math
import sys

import numpy as np
from yam_arena.task_file import load


def snapshot(scene, *, nudge=0.0, outside=False, both_out=False, at_mat=False):
    layout = {part.name: np.asarray(part.pos) for part in scene.OBJECTS}
    # Keep the same reference for the other retained rims.
    for name in scene.CUPS[1:]:
        layout[name] = layout[name] + np.array([-nudge, 0., 0.])
    if outside:
        layout[scene.CUPS[0]] = np.array([scene.FACE_X-.11, scene.CENTER_Y, scene.CENTER_Z])
    if both_out:
        layout[scene.CUPS[1]] = np.array([scene.FACE_X-.10, scene.CENTER_Y, scene.CENTER_Z])
    if at_mat:
        layout[scene.CUPS[0]] = np.array([*scene.MAT, scene.MAT_TOP+scene.CUP_LENGTH/2])
    states = {}
    for name in scene.CUPS:
        upright = at_mat and name == scene.CUPS[0]
        extent = np.array([scene.RIM_RADIUS,scene.RIM_RADIUS,scene.CUP_LENGTH/2] if upright else
                          [scene.CUP_LENGTH/2,scene.RIM_RADIUS,scene.RIM_RADIUS])
        q = (0.,-math.sqrt(.5),0.,math.sqrt(.5)) if upright else (0.,0.,0.,1.)
        states[name] = SimpleNamespace(quat_xyzw=q,low=layout[name]-extent,high=layout[name]+extent,
                                       lin_vel=np.zeros(3),ang_vel=np.zeros(3))
    scene.LIVE_CONTEXT = {'body_origins':layout,'tcp': {'left':(0.,.30,1.),'right':(0.,-.30,1.)}}
    scene.observe(states,scene.LIVE_CONTEXT,.05)
    return states


def check(scene):
    scene.on_reset()
    assert scene.score(snapshot(scene)) == 0
    assert scene.score(snapshot(scene,nudge=.008)) == 0, 'passive preload cannot score'
    assert scene.score(snapshot(scene,nudge=.008,outside=True)) == .50
    assert scene.score(snapshot(scene,nudge=.008,at_mat=True)) == 1
    assert scene.score(snapshot(scene,nudge=.008,outside=True,both_out=True)) == 0
    assert scene.score(snapshot(scene,nudge=.008,at_mat=True)) == 0, 'second-cup penalty must latch'
    scene.on_reset()
    assert scene.score(snapshot(scene)) == 0, 'reset must clear prior-episode events'
    print('Single-cup scoring checks passed')


if __name__ == '__main__':
    path = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parents[1]/'scene.py'
    check(load(path.resolve()).module)
