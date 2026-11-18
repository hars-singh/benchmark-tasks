"""Reject empty cranking, absent feed pressure and reversible-angle exploits."""
from pathlib import Path
from types import SimpleNamespace
import math
import unittest
import numpy as np
from yam_arena.task_file import load

class SharpenerTests(unittest.TestCase):
    def setUp(self):
        self.m=load(Path(__file__).resolve().parents[1]/'scene.py').module
        self.m.on_reset()

    def frame(self,angle,*,pressure=True,inserted=True,aligned=True):
        m=self.m;tool=np.asarray(m.TOOL.pos);wheel=np.asarray(m.WHEEL.pos)
        pencil=np.array((.164,-.04,m.TOP+.12)) if inserted else np.array((.120,.10,m.TOP+.034))
        cutter=np.asarray(m.CUTTER.pos)+np.array((.003 if pressure else 0,0,0))
        context={'time_s':10.,'tcp':{'left':pencil+np.array((0,0,.03)),'right':np.array((.331,-.04,m.TOP+.085))},
                 'body_origins':{'tool':tool,'wheel':wheel,'knob':wheel+np.array((.046,.035*math.cos(angle),.035*math.sin(angle))),
                                 'pencil':pencil,'cutter':cutter},'gripper':{'left':.07,'right':.16}}
        quat=np.array((0,0,0,1.)) if aligned else np.array((0,0,math.sin(.2),math.cos(.2)))
        objects={'pencil':SimpleNamespace(quat_xyzw=quat,low=pencil-np.array((.07,.0035,.0035)))}
        m.LIVE_CONTEXT=context;m.observe(objects,context,.05)
        return objects

    def revolve(self,turns,**kwargs):
        for a in np.linspace(-math.pi/2,-math.pi/2-turns*2*math.pi,round(abs(turns)*80)+1):self.frame(a,**kwargs)

    def test_empty_cranking_cannot_sharpen(self):
        self.revolve(9,inserted=False)
        self.assertEqual(self.m._loaded_rotation,0.)
        self.assertFalse(self.m._sharpened)

    def test_alignment_without_axial_load_cannot_sharpen(self):
        self.revolve(9,pressure=False)
        self.assertFalse(self.m._fed)
        self.assertFalse(self.m._sharpened)

    def test_misaligned_pencil_cannot_sharpen(self):
        self.revolve(9,aligned=False)
        self.assertFalse(self.m._sharpened)

    def test_eight_loaded_turns_advance_material_state(self):
        self.revolve(8.1)
        self.assertTrue(self.m._fed)
        self.assertTrue(self.m._sharpened)

    def test_small_back_and_forth_does_not_accumulate_turns(self):
        for _ in range(100):
            self.frame(-math.pi/2-.3)
            self.frame(-math.pi/2)
        self.assertAlmostEqual(self.m._loaded_rotation,0.,places=9)
        self.assertFalse(self.m._sharpened)

    def test_reset_clears_material_and_feed_history(self):
        self.revolve(8.1);self.m.on_reset()
        self.assertFalse(self.m._sharpened)
        self.assertFalse(self.m._fed)
        self.assertEqual(self.m._loaded_rotation,0.)

    def test_depressed_cutter_without_pencil_contact_is_not_feed(self):
        self.frame(-math.pi/2)
        context=self.m.LIVE_CONTEXT
        context['body_origins']['pencil'][0]-=.004
        objects={'pencil':SimpleNamespace(quat_xyzw=np.array((0,0,0,1.)))}
        self.assertFalse(self.m.feed_state(objects,context)[0])

    def test_either_visually_identical_pencil_end_can_be_fed(self):
        self.frame(-math.pi/2)
        context=self.m.LIVE_CONTEXT
        objects={'pencil':SimpleNamespace(quat_xyzw=np.array((0.,0.,1.,0.)))}
        loaded,_,tip,end=self.m.feed_state(objects,context)
        self.assertTrue(loaded)
        self.assertEqual(end,-1)
        self.assertAlmostEqual(tip[0],.234)

    def test_partial_cuts_on_opposite_ends_do_not_combine(self):
        self.revolve(4.1)
        original=self.m.feed_state
        def opposite_end(objects,context):
            loaded,force,tip,_=original(objects,context)
            return loaded,force,tip,-1
        self.m.feed_state=opposite_end
        for a in np.linspace(-math.pi/2-4.1*2*math.pi,-math.pi/2-8.2*2*math.pi,330):self.frame(a)
        self.assertFalse(self.m._sharpened)
        self.assertLess(self.m._loaded_rotation,4.2*2*math.pi)

if __name__=='__main__':unittest.main()
