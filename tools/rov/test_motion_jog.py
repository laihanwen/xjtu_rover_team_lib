import unittest
from motion_jog import MotionJog

class JogTests(unittest.TestCase):
    def test_refresh_cannot_extend_one_second(self):
        j=MotionJog()
        for n in range(10):
            j.request('surge+',n/10)
            self.assertEqual(j.frame(n/10,True)[0][2],159)
        self.assertTrue(j.frame(1.,True)[1])
        with self.assertRaises(ValueError):j.request('surge+',1.01)
        j.release();j.request('surge-',1.1)
        self.assertEqual(j.frame(1.1,True)[0][2],95)

    def test_loss_and_timeout_stop(self):
        for permitted,now in [(False,.1),(True,.21)]:
            j=MotionJog();j.request('yaw+',0)
            frame,expired=j.frame(now,permitted)
            self.assertTrue(expired);self.assertEqual(list(frame[1:5]),[127]*4)

    def test_axes_and_invalid_requests(self):
        j=MotionJog()
        for axis in j.CHANNELS:
            j.release();j.request(axis,0)
            frame,_=j.frame(0,True)
            self.assertEqual(sum(x!=127 for x in frame[1:5]),1)
            self.assertEqual(frame[6],0)
        with self.assertRaises(ValueError):j.request('invalid',0)
        with self.assertRaises(ValueError):j.request('surge+',0)

if __name__=='__main__':unittest.main()
