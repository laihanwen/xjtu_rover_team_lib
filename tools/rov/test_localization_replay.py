import json,unittest
from prepare_localization_replay import align
class ReplayTests(unittest.TestCase):
 def test_past_input_only_and_gap(self):
  events=[{'event':'LOCALIZATION','detail':json.dumps({'input':{'stamp':1,'valid':True,'armed':False}})},{'event':'LOCALIZATION','detail':json.dumps({'input':{'stamp':1.2,'valid':True}})}]
  result=list(align([{'capture_monotonic_sec':x} for x in [.9,1.1,1.19,1.25]],events))
  self.assertFalse(result[0]['valid']);self.assertEqual(result[1]['stamp'],1);self.assertFalse(result[2]['valid']);self.assertEqual(result[3]['stamp'],1.2)
 def test_reboot_rejected(self):
  with self.assertRaises(ValueError):list(align([{'capture_monotonic_sec':2},{'capture_monotonic_sec':1}],[]))
