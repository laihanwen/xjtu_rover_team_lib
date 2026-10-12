import json,tempfile,unittest
from pathlib import Path
from train_cone_body import validate_manifest
class ManifestTest(unittest.TestCase):
    def test_review_and_group_gate(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name in ['a','b','mask']:(root/name).write_bytes(name.encode())
            rows=[dict(image='a',mask='mask',split='train',group='recording-a',reviewed=True),dict(image='b',mask='mask',split='val',group='recording-b',reviewed=True)]
            path=root/'manifest.json'
            def save():path.write_text(json.dumps({'records':rows}))
            save();self.assertEqual(len(validate_manifest(path)),2)
            rows[1]['group']='recording-a';save()
            with self.assertRaises(ValueError):validate_manifest(path)
            rows[1]['group']='recording-b';rows[1]['reviewed']=False;save()
            with self.assertRaises(ValueError):validate_manifest(path)
    def test_empty_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manifest.json';path.write_text('{"records":[]}')
            with self.assertRaises(ValueError):validate_manifest(path)
if __name__=='__main__':unittest.main()
