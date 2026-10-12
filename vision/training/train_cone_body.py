"""Train/export alternative B from reviewed body masks; never invent labels.

Manifest JSON: records [{image, mask, split:train|val, group, reviewed:true}].
Mask classes: 0 background (including bases), 1 circle body, 2 square body.
Groups must be disjoint across train/val, including source recording/instances.
Uses the existing isolated PyTorch environment; no package installation.
"""
import argparse
import hashlib
import json
from pathlib import Path


def validate_manifest(path):
    manifest=json.loads(path.read_text(encoding='utf-8'))
    records=manifest.get('records',[]);groups={'train':set(),'val':set()};images=set()
    if not records:raise ValueError('reviewed body masks required; empty manifest cannot train')
    for record in records:
        if record.get('reviewed') is not True or record.get('split') not in groups or not record.get('group'):
            raise ValueError('explicit reviewed labels, split and recording/instance group required')
        for key in ['image','mask']:
            file=(path.parent/record[key]).resolve()
            if not file.is_file():raise ValueError(str(file))
        image=(path.parent/record['image']).resolve()
        digest=hashlib.sha256(image.read_bytes()).hexdigest()
        if digest in images:raise ValueError('duplicate image across dataset')
        images.add(digest);groups[record['split']].add(record['group'])
    if not all(groups.values()) or groups['train']&groups['val']:
        raise ValueError('train/validation groups must exist and be disjoint')
    return records


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('manifest',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--epochs',type=int,default=50);p.add_argument('--device',default='cpu');p.add_argument('--seed',type=int,default=18)
    p.add_argument('--check-only',action='store_true')
    a=p.parse_args();records=validate_manifest(a.manifest)
    if a.check_only:print('Manifest grouping/review gate passed:',len(records));return
    if a.epochs<1 or a.output.exists():raise ValueError('positive epochs and new output directory required')
    import cv2
    import numpy as np
    import torch
    from torch import nn
    torch.manual_seed(a.seed);rng=np.random.default_rng(a.seed)

    class Dataset(torch.utils.data.Dataset):
        def __init__(self,split):self.rows=[r for r in records if r['split']==split];self.train=split=='train'
        def __len__(self):return len(self.rows)
        def __getitem__(self,index):
            row=self.rows[index];image=cv2.imread(str(a.manifest.parent/row['image']),cv2.IMREAD_GRAYSCALE)
            mask=cv2.imread(str(a.manifest.parent/row['mask']),cv2.IMREAD_GRAYSCALE)
            if image is None or mask is None or image.shape!=mask.shape or not np.isin(mask,[0,1,2]).all():
                raise ValueError('native image/mask mismatch or illegal body classes')
            image=cv2.resize(image,(320,320)).astype(np.float32)/255
            mask=cv2.resize(mask,(320,320),interpolation=cv2.INTER_NEAREST)
            if self.train:
                # Change stripe intensity/count within annotated bodies while
                # leaving semantic shape labels intact. This is augmentation,
                # not synthetic ground truth or a navigation acceptance claim.
                if rng.random()<.7:
                    period=int(rng.integers(8,60));stripe=(np.arange(320)//period)%2
                    strength=float(rng.uniform(-.25,.25));image[mask>0]=np.clip(
                        image+strength*stripe[:,None],0,1)[mask>0]
                image=np.clip(image*rng.uniform(.7,1.3)+rng.uniform(-.1,.1),0,1)
                if rng.random()<.5:image=np.fliplr(image).copy();mask=np.fliplr(mask).copy()
            return torch.from_numpy(np.repeat(image[None],3,axis=0)),torch.from_numpy(mask.astype(np.int64))

    class Network(nn.Module):
        def __init__(self):
            super().__init__()
            def block(source,target,stride=1):return nn.Sequential(nn.Conv2d(source,target,3,stride=stride,padding=1),nn.ReLU(),nn.Conv2d(target,target,3,padding=1),nn.ReLU())
            self.e1=block(3,12,2);self.e2=block(12,24,2);self.e3=block(24,32,2);self.e4=block(32,48,2)
            self.d3=block(80,32);self.d2=block(56,24);self.d1=block(36,16);self.head=nn.Conv2d(16,3,1)
        def forward(self,x):
            def up(v,size):return nn.functional.interpolate(v,size=size,mode='bilinear',align_corners=False)
            a=self.e1(x);b=self.e2(a);c=self.e3(b);d=self.e4(c)
            v=self.d3(torch.cat([up(d,c.shape[-2:]),c],1));v=self.d2(torch.cat([up(v,b.shape[-2:]),b],1))
            v=self.d1(torch.cat([up(v,a.shape[-2:]),a],1));return self.head(up(v,(320,320)))

    train=Dataset('train');validation=Dataset('val');device=torch.device(a.device)
    model=Network().to(device);optimizer=torch.optim.AdamW(model.parameters(),lr=1e-3)
    criterion=nn.CrossEntropyLoss();history=[]
    for epoch in range(a.epochs):
        model.train();losses=[]
        for image,mask in torch.utils.data.DataLoader(train,batch_size=2,shuffle=True,num_workers=0):
            optimizer.zero_grad();loss=criterion(model(image.to(device)),mask.to(device));loss.backward();optimizer.step()
            losses.append(float(loss.detach()))
        model.eval();confusion=np.zeros((3,3),dtype=np.int64)
        with torch.no_grad():
            for image,mask in torch.utils.data.DataLoader(validation,batch_size=1,num_workers=0):
                predicted=model(image.to(device)).argmax(1).cpu().numpy().reshape(-1);truth=mask.numpy().reshape(-1)
                confusion+=np.bincount(truth*3+predicted,minlength=9).reshape(3,3)
        union=confusion.sum(0)+confusion.sum(1)-confusion.diagonal()
        iou=np.divide(confusion.diagonal(),union,out=np.zeros(3),where=union>0)
        history.append(dict(epoch=epoch+1,loss=float(np.mean(losses)),iou=iou.tolist(),confusion=confusion.tolist()))
        print(json.dumps(history[-1]),flush=True)
    a.output.mkdir(parents=True)
    model=model.cpu().eval();torch.save(model.state_dict(),a.output/'body.pt')
    probability=nn.Sequential(model,nn.Softmax(dim=1))
    torch.onnx.export(probability,torch.zeros(1,3,320,320),str(a.output/'body.onnx'),
                      input_names=['input'],output_names=['output'],opset_version=13,dynamo=False)
    metadata=dict(schema='cone.body.training.v1',shape_model_verified=False,seed=a.seed,
                  manifest_sha256=hashlib.sha256(a.manifest.read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256((a.output/'body.onnx').read_bytes()).hexdigest(),
                  history=history,contract='320x320 grayscale replicated /255; NCHW three probabilities',
                  limitations=['Validation pixels do not prove grid association or navigation',
                               'Pi FPS, field stripe variants and full-body/base labels require acceptance'])
    (a.output/'training.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')


if __name__=='__main__':main()
