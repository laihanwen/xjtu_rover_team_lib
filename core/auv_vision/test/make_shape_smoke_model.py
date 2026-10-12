"""Generate a tiny synthetic ONNX fixture; never a trained navigation model.

The fixture labels bright pixels as class 1 to test the real OpenCV DNN path.
Uses protobuf wire encoding directly so test generation needs only Python stdlib.
"""
import struct
import sys
from pathlib import Path


def integer(n):
    data=bytearray()
    while n>=128:data.append((n&127)|128);n>>=7
    data.append(n);return bytes(data)


def number(field,n):return integer(field<<3)+integer(n)
def message(field,data):return integer((field<<3)|2)+integer(len(data))+data
def text(field,value):return message(field,value.encode())


def tensor(name,dims,values):
    return (b''.join(number(1,n) for n in dims)+number(2,1)+text(8,name)+
            message(9,struct.pack('<'+'f'*len(values),*values)))


def value(name):
    shape=b''.join(message(1,number(1,n)) for n in [1,3,320,320])
    return text(1,name)+message(2,message(1,number(1,1)+message(2,shape)))


conv=(text(1,'input')+text(1,'weights')+text(1,'bias')+text(2,'logits')+text(4,'Conv')+
      message(5,text(1,'kernel_shape')+number(8,1)+number(8,1)+number(20,7)))
softmax=(text(1,'logits')+text(2,'output')+text(4,'Softmax')+
         message(5,text(1,'axis')+number(3,1)+number(20,2)))
graph=(message(1,conv)+message(1,softmax)+text(2,'synthetic_smoke_only')+
       message(5,tensor('weights',[3,3,1,1],[-10,0,0,10,0,0,0,0,0]))+
       message(5,tensor('bias',[3],[5,-5,-20]))+message(11,value('input'))+
       message(12,value('output')))
model=number(1,7)+text(2,'auv-synthetic-test')+message(7,graph)+message(8,number(2,13))
Path(sys.argv[1]).write_bytes(model)
