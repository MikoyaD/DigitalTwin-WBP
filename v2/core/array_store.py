"""Lossless, explicit array bundles for PRP; legacy NPZ is read-only."""
from pathlib import Path
import numpy as np

class Bundle:
 def __init__(self,path):
  self.path=Path(path);self.legacy=None
  if self.path.is_dir():
   self.files=sorted(x.stem for x in self.path.glob('*.npy'))
  else:
   self.legacy=np.load(self.path,allow_pickle=False);self.files=self.legacy.files
 def __getitem__(self,key):
  if key not in self.files:raise KeyError(key)
  return self.legacy[key] if self.legacy is not None else np.load(self.path/(key+'.npy'),allow_pickle=False)
 def __contains__(self,key):return key in self.files
 def __enter__(self):return self
 def __exit__(self,*args):self.close()
 def close(self):
  if self.legacy is not None:self.legacy.close()

def load(path,allow_pickle=False):
 if allow_pickle:raise ValueError('PICKLE_NOT_SUPPORTED')
 return Bundle(path)

def save(path,**arrays):
 path=Path(path)
 if path.suffix!='.arrays':raise ValueError('ARRAY_BUNDLE_REQUIRES_ARRAYS_SUFFIX')
 path.mkdir(parents=True,exist_ok=False)
 for key,value in arrays.items():
  if not key.isidentifier():raise ValueError('INVALID_ARRAY_NAME')
  np.save(path/(key+'.npy'),np.asarray(value),allow_pickle=False)
 return path
