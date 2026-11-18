"""Frozen signed charts: actual robot heading and signed nonholonomic progress."""
import numpy as np
from scipy.special import roots_legendre
from ..core.math_tools import quintic_basis
from ..core.contracts import PathJet, StateSchema, require
from .unified_block import MotionBlock

def basis(x,n,d=0,knots=None):return quintic_basis(np.asarray(x,float),n,d,knots)
class Chart:
 def __init__(self,index,z,anchor):
  self.i=index;self.a=z['global_u'][index];self.b=z['global_u'][index+1];self.h=self.b-self.a;self.sign=int(z['directions'][index]);self.p=z['profile_%d'%index];self.r=z['rho_%d'%index] if self.sign else None;self.anchor=anchor.copy();self.pknots=z['knots_theta'];self.rknots=z['knots_rho'];self.knots=np.unique(np.r_[self.pknots,self.rknots]);self.n,self.w=roots_legendre(20);self.prefix=np.zeros((len(self.knots),2))
  if self.sign:
   for i in range(1,len(self.knots)):self.prefix[i]=self.prefix[i-1]+self.intseg(np.array([self.knots[i-1]]),np.array([self.knots[i]]))[0]
 def intseg(self,a,b):
  x=(a[:,None]+b[:,None])/2+(b-a)[:,None]*self.n/2;flat=x.ravel();th=(basis(flat,len(self.p),knots=self.pknots)@self.p[:,0]).reshape(x.shape);r=(basis(flat,len(self.r),knots=self.rknots)@self.r).reshape(x.shape);ee=np.stack((np.cos(th),np.sin(th)),axis=2);return self.h*(b-a)[:,None]/2*np.einsum('j,ij,ijk->ik',self.w,r,ee)
 def evaluate(self,u):
  x=np.clip((np.asarray(u)-self.a)/self.h,0,1);P=[basis(x,len(self.p),i,self.pknots)@self.p/self.h**i for i in range(4)]
  if self.sign:
   idx=np.clip(np.searchsorted(self.knots,x,side='right')-1,0,len(self.knots)-1);integral=self.prefix[idx]+self.intseg(self.knots[idx],x);xy=self.anchor+self.sign*integral;r=[basis(x,len(self.r),i,self.rknots)@self.r/self.h**i for i in range(3)];e=np.column_stack((np.cos(P[0][:,0]),np.sin(P[0][:,0])));ep=np.column_stack((-np.sin(P[0][:,0]),np.cos(P[0][:,0])));xy1=self.sign*r[0][:,None]*e;xy2=self.sign*(r[1][:,None]*e+(r[0]*P[1][:,0])[:,None]*ep);xy3=self.sign*((r[2]-r[0]*P[1][:,0]**2)[:,None]*e+(2*r[1]*P[1][:,0]+r[0]*P[2][:,0])[:,None]*ep)
  else:xy=np.broadcast_to(self.anchor,(len(x),2));xy1=np.zeros_like(xy);xy2=xy1;xy3=xy1
  return tuple(np.column_stack((v,p)) for v,p in zip((xy,xy1,xy2,xy3),P))
class Geometry:
 def __init__(self,z):
  self.edges=z['global_u'];self.cs=[];anchor=z['start_xy'].copy()
  for i in range(len(self.edges)-1):
   c=Chart(i,z,anchor);self.cs.append(c);anchor=c.evaluate(np.array([c.b]))[0][0,:2]
 def evaluate(self,u):
  u=np.asarray(u);idx=np.clip(np.searchsorted(self.edges,u,side='right')-1,0,len(self.cs)-1);jets=[np.empty((len(u),9)) for _ in range(4)]
  for i,c in enumerate(self.cs):
   where=np.flatnonzero(idx==i)
   if len(where):
    value=c.evaluate(u[where])
    for d in range(4):jets[d][where]=value[d]
  return jets,idx

class FrozenSignedChart:
 def __init__(self, chart):
  self.inner=chart; self.s0=chart.a; self.s1=chart.b; self.direction=chart.sign; self.stationary=not chart.sign
 def evaluate_global(self,u): return PathJet(*self.inner.evaluate(u))

def thaw_signed(record):
 require(record.get('algorithm_variant')=='V2-SIGNED-H-v1','TM_SIGNED_CONTRACT','signed variant required')
 with np.load(record['signed_npz'],allow_pickle=False) as source:
  geom=Geometry({k:source[k] for k in source.files})
 schema=StateSchema(**record['schema'])
 block=MotionBlock('signed_tm_000',tuple(FrozenSignedChart(c) for c in geom.cs),schema)
 require(np.max(np.abs(block.start-np.asarray(record['start'])))<1e-6 and np.max(np.abs(block.end-np.asarray(record['end'])))<1e-6,'TM_SIGNED_ENDPOINT','mismatch')
 return block
