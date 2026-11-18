"""Pure signed geometry and reference-time evaluators used by G0 A+C."""
import numpy as np
import json
from scipy.special import roots_legendre
from scipy.optimize import brentq
from ..core.math_tools import quintic_basis

def basis(x,n,d=0,knots=None):return quintic_basis(np.asarray(x,float),n,d,knots)
def phi(s):return s**3*(10+s*(-15+6*s))
def phi_inv(u):
 if u<=0:return 0.
 if u>=1:return 1.
 return brentq(lambda s:phi(s)-u,0,1,xtol=1e-14)
def phi_jet(s):return phi(s),30*s*s*(1-s)**2,60*s*(1-s)*(1-2*s),60*(1-6*s+6*s*s)
def chart_events(c):
 """Global geometric knot events without changing a frozen chart."""
 if hasattr(c,'inner'):
  return c.s0+(c.s1-c.s0)*c.inner.knots
 if hasattr(c,'spline'):
  return c.s0+(c.s1-c.s0)*c.spline.knots
 values=list(c.s0+(c.s1-c.s0)*c.rho.knots)
 lo,hi=c.profile_interval or (c.s0,c.s1)
 for spline in (c.theta,c.arm): values.extend(lo+(hi-lo)*spline.knots)
 return np.asarray(values)

class LegacyChart:
 def __init__(self,c):
  self.inner=c;self.a=c.s0;self.b=c.s1;self.h=self.b-self.a;self.sign=0 if getattr(c,'stationary',False) else 1
  self.knots=np.clip((chart_events(c)-self.a)/self.h,0,1);self.anchor=c.evaluate_global([self.a]).q[0,:2]
 def evaluate(self,u):
  jet=self.inner.evaluate_global(u);return jet.q,jet.qs,jet.qss,jet.qsss
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
  if 'legacy_geometry_file' in z:
   from ..geometry.tm_rho_feedback_frozen import thaw
   with open(str(z['legacy_geometry_file'])) as f:block=thaw(json.load(f))
   self.edges=np.array([block.charts[0].s0]+[c.s1 for c in block.charts]);self.cs=[LegacyChart(c) for c in block.charts];return
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
class Law:
 def __init__(self,knots,values):
  self.k=np.asarray(knots);self.v=np.asarray(values);self.h=np.diff(self.k);self.prefix=np.r_[0,np.cumsum(self.h*(self.v[:-1]+self.v[1:])/2)]
 def at(self,s):
  s=np.asarray(s);i=np.clip(np.searchsorted(self.k,s,side='right')-1,0,len(self.k)-2);h=self.h[i];x=np.clip((s-self.k[i])/h,0,1);delta=self.v[i+1]-self.v[i];smooth=x**3*(10+x*(-15+6*x));d=30*x*x*(1-x)**2;dd=60*x*(1-x)*(1-2*x);integ=x**4*(2.5+x*(-3+x));return self.v[i]+delta*smooth,delta*d/h,delta*dd/h**2,self.prefix[i]+h*(self.v[i]*x+delta*integ)
def timeline(geom,s,law):
 u,us,uss,usss=phi_jet(s);jets,idx=geom.evaluate(u);q,qu,quu,quuu=jets;V=qu*us[:,None];A=quu*us[:,None]**2+qu*uss[:,None];J=quuu*us[:,None]**3+3*quu*(us*uss)[:,None]+qu*usss[:,None];L,Ls,Lss,t=law.at(s);v=V/L[:,None];a=A/L[:,None]**2-V*(Ls/L**3)[:,None];j=J/L[:,None]**3-3*A*(Ls/L**4)[:,None]-V*(Lss/L**4)[:,None]+3*V*(Ls**2/L**5)[:,None];return u,q,v,a,j,t,idx
