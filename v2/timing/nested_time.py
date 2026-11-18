"""Exact C density and derivatives, as a base for the existing G1 time warp."""
import numpy as np
from scipy.special import roots_legendre
from ..core.math_tools import quintic_time_jet, chain_rule

def bm(x,k):
 # bump supports are distinct original A intervals; support and first two derivatives vanish on endpoints.
 idx=np.clip(np.searchsorted(k,x,side='right')-1,0,len(k)-2); h=k[idx+1]-k[idx]; t=np.clip((x-k[idx])/h,0,1)
 p=64*t**3*(1-t)**3; p1=192*t**2*(1-t)**2*(1-2*t)/h; p2=384*t*(1-t)*(1-5*t+5*t*t)/h**2
 return idx,p,p1,p2
class Density:
 def __init__(self,k,x,active,beta):self.k=k;self.x=x;self.active=active;self.beta=beta;self.index=np.full(len(k)-1,-1,int);self.index[active]=np.arange(len(active))
 def jets(self,r):
  i,p,p1,p2=bm(r,self.k); sel=self.index[i]; b=np.where(sel>=0,self.beta[np.maximum(sel,0)] if len(self.beta) else 0.,0.); h=self.k[i+1]-self.k[i];t=np.clip((r-self.k[i])/h,0,1);delta=self.x[i+1]-self.x[i];v=self.x[i]+delta*t**3*(10+t*(-15+6*t));d=delta*30*t*t*(1-t)**2/h;e=delta*60*t*(1-t)*(1-2*t)/h**2;f=b*p;f1=b*p1;f2=b*p2;ef=np.exp(np.clip(f,-50,50));return v*ef,(d+v*f1)*ef,(e+2*d*f1+v*(f2+f1*f1))*ef
 def integrate(self,points,n=24):
  pts=np.asarray(points); ns,w=roots_legendre(n);h=np.diff(self.k);xx=(self.k[:-1,None]+self.k[1:,None])/2+h[:,None]*ns/2;prefix=np.r_[0,np.cumsum(h/2*np.sum(self.jets(xx.ravel())[0].reshape(xx.shape)*w,axis=1))]; i=np.clip(np.searchsorted(self.k,pts,side='right')-1,0,len(h)-1);dx=pts-self.k[i];xx=self.k[i,None]+dx[:,None]*(ns+1)/2;return prefix[i]+dx/2*np.sum(self.jets(xx.ravel())[0].reshape(xx.shape)*w,axis=1)
 def at(self,r):
  d,d1,d2=self.jets(r);return d,d1,d2,self.integrate(r)

class NestedBase:
 def __init__(self,path):
  with np.load(path,allow_pickle=False) as z:
   self.density=Density(z['g1_density_knots'].copy(),z['density_x'].copy(),z['active_intervals'].copy(),z['density_beta'].copy())
  self.start=float(self.density.k[0]);self.end=float(self.density.k[-1]);self.T=float(self.density.integrate(np.array([self.end]),n=64)[0])
 def from_parameter(self,chart,r):
  r=np.asarray(r,float);u,ud,udd,uj=quintic_time_jet(r,self.end);jet=chart.evaluate_global(np.clip(u,chart.s0,chart.s1));v,a,j=chain_rule(jet,ud,udd,uj);L,D,E=self.density.jets(r); l=L[:,None];d=D[:,None];e=E[:,None]
  return self.density.integrate(r,n=24),u,jet.q,v/l,a/l**2-v*d/l**3,j/l**3-3*a*d/l**4-v*e/l**4+3*v*d**2/l**5
