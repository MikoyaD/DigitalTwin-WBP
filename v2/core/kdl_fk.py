"""Small URDF chain adapter for a PyKDL FK-only feasibility experiment."""
import xml.etree.ElementTree as ET
import PyKDL as k
import numpy as np
from scipy.spatial.transform import Rotation
class LocalFK:
 def __init__(self,urdf):
  root=ET.fromstring(urdf); js=[j for j in root.findall('joint') if j.find('parent') is not None]; bychild={j.find('child').get('link'):j for j in js}; chain=[];child='Link6'
  while child!='car_link':
   j=bychild[child];chain.append(j);child=j.find('parent').get('link')
  chain.reverse();expect=['base_to_car']+['joint%d'%i for i in range(1,7)]
  if [j.get('name') for j in chain]!=expect:raise ValueError('UNEXPECTED_CHAIN')
  self.chain=k.Chain()
  for j in chain:
   o=j.find('origin');xyz=[float(z) for z in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()];rpy=[float(z) for z in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
   f=k.Frame(k.Rotation.RPY(*rpy),k.Vector(*xyz));kind=j.get('type')
   if kind=='fixed': joint=k.Joint(j.get('name'),getattr(k.Joint,'None'))
   elif kind in ('revolute','continuous'):
    ax=j.find('axis');axis=[float(z) for z in (ax.get('xyz','1 0 0') if ax is not None else '1 0 0').split()];joint=k.Joint(j.get('name'),f.p,f.M*k.Vector(*axis),k.Joint.RotAxis)
   else:raise ValueError('UNSUPPORTED_JOINT '+kind)
   self.chain.addSegment(k.Segment(j.find('child').get('link'),joint,f))
  self.solver=k.ChainFkSolverPos_recursive(self.chain)
 def pose(self,q):
  a=k.JntArray(6)
  for i,x in enumerate(q[3:]):a[i]=float(x)
  p=k.Frame();rc=self.solver.JntToCart(a,p)
  if rc<0:raise ValueError('KDL_FK_FAILED %d'%rc)
  world=k.Frame(k.Rotation.RotZ(float(q[2])),k.Vector(float(q[0]),float(q[1]),0))*p
  return np.array([world.p[i] for i in range(3)]),Rotation.from_quat(world.M.GetQuaternion())
