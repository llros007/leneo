"""URDF forward/inverse kinematics; independent of either simulation engine."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import trimesh
from .control import DESCRIPTION
from .orange_layout import NAMES,GRIP


def origin(element):
    T=np.eye(4)
    if element is not None:
        T[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return T


class RobotIK:
    def __init__(self,navigation):
        self.navigation=navigation;root=ET.parse(DESCRIPTION/'urdf/leneo.urdf').getroot()
        self.parents={};self.limits={};self.chains={};self.fingers=[]
        for j in root.findall('joint'):
            axis=np.fromstring(j.find('axis').get('xyz'),sep=' ') if j.find('axis') is not None else np.array([1,0,0])
            self.parents[j.find('child').get('link')]=(j.find('parent').get('link'),j.get('name'),j.get('type'),origin(j.find('origin')),axis)
            if j.find('limit') is not None:
                lim=j.find('limit');self.limits[j.get('name')]=(float(lim.get('lower','-3.14')),float(lim.get('upper','3.14')))
        for name in ['right_tool0','right_gripper_link','right_moving_jaw_so101_v1_link']:
            chain=[];link=name
            while link in self.parents:
                p,j,kind,T,axis=self.parents[link];chain.append((j,kind,T,axis));link=p
            self.chains[name]=list(reversed(chain))
        for name in ['right_gripper_link','right_moving_jaw_so101_v1_link']:
            pieces=[]
            for visual in root.find(f"link[@name='{name}']").findall('visual'):
                f=visual.find('geometry/mesh')
                if f is None or 'sts3215' in f.get('filename'):continue
                mesh=trimesh.load((DESCRIPTION/'urdf'/f.get('filename')).resolve(),force='mesh')
                scale=np.fromstring(f.get('scale','1 1 1'),sep=' ')
                T=origin(visual.find('origin'));vertices=(mesh.vertices*scale)@T[:3,:3].T+T[:3,3]
                pieces.append((vertices,np.asarray(mesh.faces)))
            self.fingers.append((name,pieces))

    def pose(self,name,joints=None):
        joints=self.navigation.joints if joints is None else joints
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',self.navigation.yaw).as_matrix();T[:3,3]=self.navigation.position
        for joint,kind,base,axis in self.chains[name]:
            T=T@base;value=joints.get(joint,0.)
            if kind=='prismatic':T[:3,3]+=T[:3,:3]@(axis*value)
            elif kind in ['revolute','continuous']:T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(axis*value).as_matrix()
        return T

    def grip(self,joints=None):
        T=self.pose('right_tool0',joints);return T[:3,3]+T[:3,:3]@GRIP,T[:3,:3]

    def solve(self,target,attached=False,offset=None):
        bounds=np.array([self.limits[n] for n in NAMES]);seed=np.array([self.navigation.joints[n] for n in NAMES]);best=None
        joints=dict(self.navigation.joints)
        def fun(q):
            joints.update(zip(NAMES,q));p,R=self.grip(joints)
            if attached:p+=R@offset
            return np.r_[(p-target)*10,(R[:,2]-[0,0,-1])*.12]
        for q in [seed,np.array([-.20,0,.2,1.3,-.4,0]),np.array([.30,0,-.8,1.3,-1,0])]:
            result=least_squares(fun,np.clip(q,bounds[:,0]+1e-6,bounds[:,1]-1e-6),bounds=(bounds[:,0]+1e-7,bounds[:,1]-1e-7),max_nfev=250,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            err=np.linalg.norm(fun(result.x)[:3])/10
            if best is None or err<best[0]:best=(err,result.x)
            if err<.001:break
        return dict(zip(NAMES,best[1])),float(best[0])

    def measure(self,center,rotation,radii):
        points=[];gaps=[]
        for name,pieces in self.fingers:
            T=self.pose(name);candidates=[]
            for vertices,faces in pieces:
                world=vertices@T[:3,:3].T+T[:3,3]
                mesh=trimesh.Trimesh((world-center)@rotation/radii,faces,process=False)
                point,distance,_=trimesh.proximity.closest_point_naive(mesh,[[0,0,0]])
                candidates.append((float(distance[0]),point[0]))
            distance,point=min(candidates,key=lambda x:x[0]);radial=point*radii
            gaps.append(float(np.linalg.norm(radial)*(1-1/max(distance,1e-9))));points.append(radial@rotation.T)
        a,b=points;opposition=float(a@b/(np.linalg.norm(a)*np.linalg.norm(b)))
        return dict(jaw_surface_gaps_m=gaps,opposition_cosine=opposition,between_fingers=bool(max(abs(g) for g in gaps)<.004 and opposition<-.5))
