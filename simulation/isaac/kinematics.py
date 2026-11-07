"""Prescribe USD link transforms without starting PhysX in navigation mode."""
import math
import xml.etree.ElementTree as ET
from pxr import Usd,UsdGeom,Gf
from common.control import DESCRIPTION


class Kinematics:
    def __init__(self,stage,root):
        self.root=root;self.entries={};self.previous={}
        prims={p.GetName():p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Xform) and not p.IsInstance()}
        for joint in ET.parse(DESCRIPTION/'urdf/leneo.urdf').getroot().findall('joint'):
            kind=joint.get('type')
            if kind=='fixed':continue
            name=joint.get('name');child=joint.find('child').get('link')
            if child not in prims:raise RuntimeError(f'Missing moving link {child}')
            axis=Gf.Vec3d(*map(float,joint.find('axis').get('xyz').split()))
            xform=UsdGeom.Xformable(prims[child]);base=xform.GetLocalTransformation()
            xform.ClearXformOpOrder();op=xform.AddTransformOp(opSuffix='navigation')
            self.entries[name]=(kind,axis,base,op)
        xform=UsdGeom.Xformable(root);xform.ClearXformOpOrder()
        self.translate=xform.AddTranslateOp(opSuffix='navigation');self.orient=xform.AddOrientOp(opSuffix='navigation')

    def apply(self,navigation):
        self.translate.Set(Gf.Vec3d(*navigation.position))
        q=navigation.quaternion;self.orient.Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        for name,(kind,axis,base,op) in self.entries.items():
            value=navigation.joints.get(name,0.0)
            if self.previous.get(name)==value:continue
            if kind=='prismatic':motion=Gf.Matrix4d().SetTranslate(axis*value)
            else:motion=Gf.Matrix4d().SetRotate(Gf.Rotation(axis,math.degrees(value)))
            op.Set(motion*base);self.previous[name]=value
