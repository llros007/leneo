"""Author crease-aware smooth visual normals; no topology or collision changes."""
from pathlib import Path
import numpy as np
import trimesh
from pxr import Usd,UsdGeom,Vt


def apply(robot_file):
    path=Path(robot_file).parent/'payloads/geometries.usd'
    stage=Usd.Stage.Open(str(path));count=0
    for p in stage.Traverse():
        if not p.IsA(UsdGeom.Mesh) or '_hull' in str(p.GetPath()):continue
        mesh=UsdGeom.Mesh(p)
        vertices=np.array(mesh.GetPointsAttr().Get(),dtype=float)
        indices=np.array(mesh.GetFaceVertexIndicesAttr().Get(),dtype=int)
        sizes=np.array(mesh.GetFaceVertexCountsAttr().Get())
        if not np.all(sizes==3):continue
        faces=indices.reshape(-1,3)
        # Coincident STL points must share normals across smooth triangles.
        _,inverse=np.unique(np.round(vertices,7),axis=0,return_inverse=True)
        tri=vertices[faces]
        area_normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
        length=np.linalg.norm(area_normals,axis=1)
        face_normals=area_normals/np.maximum(length[:,None],1e-20)
        accumulated=np.zeros((inverse.max()+1,3))
        for corner in range(3):np.add.at(accumulated,inverse[faces[:,corner]],area_normals)
        accumulated/=np.maximum(np.linalg.norm(accumulated,axis=1)[:,None],1e-20)
        smooth=accumulated[inverse[faces]]
        alignment=np.einsum('fci,fi->fc',smooth,face_normals)
        normals=np.where((alignment>=np.cos(np.deg2rad(45)))[:,:,None],smooth,face_normals[:,None,:])
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.reshape(-1,3).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        count+=1
    stage.GetRootLayer().Save();print('Crease-aware visual normals:',count,flush=True)
    return count
