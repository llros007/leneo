"""Remove the front dining peninsula, preserving neighbouring cabinets and UVs."""
import numpy as np
import trimesh

EDITION='without-dining-peninsula-v1'


def remove_dining_table(mesh,node):
    if not node.startswith(('Cabinets_','WoodCabinet_')):return mesh
    # The rounded dining table is part of the cabinet mesh, not a separate node.
    # Remove x < 1.89, y < .09, z < 1.15. The island and chairs are separate.
    planes=[(0,1.89),(1,.09),(2,1.15)]
    uv=getattr(mesh.visual,'uv',None)
    vertices=[];texcoords=[];faces=[]
    def clip(poly,axis,bound,positive):
        out=[]
        for a,b in zip(poly,poly[1:]+poly[:1]):
            va=(a[axis]-bound)*(1 if positive else -1);vb=(b[axis]-bound)*(1 if positive else -1)
            if va>=0:out.append(a)
            if (va>=0)!=(vb>=0):out.append(a+(b-a)*(va/(va-vb)))
        return out
    def emit(poly):
        for j in range(1,len(poly)-1):
            tri=[poly[0],poly[j],poly[j+1]]
            if np.linalg.norm(np.cross(tri[1][:3]-tri[0][:3],tri[2][:3]-tri[0][:3]))<1e-12:continue
            index=len(vertices);vertices.extend([v[:3] for v in tri]);faces.append([index,index+1,index+2])
            if uv is not None:texcoords.extend([v[3:] for v in tri])
    for face in mesh.faces:
        poly=[np.r_[mesh.vertices[i],uv[i]] if uv is not None else mesh.vertices[i].copy() for i in face]
        for axis,bound in planes:
            if not poly:break
            emit(clip(poly,axis,bound,True));poly=clip(poly,axis,bound,False)
    if not faces:return None
    result=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
    if uv is not None:result.visual=trimesh.visual.TextureVisuals(uv=texcoords,material=mesh.visual.material)
    else:result.visual=mesh.visual.copy()
    return result
