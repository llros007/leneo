"""Copy leisaac Orange001 and export a centred, UV-preserving MuJoCo mesh."""
from pathlib import Path
import json
import os
import shutil
import numpy as np
from pxr import Usd,UsdGeom
from .control import ROOT

SOURCE=Path(os.environ.get('LENEO_ORANGE_SOURCE', str(ROOT/'assets/orange/Orange001')))
OUTPUT=ROOT/'assets/orange'


def prepare():
    manifest=OUTPUT/'manifest.json'
    if manifest.exists():return json.loads(manifest.read_text())
    if not (OUTPUT/'Orange001/Orange001.usd').exists():shutil.copytree(SOURCE,OUTPUT/'Orange001',dirs_exist_ok=True)
    stage=Usd.Stage.Open(str(OUTPUT/'Orange001/Orange001.usd'))
    prim=stage.GetPrimAtPath('/root/Orange001/Visuals/Orange001');mesh=UsdGeom.Mesh(prim)
    points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
    matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
    points=(np.column_stack([points,np.ones(len(points))])@matrix)[:,:3]
    counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get(),dtype=int)
    indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),dtype=int)
    uv=UsdGeom.PrimvarsAPI(mesh).GetPrimvar('st');uv=np.asarray(uv.ComputeFlattened())
    if len(uv)==len(points):uv=uv[indices]
    if len(uv)!=len(indices):raise ValueError('Unsupported UV indexing')
    corners=points[indices];bounds=np.stack([points.min(axis=0),points.max(axis=0)])
    center=bounds.mean(axis=0);corners-=center
    lines=['# Orange001 copied from local leisaac; world metres, mesh centred.']
    lines.extend('v %.9f %.9f %.9f'%tuple(v) for v in corners)
    lines.extend('vt %.9f %.9f'%tuple(v) for v in uv)
    start=0
    for count in counts:
        for i in range(1,count-1):
            corners_i=[start+1,start+i+1,start+i+2]
            lines.append('f '+' '.join(f'{v}/{v}' for v in corners_i))
        start+=count
    (OUTPUT/'orange.obj').write_text('\n'.join(lines)+'\n')
    shutil.copy2(OUTPUT/'Orange001/texture/T_Orange001_BC001.png',OUTPUT/'albedo.png')
    result=dict(source=str(SOURCE),object='Orange001',dimensions=(bounds[1]-bounds[0]).tolist(),half_height=float((bounds[1,2]-bounds[0,2])/2),mesh_center_offset=center.tolist(),mass_kg=.09,collision_radius_m=.027)
    manifest.write_text(json.dumps(result,indent=2))
    (OUTPUT/'SOURCE.md').write_text('Copied from local leisaac asset: '+str(SOURCE)+'\nUSD and original BC/ORM textures are retained. orange.obj is transformed to metres and centred, with original UV coordinates. Asset provenance is retained; no new asset licence is asserted.\n')
    return result

if __name__=='__main__':print(json.dumps(prepare(),indent=2))
