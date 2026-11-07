"""Build cached visual LODs; keep source CAD, collision meshes and UV kitchen intact."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from .control import DESCRIPTION, ROOT

OUTPUT = ROOT / 'assets/runtime_robot'
VERSION = 4


def prepare(force=False):
    files = sorted((DESCRIPTION/'meshes').rglob('*'))
    signature = hashlib.sha256(json.dumps([(str(p.relative_to(DESCRIPTION)),p.stat().st_size,p.stat().st_mtime_ns) for p in files if p.is_file()]).encode()+str(VERSION).encode()).hexdigest()
    report_path = OUTPUT/'manifest.json'
    if not force and report_path.exists():
        report=json.loads(report_path.read_text())
        if report.get('signature')==signature:return report
    OUTPUT.mkdir(parents=True,exist_ok=True)
    entries=[]
    for source in files:
        if not source.is_file():continue
        rel=source.relative_to(DESCRIPTION);target=OUTPUT/rel;target.parent.mkdir(parents=True,exist_ok=True)
        if source.suffix.lower() not in ['.stl','.obj'] or 'collision' in rel.parts:
            shutil.copy2(source,target);continue
        mesh=trimesh.load(source,force='mesh',process=True)
        before=len(mesh.faces)
        budget=28000 if 'rubber' in source.name else 36000 if 'so101' in rel.parts else 10000
        # Preserve the complete PP cover, holes, slot geometry, and small parts.
        if '__pp' in source.name:budget=before
        if before>budget:
            mesh=mesh.simplify_quadric_decimation(face_count=budget,aggression=5)
            mesh.export(target)
        else:shutil.copy2(source,target)
        entries.append(dict(file=str(rel),before=before,after=len(mesh.faces)))
    source=DESCRIPTION/'mujoco/room_assets/gx80/gx80_thread_actual.obj'
    target=OUTPUT/'thread.obj'
    mesh=trimesh.load(source,force='mesh');before=len(mesh.faces)
    mesh=mesh.simplify_quadric_decimation(face_count=10000,aggression=5);mesh.export(target)
    entries.append(dict(file='thread.obj',before=before,after=len(mesh.faces)))
    shutil.copytree(DESCRIPTION/'urdf',OUTPUT/'urdf',dirs_exist_ok=True)
    result=dict(version=VERSION,signature=signature,source_triangles=sum(e['before'] for e in entries),runtime_triangles=sum(e['after'] for e in entries),meshes=entries)
    report_path.write_text(json.dumps(result,indent=2));print('Visual LOD:',result['source_triangles'],'->',result['runtime_triangles'],flush=True)
    return result

if __name__=='__main__':prepare()
