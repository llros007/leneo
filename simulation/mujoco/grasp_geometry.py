"""Measure the orange against both actual visual finger surfaces (not a tool point)."""
import numpy as np
import trimesh


def finger_meshes(model):
    result=[]
    for body in ['right_gripper_link','right_moving_jaw_so101_v1_link']:
        bid=model.body(body).id;parts=[]
        for i in range(model.ngeom):
            k=int(model.geom_dataid[i])
            if model.geom_bodyid[i]!=bid or model.geom_group[i]!=1 or k<0:continue
            if 'so101' not in model.mesh(k).name:continue
            vertices=model.mesh_vert[model.mesh_vertadr[k]:model.mesh_vertadr[k]+model.mesh_vertnum[k]].copy()
            faces=model.mesh_face[model.mesh_faceadr[k]:model.mesh_faceadr[k]+model.mesh_facenum[k]].copy()
            parts.append((i,vertices,faces))
        if not parts:raise ValueError('Finger visual geometry missing: '+body)
        result.append(parts)
    return result


def measure(sim):
    d=sim.data;m=sim.model;center=d.qpos[sim.orange_q:sim.orange_q+3]
    radius=m.geom_size[m.geom('orange_contact').id]
    R=d.xmat[m.body('orange').id].reshape(3,3)
    points=[];gaps=[]
    for parts in sim.finger_surfaces:
        candidates=[]
        for i,v,f in parts:
            world=v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
            unit=(world-center)@R/radius
            mesh=trimesh.Trimesh(unit,f,process=False)
            closest,distance,_=trimesh.proximity.closest_point_naive(mesh,[[0,0,0]])
            candidates.append((float(distance[0]),closest[0]))
        distance,point=min(candidates,key=lambda x:x[0])
        radial=point*radius
        gap=float(np.linalg.norm(radial)*(1-1/max(distance,1e-9)))
        points.append((radial@R.T).tolist());gaps.append(gap)
    p=np.asarray(points);opposition=float(p[0]@p[1]/(np.linalg.norm(p[0])*np.linalg.norm(p[1])))
    return dict(jaw_surface_gaps_m=gaps,opposition_cosine=opposition,
                between_fingers=bool(max(abs(g) for g in gaps)<.004 and opposition<-.5))
