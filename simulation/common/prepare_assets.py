"""Convert the Y-up kitchen into shared, metre/Z-up assets. Run with le312."""
from pathlib import Path
import argparse
import os
import json
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt

import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.kitchen_edits import remove_dining_table, EDITION


def prepare(source, force=False):
    output = ROOT / 'assets/modern_kitchen'
    manifest = output / 'manifest.json'
    if manifest.exists() and not force:
        cached=json.loads(manifest.read_text())
        if cached.get('edition')==EDITION and cached.get('material_version')==2:return cached
    output.mkdir(parents=True, exist_ok=True)
    scene = trimesh.load(source, force='scene')
    rotation = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    # The supplied GLB has a raised floor at Y=0.180 m. Use its actual floor.
    floor_heights = []
    for node in scene.graph.nodes_geometry:
        t, name = scene.graph[node]
        if 'walls_floor' in node.lower():
            floor_heights.extend(trimesh.transform_points(scene.geometry[name].vertices, rotation @ t)[:, 2])
    if not floor_heights:
        raise ValueError('Kitchen floor mesh not found; cannot infer floor height')
    floor_z = float(np.median(floor_heights))
    rotation[2, 3] = -floor_z
    stage = Usd.Stage.CreateNew(str(output / 'kitchen.usdc'))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    parent = UsdGeom.Xform.Define(stage, '/Kitchen')
    stage.SetDefaultPrim(parent.GetPrim())
    asset = ET.Element('asset')
    world = ET.Element('worldbody')
    meshes = []
    infos = []
    texture_cache = {}
    for i, node in enumerate(scene.graph.nodes_geometry):
        transform, key = scene.graph[node]
        mesh = scene.geometry[key].copy()
        mesh.apply_transform(rotation @ transform)
        mesh=remove_dining_table(mesh,node)
        if mesh is None:continue
        meshes.append(mesh)
        name = f'kitchen_{i:03d}'
        # OBJ preserves UVs. Material/texture is defined explicitly for MuJoCo.
        visual_mesh = mesh.copy()
        centered = visual_mesh.vertices - visual_mesh.vertices.mean(axis=0)
        _, singular, vh = np.linalg.svd(centered, full_matrices=False)
        if len(visual_mesh.vertices) < 4 or singular[-1] < 1e-5:
            # MuJoCo requires a volumetric hull even for visual-only triangles.
            count = len(visual_mesh.vertices)
            vertices = np.vstack([visual_mesh.vertices, visual_mesh.vertices + vh[-1] * 0.0002])
            faces = np.vstack([visual_mesh.faces, visual_mesh.faces[:, ::-1] + count])
            uv = np.vstack([visual_mesh.visual.uv, visual_mesh.visual.uv]) if hasattr(visual_mesh.visual, 'uv') else None
            visual_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
            if uv is not None: visual_mesh.visual = trimesh.visual.TextureVisuals(uv=uv)
        obj = trimesh.exchange.obj.export_obj(visual_mesh, include_texture=False, include_normals=False)
        if hasattr(visual_mesh.visual, 'uv'):
            # Write UV indices explicitly, avoiding OBJ/MTL texture re-encoding.
            lines = obj.splitlines()
            lines = [('f ' + ' '.join(f'{v}/{v}' for v in line[2:].split())) if line.startswith('f ') else line for line in lines]
            uv_text = '\n'.join('vt %.8f %.8f' % tuple(v) for v in visual_mesh.visual.uv)
            obj = uv_text + '\n' + '\n'.join(lines)
        (output / f'{name}.obj').write_text(obj)
        material = mesh.visual.material
        factor = getattr(material, 'baseColorFactor', None)
        color = np.asarray(factor if factor is not None else [255, 255, 255, 255], dtype=float) / 255
        # Keep architectural glass lightly transparent, without hiding black plastic.
        rgba = color.copy()
        rgba[3] = max(0.25, rgba[3])
        tex = getattr(material, 'baseColorTexture', None)
        texfile = None
        if tex is not None:
            cachekey = id(tex)
            if cachekey not in texture_cache:
                texfile = f'texture_{len(texture_cache):03d}.png'
                picture = tex.convert('RGB'); picture.thumbnail((2048, 2048)); picture.save(output / texfile)
                texture_cache[cachekey] = texfile
            texfile = texture_cache[cachekey]
        matname = f'{name}_mat'
        ET.SubElement(asset, 'mesh', name=name, file=f'{name}.obj', maxhullvert='32')
        attrs = dict(name=matname, rgba=' '.join(map(str, rgba)), shininess='0.2')
        if texfile:
            texname = f'{name}_tex'
            ET.SubElement(asset, 'texture', name=texname, type='2d', file=texfile)
            attrs['texture'] = texname
        ET.SubElement(asset, 'material', **attrs)
        ET.SubElement(world, 'geom', name=name, type='mesh', mesh=name, material=matname,
                      contype='0', conaffinity='0', group='1')
        usdmesh = UsdGeom.Mesh.Define(stage, f'/Kitchen/{name}')
        usdmesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(mesh.vertices.astype(np.float32)))
        usdmesh.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
        usdmesh.CreateFaceVertexIndicesAttr(mesh.faces.flatten().tolist())
        usdmesh.CreateSubdivisionSchemeAttr('none')
        usdmesh.CreateDoubleSidedAttr(True)
        usdmat = UsdShade.Material.Define(stage, f'/Kitchen/Materials/{matname}')
        shader = UsdShade.Shader.Define(stage, f'{usdmat.GetPath()}/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color[:3]))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(getattr(material, 'roughnessFactor', None) or 0.5))
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(getattr(material, 'metallicFactor', None) or 0))
        if texfile and hasattr(mesh.visual, 'uv'):
            uv = np.array(mesh.visual.uv, dtype=np.float32)
            UsdGeom.PrimvarsAPI(usdmesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv))
            reader = UsdShade.Shader.Define(stage, f'{usdmat.GetPath()}/UV')
            reader.CreateIdAttr('UsdPrimvarReader_float2')
            reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
            texture = UsdShade.Shader.Define(stage, f'{usdmat.GetPath()}/Texture')
            texture.CreateIdAttr('UsdUVTexture')
            texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texfile))
            texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            texture.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
            texture.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
            texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
            texture.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*color))
            shader.GetInput('diffuseColor').ConnectToSource(texture.ConnectableAPI(), 'rgb')
        usdmat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(usdmesh.GetPrim()).Bind(usdmat)
        infos.append(dict(name=name, source=node, faces=len(mesh.faces), bounds=mesh.bounds.tolist()))
    stage.GetRootLayer().Save()
    # Voxel surface collision, shared by both engines. Avoid a single convex hull
    # which would incorrectly fill the entire room. Separate floor stays exact.
    pitch = 0.08
    full = trimesh.util.concatenate(meshes)
    occupied = set()
    for mesh in meshes:
        points = mesh.voxelized(pitch=pitch).points
        indices = np.rint(points / pitch).astype(int)
        occupied.update(map(tuple, indices[points[:, 2] > pitch * 0.6]))
    boxes = []
    while occupied:
        x, y, z = min(occupied)
        x1 = x
        while (x1 + 1, y, z) in occupied: x1 += 1
        y1 = y
        while all((xx, y1 + 1, z) in occupied for xx in range(x, x1 + 1)): y1 += 1
        z1 = z
        while all((xx, yy, z1 + 1) in occupied for xx in range(x, x1 + 1) for yy in range(y, y1 + 1)): z1 += 1
        for xx in range(x, x1 + 1):
            for yy in range(y, y1 + 1):
                for zz in range(z, z1 + 1): occupied.remove((xx, yy, zz))
        center = (np.array([x+x1, y+y1, z+z1]) * pitch / 2).tolist()
        half = ((np.array([x1-x, y1-y, z1-z]) + 1) * pitch / 2).tolist()
        boxes.append(dict(center=center, half=half))
    for i, b in enumerate(boxes):
        ET.SubElement(world, 'geom', name=f'kitchen_collision_{i}', type='box', pos=' '.join(map(str,b['center'])),
                      size=' '.join(map(str,b['half'])), contype='2', conaffinity='1', group='3', rgba='0.4 0.5 0.7 0.2', friction='0.8 0.005 0.0001')
    wrapper = ET.Element('mujoco');wrapper.append(asset);wrapper.append(world)
    ET.indent(wrapper); ET.ElementTree(wrapper).write(output / 'kitchen.xml', encoding='unicode')
    result = dict(material_version=2,edition=EDITION,source=str(Path(source).resolve()), source_floor_height=floor_z, y_up_to_z_up=rotation.tolist(),
                  bounds=full.bounds.tolist(), collision_pitch=pitch, collision_boxes=boxes, meshes=infos,
                  spawn=[-1.4, -0.6, 0.008], spawn_yaw=1.5707963267948966)
    manifest.write_text(json.dumps(result, indent=2))
    print(f'Kitchen ready: {len(infos)} meshes, {len(full.faces)} triangles, {len(boxes)} collision boxes', flush=True)
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',default=os.environ.get('LENEO_KITCHEN_SOURCE', str(ROOT/'assets/modern_kitchen.glb')));p.add_argument('--force',action='store_true')
    args=p.parse_args();prepare(args.source,args.force)
