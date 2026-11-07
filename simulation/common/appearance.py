"""Blender reference material values and renderer-specific approximations."""
from pathlib import Path
import json
import hashlib
import os
import numpy as np
from PIL import Image
from .control import ROOT

REFERENCE = ROOT/'assets/blender_reference/materials_and_validation.json'
MAPPING = {
    'pp':'Slightly translucent milky white PP sheet',
    'arm_blue':'Light blue SO101 printed polymer',
    'aluminum':'Satin anodized aluminium',
    'steel':'Machined stainless steel',
    'metal_satin':'Satin cast aluminium',
    'rubber':'Black rubber',
    'servo_black':'Servo black polymer',
    'black':'Synced matte black camera connector',
    'lens_black':'Black camera lens barrel',
    'pcb_green':'PCB solder mask',
    'white_plastic':'Warm white printed polymer',
    'milky_white':'Milky white translucent slot tape',
}


def materials():
    result = {}
    for m in json.loads(REFERENCE.read_text())['materials']:
        for node in m['nodes']:
            if node['type']=='ShaderNodeBsdfPrincipled':
                result[m['name']] = node
    return {key:result[value] for key,value in MAPPING.items()}


def prepare_mujoco_textures():
    output = ROOT/'assets/robot_materials'; output.mkdir(parents=True,exist_ok=True)
    refs = materials()
    signature=hashlib.sha256(REFERENCE.read_bytes()+b'linear-srgb-texture-v1').hexdigest()
    stamp=output/'texture_signature.txt'
    if stamp.exists() and stamp.read_text()==signature and all((output/f'{key}.png').is_file() for key in refs):return output,refs
    rng=np.random.default_rng(76);yy,xx=np.mgrid[0:512,0:512]
    for key, ref in refs.items():
        target=output/f'{key}.png'
        color=np.array(ref['Base Color'][:3])
        # The Blender values are linear scene colours; MuJoCo textures use sRGB.
        srgb=np.where(color<=0.0031308,12.92*color,1.055*color**(1/2.4)-0.055)
        noise=rng.normal(0,1,(512,512))
        if key=='arm_blue': pattern=1+0.008*np.sin(yy*2*np.pi/3)+noise*0.002
        elif key=='rubber': pattern=1+noise*0.035
        elif key in ['aluminum','steel','metal_satin']: pattern=1+noise*0.007+np.sin(xx*0.8)*0.003
        else: pattern=1+noise*0.002
        rgb=np.clip(srgb[None,None,:]*pattern[:,:,None]*255,0,255).astype(np.uint8)
        temporary=target.with_name(target.stem+f'.{os.getpid()}.tmp.png')
        Image.fromarray(rgb).save(temporary);temporary.replace(target)
    (output/'blender_mapping.json').write_text(json.dumps({k:dict(source=MAPPING[k],parameters=v) for k,v in refs.items()},indent=2))
    stamp.write_text(signature)
    return output, refs


def apply_mujoco(root):
    output,refs=prepare_mujoco_textures()
    asset=root.find('asset')
    # The inspection model split the cover into five panels; the top OBJ has
    # zero signed volume. Use the original watertight CAD visual, preserving
    # the actual 3 mm cover, lid and longitudinal slots as one mesh.
    import xml.etree.ElementTree as ET
    from .control import DESCRIPTION
    ET.SubElement(asset,'mesh',name='blender_pp_cover',file=str(DESCRIPTION/'meshes/cad/tower_link__pp.stl'),maxhullvert='64')
    tower = root.find(".//body[@name='tower_link']")
    for geom in list(tower.findall('geom')):
        if geom.get('name','').startswith('tower_panel_'): tower.remove(geom)
    ET.SubElement(tower,'geom',name='pp_tower_cover',type='mesh',mesh='blender_pp_cover',material='mat_pp',
                  rgba='1 1 1 1',contype='0',conaffinity='0',density='0',group='1')
    for texture in asset.findall('texture'):
        key=texture.get('name','').removeprefix('tex_')
        if key in refs:texture.set('file',str(output/f'{key}.png'))
    for material in asset.findall('material'):
        key=material.get('name','').removeprefix('mat_')
        if key not in refs:continue
        ref=refs[key];rough=float(ref['Roughness']);metal=float(ref['Metallic'])
        material.set('roughness',str(rough));material.set('metallic',str(metal))
        material.set('specular',str(0.25+0.5*metal));material.set('shininess',str(max(.02,(1-rough)**3)))
        material.set('rgba','1 1 1 1')
    for geom in root.iter('geom'):
        if geom.get('material','').removeprefix('mat_') in refs:
            # PP remains a mostly opaque milky sheet. Alpha is only a MuJoCo
            # approximation; Isaac uses transmission and subsurface scattering.
            geom.set('rgba','1 1 1 1')
        if geom.get('name','').startswith('tower_panel_'): geom.set('group','1')
