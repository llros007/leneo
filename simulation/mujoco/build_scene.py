from pathlib import Path
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.control import ROOT, DESCRIPTION
from common.appearance import apply_mujoco


def build(quality="fast"):
    from common.prepare_runtime import prepare, OUTPUT
    from task_scene import SPAWN, add
    if quality=="fast": prepare()
    source = DESCRIPTION / 'mujoco/leneo_robot_only.xml'
    root = ET.parse(source).getroot()
    root.set('model', 'LeNeo modern kitchen')
    for element in root.find('asset'):
        if element.get('file'):
            element.set('file', str((source.parent / element.get('file')).resolve()))
    kitchen_file = ROOT / 'assets/modern_kitchen/kitchen.xml'
    if not kitchen_file.exists(): raise FileNotFoundError('Run ../prepare.sh first')
    kitchen = ET.parse(kitchen_file).getroot()
    for element in kitchen.find('asset'):
        if element.get('file'): element.set('file', str((kitchen_file.parent / element.get('file')).resolve()))
        root.find('asset').append(element)
    world = root.find('worldbody')
    world.extend(kitchen.find('worldbody'))
    ET.SubElement(world, 'geom', name='ground', type='plane', size='10 10 0.1', contype='2', conaffinity='1', group='3', rgba='0.2 0.2 0.2 1', friction='0.9 0.005 0.0001')
    root.find('option').set('gravity', '0 0 -9.81')
    # RobotOnly was a zero-gravity inspection asset. Its free base is retained.
    root.find('keyframe/key').set('qpos', root.find('keyframe/key').get('qpos').replace('0.0 0.0 0.003 ', ' '.join(map(str,SPAWN))+' ', 1))
    key = root.find('keyframe/key')
    qpos = key.get('qpos').split(); qpos[3:7] = ['0.7071067812','0','0','0.7071067812']; key.set('qpos', ' '.join(qpos))
    apply_mujoco(root)
    add(root)
    asset=root.find('asset')
    used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    for mesh in list(asset.findall('mesh')):
        if mesh.get('name') not in used:
            asset.remove(mesh);continue
        if quality=='fast':
            source_file=Path(mesh.get('file'))
            if source_file.is_relative_to(DESCRIPTION/'meshes'):
                mesh.set('file',str(OUTPUT/source_file.relative_to(DESCRIPTION)))
            elif mesh.get('name')=='gx80_thread_actual':mesh.set('file',str(OUTPUT/'thread.obj'))
    root.find('visual/quality').set('offsamples','4')
    target = ROOT / ('mujoco/scene.xml' if quality=='fast' else 'mujoco/scene_full.xml')
    ET.indent(root); ET.ElementTree(root).write(target, encoding='unicode')
    return target

if __name__ == '__main__': print(build())
