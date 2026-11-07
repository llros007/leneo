"""MuJoCo-only scene layout and the orange's independent contact geometry."""
import xml.etree.ElementTree as ET
import numpy as np
from common.control import ROOT
from common.prepare_orange import prepare

from common.orange_layout import SPAWN,ORANGE,TABLE_TOP,PLACE


def add(root):
    info=prepare();asset=root.find('asset');world=root.find('worldbody')
    ET.SubElement(asset,'texture',name='orange_tex',type='2d',file=str(ROOT/'assets/orange/albedo.png'))
    ET.SubElement(asset,'material',name='orange_mat',texture='orange_tex',specular='.22',shininess='.12')
    ET.SubElement(asset,'mesh',name='orange_mesh',file=str(ROOT/'assets/orange/orange.obj'))
    body=ET.SubElement(world,'body',name='orange',pos=' '.join(map(str,ORANGE)))
    ET.SubElement(body,'freejoint',name='orange_free')
    ET.SubElement(body,'inertial',pos='0 0 0',mass='.09',diaginertia='0.000022 0.000023 0.000027')
    ET.SubElement(body,'geom',name='orange_visual',type='mesh',mesh='orange_mesh',material='orange_mat',contype='0',conaffinity='0',group='1',mass='0')
    ET.SubElement(body,'geom',name='orange_contact',type='ellipsoid',size=' '.join(str(v/2) for v in info['dimensions']),mass='.09',contype='4',conaffinity='8',group='3',condim='6',friction='.8 .01 .001')
    # Kitchen proxies are deliberately coarse. The fruit uses the actual slab height.
    ET.SubElement(world,'geom',name='orange_table_support',type='box',pos='-.137662653 1.274041213 .886960607',size='.897585054 .446533254 .021079949',contype='8',conaffinity='4',group='3')
    ground=world.find("geom[@name='ground']");ground.set('contype','8');ground.set('conaffinity','5')
    anchor=ET.SubElement(world,'body',name='grasp_anchor',mocap='true',pos='0 0 3')
    ET.SubElement(anchor,'geom',type='sphere',size='.004',contype='0',conaffinity='0',group='4',rgba='1 0 0 0')
    equality=root.find('equality')
    if equality is None:equality=ET.SubElement(root,'equality')
    ET.SubElement(equality,'weld',name='orange_grasp',body1='grasp_anchor',body2='orange',active='false',relpose='0 0 0 1 0 0 0',solref='.004 1')
    tool=root.find(".//body[@name='right_tool0']")
    ET.SubElement(tool,'site',name='orange_grip',pos='-.0276954 0 .003',size='.004',group='4',rgba='1 .5 0 1')
    key=root.find('keyframe/key');key.set('qpos',key.get('qpos')+' '+' '.join(map(str,ORANGE))+' 1 0 0 0')
