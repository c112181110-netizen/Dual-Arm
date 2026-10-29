#!/usr/bin/env python3
"""Reproducibly adapt the pinned upstream model; never edit the upstream checkout."""
from pathlib import Path
from copy import deepcopy
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
UP = ROOT / 'third_party/phantomx_pincher'
DEST = ROOT / 'src/dual_arm_description'
SOURCE = UP / 'phantomx_pincher_description'


def generate():
    original = ET.parse(SOURCE / 'urdf/phantomx_pincher.urdf').getroot()
    base = 'phantomx_pincher_arm_base_link'
    links = {base}
    joints = []
    # Keep only the arm and original fixed tool mounting chain. Remove fingers,
    # task board, upstream controllers and Gazebo extensions.
    remaining = list(original.findall('joint'))
    while remaining:
        added = False
        for j in list(remaining):
            if j.find('parent').get('link') in links:
                remaining.remove(j)
                if 'finger1' in j.get('name') or 'finger2' in j.get('name') or j.get('type') == 'continuous':
                    continue
                links.add(j.find('child').get('link'))
                joints.append(j)
                added = True
        if not added:
            break

    def rename(s):
        return s.replace(base, 'arm1_base').replace('phantomx_pincher_', 'arm1_')

    robot = ET.Element('robot', name='dual_arm_pincher')
    for item in list(original.findall('link')) + joints:
        if item.tag == 'link' and item.get('name') not in links:
            continue
        item = deepcopy(item)
        for e in item.iter():
            for k, v in list(e.attrib.items()):
                e.set(k, rename(v) if k != 'filename' else v.replace(
                    'package://phantomx_pincher_description/', 'package://dual_arm_description/'))
        robot.append(item)
    # These tool dimensions are demonstration placeholders; real launch requires
    # calibration and accepts replacement model/config files.
    tool = ET.SubElement(robot, 'link', name='arm1_suction_tcp')
    for kind in ('visual', 'collision'):
        e = ET.SubElement(tool, kind)
        ET.SubElement(e, 'origin', xyz='0 0 -0.0125')
        ET.SubElement(ET.SubElement(e, 'geometry'), 'cylinder', radius='0.008', length='0.025')
    j = ET.SubElement(robot, 'joint', name='arm1_suction_fixed', type='fixed')
    ET.SubElement(j, 'parent', link='arm1_end_effector')
    ET.SubElement(j, 'child', link='arm1_suction_tcp')
    ET.SubElement(j, 'origin', xyz='0 0 0.025', rpy='0 0 0')
    (DEST / 'urdf').mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(DEST / 'urdf/arm1.urdf', encoding='unicode', xml_declaration=True)

    semantic = ET.Element('robot', name='dual_arm_pincher')
    group = ET.SubElement(semantic, 'group', name='arm1')
    ET.SubElement(group, 'chain', base_link='arm1_base', tip_link='arm1_suction_tcp')
    old_sem = ET.parse(UP / 'phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf').getroot()
    for e in old_sem.findall('disable_collisions'):
        if e.get('link1') in links and e.get('link2') in links:
            ET.SubElement(semantic, 'disable_collisions', link1=rename(e.get('link1')),
                          link2=rename(e.get('link2')), reason=e.get('reason'))
    for mount in ['arm1_end_effector', 'arm1_gripper_finger_base_link', 'arm1_gripper_servo_link']:
        ET.SubElement(semantic, 'disable_collisions', link1=mount, link2='arm1_suction_tcp', reason='Adjacent')
    (DEST / 'srdf').mkdir(exist_ok=True)
    ET.indent(semantic)
    ET.ElementTree(semantic).write(DEST / 'srdf/arm1.srdf', encoding='unicode', xml_declaration=True)
    shutil.copytree(SOURCE / 'phantomx_pincher/meshes', DEST / 'meshes', dirs_exist_ok=True)
    shutil.copyfile(UP / 'LICENSE', DEST / 'LICENSE.upstream')


if __name__ == '__main__':
    generate()
