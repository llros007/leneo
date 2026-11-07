"""Isaac Sim 6.2 URDF importer (new Python pipeline, not the removed 5.x commands)."""
from pathlib import Path
import sys
import argparse
import json
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.control import ROOT, DESCRIPTION
parser=argparse.ArgumentParser();parser.add_argument('--quality',choices=['fast','full'],default='fast');args=parser.parse_args()
output_dir=ROOT/('assets/isaac_robot_fast' if args.quality=='fast' else 'assets/isaac_robot')
description=ROOT/'assets/runtime_robot' if args.quality=='fast' else DESCRIPTION
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'active_gpu': 0, 'multi_gpu': False})
try:
    import omni.kit.app
    manager = omni.kit.app.get_app().get_extension_manager()
    for extension in ['omni.scene.optimizer.core', 'isaacsim.robot.schema', 'isaacsim.asset.importer.urdf']:
        manager.set_extension_enabled_immediate(extension, True)
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    config = URDFImporterConfig(
        urdf_path=str(description/'urdf/leneo.urdf'), usd_path=str(output_dir),
        fix_base=False, merge_fixed_joints=True, allow_self_collision=False,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness={'.*':35.0, 'lift_joint':6500.0, '.*wheel_joint':0.0, '.*caster.*':0.0},
        override_joint_damping={'.*':1.0, 'lift_joint':240.0, '.*wheel_joint':1.2, '.*caster.*':0.02})
    output = URDFImporter(config).import_urdf()
    if not output: raise RuntimeError('URDF conversion returned no asset')
    from smooth_normals import apply as smooth_normals
    smooth_normals(output)
    (output_dir/'robot_path.txt').write_text(str(Path(output).resolve()))
    if args.quality=='fast': (output_dir/'source_signature.txt').write_text(json.loads((description/'manifest.json').read_text())['signature'])
    print('LENEO_ROBOT_USD='+str(output), flush=True)
finally:
    app.close()
