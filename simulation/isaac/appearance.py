"""Rebind the imported robot to Blender-derived MDL materials."""
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
from common.appearance import materials, MAPPING
from common.control import ROOT


def apply(stage, robot):
    refs=materials();mats={}
    for key,ref in refs.items():
        mat=UsdShade.Material.Define(stage,f'/World/RobotMaterials/{key}')
        shader=UsdShade.Shader.Define(stage,f'{mat.GetPath()}/Shader')
        shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
        shader.SetSourceAsset(Sdf.AssetPath(str(ROOT/'isaac/materials/LeNeoSurface.mdl')),'mdl')
        shader.SetSourceAssetSubIdentifier('LeNeoSurface','mdl')
        shader.CreateInput('base_color',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*ref['Base Color'][:3]))
        for name,value in [('roughness',ref['Roughness']),('metallic',ref['Metallic']),('ior',ref['IOR']),
                           ('transmission',ref.get('Transmission Weight',0)),('subsurface',ref.get('Subsurface Weight',0))]:
            shader.CreateInput(name,Sdf.ValueTypeNames.Float).Set(float(value))
        shader.CreateInput('texture_kind',Sdf.ValueTypeNames.Int).Set(1 if key=='arm_blue' else 2 if key=='rubber' else 0)
        for terminal in ['surface','volume','displacement']:
            mat.CreateOutput(f'mdl:{terminal}',Sdf.ValueTypeNames.Token).ConnectToSource(shader.ConnectableAPI(),'out')
        # Valid fallback for USD viewers that cannot execute MDL.
        preview=UsdShade.Shader.Define(stage,f'{mat.GetPath()}/Preview')
        preview.CreateIdAttr('UsdPreviewSurface')
        preview.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*ref['Base Color'][:3]))
        preview.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(float(ref['Roughness']))
        preview.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(float(ref['Metallic']))
        preview.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(1.0)
        mat.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(),'surface')
        mats[key]=mat
    counts={k:0 for k in refs}
    # Bind at instance roots; strongerThanDescendants overrides the imported
    # URDF material on the instanced child mesh without breaking instancing.
    for p in Usd.PrimRange(robot):
        if p.IsInstance():
            name=p.GetName()
            key=''
            # The imported binding lives on the mesh *inside* the instance.
            # Read it before authoring an instance-root override. Collision
            # instances have no visual binding and must not be recoloured.
            for child in Usd.PrimRange(p,Usd.TraverseInstanceProxies()):
                if child.IsA(UsdGeom.Mesh):
                    bound=UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()[0]
                    if bound:
                        key=bound.GetPrim().GetName()
                        break
            if not key and '__' in name:key=name.split('__')[-1]
            # Named actuator geometry is authoritative even in older exports.
            if 'sts3215' in name and '_hull' not in name:key='servo_black'
            if key in mats:
                UsdShade.MaterialBindingAPI.Apply(p).Bind(mats[key],UsdShade.Tokens.strongerThanDescendants)
                counts[key]+=1
            if name=='tower_link__pp': UsdGeom.Imageable(p).MakeVisible()
    print('Blender material bindings:',counts,flush=True)
