# 仿真资产来源

- `leneo_description/`：LeNeo 的 URDF、MJCF、机器人网格及参数，来自项目的机器人仿真包。SO101 双臂源自 [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101)，保留 `sources/SO101_LICENSE.txt`、原始 URDF 和修改说明。机器人仿真网格不包含完整制造 CAD，本次硬件 STEP 为占位文件。
- `modern_kitchen/`：项目提供的 `modern_kitchen.glb` 的预处理结果。进行了 Y-up 到 Z-up、地板归零、餐桌裁除、碰撞盒生成及纹理 repeat 修正。包含可直接运行的 OBJ、纹理、MJCF 和 USD；原始 GLB 未重复提交。原资产作者和独立许可信息尚未记录在本地来源中，仓库 Apache-2.0 不代表为第三方场景新增授权。
- `orange/`：来自 [leisaac](https://github.com/LightwheelAI/leisaac) 的 `assets/scenes/kitchen_with_orange/objects/Orange001`。保留原 USD、BC/ORM 纹理和 leisaac 许可证，并导出居中、米制且保留 UV 的 OBJ 供 MuJoCo 使用。详见资源目录 `SOURCE.md`。
- `blender_reference/`：本项目机器人预览与 Blender 材质参数导出；大型 `.blend` 源文件未提交。程序化表面微纹理由 `common/appearance.py` 生成。

`leneo_description/LICENSES.md` 保留原始仿真包的来源记录；其中提到的柏林客厅扫描不在本次厨房场景中，也未提交该扫描网格。生成的运行 LOD、Isaac 机器人转换缓存可从源网格重建，因此不入库。
