# 来源与许可

SO101 原始 URDF、STL 和惯量来自 TheRobotStudio/SO-ARM100：
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101

使用的是本机与 SO101 CAD 同批下载的版本（2026-09-24），不是把网页当前版本直接替换进旧装配。源 URDF 原样保存在 `sources/so101_matching_cad.urdf`，SHA256 记录在 `build_manifest.json`。原许可为 Apache License 2.0，全文保存在 `sources/SO101_LICENSE.txt`。

修改内容：左右臂 link/joint 加前缀；保留运动学与连杆惯量；打印件显示色改浅蓝；effort/velocity 设为待标定起始值；加入阻尼/摩擦；固定工具坐标系移除原有零惯量占位；生成简化凸碰撞网格；接入 LeNeo 底盘和升降结构。

LeNeo CAD 几何来自用户提供的本地项目，原所有权及许可不在本次工作中变更；本包不替用户授予其 CAD 的对外再分发许可。新增配置与脚本用于该本地项目。

柏林客厅扫描：**Living Room - Berlin, Germany**，作者 **Azad Balabanian**，许可 **CC BY 4.0**。

- 原作：https://sketchfab.com/3d-models/living-room-berlin-germany-e97718b87da2415fbb0b0353d80e5115
- 许可：https://creativecommons.org/licenses/by/4.0/
- 本地来源：用户提供的 `C:/downloads/living_room_-_berlin_germany.glb`。
- 修改：从 GLB 提取 UV 网格与纹理；转换为米制 Z-up OBJ；将名义地面下移 90 mm；纹理由 4096 降至 2048 像素；调整实时显示材质；从扫描采样生成保守家具碰撞盒。未修改原 GLB。

机器人表面微纹理由本项目脚本程序生成；前轮胎纹和辐条外形沿用当前用户 CAD 的参考几何，不能视为轮毂接口的实物测绘。

## SO101 腕部相机

腕部相机支架网格与安装参数来自 MuJoCo Menagerie 的 `robotstudio_so101` 模型，通过参考项目 `robotics-antigravity-plugin/yolo_grasp/so101_yolo.py` 使用的 `mujoco_playground` 依赖定位。本地缓存版本标识为 `370c43965323`。模型源自 The Robot Studio，Menagerie SO101 描述由 I2RT Robotics 整理，采用 Apache-2.0；许可证保留于 `sources/SO101_MENAGERIE_LICENSE.txt`。

修改：将相机复制到左右夹爪固定分支，转换为 URDF 相机体与光学坐标系，保留支架、PCB、镜头和两块简化碰撞体；MuJoCo/Isaac 分别创建原生相机。参考模型的相机位置、朝向、61° 视场和 320×240 图像尺寸用于仿真，不代表实机标定值；相机支架总成的 12 g 质量及包络惯量为估计。
