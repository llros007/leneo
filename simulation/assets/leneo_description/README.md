# LeNeo 机器人仿真资源

URDF：`urdf/leneo.urdf`；ROS URDF：`urdf/leneo_ros.urdf`；机器人 MJCF：`mujoco/leneo_robot_only.xml`。请保留网格相对路径。`config/home_positions.json` 为双臂折叠初始姿态。

该目录提供原始机器人资源；完整厨房、键盘控制与演示请使用上层 `mujoco/run.sh` 或 `isaac/run.sh`，详见 [仿真说明](../../README.md)。独立机器人 MJCF 为零重力外观查看场景；完整运行器会加入重力、地板、厨房及控制。

材质纹理及 GX80 部件在 `mujoco/room_assets/`。该目录名沿用原相对路径，不包含客厅扫描。质量、惯量、摩擦与驱动参数仍为工程估计，未做实机标定。SO101 来源与许可证见 `LICENSES.md` 和 `sources/`。
