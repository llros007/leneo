# LeNeo

轮式移动升降双臂机器人。当前公开 **MuJoCo** 与 **NVIDIA Isaac Sim 6.2** 仿真，包含机器人模型、modern_kitchen 厨房场景、键盘控制和地面拾取橙子演示。

## 仿真演示

机器人降低升降台，在地面夹取橙子，抬起收臂，保持朝向直行靠近吧台，再升高、放下橙子并收臂。两套仿真共用动作流程和相机切换，分别使用各自引擎的渲染与物理。

以下为两套仿真完成放置后的画面，放置特写中保留了第二只机械臂。

| MuJoCo | Isaac Sim 6.2 |
| :---: | :---: |
| ![MuJoCo 演示完成画面](docs/images/mujoco_demo_final.png) | ![Isaac Sim 6.2 演示完成画面](docs/images/isaac_demo_final.png) |

完整演示动作流程约 59 秒。

当前演示使用运动学底盘、逆运动学机械臂及辅助夹持。橙子进入两指之间并通过几何检查后绑定夹爪，松开后恢复重力和接触求解。该演示用于验证控制流程与可视化，不代表纯摩擦抓取或实机负载验证。

## 目录

```text
hardware/
└── lenoe.step          # 空文件占位，尚不是可用 CAD 模型
software/
└── .gitkeep            # 空软件目录的 Git 占位文件
simulation/
├── common/             # 共享控制、导航、IK、任务与资产处理
├── mujoco/             # MuJoCo 场景与独立键盘窗口
├── isaac/              # Isaac Sim 6.2 场景、材质与控制
├── assets/             # 机器人、厨房、橙子及材质参考
└── benchmarks/         # 导航、抓取、相机和录制验证脚本
docs/
├── images/             # 两套仿真的完成画面
└── validation/         # 演示与直行验证报告
```

## 快速启动

```bash
git clone https://github.com/llros007/leneo.git
cd leneo
conda create -n le312 python=3.12 -y
conda activate le312
python -m pip install -r simulation/requirements-mujoco.txt

# MuJoCo：地面拾取橙子并放到吧台
./simulation/mujoco/run.sh --demo-pick-orange

# Isaac：单独安装 Isaac Sim 6.2，并指定安装目录
export ISAAC_SIM_ROOT=/path/to/isaac-sim-6.2.0
./simulation/isaac/run.sh --demo-pick-orange
```

已包含预处理厨房和橙子，无需原始 GLB 或 leisaac 安装。首次启动自动生成机器人轻量网格，Isaac 还会转换 USD 并编译着色器。MuJoCo 使用当前环境的 Python，可设置 `LENEO_PYTHON`；Isaac 使用自身安装目录的 `python.sh`。本次 Isaac 验证版本为 **6.2.0-rc.14**。

普通键盘驾驶、鼠标视角、录制和验证命令见 [仿真使用说明](simulation/README.md)。

## 硬件与软件进度

**硬件设计文件、装配说明与 BOM 清单即将上传。** 本次 `hardware/lenoe.step` 仅为空文件占位，未上传完整 CAD。软件目录暂时保留占位，后续补充实机控制与部署说明。本次发布内容以仿真为主。

## 许可与来源

保留仓库原有 [Apache-2.0 许可证](LICENSE)。SO101、橙子、厨房及材质的来源说明见 [资产说明](simulation/ASSETS.md) 与资源目录内保留的许可证；第三方资源的来源与权利信息独立保留。
