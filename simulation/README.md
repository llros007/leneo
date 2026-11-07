# LeNeo 仿真使用说明

MuJoCo 与 Isaac Sim 6.2 共用机器人、modern_kitchen 场景、导航、逆运动学及拾橙子流程。下列命令均从仓库根目录执行。

## 环境

推荐 Python 3.12 的 Conda 环境：

```bash
conda create -n le312 python=3.12 -y
conda activate le312
python -m pip install -r simulation/requirements-mujoco.txt
```

MuJoCo 验证版本为 3.17.0。启动脚本默认调用当前环境的 `python`，也可 `export LENEO_PYTHON=/path/to/python`。Isaac 独立安装 6.2，设置 `export ISAAC_SIM_ROOT=/path/to/isaac-sim-6.2.0`；它使用安装自带 Python，并隔离当前 Conda 的 Isaac 相关环境变量。本次验证为 6.2.0-rc.14，不混用 leisaac 环境中的 Isaac 5.1。

预处理厨房、橙子和机器人源网格已随仓库提供。首次运行生成 `assets/runtime_robot/` 轻量网格；Isaac 首次转换机器人 USD、编译 RTX 着色器。生成缓存和日志不进入 Git。

## 启动与演示

```bash
# 普通键盘模式
./simulation/mujoco/run.sh
./simulation/isaac/run.sh

# 地面拾橙子 → 直行靠近吧台 → 升高 → 放置
./simulation/mujoco/run.sh --demo-pick-orange
./simulation/isaac/run.sh --demo-pick-orange

# Isaac 默认亮度倍率为 1.6，可继续调高
./simulation/isaac/run.sh --demo-pick-orange --light-scale 2.0

# MuJoCo 无窗口运行与结果检查
./simulation/mujoco/run.sh --headless --demo-pick-orange \
  --screenshot simulation/outputs/final.png \
  --report simulation/outputs/report.json
```

演示约 59 秒。相机在升降台降低到位后切地面抓取近景，抬起后恢复默认视角，放置时切可见双臂的吧台近景，过渡约 0.65 秒。抓起后收臂并保持朝向直行约 36 cm。窗口模式结束后保留终态，Esc 退出；无窗口模式默认结束后退出。

底盘为 100 Hz 运动学导航，碰撞检查采用底盘包络与厨房碰撞盒。夹持采用显式辅助：MuJoCo weld；Isaac kinematic 刚体随夹爪局部位姿运动。只有两指相对且表面间隙均小于 4 mm 才允许绑定；释放后橙子以 500 Hz 求解重力及接触。Isaac 同步检查 PhysX、USD 显示和夹爪位姿。

该演示未验证手臂碰撞、纯摩擦抓取、整机负载或真机动力学。实验性的 `--drive-mode physics` 保留轮驱动力学，参数尚未实机标定，Isaac 下仍存在底盘无指令漂移；常规演示使用默认 `navigation`。

## 键盘和鼠标

点击窗口取得焦点，松开方向键或窗口失焦会停车。MuJoCo 使用独立 GLFW 窗口。

| 操作 | 按键 |
|---|---|
| 前进 / 后退 | W/S 或 ↑/↓ |
| 左转 / 右转 | A/D 或 ←/→，可与前后键组合 |
| 刹车 | Space |
| 升降 | Q/E 或 R/F |
| 左右臂切换 | Tab |
| 选择臂关节 | 1—6 |
| 所选关节运动 | J/K |
| 折叠姿态 | H |
| 跟随视角 | V |
| 退出 | Esc |

左键拖动旋转，右键平移，滚轮缩放；Isaac 也支持中键平移、B 重置视角。手动调整暂停跟随，保留当前观察角度。演示模式在下一个相机节点重新接管。

默认线速度 0.15 m/s、角速度 0.55 rad/s、升降速度 0.05 m/s。JSON 控制：

```bash
./simulation/mujoco/run.sh --command-file /tmp/leneo-command.json
```

```json
{"linear": 0.1, "angular": 0.2, "joints": {"lift_joint": 0.1}}
```

单位为米、秒、弧度。速度文件超过 0.5 秒未更新会停车，关节目标保持；建议原子替换文件。

## 画质与资产

默认 `--quality fast` 保留完整部件并生成 LOD，机器人唯一运行网格约 72 万三角面，包含 PP 塔套、黑色电机及平滑法线。MuJoCo 使用 4× MSAA，Isaac 使用 DLAA 和 RTX 实时渲染。材质参考参数位于 `assets/blender_reference/materials_and_validation.json`，未附大型 Blender 源文件。两引擎对材质的表达不同，MuJoCo 为 OpenGL 近似。

```bash
./simulation/mujoco/run.sh --quality full
./simulation/isaac/run.sh --quality full --renderer PathTracing
```

厨房为米制 Z-up，保留 UV 与纹理。椅子旁餐桌及其碰撞体已移除，保留椅子与吧台。厨房目前为静态场景。

如需从自己的原始厨房 GLB 重新生成：

```bash
./simulation/prepare.sh --source /path/to/modern_kitchen.glb --force
```

也可通过 `LENEO_KITCHEN_SOURCE` 指定源文件；重新导入橙子时可用 `LENEO_ORANGE_SOURCE` 指向 leisaac 的 Orange001 目录。正常启动无需这些原始资源。

## 验证与录像

```bash
python simulation/benchmarks/verify_navigation.py
# 检查随仓库提交的两引擎直行验证报告
python simulation/benchmarks/verify_straight_route.py
python simulation/benchmarks/verify_demo_camera.py
python simulation/benchmarks/verify_pick_orange.py
python simulation/benchmarks/verify_isaac_pick_orange.py --windowed

# MuJoCo 完整录像（需要 ffmpeg）
python simulation/benchmarks/record_mujoco_demo.py

# Isaac 原生 RTX 无窗口录像
./simulation/isaac/run.sh --headless --demo-pick-orange --seconds 62 \
  --video simulation/outputs/isaac_demo.mp4 --video-fps 30 \
  --report simulation/outputs/isaac_demo.json
```

原始录像约 62 秒、30 FPS；仓库 `docs/videos/` 为最新 2 倍速、60 FPS 演示，含结束停留及大字引擎标识。对应 JSON 保存任务结果、抓取检查和阶段记录。README 图片直接提取两段视频的最后一帧。
