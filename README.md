# RAICOM 2026 灵犀X2 仿真 —— 任务1 定向移动

本分支为个人提交的**任务1（定向移动）**解决方案：从【出发区】自主导航到【交互区-I】，
双脚完整入圈、正面朝向【交互区-II】、不撞围挡、用时 < 3 分钟。

---

## 目录结构

```
usercode/py/
  walk_forward_steps.py          # 任务1 主方案（五阶段闭环导航，最优参数已设为默认）
  walk_forward_steps_robust.py   # 抗机器差异(可移植)版，换电脑时优先用这版
  diag_velocity.py / diag_turn.py # 速度/转身实测诊断脚本（调参依据）
竞赛文档/
  任务1_导航方案与X2运动特性.md    # 方案详解：场地坐标、X2运动特性(坑)、五阶段、参数
  2026CAIM_AGI_Competition_Rules.md # 比赛规则
start_AllinOne.sh                # 一键启动/管理仿真+运动控制容器
run.sh                           # 在容器内运行工作区脚本的辅助脚本
py/                              # 平台自带示例(set_mode / get_mode / topic 等)
```

---

## 运行步骤

### 1. 启动仿真平台

```bash
./start_AllinOne.sh start     # 起 仿真(sim_mujoco) + 运动控制(mc)，并自动切 STAND
```

其它子命令：`stop` 停容器、`restart` 停后重开并重启（最干净的复位）、`clean` 删容器。
**无参数会报错**（必须显式指定子命令）。

### 2. 在仿真窗口手动点 Reset

机器人初始是断电跪坐在地，**点一次 Reset** 后才会在出发区站好（自动切的 STAND 会让它立即站起）。
不 Reset 直接跑会让机器人在地上挣扎一段时间，影响后续行程。

### 3. 运行任务1（最优参数已是默认值，一键即可）

```bash
./run.sh usercode/py/walk_forward_steps.py
```

抗机器差异版（换到别的电脑跑时建议用这个）：

```bash
./run.sh usercode/py/walk_forward_steps_robust.py
```

需要覆盖参数时再显式传，例如：

```bash
./run.sh usercode/py/walk_forward_steps.py --forward-speed 0.5 --target-y 1.6
```

完整参数见脚本 `--help` 或文件头部 brief。

---

## 方案简述

X2 仿真运动控制有几个关键"坑"：横移(lateral)完全失效、前进天然右偏+偏航、原地大转身平移漂移大、
倒走慢且有步态死区。据此设计的**五阶段闭环导航**核心思想是：
**把不稳定的 180° 大转身放在安全低位完成，最后一段朝最终朝向 -90° 倒走进圈，走完即终态，
不再有任何转身去破坏定位。**

```
热身小步 → Stage1 摆正x → Stage2 转+90°正向走到中途点(航向跟线压漂移)
→ Stage3 安全低位转180°到-90° → Stage4 倒走最后0.8m进圈 → Stage5 锁朝向-90°
```

横向纠偏靠**航向跟线(cross-track steering)**：因不能螃蟹步，用转向把机器人拉回目标轴线。

> 详见 [`竞赛文档/任务1_导航方案与X2运动特性.md`](竞赛文档/任务1_导航方案与X2运动特性.md)（含场地坐标、X2特性实测表、完整参数表）。

---

## 关于"换电脑落点漂移"

仿真二进制冻结在 Docker 镜像里，但运动控制是异步实时进程，不同电脑的实时因子(real-time factor)
不同，会被双足步态放大成厘米级漂移。因此换电脑时建议用 `walk_forward_steps_robust.py`，
并按其文件头 brief 的"每台机器特调参数"小节微调 `--target-x` / `--target-y`：

- 调 `--target-y` 使最终 held_y ≈ 1.70（留意 max-y 不要逼近顶墙 1.85）。
- 调 `--target-x` 使最终 held_x 均值 ≈ 0（X2 站姿宽 ~0.3m，几乎填满直径 0.5m 的圈，必须居中）。
- 校准只看日志里 `Final pose held (STAND, settled)` 这一行的落点。

---

## 注意

- 停稳后机器人有 ~0.08m 站定微调，每跑落点 ±几 cm 随机浮动，不保证每次完美居中。正式比赛建议多跑取最好成绩。
- 比赛要求：禁止复制粘贴代码到终端、禁用云算力、需双机位监考、全程匿名。
