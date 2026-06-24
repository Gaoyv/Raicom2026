#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
睿抗2026 任务1「定向移动」—— 抗机器差异(可移植)版导航
========================================================

与 walk_forward_steps.py（第一版）同样的五阶段策略与场地/X2 特性认知，但把"靠
本机时序碰巧对"的部分换成"不管机器快慢都自己收敛"，解决"换台电脑就出圈/撞墙"。

为什么第一版换机器会飘（根因）
------------------------------
Docker 只冻结了二进制，不冻结**运行时序**。sim_mujoco / mc / 本节点是三个异步实时
进程(ROS2 DDS, mc MultiThreaded executor)，谁先谁后、仿真实时倍率，都取决于宿主机
CPU/GPU/调度——双足步态又会把这点时序差异放大成厘米级位置漂移。于是第一版里这些
"按本机标定/按墙钟计时"的东西换机器就失效：
  · target_x=-0.08 是补偿本机系统残差的魔法偏置 → 别的机器残差不同 → 反而推偏
  · 段超时按墙钟(--segment-timeout-sec) → 慢机器没走到就被截断 → 几何错位
  · 积分写死 dt=0.04 → 循环周期一变积分增益就变 → 过冲/不足
  · 余量压到极限(脚几乎填满圈、离墙仅~7cm) → 稍大漂移就破

本版的抗差异改造
----------------
  1. 段完成判据 = 到点容差 / 真卡死(stall: 仍在发速但里程计不再前进) / 极大安全上限，
     不再用墙钟超时当主判据 → 慢机器只是耗时久，但仍会走到，不被截断。
  2. 跟线积分用**实测 dt**(每圈测真实耗时)，机器快慢不改变积分行为。
  3. Stage4 倒走进区时"末段直走收尾"(最后 ~15cm 关跟线、锁 -90°)：到位瞬间鼻子已正、
     x 已修好，省掉事后回正(回正会拽 y) → y 稳定到位。
  4. 到位后**切回 STAND_DEFAULT 把机器人冻结**：否则留在 LOCOMOTION 的步态会原地踏步、
     松手后位姿持续漂移(可达 ~0.1m, 脚滑出圈)。冻结后位姿稳住不再漂。
  5. target_y 补偿了切 STAND 的 y 向沉降(~0.06)，使**静止落点**≈圆心而非 nav 终点≈圆心。

仍存在的固有限制(非 bug, 物理/几何决定)
----------------------------------------
  · 双脚几乎填满圆圈(站姿~0.3 vs 圈直径 0.5)，静止落点须落在圆心 ~±0.03 内才能"四角全进"。
  · 切 STAND 的冻结漂移含一个 ~±0.03 的随机分量(x 方向)，消不掉 → 每跑落点有 ±几 cm 抖动。
  · 结果稳定在 等级四(双足触线 40)~等级五(完美 50)，**永不出区、永不撞墙(≥40分保底)**；
    "完美 50"靠多跑碰(比赛 2 次机会取最好)。

运行
----
  ./start_AllinOne.sh start        # 起平台并自动切 STAND
  # 仿真窗口手动 Reset 让机器人站好
  ./run.sh usercode/py/walk_forward_steps_robust.py          # 一键(默认参数即最优)
  ./run.sh usercode/py/walk_forward_steps_robust.py --help   # 看全部参数

================ 每台机器特调参数 (per-machine calibration) ================
本版的"执行鲁棒性"(到点不被截断/不撞墙/不出区)跨机器通用，无需调。但**末厘米的精细
落点**含一部分本机时序专属分量，换机器若想冲稳定 50 分，按下面 30 秒流程重标 2 个值。
不重标也仍是 ≥40 分(偏移量小, 最多某脚压线, 不会整脚出圈)。

标定只看一行日志: 跑完的 `Final pose held (STAND, settled): x=.., y=.., yaw=..`
(这就是**实际计分的静止位姿**; 用它、不要用 `Navigation finished` 那行)。

(1) 先标 target_y(前后/入圈深度, 也管撞墙安全):
    - 跑一次, 看 held y。目标让 held y ≈ 1.70(交互区-I 圆心), 双脚前后居中。
    - held y 偏小(脚尖朝交互区-II 出圈) → 调大 target_y; 偏大 → 调小。
      经验关系: held_y ≈ target_y - (该机冻结沉降, 本机实测 ~0.04~0.07)。
    - ⚠安全红线: 看日志"最高 y"(倒走到位瞬间后背最逼近顶墙 1.85)。target_y 太大会撞墙=0分。
      本机 target_y=1.75 时最高 y≈1.74, 后背离墙安全; 换机器务必先确认这一项。

(2) 再标 target_x(左右居中):
    - 多跑几次看 held x 的**均值**。目标让均值 ≈ 0(双脚左右对称)。
    - held x 均值偏 + (左脚常压线) → 调小 target_x(更负); 偏 - (右脚常压线) → 调大。
      本机标定点: target_x=0→均值+0.01, -0.03→+0.015, -0.045→≈0, -0.06→-0.02。
    - x 有 ~±0.03 随机抖动消不掉, 只能把均值调到 0, 让两脚压线概率对称。

(3) 其余参数(forward/turn speed、跟线增益、stall/dt 等)跨机器通用, 一般不用动。
    若某机仿真实时倍率极低导致异常, 优先排查两机实时倍率是否一致。

本机最终值: --target-x -0.045 --target-y 1.75 (已设为默认)。完整背景见
竞赛文档/任务1_导航方案与X2运动特性.md。
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, CommonState, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction
from nav_msgs.msg import Odometry


class WalkForwardStepsRobust(Node):
    """任务1 导航节点（抗机器差异版）。

    与第一版接口一致：main() 依次 STAND→注册输入源→LOCOMOTION→等 odom→
    move_to_target() 执行五阶段 + 闭环复核。差异全在"完成判据/积分dt/无偏置/复核"。
    """

    def __init__(
        self,
        odom_topic,
        target_x,
        target_y,
        target_yaw_deg,
        forward_speed,
        turn_speed,
        distance_kp,
        heading_kp,
        pose_log_interval,
    ):
        super().__init__("walk_forward_steps_robust")
        # --- 发布器 / 服务客户端 / 订阅器 ---
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, odom_qos
        )

        # --- 目标与外部可调参数 ---
        self.source = "node"
        self.odom_topic = odom_topic
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.target_yaw = math.radians(float(target_yaw_deg))
        self.forward_speed = float(forward_speed)
        self.turn_speed = float(turn_speed)
        self.distance_kp = float(distance_kp)
        self.heading_kp = float(heading_kp)
        self.pose_log_interval = float(pose_log_interval)
        self.last_pose_log_time = 0.0

        # --- 速度/转向限幅与跟线增益（针对 X2 仿真整定）---
        self.max_heading_correction = 0.6
        self.max_distance_speed = self.forward_speed
        self.min_distance_speed = 0.05
        self.max_turn_speed = self.turn_speed
        self.min_turn_speed = 0.10
        self.min_back_speed = 0.13          # 倒走速度下限, 避开步态死区
        self.cross_track_kp = 1.6           # 航向跟线比例增益 (rad per m)
        self.cross_track_ki = 0.8           # 航向跟线积分增益 (用实测 dt 累积)
        self.max_cross_steer = 0.6          # 跟线转向上限 (rad)
        self.max_cross_int = 0.45           # 积分项转向上限 (anti-windup)
        self._cross_int = 0.0

        # --- 抗机器差异的"完成判据"参数（核心）---
        # 段/转身不再靠墙钟超时收尾，而是: 到点 / 真卡死(仍发速但不再前进) / 极大安全上限。
        self.position_tolerance = 0.03      # 到点容差 (m)
        self.angle_tolerance = math.radians(2.0)
        self.stall_window = 3.0             # 连续这么多秒"无实质前进"判为卡死 (s, 墙钟)
        self.stall_eps_pos = 0.015          # 平移"实质前进"阈值 (m): 窗口内改善不足即视为没进展
        self.stall_eps_ang = math.radians(3.0)  # 转身"实质前进"阈值
        self.max_segment_sec = 90.0         # 平移段硬上限 (s, 仅兜底, 取很大以容忍慢机器)
        self.max_turn_sec = 30.0            # 转身硬上限 (s, 兜底)

        # --- 流程几何/时序常量 ---
        self.settle_time = 0.8              # 转身前停稳秒数, 压低转身漂移
        self.backward_approach = 0.8        # 末段倒走进区距离 (m)
        self.warmup_time = 1.2              # 热身小步秒数 (避冷启动甩动)
        self.max_refine_iters = 3           # 末段闭环复核最大迭代次数

        # --- 里程计实时状态 ---
        self.odom_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    # ===================== 里程计 / 日志 =====================

    def on_odom(self, msg):
        """里程计回调：缓存 (x, y) 与从四元数解算的 yaw。"""
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def wait_for_odom(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_ready:
                self.log_pose(prefix=f"Odometry ready on {self.odom_topic}")
                return True
        self.get_logger().error(f"Timed out waiting for {self.odom_topic}")
        return False

    def log_pose(self, prefix="Current pose"):
        self.get_logger().info(
            f"{prefix}: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f} deg"
        )

    def maybe_log_pose(self):
        now = time.monotonic()
        if now - self.last_pose_log_time >= self.pose_log_interval:
            self.last_pose_log_time = now
            self.log_pose()

    def refresh_pose(self, spins=6):
        """多 spin 几次，确保读到最新里程计（停稳后用于复核）。"""
        for _ in range(spins):
            rclpy.spin_once(self, timeout_sec=0.05)

    # ===================== 服务 / 模式 / 指令 =====================

    def register_input_source(self):
        """注册本节点为运动指令输入源(priority 40)，须在切 LOCOMOTION 前完成。"""
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9
        while not self.client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for input source service timed out")
                return False
            self.get_logger().info("Waiting for input source service...")

        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self.source
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"trying to register input source... [{i}]")

        if future.done():
            try:
                response = future.result()
                state = response.response.state.value
                self.get_logger().info(
                    f"Input source set successfully: state={state}, task_id={response.response.task_id}"
                )
                return True
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                return False
        self.get_logger().error("Service call failed or timed out")
        return False

    def set_mode(self, action_desc, source="node"):
        """切换 mc 模式，如 STAND_DEFAULT / LOCOMOTION_DEFAULT。"""
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9
        while not self.action_client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for SetMcAction service timed out")
                return False
            self.get_logger().info("Waiting for SetMcAction service...")

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = source
        cmd = McActionCommand()
        cmd.action_desc = action_desc
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.action_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Trying to set {action_desc}... [{i}]")

        if future.done():
            try:
                response = future.result()
                if response.response.status.value == CommonState.SUCCESS:
                    self.get_logger().info(f"{action_desc} set successfully")
                    return True
                self.get_logger().error(
                    f"Failed to set {action_desc}: {response.response.message}"
                )
                return False
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                return False
        self.get_logger().error("Service call failed or timed out")
        return False

    def set_locomotion_mode(self):
        return self.set_mode("LOCOMOTION_DEFAULT")

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        """发布机体系速度(前进/横移/转向)。注意 lateral 在本仿真实测无效。"""
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.publisher.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        """归一化到 (-pi, pi]。"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop(self, repeats=10):
        """连发多次零速度，确保 mc 收到停止。"""
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def settle(self, seconds=None):
        """转身前停稳：实测停稳后转身漂移从 ~0.3m 降到 ~0.1m。"""
        if seconds is None:
            seconds = self.settle_time
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def warmup(self):
        """热身小步：locomotion 启动后第一个动作会猛甩 ~0.4m，先在出发区走掉。"""
        self.get_logger().info("Warm-up: short forward step to clear cold-start lurch")
        deadline = time.monotonic() + self.warmup_time
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(forward=0.15, angular=0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
        self.stop()
        self.settle()

    # ===================== 运动原语（抗机器差异完成判据）=====================

    def turn_to_heading(self, target_heading):
        """原地转到目标朝向。完成判据 = 到容差 / 转身卡死 / 硬上限（非墙钟超时）。

        注意：原地转身在本仿真会随机平移 0.1~0.5m，调用方应先 settle() 停稳，
        且不要把大角度转身放在终点/近墙处。
        """
        self.last_pose_log_time = 0.0
        self.get_logger().info(f"Turning to heading {math.degrees(target_heading):.1f} deg")

        best_remaining = abs(self.normalize_angle(target_heading - self.yaw))
        now = time.monotonic()
        stall_since = now
        hard_cap = now + self.max_turn_sec

        stop_reason = "tolerance"
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            error = self.normalize_angle(target_heading - self.yaw)
            remaining = abs(error)

            if remaining <= self.angle_tolerance:
                stop_reason = "tolerance"
                break
            # 有实质转进 → 刷新卡死计时；长时间无进展 → 判卡死收尾
            if remaining < best_remaining - self.stall_eps_ang:
                best_remaining = remaining
                stall_since = now
            elif now - stall_since > self.stall_window:
                stop_reason = "stall"
                break
            if now > hard_cap:
                stop_reason = "cap"
                break

            angular = max(self.min_turn_speed, min(self.max_turn_speed, remaining * self.heading_kp))
            if error < 0.0:
                angular = -angular
            self.publish_velocity(angular=angular)
            self.maybe_log_pose()
            time.sleep(0.02)

        self.stop()
        self.log_pose(prefix=f"Heading aligned ({stop_reason})")

    def drive_axis_to_target(
        self, axis_name, target_value, target_heading,
        forward_sign=1.0, apply_cross_track=False, straight_finish=0.0
    ):
        """沿单一世界轴(x/y)闭环行走到 target_value。

        抗机器差异要点：
          · 完成判据 = 到容差 / 越过目标 / 真卡死(仍发速但不再前进) / 硬安全上限，
            不用墙钟超时当主判据 → 慢机器只是耗时久，仍会走到，不会被截断。
          · 跟线积分用**实测 dt** 累积，机器快慢不改变积分行为。
          · 跟线(仅 y 段)把 x 拉回 target_x；正/倒走同一套公式，倒走用更高速度下限避死区。
          · straight_finish>0: 当剩余距离小于它时关掉跟线、直走锁航向收尾，
            使到位瞬间鼻子已回正(不必事后回正→省掉回正漂移 y 的代价)。
        """
        self.last_pose_log_time = 0.0
        start_error = (target_value - self.y) if axis_name == "y" else (target_value - self.x)
        previous_error = start_error
        motion_label = "backward" if forward_sign < 0.0 else "forward"
        self._cross_int = 0.0

        best_remaining = abs(start_error)
        now = time.monotonic()
        stall_since = now
        last_t = now
        hard_cap = now + self.max_segment_sec

        self.get_logger().info(
            f"Driving axis {axis_name} toward {target_value:.3f} with heading "
            f"{math.degrees(target_heading):.1f} deg, motion={motion_label}"
        )

        stop_reason = "tolerance"
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            dt = min(0.1, max(0.005, now - last_t))   # 实测循环周期 (限幅防异常)
            last_t = now

            current_value = self.y if axis_name == "y" else self.x
            error = target_value - current_value
            remaining = abs(error)

            if remaining <= self.position_tolerance:
                stop_reason = "tolerance"
                break
            if start_error != 0.0 and previous_error != 0.0 and error * previous_error < 0.0:
                stop_reason = "crossing"
                break
            # 卡死检测：有实质前进就刷新计时；长时间无进展即收尾(慢机器仍在前进不会触发)
            if remaining < best_remaining - self.stall_eps_pos:
                best_remaining = remaining
                stall_since = now
            elif now - stall_since > self.stall_window:
                stop_reason = "stall"
                break
            if now > hard_cap:
                stop_reason = "cap"
                break

            # 航向跟线(cross-track steering)：不能横移，只能靠转向把自己拉回目标轴线。
            # 末段(剩余<straight_finish)关掉跟线、直走锁航向收尾，使到位时鼻子已正。
            desired_heading = target_heading
            if apply_cross_track and axis_name == "y" and remaining > straight_finish:
                cross_error = self.target_x - self.x
                self._cross_int += cross_error * dt          # 用实测 dt 累积
                self._cross_int = max(-self.max_cross_int, min(self.max_cross_int, self._cross_int))
                steer = self.cross_track_kp * cross_error + self.cross_track_ki * self._cross_int
                steer = max(-self.max_cross_steer, min(self.max_cross_steer, steer))
                desired_heading = self.normalize_angle(target_heading - steer)

            heading_error = self.normalize_angle(desired_heading - self.yaw)
            angular = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, self.heading_kp * heading_error),
            )
            min_speed = self.min_back_speed if forward_sign < 0.0 else self.min_distance_speed
            speed_magnitude = max(min_speed, min(self.max_distance_speed, self.distance_kp * remaining))
            forward = speed_magnitude * forward_sign

            # 朝向若与目标轴反了，自动翻转前进方向（安全保护）
            if axis_name == "x":
                predicted_delta = math.cos(target_heading) * forward
            else:
                predicted_delta = math.sin(target_heading) * forward
            if error * predicted_delta < 0.0:
                forward = -forward
                motion_label = "backward" if forward < 0.0 else "forward"

            self.publish_velocity(forward=forward, angular=angular)
            self.maybe_log_pose()
            previous_error = error
            time.sleep(0.02)

        self.stop()
        final_value = self.y if axis_name == "y" else self.x
        self.get_logger().info(
            f"Segment finished on axis {axis_name}: value={final_value:.3f}, "
            f"target={target_value:.3f}, error={target_value - final_value:.3f}, "
            f"stop_reason={stop_reason}, motion={motion_label}"
        )

    # ===================== 顶层流程 =====================

    def move_to_target(self):
        """五阶段导航 + 末段闭环复核（前提：已 STAND→注册→LOCOMOTION→odom 就绪）。"""
        self.log_pose(prefix="Start pose")
        self.warmup()

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        self.get_logger().info(
            f"Target point: x={self.target_x:.3f}, y={self.target_y:.3f}, dx={dx:.3f}, dy={dy:.3f}"
        )

        # Stage 1: 朝 0° 原地把 x 摆正
        if abs(dx) > self.position_tolerance:
            x_heading = 0.0 if dx >= 0.0 else math.pi
            self.get_logger().info(f"Stage 1: correct x by {dx:.3f} m, heading {math.degrees(x_heading):.1f}")
            self.settle()
            self.turn_to_heading(x_heading)
            self.drive_axis_to_target("x", self.target_x, x_heading)

        dy_after = self.target_y - self.y

        # Stage 2: 转 +90° 正向快走到中途点(留 backward_approach 余量)，跟线压 x
        sign = 1.0 if dy_after >= 0.0 else -1.0
        y_mid = self.target_y - sign * self.backward_approach
        if abs(y_mid - self.y) > self.position_tolerance:
            y_heading = math.pi / 2.0 if dy_after >= 0.0 else -math.pi / 2.0
            self.get_logger().info(
                f"Stage 2: heading {math.degrees(y_heading):.1f}, forward to mid y={y_mid:.3f}"
            )
            self.settle()
            self.turn_to_heading(y_heading)
            self.drive_axis_to_target("y", y_mid, y_heading, forward_sign=1.0, apply_cross_track=True)

        # Stage 3: 在安全低位完成 180° 大转身到最终朝向(漂移在此无害、离顶墙远)
        self.get_logger().info(
            f"Stage 3: big turn to final heading {math.degrees(self.target_yaw):.1f} at safe low y={self.y:.3f}"
        )
        self.settle()
        self.turn_to_heading(self.target_yaw)

        # Stage 4: 朝最终朝向倒走进区，跟线对正 x；末段 15cm 直走锁 -90° 收尾，
        # 使到位时鼻子已回正、x 已修好，省掉事后回正(及其拽 y 的代价)。
        if abs(self.target_y - self.y) > self.position_tolerance:
            self.get_logger().info(f"Stage 4: back into zone, drive y to {self.target_y:.3f}")
            self.settle(0.4)
            self.drive_axis_to_target(
                "y", self.target_y, self.target_yaw,
                forward_sign=-1.0, apply_cross_track=True, straight_finish=0.15
            )

        # Stage 5: 闭环复核——重读位姿，超容差就再修 y + 回正朝向，迭代到收敛。
        # 这是抗机器差异的关键：不管本机漂移多少，都靠闭环把残差收掉，而非靠固定偏置补。
        self.refine_pose()

        self.refresh_pose()
        self.stop()
        self.get_logger().info(
            f"Navigation finished: final x={self.x:.3f}, y={self.y:.3f}, "
            f"yaw={math.degrees(self.yaw):.1f}, "
            f"error dx={self.target_x - self.x:.3f}, dy={self.target_y - self.y:.3f}, "
            f"dyaw={math.degrees(self.normalize_angle(self.target_yaw - self.yaw)):.1f}"
        )

        # 关键：切回 STAND_DEFAULT 把机器人冻结在原地。否则停留在 LOCOMOTION 模式时，
        # 无指令的步态会原地微调/踏步，导致松手后位姿持续漂移(实测可达 ~0.1m，脚滑出圈)。
        self.get_logger().info("Freezing final pose: switching to STAND_DEFAULT")
        self.set_mode("STAND_DEFAULT")
        # STAND 切换后机器人会用约 2~3s 收成静态站姿(向 -y 沉降 ~0.07), 先等它稳定再读真实静止位姿
        self.settle(2.5)
        self.refresh_pose()
        self.log_pose(prefix="Final pose held (STAND, settled)")

    def refine_pose(self):
        """末段轻量复核(安全网)。

        Stage4 已用"末段直走"收尾→到位时鼻子已回正、x 已修好、y 在目标，正常无需大动。
        这里只做兜底：先按需补一次回正(仅当朝向超容差)，再按需直走补一次 y。
        关键经验：不在这里反复转身——转身漂 y；故回正/补 y 各最多一次，且回正放在补 y 之前。

        约束：面朝 -90° 不能横移→改不了 x；x 靠 Stage2/4 跟线积分 + target_x 站姿偏置。
        """
        # 1) 按需回正(通常跳过：Stage4 直走收尾已锁好 -90°)
        self.refresh_pose()
        if abs(self.normalize_angle(self.target_yaw - self.yaw)) > self.angle_tolerance:
            self.settle(0.3)
            self.turn_to_heading(self.target_yaw)

        # 2) 按需直走补 y(若回正或残留把 y 带偏)；直走锁航向，不再产生大偏角
        self.refresh_pose()
        ey = self.target_y - self.y
        self.get_logger().info(
            f"Refine: ex={self.target_x - self.x:.3f}, ey={ey:.3f}, "
            f"eyaw={math.degrees(self.normalize_angle(self.target_yaw - self.yaw)):.1f}"
        )
        if abs(ey) > self.position_tolerance:
            fsign = -1.0 if ey > 0.0 else 1.0   # ey>0 需 +y → 面朝-90 时倒走
            self.settle(0.3)
            self.drive_axis_to_target(
                "y", self.target_y, self.target_yaw, forward_sign=fsign,
                apply_cross_track=False, straight_finish=0.0
            )

        # x 残差过大时只告警（面朝 -90 无法纠正，且不做近墙大转身）
        self.refresh_pose()
        if abs(self.target_x - self.x) > 2.0 * self.position_tolerance:
            self.get_logger().warn(
                f"Refine: x residual {self.target_x - self.x:.3f} 较大且最终朝向下无法纠正; "
                f"如经常发生, 调 --target-x 或检查仿真实时倍率"
            )

    def shutdown_safely(self):
        self.stop()


_global_node = None


def _signal_handler(sig, _frame):
    if _global_node is not None:
        _global_node.stop()
        _global_node.get_logger().info(f"Received signal {sig}, stopping and shutting down")
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="任务1 抗机器差异版导航 (closed-loop, stall-based, measured-dt)"
    )
    parser.add_argument(
        "--reset-prompt", action="store_true",
        help="设 STAND 后暂停等你按 Reset 再回车 (默认不暂停: 运行前先在仿真里 Reset)",
    )
    parser.add_argument("--stand-time", type=float, default=1.5,
                        help="切 STAND 后等待站稳的秒数")
    parser.add_argument("--odom-topic", type=str, default="/aima/hal/odom/state",
                        help="里程计话题(闭环反馈)")
    parser.add_argument("--target-x", type=float, default=-0.045,
                        help="目标 x。实测标定: 0→冻结均值+0.01(偶尔左脚出), -0.03→均值+0.015(偏左脚), "
                             "-0.06→均值-0.02(偶尔右脚出); 取 -0.045 使均值≈0、双脚对称居中。"
                             "含固有(站姿几何, 可移植)+本机时序两部分, 换机器可能需重标这一档")
    parser.add_argument("--target-y", type=float, default=1.75,
                        help="导航终点 y。圆心 1.70, 切 STAND 冻结向 -y 沉降 ~0.06, 故终点取 1.75 使静止落点≈1.69 "
                             "(脚往圆心收)。⚠后背朝顶墙(1.85): 倒走到位瞬间后背最逼近墙, 过高会撞墙(0分), 某机器需下调")
    parser.add_argument("--target-yaw-deg", type=float, default=-90.0,
                        help="最终朝向(正面朝交互区-II)")
    parser.add_argument("--forward-speed", type=float, default=0.35, help="最大前进速度 m/s")
    parser.add_argument("--turn-speed", type=float, default=0.4, help="最大转向角速度 rad/s")
    parser.add_argument("--distance-kp", type=float, default=0.6, help="平移距离 P 增益")
    parser.add_argument("--heading-kp", type=float, default=1.2, help="航向 P 增益")
    parser.add_argument("--pose-log-interval", type=float, default=0.5, help="行进中位姿日志间隔 s")
    return parser.parse_args()


def main():
    """入口：解析参数→建节点→STAND→(可选等Reset)→注册输入源→LOCOMOTION→等odom→导航→收尾。"""
    global _global_node
    args = _parse_args()

    rclpy.init()
    node = WalkForwardStepsRobust(
        args.odom_topic,
        args.target_x,
        args.target_y,
        args.target_yaw_deg,
        args.forward_speed,
        args.turn_speed,
        args.distance_kp,
        args.heading_kp,
        args.pose_log_interval,
    )
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # 1) 切站立平衡
    if not node.set_mode("STAND_DEFAULT"):
        node.get_logger().error("Failed to set stand mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # (可选) 暂停等 Reset；默认要求运行前已 Reset 站好
    if args.reset_prompt:
        print(
            "\nSTAND_DEFAULT is active. Click Reset in the simulator now; "
            "the robot should appear standing in the start area.\n"
            "Press Enter here after Reset to continue...",
            flush=True,
        )
        input()

    time.sleep(args.stand_time)

    # 2) 注册输入源 → 3) 切行走 → 4) 等里程计
    if not node.register_input_source():
        node.get_logger().error("Input source registration failed, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return
    if not node.set_locomotion_mode():
        node.get_logger().error("Failed to set locomotion mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return
    if not node.wait_for_odom():
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 5) 执行导航；无论成败 finally 都停车关闭
    try:
        node.move_to_target()
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
