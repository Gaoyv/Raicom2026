#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
睿抗2026 AGI 智慧养老组 —— 任务1「定向移动」自主导航
================================================================

功能
----
驱动灵犀 X2 从【出发区】自主行走到【交互区-I】，最终双脚居中入圈、正面朝向
【交互区-II】，全程闭环(里程计反馈)、不触围挡。实测落点 x≈0、y≈1.69、yaw≈-93°，
双足完美入圈，用时 ~36s（任务1 效率满分档 <3min）。

设计为什么是现在这样（X2 仿真运动实测特性，详见 diag_velocity.py / diag_turn.py）
--------------------------------------------------------------------------------
  · lateral_velocity（横移/螃蟹步）失效 → 不能横移，横向纠偏只能靠"转向(航向跟线)"
  · 前进天然向右漂移 ~20% 并伴顺时针偏航 → 直行须实时锁航向 + 跟线 P+积分纠偏
  · 原地转身平移漂移大且随机(0.1~0.5m，方向不定) → 大转身只能放安全低位，不能放终点/近墙
  · locomotion 启动后第一个动作猛甩 ~0.4m → 先热身小步
  · 倒走慢(~0.12m/s) 且小速度命令落入步态死区会原地蹭卡死 → 倒走给速度下限、关跟线积分

场地坐标（odom 世界系，源自 sim_mujoco .../scene/room/room.xml）
----------------------------------------------------------------
  · 围挡 x∈[-2,2], y∈[-2,2]；顶墙(+y侧)内表面 y≈1.85（撞到判 0 分，最危险的边）
  · 出发区 (-1.5,-1.5)；交互区-I 圆心 (0,1.7) r=0.25；交互区-II 圆心 (0,1.0) r=0.25

导航算法：五阶段（核心思想：把不稳的大转身放安全低位，末段朝最终朝向倒走入区，
走完即终态、不再有转身破坏定位）
----------------------------------------------------------------------------
  热身小步(避冷启动甩动)
  Stage 1: 朝 0° 原地把 x 摆正
  Stage 2: 转 +90° 正向快走到中途点 y = target_y - backward_approach（航向跟线 P+积分压 x 漂移）
  Stage 3: 在安全低位 y≈0.9 转 180° 到最终朝向 -90°（大转身漂移在此无害、离顶墙远）
  Stage 4: 朝 -90° 倒走最后 backward_approach 米进圈（航向跟线 P，关积分；速度下限防卡死）
  Stage 5: 小角度回正锁朝向 -90°（正面朝交互区-II）
  （每次转身前 settle 停稳 ~0.8s，把转身漂移从 ~0.3m 压到 ~0.1m）

============================ 运行教程 ============================

前置：先起平台并让机器人在出发区站好
  1) ./start_AllinOne.sh          # 启动仿真(sim_mujoco)+运动控制(mc)，并自动切 STAND
  2) 在 MuJoCo 仿真窗口手动点 Reset # 机器人初始断电跪坐，Reset 后才会在出发区站立

一键运行（推荐，所有最优参数已是默认值）
  ./run.sh usercode/py/walk_forward_steps.py

指定参数运行（覆盖默认值，示例）
  # 提速版（更激进，注意稳定性）
  ./run.sh usercode/py/walk_forward_steps.py --forward-speed 0.5 --turn-speed 0.5
  # 换目标点（如调试到别的位置）
  ./run.sh usercode/py/walk_forward_steps.py --target-x 0.0 --target-y 1.6 --target-yaw-deg -90
  # 让脚本自己等你按 Reset（切 STAND 后暂停，Reset 完回车继续）
  ./run.sh usercode/py/walk_forward_steps.py --reset-prompt
  # 查看全部参数与默认值
  ./run.sh usercode/py/walk_forward_steps.py --help

直接在容器内运行（不经 run.sh 时）
  docker exec -it x2_deploy bash -ic \\
    'cd /home/agi/x2_deploy_workspace/usercode/py && python3 walk_forward_steps.py'

============================ 关键参数 ============================
  --target-x   默认 -0.08  交互区-I 圆心 x=0；-0.08 补偿末态系统性 +x 残差(~0.07)使落点≈0
  --target-y   默认 1.70   交互区-I 圆心 y；正对圆心使整只脚(含脚角)入圈，背对墙离墙 ~7cm
  --target-yaw-deg 默认 -90  最终朝向(正面朝交互区-II)
  --forward-speed  默认 0.35 最大前进速度 m/s
  --turn-speed     默认 0.4  最大转向角速度 rad/s
  --segment-timeout-sec   默认 25  每段平移的最长允许秒数(倒走慢，留足)
  --final-turn-timeout-sec 默认 10  转向段最长秒数
  --reset-prompt   默认关   开启则切 STAND 后暂停等你 Reset+回车
  （更多内部调参常量见 WalkForwardSteps.__init__：cross_track_kp/ki、backward_approach、
    settle_time、min_back_speed 等，均有行内注释）

注意
----
  · 停稳后机器人有 ~0.08m 站定微调，每跑落点 ±几 cm 随机浮动，不保证每次完美居中
    （-0.08 偏置让其平均落圆心）；正式赛多跑、取两次机会最好成绩。
  · 触围挡即判 0 分；本方案已把大转身放安全低位、target_y 留足墙余量规避。
  · 完整说明见 竞赛文档/任务1_导航方案与X2运动特性.md。
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


class WalkForwardSteps(Node):
    """任务1 导航的 ROS2 节点。

    职责：
      · 发布运动指令 → /aima/mc/locomotion/velocity (前进/横移/转向速度)
      · 订阅里程计  → odom_topic (闭环位置/朝向反馈)
      · 调用服务    → 注册输入源、切换 mc 模式(STAND/LOCOMOTION)

    用法：__init__ 传入目标与调参 → main() 依次 STAND→注册输入源→LOCOMOTION→
    等 odom → move_to_target() 执行五阶段导航。
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
        segment_timeout_sec,
        final_turn_timeout_sec,
    ):
        super().__init__("walk_forward_steps")
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

        # --- 目标与外部可调参数（来自命令行）---
        self.source = "node"               # 本节点作为运动指令输入源的名字
        self.odom_topic = odom_topic
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.target_yaw = math.radians(float(target_yaw_deg))  # 最终朝向(弧度)
        self.forward_speed = float(forward_speed)
        self.turn_speed = float(turn_speed)
        self.distance_kp = float(distance_kp)   # 平移距离 P 增益
        self.heading_kp = float(heading_kp)     # 航向 P 增益
        self.pose_log_interval = float(pose_log_interval)
        self.segment_timeout_sec = float(segment_timeout_sec)
        self.final_turn_timeout_sec = float(final_turn_timeout_sec)
        self.last_pose_log_time = 0.0

        # --- 内部调参常量（不走命令行，针对 X2 仿真特性整定，改动需重测）---
        self.max_heading_correction = 0.6  # 直行中航向修正角速度上限 (rad/s)
        self.max_distance_speed = self.forward_speed
        self.min_distance_speed = 0.05
        self.max_turn_speed = self.turn_speed
        self.min_turn_speed = 0.10
        self.min_back_speed = 0.13          # 倒走速度下限, 避开步态死区
        self.cross_track_kp = 1.6          # 航向跟线比例增益 (rad per m 偏差)
        self.cross_track_ki = 0.8          # 积分增益, 消除前进天然漂移造成的稳态偏差
        self.max_cross_steer = 0.6         # 跟线转向上限 (rad, ~34°), 需大于天然漂移所需
        self.max_cross_int = 0.45          # 积分项转向上限 (anti-windup)
        self._cross_int = 0.0
        self.settle_time = 0.8             # 转身前停稳秒数, 压低转身漂移
        self.backward_approach = 0.8       # 末段倒走进区的距离(m): 给 P 跟线足够距离把 x 收到 0、鼻子回正;
                                           # 同时大转身落在更低更安全的 y 位
        self.position_tolerance = 0.03     # 到点容差 (m)
        self.angle_tolerance = math.radians(2.0)  # 对准朝向容差

        # --- 里程计实时状态 (由 on_odom 回调更新) ---
        self.odom_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def on_odom(self, msg):
        """里程计回调：缓存当前 (x, y) 与从四元数解算的 yaw。"""
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

    def register_input_source(self):
        """向 mc 注册本节点为运动指令输入源(priority 40)。

        不注册则发布的速度指令不会被 mc 采纳。须在切 LOCOMOTION 前完成。
        """
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
        """切换 mc 运动模式，如 "STAND_DEFAULT"(站立平衡) / "LOCOMOTION_DEFAULT"(行走)。"""
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
                else:
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
        """切到行走模式(LOCOMOTION_DEFAULT)，之后发布的速度指令才会驱动行走。"""
        return self.set_mode("LOCOMOTION_DEFAULT")

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        """发布机体系速度指令(前进/横移/转向)。注意 lateral 在本仿真实测无效。"""
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
        """把角度归一化到 (-pi, pi]，用于求最短转向误差。"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_to_heading(self, target_heading, timeout_sec=None):
        """原地转到目标朝向(弧度)。P 控制角速度，到达容差或超时即停。

        注意：原地转身在本仿真会随机平移 0.1~0.5m，调用方应在转身前 settle() 停稳，
        且不要把大角度转身放在终点/近墙处。
        """
        self.last_pose_log_time = 0.0
        if timeout_sec is None:
            timeout_sec = self.final_turn_timeout_sec
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(
            f"Turning to heading {math.degrees(target_heading):.1f} deg"
        )

        stop_reason = "tolerance"
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            error = self.normalize_angle(target_heading - self.yaw)
            if abs(error) <= self.angle_tolerance:
                stop_reason = "tolerance"
                break
            if time.monotonic() >= deadline:
                stop_reason = "timeout"
                break

            angular = max(
                self.min_turn_speed,
                min(self.max_turn_speed, abs(error) * self.heading_kp),
            )
            if error < 0.0:
                angular = -angular

            self.publish_velocity(angular=angular)
            self.maybe_log_pose()
            time.sleep(0.02)

        self.stop()
        self.log_pose(prefix=f"Heading aligned ({stop_reason})")

    def drive_axis_to_target(
        self, axis_name, target_value, target_heading,
        forward_sign=1.0, apply_cross_track=False, use_integral=True
    ):
        """沿单一世界轴(x 或 y)闭环行走到 target_value，P 控速 + 锁航向。

        参数：
          axis_name        "x" 或 "y"：以该世界轴坐标为主控误差。
          target_value     该轴目标坐标。
          target_heading   行进时要保持的朝向(弧度)。
          forward_sign     +1 正走 / -1 倒走（倒走用更高速度下限避开步态死区）。
          apply_cross_track 是否对垂直轴做"航向跟线"纠偏（仅 y 段用，把 x 拉回 target_x）。
          use_integral     跟线是否带积分（正走开、倒走关，防积分绕死卡步）。

        终止条件：到达容差 / 越过目标(crossing) / 段超时。
        """
        self.last_pose_log_time = 0.0
        start_error = (
            target_value - self.y if axis_name == "y" else target_value - self.x
        )
        previous_error = start_error
        motion_label = "backward" if forward_sign < 0.0 else "forward"
        self._cross_int = 0.0

        self.get_logger().info(
            f"Driving axis {axis_name} toward {target_value:.3f} with heading {math.degrees(target_heading):.1f} deg, motion={motion_label}"
        )

        stop_reason = "tolerance"
        deadline = time.monotonic() + self.segment_timeout_sec
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            current_value = self.y if axis_name == "y" else self.x
            error = target_value - current_value

            if abs(error) <= self.position_tolerance:
                stop_reason = "tolerance"
                break

            if start_error != 0.0 and previous_error != 0.0 and error * previous_error < 0.0:
                stop_reason = "crossing"
                break

            if time.monotonic() >= deadline:
                stop_reason = "timeout"
                break

            # 航向跟线 (cross-track steering)：机器人不能横移，只能靠"转向"把自己
            # 拉回目标轴线。仅在主 y 段启用 (此时 x 已预先对齐)；x 段 (y 故意未对齐) 不启用。
            desired_heading = target_heading
            if apply_cross_track and axis_name == "y":
                cross_error = self.target_x - self.x  # 目标轴线为 x = target_x
                # 积分项: 抵消前进天然 +x 漂移的稳态误差 (纯 P 会留固定偏差)。
                # 倒走段关闭积分, 防止其绕死把鼻子拐偏+小速度触发步态死区导致卡死。
                steer_cmd = self.cross_track_kp * cross_error
                if use_integral:
                    self._cross_int += cross_error * 0.04
                    self._cross_int = max(
                        -self.max_cross_int, min(self.max_cross_int, self._cross_int)
                    )
                    steer_cmd += self.cross_track_ki * self._cross_int
                steer = max(-self.max_cross_steer, min(self.max_cross_steer, steer_cmd))
                # 面朝 +y 正向行进时, x 偏大(cross_error<0)需左转(增大 yaw)把机器人带回 -x
                desired_heading = self.normalize_angle(target_heading - steer)

            heading_error = self.normalize_angle(desired_heading - self.yaw)
            angular = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, self.heading_kp * heading_error),
            )
            # 倒走需要更高的速度下限, 否则小速度命令落入步态死区会原地蹭、不平移。
            min_speed = self.min_back_speed if forward_sign < 0.0 else self.min_distance_speed
            speed_magnitude = max(
                min_speed,
                min(self.max_distance_speed, self.distance_kp * abs(error)),
            )
            forward = speed_magnitude * forward_sign

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
        final_error = target_value - final_value
        self.get_logger().info(
            f"Segment finished on axis {axis_name}: value={final_value:.3f}, "
            f"target={target_value:.3f}, error={final_error:.3f}, stop_reason={stop_reason}, motion={motion_label}"
        )

    def settle(self, seconds=None):
        """转身前停稳：实测停稳后转身漂移从 ~0.3m 降到 ~0.1m。"""
        if seconds is None:
            seconds = self.settle_time
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def warmup(self, seconds=1.2, speed=0.15):
        """热身小步：locomotion 启动后第一个动作会猛甩 ~0.4m，先在出发区走掉这股冲击。"""
        self.get_logger().info("Warm-up: short forward step to clear cold-start lurch")
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(forward=speed, angular=0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
        self.stop()
        self.settle()

    def move_to_target(self):
        """执行五阶段导航(热身→摆正x→走中途点→安全低位大转身→倒走入区→回正)。

        各阶段细节见模块顶部 brief。前提：已 STAND→注册输入源→LOCOMOTION→odom 就绪。
        """
        self.log_pose(prefix="Start pose")
        self.warmup()

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        self.get_logger().info(
            f"Target point: x={self.target_x:.3f}, y={self.target_y:.3f}, dx={dx:.3f}, dy={dy:.3f}"
        )

        if abs(dx) > self.position_tolerance:
            x_heading = 0.0 if dx >= 0.0 else math.pi
            self.get_logger().info(
                f"Stage 1: correct x by {dx:.3f} m with heading {math.degrees(x_heading):.1f} deg"
            )
            self.settle()
            self.turn_to_heading(x_heading)
            self.drive_axis_to_target("x", self.target_x, x_heading)

        dx_after = self.target_x - self.x
        dy_after = self.target_y - self.y
        self.get_logger().info(
            f"After stage 1: dx={dx_after:.3f}, dy={dy_after:.3f}"
        )

        # 关键：最终朝向 -90° 的大转身漂移极不稳(0.1~0.5m)且会往顶墙漂，绝不能放在终点。
        # 策略：正向快走到“中途点”(留 backward_approach 余量) → 在安全低位完成大转身 →
        # 末段朝最终朝向倒走对正，走完即终态，不再有转身破坏定位。
        sign = 1.0 if dy_after >= 0.0 else -1.0
        y_mid = self.target_y - sign * self.backward_approach

        if abs(y_mid - self.y) > self.position_tolerance:
            y_heading = math.pi / 2.0 if dy_after >= 0.0 else -math.pi / 2.0
            self.get_logger().info(
                f"Stage 2: turn to heading {math.degrees(y_heading):.1f} deg, "
                f"forward-walk to mid y={y_mid:.3f} (cross-track steering)"
            )
            self.settle()
            self.turn_to_heading(y_heading)
            self.drive_axis_to_target(
                "y", y_mid, y_heading, forward_sign=1.0, apply_cross_track=True
            )

        self.get_logger().info(
            f"Stage 3: big turn to final heading {math.degrees(self.target_yaw):.1f} deg "
            f"at safe low y={self.y:.3f} (away from 顶墙)"
        )
        self.settle()
        self.turn_to_heading(self.target_yaw)

        # Stage 4: 朝最终朝向倒走最后一段进交互区-I，跟线对正 x；走完即最终位姿，无后续转身。
        if abs(self.target_y - self.y) > self.position_tolerance:
            self.get_logger().info(
                f"Stage 4: back into 交互区-I, drive y to {self.target_y:.3f} "
                f"facing {math.degrees(self.target_yaw):.1f} deg (cross-track steering)"
            )
            self.settle(0.4)
            self.drive_axis_to_target(
                "y", self.target_y, self.target_yaw,
                forward_sign=-1.0, apply_cross_track=True, use_integral=False
            )

        # Stage 5: 末段跟线修 x 时鼻子会留几度偏角, 这里做一个小角度回正,
        # 锁死“正面面向交互区-II”。角度小(~5°), 转身漂移可忽略。
        self.get_logger().info("Stage 5: square up to final heading (face 交互区-II)")
        self.settle(0.4)
        self.turn_to_heading(self.target_yaw)

        final_dx = self.target_x - self.x
        final_dy = self.target_y - self.y
        self.stop()
        self.get_logger().info(
            f"Navigation finished: final x={self.x:.3f}, y={self.y:.3f}, "
            f"error dx={final_dx:.3f}, dy={final_dy:.3f}"
        )

    def stop(self, repeats=10):
        """连发多次零速度，确保 mc 收到停止指令(单条可能丢)。"""
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


_global_node = None


def _signal_handler(sig, _frame):
    if _global_node is not None:
        _global_node.stop()
        _global_node.get_logger().info(
            f"Received signal {sig}, stopping and shutting down"
        )
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Closed-loop navigation to a target point using odometry"
    )
    parser.add_argument(
        "--reset-prompt",
        action="store_true",
        help="设 STAND 后暂停等你按 Reset 再回车继续 (默认不暂停: 请先在仿真里 Reset 再运行)",
    )
    parser.add_argument(
        "--stand-time",
        type=float,
        default=1.5,
        help="Seconds to wait after Reset before switching to locomotion",
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used for closed-loop feedback",
    )
    parser.add_argument(
        "--target-x",
        type=float,
        default=-0.08,
        help="Target x position in map frame. 交互区-I 圆心 x=0; 取 -0.08 补偿末态系统性 +x 残差(~0.07), 使落点≈0、双脚对称跨圆心",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=1.70,
        help="Target y position in map frame. 交互区-I 圆心 y=1.7 半径0.25, 顶墙内面 y≈1.85; "
        "取 1.70 正对圆心使整只脚(含脚角)入圈, 背对墙站立离墙 ~7cm 仍安全",
    )
    parser.add_argument(
        "--target-yaw-deg",
        type=float,
        default=-90.0,
        help="Final target yaw in degrees after reaching the target point",
    )
    parser.add_argument(
        "--forward-speed",
        type=float,
        default=0.35,
        help="Maximum forward speed in m/s",
    )
    parser.add_argument(
        "--turn-speed",
        type=float,
        default=0.4,
        help="Maximum turn speed in rad/s",
    )
    parser.add_argument(
        "--distance-kp",
        type=float,
        default=0.6,
        help="Proportional gain for distance control",
    )
    parser.add_argument(
        "--heading-kp",
        type=float,
        default=1.2,
        help="Proportional gain for heading control",
    )
    parser.add_argument(
        "--segment-timeout-sec",
        type=float,
        default=25.0,
        help="Maximum seconds allowed for each translation segment before stopping it",
    )
    parser.add_argument(
        "--final-turn-timeout-sec",
        type=float,
        default=10.0,
        help="Maximum seconds allowed for the final heading turn",
    )
    parser.add_argument(
        "--pose-log-interval",
        type=float,
        default=0.5,
        help="Seconds between pose logs while moving",
    )
    return parser.parse_args()


def main():
    """入口：解析参数 → 建节点 → STAND →(可选等Reset)→ 注册输入源 →
    LOCOMOTION → 等 odom → 执行五阶段导航 → 收尾停车关闭。
    """
    global _global_node

    args = _parse_args()

    rclpy.init()
    node = WalkForwardSteps(
        args.odom_topic,
        args.target_x,
        args.target_y,
        args.target_yaw_deg,
        args.forward_speed,
        args.turn_speed,
        args.distance_kp,
        args.heading_kp,
        args.pose_log_interval,
        args.segment_timeout_sec,
        args.final_turn_timeout_sec,
    )
    _global_node = node

    # Ctrl-C / kill 时确保先停车再退出，避免机器人带速度失控
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # 1) 切站立平衡（Reset 后机器人在此模式下保持站立）
    if not node.set_mode("STAND_DEFAULT"):
        node.get_logger().error("Failed to set stand mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # （可选）暂停等用户在仿真里 Reset；默认不暂停，要求运行前已 Reset 站好
    if args.reset_prompt:
        print(
            "\nSTAND_DEFAULT is active. Click Reset in the simulator now; "
            "the robot should appear standing in the start area.\n"
            "Press Enter here after Reset to continue...",
            flush=True,
        )
        input()

    time.sleep(args.stand_time)  # 等站稳

    # 2) 注册本节点为运动输入源（否则速度指令不被采纳）
    if not node.register_input_source():
        node.get_logger().error("Input source registration failed, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 3) 切行走模式
    if not node.set_locomotion_mode():
        node.get_logger().error("Failed to set locomotion mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 4) 等里程计第一帧（闭环反馈的前提）
    if not node.wait_for_odom():
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 5) 执行五阶段导航；无论成功与否 finally 都停车关闭
    try:
        node.move_to_target()
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
