#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from rmf_fleet_msgs.msg import FleetState
from rmf_fleet_msgs.msg import RobotMode
from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse
from rmf_task_msgs.msg import TaskSummary


ROBOTS = ("tinyRobot1", "tinyRobot2", "tinyRobot3")
FLEET_NAME = "tinyRobot"
NUMERIC_SUMMARY_FIELDS = (
    "accepted_tasks",
    "completed_tasks",
    "failed_or_canceled_tasks",
    "unfinished_tasks",
    "completion_rate_percent",
    "elapsed_sim_time_sec",
    "throughput_tasks_per_h_per_robot",
    "deadlock_events",
    "deadlock_frequency_per_100_tasks",
    "average_task_time_sec",
    "average_waiting_time_sec",
    "traffic_wait_fraction_percent",
    "total_busy_time_sec",
    "total_waiting_time_sec",
    "productive_ratio",
)
TERMINAL_STATES = {
    TaskSummary.STATE_COMPLETED,
    TaskSummary.STATE_FAILED,
    TaskSummary.STATE_CANCELED,
}

TASK_TEMPLATES: dict[str, dict[str, Any]] = {
    "tinyRobot1": {
        "category": "compose",
        "description": {
            "category": "warehouse_r1",
            "detail": "goal_A4 -> goal_W1 -> goal_D1 -> goal_W1",
            "phases": [
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_A4"}]},
                    }
                },
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_W1"}]},
                    }
                },
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_D1"}]},
                    }
                },
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_W1"}]},
                    }
                },
            ],
        },
    },
    "tinyRobot2": {
        "category": "compose",
        "description": {
            "category": "warehouse_r2",
            "detail": "goal_A3 -> goal_W3",
            "phases": [
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_A3"}]},
                    }
                },
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_W3"}]},
                    }
                },
            ],
        },
    },
    "tinyRobot3": {
        "category": "compose",
        "description": {
            "category": "warehouse_r3",
            "detail": "goal_C1 -> goal_W2",
            "phases": [
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_C1"}]},
                    }
                },
                {
                    "activity": {
                        "category": "go_to_place",
                        "description": {"one_of": [{"waypoint": "goal_W2"}]},
                    }
                },
            ],
        },
    },
}


@dataclass
class PendingRequest:
    robot: str
    sequence: int
    request_time_sim: float
    request_time_wall: float
    requester: str


@dataclass
class TaskRecord:
    task_id: str
    robot: str
    sequence: int
    requester: str
    request_time_sim: float
    response_success: bool = False
    submission_time: float | None = None
    start_time: float | None = None
    end_time: float | None = None
    state: int | None = None
    status: str = ""
    waiting_time: float = 0.0
    activated_by_robot: bool = False
    first_seen_by_robot_time: float | None = None
    terminal: bool = False
    completed: bool = False


@dataclass
class RobotTracker:
    last_time: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    current_task_id: str = ""
    assigned_task_id: str = ""
    current_mode: int = RobotMode.MODE_IDLE
    stationary_since: float | None = None
    last_waiting_like: bool = False
    waiting_mode_since: float | None = None


def ros_time_to_float(msg: Any) -> float | None:
    sec = getattr(msg, "sec", 0)
    nanosec = getattr(msg, "nanosec", 0)
    if sec == 0 and nanosec == 0:
        return None
    return float(sec) + float(nanosec) / 1e9


def deep_find_task_id(value: Any) -> str | None:
    if isinstance(value, dict):
        booking = value.get("booking")
        if isinstance(booking, dict):
            booking_id = booking.get("id")
            if isinstance(booking_id, str) and booking_id:
                return booking_id
        for nested in value.values():
            found = deep_find_task_id(nested)
            if found:
                return found
    elif isinstance(value, list):
        for nested in value:
            found = deep_find_task_id(nested)
            if found:
                return found
    return None


class WarehouseBenchmark(Node):
    def __init__(
        self,
        *,
        world: str,
        tasks_per_robot: int,
        startup_timeout: float,
        benchmark_timeout: float,
        max_sim_duration: float | None,
        verbose: bool,
    ) -> None:
        super().__init__(f"warehouse_benchmark_{world}_{uuid.uuid4().hex[:8]}")

        self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )

        self.world = world
        self.tasks_per_robot = tasks_per_robot
        self.startup_timeout = startup_timeout
        self.benchmark_timeout = benchmark_timeout
        self.max_sim_duration = max_sim_duration
        self.verbose = verbose
        self.total_target_tasks = tasks_per_robot * len(ROBOTS)

        self.control_period = 0.5
        # Thesis metric definitions (§6.6.1):
        # (i) no cumulative progress for >= 180s, or
        # (ii) a robot remains in WAITING-for-traffic without route advancement for >= 120s.
        self.deadlock_no_progress_window = 180.0
        self.deadlock_waiting_mode_window = 120.0
        self.deadlock_abort_window = 240.0
        self.stationary_speed_epsilon = 0.01
        self.progress_distance_epsilon = 0.03
        self.waiting_grace = 0.0
        self.min_inferred_task_duration = 5.0
        self.response_timeout_wall = 15.0

        self.ready_robots: set[str] = set()
        self.robot_trackers = {robot: RobotTracker() for robot in ROBOTS}

        self.pending_requests: dict[str, PendingRequest] = {}
        self.pending_by_robot: dict[str, str] = {}
        self.task_records: dict[str, TaskRecord] = {}
        self.tasks_accepted_per_robot = {robot: 0 for robot in ROBOTS}
        self.tasks_terminal_per_robot = {robot: 0 for robot in ROBOTS}

        self.total_busy_time = 0.0
        self.total_waiting_time = 0.0
        self.deadlock_events = 0
        self.deadlock_open = False
        self.last_progress_sim_time: float | None = None
        self.benchmark_started = False
        self.benchmark_finished = False
        self.terminated_early_due_to_deadlock = False
        self.terminated_early_due_to_time_limit = False
        self.benchmark_start_sim_time: float | None = None
        self.benchmark_end_sim_time: float | None = None
        self.benchmark_start_wall = time.monotonic()
        self.done = False
        self.error_message: str | None = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.request_pub = self.create_publisher(
            ApiRequest, "task_api_requests", transient_qos
        )
        self.response_sub = self.create_subscription(
            ApiResponse,
            "task_api_responses",
            self.on_api_response,
            transient_qos,
        )
        self.task_summary_sub = self.create_subscription(
            TaskSummary,
            "task_summaries",
            self.on_task_summary,
            state_qos,
        )
        self.fleet_state_sub = self.create_subscription(
            FleetState,
            "/fleet_states",
            self.on_fleet_state,
            state_qos,
        )
        # IMPORTANT: Do not rely solely on ROS-time timers here.
        # In some startup conditions, ROS-time may stall (e.g., /clock not flowing yet),
        # which would prevent timers from firing and can hang the benchmark indefinitely.
        # We drive `on_control_timer()` from the wall-clock loop in `run_single_world`.
        self.control_timer = None

    def log(self, message: str) -> None:
        if self.verbose:
            self.get_logger().info(message)

    def sim_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def system_ready(self) -> bool:
        now = self.sim_time()
        return (
            now > 0.0
            and self.count_publishers("task_api_responses") > 0
            and self.count_publishers("task_summaries") > 0
            and self.count_publishers("/fleet_states") > 0
            and len(self.ready_robots) == len(ROBOTS)
        )

    def readiness_status(self) -> str:
        return (
            f"sim_time={self.sim_time():.2f} "
            f"pub(task_api_responses)={self.count_publishers('task_api_responses')} "
            f"pub(task_summaries)={self.count_publishers('task_summaries')} "
            f"pub(/fleet_states)={self.count_publishers('/fleet_states')} "
            f"ready_robots={len(self.ready_robots)}/{len(ROBOTS)}"
        )

    def record_progress(self, now: float, reason: str) -> None:
        self.last_progress_sim_time = now
        if self.deadlock_open:
            self.log(f"Deadlock cleared by {reason} at sim {now:.1f}s")
            self.deadlock_open = False

    def on_api_response(self, msg: ApiResponse) -> None:
        pending = self.pending_requests.pop(msg.request_id, None)
        if not pending:
            return
        self.pending_by_robot.pop(pending.robot, None)

        try:
            payload = json.loads(msg.json_msg)
        except json.JSONDecodeError as exc:
            self.fail(f"无法解析任务响应 JSON: {exc}")
            return

        if not payload.get("success", False):
            self.fail(f"任务请求失败: {payload}")
            return

        task_id = deep_find_task_id(payload)
        if not task_id:
            self.fail(f"响应中未找到 task_id: {payload}")
            return

        record = TaskRecord(
            task_id=task_id,
            robot=pending.robot,
            sequence=pending.sequence,
            requester=pending.requester,
            request_time_sim=pending.request_time_sim,
            response_success=True,
        )
        self.task_records[task_id] = record
        self.tasks_accepted_per_robot[pending.robot] += 1
        now = self.sim_time()
        if self.benchmark_start_sim_time is None:
            self.benchmark_start_sim_time = now
        if self.last_progress_sim_time is None:
            self.last_progress_sim_time = now
        self.robot_trackers[pending.robot].assigned_task_id = task_id
        self.log(f"Accepted {task_id} for {pending.robot}")

    def on_task_summary(self, msg: TaskSummary) -> None:
        record = self.task_records.get(msg.task_id)
        if record is None:
            return

        record.state = msg.state
        record.status = msg.status
        record.submission_time = ros_time_to_float(msg.submission_time)
        record.start_time = ros_time_to_float(msg.start_time)
        record.end_time = ros_time_to_float(msg.end_time)
        if msg.robot_name:
            record.robot = msg.robot_name

        if msg.state in TERMINAL_STATES and not record.terminal:
            record.terminal = True
            record.completed = msg.state == TaskSummary.STATE_COMPLETED
            if record.end_time is None:
                record.end_time = self.sim_time()
            self.tasks_terminal_per_robot[record.robot] += 1
            tracker = self.robot_trackers[record.robot]
            if tracker.current_task_id == record.task_id:
                tracker.current_task_id = ""
            if tracker.assigned_task_id == record.task_id:
                tracker.assigned_task_id = ""
            self.record_progress(self.sim_time(), f"task {record.task_id} terminal")
            self.log(
                f"Task {record.task_id} on {record.robot} ended with state {msg.state}"
            )

    def on_fleet_state(self, msg: FleetState) -> None:
        if msg.name != FLEET_NAME:
            return

        now = self.sim_time()
        if now <= 0.0:
            return

        for robot in msg.robots:
            if robot.name not in self.robot_trackers:
                continue

            self.ready_robots.add(robot.name)
            tracker = self.robot_trackers[robot.name]
            current_x = float(robot.location.x)
            current_y = float(robot.location.y)
            current_task_id = robot.task_id
            current_mode = robot.mode.mode
            previous_task_id = tracker.current_task_id

            if current_task_id:
                current_record = self.task_records.get(current_task_id)
                if current_record is not None:
                    if not current_record.activated_by_robot:
                        current_record.activated_by_robot = True
                        current_record.first_seen_by_robot_time = now
                        self.record_progress(
                            now, f"{robot.name} acknowledged {current_task_id}"
                        )
                    if tracker.assigned_task_id == current_task_id:
                        tracker.assigned_task_id = ""

            if (
                self.benchmark_started
                and previous_task_id
                and previous_task_id != current_task_id
            ):
                previous_record = self.task_records.get(previous_task_id)
                if (
                    previous_record is not None
                    and not previous_record.terminal
                    and previous_record.activated_by_robot
                ):
                    active_for = (
                        now - previous_record.first_seen_by_robot_time
                        if previous_record.first_seen_by_robot_time is not None
                        else 0.0
                    )
                    if active_for < self.min_inferred_task_duration:
                        self.log(
                            f"Skip inferred completion for {previous_task_id} on "
                            f"{robot.name}: active_for={active_for:.2f}s is too short"
                        )
                    else:
                        previous_record.terminal = True
                        previous_record.completed = True
                        previous_record.status = "completed_inferred_from_fleet_state"
                        if previous_record.submission_time is None:
                            previous_record.submission_time = previous_record.request_time_sim
                        previous_record.end_time = now
                        self.tasks_terminal_per_robot[previous_record.robot] += 1
                        self.record_progress(
                            now,
                            f"{robot.name} switched task from {previous_task_id} to "
                            f"{current_task_id or 'idle'}",
                        )
                        self.log(
                            f"Inferred completion for {previous_task_id} on {robot.name}"
                        )
                elif previous_record is not None and not previous_record.activated_by_robot:
                    self.log(
                        f"Skip inferred completion for {previous_task_id} on "
                        f"{robot.name}: task was never observed in fleet_states"
                    )

            if (
                tracker.last_time is not None
                and tracker.last_x is not None
                and tracker.last_y is not None
                and self.benchmark_started
            ):
                dt = now - tracker.last_time
                if dt > 0.0:
                    distance = math.hypot(
                        current_x - tracker.last_x,
                        current_y - tracker.last_y,
                    )
                    speed = distance / dt if dt > 0.0 else 0.0
                    current_record = self.task_records.get(current_task_id)
                    busy = (
                        current_record is not None
                        and not current_record.terminal
                    )
                    if busy:
                        self.total_busy_time += dt

                    if busy and distance >= self.progress_distance_epsilon:
                        self.record_progress(
                            now, f"{robot.name} moved {distance:.3f}m"
                        )

                    waiting_like = False
                    if busy:
                        if current_mode in (
                            RobotMode.MODE_WAITING,
                            RobotMode.MODE_PAUSED,
                        ):
                            waiting_like = True
                        elif speed < self.stationary_speed_epsilon:
                            if tracker.stationary_since is None:
                                tracker.stationary_since = tracker.last_time
                            if now - tracker.stationary_since >= self.waiting_grace:
                                waiting_like = True
                        else:
                            tracker.stationary_since = None
                    else:
                        tracker.stationary_since = None

                    # Track continuous WAITING-for-traffic duration (for deadlock rule ii)
                    if busy and current_mode == RobotMode.MODE_WAITING:
                        if tracker.waiting_mode_since is None:
                            tracker.waiting_mode_since = tracker.last_time
                    else:
                        tracker.waiting_mode_since = None

                    if waiting_like:
                        self.total_waiting_time += dt
                        if current_record is not None:
                            current_record.waiting_time += dt

                    tracker.last_waiting_like = waiting_like

            tracker.last_time = now
            tracker.last_x = current_x
            tracker.last_y = current_y
            tracker.current_task_id = current_task_id
            tracker.current_mode = current_mode

    def dispatch_task(self, robot: str) -> None:
        sequence = self.tasks_accepted_per_robot[robot] + len(
            [p for p in self.pending_requests.values() if p.robot == robot]
        )
        requester = (
            f"warehouse_benchmark/{self.world}/{robot}/{sequence}/{uuid.uuid4().hex[:8]}"
        )
        now = self.sim_time()
        now_ms = int(now * 1000)

        payload = {
            "type": "robot_task_request",
            "robot": robot,
            "fleet": FLEET_NAME,
            "request": {
                "unix_millis_request_time": now_ms,
                "unix_millis_earliest_start_time": now_ms,
                "requester": requester,
                "category": TASK_TEMPLATES[robot]["category"],
                "description": TASK_TEMPLATES[robot]["description"],
                "fleet_name": FLEET_NAME,
            },
        }

        msg = ApiRequest()
        msg.request_id = f"bench_{uuid.uuid4()}"
        msg.json_msg = json.dumps(payload)
        self.request_pub.publish(msg)

        self.pending_requests[msg.request_id] = PendingRequest(
            robot=robot,
            sequence=self.tasks_accepted_per_robot[robot],
            request_time_sim=now,
            request_time_wall=time.monotonic(),
            requester=requester,
        )
        self.pending_by_robot[robot] = msg.request_id
        self.log(f"Dispatched request {msg.request_id} for {robot}")

    def should_dispatch(self, robot: str) -> bool:
        if self.tasks_accepted_per_robot[robot] >= self.tasks_per_robot:
            return False
        if robot in self.pending_by_robot:
            return False

        tracker = self.robot_trackers[robot]
        assigned_record = self.task_records.get(tracker.assigned_task_id)
        if assigned_record is not None and not assigned_record.terminal:
            return False
        current_record = self.task_records.get(tracker.current_task_id)
        if current_record is not None and not current_record.terminal:
            return False
        return True

    def check_pending_timeouts(self) -> None:
        now = time.monotonic()
        expired = [
            request_id
            for request_id, pending in self.pending_requests.items()
            if now - pending.request_time_wall > self.response_timeout_wall
        ]
        for request_id in expired:
            pending = self.pending_requests.pop(request_id)
            self.pending_by_robot.pop(pending.robot, None)
            self.fail(
                f"任务响应超时: robot={pending.robot}, request_id={request_id}"
            )

    def active_task_records(self) -> list[TaskRecord]:
        return [record for record in self.task_records.values() if not record.terminal]

    def detect_deadlock(self) -> None:
        if self.last_progress_sim_time is None:
            return

        now = self.sim_time()
        active = self.active_task_records()
        if not active:
            return

        # Rule (i): no fleet-level progress for >= 180s.
        no_progress_long_enough = (
            (now - self.last_progress_sim_time) >= self.deadlock_no_progress_window
        )

        # Rule (ii): any robot stays in WAITING-for-traffic for >= 120s.
        any_robot_waiting_too_long = False
        longest_waiting = 0.0
        for robot in ROBOTS:
            tracker = self.robot_trackers[robot]
            if tracker.waiting_mode_since is None:
                continue
            waiting_for = now - tracker.waiting_mode_since
            longest_waiting = max(longest_waiting, waiting_for)
            if waiting_for >= self.deadlock_waiting_mode_window:
                any_robot_waiting_too_long = True

        if (no_progress_long_enough or any_robot_waiting_too_long) and not self.deadlock_open:
            self.deadlock_events += 1
            self.deadlock_open = True
            reason = (
                f"no progress for {now - self.last_progress_sim_time:.1f}s"
                if no_progress_long_enough
                else f"robot WAITING for {longest_waiting:.1f}s"
            )
            self.log(f"Deadlock event #{self.deadlock_events} at sim {now:.1f}s ({reason})")

    def is_complete(self) -> bool:
        accepted = sum(self.tasks_accepted_per_robot.values())
        terminal = sum(self.tasks_terminal_per_robot.values())
        return (
            accepted == self.total_target_tasks
            and terminal == self.total_target_tasks
            and not self.pending_requests
        )

    def fail(self, message: str) -> None:
        if self.error_message is None:
            self.error_message = message
            self.get_logger().error(message)
            self.done = True

    def on_control_timer(self) -> None:
        if self.done:
            return

        now_sim = self.sim_time()
        if not self.system_ready():
            if self.verbose:
                self.get_logger().info(
                    f"Waiting for system readiness... {self.readiness_status()}"
                )
            if time.monotonic() - self.benchmark_start_wall > self.startup_timeout:
                self.fail("启动超时: 未等到时钟/话题/机器人状态全部就绪")
            return

        if not self.benchmark_started:
            self.benchmark_started = True
            self.benchmark_start_sim_time = now_sim
            self.last_progress_sim_time = now_sim
            self.log(f"Benchmark started for {self.world} at sim {now_sim:.1f}s")

        if now_sim - (self.benchmark_start_sim_time or now_sim) > self.benchmark_timeout:
            self.fail(f"基准测试超时: 超过 {self.benchmark_timeout:.1f}s 仿真时间")
            return

        if (
            self.max_sim_duration is not None
            and self.benchmark_start_sim_time is not None
            and (now_sim - self.benchmark_start_sim_time) >= self.max_sim_duration
        ):
            self.terminated_early_due_to_time_limit = True
            self.benchmark_end_sim_time = now_sim
            self.benchmark_finished = True
            self.done = True
            self.get_logger().warning(
                "达到固定仿真时长上限，提前结束本轮 benchmark"
            )
            return

        self.check_pending_timeouts()
        if self.done:
            return

        for robot in ROBOTS:
            if self.should_dispatch(robot):
                self.dispatch_task(robot)

        self.detect_deadlock()

        if (
            self.deadlock_open
            and self.last_progress_sim_time is not None
            and (now_sim - self.last_progress_sim_time) >= self.deadlock_abort_window
        ):
            self.terminated_early_due_to_deadlock = True
            self.benchmark_end_sim_time = now_sim
            self.benchmark_finished = True
            self.done = True
            self.get_logger().warning(
                "检测到持续死锁，提前结束本轮 benchmark 以输出部分结果"
            )
            return

        if self.is_complete():
            self.benchmark_end_sim_time = now_sim
            self.benchmark_finished = True
            self.done = True
            self.log(f"Benchmark finished for {self.world} at sim {now_sim:.1f}s")

    def build_results(self) -> dict[str, Any]:
        if self.error_message:
            raise RuntimeError(self.error_message)

        if self.benchmark_start_sim_time is None or self.benchmark_end_sim_time is None:
            raise RuntimeError("benchmark 未正常开始或结束")

        elapsed = self.benchmark_end_sim_time - self.benchmark_start_sim_time
        completed = [record for record in self.task_records.values() if record.completed]
        failed = [
            record
            for record in self.task_records.values()
            if record.terminal and not record.completed
        ]

        accepted_tasks = len(self.task_records)
        completed_tasks = len(completed)
        completion_rate_percent = (
            (completed_tasks / accepted_tasks) * 100.0 if accepted_tasks > 0 else 0.0
        )

        completed_durations = [
            (record.end_time or 0.0) - (record.submission_time or record.request_time_sim)
            for record in completed
        ]
        avg_task_time = (
            sum(completed_durations) / len(completed_durations)
            if completed_durations
            else 0.0
        )
        avg_waiting_time = (
            sum(record.waiting_time for record in completed) / len(completed)
            if completed
            else 0.0
        )

        # Throughput: tasks completed per robot per hour of simulated time (tasks/h/robot)
        throughput_tasks_per_h_per_robot = (
            (completed_tasks / (elapsed / 3600.0) / len(ROBOTS))
            if elapsed > 0.0
            else 0.0
        )

        # Traffic-wait fraction: mean over completed tasks of waiting_time / task_time (%)
        per_task_wait_fractions = []
        for record in completed:
            duration = (record.end_time or 0.0) - (
                record.submission_time or record.request_time_sim
            )
            if duration <= 0.0:
                continue
            per_task_wait_fractions.append((record.waiting_time / duration) * 100.0)
        traffic_wait_fraction_percent = (
            sum(per_task_wait_fractions) / len(per_task_wait_fractions)
            if per_task_wait_fractions
            else 0.0
        )

        # Replace saturated utilization with a congestion-sensitive productive ratio.
        productive_ratio = (
            (self.total_busy_time / (self.total_busy_time + self.total_waiting_time))
            if (self.total_busy_time + self.total_waiting_time) > 0.0
            else 0.0
        )

        deadlock_per_100 = (self.deadlock_events / max(accepted_tasks, 1) * 100.0)

        return {
            "world": self.world,
            "tasks_per_robot": self.tasks_per_robot,
            "total_target_tasks": self.total_target_tasks,
            "accepted_tasks": accepted_tasks,
            "completed_tasks": completed_tasks,
            "failed_or_canceled_tasks": len(failed),
            "unfinished_tasks": accepted_tasks - completed_tasks - len(failed),
            "completion_rate_percent": completion_rate_percent,
            "terminated_early_due_to_deadlock": self.terminated_early_due_to_deadlock,
            "terminated_early_due_to_time_limit": self.terminated_early_due_to_time_limit,
            "elapsed_sim_time_sec": elapsed,
            "throughput_tasks_per_h_per_robot": throughput_tasks_per_h_per_robot,
            "deadlock_events": self.deadlock_events,
            "deadlock_frequency_per_100_tasks": deadlock_per_100,
            "average_task_time_sec": avg_task_time,
            "average_waiting_time_sec": avg_waiting_time,
            "traffic_wait_fraction_percent": traffic_wait_fraction_percent,
            "total_busy_time_sec": self.total_busy_time,
            "total_waiting_time_sec": self.total_waiting_time,
            "productive_ratio": productive_ratio,
            "units": {
                "completion_rate_percent": "%",
                "elapsed_sim_time_sec": "s",
                "throughput_tasks_per_h_per_robot": "tasks/h/robot",
                "deadlock_frequency_per_100_tasks": "events/100 tasks",
                "average_task_time_sec": "s",
                "average_waiting_time_sec": "s",
                "traffic_wait_fraction_percent": "%",
                "productive_ratio": "ratio",
            },
            "heuristics": {
                "waiting_time_rule": (
                    "任务执行期间，机器人处于 WAITING/PAUSED，或速度低于 0.01m/s"
                ),
                "deadlock_rule": (
                    "满足任一条件则记录死锁事件：(i) 全队 180 秒无位置进展/任务完成；"
                    "(ii) 任一机器人处于 WAITING(traffic) 且无进展持续 120 秒"
                ),
            },
        }


def terminate_process_tree(process: subprocess.Popen[Any] | None) -> None:
    if process is None:
        return
    if process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return
        time.sleep(0.2)

    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return
        time.sleep(0.2)

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def cleanup_stray_sim_processes() -> None:
    """
    Best-effort cleanup of leftover simulation/launch processes.

    When running many sequential trials, it's possible for Gazebo or ros2 launch
    processes to survive after interrupts/crashes. Stray processes can interfere with
    subsequent launches and cause silent hangs. We therefore terminate them before
    starting a new trial.
    """
    patterns = [
        "ros2 launch rmf_demos_gz",
        "gz sim -s -r -v",
    ]
    signals = ["-INT", "-TERM", "-KILL"]
    for sig in signals:
        for pat in patterns:
            subprocess.run(
                ["pkill", sig, "-f", pat],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        time.sleep(1.0)


def run_single_world(
    *,
    world: str,
    run_index: int,
    tasks_per_robot: int,
    startup_timeout: float,
    benchmark_timeout: float,
    max_sim_duration: float | None,
    results_dir: Path,
    launch_world: bool,
    verbose: bool,
) -> dict[str, Any]:
    launch_process: subprocess.Popen[Any] | None = None
    launch_log_path = results_dir / f"{world}.launch.log"

    if launch_world:
        cleanup_stray_sim_processes()
        with launch_log_path.open("w", encoding="utf-8") as launch_log:
            launch_process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "rmf_demos_gz",
                    f"{world}.launch.xml",
                    "headless:=true",
                ],
                cwd=str(results_dir.parent),
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        time.sleep(2.0)

    node = WarehouseBenchmark(
        world=world,
        tasks_per_robot=tasks_per_robot,
        startup_timeout=startup_timeout,
        benchmark_timeout=benchmark_timeout,
        max_sim_duration=max_sim_duration,
        verbose=verbose,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        wall_start = time.monotonic()
        next_control_tick = wall_start
        while rclpy.ok() and not node.done:
            if launch_process is not None and launch_process.poll() is not None:
                raise RuntimeError(
                    f"{world} 启动进程提前退出，请检查 {launch_log_path}"
                )
            executor.spin_once(timeout_sec=0.2)

            now_wall = time.monotonic()
            # Drive control loop from wall-clock to avoid ROS-time stalls.
            if now_wall >= next_control_tick:
                node.on_control_timer()
                next_control_tick = now_wall + node.control_period

            # Extra watchdog: if control loop never starts benchmark, bail out by wall time.
            if (
                not node.benchmark_started
                and (now_wall - wall_start) > startup_timeout
            ):
                raise RuntimeError(
                    f"{world} 启启动超时(看门狗): {node.readiness_status()}"
                )

        result = node.build_results()
        result["run_index"] = run_index
        result_path = results_dir / f"{world}.result.json"
        result_path.write_text(
            json.dumps(result, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        result["result_json"] = str(result_path)
        result["launch_log"] = str(launch_log_path) if launch_world else None
        return result
    finally:
        executor.remove_node(node)
        node.destroy_node()
        terminate_process_tree(launch_process)
        if launch_world:
            cleanup_stray_sim_processes()


def format_result(result: dict[str, Any]) -> str:
    return (
        f"run={result['run_index']} "
        f"[{result['world']}] "
        f"completed={result['completed_tasks']}/{result['accepted_tasks']}, "
        f"throughput={result['throughput_tasks_per_h_per_robot']:.2f} tasks/h/robot, "
        f"deadlock/100={result['deadlock_frequency_per_100_tasks']:.2f}, "
        f"avg_task={result['average_task_time_sec']:.2f}s, "
        f"avg_wait={result['average_waiting_time_sec']:.2f}s, "
        f"wait_frac={result['traffic_wait_fraction_percent']:.1f}%, "
        f"completion={result['completion_rate_percent']:.1f}%"
    )


def compute_mean_std(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "std": 0.0}
    if len(values) == 1:
        return {"mean": float(values[0]), "std": 0.0}
    return {
        "mean": float(statistics.mean(values)),
        "std": float(statistics.stdev(values)),
    }


def compute_summary(values: list[float]) -> dict[str, float]:
    if not values:
        return {
            "mean": 0.0,
            "std": 0.0,
            "median": 0.0,
            "q1": 0.0,
            "q3": 0.0,
            "iqr": 0.0,
        }
    sorted_vals = sorted(float(v) for v in values)
    mean_std = compute_mean_std(sorted_vals)
    median = float(statistics.median(sorted_vals))
    if len(sorted_vals) >= 4:
        q1 = float(statistics.quantiles(sorted_vals, n=4, method="inclusive")[0])
        q3 = float(statistics.quantiles(sorted_vals, n=4, method="inclusive")[2])
    else:
        q1 = float(sorted_vals[0])
        q3 = float(sorted_vals[-1])
    return {
        **mean_std,
        "median": median,
        "q1": q1,
        "q3": q3,
        "iqr": float(q3 - q1),
    }


def aggregate_world_runs(world: str, runs: list[dict[str, Any]]) -> dict[str, Any]:
    numeric_summary = {
        field: compute_summary([float(run[field]) for run in runs])
        for field in NUMERIC_SUMMARY_FIELDS
    }
    return {
        "world": world,
        "run_count": len(runs),
        "runs_terminated_early_due_to_deadlock": sum(
            1 for run in runs if run["terminated_early_due_to_deadlock"]
        ),
        "runs_terminated_early_due_to_time_limit": sum(
            1 for run in runs if run["terminated_early_due_to_time_limit"]
        ),
        "summary": numeric_summary,
        "runs": runs,
    }


def format_aggregate_result(aggregate: dict[str, Any]) -> str:
    summary = aggregate["summary"]
    return (
        f"[{aggregate['world']}] "
        f"throughput={summary['throughput_tasks_per_h_per_robot']['mean']:.2f}"
        f"+/-{summary['throughput_tasks_per_h_per_robot']['std']:.2f} tasks/h/robot, "
        f"deadlock/100={summary['deadlock_frequency_per_100_tasks']['mean']:.2f}"
        f"+/-{summary['deadlock_frequency_per_100_tasks']['std']:.2f}, "
        f"avg_task={summary['average_task_time_sec']['mean']:.2f}"
        f"+/-{summary['average_task_time_sec']['std']:.2f}s, "
        f"avg_wait={summary['average_waiting_time_sec']['mean']:.2f}"
        f"+/-{summary['average_waiting_time_sec']['std']:.2f}s, "
        f"wait_frac={summary['traffic_wait_fraction_percent']['mean']:.1f}"
        f"+/-{summary['traffic_wait_fraction_percent']['std']:.1f}%, "
        f"completion={summary['completion_rate_percent']['mean']:.1f}"
        f"+/-{summary['completion_rate_percent']['std']:.1f}%"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark warehouse and warehouse_perf RMF worlds."
    )
    parser.add_argument(
        "--world",
        choices=["warehouse", "warehouse_perf"],
        help="Only run one world.",
    )
    parser.add_argument(
        "--all-worlds",
        action="store_true",
        help="Run both warehouse and warehouse_perf sequentially.",
    )
    parser.add_argument(
        "--tasks-per-robot",
        type=int,
        default=34,
        help="Number of targeted compose tasks dispatched to each robot.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=1,
        help="Number of repeated trials for each selected world.",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=180.0,
        help="Wall-clock seconds to wait for ROS graph readiness.",
    )
    parser.add_argument(
        "--benchmark-timeout",
        type=float,
        default=2400.0,
        help="Maximum sim time per world in seconds.",
    )
    parser.add_argument(
        "--max-sim-duration",
        type=float,
        default=None,
        help="Stop after this many seconds of simulated time, even if tasks remain.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=Path("benchmark_results"),
        help="Directory for launch logs and JSON results.",
    )
    parser.add_argument(
        "--no-launch",
        action="store_true",
        help="Attach to an already running world instead of launching one.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed progress logs.",
    )
    args = parser.parse_args(argv)

    if not args.all_worlds and not args.world:
        parser.error("请使用 --world 或 --all-worlds")
    if args.repeats < 1:
        parser.error("--repeats 必须 >= 1")
    if args.no_launch and args.repeats > 1:
        parser.error("--no-launch 模式下不支持 --repeats > 1，因为脚本无法自动重置世界")
    if args.no_launch and not args.world:
        parser.error("--no-launch 模式下请显式指定 --world")

    worlds = [args.world] if args.world else ["warehouse", "warehouse_perf"]
    args.results_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=sys.argv)
    raw_results: list[dict[str, Any]] = []
    try:
        for run_index in range(1, args.repeats + 1):
            run_results_dir = args.results_dir / f"run_{run_index:03d}"
            run_results_dir.mkdir(parents=True, exist_ok=True)
            for world in worlds:
                result = run_single_world(
                    world=world,
                    run_index=run_index,
                    tasks_per_robot=args.tasks_per_robot,
                    startup_timeout=args.startup_timeout,
                    benchmark_timeout=args.benchmark_timeout,
                    max_sim_duration=args.max_sim_duration,
                    results_dir=run_results_dir,
                    launch_world=not args.no_launch,
                    verbose=args.verbose,
                )
                raw_results.append(result)
                print(format_result(result), flush=True)
    finally:
        rclpy.shutdown()

    aggregated_worlds = []
    for world in worlds:
        world_runs = [result for result in raw_results if result["world"] == world]
        aggregate = aggregate_world_runs(world, world_runs)
        aggregated_worlds.append(aggregate)
        print(format_aggregate_result(aggregate), flush=True)

    output = {
        "generated_at_unix": time.time(),
        "tasks_per_robot": args.tasks_per_robot,
        "repeats": args.repeats,
        "worlds": aggregated_worlds,
    }
    output_path = args.results_dir / "warehouse_benchmark_results.json"
    output_path.write_text(
        json.dumps(output, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"Saved JSON results to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
