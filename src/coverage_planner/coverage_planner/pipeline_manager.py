#!/usr/bin/env python3
"""
Pipeline Manager Node  (fixed)
────────────────────────────────
Monitors /rosout for the frontier-explorer "EXPLORATION COMPLETE" message,
then saves the map and signals the CoveragePlanner via /exploration_done.

FIX: records its own start time and ignores any /rosout message whose
     timestamp predates node startup — this prevents stale log entries
     that are replayed by the ROS 2 logger from triggering a false handoff
     before exploration has even begun.
"""

import subprocess
import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


EXPLORATION_DONE_MSG = "NO FRONTIERS REMAINING — EXPLORATION COMPLETE!"
MAP_SAVE_DIR = os.path.expanduser("~/roomba_ws/maps")
MAP_NAME     = "explored_map"
STARTUP_IGNORE_SECS = 15.0
FRONTIER_EXPLORER_LOGGER = 'frontier_explorer'


class PipelineManager(Node):

    def __init__(self):
        super().__init__('pipeline_manager')

        self.declare_parameter('map_save_dir', MAP_SAVE_DIR)
        self.declare_parameter('map_name',     MAP_NAME)

        self._map_save_dir = self.get_parameter('map_save_dir').get_parameter_value().string_value
        self._map_name     = self.get_parameter('map_name').get_parameter_value().string_value

        self._exploration_done = False
        self._clock_ready = False
        self._start_time = None
        self._ignore_until = None

        self._clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self._clock_cb,
            10,
        )

        self._complete_sub = self.create_subscription(
            Bool,
            '/exploration_complete',
            self._exploration_complete_cb,
            10,
        )

        self._done_pub = self.create_publisher(Bool, '/exploration_done', 1)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self._rosout_cb,
            qos,
        )

        self.get_logger().info(
            f"Pipeline Manager ready. Watching /rosout for:\n"
            f"  '{EXPLORATION_DONE_MSG}'\n"
            f"  and /exploration_complete topic messages\n"
            f"  (ignoring messages older than node start time)"
        )
        self.get_logger().info(
            f"Map will be saved to: {self._map_save_dir}/{self._map_name}"
        )

    # ── /rosout watcher ────────────────────────────────────────────────── #

    def _clock_cb(self, msg: Clock):
        if self._clock_ready:
            return
        self._clock_ready = True
        self._start_time = self.get_clock().now()
        self._ignore_until = self._start_time + Duration(seconds=STARTUP_IGNORE_SECS)
        self.get_logger().info(
            f'Sim-time active — ignoring /rosout until {STARTUP_IGNORE_SECS:.0f}s after node startup.'
        )

    def _exploration_complete_cb(self, msg: Bool):
        if self._exploration_done:
            return
        if msg.data:
            self.get_logger().info(
                '🗺️  /exploration_complete received — starting map save + CPP handoff...'
            )
            self._exploration_done = True
            self._save_map_and_trigger()

    def _rosout_cb(self, msg: Log):
        if self._exploration_done:
            return
        if not self._clock_ready or self._start_time is None:
            return

        # ── Stale-message guard ──────────────────────────────────────────
        # msg.stamp is the timestamp of the original log call.
        # Only accept logs from the current sim-time window.
        msg_time = Time(
            nanoseconds=msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec,
            clock_type=self._start_time.clock_type,
        )
        if msg_time < self._start_time:
            return   # stale — ignore silently
        if self._ignore_until is not None and msg_time < self._ignore_until:
            return   # still in startup grace period
        if FRONTIER_EXPLORER_LOGGER not in msg.name:
            return   # only trust the frontier explorer logger
        # ────────────────────────────────────────────────────────────────

        if EXPLORATION_DONE_MSG in msg.msg:
            self.get_logger().info(
                "🗺️  Exploration complete detected! Starting map save + CPP handoff..."
            )
            self._exploration_done = True
            self._save_map_and_trigger()

    # ── Map save + signal ──────────────────────────────────────────────── #

    def _save_map_and_trigger(self):
        os.makedirs(self._map_save_dir, exist_ok=True)
        map_path = os.path.join(self._map_save_dir, self._map_name)

        self.get_logger().info(f"Saving map to {map_path} ...")

        try:
            result = subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_path,
                    "--ros-args", "-p", "save_map_timeout:=10.0",
                ],
                timeout=30,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                self.get_logger().info(
                    f"✅ Map saved → {map_path}.pgm / .yaml"
                )
            else:
                self.get_logger().error(
                    f"map_saver_cli returned {result.returncode}:\n{result.stderr}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error("Map saver timed out after 30 s.")
        except Exception as exc:
            self.get_logger().error(f"Map saver exception: {exc}")

        # Signal coverage planner
        self.get_logger().info("📡 Sending /exploration_done to Coverage Planner...")
        msg = Bool()
        msg.data = True
        for _ in range(5):
            self._done_pub.publish(msg)

        self.get_logger().info("Handoff complete — CPP will start shortly.")


def main(args=None):
    rclpy.init(args=args)
    node = PipelineManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()