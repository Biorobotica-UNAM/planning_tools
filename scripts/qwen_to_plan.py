#!/usr/bin/env python3
"""Send a command to Qwen, convert returned goals to CLIPS, and print the plan."""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from clips_interfaces.srv import String as ClipsString
from rclpy.node import Node
from semantic_parser_interfaces.srv import Parser
from std_msgs.msg import String as StringMsg


class QwenToPlanTester(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("qwen_to_plan_tester")
        self._args = args
        self._last_plan = ""
        self._parser_client = self.create_client(Parser, args.parser_service)
        self._goals_client = self.create_client(ClipsString, args.goals_service)
        self.create_subscription(StringMsg, args.results_topic, self._on_plan, 10)

    def run(self) -> int:
        qwen_response = self._call_qwen()
        if not qwen_response:
            return 1

        print("\n=== Qwen response ===")
        print(qwen_response)

        if not self._has_goals(qwen_response):
            self.get_logger().error(
                "Qwen response does not contain a top-level 'goals' list. "
                "Check qwen_response_format:=goals."
            )
            return 1

        convert_response = self._call_goals_to_clips(qwen_response)
        if not convert_response:
            return 1

        print("\n=== goals_to_clips response ===")
        print(convert_response)

        if convert_response.startswith("ERROR"):
            return 1

        plan = self._wait_for_plan()
        if not plan:
            self.get_logger().error("Timed out waiting for plan on %s", self._args.results_topic)
            return 1

        print("\n=== Generated plan ===")
        print(plan)
        return 0

    def _call_qwen(self) -> str:
        if not self._parser_client.wait_for_service(timeout_sec=self._args.service_timeout):
            self.get_logger().error("Parser service unavailable: %s", self._args.parser_service)
            return ""

        request = Parser.Request()
        request.command = self._args.command
        future = self._parser_client.call_async(request)
        if not self._wait_future(future, self._args.service_timeout):
            self.get_logger().error("Timed out waiting for Qwen response")
            return ""

        return future.result().response

    def _call_goals_to_clips(self, qwen_response: str) -> str:
        if not self._goals_client.wait_for_service(timeout_sec=self._args.service_timeout):
            self.get_logger().error("Goals service unavailable: %s", self._args.goals_service)
            return ""

        request = ClipsString.Request()
        request.data = qwen_response
        future = self._goals_client.call_async(request)
        if not self._wait_future(future, self._args.service_timeout):
            self.get_logger().error("Timed out waiting for goals_to_clips response")
            return ""

        return future.result().data

    def _wait_for_plan(self) -> str:
        deadline = time.monotonic() + self._args.plan_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_plan:
                return self._last_plan
        return ""

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done() and future.result() is not None

    def _on_plan(self, msg: StringMsg) -> None:
        self._last_plan = msg.data

    @staticmethod
    def _has_goals(payload: str) -> bool:
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            return False
        return isinstance(parsed, dict) and isinstance(parsed.get("goals"), list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test command -> Qwen goals -> goals_to_clips -> /planning/clips/results"
    )
    parser.add_argument("command", help="Natural-language command to send to Qwen")
    parser.add_argument(
        "--parser-service",
        default="/planning/semantic_parser/semantic_parser",
        help="semantic_parser_interfaces/srv/Parser service",
    )
    parser.add_argument(
        "--goals-service",
        default="/planning/goals_to_clips/convert",
        help="clips_interfaces/srv/String goals conversion service",
    )
    parser.add_argument(
        "--results-topic",
        default="/planning/clips/results",
        help="Topic where clips_node publishes the generated plan",
    )
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--plan-timeout", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = QwenToPlanTester(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
