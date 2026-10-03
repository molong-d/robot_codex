"""Read-only runtime diagnostics and optional JSON export; never sends a robot goal."""
import argparse
import json
import re
import sys
from pathlib import Path

import rclpy
from robot_interfaces.srv import GetExecution, GetRuntimeState
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser(description="查询机器人运行状态及任务记录；可导出诊断 JSON")
    parser.add_argument("--execution-id", default="", help="Action goal UUID 的 32 位小写十六进制；默认活动或最近任务")
    parser.add_argument("--output", type=Path, help="保存新的 JSON 文件；已有文件不会覆盖")
    args = parser.parse_args()
    if args.execution_id and not re.fullmatch(r"[0-9a-f]{32}", args.execution_id):
        parser.error("execution-id 需要 32 位小写十六进制 UUID，不含连字符")

    rclpy.init()
    node = rclpy.create_node("inspect_robot_runtime")
    clients = []
    try:
        def query(service, name, request):
            client = node.create_client(service, name)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f"查询服务不可用：{name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise RuntimeError(f"查询超时：{name}；不会重试或重新执行任务")
            return future.result()

        state = query(GetRuntimeState, "get_runtime_state", GetRuntimeState.Request())
        execution = query(GetExecution, "get_execution", GetExecution.Request(execution_id=args.execution_id))
        if state.schema_version != 1 or execution.schema_version != 1:
            raise RuntimeError("不支持的诊断协议版本")
        if state.runtime_id != execution.runtime_id:
            raise RuntimeError("两次查询之间 runtime 已重启；请重新查询")
        # Separate service calls are not an atomic snapshot. Preserve both responses.
        result = {"schema_version": 1, "runtime_state": message_to_ordereddict(state),
                  "execution": message_to_ordereddict(execution)}
        output = json.dumps(result, ensure_ascii=False, indent=2)
        print(output)
        if args.output is not None:
            with args.output.open("x", encoding="utf-8") as file:
                file.write(output+"\n")
        if args.execution_id and not execution.found:
            print("该任务记录未保留；可能已淘汰、来自其他 runtime 或未被接收。不能据此判断任务未执行。", file=sys.stderr)
            return 2
        return 0
    except (RuntimeError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        for client in clients:
            node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
