#!/usr/bin/env python3
"""查询技能目录，将抓取放置目标分解为技能步骤，可显式增加结果验证。"""
import argparse
import json

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from robot_interfaces.action import ExecutePlan
from robot_interfaces.msg import SkillStep
from robot_interfaces.srv import GetCatalog


def assemble(catalog, object_id, target_id, implementation, verify_outcomes=False):
    if catalog.schema_version != 1:
        raise ValueError("不支持的目录版本")
    if object_id not in catalog.object_ids or target_id not in catalog.target_ids:
        raise ValueError("对象或目标不在配置目录中")
    skills = {skill.skill_id: skill for skill in catalog.skills}
    steps = []
    sequence = [
        ("locate_object", {"object": object_id}),
        ("pick_object", {"object": object_id}),
        ("locate_object", {"object": target_id}),
        ("place_object", {"object": object_id, "target": target_id}),
    ]
    if verify_outcomes:
        sequence.insert(2, ("verify_grasp", {"object": object_id}))
        sequence.append(("verify_placement", {"object": object_id, "target": target_id}))
    for skill_id, arguments in sequence:
        skill = skills[skill_id]
        if {p.name for p in skill.inputs} != set(arguments) or any(p.type != "entity_id" or not p.required for p in skill.inputs):
            raise ValueError(f"技能输入模式不兼容：{skill_id}")
        if not any(i.implementation_id == implementation and i.dependencies_satisfied for i in skill.implementations):
            raise ValueError(f"技能实现依赖不满足：{skill_id}/{implementation}")
        steps.append(SkillStep(skill_id=skill_id, implementation_id=implementation,
                               argument_names=list(arguments), argument_values=list(arguments.values())))
    return steps


def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("请求或结果未收到；任务可能仍在执行，检查状态后再处理")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", default="workpiece")
    parser.add_argument("--target", default="tray")
    parser.add_argument("--implementation", default="standard")
    parser.add_argument("--timeout-ms", type=int, default=90000)
    parser.add_argument("--dry-run", action="store_true", help="查询目录并打印计划，不提交执行")
    parser.add_argument("--verify-outcomes", action="store_true", help="增加抓稳和放置证据验证；示例证据为合成数据")
    args = parser.parse_args()
    if not 1 <= args.timeout_ms <= 600000:
        parser.error("timeout-ms 必须为 1～600000")
    rclpy.init(args=[])
    node = rclpy.create_node("pick_place_planner_example")
    catalog_client = node.create_client(GetCatalog, "get_catalog")
    plan_client = ActionClient(node, ExecutePlan, "execute_plan")
    try:
        if not catalog_client.wait_for_service(timeout_sec=10):
            raise RuntimeError("技能目录服务不可用")
        catalog = wait(node, catalog_client.call_async(GetCatalog.Request()), 10)
        steps = assemble(catalog, args.object, args.target, args.implementation, args.verify_outcomes)
        print(json.dumps({"schema_version": 1, "steps": [
            {"skill_id": s.skill_id, "implementation_id": s.implementation_id,
             "arguments": dict(zip(s.argument_names, s.argument_values))} for s in steps
        ]}, ensure_ascii=False), flush=True)
        if args.dry_run:
            return 0
        if not plan_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("计划执行服务不可用")
        goal = ExecutePlan.Goal(schema_version=1, steps=steps, timeout_ms=args.timeout_ms)
        handle = wait(node, plan_client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError("计划被拒绝；请检查运行时日志、资源与物体持有状态")
        result = wait(node, handle.get_result_async(), args.timeout_ms / 1000 + 20)
        print(json.dumps({"success": result.result.success, "status": result.result.status,
                          "error_code": result.result.error_code, "completed_steps": result.result.completed_steps,
                          "message": result.result.message}, ensure_ascii=False), flush=True)
        return 0 if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success else 1
    except (ValueError, KeyError, RuntimeError, TimeoutError) as error:
        print(json.dumps({"error": str(error)}, ensure_ascii=False), flush=True)
        return 1
    finally:
        plan_client.destroy()
        node.destroy_client(catalog_client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
