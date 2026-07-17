# CLAUDE.md 

## 1. 目录规则

### 工作空间

```
~/ws_px4_ros2/
├── src/                  ← 唯一手动编辑区
├── build/ install/ log/  ← colcon 自动生成，禁止手动改
```

### 功能包：所有源码一律放 `src/` 下

```
功能包/
├── src/                          ← 所有源码
│   ├── 功能包/                    ← Python 可导入模块（含 __init__.py）
│   ├── src/                      ← 可执行脚本（.py / .cpp）
│   └── lib/                      ← C++ 库
├── include/                      ← C++ 头文件
├── launch/ test/ CMakeLists.txt package.xml
```

:no_entry: 禁止在包根目录放 Python 模块目录（如 `px4_ros_com/px4_ros_com/__init__.py`），应当改为 `px4_ros_com/src/px4_ros_com/__init__.py`。

CMakeLists 对应写法：
```cmake
ament_python_install_package(src/${PROJECT_NAME})               # 安装 Python 包
install(PROGRAMS src/examples/foo.py DESTINATION lib/${PROJECT_NAME})  # 注册可执行脚本
```

## 2. 编码注意事项
### 整体注意事项
**offboard 控制**
- 从第一个 tick 就同时发 **heartbeat + setpoint**（PX4 需要先收到 setpoint 才接受 offboard）
- 不要用固定计数推进状态，用 `vehicle_status` 回调中的实际状态（`arming_state`、`nav_state`）判断
- mode switch 和 arm 必须**失败重试**，不能只发一次
- 控制频率 ≥ 10Hz，低于 2Hz 触发 offboard lost
- 所有参数从config里面的yaml文件加载，如果文件没有则报错并停止，而不要在代码里重复declare
- MAVROS 的 `setpoint_position` 和 `local_position` 插件在 ROS 侧使用 **ENU**（z 正 = 向上）。MAVROS 内部自动做 ENU↔NED 转换。
- 在 launch 文件中 MAVROS 的 `Node` action 添加 `emulate_tty=True`,防止手动ctrl+c之后无法正确释放进程
- 不要高频发送重复的ros info消息，只在诸如状态切换时发送ros info
### 仿真注意事项
**环境：**
- 强制 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（Zenoh 需要额外 Router，会导致 topic 发现不了）
- 仿真需设 `COM_RCL_EXCEPT=4`（关闭 RC 检查），通过 pymavlink 连 UDP 14560 设置




### 资料来源
当编码碰到问题时，优先查询以下的网站
* ROS2官方文档：https://docs.ros.org/en/humble/、
* px4 ROS2官方文档：https://docs.px4.io/v1.15/en/ros2/
* px4_ros_com官方例程：https://github.com/PX4/px4_ros_com/blob/main/src/examples/
* px4_msg消息定义：https://github.com/PX4/px4_msgs
* px4常见问题排查 https://github.com/PX4/PX4-Autopilot/issues/ 
回答我的相关问题时，应查询这几个网站之后作出回答，并且标注来源
# Behaviour Guide

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.