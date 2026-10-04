# STM32 裁剪配置说明

日期：2026-10-04。已修改本机下位机源码，尚未烧录，也未完成实车验证。实际板上固件版本未知。

## 1. 配置位置与生效方式

集中配置文件：[jetson_robot_limits.h](</mnt/d/用户/Lenovo/桌面/Github Repository/Origional_STM32_Control_Code-main/Core/Inc/jetson_robot_limits.h:1>)。Windows 对应路径为：

```text
D:\用户\Lenovo\桌面\Github Repository\Origional_STM32_Control_Code-main\Core\Inc\jetson_robot_limits.h
```

这是**编译期配置**。修改后必须重新构建 STM32 固件并重新烧录；重启 Nano、改变 Nano 命令行参数不会修改这些固件配置。全部默认值沿用原有源码的数值。

## 2. 两个独立的裁剪

| 配置 | 默认值 | 含义 |
|---|---:|---|
| `JETSON_MOTOR_TARGET_CLIP_ENABLED` | `1` | 按关节类型裁剪要发送给电机的目标角度；`0` 关闭此层软件裁剪 |
| `JETSON_FEEDBACK_ERROR_CLIP_ENABLED` | `1` | 对回传位置与上一次目标之间的误差裁剪；`0` 关闭此层反馈裁剪 |
| `JETSON_FEEDBACK_MAX_ERROR_RAD` | `0.5f` | 反馈相对目标的误差窗口半宽，单位 rad，默认约 28.65° |

目标裁剪影响实际发给电机的指令。反馈裁剪影响模型收到的观测。例如上一次目标为 `0.2 rad`，编码器位置为 `1.0 rad`，默认窗口回传 `0.7 rad`。

**`JETSON_FEEDBACK_MAX_ERROR_RAD=0` 不是关闭裁剪：它会把回传位置压成上一次目标。关闭必须设置 `JETSON_FEEDBACK_ERROR_CLIP_ENABLED=0`。**

反馈函数保留原计算顺序：先做既有预测/原始反馈混合，再算 `delta=corrected-target`，先裁正上限、再裁负下限，最后返回 `target+delta`。原来的 `FRAME_ANGLE_LIMIT` 实际约束的是相对目标误差，名称中的“单帧”不准确；新配置据此命名。

关闭反馈裁剪后，原有滤波、前倾动作期间的反馈冻结仍按原代码运行。因此它只使正常回传路径绕过这个误差窗口。

## 3. 六类目标角度上下界

下表名称均有前缀 `JETSON_LIMIT_`、后缀 `_MIN_RAD` / `_MAX_RAD`，全部是弧度。同类型的左右关节继续共用一组限制。

| 中间名称 | 下界 | 上界 | 对称角度约值 |
|---|---:|---:|---:|
| `LEG_PITCH` | `-2.0f` | `2.0f` | ±114.59° |
| `LEG_ROLL` | `-0.8f` | `0.8f` | ±45.84° |
| `LEG_YAW` | `-1.5f` | `1.5f` | ±85.94° |
| `KNEE` | `-2.0f` | `2.0f` | ±114.59° |
| `ANKLE_PITCH` | `-1.5f` | `1.5f` | ±85.94° |
| `ANKLE_ROLL` | `-1.5f` | `1.5f` | ±85.94° |

示例完整名称：`JETSON_LIMIT_LEG_ROLL_MIN_RAD` / `JETSON_LIMIT_LEG_ROLL_MAX_RAD`。

这是电机控制入口看到的角度，在桥接层的坐标符号转换、前倾偏置之后生效，不是模型动作的归一化范围。现有电机 ID、左右映射、正负号未调整；之前审查发现的坐标合同疑点仍需独立验证。

修改示例，仅展示写法，不代表已经验证了此范围适合机械结构：

```c
#define JETSON_LIMIT_LEG_ROLL_MIN_RAD -1.1f
#define JETSON_LIMIT_LEG_ROLL_MAX_RAD  1.1f
#define JETSON_FEEDBACK_MAX_ERROR_RAD 0.8f
```

如果要关闭两个软件裁剪：

```c
#define JETSON_MOTOR_TARGET_CLIP_ENABLED 0
#define JETSON_FEEDBACK_ERROR_CLIP_ENABLED 0
```

配置宏用 `#ifndef` 包装，也支持编译器 `-D` 覆盖。例如：

```text
-DJETSON_FEEDBACK_ERROR_CLIP_ENABLED=0
-DJETSON_LIMIT_LEG_ROLL_MIN_RAD=-1.1f
-DJETSON_LIMIT_LEG_ROLL_MAX_RAD=1.1f
```

如使用 CMake，可给最终固件目标设置 `target_compile_definitions`；Keil 工程可使用 C/C++ 的 Define 设置。必须让所有相关源文件使用同一套宏，修改后做完整重构建。当前仓库的 CMake 使用 C11；没有现成 Keil 工程文件供本次实际构建验证。

## 4. 配置检查与协议范围

编译时拒绝以下配置：

- 两个启用开关不是 `0` 或 `1`。
- 反馈窗口为负值、NaN、无穷大或不是可折叠的编译期常量。
- 任一关节下界大于上界，或边界不有限，或超出 `[-12.56, 12.56] rad`。

上下界相等是有意允许的零宽目标区间。即使关闭该层裁剪，也要求其数值配置有效，防止以后重新打开时带入错误值。

浮点配置比较使用编译器常量折叠的静态断言：C11 / Clang 用 `_Static_assert`，较老的 C 编译器用文件作用域数组检查。GCC/Clang 的相关扩展诊断仅在这些断言附近抑制；错误配置仍会使编译失败。旧 Keil ARMCC 的 fallback 尚未在实际工具链验证。

电机协议本身仍按 `P_MIN=-12.56f`、`P_MAX=12.56f` 做 16 位位置编码，约 `0.0003833 rad` / `0.02196°` 一格。速度、力矩、Kp、Kd 的编码范围也保持原值。关闭软件目标限制后，这个协议可表示范围仍然存在。它不等于实际机械可运动范围；扩协议范围还涉及电机固件/手册，不能仅改本配置文件。

模型运行侧还有自己的动作、速率和角度限制，下位机关闭裁剪不会取消上位机限制。请结合总修改指南里的模型参数一起检查。

## 5. 修改涉及哪些源码

| 文件 | 修改 |
|---|---|
| `Core/Inc/jetson_robot_limits.h` | 新增集中配置、配置有效性断言、纯反馈裁剪函数 |
| `Core/Inc/motor_el05.h` | 原 `SAFE_*` 宏保留名称，改成集中配置的兼容别名 |
| `Core/Src/motor_el05.c` | 原 `get_safe_limit` 增加显式启用开关；四个既有编码入口共用 |
| `Core/Src/jetson_robot_bridge.c` | 反馈裁剪改调用集中配置函数，数学顺序与默认值保持原样 |
| `tests/test_robot_limits_host.py` | 用主机 GCC 编译真实电机编码源码，使用 CAN stub 捕获发送数据 |

本次未调整 PD 增益、倾斜/冻结动作、坐标映射、看门狗、USB 或 CAN 协议。本提交仅包含裁剪配置、调用点和对应测试。

## 6. 验证结果和边界

下位机仓库执行：

```bash
python -m unittest discover -s tests -p test_robot_limits_host.py -v
```

6 项测试通过，包括：

- 默认配置、修改数值、显式关闭两层裁剪三种配置。
- 每种配置检查 12 个电机 ID、正负请求及四个真实电机编码入口，允许一个量化单位误差。
- 正负反馈窗口与窗口内位置、零宽反馈窗口、超协议范围请求。
- 33 组错误配置编译拒绝：负/非有限反馈窗口，以及六类关节的倒置/越界/非有限边界。

实际驱动编译使用 `gcc -std=c11 -Wall -Wextra -Werror -pedantic`。真实 `jetson_robot_bridge.c` 使用原 HAL/USB/CMSIS 头文件的主机 `gcc -fsyntax-only` 也通过；64 位主机与 32 位 MCU 指针宽度的既有 CMSIS 警告已单独识别。

环境没有 `arm-none-eabi-gcc`，因此**没有完成 ARM 固件完整构建、链接、烧录或实车运行验证**。上述结果证明配置分支与编码行为符合预期，不能证明机械执行、CAN 实时性或部署成功。

实际部署时先记录宏值，完整构建并烧录，再对应记录 Nano 发出的目标、电机侧目标及原始反馈。默认数值不会自动扩大关节运动范围；修改值后的实车范围仍需根据实际机械与电机能力确认。
