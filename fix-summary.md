# MCU Control 固件修复摘要

## Issue 1 - 动作函数阻塞问题
所有6个QR动作函数（`Action_RaiseLeftHand` 等）不再使用 `__disable_irq() + HAL_Delay(3000)`，改为设置状态机状态。TIM2回调（500Hz）驱动状态机计时，3秒后自动复位到 `ACTION_IDLE` 并清除 `qrcode_id`。

## Issue 2 - volatile + 边界检查
- `controldata1number` 和 `controldata1ready` 加 `volatile`
- `controldata.h` 新增 `#define TRAJECTORY_FRAME_COUNT 125`
- TIM2回调中轨迹索引超过125时自动回绕到0
- `extern uint8_t count` 改为 `extern volatile uint8_t count`

## Issue 3 - 控制频率 + 竞争条件
- 电机控制从 `while(1)` 移到 `TIM2` 回调，固定500Hz控制率
- USB发送通过中断保护的三步操作：关中断 → `memcpy` 到本地缓冲 `usb_tx_buffer` → 开中断 → `CDC_Transmit_HS` 从本地缓冲发送，消除与CAN接收ISR写`MotorIMU_Packet_t`的竞争条件

## 修改的文件
- **`Core/Inc/controldata.h`** — 新增 `TRAJECTORY_FRAME_COUNT` 定义
- **`Core/Src/main.c`** — volatile、状态机、TIM2回调、动作函数重写、USB保护缓冲
