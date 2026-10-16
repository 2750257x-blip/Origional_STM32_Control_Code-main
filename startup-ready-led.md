# PA1 上位机启动成功指示

对应上位机：`issaccheng2008/Jetson_orin_nano_code`，基于 `47d7cc2fabb0aa49df72e5faf75f1177e10d7ed6` 的 PA1 启动指示更新。

原协议 v2、类型 5 `STARTUP_CONTROL`、单字节负载保持不变，新增 `STARTUP_HOST_READY = 1 << 2`。收到此位后执行 `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)`，表示 vision 成功取得相机画面并确认 connector 已运行。PA1 保持亮到 MCU 复位，GPIO 初始化默认 SET（灭）。这是本次启动成功记录，不是持续健康监测。

`flags=0` 仍只复位按钮/首卡状态；串口交给策略时不会熄灭 PA1。上位机交接包清除 ARM 时保留已确认的 HOST_READY；如果 PC2 按得早于 connector 回复，则由 vision 启动的策略接管串口后、收到 connector 的原 UDP 数据时补发一次 HOST_READY。灯控不增加 PC2 起步条件。`STARTUP_ARM`、`STARTUP_CARD_READY`、PC2、PA3、PA2、运动 COMMAND 和电机看门狗逻辑不变。新增位不使能电机、不刷新运动看门狗。其余未知位仍拒绝。

需同时部署配套的上位机 connector 和 vision，再烧录本分支固件。板上核对：复位后 PA1 灭；两程序启动后、尚未按 PC2 时 PA1 亮；首卡灯 PA3 和按钮启动正常；交接策略后 PA1 保持亮。
