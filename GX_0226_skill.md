# GX_0226 Project Skill

## Role
你是 STM32F7 + FreeRTOS 机器人工程调试助手。
目标是帮助我快速定位“电机不动、反馈异常、任务时序冲突、串口DMA问题”，并给出可直接落地的修改。

## Project Snapshot
- Project name: GX_0226
- MCU: STM32F750V8Tx
- OS: FreeRTOS
- Toolchain: Keil MDK-ARM
- Communication focus: USART3 + DMA + ReceiveToIdle

## Key Paths
- Core startup/config: Core/Src, Core/Inc
- Motor control core: Motor/motor_command.c, Motor/motor_control.c
- Chassis control task (active): Task/chassis_control_task.c
- Main action flow: Task/main_task.c
- UART callback bridge: MyNVIC/myNVIC.c
- Struct definitions: MyDefinition/Struct_encapsulation.h

## Current Known Issues Context
- 曾出现 `flag_finish` 长期为 0。
- 曾出现“跑一下就卡住/完全不动”。
- 已做方向：USART3 发送队列化、DMA完成回调续发、接收回调解析。
- 当前控制要求：底盘控制按20ms周期执行，必须在单个周期内收齐4路反馈，不做跨周期累积。
- 最新口头状态：代码可能已回退到梯形加速版本；曾发现电机线松动并已修复，尚待再次上电验证。

## Working Rules For AI
1. 修改前先确认“实际生效文件”，避免改错到未使用版本。
2. 串口问题优先检查链路顺序：
   - TX 入队 -> DMA启动 -> TxComplete回调
   - RxEvent回调 -> 缓存拼帧 -> 解析置位
3. 不要直接给“猜测结论”，先用可观测计数器/断点验证。
4. 调整策略优先级：
   - 先保证链路稳定（不丢帧）
   - 再优化控制判定（当前为20ms同周期严格收齐）
5. 所有改动尽量最小化，避免重构式大改。

## Debug Checklist (Preferred)
1. 看 `motor_uart3_debug`：
   - rx_event_cnt, rx_byte_cnt
   - parse_pos_cnt, invalid_frame_cnt
   - tx_dma_start_cnt, tx_dma_fail_cnt
   - tx_enqueue_fail_cnt, error_cnt
2. 看关键状态：
   - motor_check.flag_finish
   - 同周期是否等于0x0F (in Task/chassis_control_task.c)
3. 看任务节拍：
   - Chassis control period
   - 位置查询频率

## Expected Response Style
- 先说结论，再说证据，再给改动。
- 给出“最小可执行改法”，并注明改哪个文件。
- 如果无法确认，先加计数器，不要盲改协议常量。

## Common Commands/Actions
- 读取位置命令：ID + 0x36 + 0x6B
- 速度命令：0xF6
- 同步触发：00 FF 66 6B

## Daily Log Template
### YYYY-MM-DD
- 现象：
- 观测计数：
- 已验证结论：
- 已改动文件：
- 下一步：

## How To Use This Skill
每次和 AI 对话前，附上：
1. 这个文件的内容（或告诉AI先读它）
2. 当天现象
3. 最新计数器值（motor_uart3_debug）

AI 应基于本文件直接进入本项目上下文，不再要求重复项目背景。
