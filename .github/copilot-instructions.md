# Copilot Instructions for GX_0226

## Project Context
- Project: GX_0226
- MCU: STM32F750V8Tx
- OS: FreeRTOS
- Focus area: chassis motion + motor UART3 DMA communication

## Critical Rule: Verify Current Reality First
- Do not assume previous fixes are still present.
- The code may be rolled back to a trapezoidal acceleration version.
- Hardware state can change between sessions (for example, loose motor cable).
- Before proposing fixes, confirm current active code path and current observed behavior.

## Active Files to Check First
- Task/chassis_control_task.c
- Motor/motor_command.c
- Motor/motor_control.c
- MyNVIC/myNVIC.c
- Task/main_task.c

## UART/Feedback Debug Priority
1. Confirm TX chain:
   - queue enqueue
   - DMA start
   - Tx complete callback
2. Confirm RX chain:
   - RxEvent callback
   - frame cache parse
   - flag_finish bit set
3. Confirm control gate:
   - whether chassis loop requires full 0x0F same-cycle feedback
   - whether feedback bits are cleared too aggressively

## Engineering Style
- Use minimal, reversible edits.
- Prefer instrumentation before speculative protocol changes.
- When behavior is intermittent, add counters first and decide from data.
- Keep safety behavior (stop on real lost feedback), but avoid false stop due to timing jitter.

## Response Style for This Repo
- Output in Chinese.
- Give conclusion first, then evidence, then exact file changes.
- If uncertain, provide a small verification plan with observable counters.

## Existing Skill File
- Use GX_0226_skill.md as detailed project memory and checklist.
- If copilot-instructions and skill file conflict, trust latest user statement and current code state.
