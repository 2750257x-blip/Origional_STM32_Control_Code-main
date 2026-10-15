# Teaching CAN freeze / PD transition fix

This change addresses two silent-failure paths: STATE frames can continue over
USB while a leg's cached encoder values stop updating; and a full CAN TX FIFO
can discard a new zero-PD command while a previous held-position command remains.

Changes:

- Validate extended feedback type, data-frame format, and eight-byte DLC before
  updating encoder values. Track each joint's last CAN reception tick and count.
- Derive STATE_ENCODERS_VALID from all twelve feedback ages (100 ms maximum),
  rather than the sticky ever-received mask. Continue USB telemetry even when
  feedback is missing, so a host can distinguish USB loss from CAN loss.
- Add bounded TX FIFO waiting and explicit per-bus send-failure counters. Cancel
  obsolete pending frames before a new batch or stop; do not replay stale PD
  requests after recovery.
- Service FDCAN bus-off recovery by cancelling pending requests and clearing
  CCCR.INIT, rate limited to 250 ms per bus. CAN auto-retransmission by itself
  does not recover bus-off. This does not reset the MCU or run its startup pose.
- Report actual fully-enqueued COMMAND sequence and PD category (stopped,
  zero PD, nonzero PD), plus per-bus PSR/ECR/pending/error/recovery diagnostics.

Protocol v2 COMMAND remains 152 bytes. STATE now has a **272-byte payload**:
the old 152-byte prefix followed by `12H 12I 2I 2I 2I 2I 2I I I` (little endian).
These fields are feedback_age_ms, feedback_count, can_psr, can_ecr,
can_tx_failures, can_recoveries, can_pending, applied_command_sequence,
applied_gain_mode. Joint arrays are left six then right six; two-bus arrays
are CAN1/right then CAN2/left. Capability is STATE bit 8; CAN fault is bit 9.
Use the matching updated Jetson protocol decoder before running this firmware.
The PD category confirms submission to all twelve CAN transmit queues; it is
not a readback of the motor controller's internal KP/KD registers.

Validation:

```sh
cmake --preset Debug
cmake --build --preset Debug -j 4
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -ICore/Inc \
    tests/test_leg_can_health.c Core/Src/leg_can_health.c -o test_leg_can
./test_leg_can
```

ARM firmware builds successfully. The production CAN transport module was also
compiled and tested on the Jetson against a mock HAL: per-leg stale feedback,
tick wrap, bounded FIFO retry, send failures, stale request cancellation, and
bus-off recovery all pass. Real-hardware verification after flashing is pending.
The matching Jetson code passed 39 protocol/teaching/PTY tests on the Jetson.

Flashing/reset still executes the original startup pose. Support the robot
before flashing. Initial live evidence was 100 API samples with 4–23 ms USB
age and identical values on all twelve joints; the old firmware had no CAN
ages/PSR/ECR diagnostics, so a specific physical bus-off event was not proven.
