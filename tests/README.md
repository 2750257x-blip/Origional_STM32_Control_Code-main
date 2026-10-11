# Firmware motion behavior tests

The tests compile the current `Action_Goto` function extracted unchanged from
`Core/Src/main.c` and the real `Core/Src/action_goto.c` into ARM Thumb code, then
execute it in Unicorn. The wrapper fixture stubs only hardware calls and does
not generate timer interrupts. A finite instruction budget therefore reproduces
the original blocking wait instead of relying on source-text checks.

Requirements: Python 3, `unicorn`, `pyelftools`, and `arm-none-eabi-gcc`.
The default compiler is the STM32CubeCLT installation at
`F:\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe`.
Set the `ARM_GCC` environment variable to use another installation.

From the firmware repository, run:

```powershell
python -m pip install --target ..\.diagnostics\firmware-test-deps unicorn pyelftools
python tests/run_arm_tests.py
```

Installed packages in `../.diagnostics/firmware-test-deps` are loaded
automatically; normally installed packages work too. Generated C and ELF files
are written only under the ignored `build/behavior-tests/` directory.

`python tests/run_arm_tests.py --wrapper-only` runs the nonblocking regression
alone. On the original firmware it failed with `test_wrapper did not return
within 200,000 ARM instructions; the main loop remains blocked without a timer
interrupt` (observed before the implementation changed).

The portable engine cases check 20 ms output pacing, 500/1000 ms durations,
catch-up after delayed service, bit-exact endpoints, zero-step moves,
cancellation, restarting from the current sample, and unsigned tick rollover.
They execute production C; Python supplies inputs and checks outputs. These
tests do not simulate physical motors or validate FDCAN/USB hardware behavior.

The integration fixture also extracts the real robot state machine, local leg
sequence, action request handling, communication service, and host command
ownership guard. It supplies hardware/protocol input stubs and observes motor
submissions, state sends, and action statuses. Both leg actions run through their
entire 5.8 second timeline under ARM emulation. Tests cover the three second lift
hold, exact GPIOC13 standing return, continuous 20 ms state sends and motor
holding batches, duplicate event IDs, BUSY responses, watchdog abort, ESTOP
processing, and delayed USB `DONE` with the subsequent ownership grace.
Offline local actions are also exercised when a host connects and then drops,
when ESTOP arrives, and when an invalid command arrives. The three offline
safety cases were mutation-checked by reverting the affected guards in an
ignored generated C copy: all three failed with those guards removed and passed
against the current firmware.

The harness targets Cortex-M3 Thumb with software floating point to keep the
emulator independent of the target MCU's FPU setup. It checks C behavior and
bounded return, rather than physical execution time or peripheral interrupt
delivery. HAL reset is an observation stub, so ESTOP cases assert motor-stop
submissions and a reset request; actual hardware reset requires a board check.
