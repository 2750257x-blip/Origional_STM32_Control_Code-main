"""Compile the real motor encoder with a host CAN stub; no robot/HAL required."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#ifndef __FDCAN_H__
#define __FDCAN_H__
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } FDCAN_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
uint8_t FDCAN_Send_ExtFrame(FDCAN_HandleTypeDef *, uint32_t, uint8_t *, uint8_t);
#endif
'''
CHECK = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_el05.h"
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static uint8_t sent[8];
static uint32_t sent_id;
uint8_t FDCAN_Send_ExtFrame(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data, uint8_t len)
{
    (void)bus;
    assert(len == 8);
    memcpy(sent, data, 8);
    sent_id = id;
    return 0;
}
static uint16_t encode(float p)
{
    if (p > P_MAX) p = P_MAX;
    if (p < P_MIN) p = P_MIN;
    return (uint16_t)((p - P_MIN) * 65535.0f / (P_MAX - P_MIN));
}
static void check_position(float expected)
{
    uint16_t actual = (uint16_t)((sent[0] << 8) | sent[1]);
    float decoded = (float)actual * (P_MAX - P_MIN) / 65535.0f + P_MIN;
    assert(fabsf(decoded - expected) <= (P_MAX - P_MIN) / 65535.0f + 0.00001f);
}
int main(void)
{
    /* Check every motor ID and all four control entry points against known bounds. */
    const uint8_t ids[12] = {1,2,3,4,5,6,0x11,0x12,0x13,0x14,0x15,0x16};
    const float bounds[6] = {2.0f, EXPECT_ROLL_LIMIT, 1.5f, 2.0f, 1.5f, 1.5f};
    for (unsigned i = 0; i < 12; ++i) {
        for (int sign = -1; sign <= 1; sign += 2) {
            float request = (float)sign * 2.5f;
            float expected = TARGET_ENABLED ? (float)sign * bounds[i % 6] : request;
            Motor_limitCtrl_float(&hfdcan1, ids[i], request, 0.0f, 60.0f, 4.2f);
            check_position(expected);
            assert((sent_id & 255U) == ids[i]);
            EL05_Motor_Ctrl(&hfdcan1, ids[i], 0.0f, request, 0.0f, 60.0f, 4.2f);
            check_position(expected);
            uint16_t encoded = encode(request);
            uint8_t legacy10[10] = {0x7f,0xff, (uint8_t)(encoded >> 8), (uint8_t)encoded,
                                   0x7f,0xff, 0,0, 0,0};
            uint8_t legacy4[4] = {(uint8_t)(encoded >> 8), (uint8_t)encoded, 0x7f,0xff};
            Motor_Ctrl(&hfdcan1, ids[i], legacy10);
            check_position(expected);
            Motor_limitCtrl(&hfdcan1, ids[i], 60.0f, 4.2f, legacy4);
            check_position(expected);
        }
    }
    float q = JetsonLimits_ClipFeedbackPosition(1.2f, 0.0f);
    assert(fabsf(q - (FEEDBACK_ENABLED ? EXPECT_FEEDBACK_LIMIT : 1.2f)) < 0.00001f);
    q = JetsonLimits_ClipFeedbackPosition(-1.2f, 0.0f);
    assert(fabsf(q - (FEEDBACK_ENABLED ? -EXPECT_FEEDBACK_LIMIT : -1.2f)) < 0.00001f);
    assert(fabsf(JetsonLimits_ClipFeedbackPosition(0.2f, 0.1f) - 0.2f) < 0.00001f);
    /* Disabling the software target clip does not widen the protocol range. */
    Motor_limitCtrl_float(&hfdcan1, 0x77, 100.0f, 0.0f, 60.0f, 4.2f);
    check_position(12.56f);
    Motor_limitCtrl_float(&hfdcan1, 0x77, -100.0f, 0.0f, 60.0f, 4.2f);
    check_position(-12.56f);
    puts("all 12 motor IDs, all 4 encoders, feedback clipping, protocol bounds: OK");
    return 0;
}
'''


@unittest.skipUnless(shutil.which("gcc"), "host gcc is required")
class RobotLimitsHostTests(unittest.TestCase):
    def run_config(self, defines, expected_roll=.8, expected_feedback=.5,
                   target_enabled=1, feedback_enabled=1):
        with tempfile.TemporaryDirectory(prefix="jetson-limits-") as folder:
            folder = Path(folder)
            (folder / "hal_stub.h").write_text(STUB)
            (folder / "check.c").write_text(CHECK)
            command = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                       "-include", str(folder / "hal_stub.h"), "-I", str(ROOT / "Core/Inc"),
                       f"-DEXPECT_ROLL_LIMIT={expected_roll}f",
                       f"-DEXPECT_FEEDBACK_LIMIT={expected_feedback}f",
                       f"-DTARGET_ENABLED={target_enabled}",
                       f"-DFEEDBACK_ENABLED={feedback_enabled}",
                       *[f"-D{item}" for item in defines],
                       str(ROOT / "Core/Src/motor_el05.c"), str(folder / "check.c"),
                       "-lm", "-o", str(folder / "check")]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(folder / "check")], check=True,
                                    capture_output=True, text=True)
            self.assertIn(": OK", result.stdout)

    def test_unchanged_default_motor_and_feedback_limits(self):
        self.run_config([])

    def test_numeric_limits_can_be_overridden_at_compile_time(self):
        self.run_config(["JETSON_LIMIT_LEG_ROLL_MIN_RAD=-1.1f",
                         "JETSON_LIMIT_LEG_ROLL_MAX_RAD=1.1f",
                         "JETSON_FEEDBACK_MAX_ERROR_RAD=.8f"],
                        expected_roll=1.1, expected_feedback=.8)

    def test_software_clips_can_be_explicitly_disabled(self):
        self.run_config(["JETSON_MOTOR_TARGET_CLIP_ENABLED=0",
                         "JETSON_FEEDBACK_ERROR_CLIP_ENABLED=0"],
                        target_enabled=0, feedback_enabled=0)

    def compile_header(self, defines, source="int main(void) { return 0; }", run=False):
        with tempfile.TemporaryDirectory(prefix="jetson-limits-header-") as folder:
            folder = Path(folder)
            (folder / "check.c").write_text(
                '#include "jetson_robot_limits.h"\n' + source)
            result = subprocess.run(
                ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                 "-I", str(ROOT / "Core/Inc"), *[f"-D{d}" for d in defines],
                 str(folder / "check.c"), "-o", str(folder / "check")],
                capture_output=True, text=True)
            if run:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(folder / "check")], check=True)
            return result

    def test_invalid_feedback_windows_are_rejected_at_compile_time(self):
        for value in ("-0.1f", "(0.0f/0.0f)", "(1.0f/0.0f)"):
            with self.subTest(value=value):
                result = self.compile_header([f"JETSON_FEEDBACK_MAX_ERROR_RAD={value}"])
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("jetson_robot_limits.h", result.stderr)
                self.assertIn("error:", result.stderr)

    def test_all_axis_types_reject_bad_bounds_even_if_clipping_is_disabled(self):
        for kind in ("LEG_PITCH", "LEG_ROLL", "LEG_YAW", "KNEE", "ANKLE_PITCH", "ANKLE_ROLL"):
            for suffix, value in (("MIN_RAD", "3.0f"), ("MAX_RAD", "13.0f"),
                                  ("MIN_RAD", "-13.0f"), ("MAX_RAD", "(0.0f/0.0f)"),
                                  ("MAX_RAD", "(1.0f/0.0f)")):
                with self.subTest(kind=kind, suffix=suffix, value=value):
                    result = self.compile_header([
                        "JETSON_MOTOR_TARGET_CLIP_ENABLED=0",
                        f"JETSON_LIMIT_{kind}_{suffix}={value}"])
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("jetson_robot_limits.h", result.stderr)
                    self.assertIn("error:", result.stderr)

    def test_zero_feedback_width_is_not_disable(self):
        self.compile_header(["JETSON_FEEDBACK_MAX_ERROR_RAD=0.0f"],
                            "int main(void) { return "
                            "JetsonLimits_ClipFeedbackPosition(1.2f, 0.3f) != 0.3f; }",
                            run=True)


if __name__ == "__main__":
    unittest.main()
