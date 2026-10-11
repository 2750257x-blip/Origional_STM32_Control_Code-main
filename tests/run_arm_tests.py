"""Compile production C for ARM and exercise its behavior in Unicorn."""
import argparse
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
LOCAL_DEPS = ROOT.parent / '.diagnostics' / 'firmware-test-deps'
if LOCAL_DEPS.is_dir():
    sys.path.insert(0, str(LOCAL_DEPS))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR)

BUILD = ROOT / 'build' / 'behavior-tests'
COMPILER = os.environ.get('ARM_GCC',
    r'F:\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe')


def extract_function(source, name):
    match = re.search(r'(?m)^(?:static\s+)?(?:void|uint\d+_t)\s+' +
                      re.escape(name) + r'\s*\([^;]*?\)\s*\{', source)
    if not match:
        raise RuntimeError(f'Function definition not found: {name}')
    start = match.start()
    brace = source.index('{', match.start())
    depth = 1
    for end in range(brace + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if depth == 0:
            return source[start:end + 1]
    raise RuntimeError(f'Unbalanced function: {name}')


def extract_region(source, name):
    return source.split('/* USER CODE BEGIN ' + name + ' */', 1)[1].split(
        '/* USER CODE END ' + name + ' */', 1)[0]


def integration_source():
    main = (ROOT / 'Core' / 'Src' / 'main.c').read_text(encoding='utf-8-sig')
    bridge = (ROOT / 'Core' / 'Src' / 'jetson_robot_bridge.c').read_text(encoding='utf-8-sig')
    protocol = (ROOT / 'Core' / 'Inc' / 'jetson_protocol.h').read_text(encoding='utf-8-sig')
    protocol = protocol.replace('#include "stm32h7xx_hal.h"', '')
    motor = (ROOT / 'Core' / 'Inc' / 'motor_el05.h').read_text(encoding='utf-8-sig')
    motor = '\n'.join(line for line in motor.splitlines()
                      if re.match(r'#define\s+(?:[rl]_(?:leg|knee|ankle)_\w+|P_M(?:IN|AX))\s', line))
    names = ['Action_Step_Begin', 'Action_Step_Elapsed', 'Action_Goto',
             'Action_Goto_Pose', 'Action_Goto_Cancel', 'Action_Goto_Service',
             'Snapshot_Leg_Control', 'ROBOT_LEG', 'Robot_State_Machine',
             'Queue_Action_Status', 'Handle_Action_Request', 'ROBOT_Comms_Service']
    functions = extract_function(bridge, 'JetsonRobotBridge_ProcessCommand') + '\n\n'
    functions += '\n\n'.join(extract_function(main, name) for name in names)
    fixture = (ROOT / 'tests' / 'integration_fixture.c').read_text()
    for marker, code in [('PROTOCOL_HEADER', protocol), ('ROBOT_ENUM', extract_region(main, 'PTD')),
                         ('MOTOR_DEFINES', motor), ('MAIN_GLOBALS', extract_region(main, 'PV')),
                         ('PRODUCTION_FUNCTIONS', functions)]:
        fixture = fixture.replace('/* INSERT_' + marker + ' */', code)
    return fixture


def compile_elf(source, name, engine=False):
    BUILD.mkdir(parents=True, exist_ok=True)
    generated = BUILD / (name + '.c')
    generated.write_text(source, encoding='utf-8')
    elf = BUILD / (name + '.elf')
    command = [COMPILER, '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft',
               '-O1', '-ffreestanding', '-fno-builtin', '-nostdlib',
               '-I', str(ROOT / 'Core' / 'Inc'), '-Wl,--build-id=none',
               '-Wl,-e,' + ('test_begin' if name == 'engine' else 'test_wrapper'),
               '-T', str(ROOT / 'tests' / 'arm_test.ld'), str(generated)]
    if engine:
        command += ['-DHAVE_ACTION_GOTO', str(ROOT / 'Core' / 'Src' / 'action_goto.c')]
    command += ['-lgcc', '-o', str(elf)]
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode:
        raise RuntimeError('ARM compilation failed:\n' + completed.stdout + completed.stderr)
    return elf


class ArmProgram:
    def __init__(self, path):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        self.cpu.mem_map(0x10000, 0x100000)
        self.cpu.mem_map(0x20000000, 0x10000)
        self.return_address = 0x90000
        with path.open('rb') as stream:
            elf = ELFFile(stream)
            for segment in elf.iter_segments():
                if segment['p_type'] == 'PT_LOAD':
                    self.cpu.mem_write(segment['p_vaddr'], segment.data())
            table = elf.get_section_by_name('.symtab')
            self.symbols = {symbol.name: symbol['st_value']
                            for symbol in table.iter_symbols()}
        self.returned = False
        self.cpu.hook_add(UC_HOOK_CODE, self._instruction)

    def _instruction(self, cpu, address, size, data):
        if address == self.return_address:
            self.returned = True
            cpu.emu_stop()

    def call(self, symbol, *args, budget=200000):
        self.returned = False
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2000fff0)
        self.cpu.reg_write(UC_ARM_REG_LR, self.return_address | 1)
        registers = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
        for register, arg in zip(registers, args):
            self.cpu.reg_write(register, arg & 0xffffffff)
        self.cpu.emu_start(self.symbols[symbol] | 1, self.return_address + 2,
                           count=budget)
        if not self.returned:
            raise AssertionError(f'{symbol} did not return within {budget:,} ARM instructions; '
                                 'the main loop remains blocked without a timer interrupt')
        return self.cpu.reg_read(UC_ARM_REG_R0)


class WrapperTests(unittest.TestCase):
    def test_action_goto_returns_without_timer_interrupt(self):
        source = (ROOT / 'Core' / 'Src' / 'main.c').read_text(encoding='utf-8-sig')
        fixture = (ROOT / 'tests' / 'wrapper_fixture.c').read_text()
        source = fixture.replace('/* INSERT_ACTION_GOTO */',
                                 extract_function(source, 'Action_Goto'))
        engine = (ROOT / 'Core' / 'Src' / 'action_goto.c').exists()
        self.assertEqual(ArmProgram(compile_elf(source, 'wrapper', engine)).call('test_wrapper'), 1)


class EngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.elf = compile_elf((ROOT / 'tests' / 'engine_fixture.c').read_text(),
                              'engine', engine=True)

    def setUp(self):
        self.program = ArmProgram(self.elf)

    def call(self, name, *args):
        return self.program.call('test_' + name, *args)

    def position_bits(self):
        return [self.call('position', index) for index in range(12)]

    def positions(self):
        return [struct.unpack('<f', struct.pack('<I', bits))[0]
                for bits in self.position_bits()]

    def assert_pose(self, expected):
        for actual, wanted in zip(self.positions(), expected):
            self.assertAlmostEqual(actual, wanted, delta=0.00001)

    def test_begin_returns_and_keeps_current_pose(self):
        self.assertEqual(self.call('begin', 50, 100, 0), 1)
        self.assert_pose([0.0] * 12)
        self.assertEqual(self.call('service', 100), 0)

    def test_only_one_sample_per_20ms(self):
        self.call('begin', 50, 100, 0)
        self.assertEqual(self.call('service', 119), 0)
        self.assertEqual(self.call('service', 120), 1)
        self.assert_pose([(i + 1) * 0.02 for i in range(12)])
        self.assertEqual(self.call('service', 120), 0)
        self.assertEqual(self.call('service', 139), 0)
        self.assertEqual(self.call('service', 140), 1)

    def test_25_steps_finish_at_500ms(self):
        self.call('begin', 25, 1000, 0)
        self.call('service', 1480)
        self.assertEqual(self.call('busy'), 1)
        self.assertEqual(self.call('service', 1500), 1)
        self.assertEqual(self.call('busy'), 0)
        self.assert_pose([float(i + 1) for i in range(12)])

    def test_50_steps_finish_at_1000ms(self):
        self.call('begin', 50, 1000, 0)
        self.call('service', 1980)
        self.assertEqual(self.call('busy'), 1)
        self.assertEqual(self.call('service', 2000), 1)
        self.assertEqual(self.call('busy'), 0)

    def test_missed_ticks_use_elapsed_time(self):
        self.call('begin', 50, 100, 0)
        self.assertEqual(self.call('service', 360), 1)
        self.assert_pose([(i + 1) * 0.26 for i in range(12)])
        self.assertEqual(self.call('service', 1500), 1)
        self.assertEqual(self.call('busy'), 0)

    def test_endpoint_is_bit_exact_and_does_not_drift(self):
        self.call('begin', 50, 100, 1)
        for now_ms in range(120, 1101, 20):
            self.call('service', now_ms)
        target_bits = [self.call('target', index) for index in range(12)]
        self.assertEqual(self.position_bits(), target_bits)
        for now_ms in (1120, 5000, 1000000):
            self.assertEqual(self.call('service', now_ms), 0)
            self.assertEqual(self.position_bits(), target_bits)

    def test_zero_steps_reject_without_division_or_moving(self):
        self.assertEqual(self.call('begin', 0, 100, 1), 0)
        self.assertEqual(self.position_bits(), [self.call('start', i) for i in range(12)])
        self.assertEqual(self.call('service', 100), 0)

    def test_cancel_freezes_pose(self):
        self.call('begin', 50, 100, 0)
        self.call('service', 500)
        stopped_pose = self.position_bits()
        self.assertEqual(self.call('cancel'), 0)
        self.assertEqual(self.call('service', 2000), 0)
        self.assertEqual(self.position_bits(), stopped_pose)

    def test_restart_uses_current_sample_as_start(self):
        self.call('begin', 50, 100, 0)
        self.call('service', 600)
        self.assertEqual(self.call('restart', 25, 600), 1)
        self.assert_pose([(i + 1) * 0.5 for i in range(12)])
        self.call('service', 860)
        self.assert_pose([(i + 1) * 0.24 for i in range(12)])
        self.call('service', 1100)
        self.assertEqual(self.call('busy'), 0)
        self.assert_pose([0.0] * 12)

    def test_uint32_tick_wraparound(self):
        started = 0xfffffff0
        self.call('begin', 25, started, 0)
        self.assertEqual(self.call('service', 3), 0)
        self.assertEqual(self.call('service', 4), 1)
        self.assert_pose([(i + 1) * 0.04 for i in range(12)])
        self.assertEqual(self.call('service', (started + 500) & 0xffffffff), 1)
        self.assertEqual(self.call('busy'), 0)
        self.assert_pose([float(i + 1) for i in range(12)])


class IntegrationTests(unittest.TestCase):
    ACCEPTED, DONE, BUSY, INVALID, FAILED = 1, 2, 3, 4, 5
    STAND = [-0.05, -0.25, 0, -0.2, -0.2, -0.25, 0.05, 0.25, 0, 0.2, 0.2, 0.25]
    LEFT = [1.3, 0, 0, -1.6, -0.1, 0, 0, 0.4, 0, 0, 0, 0.5]
    RIGHT = [0, -0.4, 0, 0, 0, -0.5, -1.3, 0, 0, 1.6, 0.1, 0]

    @classmethod
    def setUpClass(cls):
        cls.elf = compile_elf(integration_source(), 'integration', engine=True)

    def setUp(self):
        self.program = ArmProgram(self.elf)
        self.program.call('test_wrapper')
        self.now = 0

    def call(self, name, *args):
        return self.program.call('test_' + name, *args)

    def metric(self, index):
        return self.call('metric', index)

    def advance_to(self, time_ms):
        for tick in range(self.now + 10, time_ms + 1, 10):
            self.call('loop', tick)
        self.now = time_ms

    def begin_action(self, action_id):
        self.assertEqual(self.call('request', 123, action_id), self.ACCEPTED)
        self.call('loop', 0)
        self.assertEqual(self.metric(1), 1)

    def assert_pose(self, expected):
        for index, wanted in enumerate(expected):
            actual = struct.unpack('<f', struct.pack('<I', self.call('joint', index)))[0]
            self.assertAlmostEqual(actual, wanted, delta=0.00001)

    def check_lifecycle(self, action_id, lift_pose):
        self.begin_action(action_id)
        initial_host_applies = self.metric(4)
        self.advance_to(500)
        self.assert_pose(self.STAND)
        self.advance_to(1500)
        self.assert_pose(lift_pose)
        self.assertEqual(self.metric(2), self.ACCEPTED)
        self.advance_to(4490)
        self.assert_pose(lift_pose)
        self.assertEqual(self.metric(10), 0)
        self.advance_to(4500)
        self.assertEqual(self.metric(10), 1)
        self.advance_to(5500)
        self.assert_pose(self.STAND)
        self.assertEqual(self.metric(2), self.ACCEPTED)
        self.advance_to(5790)
        self.assertEqual(self.metric(1), 1)
        self.assertEqual(self.metric(4), initial_host_applies)
        self.assertEqual(self.metric(5), 289)
        self.assertEqual(self.metric(6), 580)
        self.assertEqual(self.metric(7), 289 * 12)
        self.assertEqual(self.metric(15), 289)
        self.advance_to(5800)
        self.assertEqual(self.metric(0), 0)
        self.assertEqual(self.metric(1), 1)
        self.assertEqual(self.metric(12), 1)
        self.assert_pose(self.STAND)
        self.advance_to(5810)
        self.assertEqual(self.metric(2), self.DONE)
        self.assertEqual(self.metric(3), self.DONE)
        self.assertEqual(self.metric(1), 1)
        self.assertEqual(self.metric(4), initial_host_applies)
        self.advance_to(5900)
        self.assertEqual(self.metric(1), 1)
        self.advance_to(5920)  # Release at the next 20 ms motor sample after the grace.
        self.assertEqual(self.metric(1), 0)
        self.advance_to(5930)
        self.assertGreater(self.metric(4), initial_host_applies)

    def test_left_leg_hold_return_and_live_communications(self):
        self.check_lifecycle(3, self.LEFT)

    def test_right_leg_hold_return_and_live_communications(self):
        self.check_lifecycle(4, self.RIGHT)

    def test_repeated_event_id_does_not_restart_and_new_event_is_busy(self):
        self.begin_action(3)
        self.advance_to(1500)
        self.assertEqual(self.call('request', 123, 3), self.ACCEPTED)
        self.assertEqual(self.call('request', 123, 4), self.INVALID)
        self.assertEqual(self.call('request', 456, 4), self.BUSY)
        self.assert_pose(self.LEFT)
        self.assertEqual(self.metric(11), 3)
        self.assertEqual(self.metric(10), 0)

    def test_watchdog_aborts_local_motion_reports_failed_and_stops_motors(self):
        self.begin_action(3)
        self.advance_to(1000)
        before_sends = self.metric(7)
        self.call('fresh', 0)
        self.advance_to(1010)
        self.assertEqual(self.metric(1), 0)
        self.assertEqual(self.metric(10), 0)
        self.assertEqual(self.metric(0), 0)
        self.assertEqual(self.metric(2), self.FAILED)
        self.assertEqual(self.metric(3), self.FAILED)
        self.assertEqual(self.metric(8), 12)
        self.assertEqual(self.metric(7), before_sends)

    def test_host_estop_still_processed_during_local_motion(self):
        self.begin_action(4)
        self.advance_to(500)
        self.call('command_flags', 3)  # ENABLE | ESTOP
        self.advance_to(510)
        self.assertEqual(self.metric(13), 1)

    def test_usb_busy_keeps_local_pose_until_done_sent_and_grace_elapsed(self):
        self.begin_action(3)
        self.advance_to(5790)
        host_applies = self.metric(4)
        self.call('status_busy', 1)
        self.advance_to(6000)
        self.assertEqual(self.metric(2), self.DONE)
        self.assertEqual(self.metric(3), self.ACCEPTED)
        self.assertEqual(self.metric(1), 1)
        self.assertEqual(self.metric(4), host_applies)
        self.assert_pose(self.STAND)
        self.call('status_busy', 0)
        self.advance_to(6010)
        self.assertEqual(self.metric(3), self.DONE)
        self.advance_to(6100)
        self.assertEqual(self.metric(1), 1)
        self.advance_to(6120)
        self.assertEqual(self.metric(1), 0)
        self.advance_to(6130)
        self.assertGreater(self.metric(4), host_applies)

    def test_delayed_loop_sends_one_motor_batch_and_repeated_tick_sends_none(self):
        self.begin_action(3)
        before = self.metric(7)
        self.call('loop', 1000)
        self.assertEqual(self.metric(7) - before, 12)
        self.call('loop', 1000)
        self.assertEqual(self.metric(7) - before, 12)

    def test_watchdog_is_enforced_during_post_done_handoff(self):
        self.begin_action(3)
        self.advance_to(5810)
        self.assertEqual(self.metric(2), self.DONE)
        self.assertEqual(self.metric(1), 1)
        before_sends = self.metric(7)
        self.call('fresh', 0)
        self.advance_to(5820)
        self.assertEqual(self.metric(1), 0)
        self.assertEqual(self.metric(10), 0)
        self.assertEqual(self.metric(8), 12)
        self.assertEqual(self.metric(7), before_sends)

    def begin_offline_action(self):
        self.call('offline_action', 3)
        self.call('loop', 0)
        self.assertEqual(self.metric(1), 1)
        self.assertEqual(self.metric(16), 0)

    def test_offline_action_requires_watchdog_after_host_connects(self):
        self.begin_offline_action()
        self.advance_to(2000)
        self.assertEqual(self.metric(1), 1)
        self.call('fresh', 1)
        self.advance_to(2010)
        self.assertEqual(self.metric(16), 1)
        self.assertEqual(self.metric(4), 0)
        self.call('fresh', 0)
        before_sends = self.metric(7)
        self.advance_to(2020)
        self.assertEqual(self.metric(1), 0)
        self.assertEqual(self.metric(10), 0)
        self.assertEqual(self.metric(8), 12)
        self.assertEqual(self.metric(7), before_sends)

    def test_offline_action_estop_stops_motors_and_requests_reset(self):
        self.begin_offline_action()
        self.advance_to(1000)
        self.call('fresh', 1)
        self.call('command_flags', 3)
        self.advance_to(1010)
        self.assertEqual(self.metric(13), 1)
        self.assertGreaterEqual(self.metric(8), 12)

    def test_offline_action_invalid_command_cancels_motion(self):
        self.begin_offline_action()
        self.advance_to(1000)
        self.call('fresh', 1)
        self.call('command_valid', 0)
        before_sends = self.metric(7)
        self.advance_to(1010)
        self.assertEqual(self.metric(1), 0)
        self.assertEqual(self.metric(10), 0)
        self.assertGreaterEqual(self.metric(8), 12)
        self.assertEqual(self.metric(7), before_sends)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--wrapper-only', action='store_true')
    arguments = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(WrapperTests)
    if not arguments.wrapper_only:
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(EngineTests))
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(IntegrationTests))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(not result.wasSuccessful())
