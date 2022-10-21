# Support for a manual controlled stepper
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Edit By Xeno for automated stepper in Bambu-like Filament Buffer system.

import stepper, chelper, logging, toolhead
from . import force_move

MIN_KIN_TIME = 0.100
SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c

class BufferStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        stepper_name = config.get_name().split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        if config.get('endstop_pin', None) is None:
            raise self.printer.config_error("Must set 'endstop_pin' for buffer_stepper '%s'" % (stepper_name,))
        for n, mcu in self.printer.lookup_objects(module='mcu'):
            if mcu.get_name() == 'EBB':
                self.mcu = mcu
        if self.mcu == None:
            raise self.printer.config_error("Must set sperate MCU for buffer_stepper")
        self.debug = config.get('debug', False)
        self.reactor = self.printer.get_reactor()
        self.kin_flush_delay = SDS_CHECK_TIME
        self.last_kin_flush_time = self.last_kin_move_time = 0.
        self.buffer_time_start = config.getfloat('buffer_time_start', 0.250, above=0.)
            
        buttons = self.printer.load_object(config, 'buttons')
        self.endstop = config.get('endstop_pin')
        buttons.register_buttons([self.endstop], self._endstop_handler)
        self.endstop_triggered = False
        self.event_delay = config.getfloat('event_delay', 3., above=0.)
        self.min_event_systime = self.reactor.NEVER
        self.rail = stepper.PrinterStepper(config)
        
        
        self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.push_length = config.getfloat('push_length', 15., minval=1.)
        self.next_cmd_time = 0.
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.rail.set_stepper_kinematics(self.stepper_kinematics)
        self.rail.set_trapq(self.trapq)
        # Register commands
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command('BUFFER_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_BUFFER_STEPPER,
                                   desc=self.cmd_BUFFER_STEPPER_help)
    def _endstop_handler(self, eventtime, state):
        logging.info("endstop state changed. new state: %s", state)
        if state == self.endstop_triggered:
            return
        self.endstop_triggered = state
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime:
            return
        if self.endstop_triggered:
            if self.debug:
                self.debug_logging("buffer triggered!")
            self.do_move(self.push_length, self.velocity,self.accel)
    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 0.5
    def sync_print_time(self):
        self._calc_print_time()
    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        kin_time = max(est_print_time + MIN_KIN_TIME, self.last_kin_flush_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + self.buffer_time_start, kin_time)
        if min_print_time > self.next_cmd_time:
            self.next_cmd_time = min_print_time
    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        if enable:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_enable(self.next_cmd_time)
        else:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_disable(self.next_cmd_time)
        self.sync_print_time()
    def do_set_position(self, setpos):
        self.rail.set_position([setpos, 0., 0.])
    def do_move(self, movepos, speed, accel, sync=True):
        self.debug_logging("start pushing buffer!")
        self.sync_print_time()
        self.rail.set_position((0., 0., 0.))
        dist = movepos
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            dist, speed, accel)
        self.trapq_append(self.trapq, self.next_cmd_time, accel_t, cruise_t, accel_t,
                          0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
        self.next_cmd_time = self.next_cmd_time + accel_t + cruise_t + accel_t
        self.rail.generate_steps(self.next_cmd_time)
        self.trapq_finalize_moves(self.trapq, self.next_cmd_time + 99999.9)
        #froce mcu to flush moving command
        self.mcu.flush_moves(self.next_cmd_time)
        self.debug_logging("buffer pushed!")
    cmd_BUFFER_STEPPER_help = "Command a manually configured stepper"
    def cmd_BUFFER_STEPPER(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.do_enable(enable)
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.do_set_position(setpos)
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        if gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            sync = gcmd.get_int('SYNC', 1)
            self.do_move(movepos, speed, accel, sync)
    def get_position(self):
        return [self.rail.get_commanded_position(), 0., 0., 0.]
    def set_position(self, newpos, homing_axes=()):
        self.do_set_position(newpos[0])
    def get_last_move_time(self):
        self._calc_print_time()
        return self.next_cmd_time
    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)
    def drip_move(self, newpos, speed, drip_completion):
        self.do_move(newpos[0], speed, self.homing_accel)
    def get_kinematics(self):
        return self
    def get_steppers(self):
        return self.steppers
    def calc_position(self, stepper_positions):
        return [stepper_positions[self.rail.get_name()], 0., 0.]
    def debug_logging(self, message):
        self.gcode.respond_info(message)

def load_config_prefix(config):
    return BufferStepper(config)