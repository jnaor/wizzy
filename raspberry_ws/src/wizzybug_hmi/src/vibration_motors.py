# Simple test for NeoPixels on Raspberry Pi
import time
import threading
from struct import pack, unpack
import RPi.GPIO as gpio

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class VibrationMotor:

    DT = 0.01

    def __init__(self, m_id, pin_num):
        self.motor_id = m_id
        self.pin = pin_num

        gpio.setmode(gpio.BCM)
        gpio.setup(self.pin, gpio.OUT)
        self.gpio = gpio.PWM(self.pin, 400)
        self.gpio.start(0.0)

        self.execution = 0
        self.repetition = 0
        
        self.pulse_num = 0
        self.number_of_pulses = 1
        self.pulses = []
        for pulse in range(self.number_of_pulses):
            self.pulses.append(PulseDefinition())

        self.set_mode('wizzy_clear')
        self.is_active = False
        self.heartbeat()

    def heartbeat(self):
        self.gpio.ChangeDutyCycle(80.0)
        time.sleep(0.2)
        self.gpio.ChangeDutyCycle(0.0)
        time.sleep(0.2)
        self.gpio.ChangeDutyCycle(80.0)
        time.sleep(0.2)
        self.turn_off()

    def turn_off(self):
        self.is_active = False
        self.gpio.ChangeDutyCycle(0.0) 

    def reset_sequence(self):
        #print ('began sequence, motor:', self.motor_id)        
        self.set_state_params('attack')
        self.execution = 0
        self.repetition = 0        
        
        self.current_power = 0
        self.pulse_num = 0
        self.last_state_change = time.time()
        self.last_iteration = time.time()
        self.is_active = True

    def iterate_sequence(self):
        #while True:
        if self.is_active:
            now = time.time()
            state_diff = now-self.last_state_change
            iteration_diff = now-self.last_iteration

            if state_diff > self.ramp_time:  # change to next state
                self.set_state_params(self.next_state)
                self.last_state_change = now

            else:   # Iterate next step
                self.current_power += self.jump_value * iteration_diff
                self.current_power = clamp(self.current_power, 0.0, 100.0)
                self.gpio.ChangeDutyCycle(self.current_power)
                self.last_iteration = now

            if self.repetition == self.pulses[self.pulse_num].repetitions:
                time.sleep(self.pulses[self.pulse_num].sequence_delay)
                now = time.time()
                # Compensate for lost time while sleeping:
                self.last_state_change = now
                self.last_iteration = now
                self.pulse_num += 1
                self.repetition = 0

            if self.pulse_num == self.number_of_pulses:
                self.execution += 1
                self.pulse_num = 0
            
            # Stop after some (2) iterations:
            if self.execution == self.max_executions:
                self.turn_off()          

            #print('state:', state_diff, 'iter:', iteration_diff, 'jump:', self.jump_value, 'power:', self.current_power, 'pulse:', self.pulse_num, 'rep:', self.repetition, 'exec:', self.execution)

            #time.sleep(VibrationMotor.DT)

    def set_state_params(self, state):
        pulse = self.pulses[self.pulse_num]
        if state == 'attack':
            self.calc_state_params(pulse.attack, 0, pulse.power, 'sustain')
            self.repetition += 1

        elif state == 'sustain':
            self.calc_state_params(pulse.sustain, pulse.power, pulse.power, 'release')

        elif state == 'release':
            self.calc_state_params(pulse.release, pulse.power, 0, 'idle')

        elif state == 'idle':
            self.calc_state_params(pulse.pulse_delay, 0, 0, 'attack')            

    def calc_state_params(self, state_time, start_value, end_value, next_state):
        self.ramp_time = state_time
        if self.ramp_time == 0:
            self.ramp_time = 0.01
        self.jump_value = (end_value - start_value) / self.ramp_time
        self.next_state = next_state
           

    def set_mode(self, mode):
        # All states are in capital letters!
        # Can be shortened after values are found.
#############################################################################################
        if mode == 'wizzy_clear':
            self.current_mode = 'wizzy_clear'
    
            # First pulse definition:
            self.pulses[0].power = 1
            self.pulses[0].attack = 0.01
            self.pulses[0].sustain = 0.01
            self.pulses[0].release = 0.01
            self.pulses[0].pulse_delay = 0.1
            self.pulses[0].sequence_delay = 0.1
            self.pulses[0].repetitions = 2

            if self.number_of_pulses > 1:
                self.pulses[1].power = 100
                self.pulses[1].attack = 0.05
                self.pulses[1].sustain = 0.1
                self.pulses[1].release = 0.05
                self.pulses[1].pulse_delay = 0.1
                self.pulses[1].sequence_delay = 0.2
                self.pulses[1].repetitions = 1

            self.max_executions = 1
###########################################################################################          

        elif mode == 'wizzy_A':
            self.current_mode = 'wizzy_A'
            # First pulse definition:
            self.pulses[0].power = 70
            self.pulses[0].attack = 0.070
            self.pulses[0].sustain = 0.100
            self.pulses[0].release = 0.330
            self.pulses[0].pulse_delay = 0.1
            self.pulses[0].sequence_delay = 0.1
            self.pulses[0].repetitions = 1

            # Second pulse definition:
            if self.number_of_pulses > 1:
                self.pulses[1].power = 100
                self.pulses[1].attack = 0.05
                self.pulses[1].sustain = 0.1
                self.pulses[1].release = 0.05
                self.pulses[1].pulse_delay = 0.1
                self.pulses[1].sequence_delay = 0.2
                self.pulses[1].repetitions = 1

            self.max_executions = 1

        elif mode == 'wizzy_B':
            self.current_mode = 'wizzy_B'
            # First pulse definition:
            self.pulses[0].power = 85
            self.pulses[0].attack = 0.050
            self.pulses[0].sustain = 0.200
            self.pulses[0].release = 0.100
            self.pulses[0].pulse_delay = 0.200
            self.pulses[0].sequence_delay = 0.1
            self.pulses[0].repetitions = 2

            # Second pulse definition:
            if self.number_of_pulses > 1:
                self.pulses[1].power = 100
                self.pulses[1].attack = 0.05
                self.pulses[1].sustain = 0.1
                self.pulses[1].release = 0.05
                self.pulses[1].pulse_delay = 0.1
                self.pulses[1].sequence_delay = 0.2
                self.pulses[1].repetitions = 1

            self.max_executions = 1

        elif mode == 'wizzy_C':
            self.current_mode = 'wizzy_C'
            # First pulse definition:
            self.pulses[0].power = 110
            self.pulses[0].attack = 0.050
            self.pulses[0].sustain = 0.150
            self.pulses[0].release = 0.033
            self.pulses[0].pulse_delay = 0.100
            self.pulses[0].sequence_delay = 0.1
            self.pulses[0].repetitions = 5

            # Second pulse definition:
            if self.number_of_pulses > 1:
                self.pulses[1].power = 100
                self.pulses[1].attack = 0.05
                self.pulses[1].sustain = 0.1
                self.pulses[1].release = 0.05
                self.pulses[1].pulse_delay = 0.1
                self.pulses[1].sequence_delay = 0.2
                self.pulses[1].repetitions = 1

            self.max_executions = 1

        else:
            pass  # Non - existent mode!


class PulseDefinition:
    def __init__(self):
        self.power = 100  # full power
        self.attack = 0  # Seconds
        self.sustain = 0  # Seconds
        self.release = 0  # Seconds
        self.pulse_delay = 0
        self.sequence_delay = 0
        self.repetitions = 1

if __name__ == "__main__":

    desired_motor = 0  # [0-5]
    
    motor_list = [VibrationMotor(1, 20)]
    repeating_interval = 5  # Seconds
    
    now = time.time() - repeating_interval

    try:
        for motor in motor_list:
            motor.reset_sequence()
        while True:
            for motor in motor_list:
                motor.iterate_sequence()
            time.sleep(VibrationMotor.DT)
            
            if time.time() - now > repeating_interval:
                now = time.time()
                for motor in motor_list:
                    motor.reset_sequence()

    finally: # Quit program cleanly
        time.sleep(0.1)
        for motor in motor_list:        
            motor.turn_off()  
            motor.gpio.stop()      

        
    
