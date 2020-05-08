# Simple test for NeoPixels on Raspberry Pi
import time
import threading
import socket
from struct import pack, unpack
import struct


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class LedSection:
    LEDS_PER_SECTION = int(4)
    DT = 0.01

    def __init__(self, s_id, strip):

        self.led_strip = strip
        self.section_id = s_id  # 'id' is a built-in fucntion 
        self.execution = 0
        self.max_executions = 2

        self.set_mode('wizzy_clear')
        self.set_state_params('idle')
        
        self.is_active = False

    def set_section_color(self):
        for i in range(LedSection.LEDS_PER_SECTION):
            current_color = self.led_strip[self.section_id * LedSection.LEDS_PER_SECTION + i].color
            current_color.r = self.r
            current_color.g = self.g
            current_color.b = self.b

    def set_section_brightness(self, brightness):
        for i in range(LedSection.LEDS_PER_SECTION):
            current_color = self.led_strip[self.section_id * LedSection.LEDS_PER_SECTION + i].color
            current_color.a = clamp(brightness / 255.0, 0.1, 1)

    def normalize_colors(self):
        # Makes sure highest color is always 254, keeps original r-g-b ratio
        normalize_factor = 254.0 / max(max(max(self.r, self.g), self.b), 1)
        self.r *= normalize_factor
        self.g *= normalize_factor
        self.b *= normalize_factor

    def turn_off(self):
        self.is_active = False
        self.set_section_brightness(0)
        self.set_mode('wizzy_clear')

    def begin_sequence(self):                
        self.set_state_params('attack')
        self.execution = 0
        
        self.current_brightness = 0
        self.last_state_change = time.time()
        self.last_iteration = time.time()
        self.is_active = True

    def loop_sequence(self):
        while True:
            if self.is_active:
                now = time.time()
                state_diff = now-self.last_state_change
                iteration_diff = now-self.last_iteration

                if state_diff > self.ramp_time:  # change to next state
                    self.set_state_params(self.next_state)
                    self.last_state_change = now

                else:   # Iterate next step
                    self.current_brightness += self.jump_value * iteration_diff
                    self.current_brightness = clamp(self.current_brightness, 0, 255)
                    self.set_section_brightness(self.current_brightness)
                    self.last_iteration = now
                
                # Stop after some (2) iterations:
                if self.execution == self.max_executions:
                    self.turn_off()
                
                #print('state_time:', state_diff, 'iter_time:', iteration_diff, 'jump:', self.jump_value, 'bright:', self.current_brightness)

            time.sleep(LedSection.DT)

    def set_state_params(self, state):
        if state == 'attack':
            self.calc_state_params(self.attack, 0, self.attack_level, 'decay')
            self.execution += 1

        elif state == 'decay':
            self.calc_state_params(self.decay, self.attack_level, self.sustain_level, 'sustain')

        elif state == 'sustain':
            self.calc_state_params(self.sustain, self.sustain_level, self.sustain_level, 'release')

        elif state == 'release':
            self.calc_state_params(self.release, self.sustain_level, -1, 'idle')  #-1 to make sure its 0

        elif state == 'idle':
            self.calc_state_params(self.delay, 0, 0, 'attack')            

    def calc_state_params(self, state_time, start_value, end_value, next_state):
        self.ramp_time = state_time
        if self.ramp_time == 0:
            self.ramp_time = 0.01
        self.jump_value = (end_value - start_value) / self.ramp_time
        self.next_state = next_state
        

    def set_mode(self, mode):
        # All states are in capital letters!
        # Can be shortened after values are found.
########################################################################################
        if mode == 'wizzy_clear':
            self.current_mode = 'wizzy_clear'
            self.attack = 0.2
            self.decay = 0.1
            self.sustain = 0.5
            self.release = 0.2
            self.attack_level = 150
            self.sustain_level = 50
            self.r = int(100)
            self.g = int(100)
            self.b = int(100)
            self.delay = 1
########################################################################################

        elif mode == 'wizzy_A':
            self.current_mode = 'wizzy_A'
            self.attack = 0.3
            self.decay = 0.3
            self.sustain = 0.3
            self.release = 0.1
            self.attack_level = 150
            self.sustain_level = 50
            self.r = int(0)
            self.g = int(100)
            self.b = int(50)
            self.delay = 1

        elif mode == 'wizzy_B':
            self.current_mode = 'wizzy_B'
            self.attack = 0.2
            self.decay = 0.2
            self.sustain = 0.2
            self.release = 0.4
            self.attack_level = 150
            self.sustain_level = 50
            self.r = int(255)
            self.g = int(126)
            self.b = int(0)
            self.delay = 1

        elif mode == 'wizzy_C':
            self.current_mode = 'wizzy_C'
            self.attack = 0.1
            self.decay = 0.1
            self.sustain = 0.7
            self.release = 0.1
            self.attack_level = 200
            self.sustain_level = 100
            self.r = int(100)
            self.g = int(0)
            self.b = int(0)
            self.delay = 1
        else:
            pass  # Non - existent mode! 

        self.normalize_colors()
        self.set_section_color()
        

class CommHandler:

    def __init__(self, purpose='OUTGOING'):
        # Format will be:
        # (int num_elements, int[num_elements] indices, char[num_elements] modes)
        self.format = '2B'
        self.ip = '127.0.0.1'
        self.port = 22922
        self.open_socket(purpose)

    def open_socket(self, purpose='OUTGOING'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if purpose == 'INCOMING':
            self.sock.bind((self.ip, self.port))
            self.sock.settimeout(0.01)
            self.sock.setblocking(False)

    def receive_data(self):
        try:
            input_data, address = self.sock.recvfrom(1024)
            parsed_data = struct.unpack(self.format, input_data)
        except:
            parsed_data = None

        return parsed_data

    def send_data(self, mode, idx):
        output_data = struct.pack(self.format, int(ord(mode)), int(idx))
        self.sock.sendto(output_data, (self.ip, self.port))
        

if __name__ == "__main__":
    import board
    import neopixel

    # Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
    # NeoPixels must be connected to D10, D12, D18 or D21 to work.
    pixel_pin = board.D18

    # The number of NeoPixels
    num_pixels = 24

    # The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
    # For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
    ORDER = neopixel.GRB

    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False,
                               pixel_order=ORDER)

    desired_section = 0  # [0-5]
    
    section_list = [LedSection(0, pixels), 
                    LedSection(1, pixels), 
                    LedSection(2, pixels), 
                    LedSection(3, pixels), 
                    LedSection(4, pixels), 
                    LedSection(5, pixels)]  
    section_list[desired_section].set_mode('wizzy_clear')
    section_threads = [threading.Thread(target = sec.loop_sequence, daemon=True) for sec in section_list]
    section_threads[desired_section].start()
   
    now = time.time() - 5
    try:
        while True:
            
            pixels.show()
            if time.time() - now > 5:
                now = time.time()
                section_list[desired_section].begin_sequence()

    except KeyboardInterrupt: # Quit program cleanly
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        time.sleep(1)


    
