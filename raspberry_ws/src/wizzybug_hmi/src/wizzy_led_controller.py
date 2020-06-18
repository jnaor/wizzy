# Simple test for NeoPixels on Raspberry Pi
import time
import threading
from struct import pack, unpack
from led_sections import LedSection, CommHandler
import board
import neopixel

class ActionHandler:

    def __init__(self, sections):
        self.sections = sections
        self.data_pipe = CommHandler(purpose = 'INCOMING')

    def receive_input(self):
        incoming_data = self.data_pipe.receive_data()
        if incoming_data is not None:
            self.activate_leds(incoming_data)

    def activate_leds(self, data):
        idx = data[1]

        # Check for active leds:
        if self.sections[idx].is_active:
            self.sections[idx].turn_off()

        # Making absolute sure that they all turned off:
        time.sleep(LedSection.DT*2)

        # Apply new data:
        mode = chr(data[0])        
        new_mode = self.char_to_mode(mode)
        self.sections[idx].set_mode(new_mode)
        self.sections[idx].reset_sequence()

        #print(new_mode, idx, self.sections[idx].is_active)         

    def char_to_mode(self, char):
        if char == 'O':
            return 'wizzy_clear'
        elif char == 'A':
            return 'wizzy_A'
        elif char == 'B':
            return 'wizzy_B'
        elif char == 'C':
            return 'wizzy_C'


if __name__ == "__main__":
    # Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
    # NeoPixels must be connected to D10, D12, D18 or D21 to work.
    pixel_pin = board.D21

    # The number of NeoPixels
    num_pixels = 24

    # The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
    # For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
    ORDER = neopixel.GRB

    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False,
                               pixel_order=ORDER)

    delta_time = 0.01
    
    section_list = [LedSection(0, pixels), 
                    LedSection(1, pixels), 
                    LedSection(2, pixels), 
                    LedSection(3, pixels), 
                    LedSection(4, pixels), 
                    LedSection(5, pixels)]  

    
    handler = ActionHandler(section_list)
    try:
        while True:
            handler.receive_input()   
            for section in section_list:
                section.iterate_sequence()         
            pixels.show()
            time.sleep(delta_time)

    except KeyboardInterrupt: # Quit program cleanly
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        time.sleep(1)


    
