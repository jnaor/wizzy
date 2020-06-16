# Simple test for NeoPixels on Raspberry Pi
import time
import threading

import board
import neopixel
from vibration_motors import VibrationMotor
from led_sections import LedSection
             
def motor_thread_loop(motor_object):
    local_now = time.time() - 10
    motor_object.reset_sequence()
    while True:
        motor_object.iterate_sequence()
        time.sleep(0.02)

        if time.time() - local_now > 10:
            local_now = time.time()
            motor_object.reset_sequence()

if __name__ == "__main__":
    DT = 0.02

    # LED inits
    pixel_pin = board.D21
    num_pixels = 24
    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False,
                               pixel_order=neopixel.GRB)

    section_list = [LedSection(0, pixels), 
                    LedSection(1, pixels), 
                    LedSection(2, pixels), 
                    LedSection(3, pixels), 
                    LedSection(4, pixels), 
                    LedSection(5, pixels)]  
    
    # Motor inits:
    motor_list = [VibrationMotor(1, 20)]
    motor_threads = [threading.Thread(target = motor_thread_loop, args=[mot]) for mot in motor_list] 
    for current_thread in motor_threads:
        current_thread.daemon=True
       
    now = time.time() - 10
    try:

        for section in section_list:
            section.reset_sequence() 

        motor_threads[0].start()  
        
        while True:
            #for section in section_list:
             #   section.iterate_sequence()
            section_list[0].iterate_sequence()
            pixels.show()
            time.sleep(DT)

            if time.time() - now > 10:
                now = time.time()
                for section in section_list:
                    section.reset_sequence()

    finally: # Quit program cleanly
        time.sleep(0.1)
        for motor in motor_list:        
            motor.turn_off() 
            motor.gpio.stop()       
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        

