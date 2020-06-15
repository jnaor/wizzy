# Simple test for NeoPixels on Raspberry Pi
import time
import threading
import pigpio
import board
import neopixel
from vibration_motors import VibrationMotor
from led_sections import LedSection
             

if __name__ == "__main__":
    DT = 0.02

    # LED inits
    pixel_pin = board.D21
    num_pixels = 24
    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False,
                               pixel_order=neopixel.GRB)

    desired_section = 0  # [0-5]    
    section_list = [LedSection(0, pixels), 
                    LedSection(1, pixels), 
                    LedSection(2, pixels), 
                    LedSection(3, pixels), 
                    LedSection(4, pixels), 
                    LedSection(5, pixels)]  
    
    # Motor inits:
    gpio = pigpio.pi()
    desired_motor = 0  # [0-5]    
    motor_list = [VibrationMotor(1, 20, gpio)]
       
    now = time.time() - 7
    try:
        print(1)
        for motor in motor_list:
                    motor.reset_sequence()
        print(2)
        for section in section_list:
                    section.reset_sequence()   
        print(3)
        
        while True:
            for motor in motor_list:
                motor.iterate_sequence()
            print(4)
            for section in section_list:
                section.iterate_sequence()
            pixels.show()
            time.sleep(DT)
            print(5)

            if time.time() - now > 7:
                print(6)
                now = time.time()
                for motor in motor_list:
                    motor.reset_sequence()
                for section in section_list:
                    section.reset_sequence()

    finally: # Quit program cleanly
        time.sleep(0.1)
        for motor in motor_list:        
            motor.turn_off()        
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        

