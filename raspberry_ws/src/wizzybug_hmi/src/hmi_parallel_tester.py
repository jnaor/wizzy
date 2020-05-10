# Simple test for NeoPixels on Raspberry Pi
import time
import threading
import pigpio
import board
import neopixel
from vibration_motors import VibrationMotor
from led_sections import LedSection

if __name__ == "__main__":
    # LED inits
    pixel_pin = board.D18
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
    section_list[desired_section].set_mode('wizzy_clear')
    section_threads = [threading.Thread(target = sec.loop_sequence, daemon=True) for sec in section_list]
    section_threads[desired_section].start()
    
    # Motor inits:
    gpio = pigpio.pi()
    desired_motor = 0  # [0-5]    
    motor_list = [VibrationMotor(1, 11, gpio)]  # [25, 8, 7, 11, 9, 10]
    motor_threads = [threading.Thread(target = mot.loop_sequence, daemon=True) for mot in motor_list] 
    motor_list[desired_motor].set_mode('wizzy_clear')
    motor_threads[desired_motor].start() 
    
    now = time.time() - 10
    try:
        while True:
            pixels.show()
            if time.time() - now > 10:
                now = time.time()
                motor_list[desired_motor].begin_sequence()
                section_list[desired_section].begin_sequence()

    except KeyboardInterrupt: # Quit program cleanly
        for motor in motor_list:        
            motor.turn_off()        
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        time.sleep(1)

