# Simple test for NeoPixels on Raspberry Pi
import time
import threading
import pigpio
import board
import neopixel
from vibration_motors import VibrationMotor
from led_sections import LedSection

def led_heartbeat(led_strip):
    for current_color in range(3):
        led_color = [0, 0, 0]
        led_color[current_color] = 100
        for current_led in range(24):
            led_strip[current_led] = led_color
            if current_led == 0:
                previous_led = 23
            else:
                previous_led = current_led-1
            led_strip[previous_led] = [0, 0, 0]            
            led_strip.show()
            time.sleep(0.03)

    for current_led in range(24):
        led_strip[current_led] = [0, 0, 0]
        led_strip.show() 
             

if __name__ == "__main__":
    # LED inits
    pixel_pin = board.D18
    num_pixels = 24
    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False,
                               pixel_order=neopixel.GRB)
    #led_heartbeat(pixels);
    desired_section = 0  # [0-5]    
    section_list = [LedSection(0, pixels), 
                    LedSection(1, pixels), 
                    LedSection(2, pixels), 
                    LedSection(3, pixels), 
                    LedSection(4, pixels), 
                    LedSection(5, pixels)]  
    section_list[desired_section].set_mode('wizzy_clear')
    section_threads = [threading.Thread(target = sec.loop_sequence) for sec in section_list]
    for current_thread in section_threads:
        current_thread.daemon=True
    section_threads[desired_section].start()
    
    # Motor inits:
    gpio = pigpio.pi()
    desired_motor = 0  # [0-5]    
    motor_list = [VibrationMotor(1, 21, gpio)]
    motor_list[desired_motor].set_mode('wizzy_clear')
    motor_threads = [threading.Thread(target = mot.loop_sequence) for mot in motor_list] 
    for current_thread in motor_threads:
        current_thread.daemon=True
    motor_threads[desired_motor].start()
       
    now = time.time() - 7
    try:
        while True:
            pixels.show()
            if time.time() - now > 7:
                now = time.time()
                motor_list[desired_motor].begin_sequence()
                section_list[desired_section].begin_sequence()

    finally: # Quit program cleanly
        for motor in motor_list:        
            motor.turn_off()        
        for section in section_list:        
            section.turn_off()        
        pixels.show()
        time.sleep(1)

