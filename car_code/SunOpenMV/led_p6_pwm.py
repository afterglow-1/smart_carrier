import sensor, image, time, pyb
from pyb import Pin, Timer,LED
# 50kHz pin6 timer2 channel1
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
led = pyb.LED(3) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

while (True):

    for i in range(100):
        light.pulse_width_percent(i) # 控制亮度 0~100
        img = sensor.snapshot()         # Take a picture and return the image.
        led.on()            #亮灯
        time.sleep_ms(1000)
        print(i)
