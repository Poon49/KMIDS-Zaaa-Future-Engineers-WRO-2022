import sensor, image, time, math, pyb, random
from pyb import Servo, Pin, Timer, millis, UART
debug = False
debug_wall = False
debug_block = False
debug_line = False
uart = UART(3, 115200)
blue_led = pyb.LED(3)
red_led = pyb.LED(1)
led_front = Pin('P6')
tim = Timer(2, freq=1000)
ch = tim.channel(1, Timer.PWM, pin=led_front)
ch.pulse_width_percent(0)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False, 27)
sensor.set_auto_exposure(False, 20000)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time = 2000)
green_block = [(82, 97, -81, -47, 72, 90),
                (38, 63, -57, -32, 27, 55),
                (32, 41, -36, -24, 18, 29),
                (87, 95, -76, -40, 60, 83),
                (31, 44, -40, -19, 12, 38)]
red_block = [(52, 58, 50, 81, 46, 66),
                (25, 34, 8, 31, 8, 24),
                (50, 77, 32, 64, 27, 55),
                (36, 41, 37, 52, 20, 36),
                (56, 62, 54, 71, 31, 57),
                (32, 39, 31, 45, 21, 31)]
black_wall = [ (18, 21, -3, 9, -6, 6),
                (6, 19, -17, -5, 2, 30),
                (19, 30, -23, 6, -8, 15),
                (14, 20, -37, -21, 10, 27)]
orange_line = [(53, 65, 43, 72, 32, 58),
                (50, 75, 24, 46, 14, 55)]
blue_line = [(13, 40, -37, 18, -31, -7)]
side = 10
roi_green = (0,30,160-side,100)
roi_red = (side,30,160-side,100)
roi_left_wall = (0, 40, 80, 120-40)
roi_right_wall = (80, 40, 80, 120-40)
roi_line = (0, 50, 160, 70)
roi_front_wall = (60, 90, 30, 30)
green = 0
red = 0
turn = 0
front_wall = 0
first_see = True
clockwise = 0
lastTime = millis()
greenCy1 = 0
redCy1 = 0
greenCy2 = 0
redCy2 = 0
def sentUART(g, r, t, c, f, w):
    sub = "#W" +str(g)+ "," + str(r) + "," + str(t) +"," + str(c) + "," + str(f) + "," + str(w) + ",0*"
    uart.write(sub.encode())
while(True):
    if (uart.any()):
        if(uart.read() == b'#'):
            green = 0
            red = 0
            turn = 0
            front_wall = 0
            first_see = True
            clockwise = 0
            red_led.on()
            time.sleep_ms(100)
            red_led.off()
    img = sensor.snapshot()
    img2 = img
    img2 = img2.gamma_corr(gamma = 0.3, contrast = 4.0, brightness = 0.2)
    img2 = img2.median(1, percentile=0.7)
    for blob in img2.find_blobs(green_block, pixels_threshold=20, area_threshold=20, roi = roi_green):
        ratio = blob.w()/blob.h()
        density = blob.density()
        if (ratio > 0.3 and ratio < 1.0) and density > 0.3 and blob.w()<60:
            green = 1
            greenCy1 = blob.cy()
            if(blob.code()>1):
                greenCy2 = blob.cy()
            if debug_block:
                img.draw_rectangle(blob.rect(), color = (0, 0, 255))
                img.draw_string(blob.x(), blob.y(), "green", color = (0, 0, 255), scale = 2, mono_space = False,
                            char_rotation = 0, char_hmirror = False, char_vflip = False,
                            string_rotation = 0, string_hmirror = False, string_vflip = False)
    for blob in img2.find_blobs(red_block, pixels_threshold=20, area_threshold=20, roi = roi_red):
        ratio = blob.w()/blob.h()
        density = blob.density()
        print("code = ", blob.code(), " density = ",blob.density(), "  elogation = ", blob.elongation(), " area = ",(blob.area()))
        if (ratio > 0.3 and ratio < 1) and density > 0.2 and blob.w()<60 and blob.elongation()<0.6:
            red = 1
            redCy1 = blob.cy()
            if(blob.code()>1):
                redCy2 = blob.cy()
            if debug_block:
                img.draw_rectangle(blob.rect(), color = (0, 255, 0))
                img.draw_string(blob.x(), blob.y(), "red", color = (0, 255, 0), scale = 1, mono_space = False,
                            char_rotation = 0, char_hmirror = False, char_vflip = False,
                            string_rotation = 0, string_hmirror = False, string_vflip = False)
    left_black_area = 0
    for blob in img2.find_blobs(black_wall, pixels_threshold=10, area_threshold=10, roi = roi_left_wall):
    	if blob.elongation() > 0.6 :           
		left_black_area = blob.area() * blob.density()
      
    for blob in img2.find_blobs(black_wall, pixels_threshold=10, area_threshold=10, roi = roi_right_wall):
    	if blob.elongation() > 0.6 :           
    		right_black_area = blob.area() * blob.density()

    walls_error = int(left_black_area - right_black_area)
    for blob in img2.find_blobs(orange_line, pixels_threshold=40, area_threshold=40, roi = roi_line):
                if blob.elongation() > 0.6 :
                    turn = 1
                    if first_see:
                        first_see = False
                        clockwise = 1
                        print("orange First")
                    img.draw_string(blob.x(), blob.y(), "orange", color = (0, 255, 255), scale = 2, mono_space = False,
                            char_rotation = 0, char_hmirror = False, char_vflip = False,
                            string_rotation = 0, string_hmirror = False, string_vflip = False)
                    img.draw_rectangle(blob.rect(), color = (0, 255, 255))
    for blob in img2.find_blobs(blue_line, pixels_threshold=30, area_threshold=30, roi = roi_line):
                if blob.elongation() > 0.6:
                    turn = 1
                    if first_see:
                        first_see = False
                        clockwise = 2
                        print("blue First")
                    img.draw_string(blob.x(), blob.y(), "blue", color = (0, 255, 255), scale = 2, mono_space = False,
                            char_rotation = 0, char_hmirror = False, char_vflip = False,
                            string_rotation = 0, string_hmirror = False, string_vflip = False)
                    img.draw_rectangle(blob.rect(), color = (0, 255, 255))
    for blob in img2.find_blobs(black_wall, pixels_threshold=10, area_threshold=10, roi = roi_front_wall):
                if blob.density()*blob.area() > 100:
                    front_wall = 1
                    if debug_wall: img.draw_rectangle(blob.rect(), color = (100, 200, 55))
    if redCy1 > greenCy1:
        green = 0
    elif redCy1 < greenCy1:
        red = 0
    if ((millis() - lastTime) > 80):
        sentUART(green, red, turn, clockwise, front_wall,walls_error)
        turn = 0
        green = 0
        red = 0
        front_wall = 0
        greenCy1 = 0
        redCy1 = 0
        greenCy2 = 0
        redCy2 = 0
        lastTime = millis()
        blue_led.toggle()
    if debug:
        img.draw_rectangle(roi_front_wall, color = (200, 100, 50), thickness = 1, fill = False)
