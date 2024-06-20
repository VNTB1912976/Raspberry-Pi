import RPi.GPIO as GPIO
from simple_pid import PID
import time
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont , ImageOps
import board
import digitalio
import adafruit_max31865
import threading

spi = board.SPI()
cs = digitalio.DigitalInOut(board.D4)
sensor = adafruit_max31865.MAX31865(spi, cs, wires=3, rtd_nominal=100.0, ref_resistor=430.0)


# Khởi tạo màn hình OLED
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None)
disp.begin()
disp.clear()
disp.display()

# Thiết lập chân GPIO
clk = 22
dt = 27
button = 17
last_button_state = False
pin_relay1 = 23
pin_relay2 = 24
pin_relay3 = 25
GPIO.setmode(GPIO.BCM)

GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Nút nhấn
GPIO.setup(pin_relay1, GPIO.OUT)
GPIO.setup(pin_relay2, GPIO.OUT)
GPIO.setup(pin_relay3, GPIO.OUT)
# Cài đặt PID
  # Đầu ra là một giá trị boolean (0 hoặc 1)
counter = 0
pid = None
btok = 0
counter1 = 0
counter2 = 0
counter3 = 0
clkLastState = GPIO.input(clk)
start = False
pwmstart = False
timePWM = 0.1
# Load phông chữ với kích thước lớn hơn
font = ImageFont.truetype('/usr/share/fonts/truetype/freefont/FreeSerif.ttf', 18)  # Thay đổi đường dẫn và kích thước phông
# Load và chuẩn bị hình ảnh logo
img_path = '/home/pi/pi_logo.png'
logo = Image.open(img_path).convert("RGBA")
fff = Image.new(logo.mode, logo.size, (255,) * 4)
##########################################################################
def display_logo(angle):
    # Tạo nền trắng
    background = Image.new("1", (disp.width, disp.height), "white")
    
    # Xoay logo
    rot = logo.rotate(angle, resample=Image.BILINEAR)
    img = ImageOps.fit(rot, fff.size, centering=(0.5, 0.5))
    img = Image.composite(rot, fff, rot).convert("1")
    
    # Vị trí paste logo
    posn = ((disp.width - img.width) // 2, (disp.height - img.height) // 2)
    background.paste(img, posn)
    background = background.rotate(180)
    disp.image(background)
    disp.display()
angle = 0
for i in range (17) :
    display_logo(angle)
    angle = (angle + 20) % 360  # Cập nhật góc xoay
    time.sleep(0.001)
display_logo(0)
#########################################################


def update_display(sensor_temp, set_temp1,set_temp2, ok_pressed):
    global  btok , counter3
    disp.clear()
    image = Image.new('1', (disp.width, disp.height))
    draw = ImageDraw.Draw(image)

    # Định dạng giá trị nhiệt độ để làm tròn tới hai chữ số thập phân
    formatted_sensor_temp = "{:.2f} ℃".format(sensor_temp)

    # Vẽ nhiệt độ của cảm biến
    draw.text((0, 0), 'C: {}'.format(formatted_sensor_temp), font=font, fill=255)
    
    # Vẽ đường ngăn cách
    draw.line((0, 20, disp.width, 20), fill=255)
    # Vẽ nhiệt độ cài đặt
    if ok_pressed == 1 :
        draw.text((0, 25), 'S: '+'▶ '+str(set_temp1) +'.'+ str(set_temp2)+'℃' , font=font, fill=255)
    elif ok_pressed == 2 :
        draw.text((0, 25), 'S: '+str(set_temp1) +'.'+ str(set_temp2)+'◀'+'℃' , font=font, fill=255)
    else :
         draw.text((0, 25), 'S: '+str(set_temp1) +'.'+ str(set_temp2)+'℃' , font=font, fill=255)
    # Vẽ các nút điều khiển
    # Đường kẻ dưới các nút
    draw.line((0, 45, disp.width, 45), fill=255)
    # Vẽ nút -
    if counter3 == 0 :
        draw.text((10, 50), '▶-', font=font, fill=255)
    else:
        draw.text((10, 50), '-', font=font, fill=255)
    # Vẽ nút OK với màu thay đổi dựa vào trạng thái nhấn

    ok_fill = 0 
    if ok_pressed == 3:
        btok = 0
        ok_fill =  255
    draw.rectangle([(disp.width // 2 - 20, 50), (disp.width // 2 + 20, 64)], outline=255, fill=ok_fill)
    draw.text((disp.width // 2 - 15, 50), 'OK', font=font, fill=255 - ok_fill)
    # Vẽ nút +
    if counter3 == 1 :
        draw.text((disp.width - 30, 50), '▶+', font=font, fill=255)
    else:
        draw.text((disp.width - 20, 50), '+', font=font, fill=255)
    image = image.rotate(180)
    disp.image(image)
    disp.display()
def PWMcontrol():
    global pwmstart , timePWM
    while True:
        if pwmstart == True:
            GPIO.output(pin_relay1, 1)
            time.sleep(0.05)
            GPIO.output(pin_relay2, 1)
            time.sleep(0.05)
            GPIO.output(pin_relay3, 1)
            time.sleep(0.05)
            GPIO.output(pin_relay1, 0)
            time.sleep(0.05)
            GPIO.output(pin_relay2, 0)
            time.sleep(0.05)             
            GPIO.output(pin_relay3, 0)
            time.sleep(0.05)
            time.sleep(0.3)
        else:
            pass
        time.sleep(0.01)

def PIDcontroller ():
    global  temp , pid , start ,pwmstart , timePWM ,counter3
    while True:
        if start == True:
            control = pid(temp)
            # Điều khiển thiết bị
            #print(control)
            if counter3 == 1 :
                if control > 0 :
                    pwmstart = True
                else:
                    pwmstart = False
                    GPIO.output(pin_relay1, 0)
                    GPIO.output(pin_relay2, 0)
                    GPIO.output(pin_relay3, 0)
                    

            else :
                if control < 0 :
                    pwmstart = True
                else:
                    pwmstart = False
                    GPIO.output(pin_relay1, 0)
                    GPIO.output(pin_relay2, 0)
                    GPIO.output(pin_relay3, 0)

              
        else:
            pass

def rotaryread():
    global counter1,counter2 ,counter3, btok , clkLastState , pid ,counter , start ,pwmstart
    while True:
       
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        if clkState != clkLastState:
            if dtState != clkState:
                if btok == 0 :
                    counter3 = 1
                if btok == 1 :
                    counter1 += 1
                    if counter1 > 400:
                        counter1 = 400
                        counter2 = 0
                elif btok == 2 :
                    counter2 += 1
                    if counter2 > 99:
                        counter2 = 0
                        counter1 += 1
                    if counter1 > 399 :
                        counter2 = 0
            else:
                if btok == 0 :
                    counter3 = 0
                if btok == 1 :
                    counter1 -= 1
                    if counter1 < -40:
                        counter1 = -40
                        counter2 = 0
                elif btok == 2 :
                    counter2 -= 1
                    if counter2 < 0:
                        counter2 = 99
                        counter1 -=1
            print(counter1 , counter2 )
            # update_display(temp,counter,False)
        clkLastState = clkState
        current_button_state = GPIO.input(button) == GPIO.LOW
        
        # Kiểm tra xem nút có được nhấn và trạng thái trước đó có phải là không nhấn không

        if current_button_state == 1 :
            counter +=1
            print(counter)
            if counter > 100:
                btok = 0
                counter = 0
                counter2 = 0
                counter1 = 0
                GPIO.output(pin_relay1, 0)
                GPIO.output(pin_relay2, 0)
                GPIO.output(pin_relay3, 0)
                start = False
                pwmstart = False
                pid = PID(1.0, 0.1, 0.3, setpoint=0)
                pid.sample_time = 0.3  # Cập nhật mỗi 1 giây
                pid.output_limits = (-1, 1)
                print("reset")
        if current_button_state and not last_button_state:
            counter = 0
            btok += 1
            print(btok)
            # update_display(temp,counter,True)
            if btok == 3 :
                start = True
                pid = PID(1.0, 0.1, 0.3, setpoint=(counter1 + counter2/100))
                pid.sample_time = 0.3  # Cập nhật mỗi 1 giây
                pid.output_limits = (-1, 1)
                
        
        # Cập nhật trạng thái nút cho lần lặp tiếp theo
        last_button_state = current_button_state
        # temp = sensor.temperature
        #print("Temperature: {0:0.3f}C".format(temp))
        # update_display(temp,counter,False)
        time.sleep(0.001)

def mainprogram():
    global counter1 ,counter2, btok , temp
    try:
        while True:
            temp = sensor.temperature
            update_display(temp,counter1,counter2,btok)

    finally:
        GPIO.cleanup()

thread1 = threading.Thread(target=rotaryread)
thread2 = threading.Thread(target=mainprogram)
thread3 = threading.Thread(target=PIDcontroller)
thread4 = threading.Thread(target=PWMcontrol)

# Khởi động thread
thread1.start()
thread2.start()
thread3.start()
thread4.start()