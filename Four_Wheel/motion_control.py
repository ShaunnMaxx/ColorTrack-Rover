import RPi.GPIO as GPIO
import time

# 定义移动速度
Speed = 40
move_sleep = 0.05  # 每次运动持续时间
stop_sleep = 0.2  # 停止时间
rotate_sleep = 0.1  # 旋转时间

# GPIO引脚定义
IN1, IN2 = 19, 13  # 左前电机
IN3, IN4 = 6, 5  # 右前电机
IN5, IN6 = 9, 10  # 左后电机
IN7, IN8 = 22, 27  # 右后电机
ENA, ENB, ENC, END = 26, 0, 17, 11  # 左前、右前、左后、右后电机的使能引脚

# 禁用警告
GPIO.setwarnings(False)

# 初始化GPIO模式
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8], GPIO.OUT)
GPIO.setup([ENA, ENB, ENC, END], GPIO.OUT)

# 使能电机，设置所有使能引脚为高电平
GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENB, GPIO.HIGH)
GPIO.output(ENC, GPIO.HIGH)
GPIO.output(END, GPIO.HIGH)

# 设置PWM频率为100Hz
pwm_left_front = GPIO.PWM(ENA, 100)
pwm_right_front = GPIO.PWM(ENB, 100)
pwm_left_back = GPIO.PWM(ENC, 100)
pwm_right_back = GPIO.PWM(END, 100)

# 启动PWM，并将初始速度设置为0
pwm_left_front.start(0)
pwm_right_front.start(0)
pwm_left_back.start(0)
pwm_right_back.start(0)


# 前进
def forward():
    set_speed(Speed)
    GPIO.output([IN1, IN3, IN5, IN7], GPIO.HIGH)
    GPIO.output([IN2, IN4, IN6, IN8], GPIO.LOW)
    time.sleep(move_sleep)  # 控制运动时间


# 后退
def backward():
    set_speed(Speed)
    GPIO.output([IN1, IN3, IN5, IN7], GPIO.LOW)
    GPIO.output([IN2, IN4, IN6, IN8], GPIO.HIGH)
    time.sleep(move_sleep)  # 控制运动时间


# 右移
def move_right():
    set_speed(Speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.HIGH)
    GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.LOW)
    GPIO.output(IN8, GPIO.HIGH)
    time.sleep(move_sleep)  # 控制运动时间


# 左移
def move_left():
    set_speed(Speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN5, GPIO.LOW)
    GPIO.output(IN6, GPIO.HIGH)
    GPIO.output(IN7, GPIO.HIGH)
    GPIO.output(IN8, GPIO.LOW)
    time.sleep(move_sleep)  # 控制运动时间


# 旋转
def rotate(clockwise):
    set_speed(Speed)
    if clockwise:
        GPIO.output([IN1, IN5], GPIO.HIGH)
        GPIO.output([IN2, IN6], GPIO.LOW)
        GPIO.output([IN3, IN7], GPIO.LOW)
        GPIO.output([IN4, IN8], GPIO.HIGH)
        time.sleep(rotate_sleep)
        
    else:
        GPIO.output([IN1, IN5], GPIO.LOW)
        GPIO.output([IN2, IN6], GPIO.HIGH)
        GPIO.output([IN3, IN7], GPIO.HIGH)
        GPIO.output([IN4, IN8], GPIO.LOW)
        time.sleep(rotate_sleep)


# 停止立即
def stop():
    set_speed(0)
    GPIO.output([IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8], GPIO.LOW)

# 停止持续
def stop2():
    set_speed(0)
    GPIO.output([IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8], GPIO.LOW)
    time.sleep(stop_sleep) # 停止时间


# 设置所有电机速度
def set_speed(speed):
    pwm_left_front.ChangeDutyCycle(speed)
    pwm_right_front.ChangeDutyCycle(speed)
    pwm_left_back.ChangeDutyCycle(speed)
    pwm_right_back.ChangeDutyCycle(speed)
