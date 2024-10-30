# 根据目标在不同区域，及目标面积与初始值对比，对小车控制
# 使用pwm速度控制，可以四个方向移动，有自旋，一圈后停止
# 使用麦克纳姆轮，两个L298N分别驱动前轮和后轮
# 不旋转
# 橙色
# with PID
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# 定义移动速度
Speed = 20

# GPIO引脚定义
IN1, IN2 = 19, 13  # 左前电机
IN3, IN4 = 6, 5  # 右前电机
IN5, IN6 = 9, 10  # 左后电机
IN7, IN8 = 22, 27  # 右后电机
ENA, ENB, ENC, END = 26, 0, 17, 11  # 左前、右前、左后、右后电机的使能引脚

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
    time.sleep(0.1)  # 延时100毫秒


# 后退
def backward():
    set_speed(Speed)
    GPIO.output([IN1, IN3, IN5, IN7], GPIO.LOW)
    GPIO.output([IN2, IN4, IN6, IN8], GPIO.HIGH)
    time.sleep(0.1)  # 延时100毫秒


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
    time.sleep(0.1)  # 延时100毫秒


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
    time.sleep(0.1)  # 延时100毫秒


# 旋转
def rotate(clockwise=True):
    set_speed(Speed)
    if clockwise:
        GPIO.output([IN1, IN5], GPIO.HIGH)
        GPIO.output([IN2, IN6], GPIO.LOW)
        GPIO.output([IN3, IN7], GPIO.LOW)
        GPIO.output([IN4, IN8], GPIO.HIGH)
        time.sleep(0.1)  # 延时100毫秒
    else:
        GPIO.output([IN1, IN5], GPIO.LOW)
        GPIO.output([IN2, IN6], GPIO.HIGH)
        GPIO.output([IN3, IN7], GPIO.HIGH)
        GPIO.output([IN4, IN8], GPIO.LOW)
        time.sleep(0.1)  # 延时100毫秒


# 停止
def stop():
    set_speed(0)
    GPIO.output([IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8], GPIO.LOW)


# 设置所有电机速度
def set_speed(speed):
    pwm_left_front.ChangeDutyCycle(speed)
    pwm_right_front.ChangeDutyCycle(speed)
    pwm_left_back.ChangeDutyCycle(speed)
    pwm_right_back.ChangeDutyCycle(speed)


# 设置初始面积阈值和中心区域阈值
initial_area_threshold_min = 1500
initial_area_threshold_max = 3000
tracking_area_threshold = 300  # 只有面积大于这个阈值才跟踪
center_x_min, center_x_max = 120, 190
center_y_min, center_y_max = 90, 150

# 旋转一圈所需的时间（根据实际情况调整）
ROTATION_DURATION = 0  # 5秒假设能旋转一圈

kp, ki, kd = 0.1, 0.01, 0.05  # 这些参数可以根据需要调整
prev_error_x, prev_error_y = 0, 0
integral_x, integral_y = 0, 0

def adjust_size(area):
    """根据目标面积调整距离，加入PID控制"""
    error_area = area - (initial_area_threshold_min + initial_area_threshold_max) / 2

    # PID计算
    integral_area += error_area
    derivative_area = error_area - prev_error_area
    output_area = kp * error_area + ki * integral_area + kd * derivative_area
    prev_error_area = error_area

    # 根据PID输出调整距离
    if output_area > threshold:  # 前进
        forward()
        print("PID控制 - 前进")
    elif output_area < -threshold:  # 后退
        backward()
        print("PID控制 - 后退")
    else:
        stop()
        print("PID控制 - 停止")


def move_to_target(center_x, center_y):
    """根据目标位置调整运动方向，加入PID控制"""
    error_x = center_x - (center_x_min + center_x_max) / 2
    error_y = center_y - (center_y_min + center_y_max) / 2

    # PID计算
    integral_x += error_x
    integral_y += error_y
    derivative_x = error_x - prev_error_x
    derivative_y = error_y - prev_error_y

    output_x = kp * error_x + ki * integral_x + kd * derivative_x
    output_y = kp * error_y + ki * integral_y + kd * derivative_y

    prev_error_x, prev_error_y = error_x, error_y

    # 根据PID输出调整移动
    if output_x > threshold:  # 右移
        move_right()
    elif output_x < -threshold:  # 左移
        move_left()
    if output_y > threshold:  # 前进
        forward()
    elif output_y < -threshold:  # 后退
        backward()
    print("PID调整: X方向输出=", output_x, ", Y方向输出=", output_y)


# 定义检测红色区域的函数
def detect_target(hsv):
# 定义橙色在HSV颜色空间中的范围
    lower_orange = np.array([10, 100, 100])  # 低阈值，接近橙色
    upper_orange = np.array([25, 255, 255])  # 高阈值，接近橙色

    # 创建用于检测橙色的掩码
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    ## 使用形态学操作清理掩码，去除噪声
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))
    
    ## 在掩码中查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # 获取最大轮廓，通常对应红色物体
        largest_contour = max(contours, key=cv2.contourArea)

        # 计算最大轮廓的面积
        area = cv2.contourArea(largest_contour)

        # 使用图像矩（moments）计算红色区域的质心（中心点）
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
        else:
            center_x, center_y = 0, 0

        return area, center_x, center_y, mask
    else:
        # 如果没有检测到红色区域，返回0面积和掩码
        return 0, 0, 0, mask


try:
    rotating = False  # 标识是否处于旋转状态
    tracking = False  # 标识是否处于目标跟踪状态
    start_rotation_time = None  # 记录旋转开始的时间

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('output.avi',fourcc,10.0, (320, 240))

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    frame_counter = 0
    frame_interval = 3  # 设定每3帧处理一次

    while cap.isOpened():
        frame_counter += 1
        ret, img = cap.read()
        if frame_counter % frame_interval != 0:
            continue  # 跳过部分帧
        if not ret or img is None:
            print("未能从摄像头读取帧，请检查摄像头连接。")
            break

        # 将图像从BGR颜色空间转换为HSV颜色空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 调用detect_target函数，检测红色区域并返回相关信息
        area, center_x, center_y, mask = detect_target(hsv)

        if area and area > tracking_area_threshold:
            # 检测到足够大的目标，进入跟踪状态
            rotating = False
            tracking = True
            start_rotation_time = None
            print("检测到目标，开始跟踪，面积:", area)
            print("x= ", center_x, "y= ", center_y)

            if (
                center_x_min < center_x < center_x_max
                and center_y_min < center_y < center_y_max
            ):
                adjust_size(area)
            else:
                move_to_target(center_x, center_y)

        else:
            if tracking:
                # 如果之前是跟踪状态，现在目标消失了，开始旋转
                print("目标消失，开始旋转")
                tracking = False
                rotating = True
                start_rotation_time = time.time()  # 开始旋转计时
                rotate()

            if rotating and time.time() - start_rotation_time > ROTATION_DURATION:
                # 如果旋转一圈后仍未找到目标，停止旋转
                stop()
                print("旋转一圈后没有找到目标，停止等待")
                rotating = False  # 停止旋转状态
                start_rotation_time = None  # 重置旋转时间

        # 显示结果
        #out.write(img)
        #cv2.imshow("frame", img)
        #cv2.imshow("mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
