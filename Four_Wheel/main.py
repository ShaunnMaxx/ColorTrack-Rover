# 根据目标在不同区域，及目标面积与初始值对比，对小车控制
# 使用pwm速度控制，可以四个方向移动，有自旋，一圈后停止
# 使用麦克纳姆轮，两个L298N分别驱动前轮和后轮
# 不旋转
# 橙色
# without PID
import cv2
import time
from motion_control import (
    forward,
    backward,
    move_right,
    move_left,
    rotate,
    stop,
    control_speed,
)
from object_detection import detect_target
Speed = 20
# 设置初始面积阈值和中心区域阈值
initial_area_threshold_min = 1500
initial_area_threshold_max = 3000
tracking_area_threshold = 300  # 只有面积大于这个阈值才跟踪
center_x_min, center_x_max = 120, 190
center_y_min, center_y_max = 90, 150

# 旋转一圈所需的时间（根据实际情况调整）
ROTATION_DURATION = 0  # 5秒假设能旋转一圈


def adjust_size(area):
    """根据目标面积调整距离"""
    if area < initial_area_threshold_min:
        forward()
        print("中央过小 - 前进")
    elif area > initial_area_threshold_max:
        backward()
        print("中央过大 - 后退")
    else:
        stop()
        print("中央区域 - 不动")


def move_to_target(center_x, center_y):
    """根据目标位置调整运动方向"""
    if center_x > center_x_max and center_y < center_y_max:
        move_right()
        print("右上角 - 右转")
    elif center_x < center_x_min and center_y < center_y_max:
        move_left()
        print("左上角 - 左转")
    elif center_y < center_y_min:
        #control_speed(Speed)
        forward()
        print("中间 - 前进")
    elif center_y > center_y_max:
        #control_speed(Speed)
        backward()
        print("下方 - 后退")
    else:
        stop()
        print("未识别区域 - 停止")


try:
    rotating = False  # 标识是否处于旋转状态
    tracking = False  # 标识是否处于目标跟踪状态
    start_rotation_time = None  # 记录旋转开始的时间

    fourcc = cv2.VideoWriter_fourcc(*"XVID")

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    frame_counter = 0
    frame_interval = 3  # 设定每3帧处理一次

    while cap.isOpened():
        frame_counter += 1
        ret, frame = cap.read()

        if ret:
            # 将图像从BGR转换为HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 检测目标
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
            # 显示掩码和原始图像
            cv2.imshow("Mask", mask)
            cv2.imshow("Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
except Exception as e:
    print("发生错误:", e)
finally:
    GPIO.cleanup()  # 清理GPIO设置
