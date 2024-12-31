# 基于树莓派 zero 2W 的颜色跟随小车，双线程（图像处理线程，运动控制线程）
# 线程之间通信：queue （大小为1）
# 线程同步控制：Event
# 1. 图像处理线程：检测目标，将结果放入队列
# 2. 运动控制线程：根据队列中的数据调整小车运动
# 目标消失后，先等待三秒，再旋转一圈寻找目标
# 采用麦克纳姆轮，两个L298N对四个轮子单独控制
# 使用HSV颜色空间检测橙色物体
# 时间戳：2024.12.31

import cv2
import time
import threading
import queue
from motion_control import (
    forward,
    backward,
    move_right,
    move_left,
    rotate,
    stop,
    stop2
)
from object_detection import detect_target

# 设置初始参数
tracking_area_threshold = 300  # 面积阈值
center_x_min, center_x_max = 100, 220
center_y_min, center_y_max = 60, 180

# 创建线程间通信队列
result_queue = queue.Queue(maxsize=1)  # 队列大小设置为1
# 创建Event对象，用于同步控制
image_processing_event = threading.Event()
motion_control_event = threading.Event()  # 用于标记运动控制线程是否完成任务
terminate_event = threading.Event()  # 用于标记程序退出
image_processing_event.set()  # 初始状态为True，允许图像处理线程插入数据

# 使用类封装目标状态
class TargetState:
    def __init__(self):
        self._target_lost = False
        self._last_target_lost_time = 0
        self._lock = threading.Lock()

    def set_target_lost(self, lost: bool):
        with self._lock:
            self._target_lost = lost
            if lost:
                self._last_target_lost_time = time.time()

    def get_target_lost(self):
        with self._lock:
            return self._target_lost

    def get_last_lost_time(self):
        with self._lock:
            return self._last_target_lost_time


target_state = TargetState()

def image_processing_thread(queue):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    try:
        while cap.isOpened():
            if terminate_event.is_set():
                break
            ret, frame = cap.read()
            if ret:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                area, center_x, center_y, mask = detect_target(hsv)

                if area > tracking_area_threshold:
                    # 等待运动控制线程完成任务后才能插入数据
                    image_processing_event.wait()

                    # 只有队列为空时才插入数据
                    if queue.empty():
                        queue.put((area, center_x, center_y))
                        print(f"图像处理线程：检测到目标 - 面积: {area}, 中心: ({center_x}, {center_y})")
                        motion_control_event.set()
                        target_state.set_target_lost(False)
                else:
                    # 目标消失，清空队列并通知运动控制线程目标已消失
                    if not queue.empty():
                        print("图像处理线程：目标消失，清空队列")
                        queue.queue.clear()
                        target_state.set_target_lost(True)
                        motion_control_event.set()

    except Exception as e:
        print("图像处理线程出错:", e)

    finally:
        cap.release()
        cv2.destroyAllWindows()

def motion_control_thread(queue):
    """运动控制线程：根据检测结果调整车辆运动"""
    rotation_count = 0
    waiting_start_time = None

    while not terminate_event.is_set():
        try:
            # 检查队列是否有数据
            if not queue.empty():
                # 目标重新出现，处理新数据
                area, center_x, center_y = queue.get()
                print(f"运动控制线程：收到目标数据 - 面积: {area}, 中心: ({center_x}, {center_y})")

                # 重置旋转状态
                rotation_count = 0
                waiting_start_time = None

                # 等待运动控制完成任务
                motion_control_event.wait()
                motion_control_event.clear()

                # 根据目标位置移动
                move_to_target(center_x, center_y)
                image_processing_event.set()  # 通知图像处理线程继续工作
                target_state.set_target_lost(False)  # 确认目标存在

            else:
                # 如果目标丢失
                if target_state.get_target_lost():
                    # 记录目标丢失后的首次等待时间
                    if waiting_start_time is None:
                        waiting_start_time = time.time()
                        print("目标丢失，开始计时等待...")

                    # 检查等待时间是否超过3秒
                    if time.time() - waiting_start_time >= 3:
                        if rotation_count < 23:  # 限制旋转次数
                            rotate(True)
                            print(f"目标丢失，旋转中 ({rotation_count + 1}/23)")
                            stop2()  # 每次旋转后短暂停止
                            rotation_count += 1
                        else:
                            stop2()
                            print("旋转完成，无目标，保持停止")
                            target_state.set_target_lost(False)  # 重置目标状态
                            waiting_start_time = None
                    else:
                        print(f"目标丢失，等待 {3 - (time.time() - waiting_start_time):.1f} 秒后旋转")
                else:
                    # 如果目标丢失状态未设置，确保初始化
                    print("检测到队列为空，但未设置目标丢失状态，重新初始化...")
                    target_state.set_target_lost(True)
                    waiting_start_time = time.time()

        except Exception as e:
            print("运动控制线程出错:", e)


def move_to_target(center_x, center_y):
    """根据目标位置调整运动方向"""
    if center_x > center_x_max and center_y < center_y_max:
        move_right()
        print("右上角 - 右移")
        stop()
    elif center_x < center_x_min and center_y < center_y_max:
        move_left()
        print("左上角 - 左移")
        stop()
    elif center_y < center_y_min:
        forward()
        print("中间 - 前进")
        stop()
    elif center_y > center_y_max:
        backward()
        print("下方 - 后退")
        stop()
    else:
        stop2()
        print("目标在中央区域，保持停止")

if __name__ == "__main__":
    image_thread = threading.Thread(target=image_processing_thread, args=(result_queue,))
    motion_thread = threading.Thread(target=motion_control_thread, args=(result_queue,))

    image_thread.daemon = True
    motion_thread.daemon = True

    image_thread.start()
    motion_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止")
        terminate_event.set()
        image_thread.join()
        motion_thread.join()
        stop()
