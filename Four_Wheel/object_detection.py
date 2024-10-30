import cv2
import numpy as np


# 定义检测红色区域的函数
def detect_target(hsv):
    # 定义橙色在HSV颜色空间中的范围
    lower_orange = np.array([10, 100, 100])  # 低阈值，接近橙色
    upper_orange = np.array([25, 255, 255])  # 高阈值，接近橙色

    # 创建用于检测橙色的掩码
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # 使用形态学操作清理掩码，去除噪声
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))

    # 在掩码中查找轮廓
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
