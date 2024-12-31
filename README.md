# ColorTrack-Rover



ColorTrack Rover 是一款由树莓派驱动的自主机器人小车，能够实时检测并跟随特定颜色目标。小车利用OpenCV的计算机视觉算法，通过广角CSI摄像头在HSV颜色空间中识别并追踪目标，适应动态环境。配备麦克纳姆轮，小车可以实现全向移动，包括前进、后退、横移以及原地旋转，从而确保精确且流畅的跟踪效果。为了稳定运动，PID控制算法优化了速度和方向的调整，减少了振荡，提高了跟踪的准确性。ColorTrack Rover 适用于物体追踪、基于视觉的导航和机器人实验等应用场景，将图像处理技术与实际运动控制相结合，展现了出色的功能性。

**ColorTrack Rover** is an autonomous, Raspberry Pi-powered robotic vehicle designed to detect and follow a specific color target in real-time. Using computer vision algorithms with OpenCV, the rover identifies and tracks the target in the HSV color space via a wide-angle CSI camera, allowing it to adapt to a dynamic environment. Equipped with Mecanum wheels, the vehicle can move omnidirectionally—forward, backward, sideways, and spin—ensuring precise and smooth tracking. To stabilize movement, a PID control algorithm refines speed and direction adjustments, reducing oscillations and enhancing tracking accuracy. Ideal for applications in object tracking, visual-based navigation, and robotics experimentation, the ColorTrack Rover combines advanced image processing with real-world movement control.



目录结构
├─README.md
├─project
|    ├─myservice.service
|    └run.sh
├─Four_Wheel
|     ├─main.py
|     ├─motion_control.py
|     └object_detection.py