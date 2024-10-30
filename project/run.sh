#!/bin/bash  

# 此脚本为Windows系统版本，若在Linux上运行，请执行 dos2unix run.sh 命令

# 定义日志文件和超时时间  
LOGFILE="/home/pi/Desktop/project/run.log"  
TIMEOUT=60  # 超时时间，单位为秒  
  
# 开始时间  
START_TIME=$(date +%s)  
  
while true; do  
    CAMERA_STATUS=$(vcgencmd get_camera | grep -q 'supported=1 detected=1' && echo "ready" || echo "not_ready")  
      
    if [ "$CAMERA_STATUS" == "ready" ]; then  
        echo "$(date +'%Y-%m-%d %H:%M:%S') 摄像头已准备好" >> $LOGFILE  
        break  
    else  
        echo "$(date +'%Y-%m-%d %H:%M:%S') 等待摄像头可用..." >> $LOGFILE  
        sleep 2  
          
        # 检查是否超时  
        ELAPSED_TIME=$(( (date +%s) - START_TIME ))  
        if [ "$ELAPSED_TIME" -ge "$TIMEOUT" ]; then  
            echo "$(date +'%Y-%m-%d %H:%M:%S') 超时，摄像头未准备好，脚本退出" >> $LOGFILE  
            exit 1  
        fi  
    fi  
done  
  
# 切换到项目目录  
CD_RESULT=$(cd /home/pi/Desktop/project && echo "success" || echo "failure")  
if [ "$CD_RESULT" != "success" ]; then  
    echo "$(date +'%Y-%m-%d %H:%M:%S') 无法切换到项目目录" >> $LOGFILE  
    exit 1  
fi  
  
# 执行 Python 脚本  
PYTHON_RESULT=$(/usr/bin/python3 main.py 2>&1)  
EXIT_CODE=$?  
  
if [ "$EXIT_CODE" -eq 0 ]; then  
    echo "$(date +'%Y-%m-%d %H:%M:%S') Python 脚本执行成功" >> $LOGFILE  
else  
    echo "$(date +'%Y-%m-%d %H:%M:%S') Python 脚本执行失败: $PYTHON_RESULT" >> $LOGFILE  
fi  
  
exit $EXIT_CODE


