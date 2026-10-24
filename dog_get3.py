from xgolib import XGO
from xgoedu import XGOEDU
import time
import cv2
import numpy as np
import signal
import sys
import RPi.GPIO as GPIO
#from xgoedu import XGOEDU

#XGO_edu = XGOEDU()
print(cv2.__version__)
blue_color_low = [0, 80, 73]
blue_color_high = [80, 255, 220]

green_color_low = [42, 80, 70]
green_color_high = [82, 255, 230]

red_color_low = [100, 80, 70]
red_color_high = [140, 255, 230]

blue_color_mask = [blue_color_low, blue_color_high]
green_color_mask = [green_color_low, green_color_high]
red_color_mask = [red_color_low, red_color_high]

target_x = 160#111111111
target_y = 190

target_ball_green=24#21

target_ball_red=24#21
target_ball_blue=24
angle_position=0
target_yaw=-4.85

# Global variables for cleanup
XGO_edu = None
robot_dog = None

# 摄像头由 XGOEDU 内部统一管理（树莓派5的 picamera2），
# 不再单独创建 Picamera2 实例，避免双重 acquire 导致 Camera busy。

def cleanup_gpio():
    """Clean up GPIO resources"""
    try:
        GPIO.cleanup()
        print("GPIO cleanup completed")
    except Exception as e:
        print(f"Error during GPIO cleanup: {e}")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\nShutting down gracefully...')
    cleanup_gpio()
    if robot_dog:
        robot_dog.stop()
    sys.exit(0)

# Register signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

class RobotDog:
    def __init__(self, model):
        self.dog = XGO(model)

    def move_dog(self, direction, step,runtime): #正值左移，负值右移
        self.dog.pace('slow')
        self.dog.move(direction,step)
        #self.dog.move('y',-1)
        time.sleep(runtime)
        self.dog.stop()

    def adjust_attitude(self, roll, pitch, yaw):
        # 机身姿态调整
        self.dog.attitude(['r', 'p', 'y'], [roll, pitch, yaw])
        # 等待姿态调整完成
        time.sleep(0.5)

    def periodic_motion(self, direction, period):
        # 机身周期运动
        if direction == 'x':
            self.dog.periodic_tran('x', period)
        elif direction == 'y':
            self.dog.periodic_tran('y', period)
        # 持续一定时间后停止
        time.sleep(period * 2)
        self.dog.periodic_tran(direction, 0)

    def move_arm(self, arm_x, arm_z):
        """
        控制机械臂移动到指定位置。
        
        :param arm_x: 水平面移动距离(mm)
        :param arm_z: 垂直面移动距离(mm)
        """
        # 检查输入值是否在机械臂工作范围内
        if -80 <= arm_x <= 155 and -50 <= arm_z <= 155:
            self.dog.arm(arm_x, arm_z)
        else:
            print("Error: Arm position out of range.")

    def control_claw(self, pos):
        """
        控制机械臂夹爪的开合。
        
        :param pos: 夹爪开合位置(0-255)
        """
        self.dog.claw(pos)

    def arm_grab_ball(self,y_distance=0,run_time=3,grab_angel=0,back_runtime=0):

        self.dog.arm(100, 100)  #移动到某位置
        time.sleep(2)
        self.dog.claw(80)  #夹爪张开 grab_angel
        time.sleep(2)
        self.dog.arm(140, -40)  #移动到抓取位置 111111111111 150,-10
        time.sleep(2)

        self.dog.claw(230)  #抓取小球  180
        time.sleep(2)

        self.dog.arm(100, 20)  #移动回某位置
        
        time.sleep(1)
        robot_dog.dog.translation('z', 75)
        time.sleep(1)
        
        robot_dog.adjust_attitude(0,0,0)

        time.sleep(1)
        self.move_dog('y', -y_distance-3,run_time)  #机器狗移动到杯子前
        time.sleep(1)

        #调整方便进球
        #离杯子近一点
        self.move_dog('x',8,2)#18 1.5
        time.sleep(1)

        self.dog.claw(1)  #夹爪张开
        time.sleep(3)
        
        self.move_dog('x',-8,1.5)
        time.sleep(1)

        

        #self.dog.arm_polar(-70, -80)  11111111
        time.sleep(1)
        self.move_dog('y', y_distance-1,back_runtime)  #机器狗回原位置
        
        # self.dog.arm(0, 0)  #机械臂移动到杯子上
        
        # time.sleep(1)
        # self.dog.claw(128)  #夹爪回到初始状态
        # self.dog.arm(0, 0)  #机械臂移动回初始位置
        # self.move_dog('y', -y_distance)  #机器狗移动到初始位置
    def stop(self):
        self.dog.stop()

    def turn(self,step,runtime):
        self.dog.turn(step)
        time.sleep(runtime)
        self.dog.stop()

    def adjust_direction(self):
        now_angle=self.dog.read_yaw()
        self.dog.turn_to(target_yaw-self.dog.init_yaw,10,1)

    def get_ball(self,ball_color,cup_distance,target_ball,grab_angel=0):
        # BallRecognition 内部按 320x240 设计（target_x=160），
        # 而 take_photo 把摄像头切到了 640x480，这里需要切回。
        if XGO_edu is not None and XGO_edu.cap is not None and int(XGO_edu.cap.get(3)) != 320:
            XGO_edu.cap.set(3, 320)
            XGO_edu.cap.set(4, 240)
        #self.adjust_direction() #调整方位
        while True:
            time.sleep(1)
            robot_dog.dog.translation('z', 100)
            time.sleep(1)                                
            self.adjust_attitude(0,20,0) #调整倾角
            time.sleep(1)
            cnt=0
            #target_ball
            x_sum=0
            x_average=0
            blob_sum=0
            blob_average=0
            while True: 
                ball_position = XGO_edu.BallRecognition(ball_color)
                if ball_position[1] and ball_position[0] :
                    x_sum+=ball_position[0]
                    blob_sum+=ball_position[2]
                    cnt+=1
                if cnt>=3:
                    x_average=x_sum/cnt
                    blob_average=blob_sum/cnt
                    break
            self.adjust_attitude(0,0,0)
            time.sleep(1)
            print(x_average - target_x)
            print("ball:")
            print(blob_average)
            if abs(x_average - target_x) > 14:#10
                speed=max(abs((x_average - target_x))/2,12)   #限定最小调整速度
                if ((x_average - target_x))>0:
                    speed=-speed
                
                robot_dog.dog.translation('z', 75)
                time.sleep(1)
                self.move_dog('y',speed,1)
                #robot_dog.turn(-(ball_position[0] - target_x)/5,1)
                #robot_dog.adjust_attitude(0,10,angle_position)
            self.stop()
            time.sleep(1)#111111111
            if abs(blob_average - target_ball)>2:
                robot_dog.dog.translation('z', 75)
                time.sleep(1)
                if (blob_average - target_ball)>0:
                    self.move_dog("x",-8,1)
                    #self.move_dog("x",15,1)
                else:
                    self.move_dog("x",16,1)
            self.stop()
            #or (blob_average - target_ball)>1

            if abs(ball_position[0] - target_x) <=14 and abs(blob_average - target_ball)<=2:
                break



        #     else:
        #         self.stop()
        #         break
        # while True:
        #     time.sleep(1)                                
        #     self.adjust_attitude(0,20,0) #调整倾角
        #     time.sleep(2)
        #     while True: 
        #         ball_position = XGO_edu.BallRecognition(ball_color)
        #         if ball_position[2] :
        #             break
        #     self.adjust_attitude(0,0,0)
        #     time.sleep(1)
        #     print("ball:")
        #     print(ball_position[2])

        #     if abs(ball_position[2] - target_ball)>=1:
        #         if (ball_position[2] - target_ball)>0:
        #             self.move_dog("x",-10,1)
        #         else:
        #             self.move_dog("x",10,1)

        #     else:
        #         self.stop()
        #         break
        
        time.sleep(1)
        robot_dog.dog.translation('z', 100)
        time.sleep(1)
        self.adjust_attitude(0,20,0)
        if ball_color==blue_color_mask:
            self.arm_grab_ball(cup_distance,3.7,grab_angel=grab_angel,back_runtime=2.5)#4,2.5
        elif ball_color==green_color_mask:
            self.arm_grab_ball(cup_distance,3,grab_angel=grab_angel,back_runtime=4)#夹取球，并放置到杯子中，再回到原位 2.5
        elif ball_color==red_color_mask:
            self.arm_grab_ball(cup_distance,2.2,grab_angel=grab_angel,back_runtime=2.5)#夹取球，并放置到杯子中，再回到原位 2,2.5
    def read_yaw(self):
        return self.dog.read_yaw()



def detect_circles_multi_color(image):
    """
    同时检测黄色和黑色圆形标识物
    返回: 检测到的圆形列表和标注后的图像
    """
    result_img = image.copy()
    detected_circles = []
    color=[0,0]
    # 转换颜色空间
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 中值滤波降噪
    gray = cv2.medianBlur(gray, 5)
    hsv = cv2.medianBlur(hsv, 5)

    # === 1. 检测黑色圆形 ===
    black_circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=100, param2=50, minRadius=10, maxRadius=50
    )

    if black_circles is not None:
        circles = np.uint16(np.around(black_circles))
        for (x, y, r) in circles[0, :]:
            # 创建圆形掩膜
            mask = np.zeros(image.shape[:2], dtype="uint8")
            cv2.circle(mask, (x, y), r, 255, -1)

            # 计算圆内平均HSV值
            hsv_temp = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mean_hsv = cv2.mean(hsv_temp, mask=mask)
            h, s, v, _ = mean_hsv

            # 判断是否为黑色 (低亮度)
            if v < 80 and s < 120:
                detected_circles.append({
                    'x': x, 'y': y, 'r': r,
                    'color': 'Black',
                    'hsv': (h, s, v)
                })
                # 绘制黑色圆形 (蓝色标注)
                cv2.circle(result_img, (x, y), r, (255, 0, 0), 2)
                cv2.circle(result_img, (x, y), 2, (255, 0, 0), 3)
                color[0] = 1
    # === 2. 检测黄色圆形 ===
    # 黄色HSV范围
    lower_yellow = np.array([10, 150, 100])
    upper_yellow = np.array([30, 255, 255])

    # 生成黄色掩膜
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 提取黄色区域用于霍夫检测
    yellow_region = cv2.bitwise_and(gray, gray, mask=yellow_mask)

    yellow_circles = cv2.HoughCircles(
        yellow_region, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=100, param2=20, minRadius=5, maxRadius=50
    )

    if yellow_circles is not None:
        circles = np.uint16(np.around(yellow_circles))
        for (x, y, r) in circles[0, :]:
            # 创建圆形掩膜
            mask = np.zeros(image.shape[:2], dtype="uint8")
            cv2.circle(mask, (x, y), r, 255, -1)

            # 计算圆内平均HSV值
            hsv_temp = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mean_hsv = cv2.mean(hsv_temp, mask=mask)
            h, s, v, _ = mean_hsv

            # 判断是否为黄色
            if 10 <= h <= 35 and s > 120 and v > 80:
                detected_circles.append({
                    'x': x, 'y': y, 'r': r,
                    'color': 'Yellow',
                    'hsv': (h, s, v)
                })
                # 绘制黄色圆形 (绿色标注)
                cv2.circle(result_img, (x, y), r, (0, 255, 0), 2)
                cv2.circle(result_img, (x, y), 2, (0, 0, 255), 3)
                color[1] = 1
    return detected_circles, result_img, color

def take_photo():
    # 复用 XGOEDU 的摄像头实例，并切到 640x480 用于圆形检测
    XGO_edu.open_camera()
    if int(XGO_edu.cap.get(3)) != 640:
        XGO_edu.cap.set(3, 640)
        XGO_edu.cap.set(4, 480)
    success, frame = XGO_edu.cap.read()
    if not success or frame is None:
        # 抓帧失败时返回一张黑图，避免后续 cv2 处理崩溃
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
    return frame

# 调用函数





if __name__ == "__main__":
    
    # Clean up any existing GPIO resources first
    cleanup_gpio()
    time.sleep(1)  # Give time for cleanup
        
    # Initialize robot components
    robot_dog = RobotDog("xgomini")
    XGO_edu = XGOEDU()
        
    time.sleep(8)
    for _ in range(5):
        target_yaw += robot_dog.read_yaw()
    target_yaw /= 5
    
    robot_dog.dog.translation('z', 75)
    time.sleep(2)
    
    robot_dog.move_dog('x',30,3.5)#30 6.5
    robot_dog.stop()
    time.sleep(1)
    
    while True:
        #视觉接入
        image = take_photo()
        circles,circle_image,color = detect_circles_multi_color(image)
        #ret = color[0]-color[1]
        ret = -1
        if ret!= 0:
            break
    if circles:
        i=0
        for i, circle in enumerate(circles):
            print(f"  圆形 {i + 1}: {circle['color']} - 位置({circle['x']}, {circle['y']}) 半径{circle['r']} HSV{circle['hsv']}")
    else:
        print("  未检测到圆形")
    print()
    if(ret==1):#hei

        #黑逻辑
        robot_dog.dog.claw(0)
        XGO_edu.xgoSpeaker('anger.wav')
        robot_dog.move_arm(120,-20)
        time.sleep(3)
        robot_dog.move_dog('x',30,3)
        time.sleep(1)
        robot_dog.move_dog('y',-20,2)
        time.sleep(1)
        robot_dog.dog.reset()
        time.sleep(1)
        
        robot_dog.dog.translation('z', 75)
        time.sleep(2)
        
        robot_dog.move_dog('y',20,2.3)

        time.sleep(1)
        robot_dog.turn(-8,1.3)#1.5

        time.sleep(1)
        robot_dog.move_dog('x',30,6.5)
        time.sleep(1)
        #time.sleep(1)
        #robot_dog.turn(8,1.5)
        
        robot_dog.get_ball(green_color_mask,250,target_ball_green,30)
        time.sleep(1)
        robot_dog.get_ball(red_color_mask,250,target_ball_red,30)
        time.sleep(1)
        
        
        robot_dog.turn(8,1.6)
        time.sleep(1)
        
        robot_dog.move_dog('y',20,2.4)
        time.sleep(1)
        
        robot_dog.turn(8,1.6)
        time.sleep(1)
        
        robot_dog.move_dog('x',-20,20)

        robot_dog.dog.reset()
        robot_dog.stop()
        
    elif (ret==-1):#huang
        XGO_edu.xgoSpeaker('anger.wav')
        robot_dog.move_dog('y',20,3)
        time.sleep(1)
       
        #robot_dog.turn(-8,1.3)
        #time.sleep(1)
        
        robot_dog.move_dog('x',30,8)
        time.sleep(1)
        robot_dog.move_dog('y',-20,3)#-20,3.5
        time.sleep(1)

        #robot_dog.turn(-8,1)
        #time.sleep(1)
        
        robot_dog.get_ball(green_color_mask,250,target_ball_green,30)
        time.sleep(1)
        robot_dog.get_ball(blue_color_mask,250,target_ball_blue,30)
        time.sleep(1)
        robot_dog.get_ball(red_color_mask,250,target_ball_red,30)
        time.sleep(1)

        #修改倒车逻辑
        robot_dog.move_dog('y',20,4)
        time.sleep(1)

        robot_dog.move_dog('x',-30,11)
        time.sleep(1)

        robot_dog.dog.reset()
        robot_dog.stop()
        
        #robot_dog.turn(8,1.6)
        #time.sleep(1)
        
        #robot_dog.move_dog('x',-20,2.5)
        #time.sleep(1)
        
        #robot_dog.move_dog('y',20,4)
        #time.sleep(1)
        
        #robot_dog.turn(8,1.6)
        #time.sleep(1)
        
        #robot_dog.move_dog('x',-20,10)
        #time.sleep(1)
        
        #robot_dog.move_dog('y',-20,4)
        #time.sleep(1)
        
        #robot_dog.move_dog('x',-20,8)
        #time.sleep(1)
        
        #robot_dog.dog.reset()
        #robot_dog.stop()

# 摄像头由 XGOEDU/Picamera2Capture 在进程退出时自动 release（atexit）

