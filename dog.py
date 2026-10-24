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
target_ball_blue=21
angle_position=0
target_yaw=-4.85

# 以下距离通过“速度 × 时间”估算，必须在实际场地量测后调整。
# x 正向为前进，y 正向为机械狗左侧；图中的黄色绕行先向机械狗右侧。
CALIBRATION_CONFIRMED = False  # 标定完成并在空场地验证后，改为 True 才允许自动移动
FORWARD_CM_PER_SEC = 5.0       # x 方向速度参数 30 时的初始估计，待实测
BACKWARD_CM_PER_SEC = 5.0      # x 方向速度参数 -20 时的初始估计，待实测
SIDE_CM_PER_SEC = 5.0          # y 方向速度参数 20 时的初始估计，待实测
MARKER_RADIUS_AT_20CM = 45    # 640x480 图像中，20cm 处牌子的半径像素；需标定
MARKER_TIMEOUT_SEC = 25
MARKER_MAX_APPROACH_CM = 80     # 识别失败时的最大前进量，需按起点位置收紧
BALL_SEARCH_TIMEOUT_SEC = 45
BUZZER_PIN = 18                 # BCM 编号；核对实际蜂鸣器接线
YELLOW_SIDE_CM = 15
YELLOW_PASS_CM = 40
BLACK_APPROACH_CM = 15         # 报警后靠近到机械臂可击倒的位置，需实测
BLACK_RETREAT_CM = 2
LINE_AFTER_YELLOW_CM = 20      # 黄牌绕行后到赋值区转向点，需实测
LINE_AFTER_BLACK_CM = 47       # 黑牌击倒后到同一转向点，需实测
VALUE_ZONE_ENTRY_CM = 15       # 从黑线向画面右侧进入赋值区，需实测
CUP_3X_GREEN_CM = 25           # 从绿球位到左侧 3 倍杯，需实测
CUP_3X_RED_CM = 20             # 从红球位到左侧 3 倍杯，需实测

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

    def move_cm(self, axis, distance_cm):
        """按实测速度换算运动时间；distance_cm 的正负决定方向。"""
        if distance_cm == 0:
            return
        if axis == 'x':
            speed = 30 if distance_cm > 0 else -20
            cm_per_sec = FORWARD_CM_PER_SEC if distance_cm > 0 else BACKWARD_CM_PER_SEC
        elif axis == 'y':
            speed = 20 if distance_cm > 0 else -20
            cm_per_sec = SIDE_CM_PER_SEC
        else:
            raise ValueError(f"Unsupported movement axis: {axis}")
        if cm_per_sec <= 0:
            raise ValueError("Please calibrate movement speed before running")
        self.move_dog(axis, speed, abs(distance_cm) / cm_per_sec)

    def beep_alarm(self):
        """蜂鸣 0.4 秒；BCM 18 只是接线初始配置。"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
        try:
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            time.sleep(0.4)
        finally:
            GPIO.output(BUZZER_PIN, GPIO.LOW)

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

    def arm_grab_ball(self,cup_offset_cm,cup_side='left'):

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
        # 面向赋值区时，左杯是 3 倍杯；本次任务两个球都送到左杯。
        lateral_cm = cup_offset_cm if cup_side == 'left' else -cup_offset_cm
        self.move_cm('y', lateral_cm)
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
        self.move_cm('y', -lateral_cm)  # 回到抓球线附近
        
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

    def get_ball(self,ball_color,cup_offset_cm,target_ball):
        # BallRecognition 内部按 320x240 设计（target_x=160），
        # 而 take_photo 把摄像头切到了 640x480，这里需要切回。
        if XGO_edu is not None and XGO_edu.cap is not None and int(XGO_edu.cap.get(3)) != 320:
            XGO_edu.cap.set(3, 320)
            XGO_edu.cap.set(4, 240)
        #self.adjust_direction() #调整方位
        deadline = time.monotonic() + BALL_SEARCH_TIMEOUT_SEC
        while True:
            if time.monotonic() >= deadline:
                self.stop()
                raise TimeoutError("Ball could not be aligned within the search timeout")
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
                if time.monotonic() >= deadline:
                    self.stop()
                    raise TimeoutError("Ball recognition did not return three valid frames")
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

            if abs(x_average - target_x) <=14 and abs(blob_average - target_ball)<=2:
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
        self.arm_grab_ball(cup_offset_cm, cup_side='left')
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
        param1=100, param2=50, minRadius=10, maxRadius=100
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
        param1=100, param2=20, minRadius=5, maxRadius=100
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
        raise RuntimeError("Camera frame unavailable; stop before approaching the marker")
    return frame


def line_offset(image):
    """寻找画面下方的黑色引导线，返回相对画面中心的像素偏差。"""
    height, width = image.shape[:2]
    top, bottom = int(height * 0.65), int(height * 0.95)
    gray = cv2.cvtColor(image[top:bottom], cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, 75)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) >= 80 and h >= 12 and h >= w * 0.7 and w <= width * 0.18:
            candidates.append((abs(x + w / 2 - width / 2), x + w / 2 - width / 2))
    return min(candidates)[1] if candidates else None


def follow_line_step(image, forward_cm=2):
    offset = line_offset(image)
    if offset is None:
        robot_dog.stop()
        raise RuntimeError("Black guide line lost; stop rather than driving blindly")
    if abs(offset) > 40:
        # 线在画面右边时，机械狗向右横移；反之向左。
        robot_dog.move_cm('y', -1 if offset > 0 else 1)
    robot_dog.move_cm('x', forward_cm)


def follow_line_cm(distance_cm):
    traveled = 0
    while traveled < distance_cm:
        image = take_photo()
        step = min(2, distance_cm - traveled)
        follow_line_step(image, step)
        traveled += step


def identify_marker_at_20cm():
    """边循线边观察立牌，连续三帧确认颜色和约 20cm 的视觉距离。"""
    deadline = time.monotonic() + MARKER_TIMEOUT_SEC
    recent_colors = []
    traveled_cm = 0
    near_marker_seen = False
    while time.monotonic() < deadline and traveled_cm < MARKER_MAX_APPROACH_CM:
        image = take_photo()
        circles, _, _ = detect_circles_multi_color(image)
        height, width = image.shape[:2]
        candidates = [c for c in circles
                      if abs(int(c['x']) - width / 2) < width * 0.30
                      and height * 0.10 < int(c['y']) < height * 0.80]
        marker = max(candidates, key=lambda c: int(c['r']), default=None)
        if marker:
            if len({c['color'] for c in candidates
                    if int(c['r']) >= int(marker['r']) * 0.7}) > 1:
                recent_colors.clear()
                time.sleep(0.1)  # 黄黑结果冲突时原地重拍
                continue
            recent_colors.append(marker['color'])
            recent_colors = recent_colors[-3:]
            if (len(recent_colors) == 3
                    and len(set(recent_colors)) == 1
                    and int(marker['r']) >= MARKER_RADIUS_AT_20CM):
                robot_dog.stop()
                print(f"Marker confirmed: {marker['color']}, radius={marker['r']} px")
                return marker['color']
            if int(marker['r']) >= MARKER_RADIUS_AT_20CM:
                near_marker_seen = True
                time.sleep(0.1)  # 已到报警距离，等待多帧确认，不再向立牌靠近
                continue
        else:
            recent_colors.clear()
        if near_marker_seen:
            if marker is None:
                robot_dog.stop()
                raise RuntimeError("Marker lost at close range; stop before contact")
            time.sleep(0.1)  # 已接近立牌后绝不再前进
            continue
        follow_line_step(image, forward_cm=1)
        traveled_cm += 1
    robot_dog.stop()
    raise TimeoutError("Marker was not confirmed within the approach timeout")

# 调用函数





if __name__ == "__main__":
    if not CALIBRATION_CONFIRMED:
        raise RuntimeError("Calibrate marker size, movement speed, buzzer pin and cup positions first")
    
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
    
    try:
        marker_color = identify_marker_at_20cm()
        robot_dog.beep_alarm()

        if marker_color == 'Yellow':
            # 第二张图：先向画面右侧 15cm，前进 40cm，再向左 15cm 回到黑线。
            robot_dog.move_cm('y', -YELLOW_SIDE_CM)
            robot_dog.move_cm('x', YELLOW_PASS_CM)
            robot_dog.move_cm('y', YELLOW_SIDE_CM)
            line_to_value_cm = LINE_AFTER_YELLOW_CM
        elif marker_color == 'Black':
            # 第三张图：沿线靠近立牌，机械臂击倒，后退 2cm 后继续循线。
            robot_dog.move_cm('x', BLACK_APPROACH_CM)
            robot_dog.dog.claw(0)
            robot_dog.move_arm(120, -20)
            time.sleep(2)
            robot_dog.move_arm(100, 100)
            robot_dog.move_cm('x', -BLACK_RETREAT_CM)
            line_to_value_cm = LINE_AFTER_BLACK_CM
        else:
            raise RuntimeError(f"Unexpected marker color: {marker_color}")

        follow_line_cm(line_to_value_cm)
        robot_dog.move_cm('y', -VALUE_ZONE_ENTRY_CM)

        # 到达赋值区后抓绿、红两球；两个球都投入左侧的 3 倍杯。
        # 从黑线进入赋值区的距离，以及每个球到杯位的距离，需要在现场再标定。
        robot_dog.get_ball(green_color_mask, CUP_3X_GREEN_CM, target_ball_green)
        time.sleep(1)
        robot_dog.get_ball(red_color_mask, CUP_3X_RED_CM, target_ball_red)
        time.sleep(1)

        # 保留旧版返程动作；正式运行前应验证机械狗完全回到出发区。
        robot_dog.move_dog('y', 20, 4.5)
        time.sleep(1)
        robot_dog.move_dog('x', -30, 11)
        robot_dog.dog.reset()
        robot_dog.stop()
    except Exception:
        robot_dog.stop()
        raise

# 摄像头由 XGOEDU/Picamera2Capture 在进程退出时自动 release（atexit）

