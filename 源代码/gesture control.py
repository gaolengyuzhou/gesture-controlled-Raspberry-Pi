import numpy as np
import cv2
import Adafruit_PCA9685
import  RPi.GPIO as GPIO
import time
import math

cap = cv2.VideoCapture(0)
#---------------------------------------
# 将视频尺寸减小到300x300，这样rpi处理速度就会更快
cap.set(3,320)
cap.set(4,320)

#-------------------------树莓派小车电机驱动初始化
PWMA = 18
AIN1   =  22
AIN2   =  27

PWMB = 23
BIN1   = 25
BIN2  =  24

BtnPin  = 19
Gpin    = 5
Rpin    = 6

#-------------------------初始化舵机
pwm = Adafruit_PCA9685.PCA9685()
# 配置PWM最小值和最大值
servo_min = 150
servo_max = 600
   
# 辅助功能，使设置舵机脉冲宽度更简单
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000
    pulse_length //= 60
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def set_servo_angle(channel,angle):
    angle=4096*((angle*11)+500)/20000
    pwm.set_pwm(channel,0,int(angle))

# 频率设置为50hz，适用于舵机系统
pwm.set_pwm_freq(50)
set_servo_angle(5,90)  # 底座舵机90
set_servo_angle(4,115)  # 顶部舵机145

time.sleep(0.5)

#------------------------树莓派小车运动函数
def t_up(speed,t_time):
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,True) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,True) #BIN1
        time.sleep(t_time)
        
def t_stop(t_time):
        L_Motor.ChangeDutyCycle(0)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,False) #AIN1

        R_Motor.ChangeDutyCycle(0)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,False) #BIN1
        time.sleep(t_time)
        
def t_down(speed,t_time):
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,True)#AIN2
        GPIO.output(AIN1,False) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,True)#BIN2
        GPIO.output(BIN1,False) #BIN1
        time.sleep(t_time)

def t_left(speed,t_time):
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,True) #AIN1
        speed =speed + 10
        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,True) #BIN1
        time.sleep(t_time)

def t_right(speed,t_time):
        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,True) #BIN1    
        speed = speed + 10
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,True) #AIN1    
        time.sleep(t_time)
        
def keysacn():
    val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == False:
        val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == True:
        time.sleep(0.01)
        val = GPIO.input(BtnPin)
        if val == True:
            GPIO.output(Rpin,1)
            while GPIO.input(BtnPin) == False:
                GPIO.output(Rpin,0)
        else:
            GPIO.output(Rpin,0)

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
GPIO.setup(Gpin, GPIO.OUT)     # 设置绿色Led引脚模式输出
GPIO.setup(Rpin, GPIO.OUT)     # 设置红色Led引脚模式输出
GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # 设置输入BtnPin模式，拉高至高电平(3.3V)

GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

L_Motor= GPIO.PWM(PWMA,100)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,100)
R_Motor.start(0)

keysacn()
#----------------------------------------
while(1):
        
    try:  # 如果它在窗口中找不到任何东西，因为找不到最大面积的轮廓，就会出现错误
          # 因此，此try错误语句
          
        ret, frame = cap.read()
        frame=cv2.flip(frame,1)
        kernel = np.ones((1,1),np.uint8)
        
    # 定义感兴趣的区域
        roi=frame[0:300, 0:250]
        cv2.rectangle(frame,(0,0),(250,300),(0,255,0),0)

        blur = cv2.bilateralFilter(roi,20,75,75)

        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    # 定义HSV中肤色的范围
        lower_skin = np.array([0,20,40], dtype=np.uint8)
        upper_skin = np.array([20,255,255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
    # 外推手以填补其中的黑点

    # 找到轮廓
        draw = np.zeros(roi.shape,np.uint8)*255
        _,contours,hierarchy= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(draw,contours,-1,(0,0,255),5)
    # 找到最大面积的轮廓（手）
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
        
    # 大概轮廓
        epsilon = 0.0005*cv2.arcLength(cnt,True)
        approx= cv2.approxPolyDP(cnt,epsilon,True)

    # 围绕手做凸包
        hull = cv2.convexHull(cnt)
        
    # 计算手的面积和凸包的面积
        areahull = cv2.contourArea(hull)
        areacnt = cv2.contourArea(cnt)
      
    # 查找凸包中未被手覆盖的区域的百分比
        arearatio=((areahull-areacnt)/areacnt)*100
    
    # 发现凸包相对于手的缺陷
        hull = cv2.convexHull(approx, returnPoints=False)
        defects = cv2.convexityDefects(approx, hull)

        l=0
        
    # 查找编号的代码 手指造成的缺陷
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(approx[s][0])
            end = tuple(approx[e][0])
            far = tuple(approx[f][0])
            pt= (100,180)

            # 求出三角形所有边的长度
            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            s = (a+b+c)/2
            ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
            
            # 点与凸包之间的距离
            d=(2*ar)/a
            
            # 在这里应用余弦规则
            angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57

            # 忽略大于90度的角度并忽略非常靠近凸包的点（它们通常是由于噪声而来）
            if angle <= 90 and d>30:
                l += 1
                cv2.circle(roi, far, 8, [255,0,0], -1)
            
            # 在手周围画线
            cv2.line(roi,start, end, [0,255,0], 2)

        l+=1
        
        # 打印范围内的相应手势,并下达相应运动
        font = cv2.FONT_HERSHEY_SIMPLEX
        if l==1:
            if areacnt<2000:
                cv2.putText(frame,'Put hand in the box',(0,50), font, 1, (0,0,255), 3, cv2.LINE_AA)
                t_stop(0)
            else:
                if arearatio<12:
                    cv2.putText(frame,'stop',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    t_stop(0)
                else:
                    cv2.putText(frame,'go go go',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                    t_up(30,0)
        elif l==3:
            if arearatio<27:
                cv2.putText(frame,'left',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                t_left(20,0)
            else:
                cv2.putText(frame,'left',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)  
                t_left(20,0)
        elif l==4:
            cv2.putText(frame,'right',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
            t_right(20,0)
        elif l==5:
            cv2.putText(frame,'break',(0,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
            t_down(20,0)
        else :
            cv2.putText(frame,'reposition',(10,50), font, 1, (0,0,255), 3, cv2.LINE_AA)
            t_stop(0)
            
        # 显示重要窗户
        cv2.imshow('mask',mask)
        cv2.imshow('draw',draw)
        cv2.imshow('frame',frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            L_Motor.stop()
            R_Motor.stop()
            GPIO.cleanup()
            break
    except:
        pass

cv2.destroyAllWindows()
cap.release()    
    




