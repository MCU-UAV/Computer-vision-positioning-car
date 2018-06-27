from collections import  deque  
import numpy as np
import time
from math import *
import cv2
import serial

def nothing(x):
    pass

#设定颜色阈值，HSV空间  
debug = 0
trickUpper1 = np.array([42, 238, 255])
trickLower1 = np.array([22, 47, 115])

trickUpper2 = np.array([118, 255, 255])
trickLower2 = np.array([97, 45, 0])

#初始化追踪点的列表  
mybuffer = 16  
pts = deque(maxlen=mybuffer)  
counter = 0
targX = 0
targY = 0
setFlag = False

#定义字体
font = cv2.FONT_HERSHEY_SIMPLEX

def nothing(x):
    pass


def OnMouseAction(event,x,y,flags,param):
    global targX,targY,setFlag
    if event == cv2.EVENT_LBUTTONDBLCLK: #双击
        setFlag = True
        targX = x
        targY = y
        #command = b'#@%.2f,%.2f\n'%(x,y)
        #n = ser.write(command)
        #line = ser.readline()
        #print(line)

    elif event == cv2.EVENT_RBUTTONDOWN: #右击取消
        setFlag = False
        command = b'ST\n'    #发送特殊字符串给下位机以结束任务
        print(command)
        n = ser.write(command)
        #line = ser.read(1)   # read a '/n' terminated line
        #print(line)

#打开摄像头  
camera = cv2.VideoCapture(1)
ser = serial.Serial('COM7',9600)  



#等待三秒  
time.sleep(3)

#开启窗口
cv2.namedWindow('image')
cv2.namedWindow('set1')
cv2.namedWindow('set2')
cv2.namedWindow('temp1')
cv2.namedWindow('temp2')

cv2.createTrackbar('Hmax','set1',0,180,nothing)
cv2.createTrackbar('Hmin','set1',0,180,nothing)

cv2.createTrackbar('Smax','set1',0,255,nothing)
cv2.createTrackbar('Smin','set1',0,255,nothing)

cv2.createTrackbar('Vmax','set1',0,255,nothing)
cv2.createTrackbar('Vmin','set1',0,255,nothing)


#################################
cv2.createTrackbar('Hmax','set2',0,180,nothing)
cv2.createTrackbar('Hmin','set2',0,180,nothing)

cv2.createTrackbar('Smax','set2',0,255,nothing)
cv2.createTrackbar('Smin','set2',0,255,nothing)

cv2.createTrackbar('Vmax','set2',0,255,nothing)
cv2.createTrackbar('Vmin','set2',0,255,nothing)

cv2.setMouseCallback('image',OnMouseAction)


global targYaw,err_pos
targYaw = 0.0
err_pos =0.0
#遍历每一帧，检测颜色
while True:
    
    #读取帧  
    (ret, frame) = camera.read()  
    #判断是否成功打开摄像头  
    if not ret:  
        print ('No Camera')  
        break  
  
    #转到HSV空间  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
    #根据阈值构建掩膜
  
    Hmax1 = cv2.getTrackbarPos('Hmax','set1')
    Hmin1 = cv2.getTrackbarPos('Hmin','set1')
    Smax1 = cv2.getTrackbarPos('Smax','set1')
    Smin1 = cv2.getTrackbarPos('Smin','set1')
    Vmax1 = cv2.getTrackbarPos('Vmax','set1')
    Vmin1 = cv2.getTrackbarPos('Vmin','set1')

    Hmax2 = cv2.getTrackbarPos('Hmax','set2')
    Hmin2 = cv2.getTrackbarPos('Hmin','set2')
    Smax2 = cv2.getTrackbarPos('Smax','set2')
    Smin2 = cv2.getTrackbarPos('Smin','set2')
    Vmax2 = cv2.getTrackbarPos('Vmax','set2')
    Vmin2 = cv2.getTrackbarPos('Vmin','set2')

    #取消注释可动态修改阈值
    if(debug == 1):
        trickUpper1[:] = [Hmax1,Smax1,Vmax1]
        trickLower1[:] = [Hmin1,Smin1,Vmin1]
    elif(debug == 2):
        trickUpper2[:] = [Hmax2,Smax2,Vmax2]
        trickLower2[:] = [Hmin2,Smin2,Vmin2]

    #HSV分割
    mask1 = cv2.inRange(hsv, trickLower1, trickUpper1)
    mask2 = cv2.inRange(hsv, trickLower2, trickUpper2)

    #中值滤波
    mask1 =cv2.medianBlur(mask1,5)
    mask2 =cv2.medianBlur(mask2,5)
    #便于输出调节阈值
    test1 = mask1
    test2 = mask2
    #腐蚀操作  
    mask1 = cv2.erode(mask1, None, iterations=2)
    mask2 = cv2.erode(mask2, None, iterations=2)
    
    #膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点  
    mask1 = cv2.dilate(mask1, None, iterations=2)
    mask2 = cv2.dilate(mask2, None, iterations=2)
    
    cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    #初始化跟踪体质心
    center1 = None
    center2 = None

    #如果存在轮廓
    if len(cnts1) > 0 and len(cnts2) > 0:
        #找到面积最大的轮廓
        c1 = max(cnts1,key = cv2.contourArea)
        c2 = max(cnts2,key = cv2.contourArea)
        #确定面积最大的轮廓的外接圆
        ((x1,y1),radius1) = cv2.minEnclosingCircle(c1)
        ((x2,y2),radius2) = cv2.minEnclosingCircle(c2)
        #计算轮廓的矩
        M1 = cv2.moments(c1)
        M2 = cv2.moments(c2)
        #计算质心
        center1 = (int(M1["m10"]/M1["m00"]), int(M1["m01"]/M1["m00"]))
        center2 = (int(M2["m10"]/M2["m00"]), int(M2["m01"]/M2["m00"]))  
        #只有当半径大于10时，才执行画图  
        if radius1 > 1 and radius2 > 1:
            #绘制最小外接圆
            cv2.circle(frame, (int(x1), int(y1)), int(radius1), (0, 255, 255), 2)
            cv2.circle(frame, (int(x2), int(y2)), int(radius2), (0, 255, 255), 2)  
            #绘制质心
            cv2.circle(frame, center1, 5, (0, 0, 255), -1)
            cv2.circle(frame, center2, 5, (0, 0, 255), -1)
            #绘制质心连线
            cv2.line(frame,center1,center2 ,(0,255,0),2)
            #求中点
            x0 = int((center1[0] + center2[0])/2)
            y0 = int((center1[1] + center2[1])/2)
            #求垂直平分线上的一点
            toward = ((center1[1]-y0)+x0,-(center1[0] - x0) + y0)

            vectorYaw = (toward[0] - x0,toward[1] - y0)
            #保存路径
            pts.appendleft((x0,y0))
            #计算偏航角
            yaw = atan2(vectorYaw[0],-vectorYaw[1]) *(180/pi)
            #绘制中垂线
            cv2.line(frame,(x0,y0),toward ,(0,255,0),2)
            #绘制中点
            cv2.circle(frame, (x0,y0), 5, (0, 0, 255), -1)
            #绘制直角坐标轴
            cv2.line(frame,(x0,y0-50),(x0,y0+50) ,(255,255,255),1)
            cv2.line(frame,(x0-50,y0),(x0+50,y0) ,(255,255,255),1)
            #绘制角度标识
            cv2.ellipse(frame,(x0,y0),(20,20),-90,0,yaw,(0,255,0),2)

            if(setFlag): #设置了目的地
           
                command = b'%d,%d,%d,%d,%d,\n'%(yaw*100,x0,y0,targX,targY)
                targYaw = yaw + atan2(x0 - targX, y0 - targY) * 57.3
                if(targYaw < -180): targYaw = targYaw + 360;
                if(targYaw > 180): targYaw = targYaw - 360;
                err_pos = sqrt((x0 - targX) * (x0 - targX) + (y0 - targY) * (y0 - targY))
                n = ser.write(command)
                #line = ser.readline()   # read a '/n' terminated line
                #print(line)
                cv2.line(frame,(x0,y0),(targX,targY),(255,255,0),2)
                cv2.circle(frame, (targX,targY), 5, (255, 125, 0), -1)
            
            #绘制路径
            for i in range(1, len(pts)):  
                if pts[i - 1] is None or pts[i] is None:  
                    continue  
                #计算所画小线段的粗细  
                thickness = int(np.sqrt(mybuffer / float(i + 1)) * 2.5)  
                #画出小线段  
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)  
            #叠加输出关键信息
            cv2.putText(frame,'Find',(50,50), font, 1,(255,25,255),2,cv2.LINE_AA)
            cv2.putText(frame,'Yaw:%.2f'% yaw,(50,100), font, 1,(255,25,255),2,cv2.LINE_AA)
            cv2.putText(frame,'X:%d Y:%d'% (x0,y0),(50,150), font, 1,(255,25,255),2,cv2.LINE_AA)
            cv2.putText(frame,'TargYawErr:%.2f'% (targYaw),(50,200), font, 1,(255,25,255),2,cv2.LINE_AA)
            cv2.putText(frame,'err_pos:%.2f'% (err_pos),(50,250), font, 1,(255,25,255),2,cv2.LINE_AA)
        else:#半径太小  
            cv2.putText(frame,'The radius is too small!',(50,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    else:#如果图像中没有检测到无人机        
        cv2.putText(frame,'No object',(50,50), font, 1,(255,255,255),2,cv2.LINE_AA)
  
           
    cv2.imshow('image', frame)  
    cv2.imshow('temp1',test1)
    cv2.imshow('temp2',test2)
    #键盘检测，检测到esc键退出  
    k = cv2.waitKey(1)&0xFF  
    counter += 1  
    if k == 27:  
        break
    elif k == 'w'or k == 'W':
        ser.write('w')
    elif k == 'a'or k == 'A':
        ser.write('a')
    elif k == 's'or k == 'S':
        ser.write('s')
    elif k == 'd'or k == 'D':
        ser.write('d')

#关闭串口
ser.close()
#摄像头释放  
camera.release()  
#销毁所有窗口  
cv2.destroyAllWindows()  
