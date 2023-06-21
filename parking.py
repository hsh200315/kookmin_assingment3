#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 

#planning 할 때, 경로 위 각각의 점들의 x,y좌표를 담을 list
rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]
#savepoint에 도착했는지 판단할 수 있게하는 변수
p = 0
# 처음에 적절한 경로를 생성하지 못해 후진을 하는 경우를 판단하는 변수
k = 0
#후진할 때, 충분히 후진할 수 있도록 반복문에 사용될 변수
timer = 100
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
P_MIDDLE = (1082,115) #주차라인 진입과 끝의 중간, 차가 제대로 주차되었는지 판단하기 위해 사용

#경로 추적을 위해 사용되는 함수, list와 value를 넣으면 list내의 값 중 value와 가장 가까운 값의 index를 return한다
#rx와 차의 현재 x좌표를 넣어, 거리를 계산하는데 사용
def findNearNum(exList1,value1):
    global p
    if p==0:
        minValue = min(exList1, key=lambda x:abs(x-value1))
    else :
        minValue = min(exList1[25001:], key=lambda x:abs(x-value1))
    minIndex = exList1.index(minValue)
    answer = minIndex
    return answer

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, inclination,StartPointX,StartPointY,SavePointY,SavePointX,p,timer,k
    # 원래 저장되어 있던 경로를 삭제
    del rx[:]
    del ry[:]
    # rx와 ry에 담겨있는 점의 개수
    count = 0
    # tracking에 사용되는 변수들 초기화
    p = 0
    # 
    k = 0
    timer = 100
    print("Start Planning")
    # syaw가 오른 쪽을 볼 때 0을 갖도록 각도 변환, > = 0, < = 180, ^ = 90
    syaw = syaw - 270
    if syaw < 0:
        syaw = -1*syaw


    # 차의 실질적인 출발 위치, syaw에 따라 차가 출발하는 장소가 달라지므로 값 계산 후 변수에 저장
    StartPointX = sx+70*math.cos(math.pi*(syaw/180))
    StartPointY = sy-70*math.sin(math.pi*(syaw/180))
    # P_ENTRY에서 약 70만큼 떨어져 있는 점, 차 출발 직후 이 점을 향해 이동한다.
    # 이 점에 도착히면, p의 값이 변하여 주차라인에 주차 시작
    SavePointX = 976
    SavePointY = 221

    # 경로 생성
    rx.append(StartPointX) 
    ry.append(StartPointY)
    # StartPoint에서 SavePoint까지 경로 생성ㅇ
    while count < 20000:
        rx.append(rx[count]+((SavePointX-StartPointX)/20000))
        ry.append(ry[count]+((SavePointY-StartPointY)/20000))
        count += 1
    rx.append(SavePointX)
    ry.append(SavePointY)
    count+=1
    # SavePoint에서 P_ENTRY까지 경로 생성
    while count < 25000:
        rx.append(rx[count]+((P_ENTRY[0]-SavePointX)/5000))
        ry.append(ry[count]+((P_ENTRY[1]-SavePointY)/5000))
        count += 1
    rx.append(P_ENTRY[0])
    ry.append(P_ENTRY[1])
    count+= 1
    # P_ENTRY에서 P_MIDDLE까지 경로 생성
    while count < 28000:
        rx.append(rx[count]+((P_MIDDLE[0]-P_ENTRY[0])/3000))
        ry.append(ry[count]+((P_MIDDLE[1]-P_ENTRY[1])/3000))
        count += 1
    rx.append(P_MIDDLE[0])
    ry.append(P_MIDDLE[1])
    # StartPoint와 Save Point 사이 경로의 기울기
    inclination = (-1)*(SavePointY-StartPointY)/(SavePointX-StartPointX)
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, inclination,p,timer,k, carlength
    angle = 0 # -50 ~ 50 50 right, -50 left
    speed = 50 # -50 ~ 50
    
    # rx에서 현재 차의 x좌표와 가장 가까운 값의 인덱스를 찾아 저장한다.
    NearNumIdx = findNearNum(rx,x)

    # line_x 는 rx 중 현재 차의 x값과 가장 가까운 점의 x 좌표, line_y는 그 점의 y좌표를 저장한다.
    line_x = rx[NearNumIdx]
    line_y = ry[NearNumIdx]
    # P_END와 P_ENTRY를 통해 주차라인의 각도를 구함.
    parkingAngle = -(P_END[0] - P_ENTRY[0])/(P_END[1] - P_ENTRY[1])
    parkingAngle = math.atan(parkingAngle)*180/math.pi
    # 차의 앞 부분 좌표. x,y는 차의 중앙의 좌표를 가져오기 때문에 차의 앞부분 좌표를 계산
    fx = x + (math.cos(math.pi*((yaw)/180)))*70
    fy = y - (math.sin(math.pi*((yaw)/180)))*70
    # Planning에서 계산한 StartPoint와 SavePoint의 기울기를 이용해, 경로의 각도를 구함
    theta = math.atan(inclination)*180/math.pi
    # 각도가 90도를 넘어가는 경우 -가 되는 각도를 +로 전환 바꿈(처음에 후진 해야 할 때 고려)
    if theta <0:
        theta = theta+180 
    # 주차지에 들어가기 전 주차지 근처에 갔다는 것을 인식하기 위한 지점    
    distanceToSavePoint = math.sqrt((SavePointX - fx)**2 + (SavePointY-fy)**2)
    # P_End 까지의 거리
    distanceToP_End = math.sqrt((P_END[0] - x) ** 2 + (P_END[1] - y) ** 2)
    # 루트의 각도가 90이하인 경우 차량과 루트 사이의 최단거리
    if theta <= 90: 
        distance = (math.sqrt((line_x - x) ** 2 + (line_y - y) ** 2)) * (math.cos(math.pi * (theta/180)))
    # 루트의 각도가 90보다 큰 경우 차량과 루트 사이의 최단거리
    else :
        distance = (math.sqrt((line_x - x) ** 2 + (line_y - y) ** 2)) * (math.cos(math.pi*((180-theta)/180)))
    # 거리가 음수로 나오는 경우 양수로 변환
    if distance < 0:
        distance = -distance
    #차량과 생성된 길의 각도 차이
    angle_diff_routine = yaw - theta
    #차량과 주차지의 각도 차이
    angle_diff_parking = yaw - parkingAngle
    # 후진을 지속적으로 하기 위한 조건문
    if timer < 100:
        # 차량이 주차지보다 아래에 있는 경우
        if y > P_END[1]:
            angle = 10
            speed = -50
            timer += 1
        # 차량이 주차지보다 위에 있는 경우
        elif y < P_END[1]:
            angle = -10
            speed = -50
            timer += 1
    # savePoint에 도착하지 않았을 때
    elif p == 0:
        # 적절한 경로를 생성하기 좋은 위치에 도달한 상태가 될 때까지 후진 지속
        if x > SavePointX and abs(yaw - theta) > 20 and (k==0 or k==1):
            # replanning을 해야 한다는 것을 표현하기 위한 값
            k = 1
            # 후진 시행을 위한 조건문
            if angle_diff_routine < 0:
                angle = 10
                speed = -50
        # savePoint와의 거리가 가까운 경우 p를 1로 변경해 parking을 시작
        elif distanceToSavePoint < 30:
            p = 1
        # savePoint와의 거리가 아직 먼 경우
        else:
            # 후진해서 차 위치를 다시 잡은 경우 replanning 시행
            if k==1:
                speed = 0
                del rx[:]
                del ry[:]
                count = 0
                p = 0
                timer = 100

                # 처음 시작할 때 차량의 처음 앞 부분 위치 좌표
                StartPointX = x+70*math.cos(math.pi*(yaw/180))
                StartPointY = y-70*math.sin(math.pi*(yaw/180))

                # 경로 생성 시작
                rx.append(StartPointX) 
                ry.append(StartPointY)
                # StartPoint에서부터 SavePoint까지의 경로 생성
                while count < 20000:
                    rx.append(rx[count]+((SavePointX-StartPointX)/20000))
                    ry.append(ry[count]+((SavePointY-StartPointY)/20000))
                    count += 1
                rx.append(SavePointX)
                ry.append(SavePointY)
                count+=1
                # SavePoint에서 주차라인 진입 지점 사이의 경로 생성
                while count < 25000:
                    rx.append(rx[count]+((P_ENTRY[0]-SavePointX)/5000))
                    ry.append(ry[count]+((P_ENTRY[1]-SavePointY)/5000))
                    count += 1
                rx.append(P_ENTRY[0])
                ry.append(P_ENTRY[1])
                count+= 1
                # 주차라인 진입 지점에서 중간 지점 사이의 경로 생성
                while count < 28000:
                    rx.append(rx[count]+((P_MIDDLE[0]-P_ENTRY[0])/3000))
                    ry.append(ry[count]+((P_MIDDLE[1]-P_ENTRY[1])/3000))
                    count += 1
                rx.append(P_MIDDLE[0])
                ry.append(P_MIDDLE[1])
                inclination = (-1)*(SavePointY-StartPointY)/(SavePointX-StartPointX)
                # replanning이 다시 이루어지지 않게 하기 위한 값 변경
                k = 2
            # theta >= 90이면서 라인 아래에 있는 경우
            if theta >= 90 and y >= line_y:
                # 차량과 라인 사이의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_routine < -15:
                    angle = -50
                elif angle_diff_routine > 15:
                    angle = 50
                elif distance > 12 and angle_diff_routine < 0:
                    angle = -50
                elif distance > 12 and angle_diff_routine > 0:
                    angle = 8
                elif 7 < distance < 12 and angle_diff_routine > 0:
                    angle = 8
                elif 7 < distance < 12 and angle_diff_routine < 0:
                    angle = 8
                elif 2 < distance < 7 and angle_diff_routine > 0:
                    angle = 8
                elif 2 < distance < 7 and angle_diff_routine < 0:
                    angle = 8
                elif distance < 2:
                    if angle_diff_routine > 2:
                        angle = 2
                    elif angle_diff_routine < -2:
                        angle = -2
                    else:
                        if angle_diff_routine < 2:
                            angle = 1
                        elif angle_diff_routine > -2:
                            angle = -1
            # theta >= 90 이면서 라인 위에 있는 경우
            elif  theta >= 90 and y < line_y:
                # 차량과 라인 사이의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_routine <-20:
                    angle = -50
                elif distance > 12 and angle_diff_routine > 0:
                    angle = 30
                elif distance > 12 and angle_diff_routine < 0:
                    angle = -40
                elif distance > 12:
                    angle = -20
                elif 7 < distance < 12 and angle_diff_routine > 0:
                    angle = 15
                elif 7 < distance < 12 and angle_diff_routine < 0:
                    angle = -40
                elif 2 < distance < 7 and angle_diff_routine > 0:
                    angle = 12
                elif 2 < distance < 7 and angle_diff_routine < 0:
                    angle = -40
                elif distance < 2:
                    if angle_diff_routine > 2:
                        angle = 30
                    elif angle_diff_routine < -2:
                        angle = -15
                    else:
                        if angle_diff_routine < 0:
                            angle = -1
                        elif angle_diff_routine > 0:
                            angle = 1
            # 차량이 라인보다 아래에 있는 경우
            elif y >= line_y: 
                # 차량과 라인 사이와의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_routine < -20:
                    angle = -50
                elif angle_diff_routine > 20:
                    angle = 50
                elif distance > 12 and angle_diff_routine < 5:
                    angle = -20
                elif distance > 12 and angle_diff_routine > 5:
                    angle = 15
                elif 7 < distance < 12 and angle_diff_routine > 4:
                    angle = 15
                elif 7 < distance < 12 and angle_diff_routine < 4:
                    angle = -10
                elif 2 < distance < 7 and angle_diff_routine > 3:
                    angle = 7
                elif 2 < distance < 7 and angle_diff_routine < 3:
                    angle = -5
                elif distance < 2:
                    if angle_diff_routine > 2:
                        angle = 2
                    elif angle_diff_routine < -2:
                        angle = -2
                    else:
                        if angle_diff_routine < 0:
                            angle = -1
                        elif angle_diff_routine > 0:
                            angle = 1 
            # 차량이 라인보다 위에 있는 경우
            elif y < line_y:
                # 차량과 라인 사이와의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_routine <-20:
                    angle = -50
                elif distance > 12 and angle_diff_routine > 0:
                    angle = 50
                elif distance > 12 and angle_diff_routine < -10:
                    angle = 5
                elif distance > 12:
                    angle = 10
                elif 7 < distance < 12 and angle_diff_routine > 0:
                    angle = 15
                elif 7 < distance < 12 and angle_diff_routine < -5:
                    angle = -10
                elif 2 < distance < 7 and angle_diff_routine > 0:
                    angle = 10
                elif 2 < distance < 7 and angle_diff_routine < -4:
                    angle = -5
                elif distance < 2:
                    if angle_diff_routine > 2:
                        angle = 2
                    elif angle_diff_routine < -2:
                        angle = -2
                    else:
                        if angle_diff_routine < 0:
                            angle = -1
                        elif angle_diff_routine > 0:
                            angle = 1
    # SavePoint에 도착한 이후 주차를 시작하는 상태
    else:
        # 주차지에 제대로 도착한 경우 차를 정지시킴
        if (x < P_END[0] and y > P_END[1]) and distanceToP_End < 60:
            speed = 0
        # 주차지에 제대로 도착하지 못하고 지나가게 된 경우 적절히 후진
        elif distanceToP_End < 60 and (yaw > parkingAngle + 10 or yaw < parkingAngle - 10):
            import time
            time.sleep(1)
            speed = -50
            if y > P_END[1]:
                angle = -10
            elif y < P_END[1]:
                angle = 10
        # 후진이 이루어진 이후 다시 직진하면서 주차지에 맞게 들어가는 과정
        elif distanceToP_End > 0 and x < P_END[0]:
            # 차량이 라인보다 아래에 있는 경우
            if y >= line_y: 
                # 차량과 주차지와의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_parking<-25:
                    angle = -50
                elif angle_diff_parking>25:
                    angle = 50
                elif angle_diff_parking > -1 and angle_diff_parking <1:
                    angle = 0
                elif distance > 30 and angle_diff_parking < 5:
                    angle = -10
                elif distance > 30 and angle_diff_parking > 5:
                    angle = -5
                elif 10 < distance < 30 and angle_diff_parking > 4:
                    angle = 10
                elif 10 < distance < 30 and angle_diff_parking < 4:
                    angle = -10
                elif 5 < distance < 10 and angle_diff_parking > 3:
                    angle = 8
                elif 5 < distance < 10 and angle_diff_parking < 3:
                    angle = -8
                elif distance < 5:
                    if angle_diff_parking > 2:
                        angle = 15
                    elif angle_diff_parking < -2:
                        angle = -15
                    else:
                        if angle_diff_parking < 0:
                            angle = -3
                        elif angle_diff_parking > 0:
                            angle = 3 
            # 차량이 라인 위에 있는 경우
            elif y < line_y:
                # 차량과 주차지와의 각도 차이와 거리에 따라 angle을 조정해 올바르게 차량이 이동할 수 있도록 하기 위한 조건문
                if angle_diff_parking > 25:
                    angle = 40
                elif angle_diff_parking < -25:
                    angle = -30
                elif angle_diff_parking > -1 and angle_diff_parking <1:
                    angle = 0
                elif distance > 30 and angle_diff_parking > 0:
                    angle = 20
                elif distance > 30 and angle_diff_parking < -10:
                    angle = 5
                elif distance > 30:
                    angle = 10
                elif 10 < distance < 30 and angle_diff_parking > 0:
                    angle = 20
                elif 10 < distance < 30 and angle_diff_parking < -5:
                    angle = -10
                elif 5 < distance < 10 and angle_diff_parking > 0:
                    angle = 15
                elif 5 < distance < 10 and angle_diff_parking < -4:
                    angle = -10
                elif distance < 5:
                    if angle_diff_parking > 2:
                        angle = 10
                    elif angle_diff_parking < -6:
                        angle = -10
                    else:
                        if angle_diff_parking < 0:
                            angle = -3
                        elif angle_diff_parking > 0:
                            angle = 3
        # 후진이 필요한 경우 후진을 시행하기 위한 조건문
        else:
            if angle_diff_parking > 0:
                angle = 5
                speed = -50
                timer = 0
            elif angle_diff_parking < 0:
                angle = -5
                speed = -50
                timer = 0
    #Between StartPoint and SavePoint
    if angle > 50:
        angle = 50
    elif angle < -50:
        angle = -50  
    drive(angle, speed)