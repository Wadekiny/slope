#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
#import cv2
from robotpi_Cmd import UPComBotCommand
from robotpi_serOp import serOp
from rev_cam import rev_cam

t_flag = 0
t_judge_y=10
t_line = 0

speed = 15  
DEFAULT_END= 49
DEFAULT_START = 59
err_slope = 0
ec_slope = 0

p_slope = 0
d_slope = 0

err_center = 0
ec_center = 0

p_center = 5
d_center = 0

pid_value = 0.0

right_angle_flag = 0
def PID_Caculate(para_slope, para_center):
    global err_slope
    global ec_slope 
    global p_slope 
    global d_slope 
    
    global err_center
    global ec_center
    global p_center 
    global d_center 
    
    global pid_value

    temp_slope = err_slope
    err_slope = para_slope
    ec_slope = err_slope - temp_slope

    temp_center = err_center
    err_center = para_center - 40
    ec_center = err_center - temp_center

    cal_p_slope = p_slope * err_slope
    cal_d_slope = d_slope * ec_slope

    cal_p_center = p_center * err_center
    cal_d_center = d_center * ec_center

    pid_value = cal_p_center + cal_p_slope + cal_d_center + cal_d_slope;
    pid_value = -pid_value
    return pid_value

def Camera_Init():
    cap = cv2.VideoCapture(0)
    print(cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320))
    print(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240))
    return cap

mean = 80

def Camera_Binary(para_image):
    ret, frame = para_image.read()
    mini = cv2.resize(frame, (80,60))
    gray = cv2.cvtColor(mini, cv2.COLOR_BGR2GRAY)
    ret2, binary = cv2.threshold(gray, mean,255, cv2.THRESH_BINARY) 
    binary_flip = cv2.flip(binary,-1)
    return binary_flip



center_line = np.arange(60)
center_start_y=0
center_end_y=0
center_valid_y=0
center_valid_bottom=59
LIGHT = 0xff
DARK = 0
def FindCenter(binary):
    global center_line
    global t_line
    global center_valid_y,center_valid_bottom
    global right_angle_flag
    global t_flag,t_judge_y
    IMG_HEIGHT = binary.shape[0]
    IMG_WIDTH = binary.shape[1]
    mid = int(IMG_WIDTH/2)
    left_line = 0
    right_line = IMG_WIDTH-1
    bottom_flag = 0
    center_valid_bottom = 59
    center_valid_y = 58
    center_line[center_valid_y]=40
    center_line[center_valid_bottom]=40
    t_flag = 0
    for bottom_line in range(IMG_HEIGHT-1, IMG_HEIGHT-40, -1):
        for mid_add in range(0,39,3): 
            if binary[bottom_line, mid + mid_add] == DARK:
                mid_temp = mid + mid_add
                bottom_flag = 1
            #中点在黑线内
                for left_index in range(mid_temp,-1,-1):
                    if binary[bottom_line, left_index] == LIGHT:
                       left_line = left_index
                       break
                for right_index in range(mid_temp,IMG_WIDTH,1):
                    if binary[bottom_line, right_index] == LIGHT:
                       right_line = right_index
                       break
                break
            elif binary[bottom_line, mid - mid_add] == DARK:
                bottom_flag = 1
                mid_temp = mid - mid_add
                for left_index in range(mid_temp,-1,-1):
                    if binary[bottom_line, left_index] == LIGHT:
                       left_line = left_index
                       break
                for right_index in range(mid_temp,IMG_WIDTH,1):
                    if binary[bottom_line, right_index] == LIGHT:
                       right_line = right_index
                       break
                break
        center_line[bottom_line] = int((left_line + right_line)/2)
        center_valid_y = bottom_line#有效行为最后一行
        center_valid_bottom = bottom_line
        if bottom_flag == 1:
            break
        #从下向上
    if bottom_flag == 0:
        return 0
    center_valid_y = center_valid_bottom - 1
    for line_index in range(center_valid_bottom-1,-1,-1):#从倒数第二行开始
        mid_temp = center_line[line_index+1]
        left_line = 0
        right_line = IMG_WIDTH - 1
        if binary[line_index, mid_temp] == DARK:#是黑色
            for left_index in range(mid_temp,-1,-1):
                if binary[line_index, left_index] == LIGHT:
                   left_line = left_index
                   break
            for right_index in range(mid_temp,IMG_WIDTH,1):
                if binary[line_index, right_index] == LIGHT:
                   right_line = right_index
                   break
            center_valid_y = line_index
            center_line[line_index] = int((left_line + right_line)/2)
            if left_line == 0 and right_line != IMG_WIDTH - 1:
                right_angle_flag = -line_index
                break
            elif left_line != 0 and right_line == IMG_WIDTH - 1:
                right_angle_flag  = line_index
                break
            else:
                right_angle_flag = 0
        else:#不是黑色
            break
    if right_angle_flag != 0:
        #have a right angle, t or angle
        abs_flag = abs(right_angle_flag) #right_angle_flag  ABS
        t_flag = 1
        temp_center = 40    #use for judge T center
        temp_y = abs_flag
        for l in range(abs_flag,center_valid_bottom-1, 1):#from top to bottom find the interrupt (3)
            if abs(center_line[l] - center_line[l+1]) > 3 and abs(center_line[l+1] - center_line[l+2] < 3):# l to l+1  the change is 3+
                temp_center = center_line[l+1]
                t_line = l+1
                temp_y = l+1
                break
        #print("specialspecial--- ", temp_center)
        #now have a temp center number , have a temp_y  form temp_y to top find which pix is not dark
        for i in range(temp_y, temp_y - t_judge_y, -1) :# must judge to t_judge_y
            if i == 0: #find the top ,not break, is a T
                t_flag = 1
                break
            if binary[i,temp_center] != DARK:
                t_flag = 0
                break

           # for j in range(-10,10,1):#roi is -10 ~ 10
           #     if binary[i,temp_center+j] != DARK:
           #         t_flag = 0
           #         break
           # if t_flag == 0:
           #         break
def PrintPara():
    print("----------------")
    print("mean:     ", mean)
    print("1+ 2- p_slope:  ", p_slope)
    print("3+ 4- d_slope:  ", d_slope)
    print("5+ 6- p_center: ", p_center)
    print("7+ 8- d_center: ", d_center)
    print("9+ 0- speed:    ", speed)
    print("----------------")

def PrintDynamic():
    print("================")
    print("center:   ", center_line[center_valid_bottom])
    print("slope:    ", slope)
    print("pidValue: ", pid_value)
    print("angleFlag:", right_angle_flag)
    print("valid_y:  ", center_valid_y)
    print("bottom_y  ", center_valid_bottom)
    print("T_Flag    ", t_flag)
    print("T_line    ", t_line)
    print("================")


if __name__ == '__main__':
    my_ser = serOp()
    image = Camera_Init()
    #         head   head                        tail
    pid_change = 0
    while True:
        binary = Camera_Binary(image)   #get binary img  name is "binary"

        FindCenter(binary)              #find center line
        #slo_start = DEFAULT_START
        
        slo_start = center_valid_bottom
        if center_valid_y > DEFAULT_END :
            slo_end = center_valid_y
        else:
            slo_end = DEFAULT_END
        #if slo_start > center_valid_bottom:
        #    slo_start = center_valid_bottom
        #if center_valid_y < abs(right_angle_flag) :
        #    center_valid_y = abs(right_angle_flag)
        #if slo_end < center_valid_y:
        #    slo_end =  center_valid_y
        #calslo
        slope = (center_line[slo_end] - center_line[slo_start])#/(slo_end - slo_start)
        PID_Caculate(slope,center_line[center_valid_bottom])
        if pid_value < 0:
            pid_abs = -pid_value
        else:
            pid_abs = pid_value
        pid_high = int(pid_abs / 256)
        pid_low = int(pid_abs %256)
#        print("high",pid_high,"low",pid_low)

        #print(t_flag)
        temp_speed = speed
        if t_flag == 1:
            temp_speed = 0
        else :
            temp_speed = speed
        special_t = 0
        if t_flag == 1 and t_line > 40:
            special_t = 1
        if pid_value >= 0:
            #          head  head                                  tail
            command = [0xf5, 0x5f, special_t, pid_high , pid_low, 0, temp_speed, 0x6f]
        else:
            command = [0xf5, 0x5f, special_t, pid_high , pid_low, 1, temp_speed, 0x6f]
        my_ser.write_serial(command)

        for index in range(center_valid_bottom,center_valid_y-1,-1):
            binary[index,center_line[index]] = LIGHT
            #print(center_line[index])
    
        cv2.imshow("binary2", binary)
        
    
        keytemp = cv2.waitKey(1) & 0xff
        if keytemp == ord('q'):
            break
        elif keytemp == ord('p'):
            PrintDynamic()
        

        elif keytemp == ord('='):
            mean = mean + 1
            PrintPara()
        elif keytemp == ord('-'):
            mean = mean - 1
            PrintPara()

        elif keytemp == ord('1'):
            p_slope += 1
            PrintPara()
        elif keytemp == ord('2'):
            p_slope -= 1
            PrintPara()
        elif keytemp == ord('3'):
            d_slope += 1
            PrintPara()
        elif keytemp == ord('4'):
            d_slope -= 1
            PrintPara()

        elif keytemp == ord('5'):
            p_center += 1
            PrintPara()
        elif keytemp == ord('6'):
            p_center -= 1
            PrintPara()
        elif keytemp == ord('7'):
            d_center += 1
            PrintPara()
        elif keytemp == ord('8'):
            d_center -= 1
            PrintPara()

        elif keytemp == ord('9'):
            speed += 1
            PrintPara()
        elif keytemp == ord('0'):
            speed -= 1
            PrintPara()


    image.release()
    cv2.destroyAllWindows()
