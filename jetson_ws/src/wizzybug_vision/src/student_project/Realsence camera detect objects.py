# 27/4/23 16:35 work ! needed file:
#  MobileNetSSD_deploy.prototxt,MobileNetSSD_deploy.caffemodel ,realsense_depth
# with offset not checked yet

import pyrealsense2
from realsense_depth import *       #another function file
import math
import numpy as np
import cv2
import time
from PIL import Image
import matplotlib as plt

max_angular_verticle = 20               # the width angular of the lens on the vertical view from the center
max_angular_horizen = 24                # the width angular of the lens on the horizen view from the center
camera_hight = 975                      # the hige of cammera is positioned in [mm] ***changeable
ignored_object_high =150                # the hige of the biggest object we want to ignore [mm] ***changeable
error_positive = 0                      # the error of the hard ware we want relate on in the computing [mm]
matrix_detect = range(-10, 10)          # the minimum area that defiend an object in pixels [pixels X pixels] ***changeable
wizzy_wide = 500 / 2                    # the wizzybug width/2 ,to calculate the path of the wizzybug 
teta_ofset = 14.25                      # positive it mean the camera looking "down" becarfull over 29 degree may cause problems
gama_ofset = 0                          # positive it mean the camera looking "right"
prototxt_path = 'MobileNetSSD_deploy.prototxt'          #Nural network DNN
modle_path = 'MobileNetSSD_deploy.caffemodel'
min_confidence = 0.2                                                                                      # the presentage of dnn network to classify object, over the nuber we accept this classification
classes = ["background", "aeroplan", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable" ,
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
             "sofa", "train", "tvmonitor"]                                                                #list of objects the nural Network can identify
np.random.seed(543210)
colors = np.random.uniform(0, 255, size=(len(classes), 3))                                                #creat array of the color of the boundry box for each classification
net = cv2.dnn.readNetFromCaffe(prototxt_path, modle_path)                                                 #openCV function for classificate an image
row_range_scan = range(min(max(0, 240 - int(teta_ofset * 240 / max_angular_verticle)), 477), 480, 5)      # to accelrate the procecing time we loop over the objects under the horizen line
angular_matrix = np.ones((480, 640)) * float('inf')                                                       #this matrix used to predicrt....
floor_matrix = np.ones((480, 640)) * float('inf')                                                         #this matrix used to predicrt....
teta_matrix= np.ones((480, 640))                                                                          #this matrix used to predicrt....
gama_matrix = np.ones((480, 640))                                                                         #this matrix used to predicrt....

for row in row_range_scan:
    for column in range(320):
        teta = np.longfloat(math.radians((max_angular_verticle / 240) * (row - 240) + teta_ofset))
        gama = np.longfloat(math.radians((max_angular_horizen / 320) * (320 - column) + gama_ofset))
        floor_pix_dist = float('inf') if math.sin(teta) == 0 or math.cos(gama) == 0 else np.longfloat(abs(
            (camera_hight / (math.sin(teta))) / math.cos(gama)))  # calculate the distance to the floor from that pixel and ignor deviding in zero
        ignored_object = 0 if math.sin(teta) == 0 or math.cos(gama) == 0 else np.longfloat(abs((ignored_object_high / (math.sin(teta))) / math.cos(gama)))  # calculate the distance min((ignored_object_high / ((math.sin(teta)) * math.cos(gama))), 9000)
        if row > 270 and row <= 300:
            floor_pix_dist += ((500) / (450 - 270)) * (450 - row) +500
        if row > 315 and row <= 380:
            floor_pix_dist += ((590) / (450 - 315)) * (450 - row)
        if row > 380 and row <= 450:
            floor_pix_dist += ((220) / (450 - 380)) * (450 - row)
        floor_fix_size = floor_pix_dist - ignored_object - error_positive
        floor_matrix[row][column] = floor_pix_dist
        floor_matrix[row][-1 - column] = floor_pix_dist
        angular_matrix[row][column] = floor_fix_size
        angular_matrix[row][-1 - column] = floor_fix_size

floor_matrix = np.loadtxt('predict_matrix4.txt', usecols=range(640))
teta_matrix = np.loadtxt('teta_matrix.txt', usecols=range(640))
floor_matrix[0:121,:]=100000


def detect_obstacles(image, depth_img):  
##  this method get the same image in deffrent configure RGB & Depth, using those images the function will identify the relevents objects(the closest & above min high) for each derection of movments 

    lenght_row = len(depth_img)
    lenght_column = len(depth_img[0])-40
    range_rows = range(int(lenght_row * 0.4), lenght_row, 3)
    range_column_LEFT = range(0, lenght_column // 3, 3)
    range_column_MIDDLE = range((lenght_column // 3) + 1, (lenght_column // 3) * 2, 3)
    range_column_RIGHT = range((lenght_column // 3) * 2 + 1, lenght_column, 3)
    min_array_points = []
    relevent_obsticals1 = [0, 0, 0]
    # floor_pix_dist = float('inf')  # the distance that the floor for that pixel
    situations = [range_column_LEFT, range_column_MIDDLE, range_column_RIGHT]
    limits = []

    ###first part finding the suspect "point"(pixel) for each direction. 1->LEFT,2->MIDDLE,3->RIGHT #####
    for i in [0, 1, 2]:
        count = 0                                                                                 #this varible used to count and verfied that suspect point is an object                               
        for row in row_range_scan:                                                              
        #looping all the rows of the image under horizen level    
            wid = int(min(math.degrees(math.atan(wizzy_wide / min(floor_matrix[row][int(lenght_column / 2)], 5000))),
                          max_angular_horizen) * (int(lenght_column / 2) / max_angular_horizen))  #calculating for each row the pixels that limits the wizzybug width in the picture for heading strait
            w_range = 0     #this variable used to store the column for each row for each direction 

            #the method of skiping 3 pixels here in the range accelrate the time and was implement afther thoughts 
            #and checks that a real objects will cover more then 3 pixels                                                                                             
            if i == 0:
                w_range = range(0,max(int(lenght_column / 2) - wid, 1),3)                                      #the column numbers of LEFT direction
            elif i == 1:
                w_range = range(int(lenght_column / 2) - max(wid, 1), int(lenght_column / 2) + max(wid, 1),3)  #the column numbers of MIDDLE direction
            else:
                w_range = range(min(int(lenght_column / 2) + wid, lenght_column - 1), lenght_column,3)         #the column numbers of RIGHT direction
            if i == 1:
            #if we are in the middle save the pixels that represent direction of the WizzyBug to color the picture later
                limits.append([row, w_range[0]])
                limits.append([row, w_range[-1]])


            for column in w_range:
                #looping on the columns for each row for each direction 
                if depth_img[row][column] == 0: #if the pixels wasn't get his real distance ,skip it 
                    continue                        
                if depth_img[row][column] < floor_matrix[row][column]-ignored_object-error_positive:  # this method is used based on geometric calculations to find an object with high over "maximum high of object to ignore"
                    min_array_points.append([row, column, depth_img[row][column]])                    # if it pass the condition ,add it to the list of optional points  
        min_array_points.sort(key=lambda x: x[2])                                                     # sort the list by the distance
        for point in min_array_points:     
        #looping on the sorted list                                                            
            rows_range = range(max(0, point[0] + matrix_detect[0]), min(lenght_row, point[0] + matrix_detect[-1]))
            column_range = range(max(0, point[1] + matrix_detect[0]),min(situations[i][-1], point[1] + matrix_detect[-1]))
            for row in (rows_range):
                for column in (column_range):
                #looping on the surrounding matrix of point from the list on an area that defind an object
                    zz = (depth_img[row][column] - point[2]) if depth_img[row][column] > point[2] else point[2] - depth_img[row][column]
                    if zz < 100: 
                    #if the distance of the pixel is less then the point distance it implict on an object 
                        count += 1
            if count >= 10:
            #if we find in the surrounding matrix more then 10 pixel that defined the same distance of the point
            #we declear it as a relevnts obsical for this direction
                relevent_obsticals1[i] = point
                min_array_points = []
                break
   
    # part 2 - afther finding 3 relavent points for each direction
    # we prepering the classication part with methods and code from openCV & open Source Code
    height, width = image.shape[0], image.shape[1]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007, (300, 300), 130)
    net.setInput(blob)
    detected_object = net.forward()
    Ob = [0, 0, 0]
    color12 = 1

    for idx, point in enumerate(relevent_obsticals1):
    #trying to classify object to each relavent point that we found 
        objects =[]
        for i in range(detected_object.shape[2]):
        #looping on the all classification that the DNN Network found.
            # x represent column and y represent rows and with next four variable we can defined bounding box.
            upper_left_x = int(detected_object[0, 0, i, 3] * width)
            upper_left_y = int(detected_object[0, 0, i, 4] * height)
            lower_right_x = int(detected_object[0, 0, i, 5] * width)
            lower_right_y = int(detected_object[0, 0, i, 6] * height)
            upper_left_y = upper_left_y - 15 if upper_left_y > 30 else upper_left_y + 15  # dont exit the limits
            confidence = detected_object[0][0][i][2]
            if confidence > min_confidence:
            #we set the minimum confidence we want to relate from the Nural Network
                objects.append([i,upper_left_x,upper_left_y,lower_right_x,lower_right_y,confidence])

                if point[1] >= upper_left_x and point[1] <= lower_right_x and point[0] <= lower_right_y and point[0] >= upper_left_y:
                # if the boundry box contain the relavent point
                # here we going to use the depth dimention 
                # we will calculate the the avarge of the center of the classification
                # and if it more then 50 cm then probably it's not the right classification for that point 
                #other wise it is ,probably.
                    avg = 0
                    center_row = (upper_left_y + lower_right_y) // 2
                    center_column = (lower_right_x + upper_left_x) // 2
                    for iii in matrix_detect:
                        for jjj in matrix_detect:
                            avg += depth_img[iii + center_row][jjj + center_column]
                    avg = avg / (len(matrix_detect) ** 2)
                    if abs(point[2] - avg) < 500:
                        class_index = int(detected_object[0, 0, i, 1])
                        Ob[idx] = 1    #we find classification for that point 
                        objects.append(
                            [i, upper_left_x, upper_left_y, lower_right_x, lower_right_y, confidence, class_index])

                        #estimating the real size using pitagoras law  
                        teta_high = math.radians(((max_angular_verticle) / 240) * ((lower_right_y - upper_left_y) / 2))
                        gama_width = math.radians(((max_angular_verticle + gama_ofset) / 240) * ((lower_right_x - upper_left_x) / 2))
                        midlleh = int((lower_right_y + upper_left_y) / 2)
                        midllew = int((lower_right_x + upper_left_x) / 2)
                        avg_h = 0
                        avg_w = 0
                        count_p = 0
                        for indxh in range(max(midlleh - 50, 0), min(midlleh + 50, lenght_row), 5):
                            #check the avg distance of "the center" for each classification to do pitagoras law from the center of the object 
                            if depth_img[indxh][midllew] != 0: #
                                avg_h += depth_img[indxh][midllew]
                                count_p += 1
                        avg_h = point[2] if count_p == 0 else int(avg_h / count_p) #avoiding blind "spots"
                        count_p = 0
                        for indxw in range(max(midllew - 50, 0), min(midllew + 50, lenght_column), 5):
                            #check the avg distance of "the center" for each classification to do pitagoras law from the center of the object 
                            if depth_img[midlleh][indxw] != 0:
                                avg_w += depth_img[midlleh][indxw]
                                count_p += 1
                        avg_w = point[2] if count_p == 0 else int(avg_w / count_p)#avoiding blind "spots"

                        high = int(avg_h * math.tan(teta_high) / 10) * 2        #calculate the high
                        width1 = int(avg_w * math.tan(gama_width) / 10) * 2     #calculate the width
                        dist = int(point[2] / 10)                               #transfer the distance to [cm]
                        upper_left_y = upper_left_y - 15 if upper_left_y > 30 else upper_left_y + 15
                        prediction_text = f"{classes[class_index]},d:{dist},h:{high},w:{width1}"                                                  #title with information for each object
                        cv2.rectangle(image, (upper_left_x, upper_left_y), (lower_right_x, lower_right_y), colors[color12],3)                     #create boundry box
                        cv2.putText(image, prediction_text, (upper_left_x, upper_left_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[color12], 2)       
                        Ob[idx] = {'p': point[0:2], 'h,w': [high, width1], 'd': dist, 't': f"{classes[class_index]}%"}                            #update the final object list
                        color12 += 3
                        break
            
        if Ob[idx] == 0:
            #same logics of the classification just for  here is for unknown object 
            upper_left_x = max(0, point[1] + matrix_detect[0])
            lower_right_x = min(lenght_column, point[1] + matrix_detect[-1])
            lower_right_y = min(lenght_row, point[0] + matrix_detect[-1])
            upper_left_y = max(0, point[0] + matrix_detect[0])  # dont exit the limits
            if upper_left_y<121 and lower_right_y >121 :
                teta_high = abs(teta_matrix[upper_left_y][(lower_right_x+upper_left_x)//2] + teta_matrix[upper_left_y][(lower_right_x+upper_left_x)//2])
            else:
                teta_high = abs(teta_matrix[upper_left_y][(lower_right_x+upper_left_x)//2] - teta_matrix[upper_left_y][(lower_right_x+upper_left_x)//2])

            gama_width = math.radians((max_angular_verticle / 240) * (len(matrix_detect) / 2))
            high = int(point[2] * math.tan(teta_high/2) * 2 / 10)
            width1 = int(point[2] * math.tan(gama_width) * 2 / 10)
            dist = int(point[2] / 10)
            prediction_text = f"{'t'}:{'unknon'},d:{dist},h:{high},w:{width1}%"
            cv2.rectangle(image, (upper_left_x, upper_left_y), (lower_right_x, lower_right_y), colors[color12], 3)
            cv2.putText(image, prediction_text, (upper_left_x, upper_left_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        colors[color12], 2)
            Ob[idx] = {'p': point[0:2], 'h,w': [high, width1], 'd': dist, 't': "unknon"}
            color12 += 3
    for item in limits:
        image[item[0]][item[1]] = 0
    cv2.imshow("Detect Objects", image)
    cv2.imwrite('final_picture.jpg', image)
   
    return Ob



# Initialize Camera Intel Realsense
dc = DepthCamera()

depth_frame = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
count_frame = 0
start = time.time()
while True:
    count_frame += 1
    if (time.time() - start) >= 1:
        start = time.time()
        print("frame in sec is:")
        print(count_frame)
        count_frame = 0
    count22 = 0
    i = 0
    for num in range(10):
        ret, depth_frame[i], color_frame = dc.get_frame()
        i += 1
    for row in range(0, len(depth_frame), 3):
        for column in range(20, len(depth_frame[0]), 3):
            if depth_frame[9][row][column] == 0:
                for indx in range(9, 0, -1):
                    if depth_frame[indx][row][column] != 0:
                        depth_frame[9][row][column] == depth_frame[indx][row][column]
                        count22 += 1
                        break

    cv2.imwrite('Color.jpg', color_frame)

    image = cv2.imread('Color.jpg')
   
    print(detect_obstacles(image, depth_frame[9]))
    cv2.imwrite('Dept.jpg', depth_frame[9])

    key = cv2.waitKey(1)
    if key == 27:
        break