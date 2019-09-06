# -*- coding: UTF-8 -*-
import cv2
import numpy as np
from time import sleep, time, clock
import math
import tensorflow as tf
from decimal import Decimal

global address
address = ["1_1","1_2","1_3","1_4","1_5","2_1","2_2","2_3","2_4","2_5","3_1","3_2","3_3"
             ,"3_4","3_5","4_1","4_2","4_3","4_4","4_5","5_1","5_2","5_3","5_4","5_5","6_1",
			 "6_2","6_3","6_4","6_5","7_1","7_2","7_3","7_4","7_5","8_1","8_2","8_3","8_4",
			 "8_5","9_1","9_2","9_3","9_4","9_5","9_6"]
###################################################################################
# --------------开启线程监听--------------

def listen_thread(port):  # 切换大小符  B b S s
    global big_or_small_flag
    print("listen thread is running...")
    data = ""
    while True:
        data = port.read()
        big_or_small_flag = data.decode()

###################################################################################
# --------------解算角度所需的类--------------

def limit(point):
    if point[1] > 480:
        point[1] = 480
    if point[1] < 0:
        point[1] = 0
    if point[0] > 640:
        point[0] = 640
    if point[0] < 0:
        point[0] = 0

class RectPnPSolver(object):
    # 设置相机内参，畸变系数，目标装甲的宽度和高度
    def __init__(self, camera_matrix, dist_coeff, target_width, target_height):
        self.cam_matrix = camera_matrix.copy()
        self.distortion_coeff = dist_coeff.copy()
        self.width_target = target_width
        self.height_target = target_height

    def setCameraParam(self, camera_matrix, dist_coeff):
        self.cam_matrix = camera_matrix.copy()
        self.distortion_coeff = dist_coeff.copy()


class AngleSolver(RectPnPSolver):
    # 设置相机内参，畸变系数，目标装甲的宽度和高度
    def __init__(self, camera_matrix, dist_coeff, target_width, target_height, z_scale=1.0, min_dist=50.0,
                 max_dist=600.0):
        super(AngleSolver, self).__init__(camera_matrix, dist_coeff, target_width, target_height)
        self.min_distance = min_dist
        self.max_distance = max_dist
        self.rot_camera2ptz = np.eye(3)  # 相机转云台的旋转矩阵
        self.trans_camera2ptz = np.zeros((3, 1), np.float32)  # 相机转云台的平移矩阵
        self.offset_y_barrel_ptz = 0
        self.scale_z = z_scale

    # 设置相机坐标转换到云台坐标的矩阵参数
    def setRelationPoseCameraPTZ(self, trans_camera_ptz, y_offset_barrel_ptz):
        self.trans_camera2ptz = trans_camera_ptz
        self.offset_y_barrel_ptz = y_offset_barrel_ptz

    def setScaleZ(self, scale):
        self.scale_z = scale

    def tranformationCamera2PTZ(self, camera, tran):  # 相机转云台坐标系
        return camera + tran

    # 获得目标矩阵上的四个2D点
    def getTarget2dPoinstion(self, rect):
        box = cv2.boxPoints(rect)
        # box=np.int0(box)
        box = sorted(box, key=lambda student: student[0])  # 按第一列(x坐标)排序

        if (box[0][1] < box[1][1]):
            lu = box[0]
            ld = box[1]
        else:
            lu = box[1]
            ld = box[0]
        if (box[2][1] < box[3][1]):
            ru = box[2]
            rd = box[3]
        else:
            ru = box[3]
            rd = box[2]

        # target2d=np.float32([lu,ru,rd,ld])

        limit(lu)
        limit(ru)
        limit(rd)
        limit(ld)
        target2d = np.array([lu, ru, rd, ld])
        target2d = np.ascontiguousarray(target2d.reshape((4, 1, 2)))

        # cv2.rectangle(img3,(lu[0],lu[1]),(rd[0],rd[1]),(0,0,255),2)

        return target2d

    # 获得角度
    def getAngle(self, rect):
        target2d = self.getTarget2dPoinstion(rect)  # 获得2D点

        half_x = self.width_target / 2.0
        half_y = self.height_target / 2.0

        # 获得3D点
        # point3d=np.float32([[-half_x, -half_y, 0.0],[half_x, -half_y, 0.0],[half_x, half_y, 0.0],[-half_x, half_y, 0.0]])
        point3d = np.array(
            [[-half_x, -half_y, 0.0], [half_x, -half_y, 0.0], [half_x, half_y, 0.0], [-half_x, half_y, 0.0]])

        # r 旋转向量   外参数
        # trans 平移向量   外参数
        # PNP需要显示3D点，2D图像点，相机参数，畸变系数
        retval, r, position_in_camera = cv2.solvePnP(point3d, target2d, self.cam_matrix, self.distortion_coeff, flags=2)
        # cv2.Rodrigues(r, rot)     #罗德里格斯旋转变量，将旋转向量变为旋转矩阵

        #position_in_camera = self.tranformationCamera2PTZ(position_in_camera, self.trans_camera2ptz)
        # 相机转云台坐标
        '''
        [[ -521.36296224]
        [ -241.52294343]
        [ 3227.29784705]]
        '''

        position_in_camera = position_in_camera.ravel()  # 降维

        alpha = 0.0
        theta = 0.0
        angle_x = 0.0
        angle_y = 0.0

        self.offset_y_barrel_ptz = 0
        alpha = 0
        """
        alpha = math.asin(self.offset_y_barrel_ptz / math.sqrt(
            position_in_camera[1] * position_in_camera[1] + position_in_camera[2] * position_in_camera[2]))
        """

        if (position_in_camera[1] < 0):
            theta = math.atan(-position_in_camera[1] / position_in_camera[2]);  # 目标在上面
            angle_y = -(alpha + theta);  # camera coordinate 枪管要向上移动
        elif (position_in_camera[1] < self.offset_y_barrel_ptz):
            theta = math.atan(position_in_camera[1] / position_in_camera[2]);  # 目标在枪管和云台之间
            angle_y = -(alpha - theta);
        else:
            theta = math.atan(position_in_camera[1] / position_in_camera[2])  # 目标在枪管下面
            angle_y = (theta - alpha)

        angle_x = math.atan2(position_in_camera[0], position_in_camera[2]);  # x轴移动
        angle_x = angle_x * 180 / 3.1415926
        angle_y = angle_y * 180 / 3.1415926

        return angle_x, angle_y

###################################################################################
# -----------------共同-------------------
def Sort(con_number):
    length = len(con_number)
    for i in range(0, length - 1):
        for j in range(0, len(con_number) - i - 1):
            centerA = con_number[j][0]
            centerB = con_number[j+1][0]
            sumA = centerA[0] + centerA[1]
            sumB = centerB[0] + centerB[1]
            if (sumA > sumB):
                con_number[j], con_number[j + 1] = con_number[j + 1], con_number[j]

    con_number[1], con_number[3] = con_number[3], con_number[1]
    con_number[5], con_number[7] = con_number[7], con_number[5]

    if (con_number[1][0][0] - con_number[1][0][1]) < (con_number[2][0][0] - con_number[2][0][1]):
        con_number[1], con_number[2] = con_number[2], con_number[1]

    if (con_number[6][0][0] - con_number[6][0][1]) < (con_number[7][0][0] - con_number[7][0][1]):
        con_number[6], con_number[7] = con_number[7], con_number[6]

    con_number[2], con_number[6] = con_number[6], con_number[2]
    return con_number

# 这个函数用来确定四个顶点位置
def sortCorners(box, center):  # 四个点和中心
    result = []
    top = []
    bot = []
    for corner in box:  # 以中心为基准点分上下
        if corner[1] < center[1]:
            top.append(corner)
        else:
            bot.append(corner)

    tl = top[1] if (top[0][0] > top[1][0]) else top[0]
    tr = top[0] if (top[0][0] > top[1][0]) else top[1]
    bl = bot[1] if (bot[0][0] > bot[1][0]) else bot[0]
    br = bot[0] if (bot[0][0] > bot[1][0]) else bot[1]

    result.append(list(tl))  # 转为list
    result.append(list(tr))
    result.append(list(br))
    result.append(list(bl))

    return result

# 对灯柱各数字排序
def Sort2(con_number):
    for i in range(len(con_number) - 1):
        for j in range(len(con_number) - 1 - i):
            if (con_number[j][0] > con_number[j + 1][0]):
                con_number[j], con_number[j + 1] = con_number[j + 1], con_number[j]


def compare(src):  # 穿线法识别数字
    num = 0
    dst = np.zeros((50, 35, 1), np.uint8)
    cv2.bitwise_and(src, coordinate_y, dst)
    dst, contours1, hi = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dst = np.zeros((50, 35, 1), np.uint8)
    cv2.bitwise_and(src, coordinate_x_up, dst)
    dst, contours2, hi = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dst = np.zeros((50, 35, 1), np.uint8)
    cv2.bitwise_and(src, coordinate_x_down, dst)
    dst, contours3, hi = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dst = np.zeros((50, 35, 1), np.uint8)
    cv2.bitwise_and(src, coordinate_z, dst)
    dst, contours4, hi = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if (len(contours1) == 1):

        if (len(contours2) == 1):
            num = 7

        #elif (len(contours2) == 2):
        else:
            num = 4
    if (len(contours1) == 3):
        # 2 3 5 6 8 9
        if (len(contours2) == 2):
            # 8 9
            if (len(contours3) == 1):
                num = 9
            elif (len(contours3) == 2):
                num = 8
        if (len(contours2) == 1):
            if (len(contours3) == 2):
                num = 6
            elif (len(contours3) == 1):
                if (len(contours4) == 1):
                    num = 2
                elif (len(contours4) == 2):
                    num = 3
                elif (len(contours4) == 3):
                    num = 5
    return num


# 对灯管的矩阵作预处理，扣出各个数字
def digital_number_treat(gray, pswArea):
    center = pswArea[0]  # 灯管的最小面积矩阵
    box = cv2.boxPoints(pswArea)
    box = np.int0(box)

    width, height = pswArea[1]
    rect_pts = np.float32(sortCorners(box, center))
    quad_pts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    M = cv2.getPerspectiveTransform(rect_pts, quad_pts)
    dst = cv2.warpPerspective(gray, M, (int(width), int(height)))  # 透视投影让它变正
    dst = dst[3:int(height) - 3, 3:int(width) - 3]

    _, dst = cv2.threshold(dst, 200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    #_, dst = cv2.threshold(dst, 80, 255, cv2.THRESH_BINARY)  # 用局部阈值

    temp = cv2.erode(dst, (3, 3), iterations=1)
    temp = cv2.dilate(temp, (3, 3), iterations=2)
    temp = cv2.erode(temp, (3, 3), iterations=1)

    #cv2.imshow("lllll", temp)
    temp2 = temp.copy()

    ret, temp = cv2.threshold(temp, 100, 255, cv2.THRESH_BINARY)
    temp, con, hire = cv2.findContours(temp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    target = []
    con = [elem for elem in con if cv2.contourArea(elem) >= 45]
    if (len(con) == 5):  # 只剩下五个时，说明是数字
        for c in con:
            rect = cv2.boundingRect(c)  # 对轮廓取正矩阵，得到坐标和长宽
            rects.append(rect)
        Sort2(rects)
        for rect in rects:
            _x, y, w, h = rect

            if (float(h) / float(w) >= 2.0):  # 根据长宽比直接判断1
                target.append(1)
            else:
                roi = dst[y:y + h, _x:_x + w]  # 取包含该数字的ROI区域
                roi = cv2.resize(roi, (35, 50))
                roi = cv2.erode(roi, (2, 2), iterations=2)

                SHUMA_result = compare(roi)

                if SHUMA_result == 0:
                    SHUMA_result = 7
                target.append(SHUMA_result)

    return target

###################################################################################
# -----------------小符-------------------

def angle(pt1, pt2, pt0):
    dx1 = float(pt1[0][0] - pt0[0][0])
    dy1 = float(pt1[0][1] - pt0[0][1])
    dx2 = float(pt2[0][0] - pt0[0][0])
    dy2 = float(pt2[0][1] - pt0[0][1])
    return (dx1 * dx2 + dy1 * dy2) / math.sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10)


def Xiaofu_FindSquares(img):

    # 阈值分割，大津二值化
    ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(img, 60, 255, cv2.THRESH_BINARY)

    cnt = 0
    con = []
    con22 = [] #用来存适合条件的原轮廓

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    thresh = cv2.erode(thresh, (3, 3), iterations=1)
    thresh = cv2.dilate(thresh, element)
    #cv2.imshow("ddddd",thresh)
    minArea = 999999  #计算轮廓的最大最小面积
    maxArea = -1

    thresh, contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for index in range(len(contours)):
        counter = contours[index]
        approx = cv2.approxPolyDP(counter, cv2.arcLength(counter, True) * 0.1, True)

        if len(approx) == 4 and math.fabs(cv2.contourArea(approx)) <= 20000 and math.fabs(
                cv2.contourArea(approx)) >= 3000 and cv2.isContourConvex(approx):
            maxCosine = 0.0
            for j in range(2, 5):
                cosine = math.fabs(angle(np.array(approx[j % 4]), np.array(approx[j - 2]), np.array(approx[j - 1])))
                maxCosine = max(maxCosine, cosine)
            if maxCosine < 0.2:
                con22.append(index)
                con.append(approx)
                cnt += 1
                #得到已经筛选出的面积的最大最小值
                if math.fabs(cv2.contourArea(counter)) > maxArea:
                    maxArea = math.fabs(cv2.contourArea(approx))
                if math.fabs(cv2.contourArea(counter)) < minArea:
                    minArea = math.fabs(cv2.contourArea(approx))
    print(cnt)
    if ( cnt<9 and cnt == 8):  #当少于9个时 再次筛选
        for index in range(len(contours)):
            counter = contours[index]
            approx = cv2.approxPolyDP(counter, cv2.arcLength(counter, True) * 0.1, True)
            rect = cv2.minAreaRect(approx)
            center, size, _ = rect
            if index not in con22:
                if 40 < center[0] and center[0] < 620 and center[1] > 80 and center[1] < 440 and cv2.isContourConvex(approx):
                    if math.fabs(cv2.contourArea(counter))<=maxArea*1.3 and math.fabs(cv2.contourArea(counter))>=minArea*0.8:
                        con.append(counter)  #因为有可能是个三角形 所以要放counter
                        cnt = cnt + 1
    print(cnt)
    return con, cnt

def XiaoFu_getSquaresList(gray, squares):  # 对每个格子的旋转矩阵作透视变换,返回一张图片
    result = []
    for square in squares:
        center = square[0]
        box = cv2.boxPoints(square)
        box = np.int0(box)
        rect_pts = np.float32(sortCorners(box, center))
        side_length = 32
        quad_pts = np.float32([[0, 0], [side_length, 0], [side_length, side_length], [0, side_length]])
        M = cv2.getPerspectiveTransform(rect_pts, quad_pts)  # 透视变换让每张图片变正
        dst = cv2.warpPerspective(gray, M, (side_length, side_length))
        dst = dst[2:side_length, 2:side_length]
        _, dst = cv2.threshold(dst, 200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        dst = cv2.resize(dst, (24, 24))
        dst = cv2.copyMakeBorder(dst, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=255)
        # 加上四条白边，让边缘变白
        result.append(dst)
    return result

def Xiaofu_attack(target, pics, results,angle ,port=1):
    global last_attack
    success_flag = False
    list_of_imvalue = []

    for pic in pics:
        imval = []
        width, height = pic.shape
        for i in range(0, height):
            for j in range(0, width):
                imval.append(pic[i, j])
        # 标准化
        tva = [(255 - j) * 1.0 / 255.0 for j in imval]
        list_of_imvalue.append(tva)

    # tensorflow作预测
    results = prediction_xiao.eval(feed_dict={x_xiao: list_of_imvalue, keep_prob_xiao: 1.0}, session=sess_1)
    results = results.tolist()

    print("chubu results:")
    print(results)

    print("target number: ", target)

    num_of_equal = 0

    if len(_results) == 0:
        _results[:] = results[:]

    else:
        for i in range(0, 9):
            if results[i] == _results[i]:
                num_of_equal += 1

    # 九个格子和上一次一样的话，就不进行打击
    # 或者当前很多格子一样时

    cout2 = 0
    for nu in results:
        if (nu == 0):
            cout2 += 1


    if num_of_equal >= 7 or int(len(set(results))) <= 7 or cout2 >= 2:
        print("gezi same,pass")
        return False


    # 不一样的话，进行打击
    else:

        for i in range(0, 9):
            if results[i] == 0:
                results[i] = 9
            if results[i] == target:  # 如果手写数字和数码管数字相等
                # 向右时作补偿

                if last_attack == 1 or last_attack == 4 or last_attack == 7:
                    if (i + 1) == 2 or (i + 1) == 5 or (i + 1) == 8:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        if (i + 1) == 2:
                            temp = angle[0][1]
                        if (i + 1) == 5:
                            temp = angle[3][1]
                        if (i + 1) == 8:
                            temp = angle[6][1]
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[0][0]) * 0.1, temp)

                    if (i + 1) == 3 or (i + 1) == 6 or (i + 1) == 9:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        if (i + 1) == 3:
                            temp = angle[0][1]
                        if (i + 1) == 6:
                            temp = angle[3][1]
                        if (i + 1) == 9:
                            temp = angle[6][1]
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[1][0]) * 0.2, temp)

                if last_attack == 2 or last_attack == 5 or last_attack == 8:
                    if (i + 1) == 3 or (i + 1) == 6 or (i + 1) == 9:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[0][0]) * 0.07, angle[i][1])

                print("last_attack: ", last_attack)
                last_attack = i+1

                angle_x = angle[i][0]
                angle_y = angle[i][1]
                angle_x = Decimal(str(angle_x)).quantize(Decimal('0.00'))  # 保留两位小数
                angle_y = Decimal(str(angle_y)).quantize(Decimal('0.00'))  # 保留两位小数
                angle_x = str(int((float(angle_x) * 100)))  # 转为字符串
                angle_y = str(int((float(angle_y) * 100)))  # 转为字符串

                # port.write(("n" + str(i + 1) + "e").encode())
                print("Attack " + str(i + 1) + " square." + "(" + str(results[i]) + ")")
                # "x: "+str(angle[i][0])+" y: "+str(angle[i][1])
                # port.write(("x"+angle_x+"y"+angle_y+"e").encode())
                print("x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                print('\n')

                _results[:] = results[:]
                success_flag = True
                break

        for i in range(len(angle)):
            print("x: ", angle[i][0], "  y: ", angle[i][1])

    return success_flag

###################################################################################
# -----------------大符-------------------
def FindDigit(img):  #找数码管数字
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 大津二值化
    #ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    ret, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

    thresh = cv2.dilate(thresh, (3, 3), iterations=3)  # 二值化后膨胀

    #cv2.imshow("thresh_first", thresh)

    final, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    con = []
    cnt = 0
    temp_con = []
    temp_cnt = 0

    # 第一层筛选，长宽和面积
    for counter in contours:
        rect = cv2.minAreaRect(counter)
        center, size, _ = rect
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        if math.fabs(cv2.contourArea(box)) <= 4500 and math.fabs(cv2.contourArea(box)) >= 60:
            x3, y3, w3, h3 = cv2.boundingRect(counter)
            # 通过正矩阵长宽比去除左右小方格
            if (w3 / h3 < 1.0):
                temp_con.append(counter)
                temp_cnt += 1

    # 第二层筛选
    if temp_cnt != 0:
        for counter in temp_con:  # 第一层循环，数字本身
            rect = cv2.minAreaRect(counter)
            center, size, _ = rect
            temp_con_2 = []
            temp_con_2.append(rect)
            for counter_2 in temp_con:  # 第二层循环，别的数字,比较 y 坐标偏差
                rect_2 = cv2.minAreaRect(counter_2)
                center_2, size_2, _ = rect_2
                if center[0] != center_2[0]:
                    if math.fabs(center[1] - center_2[1]) < 0.5 * size[1]:
                        temp_con_2.append(rect_2)

            temp_con_2 = sorted(temp_con_2, key=lambda x: x[0][0])  # 按 x 坐标排序

            temp_con_3 = []
            for index_xx in range(len(temp_con_2) - 1):  # 第二层循环，,比较两两数字间 x 坐标聚集程度
                if math.fabs(temp_con_2[index_xx][0][0] - temp_con_2[index_xx + 1][0][0]) < 2.0 * size[0]:
                    temp_con_3.append(temp_con_2[index_xx])
                    temp_con_3.append(temp_con_2[index_xx + 1])

            con = list(set(temp_con_3))[:]
            cnt = len(set(temp_con_3))
            if len(set(temp_con_3)) == 5:
                break

    for rect in con:
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (255, 255, 255), 1)
    #cv2.imshow("66666666666", img)

    return cnt, con

    """
    img = cv2.GaussianBlur(img, (3, 3), 0)  # 去噪，模糊图片

    # 取反再转为HSV
    img2 = ~img
    HSV = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

    # S越大，表示火焰字的色越深
    minHSV = np.array([75, 60, 70])  # S越大，越深色，V越大，越发白
    maxHSV = np.array([100, 255, 255])
    final = cv2.inRange(HSV, minHSV, maxHSV)

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    final = cv2.dilate(final, element)  #膨胀

    cv2.imshow("LAB", final)

    # 找所有红色的外层轮廓
    final, contours, _ = cv2.findContours(final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

    con = []
    cnt = 0

    sum_y = 0   # 用来记录平均y坐标的值
    temp_con = []
    temp_cnt = 0

    # 初步筛选 面积过滤
    for counter in contours:
        rect = cv2.minAreaRect(counter)
        center, size, _ = rect
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        if math.fabs(cv2.contourArea(box)) <= 7000 and math.fabs(cv2.contourArea(box)) >= 180:
            x3, y3, w3, h3 = cv2.boundingRect(counter)
            # 通过正矩阵长宽比去除左右小方格
            if(w3 / h3 < 1.3):
                temp_con.append(counter)
                sum_y = center[1] + sum_y
                temp_cnt = temp_cnt + 1

    if (temp_cnt != 0):
        # 要高于一定高度
        for counter in temp_con:
            rect = cv2.minAreaRect(counter)
            center, size, _ = rect
            if center[1] < 300:
                con.append(rect)  # 将旋转矩阵存进去
                cnt = cnt + 1

        if cnt > 5:  #大于五个时
            con = sorted(con, key=lambda x: x[0][1])  # 按y坐标排序
            con = con[0:5]
            flag = True
            for index in range(4):
                if math.fabs(con[index][0][1]-con[index+1][0][1]) > 100:
                    flag = False
                    break
            if flag == True:
                cnt = 5

        print("first shuma:", cnt)

        if cnt < 5 and cnt == 4:  # 少于5个时，重新筛选
            min_height = 9999
            max_height = -1
            for counter in contours:   # 找出数码管的y坐标的上下界
                rect = cv2.minAreaRect(counter)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                for corner in box:
                    if corner[1] > max_height:
                        max_height = corner[1]
                    if corner[1] < min_height:
                        min_height = corner[1]
            min_height = min_height * 0.9
            max_height = max_height * 1.1
            # 上面这个是数码管的区域边界

            for counter in contours:
                rect = cv2.minAreaRect(counter)
                center, size, _ = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if center[0] > 20 and center[0] < 620 and center[1] > min_height and center[1] < max_height:
                    if math.fabs(cv2.contourArea(box)) <= 2000 and math.fabs(cv2.contourArea(box)) >= 60:
                        if rect not in con:
                            con.append(rect)  # 将旋转矩阵存进去
                            cnt = cnt + 1
                            break

            print("second shuma:", cnt)

    return cnt, con
    """

# 从找九宫格数字到对每个旋转矩阵作转换
def getSquaresList(gray, squares):  # 对每个格子的旋转矩阵作透视变换,返回一张图片
    result = []
    for square in squares:
        center = square[0]  # center有可能为负值
        box = cv2.boxPoints(square)
        box = np.int0(box)
        rect_pts = np.float32(sortCorners(box, center))  # 返回矩阵四个点
        side_length = 52
        quad_pts = np.float32([[0, 0], [side_length, 0], [side_length, side_length], [0, side_length]])
        M = cv2.getPerspectiveTransform(rect_pts, quad_pts)  # 透视变换让每张图片变正
        dst = cv2.warpPerspective(gray, M, (side_length, side_length))
        dst = dst[1:side_length, 1:side_length]

        dst = cv2.resize(dst, (48, 48))
        result.append(dst)

    return result

# 找到九宫格区域
# gray_for_jiugongge是一个完整的灰度图
def FindSquares(pswArea, gray_for_jiugongge):
    squares = []
    squares2 = []
    cnt2 = 0
    con2 = []
    pics = []

    # 计算九宫格的旋转矩阵
    center, size, _ = pswArea
    new_Y = center[1] + size[1] * 3.3
    new_Width = size[0] * 2.1
    new_Height = size[1] * 5.3
    jiugonge = ((center[0], new_Y), (new_Width, new_Height), _)  # 都是一些数值，越界没事

    # 这里要防止越界,因为要用
    box = cv2.boxPoints(jiugonge)
    box = np.int0(box)

    # 计算边界
    center = jiugonge[0]  # 矩阵中心
    top = []
    bot = []
    for corner in box:  # 以中心为基准点分上下
        if corner[1] < center[1]:
            top.append(corner)
        else:
            bot.append(corner)

    tl = top[1] if (top[0][0] > top[1][0]) else top[0]
    tr = top[0] if (top[0][0] > top[1][0]) else top[1]
    bl = bot[1] if (bot[0][0] > bot[1][0]) else bot[0]

    # 在完整灰度图中画出九宫格矩阵
    #cv2.drawContours(gray_for_jiugongge, [box], 0, (255, 255, 255), 2)
    #cv2.imshow("jiugongge", gray_for_jiugongge)  # 这是原图上画九宫格区域

    limit(tl)
    limit(bl)
    limit(tr)

    if tl[1] == 480 and bl[1] == 480:
        return cnt2, pics, squares2

    # 九宫格的ROI区域
    jiugongge_Area = gray_for_jiugongge[tl[1]:bl[1], tl[0]:tr[0]]
    gray_for_jiugongge_3 = gray_for_jiugongge.copy()  # 截九宫格数字用的全图
    gray_for_jiugongge_2 = gray_for_jiugongge.copy()  # 自动标用的全图

    ret, thresh = cv2.threshold(jiugongge_Area, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    thresh = cv2.erode(thresh, (3, 3), iterations=3)
    thresh = cv2.dilate(thresh, element)
    #cv2.imshow("thresh", thresh)   # 对九宫格区域二值化

    # 找外层轮廓
    thresh, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 筛选出九宫格
    for counter in contours:  # 每个轮廓
        if math.fabs(cv2.contourArea(counter)) <= 10000 and math.fabs(cv2.contourArea(counter)) >= 400:
            x3, y3, w3, h3 = cv2.boundingRect(counter)
            # 通过正矩阵长宽比去除左右小方格
            if (w3 / h3 < 1.5):
                rect = cv2.minAreaRect(counter)
                con2.append(rect)
                cnt2 = cnt2 + 1
    print("jiugonge num:", cnt2)

    # 等于九个时，由轮廓中心点扩增得到旋转矩形
    # 注意这里找到的是ROI区域的，不是整幅图的
    if cnt2 == 9:
        Sort(con2)  # 排序

        for index_22 in range(len(con2)):
            center, size, _ = con2[index_22]
            con2[index_22] = ((center[0] + tl[0], center[1] + tl[1]), (size[0], size[1]), _)

        con3 = con2[:]

        rect1 = con2[0]
        rect2 = con2[1]
        rect3 = con2[3]

        center1, size1, _ = rect1
        center2, size2, _ = rect2
        center3, size3, _ = rect3

        width_dis = center2[0] - center1[0]
        height_dis = center3[1] - center1[1]

        # 求旋转矩阵的长和宽
        width_ = 0.55 * width_dis
        height_ = 0.82 * height_dis

        for rect in con2:  # 转换为旋转矩阵squares
            center, size, _ = rect
            width, height = size
            y = center[1] * 0.99
            width, height = width_, height_
            _ = 0   # 设置角度为0度

            # 由轮廓重心扩增 得到旋转矩形
            rect = ((center[0], y), (width, height), _)

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # 对ROI画, 原图也会有痕迹
            cv2.drawContours(gray_for_jiugongge, [box], 0, (255, 255, 255), 1)
            squares.append(rect)

        pics = getSquaresList(gray_for_jiugongge_3, squares)  # 对新的区域图作提取数字

        #cv2.imshow("detection_2", gray_for_jiugongge)

        # --------------------求自动标用的格子-----------------------

        rect1 = con3[0]
        rect2 = con3[1]
        rect3 = con3[3]

        center1, size1, _ = rect1
        center2, size2, _ = rect2
        center3, size3, _ = rect3

        width_dis = center2[0] - center1[0]
        height_dis = center3[1] - center1[1]

        # 求旋转矩阵的长和宽
        width_ = 0.75 * width_dis
        height_ = 0.72 * height_dis

        for rect in con3:  # 转换为旋转矩阵squares
            center, size, _ = rect
            width, height = size
            y = center[1] * 0.99
            width, height = width_, height_
            _ = 0   # 设置角度为0度

            # 由轮廓重心扩增 得到旋转矩形
            rect = ((center[0], y), (width, height), _)

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            cv2.drawContours(gray_for_jiugongge_2, [box], 0, (255, 255, 255), 1)
            squares2.append(rect)

        #cv2.imshow("detection_3", gray_for_jiugongge_2)

    return cnt2, pics, squares2

def attack(target, pics,_results, angle, port=1):
    global last_attack
    success_flag = False
    list_of_imvalue = []

    for pic in pics:
        imval = []
        width, height = pic.shape
        for i in range(0, height):
            for j in range(0, width):
                imval.append(pic[i, j])
                #存入图片的每个像素点

        #前面二值化过了，这里对每个值取反，保存在list里
        #tva = [(255 - j) * 1.0 / 255.0 for j in imval]
        tva = [j * 1.0 / 255.0 for j in imval]
        list_of_imvalue.append(tva)

    #tensorflow作预测
    results = prediction.eval(feed_dict={x: list_of_imvalue, keep_prob: 1.0}, session=sess_2)
    results = results.tolist()

    results = [num+1 for num in results]

    probability = y_conv.eval(feed_dict={x: list_of_imvalue, keep_prob: 1.0}, session=sess_2)

    #print(probability)

    print("1st results:", results)

    cout = 0
    index1 = []
    for nn in range(len(results)):
        if int(results[nn]) == 2:
            cout = cout + 1
            index1.append(nn)
    if cout == 2:   # 出现两个2
        if probability[index1[0]][1] > probability[index1[1]][1]:
            results[index1[0]] = 2
            results[index1[1]] = 1
        else:
            results[index1[0]] = 1
            results[index1[1]] = 2

    print("2nd results:", results)
    print("last results:",_results)
    print("target number: ", target)

    num_of_equal = 0

    """
    这里的if...else语句，保证一开始不用等就可以打，
    如果一开始要等变化才开始打的话，把下面的else去掉
    """

    if len(_results) == 0:
        _results[:] = results[:]
    else:
        for i in range(0, 9):
            if results[i] == _results[i]:
                num_of_equal += 1


    #九个格子和上一次一样的话，就不进行打击
    #或者当前很多格子一样时，一般出现在屏幕切换时
    if num_of_equal >= 7 or int(len(set(results))) <= 7:
        print("gezi same,pass")
        return False

    #不一样的话，进行打击
    else:
        for i in range(0, 9):
            if results[i] == 0:
                results[i] = 9
            if results[i] == target:  # 如果手写数字和数码管数字相等
                # 向右时作补偿
                """
                if last_attack == 1 or last_attack == 4 or last_attack == 7:
                    if (i + 1) == 2 or (i + 1) == 5 or (i + 1) == 8:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        if (i + 1) == 2:
                            temp = angle[0][1]
                        if (i + 1) == 5:
                            temp = angle[3][1]
                        if (i + 1) == 8:
                            temp = angle[6][1]
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[0][0]) * 0.1, temp)

                    if (i + 1) == 3 or (i + 1) == 6 or (i + 1) == 9:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        if (i + 1) == 3:
                            temp = angle[0][1]
                        if (i + 1) == 6:
                            temp = angle[3][1]
                        if (i + 1) == 9:
                            temp = angle[6][1]
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[1][0]) * 0.2, temp)
                """
                if last_attack == 2 or last_attack == 5 or last_attack == 8:
                    if (i + 1) == 3 or (i + 1) == 6 or (i + 1) == 9:
                        print("before: x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                        angle[i] = (angle[i][0] - (angle[i][0] - angle[0][0]) * 0.07, angle[i][1])

                print("last_attack: ", last_attack)
                last_attack = i + 1

                angle_x = angle[i][0]
                angle_y = angle[i][1]
                angle_x = Decimal(str(angle_x)).quantize(Decimal('0.00'))  # 保留两位小数
                angle_y = Decimal(str(angle_y)).quantize(Decimal('0.00'))  # 保留两位小数
                angle_x = str(int((float(angle_x) * 100)))  # 转为字符串
                angle_y = str(int((float(angle_y) * 100)))  # 转为字符串

                # port.write(("x" + angle_x + "y" + angle_y + "e").encode())
                # port.write(('n' + str(i + 1) + 'e').encode())

                print("x: " + str(angle[i][0]) + "  y: " + str(angle[i][1]))
                print('\n')

                _results[:] = results[:]
                success_flag = True
                break

        for i in range(len(angle)):
            print("x: ", angle[i][0], "  y: ", angle[i][1])

    return success_flag

# 公共main函数
def main():
    # ---------------------公共部分------------------------
    global cap
    global big_or_small_flag
    big_or_small_flag = 'B'    # 默认小符
    global address

    global last_attack  # 上一次击打第几个格子
    global destNums, destNumsTEMP, _results  # 数码管的数字存储
    last_attack = 0
    destNums = []  # 数码管的数字存储
    destNumsTEMP = [11, 12, 13, 14, 15]  # 一开始总是不一样的
    _results = []  # 存储九宫格

    global cameraMatrix, distCoeffs
    global digit_count
    digit_count = np.zeros((5, 10), dtype=np.uint8)

    global jishu
    jishu = 0

    # 相机内参
    cameraMatrix = np.float32([[608.4612, 0, 333.9768],
                               [0, 611.7066, 272.1556],
                               [0, 0, 1]])
    # 畸变系数
    distCoeffs = np.float32([[-0.4502, 0.3081, 0, 0]])

    global coordinate_y
    coordinate_y = cv2.imread('./coordinate/coordinatey.png', 0)
    global coordinate_x_up
    coordinate_x_up = cv2.imread('./coordinate/coordinatexup.png', 0)
    global coordinate_x_down
    coordinate_x_down = cv2.imread('./coordinate/coordinatexdown.png', 0)
    global coordinate_z
    coordinate_z = cv2.imread('./coordinate/coordinatez.png', 0)

    ###########################################################
    # ---------------------小符定义部分------------------------
    g1 = tf.Graph()
    with g1.as_default():
        global x_xiao, keep_prob_xiao, prediction_xiao, sess_1

        def weight_variable(shape):
            initial = tf.truncated_normal(shape, stddev=0.1)
            return tf.Variable(initial)

        def bias_variable(shape):
            initial = tf.constant(0.1, shape=shape)
            return tf.Variable(initial)

        def conv2d(x, W):
            return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

        def max_pool_2x2(x):
            return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

        x_xiao = tf.placeholder(tf.float32, [None, 784])

        x_image_xiao = tf.reshape(x_xiao, [-1, 28, 28, 1])

        W_conv1_xiao = weight_variable([5, 5, 1, 32])
        b_conv1_xiao = bias_variable([32])

        h_conv1_xiao = tf.nn.relu(conv2d(x_image_xiao, W_conv1_xiao) + b_conv1_xiao)
        h_pool1_xiao = max_pool_2x2(h_conv1_xiao)

        W_conv2_xiao = weight_variable([5, 5, 32, 64])
        b_conv2_xiao = bias_variable([64])

        h_conv2_xiao = tf.nn.relu(conv2d(h_pool1_xiao, W_conv2_xiao) + b_conv2_xiao)
        h_pool2_xiao = max_pool_2x2(h_conv2_xiao)

        W_fc1_xiao = weight_variable([7 * 7 * 64, 1024])
        b_fc1_xiao = bias_variable([1024])

        h_pool2_flat_xiao = tf.reshape(h_pool2_xiao, [-1, 7 * 7 * 64])
        h_fc1_xiao = tf.nn.relu(tf.matmul(h_pool2_flat_xiao, W_fc1_xiao) + b_fc1_xiao)

        keep_prob_xiao = tf.placeholder(tf.float32)
        h_fc1_drop_xiao = tf.nn.dropout(h_fc1_xiao, keep_prob_xiao)

        W_fc2_xiao = weight_variable([1024, 10])
        b_fc2_xiao = bias_variable([10])

        y_conv_xiao = tf.nn.softmax(tf.matmul(h_fc1_drop_xiao, W_fc2_xiao) + b_fc2_xiao)

        saver_xiao = tf.train.Saver()

        sess_1 = tf.Session(graph=g1)

        sess_1.run(tf.global_variables_initializer())
        saver_xiao.restore(sess_1, "./xiaofu_model/model.ckpt")  # 加载小符模型在sess1中
        print("XiaoFu_Model restored.")

        prediction_xiao = tf.argmax(y_conv_xiao, 1)

    ###########################################################
    # ---------------------大符定义部分------------------------
    g2 = tf.Graph()
    with g2.as_default():
        global x, keep_prob, sess_2, prediction, y_conv
        x = tf.placeholder(tf.float32, [None, 2304])

        def weight_variable(shape):
            var = tf.Variable(tf.truncated_normal(shape, stddev=0.1))
            return var

        def bias_variable(shape):
            initial = tf.constant(0.1, shape=shape)
            return tf.Variable(initial)

        def conv2d(x, W):
            return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

        def max_pool_2x2(x):
            return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

        x_image = tf.reshape(x, [-1, 48, 48, 1])

        W_conv1 = weight_variable([5, 5, 1, 32])
        b_conv1 = bias_variable([32])

        h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
        h_pool1 = max_pool_2x2(h_conv1)

        W_conv2 = weight_variable([5, 5, 32, 64])
        b_conv2 = bias_variable([64])

        h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
        h_pool2 = max_pool_2x2(h_conv2)

        W_fc1 = weight_variable([12 * 12 * 64, 1024])
        b_fc1 = bias_variable([1024])

        h_pool2_flat = tf.reshape(h_pool2, [-1, 12 * 12 * 64])
        h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

        keep_prob = tf.placeholder(tf.float32)
        h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

        W_fc2 = weight_variable([1024, 9])
        b_fc2 = bias_variable([9])

        y_conv = tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

        saver = tf.train.Saver()

        sess_2 = tf.Session(graph=g2)

        sess_2.run(tf.global_variables_initializer())
        saver.restore(sess_2, "./DAFU_model/model.ckpt")  # 加载模型在sess中
        print("DAFU_Model restored.")
        prediction = tf.argmax(y_conv, 1)

    global lll
    lll = 0
    for ggg in address:
        # 在外部定义Capture类
        cap = cv2.VideoCapture("/data/Team2/RP/Video/"+ggg+'.avi')  # 读取摄像头

        if not cap.isOpened:
            print("Camera not opened.")
            return 0

        # -----------------------摄像头读取-----------------------
        # 不用重新创建窗口，用同一个，减少时间
        capture_buffer = np.empty(shape=(480, 640, 3), dtype=np.uint8)

        while(True):
            while_start = time()

            if jishu>750:
                jishu = 0
                break

            cap.read(capture_buffer)
            ret, img = cap.read(capture_buffer)

            if ret and (big_or_small_flag == 'S' or big_or_small_flag == 'B'):
                #cv2.imshow("Original", img)

                if big_or_small_flag == 'S':
                    XiaoFu_main(img)
                else:
                    DAFU_main(img)

            print("while time cost: " + str(time() - while_start))
            cv2.waitKey(1)

        cv2.destroyAllWindows()
        cap.release()
        lll += 1

    sess_1.close()
    sess_2.close()

def XiaoFu_main(img):
    global cameraMatrix, distCoeffs
    global destNums, destNumsTEMP, _results, big_or_small_flag
    global last_attack  # 上一次击打第几个格子
    global digit_count

    img2 = img.copy()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # 去噪，模糊
    gray_for_digit = gray.copy()

    # 先预处理加找出九宫格
    con, cnt = Xiaofu_FindSquares(gray)  # con是单纯的轮廓
    cv2.drawContours(img2, con, -1, (0, 0, 255), 2)
    #cv2.namedWindow("detection")
    #cv2.imshow("detection", img2)
    # 无论有没有9个都会标出来

    squares = []

    if cnt == 9:  # 当找出九个格子时才开始找数码管数字
        if big_or_small_flag == 'S':
            for rect in con:  # 转换为旋转矩阵squares
                rect = cv2.minAreaRect(rect)
                center, size, _ = rect
                width, height = size
                if (width < height):  # 旋转矩阵有可能长宽和定义的不一样
                    width, height = height, width
                    _ += 90.0
                    rect = (center, (width, height), _)

                # 进行裁剪是因为想让手写数字没有外界噪音干扰
                newWidth = int(width * 0.8)
                newHeight = int(height * 0.9)
                rect = (center, (newWidth, newHeight), _)
                squares.append(rect)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
            #cv2.imshow("66666", img)  #裁剪过后的只能拿来作手写数字识别

            Sort(squares)

            # -----------------------自动标定求出所有格子该转的角度--------------------------
            angle_x = 0.0
            angle_y = 0.0

            angle = []
            for a in squares:
                angleSolver = AngleSolver(cameraMatrix, distCoeffs, 280.0, 160.0)  # 初始化类

                # 相机坐标系转云台坐标系的平移矩阵
                tran = np.float32([[130.50], [200.0], [212.0]])

                angleSolver.setRelationPoseCameraPTZ(tran, 0)  # 设置云台相对相机的参数

                angle_x, angle_y = angleSolver.getAngle(a)  # 获取角度

                angle.append((angle_x, angle_y))

            # -----------------------识别数码管数字-----------------------
            result = []
            center, size, _ = squares[1]  # 框出数码管位置
            newY = center[1] - 1.5 * size[1]
            newHeight = size[1] * 1.4
            newWidth = size[0] * 2.4
            pswArea = ((center[0], newY), (newWidth, newHeight), _)
            # 数码管的最小面积矩阵

            # 利用四个点作轮廓点
            box = cv2.boxPoints(pswArea)
            box = np.int0(box)
            cv2.drawContours(gray, [box], 0, (255, 255, 255), 2)

            #cv2.imshow("huidu1", gray)

            result = digital_number_treat(gray_for_digit, pswArea)

            print("digital num:", result)

            # -----------------------Tensorflow识别-----------------------
            if len(result) != 0 and int(len(set(result))) == 5:
                count_Same_shuma = 0  # 计算相同的数量
                for i in range(0, 5):
                    if result[i] == destNumsTEMP[i]:
                        count_Same_shuma += 1

                # 不一样，就变成一样
                if (count_Same_shuma <= 2):  # 如果相同数量少，表示不是同一个数码管数字
                    destNums = result
                    digit_count = np.zeros((5, 10), dtype=np.uint8)

                if len(destNums) == 5:  # destNumsTEMP用来和当前帧作比较
                    destNumsTEMP = destNums[:]

                for i in range(len(result)):
                    digit_count[i][result[i] - 1] += 1

                pics = XiaoFu_getSquaresList(gray, squares)  # 取得九个格子的列表

                #for pic_index in range(0, 9):
                    #cv2.imshow(str(pic_index), pics[pic_index])

                # 每一帧放入数码管数字的第一个
                if len(destNums) != 0:

                    # 统计频率最大的数 作为下一个要打的
                    destNums[0] = digit_count[5 - len(destNums)].tolist().index(max(digit_count[5 - len(destNums)])) + 1

                    boolean = Xiaofu_attack(destNums[0], pics, _results, angle)
                    if (boolean):
                        destNums.pop(0)
                        if len(destNums) == 0:
                            print("Attack Over.")

            else:
                print("pass")

def DAFU_main(img):
    global cameraMatrix, distCoeffs
    global destNums, destNumsTEMP, _results, big_or_small_flag
    global last_attack  # 上一次击打第几个格子
    global digit_count
    global jishu
    global lll

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # 去噪,用作灯管数字
    gray_for_digit = gray.copy()
    gray_for_jiugongge = gray.copy()

    cnt, con = FindDigit(img)  # 找出所有的数码管数字

    for rect in con:
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (255, 255, 255), 1)

    # cv2.imshow("detection", img)

    if (cnt == 5):  # 找到了五个数字,圈出数码管并识别
        if big_or_small_flag == 'B':
            min_centerX = 9999
            max_centerX = -1
            min_centerY = 9999
            max_centerY = -1

            # 找边界
            for rect in con:
                center, size, _ = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                for corner in box:
                    if corner[1] > max_centerY:
                        max_centerY = corner[1]
                    if corner[1] < min_centerY:
                        min_centerY = corner[1]
                    if corner[0] > max_centerX:
                        max_centerX = corner[0]
                    if corner[0] < min_centerX:
                        min_centerX = corner[0]

            # 找到数码管的位置
            newX = (max_centerX + min_centerX) / 2
            newY = (max_centerY + min_centerY) / 2
            newWidth = max_centerX - min_centerX
            newHeight = max_centerY - min_centerY
            size_shuma = (newWidth * 1.1, newHeight * 1.1)
            pswArea = ((newX, newY), size_shuma, 0)  # 数码管的角度设置为0

            wrong_flag = 0

            """
            # 如果数码管越界，不进行下一步
            for corner in box:
                limit
                if corner[1] > 480 or corner[1] < 0 or corner[0] > 640 or corner[0] < 0:
                    wrong_flag = 1
                    print(11111111111111111)
                    print(corner)
            """

            # 如果不越界 (在正常范围内)
            if wrong_flag == 0:

                # 利用四个点作轮廓点
                box = cv2.boxPoints(pswArea)
                box = np.int0(box)
                # 画出数码管区域
                #cv2.drawContours(gray, [box], 0, (255, 255, 255), 2)
                #cv2.imshow("huidu1", gray)

                result = []
                result = digital_number_treat(gray_for_digit, pswArea)  # 识别数码管
                print("digital num:", result)

                if len(result) != 0 and int(len(set(result))) == 5:
                    count_Same_shuma = 0  # 计算相同的数量
                    for i in range(0, 5):
                        if result[i] == destNumsTEMP[i]:
                            count_Same_shuma += 1

                    if (count_Same_shuma <= 2):  # 如果相同数量少，表示不是同一个数码管数字
                        destNums = result
                        digit_count = np.zeros((5, 10), dtype=np.uint8)

                    if len(destNums) == 5:  # destNumsTEMP用来和当前帧作比较
                        destNumsTEMP = destNums[:]

                    for i in range(len(result)):
                        digit_count[i][result[i] - 1] += 1

                    squares_2 = []
                    cnt2, pics, squares_2 = FindSquares(pswArea, gray_for_jiugongge)  # 开始圈九宫格

                    if cnt2 == 9:
                        # for pic_index in range(0, 9):
                        # cv2.imshow(str(pic_index), pics[pic_index])
                        # pic是48X48的图片列表

                        for pic_index in range(0, 9):
                            cv2.imwrite(address[lll] + '/' + str(jishu)+".jpg", pics[pic_index])
                            jishu += 1
                        # 每一帧放入数码管数字的第一个

                        # -----------------------自动标定求出所有格子该转的角度--------------------------
                        angle_x = 0.0
                        angle_y = 0.0

                        angle = []
                        for a in squares_2:
                            angleSolver = AngleSolver(cameraMatrix, distCoeffs, 280.0, 160.0)  # 初始化类

                            # 相机坐标系转云台坐标系的平移矩阵
                            tran = np.float32([[130.50], [200.0], [212.0]])

                            angleSolver.setRelationPoseCameraPTZ(tran, 0)  # 设置云台相对相机的参数

                            angle_x, angle_y = angleSolver.getAngle(a)  # 获取角度

                            angle.append((angle_x, angle_y))

                        # ------------------------打击--------------------------

                        if len(destNums) != 0:

                            # 统计频率最大的数 作为下一个要打的
                            destNums[0] = digit_count[5 - len(destNums)].tolist().index(max(digit_count[5 - len(destNums)])) + 1

                            boolean = attack(destNums[0], pics, _results, angle)
                            if (boolean):
                                #destNums.pop(0)
                                if len(destNums) == 0:
                                    print("Attack Over.")
                else:
                    print("shumaguan same")

if __name__ == "__main__":
    main()