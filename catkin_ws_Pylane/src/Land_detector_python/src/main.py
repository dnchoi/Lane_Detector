#!/usr/bin/python3

import cv2
import rospy
from std_msgs.msg import String
import numpy as np
import time
from capture_reader import video_cap
from utils import tools
import math

class visualize:
    def __init__(self):
        super().__init__()

    def set_roi_region(self, pf, xywh):
        crop_img = pf[xywh[1] : xywh[1]+xywh[3], xywh[0] : xywh[0]+xywh[2]]
        return crop_img

    def _draw_roi(self, f, xywh):
        f = cv2.rectangle(f, (xywh[0], xywh[1]), (xywh[0]+xywh[2], xywh[1]+xywh[3]), (255, 255, 0), 1)
        return f

    def visualized(self, f, winname, x, y):
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, x, y)
        cv2.imshow(winname, f)

    def vision_pub(self, publisher, th, s, b, vaild):
        receive_data = "{},{},{},{}".format(str(round(th, 2)), str(s), str(b), str(vaild))
        publisher.publish(receive_data)
        
class Lane_detector:
    def __init__(self):
        super().__init__()
        self._init()

    def _init(self):
        self.capture = video_cap(0, "480p", 30)
        # self.capture = video_cap("/home/vcbuild/Downloads/(seg)WIN_20210303_13_35_05_Pro.avi", "480p", 300)
        # self.capture = video_cap("/home/vcbuild/golf_video/210122_IRCAM.mp4", "480p", 300)
        self.cam_w, self.cam_h = self.capture.check_cam_width_height()
        # cam_w = 640
        # cam_h = 480

        self.TOOL = tools()

        # roi_list = "{} {} {} {} {} {} {} {}".format(int(cam_w - cam_w + 1), int(cam_h * 0.4), int(cam_w / 2), int(cam_h * 0.5), int(cam_w / 2), int(cam_h * 0.4), int(cam_w / 2 - 1), int(cam_h * 0.5))
        self.left_region = []
        self.left_region.append(10) # x
        self.left_region.append(250) # y
        self.left_region.append(310) # w
        self.left_region.append(200) # h
        # self.left_region.append(int(self.cam_w - self.cam_w + 1)) # x
        # self.left_region.append(int(self.cam_h * 0.4)) # y8
        # self.left_region.append(int(self.cam_w / 2))
        # self.left_region.append(int(self.cam_h * 0.5))

        self.right_region = []
        self.right_region.append(320)
        self.right_region.append(250)
        self.right_region.append(310)
        self.right_region.append(200)
        # self.right_region.append(int(self.cam_w / 2))
        # self.right_region.append(int(self.cam_h * 0.4))
        # self.right_region.append(int(self.cam_w / 2 - 1))
        # self.right_region.append(int(self.cam_h * 0.5))

        self.filtered_Angle = 60
        self.alpha = 0.1
        self.LLnPt = None
        self.RLnPt = None
        self.LLnMovAvg = [0, 0]
        self.RLnMovAvg = [0, 0]
        self.Up_offset = int(self.cam_w * 0.01)
        self.Down_offset = int(self.cam_h * 0.49)
        self.Up_offset = 10
        self.Down_offset = 180
        self.HeadingPt = [320, self.left_region[1]+self.Up_offset]
        self.show = visualize()

        rospy.init_node('vision')
        self.vision_p = rospy.Publisher('vision_can', String, queue_size=1000)

    def main(self):
        ori_img = "frame"
        left_img = "left"
        right_img = "right"
        L_canny_img = "L_Canny"
        R_canny_img = "R_Canny"
        cam_cnt = 0
        prevTime = 0 

        while True:
            if self.capture.is_open():
                frame = self.capture.read_frame()


                if frame is not None and cam_cnt > 30:
                    # if frame.shape[0] > 640:
                        # frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
                    process_frame = frame.copy() 
                    # print(roi_list)
                    left_roi = self.show.set_roi_region(process_frame, self.left_region)
                    right_roi = self.show.set_roi_region(process_frame, self.right_region)
                    frame = self.show._draw_roi(frame, self.left_region)
                    frame = self.show._draw_roi(frame, self.right_region)
                    O_h, O_w, O_c = frame.shape
                    L_h, L_w, L_c = left_roi.shape
                    R_h, R_w, R_c = right_roi.shape

                    left_roi = cv2.medianBlur(left_roi, 7)
                    right_roi = cv2.medianBlur(right_roi, 7)
                    kernel_ = np.ones((5,5), np.uint8)
                    left_roi = cv2.erode(left_roi, kernel_, iterations=5) 
                    left_roi = cv2.dilate(left_roi, kernel_, iterations=5)
                    right_roi = cv2.erode(right_roi, kernel_, iterations=5) 
                    right_roi = cv2.dilate(right_roi, kernel_, iterations=5)


                    L_canny = cv2.Canny(left_roi, 10, 150)
                    R_canny = cv2.Canny(right_roi, 10, 150)
                    
                    L_lines = cv2.HoughLines(L_canny,1,np.pi/180,10)
                    if L_lines is not None:
                        LLnPt = self.TOOL.filter_houghlines(False, L_lines, self.filtered_Angle, left_roi)    
                    R_lines = cv2.HoughLines(R_canny,1,np.pi/180,10)
                    if R_lines is not None: 
                        RLnPt = self.TOOL.filter_houghlines(True, R_lines, self.filtered_Angle, right_roi)    

                    if LLnPt is not None and RLnPt is not None:    
                        if math.fabs(LLnPt[0]) <= 300:
                            self.LLnMovAvg[0] = self.alpha * LLnPt[0] + (1 - self.alpha) * self.LLnMovAvg[0]
                            self.LLnMovAvg[1] = self.alpha * LLnPt[1] + (1 - self.alpha) * self.LLnMovAvg[1]
                            
                        if math.fabs(RLnPt[0]) >= -100:
                            self.RLnMovAvg[0] = self.alpha * RLnPt[0] + (1 - self.alpha) * self.RLnMovAvg[0]
                            self.RLnMovAvg[1] = self.alpha * RLnPt[1] + (1 - self.alpha) * self.RLnMovAvg[1]

                    self.TOOL.draw_polar_line(self.LLnMovAvg, [self.left_region[0], self.left_region[1]], frame, False)
                    self.TOOL.draw_polar_line(self.RLnMovAvg, [self.right_region[0], self.right_region[1]], frame, True)
                    
                    if self.LLnMovAvg[1] > 0 and self.RLnMovAvg[1] > 0:
                        UpLfLnPt = self.TOOL.add_i(self.TOOL.getIntersection(self.TOOL.polar2cart(self.LLnMovAvg), (-1 / math.tan(self.LLnMovAvg[1])), [0, self.Up_offset], 0), [self.left_region[0], self.left_region[1]])
                        UpRfLnPt = self.TOOL.add_i(self.TOOL.getIntersection(self.TOOL.polar2cart(self.RLnMovAvg), (-1 / math.tan(self.RLnMovAvg[1])), [0, self.Up_offset], 0), [self.right_region[0], self.right_region[1]])
                        LoLfLnPt = self.TOOL.add_i(self.TOOL.getIntersection(self.TOOL.polar2cart(self.LLnMovAvg), (-1 / math.tan(self.LLnMovAvg[1])), [0, self.Down_offset], 0), [self.left_region[0], self.left_region[1]])
                        LoRfLnPt = self.TOOL.add_i(self.TOOL.getIntersection(self.TOOL.polar2cart(self.RLnMovAvg), (-1 / math.tan(self.RLnMovAvg[1])), [0, self.Down_offset], 0), [self.right_region[0], self.right_region[1]])
                        cv2.line(frame, (UpLfLnPt[0], UpLfLnPt[1]), (UpRfLnPt[0], UpRfLnPt[1]), (0,0,255), 1)
                        cv2.line(frame, (LoLfLnPt[0], LoLfLnPt[1]), (LoRfLnPt[0], LoRfLnPt[1]), (0,0,255), 1)
                        
                        UpCenterPt = (int((UpLfLnPt[0] + UpRfLnPt[0])/2), int(self.left_region[1]+self.Up_offset))
                        LoCenterPt = (int((LoLfLnPt[0] + LoRfLnPt[0])/2), int(self.left_region[1]+self.Down_offset))
                        
                        frame = cv2.circle(frame, UpCenterPt, 2, (0, 255, 0), 3, 8, 0)
                        frame = cv2.circle(frame, LoCenterPt, 2, (0, 255, 0), 3, 8, 0)
                        
                        keyIn = cv2.waitKey(1)
                        
                        if keyIn == 97: # command A
                            self.HeadingPt[0] -= 2
                        elif keyIn == 100: # command D
                            self.HeadingPt[0] += 2
                            
                        frame = cv2.circle(frame, (self.HeadingPt[0], self.HeadingPt[1]), 2, (0, 0, 255), 3, 8, 0)
                        cv2.line(frame, (UpCenterPt[0], UpCenterPt[1]), (LoCenterPt[0], LoCenterPt[1]), (0,255,255), 2)
                        cv2.line(frame, (self.HeadingPt[0], self.HeadingPt[1]), (LoCenterPt[0], LoCenterPt[1]), (0,255,255), 2)
                        
                        Lane_slope = (UpCenterPt[0] - self.HeadingPt[0]) / (self.Down_offset - self.Up_offset)
                        
                        TargetAngle = np.rad2deg(math.atan(Lane_slope)) * -1
                        curTime = time.time()
                        sec = curTime - prevTime
                        prevTime = curTime
                        fps = 1/(sec)
                        
                        print_text = "Theta : {0:.3f} FPS : {1:.3f} CAN : {2:.3f}".format(TargetAngle, fps, 9000+(TargetAngle*100))
                        cv2.putText(frame, print_text, (10,30), 0, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                        self.show.vision_pub(self.vision_p, TargetAngle, 2000, 0, "True")

                        if keyIn == 113: #command Q
                            break
                        
                    self.show.visualized(frame, ori_img, 0, 0)
                    self.show.visualized(left_roi, left_img, O_w+10, 0)
                    self.show.visualized(right_roi, right_img, O_w + L_w+50, 0)
                    self.show.visualized(L_canny, L_canny_img, O_w+10, L_h+100)
                    self.show.visualized(R_canny, R_canny_img, O_w + L_w+50, L_h+100) 
                else:
                    print("@@    Checked Cam or Video    @@")
                    cam_cnt += 1
                    if cam_cnt > 100:
                        print("@@    Good bye    @@")
                        break
                    continue

        self.capture.close_cap()

if __name__ == "__main__":
    LD = Lane_detector()
    LD.main()
