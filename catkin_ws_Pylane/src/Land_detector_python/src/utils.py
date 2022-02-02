import cv2
import numpy as np

class tools:
    def __init__(self):
        super().__init__()

    def min_max(self, lines, min_or_max):
        total_lines = 100
        if min_or_max:
            if total_lines < len(lines):
                return lines
            elif total_lines > len(lines):
                return lines
        elif not min_or_max:
            if total_lines < len(lines):
                return lines
            elif total_lines > len(lines):
                return lines
         
    def polar2cart(self, pt):
        X = int(pt[0] * np.cos(pt[1]))
        Y = int(pt[0] * np.sin(pt[1]))
        
        res = [X, Y]
        return res
    
    def filter_houghlines(self, isRight, lines, ang, frame):
        avgPt = []
        max_Angle = 90; cnt = 0
        tmpTheta = 0; sumTheta = 0; sumRho = 0; sumDeviation = 0
        nn = 0
        for lines_tmp in lines:
            nn += 1
            rho,theta = lines_tmp[0]
            if isRight:
                tmpTheta = theta
            else:
                tmpTheta = theta

            if isRight:
                if tmpTheta > np.deg2rad(180-ang) and tmpTheta < (np.pi):
                    sumTheta += theta
                    sumRho += rho
                    cnt+=1
                    scale = frame.shape[0] + frame.shape[1]

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + scale*(-b))
                    y1 = int(y0 + scale*(a))
                    x2 = int(x0 - scale*(-b))
                    y2 = int(y0 - scale*(a))
                    cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)    
            else:
                if tmpTheta < np.deg2rad(max_Angle) and tmpTheta < np.deg2rad(ang):
                    sumTheta += theta
                    sumRho += rho
                    cnt+=1
                    scale = frame.shape[0] + frame.shape[1]

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + scale*(-b))
                    y1 = int(y0 + scale*(a))
                    x2 = int(x0 - scale*(-b))
                    y2 = int(y0 - scale*(a))
                    cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),1)
            if nn > 100:
                break
            
        if cnt > 0:  
            avgRho = sumRho / cnt
            avgTheta = sumTheta / cnt
            avgPt.append(avgRho)
            avgPt.append(avgTheta)
                
            return avgPt

    def add_i(self, pt, offset):
        return [int(pt[0] + offset[0]), int(pt[1] + offset[1])]
    
    def draw_polar_line(self, pt, offset, frame, isRight):
        rho = pt[0]
        theta = pt[1]
        
        pt1 = [0,0]
        pt2 = [0,0]
        if isRight:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho; y0 = b*rho
            
            pt1[0] = np.round(x0 + 1000 * (-b))
            pt1[1] = np.round(y0 + 1000 * (a))
            pt2[0] = np.round(x0 - 1000 * (-b))
            pt2[1] = np.round(y0 - 1000 * (a))
        else:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho; y0 = b*rho
            
            pt1[0] = np.round(x0 + 1000 * (-b))
            pt1[1] = np.round(y0 + 1000 * (a))
            pt2[0] = np.round(x0 - 1000 * (-b))
            pt2[1] = np.round(y0 - 1000 * (a))
            
        line_pt1 = self.add_i(pt1, offset)
        line_pt2 = self.add_i(pt2, offset)
        
        cv2.line(frame,(line_pt1[0], line_pt1[1]),(line_pt2[0], line_pt2[1]),(255,0,0),2)
        
    def getIntersection(self, pt1, slope1, pt2, slope2):
        intersection_pt = [0, 0]
        intersection_pt[0] = ((slope1 * pt1[0]) - (slope2 * pt2[0]) + pt2[1] - pt1[1]) / (slope1 - slope2)
        intersection_pt[1] = slope1 * (intersection_pt[0] - pt1[0]) + pt1[1]
        return intersection_pt