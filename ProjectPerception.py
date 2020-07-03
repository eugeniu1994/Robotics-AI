import numpy as np
import cv2 as cv

def getImageJacobian(u, v, z, f, diameter):
    Jv = np.zeros((3, 3))

    Jv[0][0] = -f / z
    Jv[0][1] = 0
    Jv[0][2] = u / z

    Jv[1][0] = 0
    Jv[1][1] = -f / z
    Jv[1][2] = v / z

    Jv[2][0] = 0
    Jv[2][1] = 0
    Jv[2][2] = -(diameter * f) / (z * z)

    return Jv

low_blue = np.array([94, 80, 2])
high_blue = np.array([126, 255, 255])
def findBall_VS(frame):
    c1,c2,r1,found = 0,0,0,False
    binary = cv.inRange(cv.cvtColor(frame, cv.COLOR_BGR2HSV), low_blue, high_blue)
    contours, hier = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        found=True
        (c1, c2), r1 = cv.minEnclosingCircle(contours[0])
        for c in contours:
            (x1, y1), r = cv.minEnclosingCircle(c)
            if r>r1:
               (c1,c2)=(x1, y1)
               r1 = r

    return int(c1),int(c2),int(r1),found, binary
    
def depth_data_to_point(pt, fxypxy):
    pt[0] =  pt[2] * (pt[0] - fxypxy[2]) / fxypxy[0]
    pt[1] = -pt[2] * (pt[1] - fxypxy[3]) / fxypxy[1]
    pt[2] = -pt[2]
    return pt

def camera_to_world(pt, cameraFrame, fxfypxpy):
    """pt = np.array([u, v, distance])"""
    cam_rot = cameraFrame.getRotationMatrix()
    cam_trans = cameraFrame.getPosition()
    pt = depth_data_to_point(pt, fxfypxpy) @ cam_rot.T + cam_trans
    return pt

