import numpy as np
import cv2
import matplotlib.pyplot as plt

use_ground_truth = True
see_tracking = True
f = 0.895
f *= 360.
fx = fy = f
cx = 320.
cy = 180.

camera_matrix =  np.array([[fx, 0.0, cx],
                            [0.0, fx, cy],
                            [0.0, 0.0, 1.0]])

total_points_used=10  #Opencv points for camera calibration
X_center=10.9
Y_center=10.7
Z_center=43.4
worldPoints=np.array([[X_center,Y_center,Z_center],
                       [5.5,3.9,46.8],
                       [14.2,3.9,47.0],
                       [22.8,3.9,47.4],
                       [5.5,10.6,44.2],
                       [14.2,10.6,43.8],
                       [22.8,10.6,44.8],
                       [5.5,17.3,43],
                       [14.2,17.3,42.5],
                       [22.8,17.3,44.4]], dtype=np.float32)
imagePoints=np.array([[cx,cy],
                       [502,185],
                       [700,197],
                       [894,208],
                       [491,331],
                       [695,342],
                       [896,353],
                       [478,487],
                       [691,497],
                       [900,508]], dtype=np.float32)

for i in range(1,total_points_used):
    wX=worldPoints[i,0]-X_center
    wY=worldPoints[i,1]-Y_center
    wd=worldPoints[i,2]

    d1=np.sqrt(np.square(wX)+np.square(wY))
    wZ=np.sqrt(np.square(wd)-np.square(d1))
    worldPoints[i,2]=wZ

dist =  np.array([0.03, 0.001, 0.0, 0.0, 0.01])
w,h = 640,360
#camera_matrix_new, roi=cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w,h), 1, (w,h))
camera_matrix_new = camera_matrix
_, R, Translation=cv2.solvePnP(worldPoints,imagePoints,camera_matrix_new, dist)
R_mtx, jac=cv2.Rodrigues(R)
inverse_R_mtx = np.linalg.inv(R_mtx)
inverse_camera_matrix_new = np.linalg.inv(camera_matrix_new)
s = 0.1 # 1.0 #scaling

print('newcam_mtx: ',np.shape(camera_matrix_new))
print('Translation: ', np.shape(Translation))
print("R - rodrigues: ", np.shape(R_mtx))

def compute_XYZ(u, v): #from 2D pixels to 3D world
    uv_ = np.array([[u, v, 1]], dtype=np.float32).T
    suv_ = s * uv_
    xyz_ = inverse_camera_matrix_new.dot(suv_) - Translation
    XYZ = inverse_R_mtx.dot(xyz_)

    pred = XYZ.T[0]
    pred[0] -= 3.2  # 3.50314012
    pred[1] -= 5.4  # 6.11871545
    return pred

def Rotation(pts, angle, degrees=False):
    if degrees == True:
        theta = np.radians(angle)
    else:
        theta = angle
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    rot_pts = []
    for v in pts:
        v = np.array(v).transpose()
        v = R.dot(v)
        v = v.transpose()
        rot_pts.append(v)
    return rot_pts

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

#1) Detect features  using FAST algorithm -------------------------------------
#detector = cv2.FastFeatureDetector_create(threshold=10, nonmaxSuppression=True)
detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

# Optical Flow Algorithm parameters----------------------------------------------
lk_params = dict(winSize=(21, 21), criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))
#lk_params = dict(winSize=(5, 5),criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))

#goodFeaturesToTrack to track-----------------------------------------------------
#feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
feature_params = dict( maxCorners = 100,qualityLevel = 0.3, minDistance = 1,blockSize = 1 )

#---------------------------------------------------------------------------------------------
distortion =  np.array([0.3, 0.001, 0.0, 0.0, 0.01])
distortion = -np.array([0.3, 0.001, 0.0, 0.0, 0.01])
distortion =  np.array([0.03, 0.001, 0.0, 0.0, 0.01])

grid_size, square_size = [20, 20], 0.2
object_points = np.zeros([grid_size[0] * grid_size[1], 3])
mx, my = [(grid_size[0] - 1) * square_size / 2, (grid_size[1] - 1) * square_size / 2]
for i in range(grid_size[0]):
    for j in range(grid_size[1]):
        object_points[i * grid_size[0] + j] = [i * square_size - mx, j * square_size - my, 0]

f, p = [5e-3, 120e-8]
intrinsic_cam_matrix = np.array([[f/p, 0, 0], [0, f/p, 0], [0, 0, 1]])
rvec = np.array([0.0, 0.0, 0.0])
tvec = np.array([0.0, 0.0, 3.0])

img_points, J = cv2.projectPoints(object_points, rvec, tvec, intrinsic_cam_matrix, distortion)
plt.scatter(*zip(*img_points[:, 0, :]), marker='.', c='r')
plt.axis('equal')
plt.grid()
plt.show()











