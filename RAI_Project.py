import sys

sys.path.append('../build')  # also here change
sys.path.append('/home/eugen/git/robotics-course/build')

import cv2 as cv
import numpy as np
import libry as ry
import time
from IPython.display import display, clear_output
import ProjectPerception as perception
from collections import deque
import matplotlib.pyplot as plt
import math

attractive_gain = 4.0
repulsive_gain = 250.0
GRID_WIDTH_X = 70.0  # m
GIRD_WIDTH_Y = 53.0  # m


def Potential_Field(gx, gy, ox, oy, resolution, robot_radius):
    minx = min(ox) - GRID_WIDTH_X / 2.0
    miny = min(oy) - GIRD_WIDTH_Y / 2.0
    maxx = max(ox) + GRID_WIDTH_X / 2.0
    maxy = max(oy) + GIRD_WIDTH_Y / 2.0
    xw = int(round((maxx - minx) / resolution))
    yw = int(round((maxy - miny) / resolution))

    potential_map = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * resolution + minx
        for iy in range(yw):
            y = iy * resolution + miny
            U_attractive_goal = Attractive_potential(x, y, gx, gy)
            U_repulsive_obstacle = Repulsive_potential(x, y, ox, oy, robot_radius)
            U_final = U_attractive_goal + U_repulsive_obstacle
            potential_map[ix][iy] = U_final

    return potential_map, minx, miny


def Attractive_potential(x, y, gx, gy):
    return 0.5 * attractive_gain * np.hypot(x - gx, y - gy)  # sqrt(x1**2 + x2**2) = np.hypot


def Repulsive_potential(x, y, ox, oy, robot_radius):
    id_min = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):  # get closest obstacle
        distance_to_obstacle = np.hypot(x - ox[i], y - oy[i])
        if dmin >= distance_to_obstacle:
            dmin = distance_to_obstacle
            id_min = i

    dq = np.hypot(x - ox[id_min], y - oy[id_min])
    if dq <= robot_radius:
        if dq <= 0.1:
            dq = 0.1
        return 0.5 * repulsive_gain * (1.0 / dq - 1.0 / robot_radius) ** 2
    else:
        return 0.0


def Motion_Model():
    model = [[1, 0],
             [0, 1],
             [-1, 0],
             [0, -1],
             [-1, -1],
             [-1, 1],
             [1, -1],
             [1, 1]]
    return model

def potential_field_planning(sx, sy, gx, gy, ox, oy, resolution, robot_radius):
    status = True
    potential_map, minx, miny = Potential_Field(gx, gy, ox, oy, resolution, robot_radius)

    distance = np.hypot(sx - gx, sy - gy)  # distance from start to goal
    ix = round((sx - minx) / resolution)
    iy = round((sy - miny) / resolution)
    gix = round((gx - minx) / resolution)
    giy = round((gy - miny) / resolution)

    draw_heatmap(potential_map)
    plt.scatter(ix, iy, c='r', s=100, marker='*')
    plt.scatter(gix, giy, c="g", s=100, marker='*')
    plt.show()

    rx, ry = [sx], [sy]
    motion = Motion_Model()
    ix_, iy_ = [], []
    while distance >= resolution:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(potential_map) or iny >= len(potential_map[0]):
                p = float("inf")  # outside
            else:
                p = potential_map[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * resolution + minx
        yp = iy * resolution + miny
        distance = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)
        ix_.append(ix)
        iy_.append(iy)

        if len(ix_) > 50:
            status = False
            break

    draw_heatmap(potential_map)
    plt.plot(ix, iy, "*k")
    plt.plot(gix, giy, "*m")
    plt.plot(ix_, iy_, ".r")
    plt.scatter(gix, giy, c="g", s=100, marker='*')
    plt.show()

    return np.array([rx, ry]), status

def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def callPotentialPath(sx, sy, gx, gy, grid_size, robot_radius, ox, oy):
    return potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

def Word2Grid(p):
    point = (-10. * p[0] + 35., -10. * p[1] + 26.5)
    return point

def Grid2World(path):
    s = [((-p[0] + 35.) / 10., -(p[1] - 26.5) / 10.) for p in path.T]
    return s

use_Referee = True
if use_Referee:
    scene = "../scenarios/MyScenario2.g"
else:
    scene = "../scenarios/MyScenario.g"

pts = deque(maxlen=5)
ptsRef = deque(maxlen=5)


class PlayGame:
    def __init__(self, perception='cheat', use_VS=False):
        self.k = 0
        self.tau = 0.01
        self.focal = 0.895 * 360.
        self.diameter_real = .1 * 360.
        self.use_VS = use_VS
        self.Z = (self.focal * (self.diameter_real / 2.0))  # circle depth
        self.fxfypxpy = [self.focal, self.focal, 320., 180.]

        self.cheat = perception == 'cheat'

        self.setUpSimulation()
        self.setUpC()
        self.StartScene()

        self.ox = [23., 23., 24., 25.0, 26.0, 27.0, 43.0, 44.0, 45.0, 46.0, 26., 42.]
        self.oy = [22., 23., 26.5, 22.5, 26.5, 23.5, 26.5, 23.5, 26.5, 22., 21., 21.]

        self.tau_grab_tools = 0.05

        self.timeoutL = 0
        self.timeoutR = 0

    def setUpSimulation(self):
        self.RealWorld = ry.Config()
        self.RealWorld.addFile(scene)
        self.RealWorld.getFrame("target").setContact(1)
        self.RealWorld.getFrame("ring2").setContact(1)
        self.RealWorld.getFrame("ring_2").setContact(1)
        self.RealWorld.getFrame("tool_left").setContact(1)
        self.RealWorld.getFrame("tool_right").setContact(1)
        self.RealWorld.getFrame("left_wall").setContact(1)
        self.RealWorld.getFrame("right_wall").setContact(1)

        self.base = self.RealWorld.addFrame("base")
        self.base.setPosition([0., 0, 0])
        d1 = .005
        self.base.setShape(ry.ST.ssBox, size=[d1, d1, d1, d1])
        self.base.setColor([0, 1, 0])

        self.S = self.RealWorld.simulation(ry.SimulatorEngine.physx, True)
        # self.S.addImp(ry.ImpType.objectImpulses, ['target'], [])

        f = self.RealWorld.addFrame("Ref_gripperCamera", "Ref_gripper")
        f.setRelativePosition([0, 0, 0])
        f.setShape(ry.ST.ssBox, [.02, .02, .02, .02])
        f.setColor([0., 0., 1.])
        self.S.addSensor("camera")

        self.S.addSensor("Refcamera", "Ref_gripperCamera", 640, 360, 1.)
        self.S.selectSensor("camera")

        self.leftScore = 0
        self.rightScore= 0

        self.position_threshold = 0.1#  0.05 # 9e-2 #distance from the tool to the ball 0.09
        self.table_region = .15  # .1  #robot worspace

    def mainCamera(self):
        self.k += 1
        if self.cheat == False:
            if self.k % 5 == 0: #2
                if self.use_VS:
                    self.S.selectSensor("camera")
                [rgb1, depth1] = self.S.getImageAndDepth()
                #self.cameraFrame.setPointCloud(self.S.depthData2pointCloud(depth1, self.fxfypxpy), rgb1)
                xCenter, yCenter, r1, found, binary = perception.findBall_VS(rgb1)
                if found:
                    cv.circle(rgb1, (xCenter, yCenter), r1, (0, 255, 0), 3)
                    u, v, z = xCenter, yCenter, depth1[int(yCenter), int(xCenter)]
                    pt = np.array([u, v, z])
                    world_position = perception.camera_to_world(pt, self.cameraFrame, self.fxfypxpy)
                    self.ball.setPosition(world_position)
                    pts.appendleft((xCenter, yCenter))
                    for i in range(1, len(pts)):
                        if pts[i - 1] is None or pts[i] is None:
                            continue
                        thickness = int(np.sqrt(6 / float(i + 1)) * 2.5)
                        if thickness > 0:
                            cv.line(rgb1, pts[i - 1], pts[i], (0, 0, 255), thickness)
                else:
                    # cannot find the ball -> ball is outside the table
                    # here the ball is out, and we send trigger the referee to go after the ball
                    self.ball.setPosition(self.RealWorld.getFrame("target").getPosition())

                if len(rgb1) > 0:
                    font = cv.FONT_HERSHEY_SIMPLEX
                    org = (270, 30)
                    fontScale = 1
                    color = (0, 255, 0)
                    thickness = 4
                    image = cv.putText(rgb1, '{} - {}'.format(self.rightScore, self.leftScore), org, font,
                                        fontScale, color, thickness, cv.LINE_AA)
                    cv.imshow('Main camera', image)
                self.S.step([], 0.01, ry.ControlMode.none)
                rv = cv.waitKey(33)
                if rv == ord('q'):
                    print('Stop')
                    self.kill()

    def refereeCamera(self, desired_radius=30.0, move = False, compute_position=False):
        world_position = None
        if self.use_VS:
            if self.k % 5 == 0:  # 10
                self.S.selectSensor("Refcamera")
                [rgb2, depth2] = self.S.getImageAndDepth()
                c1, c2, r1, found, binary = perception.findBall_VS(rgb2)

                if found:
                    cv.circle(rgb2, (c1, c2), r1, (0, 255, 0), 3)
                    z = self.Z / r1
                    xy_target = np.array([c1, c2, r1])
                    xy_desired = np.array([640. / 2, 360. / 2, desired_radius])
                    err = xy_target - xy_desired
                    k = 0.03
                    Jv = perception.getImageJacobian(u=xy_target[0], v=xy_target[1], z=z, f=self.focal,
                                                     diameter=self.diameter_real)
                    vel_J = np.dot(np.linalg.inv(Jv), err)
                    [_, J2] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
                    y2 = np.array([k * vel_J[0], -k * vel_J[2], -k * vel_J[1]])

                    [y3, J3] = self.RealWorld.evalFeature(ry.FS.quaternion, ["Ref_gripperCenter"])
                    y3 = np.array([y3 - self.refereeOrientation])

                    if move:
                        y = np.block([y2, y3])
                        J = np.block([[J2], [J3]])
                    else:
                        [y1, J1] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
                        y1 = y1 - [0., -1.5, 0.1]
                        y = np.block([y1, y2, y3])
                        J = np.block([[J1], [J2], [J3]])

                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[1])) @ -y.T;
                    self.S.step(vel, 0.045, ry.ControlMode.velocity)

                    ptsRef.appendleft((c1, c2))
                    for i in range(1, len(ptsRef)):
                        if ptsRef[i - 1] is None or ptsRef[i] is None:
                            continue
                        thickness = int(np.sqrt(3 / float(i + 1)) * 2.5)
                        if thickness > 0:
                            cv.line(rgb2, ptsRef[i - 1], ptsRef[i], (0, 0, 255), thickness)

                    if compute_position:
                        u, v, z = c1, c2, depth2[int(c2), int(c1)]
                        pt = np.array([u, v, z])
                        world_position = perception.camera_to_world(pt, self.RealWorld.frame("Ref_gripperCamera"), self.fxfypxpy)

                if len(rgb2) > 0: cv.imshow('OPENCV - Refcamera', rgb2)
            rv = cv.waitKey(33)
            if rv == ord('q'):
                print('Stop')
                self.kill()

        return world_position

    def setUpC(self):
        self.C = ry.Config()
        self.C.addFile(scene)
        self.V = ry.ConfigurationViewer()
        self.C.getFrame("tool_left").setContact(1)
        self.C.getFrame("tool_right").setContact(1)
        self.C.getFrame("left_wall").setContact(1)
        self.C.getFrame("right_wall").setContact(1)
        self.ball = self.C.addFrame("ball")
        self.ball.setShape(ry.ST.sphere, [.1])
        self.ball.setContact(1)
        self.V.setConfiguration(self.C)

        self.cameraFrame = self.C.frame("camera")
        self.C.delFrame('target')

    def StartScene(self):
        if use_Referee:
            self.S.openGripper("Ref_gripper")
        for t in range(200):
            self.mainCamera()
            time.sleep(0.001)
            clear_output(wait=True)
            display(self.S.getGripperWidth('Ref_gripper'))
            if self.cheat:
                self.ball.setPosition(self.RealWorld.getFrame("target").getPosition())
            [y, J] = self.C.evalFeature(ry.FS.scalarProductXX, ["Ref_gripperCenter", "base"])
            y = y - [1.]
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, self.tau, ry.ControlMode.velocity)
            #self.V.recopyMeshes(self.C)
            self.V.setConfiguration(self.C)
        [self.refereeOrientation, _] = self.RealWorld.evalFeature(ry.FS.quaternion, ["Ref_gripperCenter"])

    def kill(self):
        cv.destroyAllWindows()
        self.S = 0
        self.V = 0
        self.C = 0
        self.RealWorld = 0
        print('-KILLED-')

    def getTool_new(self, arm):
        self.V.recopyMeshes(self.C)
        if arm == "L_gripper":
            while True:  #align above
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["L_gripperCenter"])
                y1 = y1 - [0, 0, 1]
                [y2, J2] = self.C.evalFeature(ry.FS.positionDiff, ["L_gripperCenter", "ring2"])
                y2[-1] -= 0.1
                [y3, J3] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripperCenter", "ring2"])
                y3 = y3 - [1]
                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)
                if np.linalg.norm(y2) < 1e-2:
                    break

            [y2, _] = self.C.evalFeature(ry.FS.positionDiff, ["L_gripperCenter", "ring2"])
            while np.linalg.norm(y2) > 2e-3:  # go to position
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10==0:
                    self.V.setConfiguration(self.C)
                [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["L_gripperCenter"])
                y1 = y1 - [0, 0, 1]
                [y2, J2] = self.C.evalFeature(ry.FS.positionDiff, ["L_gripperCenter", "ring2"])
                [y3, J3] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripperCenter", "ring2"])
                y3 = y3 - [1]
                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)
            self.S.closeGripper('L_gripper')
            while True:
                time.sleep(0.01)
                self.S.step([], self.tau_grab_tools, ry.ControlMode.none)
                if self.k % 10==0:
                    self.V.setConfiguration(self.C)
                if self.S.getGripperIsGrasping('L_gripper'):
                    print('Left robot got the tool')
                    break
                if self.S.getGripperWidth('L_gripper') < -0.05:
                    print("FAILED")
                    break
        elif arm == "R_gripper":
            while True > 2e-3:  #align above
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["R_gripperCenter"])
                y1 = y1 - [0, 0, 1]
                [y2, J2] = self.C.evalFeature(ry.FS.positionDiff, ["R_gripperCenter", "ring_2"])
                y2[-1] -= 0.1
                [y3, J3] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripperCenter", "ring_2"])
                y3 = y3 - [1]
                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)
                if np.linalg.norm(y2) < 1e-2:
                    break

            [y2, _] = self.C.evalFeature(ry.FS.positionDiff, ["R_gripperCenter", "ring_2"])
            while np.linalg.norm(y2) > 2e-3:  # go to position
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["R_gripperCenter"])
                y1 = y1 - [0, 0, 1]
                [y2, J2] = self.C.evalFeature(ry.FS.positionDiff, ["R_gripperCenter", "ring_2"])
                [y3, J3] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripperCenter", "ring_2"])
                y3 = y3 - [1]
                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)
            self.S.closeGripper('R_gripper')
            while True:
                time.sleep(0.01)
                self.S.step([], self.tau_grab_tools, ry.ControlMode.none)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                if self.S.getGripperIsGrasping('R_gripper'):
                    print('Right robot got the tool')
                    break
                if self.S.getGripperWidth('L_gripper') < -0.05:
                    print("FAILED")
                    break

    def GoToReady_new(self, arm):
        if arm == "L_gripper":
            [y3, _] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
            y3 = y3 - [-0.7, .05, 0.4]  # predefined ready position
            while np.linalg.norm(y3) > 2e-3:
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)  # set your robot model to match the real q
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)

                [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
                y1 = y1 - [-1]
                [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["L_gripper", "base"])
                y2 = y2 - [-1]

                [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
                y3 = y3 - [-0.7, .05, 0.4]  # predefined ready position

                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)
        elif arm == "R_gripper":
            [y3, _] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
            y3 = y3 - [0.7, -.05, 0.4]  # predefined ready position
            while np.linalg.norm(y3) > 2e-3:
                self.mainCamera()
                self.refereeCamera()
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)  # set your robot model to match the real q
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)

                [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
                y1 = y1 - [1]
                [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["R_gripper", "base"])
                y2 = y2 - [1]

                [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
                y3 = y3 - [0.7, -.05, 0.4]  # predefined ready position

                y = np.block([y1, y2, y3])
                J = np.block([[J1], [J2], [J3]])
                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                self.S.step(vel, self.tau_grab_tools, ry.ControlMode.velocity)

    def Both_Arms_Same_Thread(self):
        tried = False
        leftWin = False
        ball_center = 0
        self.k=4 #required to reset the ball position
        self.mainCamera()
        while True:
            self.mainCamera()
            self.refereeCamera()
            if self.cheat:
                p = self.RealWorld.getFrame('target').getPosition()
            else:
                p = self.ball.getPosition()

            if p[0] < -self.table_region:  # -0.17:  # ball is in the area of the left robot
                self.timeoutL+=1
                self.timeoutR=0
                ball_center, tried = 0, False
                if p[0] < -1.1 or p[1] < -.55 or p[1] > .55:  # ball is ouside of the table
                    display('Right Robot win! ======================================= ')
                    leftWin = False
                    break
                self.tau = 0.18# 0.2
                #[leftR, _] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
                #target = [p[0], p[1], leftR[2] - 0.01]
                target = [p[0], p[1], 0.4]

                self.C.setJointState(self.S.get_q())
                [y3, _] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
                y3 = y3 - target
                if np.linalg.norm(y3) > self.position_threshold:  # go to position =======================
                    print('==============================Left robot go to position==================================')
                    time.sleep(0.01)
                    q = self.S.get_q()
                    self.C.setJointState(q)
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["L_gripper", "base"])
                    y1 = y1 - [.5]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
                    y2 = y2 - [-1]
                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
                    y3 = y3 - target

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)
                else:  # Hit the ball===================================================
                    print('==============================Left robot hit==================================')
                    self.tau = 0.05
                    [y1, _] = self.C.evalFeature(ry.FS.scalarProductXZ, ["L_gripper", "base"])
                    y1 = y1 - [-1.]
                    if np.abs(y1) - 1 > 1e-4:
                        time.sleep(0.01)
                        q = self.S.get_q()
                        self.C.setJointState(q)
                        if self.k % 10 == 0:
                            self.V.setConfiguration(self.C)

                        [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["L_gripper", "base"])
                        y1 = y1 - [-1.]
                        [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
                        y2 = y2 - [-1]
                        [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
                        y3 = y3 - target

                        y = np.block([y1, y2, y3])
                        J = np.block([[J1], [J2], [J3]])
                        vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                        self.S.step(vel, np.random.uniform(0.05, 0.075), ry.ControlMode.velocity)

                # right robot go ready----------------------------------------------
                self.tau = 0.1
                [y3, _] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
                y3 = y3 - [0.7, p[1], 0.4]  # [0.7, -.05, 0.4]
                if np.linalg.norm(y3) > 2e-3:
                    time.sleep(0.005)
                    q = self.S.get_q()
                    self.C.setJointState(q)  # set your robot model to match the real q
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
                    y1 = y1 - [1]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["R_gripper", "base"])
                    y2 = y2 - [1]

                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
                    y3 = y3 - [0.7, p[1], 0.4]  # [0.7, -.05, 0.4]

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)

                if self.timeoutL > 50:
                    leftWin = False
                    break

            elif p[0] > self.table_region:  # 0.17:  # the ball is in the area of the right robot
                self.timeoutL = 0
                self.timeoutR +=1
                ball_center, tried = 0, False
                if p[0] > 1.1 or p[1] < -.55 or p[1] > .55:  # the ball is outside of the table
                    display('Left Robot win! =============================================')
                    leftWin = True
                    break
                self.tau =  0.18#  0.2
                #[rightR, _] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
                #target = [p[0], p[1], rightR[2] - 0.01]
                target = [p[0], p[1], 0.4]

                self.C.setJointState(self.S.get_q())
                [y3, _] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
                y3 = y3 - target
                if np.linalg.norm(y3) > self.position_threshold:  # go to position============================
                    print('==============================Right robot go to position==================================')
                    time.sleep(0.01)
                    q = self.S.get_q()
                    self.C.setJointState(q)
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["R_gripper", "base"])
                    y1 = y1 - [.5]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
                    y2 = y2 - [1]
                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
                    y3 = y3 - target

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)

                else:  # hit the ball ==================================================
                    print('==============================Right robot hit==================================')
                    self.tau = 0.05
                    [y1, _] = self.C.evalFeature(ry.FS.scalarProductXZ, ["R_gripper", "base"])
                    y1 = y1 - [-1.]
                    if np.abs(y1) - 1 > 1e-4:
                        time.sleep(0.01)
                        q = self.S.get_q()
                        self.C.setJointState(q)
                        if self.k % 10 == 0:
                            self.V.setConfiguration(self.C)

                        [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["R_gripper", "base"])
                        y1 = y1 - [-1.]
                        [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
                        y2 = y2 - [1]
                        [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
                        y3 = y3 - target

                        y = np.block([y1, y2, y3])
                        J = np.block([[J1], [J2], [J3]])
                        vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                        self.S.step(vel, np.random.uniform(0.045, 0.075), ry.ControlMode.velocity)

                # left robot go ready----------------------------------------------
                self.tau = 0.1
                [y3, _] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
                y3 = y3 - [-0.7, p[1], 0.4]  # [-0.7, .05, 0.4]
                if np.linalg.norm(y3) > 2e-3:
                    time.sleep(0.005)
                    q = self.S.get_q()
                    self.C.setJointState(q)
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
                    y1 = y1 - [-1]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["L_gripper", "base"])
                    y2 = y2 - [-1]
                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
                    y3 = y3 - [-0.7, p[1], 0.4]  # [-0.7, .05, 0.4]

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)

                if self.timeoutR > 50:
                    leftWin = True
                    break

            else:
                ball_center += 1
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                self.S.step([], self.tau, ry.ControlMode.none)
                print('==============================Ball center {}=================================='.format(ball_center))
                if ball_center > 50:
                    if tried:
                        break
                    else:
                        if p[0] <= 0:
                            self.leftR_Hit()
                        else:
                            self.rightR_Hit()
                        tried = True

        if ball_center < 50:
            if leftWin:
                self.leftScore += 1
                self.takeFun('L')
            else:
                self.rightScore += 1
                self.takeFun('R')
        else:
            print('==============================Equality {}=================================='.format(ball_center))
        self.timeoutL = 0
        self.timeoutR = 0
        return leftWin

    def takeFun(self, arm):  # Robot Won
        t = np.linspace(0, 2 * np.pi, 800)
        i = 0
        while True:
            self.mainCamera()
            self.refereeCamera()
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)

            if arm == 'L':
                [y, J] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
            elif arm == 'R':
                [y, J] = self.C.evalFeature(ry.FS.position, ["R_gripper"])

            y[2] += 0.5
            y[1] = np.sin(t[i])
            y[0] = 0

            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ (y)
            self.S.step(vel, self.tau, ry.ControlMode.velocity)

            i = i + 1 if i < 798 else 0
            if arm == 'R':
                [y3, _] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
                y3 = y3 - [-0.7, .05, 0.4]
                if np.linalg.norm(y3) > 2e-3:
                    time.sleep(0.01)
                    q = self.S.get_q()
                    self.C.setJointState(q)
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
                    y1 = y1 - [-1]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["L_gripper", "base"])
                    y2 = y2 - [-1]
                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripperCenter"])
                    y3 = y3 - [-0.7, .05, 0.4]

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y;
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)
                else:
                    break
            elif arm == 'L':
                [y3, _] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
                y3 = y3 - [0.7, -.05, 0.4]
                if np.linalg.norm(y3) > 2e-3:
                    time.sleep(0.01)
                    q = self.S.get_q()
                    self.C.setJointState(q)
                    if self.k % 10 == 0:
                        self.V.setConfiguration(self.C)

                    [y1, J1] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
                    y1 = y1 - [1]
                    [y2, J2] = self.C.evalFeature(ry.FS.scalarProductXX, ["R_gripper", "base"])
                    y2 = y2 - [1]

                    [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripperCenter"])
                    y3 = y3 - [0.7, -.05, 0.4]

                    y = np.block([y1, y2, y3])
                    J = np.block([[J1], [J2], [J3]])
                    vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                    self.S.step(vel, self.tau, ry.ControlMode.velocity)
                else:
                    break

    def rightR_Hit(self):
        self.tau = 0.2
        [y1, _] = self.C.evalFeature(ry.FS.scalarProductXZ, ["R_gripper", "base"])
        y1 = y1 - [-.5]
        while np.abs(y1) > 1e-2:
            self.refereeCamera()
            self.mainCamera()
            if self.cheat:
                p = self.RealWorld.getFrame('target').getPosition()
            else:
                p = self.ball.getPosition()

            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["R_gripper", "base"])
            y1 = y1 - [-.5]
            [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["R_gripper", "base"])
            y2 = y2 - [1]
            [y3, J3] = self.C.evalFeature(ry.FS.position, ["R_gripper"])
            target = [p[0], p[1], 0.4]
            y3 = y3 - target

            y = np.block([y1, y2, y3])
            J = np.block([[J1], [J2], [J3]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, self.tau, ry.ControlMode.velocity)

    def leftR_Hit(self):
        self.tau = 0.2
        [y1, _] = self.C.evalFeature(ry.FS.scalarProductXZ, ["L_gripper", "base"])
        y1 = y1 - [-.5]
        while np.abs(y1) > 1e-2:
            self.refereeCamera()
            self.mainCamera()
            if self.cheat:
                p = self.RealWorld.getFrame('target').getPosition()
            else:
                p = self.ball.getPosition()
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y1, J1] = self.C.evalFeature(ry.FS.scalarProductXZ, ["L_gripper", "base"])
            y1 = y1 - [-.5]
            [y2, J2] = self.C.evalFeature(ry.FS.scalarProductYY, ["L_gripper", "base"])
            y2 = y2 - [-1]
            [y3, J3] = self.C.evalFeature(ry.FS.position, ["L_gripper"])
            target = [p[0], p[1], 0.4]
            y3 = y3 - target

            y = np.block([y1, y2, y3])
            J = np.block([[J1], [J2], [J3]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, self.tau, ry.ControlMode.velocity)

    def followPath(self, path, not_too_close=False, bring_back=True):
        n = len(path)
        for p in path:
            if not_too_close and n < 5:
                break
            n -= 1
            position = [p[0], p[1], .38]
            [y1, _] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
            y1 = y1 - position
            while np.linalg.norm(y1) > 0.01:
                time.sleep(0.01)
                q = self.S.get_q()
                self.C.setJointState(q)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)

                [y1, J1] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
                y1 = y1 - position

                [robot_Pos, J_base] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
                theta = math.atan2(p[1] - robot_Pos[1], p[0] - robot_Pos[0])
                desiredX = [np.cos(theta), np.sin(theta), 0]

                [vectorX, J_x] = self.C.evalFeature(ry.FS.vectorX, ["base_footprint"])
                vectorX = vectorX - desiredX
                if bring_back:
                    theta1 = math.atan2(robot_Pos[1]-p[1],robot_Pos[0]- p[0])
                    desired_base_position = [p[0] + 0.5*np.cos(theta1), p[1] +0.5*np.sin(theta1), robot_Pos[-1]]
                    base_position = robot_Pos - desired_base_position

                    y = np.block([y1, base_position, vectorX])
                    J = np.block([[J1], [J_base], [J_x]])
                else:
                    y = np.block([y1, vectorX])
                    J = np.block([[J1], [J_x]])

                vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
                self.S.step(vel, 0.25, ry.ControlMode.velocity)

    def Go_Get_The_Ball_Smart(self, pos):
        print('==============================Bring back the ball==================================')
        p = self.RealWorld.getFrame('target').getPosition()

        self.C.setJointState(self.S.get_q())
        [robot, _] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
        f = Word2Grid((p[0], p[1]))
        s = Word2Grid((robot[0], robot[1]))
        path, status = callPotentialPath(s[0], s[1], f[0], f[1], grid_size=1, robot_radius=40.0, ox=self.ox, oy=self.oy)
        path = Grid2World(path)
        self.followPath(path, not_too_close=True, bring_back=False)

        while True:  # align above===========================================
            # self.mainCamera()
            p = self.RealWorld.getFrame('target').getPosition()
            p[2] = 0.45
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y2, J2] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
            y2 = y2 - p
            [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["Ref_gripperCenter"])
            y1 = y1 - [0, 0, 1]

            y = np.block([y1, y2])
            J = np.block([[J1], [J2]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, 0.05, ry.ControlMode.velocity)
            if np.linalg.norm(y2) < 0.01:
                break

        while True:  # align for grasping========================
            # self.mainCamera()
            p = self.RealWorld.getFrame('target').getPosition()

            p[-1] += 0.05
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y2, J2] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
            y2 = y2 - p
            [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["Ref_gripperCenter"])
            y1 = y1 - [0, 0, 1]
            # prevent base_frame from moving
            [y3, J3] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
            y3 *= 0.0

            y = np.block([y1, y2, y3])
            J = np.block([[J1], [J2], [J3]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, 0.01, ry.ControlMode.velocity)

            if np.linalg.norm(y2) < 0.04:
                break

        self.S.closeGripper('Ref_gripper')
        while True:  # closing the gripper====================
            time.sleep(self.tau)
            self.S.step([], 0.01, ry.ControlMode.none)
            self.V.recopyMeshes(self.C)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)
            if self.S.getGripperIsGrasping('Ref_gripper'):
                print('Ref robot got the tool')
                break
            if self.S.getGripperWidth('L_gripper') < -0.05:
                print("FAILED-----------------")
                break

        p = self.RealWorld.getFrame('target').getPosition()
        s = Word2Grid((p[0], p[1]))
        f = Word2Grid((pos[0], pos[1]))
        path, status = callPotentialPath(s[0], s[1], f[0], f[1], grid_size=1, robot_radius=40.0, ox=self.ox, oy=self.oy)

        path = Grid2World(path)
        self.followPath(path)

        self.S.openGripper("Ref_gripper")  # left the ball
        self.V.recopyMeshes(self.C)
        for i in range(50):
            time.sleep(0.01)
            self.S.step([], 0.01, ry.ControlMode.none)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

        print('Done================================================')

    def Arms_go_to_Ready(self, firstTime=True):
        if firstTime:
            Game.GoToReady_new('L_gripper')
            Game.GoToReady_new('R_gripper')
            self.q0 = self.S.get_q()
        else:
            T = 40  # go to initial positions  (here I was lazy to use IK)
            self.C.setJointState(self.S.get_q())
            komo = self.C.komo_path(1., T, T * 0.01, True)
            komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [2e1], target=self.q0)
            komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
            komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
            komo.optimize()
            for t in range(T):
                self.mainCamera()
                self.C.setFrameState(komo.getConfiguration(t))
                q = self.C.getJointState()
                self.S.step(q, 0.01, ry.ControlMode.position)
                if self.k % 10 == 0:
                    self.V.setConfiguration(self.C)
                time.sleep(0.01)

    def test(self):
        #leftWin = Game.Both_Arms_Same_Thread()
        while True:
            world_position = Game.refereeCamera(desired_radius=50.0, move=True, compute_position=True)
            Game.k += 1
            if world_position is not None:
                [robot_hand, J_] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
                print('distance ',np.linalg.norm(robot_hand-world_position))
                if np.linalg.norm(robot_hand-world_position)<1.45:
                    break
        p = world_position
        p[2] = 0.45
        while True:  # align above===========================================
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y2, J2] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
            y2 = y2 - p
            [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["Ref_gripperCenter"])
            y1 = y1 - [0, 0, 1]

            y = np.block([y1, y2])
            J = np.block([[J1], [J2]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, 0.05, ry.ControlMode.velocity)
            if np.linalg.norm(y2) < 0.01:
                break

        p = world_position
        p[-1] += 0.05
        while True:  # align for grasping========================
            time.sleep(0.01)
            q = self.S.get_q()
            self.C.setJointState(q)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)

            [y2, J2] = self.C.evalFeature(ry.FS.position, ["Ref_gripperCenter"])
            y2 = y2 - p
            [y1, J1] = self.C.evalFeature(ry.FS.vectorZ, ["Ref_gripperCenter"])
            y1 = y1 - [0, 0, 1]
            # prevent base_frame from moving
            [y3, J3] = self.C.evalFeature(ry.FS.position, ["base_footprint"])
            y3 *= 0.0

            y = np.block([y1, y2, y3])
            J = np.block([[J1], [J2], [J3]])
            vel = J.T @ np.linalg.inv(J @ J.T + 1e-2 * np.eye(y.shape[0])) @ -y
            self.S.step(vel, 0.01, ry.ControlMode.velocity)

            if np.linalg.norm(y2) < 0.04:
                break

        self.S.closeGripper('Ref_gripper')
        while True:  # closing the gripper====================
            time.sleep(self.tau)
            self.S.step([], 0.01, ry.ControlMode.none)
            self.V.recopyMeshes(self.C)
            if self.k % 10 == 0:
                self.V.setConfiguration(self.C)
            if self.S.getGripperIsGrasping('Ref_gripper'):
                print('Ref robot got the tool')
                break
            if self.S.getGripperWidth('L_gripper') < -0.05:
                print("FAILED-----------------")
                break

        print('Done-------------------------------')
        time.sleep(5)


#Game = PlayGame(perception='cheat', use_VS=True)
Game = PlayGame(perception='opencv', use_VS=False)

Game.getTool_new('L_gripper')
Game.getTool_new('R_gripper')
Game.Arms_go_to_Ready()

i = 1
while True:
    leftWin = Game.Both_Arms_Same_Thread()
    if Game.use_VS==False:
        Game.Arms_go_to_Ready(firstTime=False)
    if leftWin:
        Game.Go_Get_The_Ball_Smart([-.3, -.07, 0.4])
    else:
        Game.Go_Get_The_Ball_Smart([.3, -.07, 0.4])
    Game.Arms_go_to_Ready(firstTime=False)

    if i >= 3:
        break
    print('Step ', i)
    i += 1

Game.kill()

