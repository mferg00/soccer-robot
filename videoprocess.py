#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 15:03:27 2019

@author: pi
"""

import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

FONT = cv2.FONT_HERSHEY_COMPLEX

FRAME_X = 320
FRAME_Y = 240
CENTER_X = FRAME_X / 2
CENTER_Y = FRAME_Y / 2

# not confirmed
BALL_WIDTH = 42.67 #42.67 is min

FULL_RESOLUTION = "3280 * 2464"
FOCAL_LENGTH = 3.04
SENSOR_DIAG_MM = 4
SENSOR_WIDTH_MM = 3.2
SENSOR_HEIGHT_MM = 2.4
PIXEL_SIZE = 0.00112
FOV_HORIZONTAL = 62.2
FOV_VERTICAL = 48.8

GOAL_HEIGHT = 225
GOAL_WIDTH = 700

WALL_HEIGHT = 450

OBSTACLE_WIDTH = 180
OBSTACLE_HEIGHT = 225
OBSTACLE_RADIUS = 85
MAX_POSSIBLE_DISTANCE = 300

# hsv = np.zeros((640, 480), np.uint8)
KERNEL = np.ones((3, 3), np.uint8)


MIN_OBSTACLE_AREA = 750
MAX_OBSTACLE_AREA = 9999999
MIN_WALL_AREA = 3000
MAX_WALL_AREA = 9999999
MIN_GOAL_AREA = 1000
MAX_GOAL_AREA = 300000
BALL_TRAIL = 16
MIN_BALL_RAD = 4

RGB = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 128, 255),
    "black": (0, 0, 0),
    "white": (255, 255, 255),
    "lblue": (255, 200, 50)
}

# ball
ballHLow = 175
ballHHigh = 15
ballS = 150
ballSThresh = 100
ballV = 155
ballVThresh = 90

ballLower = np.array([ballHLow, ballS - ballSThresh, ballV - ballVThresh])
ballLowerEnd = np.array([180, ballS + ballSThresh, ballV + ballVThresh])
ballUpperStart = np.array([0, ballS - ballSThresh, ballV - ballVThresh])
ballUpper = np.array([ballHHigh, ballS + ballSThresh, ballV + ballVThresh])

# obstacles
# 80-180 (40-90), 6-33% (15-?), 3-5% (14)
obstacleH = 127
obstacleHThresh = 127
obstacleS = 127
obstacleSThresh = 127
obstacleV = 25
obstacleVThresh = 25

obstacleLower = np.array([obstacleH - obstacleHThresh, obstacleS - obstacleSThresh, obstacleV - obstacleVThresh])
obstacleUpper = np.array([obstacleH + obstacleHThresh, obstacleS + obstacleSThresh, obstacleV + obstacleVThresh])

# yellow goal
# HSL bright: 41, 54%, 47%, HSL dark: 39, 63%, 28%: avg: 40, 60%, 40%
# (20, 150, 100)
yellowH = 20
yellowHThresh = 10
yellowS = 200
yellowSThresh = 55
yellowV = 165
yellowVThresh = 60

yellowLower = np.array([yellowH - yellowHThresh, yellowS - yellowSThresh, yellowV - yellowVThresh])
yellowUpper = np.array([yellowH + yellowHThresh, yellowS + yellowSThresh, yellowV + yellowVThresh])

# blue goal
# 202 (101), 40% (144), 30% (108)
blueH = 100
blueHThresh = 40
blueS = 127
blueSThresh = 100
blueV = 127
blueVThresh = 75

blueLower = np.array([blueH - blueHThresh, blueS - blueSThresh, blueV - blueVThresh])
blueUpper = np.array([blueH + blueHThresh, blueS + blueSThresh, blueV + blueVThresh])

# floor
# HSL: 97, 27%, 57%, HSL: 150, 42%, 17%
# 60, 76, 56
floorH = 60
floorHThresh = 15
floorS = 110
floorSThresh = 60
floorV = 100
floorVThresh = 60

floorLower = np.array([floorH - floorHThresh, floorS - floorSThresh, floorV - floorVThresh])
floorUpper = np.array([floorH + floorHThresh, floorS + floorSThresh, floorV + floorVThresh])

# walls h 260 (130) s 0-10% = 25
wallH = 127
wallHThresh = 127
wallS = 127
wallSThresh = 127
wallV = 240
wallVThresh = 15

wallLower = np.array([wallH - wallHThresh, wallS - wallSThresh, wallV - wallVThresh])
wallUpper = np.array([wallH + wallHThresh, wallS + wallSThresh, wallV + wallVThresh])


def objectAngle(objectX):
        # returns in degrees
    return (objectX - CENTER_X) * (FOV_HORIZONTAL / FRAME_X)


def relativeDistWidth(width, pwidth):
    # returns in cm
    return 0.1 * ((FOCAL_LENGTH * width * FRAME_X) / (pwidth * SENSOR_WIDTH_MM))


def relativeDistHeight(height, pheight):
    # returns in cm
    return 0.1 * ((FOCAL_LENGTH * height * FRAME_Y) / (pheight * SENSOR_HEIGHT_MM))


def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/ (mdev if mdev else 1.)
    return data[s<m]


class VideoProcess:

    def __init__(self, debug=True, goal='none', ball=True, obstacles=True, wall=True):

        self.goaltarget = 'none'
            
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)   
        GPIO.output(23, GPIO.HIGH)

        self.findball = ball

        self.findobs = obstacles
        
        self.findwall = wall

        self.obs1 = [None, None]
        self.obs2 = [None, None]
        self.obs3 = [None, None]

        self.goal = [None, None]

        self.ball = [None, None]
        
        self.wall = [None, None]

        self.debug = debug
        self.framecopy = np.empty([640, 480, 3], np.uint8)
        self.hsv = np.empty([640, 480, 3], np.uint8)

        self.ball_smooth = np.array([-1, -1, -1], np.uint8)
        self.blue_smooth = np.array([-1, -1, -1], np.uint8)

        if(self.debug):
            self.frames_this_time = 0
            self.last_fps = 0
            self.last_time = time.time()
    
    
    def change_goal(self):
        if(self.goaltarget == 'blue'):
            self.goaltarget = 'yellow'
        else:
            self.goaltarget = 'blue'
                    
    def update(self, frame):
        
        switch_in = GPIO.input(24)
        
        if switch_in == 1:
            self.goaltarget = 'yellow'
            
        else:
            self.goaltarget = 'blue'

        if frame is not None:

            if self.debug:
                self.fps()
                self.framecopy = frame.copy()
                cv2.putText(self.framecopy, "fps: %d" % self.last_fps, (15, 15), FONT, 0.5, RGB["red"], 1, cv2.LINE_AA)

            ##cv2.imshow("orignal frame", frame)
            blurred = cv2.GaussianBlur(frame, (3, 3), 0)
            ##cv2.imshow("blurred frame", blurred)
            self.hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            ##cv2.imshow("hsv frame", self.hsv)

            if self.findball:
                self.ball = self.findBall()

            if self.findobs:
                obs = self.findMaxObstacle()

            if self.goaltarget == 'yellow':
                self.goal = self.findYellow()

            elif self.goaltarget == 'blue':
                self.goal = self.findBlue()
            
            if self.findwall:
                self.wall = self.findWall()

            return obs, self.ball, self.goal, self.wall
    

    def fps(self):

        self.frames_this_time += 1
        now = time.time()

        if now - self.last_time >= 1:
            self.last_time = now
            self.last_fps = self.frames_this_time
            self.frames_this_time = 0

    def drawStats(self, cx, cy, bounding, _object, colour, area, dist, angle):

        if self.debug:
            cy -= 20
            padding = 0

            if bounding is not None:
                cv2.drawContours(self.framecopy, [bounding], 0, RGB[colour], 3)

            cv2.putText(self.framecopy, _object, (cx, cy), FONT, 0.5, RGB[colour], 1, cv2.LINE_AA)

            if _object == 'wall':
                txtcolour = RGB["black"]
            else:
                txtcolour = RGB["white"]
                
            if area is not None:
                padding += 20
                cv2.putText(self.framecopy, "area: %.1f px" % area, (cx, cy + padding), FONT, 0.5, txtcolour, 1, cv2.LINE_AA)

            if dist is not None:
                padding += 20
                cv2.putText(self.framecopy, "dist: %.2f cm" % dist, (cx, cy + padding), FONT, 0.5, txtcolour, 1, cv2.LINE_AA)

            if angle is not None:
                padding += 20
                cv2.putText(self.framecopy, "angle: %.2f deg" % angle, (cx, cy + padding), FONT, 0.5, txtcolour, 1, cv2.LINE_AA)

    def get(self):

        if self.debug:
            return self.framecopy

        else:
            return self.frame

    def findBall(self):

        ballMask0 = cv2.inRange(self.hsv, ballLower, ballLowerEnd)
        ballMask1 = cv2.inRange(self.hsv, ballUpperStart, ballUpper)
        ballMask = cv2.bitwise_or(ballMask0, ballMask1)

        ballMask = cv2.morphologyEx(ballMask, cv2.MORPH_OPEN, KERNEL)
        ballMask = cv2.dilate(ballMask, (1, 1), iterations=1)
        #cv2.imshow("ball mask", ballMask)
        _, contours, _ = cv2.findContours(ballMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # only proceed if at least one contour was found
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius < MIN_BALL_RAD:
                return None, None
            M = cv2.moments(c)
            distCheck = relativeDistWidth(BALL_WIDTH, (2 * radius))
            self.ball_smooth = np.roll(self.ball_smooth, -1)
            self.ball_smooth[2] = distCheck
            if self.ball_smooth[0] == -1:
                return None, None
            outliers = reject_outliers(self.ball_smooth, m=5.)
            dist = np.mean(outliers)
            # print(dist)

            if M["m00"] > 0 and dist < 300:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                angle = objectAngle(cx)

                if self.debug:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(self.framecopy, (int(x), int(y)), int(radius),
                               RGB["orange"], 1)
                    # cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    self.drawStats(cx + 10, cy, None, "ball", "orange", None, dist, angle)

                return dist, angle

        return None, None
        ##########

    def findYellow(self):

        yellowMask = cv2.inRange(self.hsv, yellowLower, yellowUpper)
        #cv2.imshow("yellow goal mask", yellowMask)

        modified = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, KERNEL)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if MIN_GOAL_AREA < M["m00"] < MAX_GOAL_AREA:
                rect = cv2.minAreaRect(c)
                (_, _), (_, _), angle = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                height_1 = box[0][1] - box[3][1]
                height_2 = box[1][1] - box[2][1]
                if(height_1 != 0 and height_2 != 0):
                    self.goal_height = (height_1 + height_2) / 2
                dist = relativeDistHeight(GOAL_HEIGHT, self.goal_height)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                if cy < 200:
                    angle = objectAngle(cx)
    
                    if self.debug:
                        self.drawStats(cx, cy, box, "yellow goal", "yellow", None, dist, angle)
    
                    return [dist, angle]
                return None, None
            return None, None
        return None, None

    goal_height = 1

    def findBlue(self):

        blueMask = cv2.inRange(self.hsv, blueLower, blueUpper)
        #cv2.imshow("blue goal mask", blueMask)

        modified = cv2.morphologyEx(blueMask, cv2.MORPH_CLOSE, KERNEL)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if MIN_GOAL_AREA < M["m00"] < MAX_GOAL_AREA:
                rect = cv2.minAreaRect(c)
                (_, _), (height, _), _ = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                height_1 = box[0][1] - box[3][1]
                height_2 = box[1][1] - box[2][1]
                if(height_1 != 0 and height_2 != 0):
                    self.goal_height = (height_1 + height_2) / 2
                dist = relativeDistHeight(GOAL_HEIGHT, self.goal_height)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                if cy < 200:
                    angle = objectAngle(cx)
    
                    if self.debug:
                        self.drawStats(cx, cy, box, "blue goal", "blue", None, dist, angle)
                        # #cv2.imshow("blue goal", blueMask)
                    return [dist, angle]
                return None, None
            return None, None
        return None, None

        #########

    def findObstacles(self):

        _obs = [[None for x in range(2)] for y in range(3)]
        obstacleMask = cv2.inRange(self.hsv, obstacleLower, obstacleUpper)
        #cv2.imshow("obstacle mask", obstacleMask)

        modified = cv2.morphologyEx(obstacleMask, cv2.MORPH_CLOSE, KERNEL)
        # #cv2.imshow("obstacles", obstacleMask)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        c = sorted(contours, key=cv2.contourArea, reverse=True)
        length = min(3, len(c))
        count = 0

        while count < 3:
            if count >= length:
                _obs[count][:] = [None, None]
            else:
                _obs[count][:] = self.findObstacle(c[count])
            count += 1

        return _obs[0][:], _obs[1][:], _obs[2][:]

    # do not base on largest area, as there are 1-3 obstacles
    # TODO: add overlapping detection

    def findObstacle(self, contour):
        
        M = cv2.moments(contour)
        if MIN_OBSTACLE_AREA < M["m00"] < MAX_OBSTACLE_AREA:
            rect = cv2.minAreaRect(contour)
            (_, _), (width, height), _ = rect
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            dist = relativeDistWidth(OBSTACLE_WIDTH, width)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
                
            if(cy < 200):
                angle = objectAngle(cx)
                if self.debug:
                    self.drawStats(cx, cy, box, "obstacle", "white", None, dist, angle)

                return [dist, angle]
            return [None, None]
        return [None, None]
    
    def findMaxObstacle(self):
        
        obstacleMask = cv2.inRange(self.hsv, obstacleLower, obstacleUpper)
        #cv2.imshow("obstacle mask", obstacleMask)

        modified = cv2.morphologyEx(obstacleMask, cv2.MORPH_CLOSE, KERNEL)
        # #cv2.imshow("obstacles", obstacleMask)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
       
            if MIN_OBSTACLE_AREA < M["m00"] < MAX_OBSTACLE_AREA:
                rect = cv2.minAreaRect(c)
                (_, _), (width, height), _ = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                dist = relativeDistWidth(OBSTACLE_WIDTH, width)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                    
                if(cy < 160):
                    angle = objectAngle(cx)
                    if self.debug:
                        self.drawStats(cx, cy, box, "obstacle", "white", None, dist, angle)
    
                    return [dist, angle]
                return [None, None]
            return [None, None]
        return [None, None]
        
    
    '''
    def findWalls(self):

        _wall = [[None for x in range(2)] for y in range(3)]
        wallMask = cv2.inRange(self.hsv, wallLower, wallUpper)
        # #cv2.imshow("wall", wallMask)
    
        modified = cv2.morphologyEx(wallMask, cv2.MORPH_CLOSE, KERNEL)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        c = sorted(contours, key=cv2.contourArea, reverse=True)
        length = min(3, len(c))
        count = 0
    
        while count < 3:
            if count >= length:
                _wall[count][:] = [None, None]
            else:
                _wall[count][:] = self.findWall(c[count])
            count += 1
    
        #return _wall[0][:], _wall[1][:], _wall[2][:]
        sort_dist = sorted(_wall, key=lambda x:( float('inf') if x[0] is None else x[0]))
        return sort_dist'''

    # do not base on largest area, as there are 1-3 obstacles
    # TODO: add overlapping detection
    
    def findWall(self):

        wallMask = cv2.inRange(self.hsv, wallLower, wallUpper)
        #cv2.imshow("wall mask", wallMask)

        modified = cv2.morphologyEx(wallMask, cv2.MORPH_OPEN, KERNEL)
        _, contours, _ = cv2.findContours(modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
    
            if MIN_WALL_AREA < M["m00"] < MAX_WALL_AREA:
                rect = cv2.minAreaRect(c)
                (_, _), (width, height), _ = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                dist = relativeDistHeight(WALL_HEIGHT, width)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                if cy < 160:
                    angle = objectAngle(cx)
                    if self.debug:
                        self.drawStats(cx, cy, box, "wall", "black", None, dist, angle)
    
                    return [dist, angle]
                return [None, None]
            return [None, None]
        return [None, None]


if __name__ == '__main__':

    from videostream import VideoStream

    vision = VideoProcess(debug=True, goal='yellow', ball=True, obstacles=True)
    stream = VideoStream().start()
    time.sleep(2)

    try:
        while True:

            frame = stream.read()

            if frame is None:
                break

            # (obs1, obs2, obs3), goal, ball
            (obs), (ball), (goal), (wall) = vision.update(frame)

            cv2.imshow("scene", vision.get())
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        stream.stop()

    finally:
        cv2.destroyAllWindows()
        stream.stop()
