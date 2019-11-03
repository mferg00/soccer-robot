import numpy as np
from drive import Drive
from kicker import Kicker
from dribbler import Dribbler
import time
import RPi.GPIO as GPIO

class Timer:
    def __init__(self):
        self.start_time = 0
        self.end = 0
        self.time_dif = 0
        self.lock = False

    def start(self):
        if self.lock == False:
            self.start_time = time.time()
            self.lock = True
            
    def update(self):
        self.time_dif = time.time() - self.start_time
        
    def get(self):
        return self.time_dif

    def clear(self):
        print("clear")
        self.lock = False
        self.start_time = 0
        self.end = 0


def cap_range(value):
    
    if value is None:
        return None
    
    if value < 5:
        value = 5
    elif value > 250:
        value = 250
    return value

def within_angle(angle, lim):
    if(-lim <= angle <= lim):
        return True
    return False

def rb_to_pwm(ran, bearing):
    
    def speed(dist):
        return 1 - ((251 - dist) * (0.75/250))
    if ran is None:
        return 0, 0
    
    ml = (50 + bearing) * speed(ran)
    mr = (50 + bearing) * speed(ran)

    return ml, mr

def avg(ml1, mr1, ml2, mr2):
    return ((ml2 - ml1) / 2), ((mr2 - mr1) / 2)

def turn(target_side, mins, maxs):
    if target_side == -1:
        return mins, maxs
    else:
        return maxs, mins

def spin(dir, speed):
    return dir * speed, -dir * speed

MIN_BALL_DIST = 9
MAX_BALL_ANGLE = 7

MIN_GOAL_DIST = 90
MAX_GOAL_ANGLE = 3

MIN_OBS_DIST = 40

class Navigation:

    def __init__(self):

        self.circle_obstacle = False
        self.goto_ball = False
        self.ball_captured = False
        self.goto_obstacle = False
        self.goto_goal = False
        self.ball_found = False
        self.goal_found = False
        self.goal_clear = False
        self.spin = False
        self.search_obs = False
        self.timer = Timer()

        self.target_side = -1
        self.ball_side = -1
        self.goal_side = -1
        self.wall_side = -1
        self.obs_side = -1
        self.speed_mod = 1

        self.no_target_count = 0
        self.circle_count = 0

        self.obs_r = None
        self.obs_h = None
        self.ball_r = None
        self.ball_h = None
        self.goal_r = None
        self.goal_h = None
        self.wall_r = None
        self.wall_h = None
        self.target_r = None
        self.target_h = None
        self.target_found = False

        self.drive_sys = Drive()
        self.dribbler = Dribbler()
        self.kicker = Kicker()

    obs_buffer = [None, None]
    obs_buf_count = 0
    def update(self, obs, ball, goal, wall):

        if obs[0] is None:
            self.obs_buf_count += 1
            
            if(self.obs_buf_count > 2):
                self.obs_buffer = [None, None]
                self.obs_r = None
                self.obs_h = None
                
            else:
                self.obs_r = cap_range(self.obs_buffer[0])
                self.obs_h = self.obs_buffer[1]
            
        else:
            self.obs_buf_count = 0
            self.obs_buffer = obs
        
        self.ball_r = cap_range(ball[0])
        self.ball_h = ball[1]
        self.goal_r = cap_range(goal[0])
        self.goal_h = goal[1]
        self.wall_r = cap_range(wall[0])
        self.wall_h = wall[1] 

        self.conditions()
        self.state()
        ml, mr, drib, kick = self.decide()

        self.drive_sys.drive(ml, mr)
        self.dribbler.start(drib)
        if kick:
            self.kicker.kick()
            self.goal_clear = False

        return kick

    def conditions(self):

        if self.ball_r is not None:
            self.ball_side = np.sign(self.ball_h)
            self.ball_found = True
            self.no_target_count = 0

            if(within_angle(self.ball_h, MAX_BALL_ANGLE) and self.ball_r < MIN_BALL_DIST):
                self.ball_captured = True
            else:
                self.ball_captured = False

        else:
            self.ball_found = False

        if self.goal_r is not None:
            self.goal_side = np.sign(self.goal_h)
            self.goal_found = True

            if(within_angle(self.goal_h, MAX_GOAL_ANGLE) and self.goal_r < MIN_GOAL_DIST and self.target_clear()):
                self.goal_clear = True
            else:
                self.goal_clear = False

        else:
            self.goal_found = False

        if self.ball_captured:
            self.target_r = self.goal_r
            self.target_h = self.goal_h
            self.target_side = self.goal_side
            if self.goal_found:
                self.target_found = True
            else:
                self.target_found = False

        else:
            self.target_r = self.ball_r
            self.target_h = self.ball_h
            self.target_side = self.ball_side
            if self.ball_found:
                self.target_found = True
            else:
                self.target_found = False
                
        if self.wall_r is not None:
            self.wall_side = np.sign(self.wall_h)
        
        if self.obs_h is not None:
            self.obs_side = np.sign(self.obs_h)            
            
    counter = 0
    def state(self):

        if not self.target_found:
            
            print(self.counter)
            
            if self.counter < 50 * self.speed_mod:
                print("looking for ball")
                self.counter += 1
                self.spin = True

            elif self.counter >= 50 * self.speed_mod:
                self.spin = False
                # spin until first obs found
            
                if not self.search_obs:
                    print(self.obs_r)
                    
                    if(self.obs_r is None):
                        self.spin = True
                        print("looking for obs")
                    else:
                        self.spin = False
                        if(self.obs_r > MIN_OBS_DIST):
                            print("going to obs")
                            self.goto_obstacle = True
                        else:
                            print("spinning to circle obs")
                            self.goto_obstacle = False
                            self.spin = True
                            self.target_side = -self.wall_side
                            self.search_obs = True  
        
                else:                    
                                    
                    if(self.counter >= 100 * self.speed_mod):
                        print("circling obs")
                        self.spin = False
                        self.counter += 1
                        self.circle_obstacle = True

                        if(self.counter >= 150 * self.speed_mod):
                            print("circle obs done, spin out")
                            self.counter += 1
                            self.circle_obstacle = False
                            self.spin = True

                            if(self.counter >= 200 * self.speed_mod):
                                self.counter = 0
                                print("done")
                                self.spin = False
                                self.circle_obstacle = False
                                self.search_obs = False                            

            # target seen
            else:
                print("going to target")
                self.counter = 0

            # if obstacle is too close
            if self.obs_r is not None and self.obs_r < 25:
                self.spin = True
                self.target_side = -self.obs_side


    def decide(self):

        ml = 0
        mr = 0
        drib = 0
        kick = False

        # most complex situations at the top, least complex at the bottom
        if self.circle_obstacle and self.target_found:
            ml1, mr1 = turn(self.target_side, 45, 90)
            ml2, mr2 = self.goto_target()

            if ml1 > mr1:
                ml, mr = min(ml2, ml1), max(mr2, mr1)
            else:
                ml, mr = max(ml2, ml1), min(mr2, mr1)

        elif self.target_found:
            ml, mr = self.goto_target()

        elif self.circle_obstacle:
            ml, mr = turn(self.target_side, 45, 90)

        elif self.goto_obstacle:
            ml, mr = rb_to_pwm(self.obs_r, self.obs_h)

        elif self.spin:
            ml, mr = spin(self.target_side, 40)

        # dribbler adn speed adjusts
        if self.ball_r is not None and self.ball_r < 25:
            drib = 80

            if self.ball_captured:
                if self.spin:
                    ml *= 0.5
                    mr *= 0.5
                    self.speed_mod = 2
                    drib = 100

                if self.circle_obstacle:
                    ml *= 0.65
                    mr *= 0.65
                    self.speed_mod = 1.5
                    drib = 85

                else:
                    ml *= 1
                    mr *= 1
                    self.speed_mod = 1
                    drib = 75

        else:
            drib = 0
            self.speed_mod = 1

        if self.goal_clear:
            drib = 0
            kick = True
            ml = 0
            mr = 0

        return ml, mr, drib, kick

    '''
    def target_clear_many(self):

        def clear(obs_h, obs_r, thresh):
            if self.target_r - 10 < obs_r:
                return True
            else:
                if (obs_h - thresh <= self.target_h <= obs_h + thresh):
                    return False
                return True

        if self.target_h is None:
            return False

        for i in range(3):
            if self.obs_h[i] is None:
                return True
            if not clear(self.obs_h[i], self.obs_r[i], 16):
                return False
            return True'''

    def target_clear(self):
        if self.obs_r is None:
            return True

        if self.target_r is None:
            return False

        if self.target_r - 20 < self.obs_r:
            return True
        else:
            if (self.obs_h - 15 <= self.target_h <= self.obs_h + 15):
                return False
            return True

    def target_between(self):

        closest_left_h = -60
        closest_left_r = 0
        closest_right_h = 60
        closest_right_r = 0
        ret = [None, None], [None, None]

        for i in range(3):
            if self.obs_h[i] is None:
                continue

            if closest_left_h < self.obs_h[i] < self.target_h:
                closest_left_h = self.obs_h[i]
                closest_left_r = self.obs_r[i]

            elif closest_right_h > self.obs_h[i] > self.target_h:
                closest_right_h = self.obs_h[i]
                closest_right_r = self.obs_r[i]

        if closest_left_h != -60:
            ret[0][:] = [closest_left_r, closest_left_h]

        if closest_right_h != 60:
            ret[1][:] = [closest_right_r, closest_right_h]

        return ret

    '''
    def goto_target(self):
          
        left_obs, right_obs = self.target_between()

        if left_obs[0] is None and right_obs[0] is None:
            #print("clear")
            return self.goto_target_simple()
            

        if left_obs[0] is not None and right_obs[0] is not None:
            
            #print("between")
            # ball inbetween left and right obs
            gap = abs(np.sqrt(left_obs[0]**2 + right_obs[0]**2 - 2*left_obs[0]*right_obs[0]*np.cos(left_obs[1] - right_obs[1])))
            dist_dif = - left_obs[0] + right_obs[0]

            m1l, m1r = self.goto_target_simple()
            m2l, m2r = 0, 0

            if gap > 30:
                # right obs below left obs
                if dist_dif <= 0:
                    m2l, m2r = rb_to_pwm(left_obs[0], left_obs[1] * (10 + abs(dist_dif)) / 10)
                else:
                    m2l, m2r = rb_to_pwm(right_obs[0], right_obs[1] * (10 + abs(dist_dif)) / 10)
            else:
                if dist_dif <= 0:
                    m2l, m2r = rb_to_pwm(right_obs[0], right_obs[1] + 16)
                else:
                    m2l, m2r = rb_to_pwm(left_obs[0], left_obs[1] - 16)

            ml = (m1l - m2l) / 2
            mr = (m1r - m2r) / 2

            return ml, mr

        if left_obs[0] is None:
            # ball to the right of right obs
            #print("right")
            heading = max((right_obs[1] - (251 - right_obs[0])), self.target_h)
            return rb_to_pwm(self.target_r, heading)
            

        if right_obs[0] is None:
            print("left")
            # ball to the left of left object
            heading = max(left_obs[1] - (251 - left_obs[0]), self.target_h)
            return rb_to_pwm(self.target_r, heading)
            
        else:
            print("uh oh")'''
            
    def goto_target(self):
        
        
        def avoid(heading, dist):
            left = heading - ((251 - dist) * (175 / 250))
            right = heading + ((251 - dist) * (175 / 250))
            return left, right
        
        if self.obs_r is None or self.obs_r - 10 > self.target_r:
            return self.goto_target_simple()
        
        else:
            left_avoid, right_avoid = avoid(self.obs_h, self.obs_r)
            
            if self.target_h <= left_avoid or self.target_h >= right_avoid:
                return rb_to_pwm(self.target_r, self.target_h)
            
            elif left_avoid < self.target_h <= self.obs_h:
                return rb_to_pwm(self.obs_r, left_avoid)
            
            elif right_avoid > self.target_h > self.obs_h:
                return rb_to_pwm(self.obs_r, right_avoid)
            
            else:
                return 0, 0                
            


    def goto_target_simple(self):
        return rb_to_pwm(self.target_r, self.target_h)
    
    def stop(self):
        self.drive_sys.stop()
        self.dribbler.stop()
        GPIO.cleanup()
