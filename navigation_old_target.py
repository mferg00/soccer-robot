import RPi.GPIO as GPIO
from drive import Drive
from dribbler import Dribbler
from field import Field
from enum import Enum
from kicker import Kicker
import time
import numpy as np

#########
# ENUMS #
#

class state(Enum):
    none = 1
    ball_search = 2
    go_to_ball = 3
    goal_search = 4
    go_to_goal = 5
    shoot_ball = 6

class search(Enum):
    none = 1
    spin = 2
    go_to_obstacle = 3
    circle_obstacle = 4
    next_obstacle = 5


############
# FUNCTIONS
# these should be very generic

def within_angle(heading, limit):
    if(heading <= limit and heading >= -limit):
        
        return True
    return False

def check_speed(speed):
    if speed > 100:
        speed = 100
    if speed < -100:
        speed = -100
    return speed

def cap_ran(ran):
    if ran is None:
        return None
    
    if(ran < 5):
        ran = 5
    elif(ran > 250):
        ran = 250
        
    return ran

###########
# VARIABLES
# used for condition checking or generic functions within navigation class

BALL_IN_DRIBBLER_DIST = 9 # min distance when ball is in the dribbler
BALL_IN_DRIBBLER_ANGLE = 15 # ball must be within +-28 deg of fov to be in dribbler (32 is max)

GOAL_DIST = 90 # distance to stop at to shoot for goal
GOAL_ANGLE = 3  # goal must be within +-3 deg of fov to be in dribbler

OBSTACLE_DIST = 35  # min dist to begin circ,ing object

MIN_SPEED = 50  # the speed the robot will be at when its target is very close


#############
# NAV CLASS #
#

class Navigation():

    def __init__(self):

        # this is only changed once the ball is kicked
        self.switch_goal = False
        self.prev_state = state.none
        self.prev_ball = -1
        self.prev_goal = -1
        self.prev_wall = -1

        # CONDITIONS
        self.ball_in_dribbler = False
        self.ball_found = False
        self.goal_found  = False
        self.goal_aligned = False

        # STATES
        self.state = state.none
        self.search = search.none

        # OBJECTS
        self.drive_sys = Drive()
        self.dribbler = Dribbler()
        self.field = Field()
        self.kicker = Kicker()

        # MOTOR pwm
        self.left_motor = 0
        self.right_motor = 0

        # range/headings
        self.obs_range = None
        self.obs_heading = None
        self.ball_range = None
        self.ball_heading = None
        self.goal_range = None
        self.goal_heading = None


    ####################
    # GENERIC FUNCTIONS
    #

    # input: left and right motor pwms
    # output: pwm value that gets larger the sharper the robot is turning
    # WORK IN PROGRESS
    def choose_drib_speed(self):

        # find ratio: -1 is spinning on spot, 1 is going straight
        ratio = 0
        if(self.left_motor == 0 or self.right_motor == 0):
            ratio = 0

        else:
            ratio = min(self.left_motor, self.right_motor)/max(self.left_motor, self.right_motor)

        # convert ratio to: 0 is straight, 2 is spinning on spot
        ratio = -ratio + 1

        # if motor is going forward (ratio = 0), dribbler is at 50 pwm
        # if motor is doing a full spin (ratio = 2), dribbler is at 90 pwm
        speed = 60 + (ratio * 100)

        return check_speed(speed)

    # input: object heading/range
    # output: drive and center towards object, with speed slowing down based on proximity
    # example: object at 100cm, -16deg:
    #          left_motor: 50 + 100/8 - -16 = 78.5
    #          right_motor: 50 + 100/8 + -16 = 46.5
    def goto_object_simple(self, speed, ran, head):

        def speed(dist):
            return 1 - ((251 - dist) * (0.5/250))
        
        if ran is None:
            return 0, 0
        
        # print(head)
        
        ml = (60 + head) * speed(ran)
        mr = (60 - head) * speed(ran)

        self.drive_sys.drive(ml, mr)

    # input: object target
    # output: drive towards object and avoid obstacles
    def goto_object(self, speed=1, object='ball'):
        
        def avoid(heading, dist):
            left = heading - ((251 - dist) * (100 / 250))
            right = heading + ((251 - dist) * (100 / 250))
            return left, right
        
        if object == 'ball':
            target_r = self.ball_range
            target_h = self.ball_heading
            
        else:
            target_r = self.goal_range
            target_h = self.goal_heading
        
        if target_r is None:
            self.drive_sys.spin(25, direction=self.prev_ball)
            
        elif self.obs_range is None:
            self.goto_object_simple(1, target_r, target_h)
        
        else:
            left_avoid, right_avoid = avoid(self.obs_heading, self.obs_range)
            
            if target_h <= left_avoid or target_h >= right_avoid:
                #print("space clear")
                self.goto_object_simple(1, target_r, target_h)
            
            elif left_avoid < target_h <= self.obs_heading:
                #print("avoiding")
                self.goto_object_simple(1, target_r, left_avoid)
            
            elif right_avoid > target_h >= self.obs_heading:
                self.goto_object_simple(1, target_r, right_avoid)
            
            else:
                self.drive_sys.spin(speed=25, direction=self.prev_ball) 


    def stop(self):
        self.drive_sys.stop()
        self.dribbler.stop()
        GPIO.cleanup()


    ####################
    # UPDATE FUNCTION
    #

    # input: vision feed values
    # output: true if goal needs to be switched
    def update(self, obs, ball, goal, wall):

        # return true if goal needs to be switched, so vision can be changed from main function
        if(self.switch_goal):
            self.switch_goal = False
            return True

        # VISION INFO
        self.obs_range = cap_ran(obs[0])
        self.obs_heading = obs[1]
        self.ball_range = cap_ran(ball[0])
        self.ball_heading = ball[1]
        self.goal_range = cap_ran(goal[0])
        self.goal_heading = goal[1]

        if wall[0] is not None:
            self.prev_wall = np.sign(wall[1])
            
        # ACT BASED ON VISION INFO
        self.decide_conditions()
        self.decide_state()
        self.decide_action()

        return False


    ##################
    # DECIDE FUNCTIONS
    # decides what to do based on the data from update()

    # input: vision data
    # output: condition bool variables
    dribbler_counter = 0
    def decide_conditions(self):

        # if ball is seen
        if(self.ball_range is not None):
            self.ball_found = True
            self.prev_ball = np.sign(self.ball_heading)

            # if ball is within dribbler range/angle
            if(self.ball_range < BALL_IN_DRIBBLER_DIST and within_angle(self.ball_heading, BALL_IN_DRIBBLER_ANGLE)):
                self.ball_in_dribbler = True
            else:
                self.ball_in_dribbler = False

        else:
            self.ball_found = False

        # if goal is seen
        if(self.goal_range is not None):
            self.goal_found = True
            self.prev_goal = np.sign(self.goal_heading)

            #if goal is within shooting range/angle
            if(self.goal_range < GOAL_DIST and within_angle(self.goal_heading, GOAL_ANGLE)):
                self.goal_aligned = True
            else:
                self.goal_aligned = False
        else:
            self.goal_found = False

    def change_state(self, state):
        self.state = state
        
    # input: condition variables
    # output: state value
    def decide_state(self):

        if(not self.ball_found and not self.ball_in_dribbler):
            self.state = state.ball_search

        elif(self.ball_found and not self.ball_in_dribbler):
            self.state = state.go_to_ball

        elif(self.ball_found and self.ball_in_dribbler and not self.goal_found):
            self.state = state.goal_search
            
            if(self.prev_state != self.state):
                self.drive_sys.stop()
                time.sleep(0.25)
            

        elif(self.ball_found and self.ball_in_dribbler and self.goal_found and not self.goal_aligned):
            self.state = state.go_to_goal

        elif(self.ball_found and self.ball_in_dribbler and self.goal_found and self.goal_aligned):
            self.state = state.shoot_ball

        else:
            self.state = state.ball_search
            
        self.prev_state = self.state

        # print(self.state)


    # input: state/search value
    # output: appropriate function
    def decide_action(self):

        if(self.state == state.ball_search):
            self.ball_search()
        
        else:
            
            self.search_lock = False
            if(self.state == state.go_to_ball):                
                self.go_to_ball()
        
            if(self.state == state.goal_search):
                self.goal_search()
        
            if(self.state == state.go_to_goal):
                self.go_to_goal()
        
            if(self.state == state.shoot_ball):
                self.shoot_ball()

        # if state == none
            else:
                pass


    ##################
    # ACTION FUNCTIONS
    #

    # input: search value
    # output: appropriate ball searching method
    search_lock = False
    def ball_search(self):

        if(not self.search_lock):
            if(self.search == search.none):
                self.search = search.spin
    
            if(self.search == search.spin):
                # will not be true until until spin is complete
                #print("searching for ball")
                if(self.drive_sys.spin(speed=30, direction=self.prev_ball, radius=0, cycles=10)):
                    self.search = search.go_to_obstacle

        if(self.search == search.go_to_obstacle):
            self.search_lock = True
            if(self.obs_range is None):
                #print("searching for obstacle")
                self.drive_sys.spin(speed=30, direction=self.prev_ball, radius=0, cycles=None)
            else:
                if(self.obs_range > OBSTACLE_DIST):
                    #print("goi8ng to obstacle")
                    self.goto_object_simple(1, self.obs_range, self.obs_heading)
                else:
                    
                    self.search = search.circle_obstacle

        if(self.search == search.circle_obstacle):
            #print("circling obstacle")
            # will not be true until circle is complete
            if(self.circle_obstacle(adjust=100)):
                #print("CIRCLE OBSTACLE COMPLETE")
                # if after all that the ball is not found, restart the cycle again
                self.search = search.none
                self.search_lock = False


    orientate = True
    spin_out_timer = 0
    right_wheel = 40
    left_wheel = 30
    prev_state = 0
    adjust_counter = 0

    # input: largest obstacle range/heading
    # output: circling obstacle
    def circle_obstacle(self, adjust=10):
        
        # count how many times the robot has to adjust based on if it saw the object
        if(self.obs_range is None and self.prev_state is not None):
            self.adjust_counter += 1

        self.prev_state = self.obs_range

        # if it has reached the limit, reset everything for the next time the function is called
        if(self.adjust_counter >= adjust):
            self.orientate = True
            self.spin_out_timer = 0
            self.right_wheel = 40
            self.left_wheel = 30
            self.prev_state = 0
            self.adjust_counter = 0
            self.drive_sys.stop()
            return True

        # if the robot needs to oreintate itself
        if(self.orientate and self.obs_range is not None):
            self.drive_sys.drive(-30, 30)
            self.spin_out_timer = 0

        # else if it needs to circle
        else:
            self.orientate = False

            if self.obs_range is not None:
                self.spin_out_timer = 0
                self.right_wheel = 40
                self.left_wheel = 30

            if(self.obs_range is None):
                self.spin_out_timer += 1
                self.right_wheel += 0.4
                self.left_wheel -= 0.4

            # spin in
            if(self.spin_out_timer >= 5):
                self.drive_sys.drive(self.right_wheel, self.left_wheel)

            # spin out
            if(self.spin_out_timer < 5):
                self.drive_sys.drive(-40, 40)

        return False


    # input: ball range/heading, ball_in_dribbler variable
    # output: driving to ball, turning dribbler on if in range
    #         WILL DRIVE STRAIGHT TOWARDS BALL, IGNORING OPBSTACLES
    def go_to_ball(self):
        # self.goto_object_simple(1, self.ball_range, self.ball_heading)
        # REPLACE ^ WITH:
        self.goto_object(speed=1, object='ball')

        if(self.ball_range < 20):
            self.dribbler.start(95)
        else:
            self.dribbler.stop()

    # input:
    # output: turning in a circle without losing ball
    def goal_search(self):
        self.dribbler.start(100)
        self.drive_sys.spin(speed=20, direction=self.prev_goal, radius=10, cycles=None)

    def go_to_goal(self):
        self.dribbler.start(95)
        self.goto_object(speed=1, object='goal')
        # REPLACE ^ WITH:
        # self.goto_object(speed=0.65, object='goal')

    def shoot_ball(self):

        self.dribbler.stop()
        self.kicker.kick()
        self.drive_sys.stop()
        self.switch_goal = True
