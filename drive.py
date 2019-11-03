import RPi.GPIO as GPIO
import time

# left motor is slightly slower than right, so this blances it

def fix_pwm(left, right):
    left = abs(left)
    right = abs(right)

    if right > 100:
        right = 100

    elif right < 0:
        right = 0

    if left > 100:
        left = 100

    elif left < 0:
        left = 0   
        
    return left, right

def check_speed(speed):
    if speed > 100:
        speed = 100
    if speed < 0:
        speed = 0
    return speed

class Drive():
# OLD
#    LEFTPWM = 21
#    LEFTA = 20
#    LEFTB = 16
#    RIGHTPWM = 26
#    RIGHTA = 19
#    RIGHTB = 13

# GPIO 
    '''
    LEFTPWM = 21
    LEFTA = 5
    LEFTB = 22
    '''
    RIGHTPWM = 4
    RIGHTA = 27
    RIGHTB = 17 
    
  
# PINS
    
    LEFTPWM = 13
    LEFTA = 19
    LEFTB = 26
    '''
    RIGHTPWM = 7
    RIGHTA = 13
    RIGHTB = 11 
    '''

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.LEFTPWM, GPIO.OUT)
        GPIO.setup(self.LEFTA, GPIO.OUT)
        GPIO.setup(self.LEFTB, GPIO.OUT)
        GPIO.setup(self.RIGHTPWM, GPIO.OUT)
        GPIO.setup(self.RIGHTA, GPIO.OUT)
        GPIO.setup(self.RIGHTB, GPIO.OUT)

        self.pwmL = GPIO.PWM(self.LEFTPWM, 500)
        self.pwmR = GPIO.PWM(self.RIGHTPWM, 500)

    def drive(self, a, b):

        duty_left, duty_right = fix_pwm(a, b)

        # left wheel
        self.pwmL.start(duty_left)
        if a < 0:
            GPIO.output(self.LEFTA,0)
            GPIO.output(self.LEFTB,1)
        else:
            GPIO.output(self.LEFTB,0)
            GPIO.output(self.LEFTA,1)

        # right wheel
        self.pwmR.start(duty_right)
        if b < 0:
            GPIO.output(self.RIGHTA,0)
            GPIO.output(self.RIGHTB,1)
        else:

            GPIO.output(self.RIGHTB,0)
            GPIO.output(self.RIGHTA,1)


    cycle_count = 0

    # will return True when finished
    def spin(self, speed=50, direction='ccw', radius=0, cycles=None):

        self.cycle_count += 1
        
        def _spin(speed, direction, radius):
            if(direction == 1):
                self.drive( (direction * speed), (-direction * speed) + radius)
            else:
                self.drive( (direction * speed) + radius, (-direction * speed))


        if direction == 'cw' or 1:
            direction = 1
        elif direction == 'ccw' or -1:
            direction = -1
        else:
            self.stop()
            return True

        if cycles is None:
            _spin(speed, direction, radius)
            return False
            
        if self.cycle_count <= cycles:
            _spin(speed, direction, radius)
            return False

        else:
            self.stop()
            self.cycle_count = 0
            return True

    def straight(self, speed):
        self.drive(check_speed(speed), check_speed(speed))

    def reverse(self):
        self.drive(check_speed(-self.speed), check_speed(-self.speed))

    def stop(self):
        self.pwmL.stop()
        self.pwmR.stop()


if __name__ == '__main__':
    drive = Drive()

    ml = 50
    mr = 50

    try:
        drive.drive(ml, mr)
        time.sleep(3)
        drive.pwmR.stop()
        drive.pwmL.stop()

    except KeyboardInterrupt:
        drive.stop()
        GPIO.cleanup()

    finally:
        drive.stop()
        GPIO.cleanup()
