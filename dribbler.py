import RPi.GPIO as GPIO
import time

def check_pwm(pwm):
    if pwm > 100:
        pwm = 100
    elif pwm < 0:
        pwm = 0
    return pwm

class Dribbler():
    
#    DRIBPWM = 6
#    DRIBA = 5
#    DRIBB = 12
    DRIBPWM = 16
    DRIBA = 20
    DRIBB = 21


    def __init__(self):

        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.DRIBPWM, GPIO.OUT)
        GPIO.setup(self.DRIBA, GPIO.OUT)
        GPIO.setup(self.DRIBB, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.DRIBPWM, 500)
    
    def start(self, speed=90):
        self.pwm.start(check_pwm(speed))
        GPIO.output(self.DRIBA,0)
        GPIO.output(self.DRIBB,1)
    
    def stop(self):
        self.pwm.stop()
 
        
        
if __name__ == '__main__':
    dribbler = Dribbler()
    time.sleep(2)


    try:
        dribbler.start()
        time.sleep(1)
        dribbler.stop()
            # if drive.spin(direction='ccw', revolutions=1) == False:
                # break

    except KeyboardInterrupt:
        GPIO.cleanup()

    finally:
        GPIO.cleanup()
