import RPi.GPIO as GPIO
import time

class Kicker():
    KICKRELAY = 18
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.KICKRELAY, GPIO.OUT)
    
    def kick(self):
        GPIO.output(self.KICKRELAY,1)
        time.sleep(0.2)
        GPIO.output(self.KICKRELAY,0)
        
if __name__ == '__main__':
    kicker = Kicker()
    time.sleep(2)


    try:
        kicker.kick()
            # if drive.spin(direction='ccw', revolutions=1) == False:
                # break

    except KeyboardInterrupt:
        GPIO.cleanup()

    finally:
        GPIO.cleanup()
