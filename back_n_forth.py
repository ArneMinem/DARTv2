import drivers_v2.drivers_v2 as drv2
import time

if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()

    
    mybot.powerboard.set_speed (50,50)
    time.sleep(1.0)
    mybot.powerboard.set_speed (-50,-50)
    time.sleep(1.0)
    mybot.powerboard.set_speed (0,0)
    time.sleep(0.05)
    
    mybot.end() # clean end of the robot mission

