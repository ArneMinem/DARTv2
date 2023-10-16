import drivers_v2.drivers_v2 as drv2
import drivers_v2.dartv2_control as dartv2_control 
import time


if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot_ctrl = dartv2_control.DartV2Control(mybot)

    # place your work here
    print ("Front encoders before : ",mybot_ctrl.get_front_encoders(init=True))
    
    mybot.powerboard.set_speed (100,-100)
    time.sleep(1.0)
    mybot.powerboard.set_speed (0,0)

    odo_left,odo_right = mybot_ctrl.get_front_encoders()
    print ("Front encoders after : ",[odo_left,odo_right])
    deltaOdoLeft = mybot_ctrl.delta_front_odometers(side="left")
    deltaOdoRight = mybot_ctrl.delta_front_odometers(side="right")
    print ("Delta odometer left :", deltaOdoLeft)
    print ("Delta odometer right :", deltaOdoRight)

    mybot.powerboard.set_speed (100,-100)
    time.sleep(1.0)
    mybot.powerboard.set_speed (0,0)

    odo_left,odo_right = mybot_ctrl.get_front_encoders()
    odos = mybot_ctrl.delta_front_odometers()
    dist_m = mybot_ctrl.ticks_meters(odos[0],odos[1])
    print ("Delta odometers both (reverse motion):",odos)
    print ("Distance parcourue:",dist_m)
    
    mybot.end() # clean end of the robot mission

