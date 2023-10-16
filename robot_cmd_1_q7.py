import drivers_v2.drivers_v2 as drv2
import drivers_v2.dartv2_control as dartv2_control 
import time

if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot_ctrl = dartv2_control.DartV2Control(mybot)

    # place your work here
    print ("Encoder Microcode Version : ",mybot.encoders.get_version())
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
    print ("Rear encoders before : ",mybot_ctrl.get_rear_encoders(init=True))

    for ileg in range(2):
        mybot.powerboard.set_speed (100, 100)
        for i in range(10):
            print ("Rear encoders [L,R]",mybot.encoders.read_encoders())
            print ("Sonars front diag",mybot.sonars.read_diag_both())
            time.sleep(0.5)
        mybot.powerboard.set_speed (100, -100)
        time.sleep(1.2156) # empirical !! may change with cpu !!! 
        mybot.powerboard.set_speed (0,0)

    odo_left,odo_right = mybot_ctrl.get_rear_encoders()
    
    print ("Rear encoders after : ",[odo_left,odo_right])
    deltaOdoLeft = mybot_ctrl.delta_rear_odometers(side="left")
    deltaOdoRight = mybot_ctrl.delta_rear_odometers(side="right")
    print ("Delta odometer left :", deltaOdoLeft)
    print ("Delta odometer right :", deltaOdoRight)
    
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
    
    mybot.end() # clean end of the robot mission

