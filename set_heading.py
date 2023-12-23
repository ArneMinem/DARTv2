#import dartv2b
import drivers_v2.drivers_v2 as drv2
import time
import sys
import math

def delta_heading(head_ref,head):
    head_err = head_ref-head
    while head_err > 180.0:
        head_err -= 360.0
    while head_err <= -180.0:
        head_err += 360.0
    #print ("heading error",head_ref,head,head_ref-head,head_err)
    return head_err


if __name__ == "__main__":
    #mybot = dartv2b.DartV2()
    mybot = drv2.DartV2DriverV2()


    head0 = 90.0
    try:
        head0 = float(sys.argv[1])
    except:
        pass

    duration = 10.0
    try:
        duration = float(sys.argv[2])
    except:
        pass

    kp = 0.01
    try:
        kp = float(sys.argv[3])
    except:
        pass

    spd_offs = 140
    try:
        spd_offs = float(sys.argv[4])
    except:
        pass

    spd_lin = 80
    try:
        spd_lin = int(sys.argv[5])
    except:
        pass

    accel_filter_mode = 2
    mybot.imu.setup_accel_filter(accel_filter_mode)
    
    # set up front sonar
    mybot.sonars.set_dist_max(1,2.0)
    mybot.sonars.set_mode(1,1)

    # set fast mag calib data DART02
    magx_min = 902
    magx_max = 3894
    magy_min = -3510
    magy_max = -731
    # set fast mag calib data DART04
    magx_min = -825
    magx_max = 3088
    magy_min = -4548
    magy_max = -746    

    # DART02 - 2021/12/2
    magx_min = 779
    magx_max = 4088
    magy_min = -3622
    magy_max = -553

    # DART13 - 2023/10/20
    magx_min =  -1306
    magx_max = 3078
    magy_min = -5092
    magy_max = -923

    mybot.imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)

    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    t0 = time.time()
    while True:
        if time.time()-t0 >= duration:
            break
        time.sleep(0.050)
        mag = mybot.imu.read_mag_raw()
        accel = mybot.imu.read_accel_raw()
        gyro = mybot.imu.read_gyro_raw()
        choc = math.sqrt(accel[0]*accel[0]+accel[1]*accel[1])
        if math.fabs(choc) > 3000:
            print ("abort on choc ...",choc)
            break
        head = mybot.imu.heading_deg(mag[0],mag[1])
        head_err = delta_heading(head0,head)
        d_spd = kp*head_err
        if d_spd >= 0:
            d_spd += spd_offs
        else:
            d_spd -= spd_offs
        spdl = spd_lin + int(-d_spd)
        spdr = spd_lin + int(+d_spd)
        print (choc,head0,head,head_err,d_spd,spdl,spdr)
        if abs(head_err) < 2 and spd_lin == 0:
            break
        mybot.powerboard.set_speed(spdl,spdr)
    
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
    mybot.powerboard.set_speed(0,0) # stop motors
    mybot.end() # clean end of the robot mission
