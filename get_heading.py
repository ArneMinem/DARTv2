#import sys
#sys.path.insert(0, "../drivers_v2")
#sys.path.insert(0, "../vDartV2")
#import drivers_v2 as drv2
import drivers_v2.drivers_v2 as drv2
import time
import sys
import math
#import mag_data

def delta_heading(head_ref,head):
    head_err = head_ref-head
    while head_err > 180.0:
        head_err -= 360.0
    while head_err <= -180.0:
        head_err += 360.0
    #print ("heading error",head_ref,head,head_ref-head,head_err)
    return head_err


if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()

    duration = 1.0
    try:
        duration = float(sys.argv[2])
    except:
        pass

    dt = 0.1
    try:
        dt = float(sys.argv[2])
    except:
        pass

    #magx_min, magx_max, magy_min, magy_max = mag_data.get_calib()
    magx_min, magx_max, magy_min, magy_max = -363, 2843, -4162, -1107 # -1000, 999, -1000, 998 # -631,3170,-4489,-900
    mybot.imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)

    #print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    t0 = time.time()
    while True:
        if time.time()-t0 >= duration:
            break
        mag = mybot.imu.read_mag_raw()
        print ("heading : %.2f"%(mybot.imu.heading_deg(mag[0],mag[1])))
        time.sleep(dt)
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    mybot.powerboard.stop() # stop motors
    mybot.end() # clean end of the robot mission
