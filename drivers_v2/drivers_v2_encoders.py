#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import time

class EncodersIO():
    def __init__(self,exec_robot,vsv=None):
        self.vsv = vsv
        self.__exec_robot = exec_robot
        self.__sim = False
        self.__ros = False
        self.__real = False
        if exec_robot == "Sim V-REP":
            self.__sim = True
        elif exec_robot == "Sim GAZEBO":
            self.__sim = True
        elif exec_robot == "Real":
            self.__real = True
        elif exec_robot == "Real ROS":
            self.__ros = True

        # define I2C bus number 
        self.__bus_nb = 2
        # define I2C address of the encoders sensor
        self.__addr = 0x00 # replace 0x00 with the correct address

        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb,vsv=self.vsv)
        elif self.__real:
            import i2creal as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
        elif self.__ros:
            import dartv2_drivers.drivers.i2creal as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
            

    # define drivers functions here 
    """
    def get_version(self):
        version = ?
        return version

        
    def read_encoders(self):
        self.enc_left = ?
        self.enc_right = ?
        return [self.enc_left, self.enc_right]

    def read_motors_direction (self):
        self.motor_dir_left = ?
        self.motor_dir_right = ?
        return [self.motor_dir_left, self.motor_dir_right]
        
    def get_battery_level (self):
        return ?
        
    """
    
    # low level functions to access I2C bus
    def __read(self,offs):  # read  16 bits (2 bytes)
        v=0
        while True:
            try:
                v = self.__dev_i2c.read(offs,2)
                #print (v)
                v = v[0] + (v[1] << 8)
                break
            except:
                v = None
                time.sleep(0.0005)
        return v        

    def __read_byte (self,offs): # read 8 bits (1 byte) at address offs
        v=0
        try:
            v = self.__dev_i2c.read_byte(offs)
        except:
            v = None
        return v
    
    def __write_byte (self,offs,val): # write 8 bits (1 byte) value val at address offs
        v = 0
        try:
            self.__dev_i2c.write(offs,[val])
        except:
            v = None
        return v
                  
if __name__ == "__main__":
    # warning, tests are quite complex in simulation as we need to connect
    # the module to the V-REP simulator...

    # test if on real robot , if gpio exists (not very robust)
    # add test on processor type, real robot has armv7l
    tstsim = False
    if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
       and (platform.processor() == 'armv7l'):
        encoders = EncodersIO("Real")
        print ("Work with real DART")
    # if not the virtual robot is running in V-REP
    else :
        tstsim = True
        sys.path.append('../vDartV2')
        import vSimVar as vsv
        tSimVar= vsv.tSimVar
        encoders = EncodersIO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")
