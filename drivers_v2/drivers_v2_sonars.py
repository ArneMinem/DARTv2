#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading

class SonarsIO():
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
        
        self.__bus_nb = 2
        self.__addr_4_sonars = 0x00  # replace 0x00 with correct IEC address


        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,bus_nb=self.__bus_nb,vsv=self.vsv)
        elif self.__real:
            import i2creal as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
        elif self.__ros:
            import dartv2_drivers.drivers.i2creal as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
       
        self.front = -1.0
        self.left = -1.0
        self.right = -1.0
        self.rear = -1.0

    # add the driver functions here
    
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
        sonars = SonarsIO("Real")
        print ("Work with real DART")
    # if not the virtual robot is running in V-REP
    else :
        tstsim = True
        sys.path.append('../vDartV2')
        import vSimVar as vsv
        tSimVar= vsv.tSimVar
        sonars = SonarsIO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")

