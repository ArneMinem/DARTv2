""" Script de controle des mouvements du robot """

import drivers_v2.drivers_v2 as drv2
import sys
import time
import numpy as np
import drivers_v2.dartv2_control as dartv2_control
import drivers_v2.filt as filter

class mvmt():
    def __init__(self):
        self.distBetweenWheels = 0
        self.nTicksPerRevol = 300
        self.wheelDiameter = 0.12
        self.FOL = filter.LowPassFilter()
        self.FOR = filter.LowPassFilter()
        self.FOB = filter.LowPassFilter()
        self.speed_left = 0
        self.speed_right = 0
        self.PID = filter.PID(0.01, 0.01, 0.8, 0.1)

    def forth(self,v):
        mybot = drv2.DartV2DriverV2()
        mybot.powerboard.set_speed (v,v)

    def back(self,v):
        mybot = drv2.DartV2DriverV2()
        mybot.powerboard.set_speed (-v,-v)

    def turnLeft(self):
        mybot = drv2.DartV2DriverV2()
        mybot.powerboard.set_speed (-100,100)
        time.sleep(1.2156/2) # empirical !! may change with cpu !!! 
        mybot.powerboard.set_speed (0,0)

    def turnRight(self):
        mybot = drv2.DartV2DriverV2()
        mybot.powerboard.set_speed (100,-100)
        time.sleep(1.2156/2) # empirical !! may change with cpu !!! 
        mybot.powerboard.set_speed (0,0)

    def detectObstacle(self, dist, v):
        mybot = drv2.DartV2DriverV2()
        mvmt.forth(v)
        f = mybot.sonars.read_front()
        if f < dist:
            mvmt.forth(0)

    def pos_wall(self, dist, v, w):
        mybot = drv2.DartV2DriverV2()
        if w == 'front':
            f = mybot.sonars.read_front()
            if f > dist-0.01 and f < dist+0.01:
                mvmt.forth(0)
            elif f < dist:
                mvmt.back(v)
                while f < dist:
                    f = mybot.sonars.read_front()
                mvmt.forth(0)
            elif f > dist:
                mvmt.forth(v)
                while f < dist:
                    f = mybot.sonars.read_front()
                mvmt.forth(0)
        elif w == 'rear':
            f = mybot.sonars.read_rear()
            if f > dist-0.01 and f < dist+0.01:
                mvmt.forth(0)
            elif f < dist:
                mvmt.front(v)
                while f < dist:
                    f = mybot.sonars.read_rear()
                mvmt.forth(0)
            elif f > dist:
                mvmt.back(v)
                while f < dist:
                    f = mybot.sonars.read_rear()
                mvmt.forth(0)

    def followWalls(self,spd_lin,duration_max):
        mybot = drv2.DartV2DriverV2()
        mybot_ctrl = dartv2_control.DartV2Control(mybot)
        mode = "FollowWalls"
        loop_iter_time = 0.1 # control at 10 Hz (10 commands/second)
        t_start = time.time()
        while True:
            df = mybot.sonars.read_front()  # get distance from front sonar
            if df > 0.0:
                print("front sonar distance = %.2f m" % (df))  # debug : print front distance on terminal

            dL = mybot.sonars.read_left()
            dR = mybot.sonars.read_right()
            dLFO = self.FOL.simpleLowPassFilter(dL)
            dRFO = self.FOR.simpleLowPassFilter(dR)

            if (dL > 0.50 or dL == 0) and (mode != "TurnLeft" and mode != "TurnRight"):
                print("changemode")
                odoLeft, odoRight = mybot_ctrl.get_front_encoders()
                NOR = odoRight
                mode = "TurnLeft"

            if (dR > 0.50 or dR == 0) and (mode != "TurnLeft" and mode != "TurnRight"):
                print("changemode")
                odoLeft, odoRight = mybot_ctrl.get_front_encoders()
                NOL = odoLeft
                mode = "TurnRight"

            diff = dLFO - dRFO

            coeff = self.PID.calculate(diff, 0)

            self.speed_left = spd_lin * (1 + coeff)
            self.speed_right = spd_lin * (1 - coeff)
            if mode == "FollowWalls":
                mybot.powerboard.set_speed (self.speed_left,self.speed_right)

            dBFO = self.FOB.simpleLowPassFilter(mybot.sonars.read_rear())
            print ("dBFO = ", dBFO)

            if mode == "TurnLeft":
                if NOR - odoRight < 250 :
                    NOL, NOR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(100, 100)
                    print("odoRight = ", odoRight)
                    N2OR = NOR
                    print("NOR = ", NOR)
                elif N2OR - NOR < 110 :
                    N2OL, N2OR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(-100,100)
                    print("N2OR = ", N2OR)
                    N3OR = N2OR
                elif N3OR - N2OR < 400 :
                    N3OL, N3OR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(spd_lin, spd_lin)
                    print("N3OR = ", N3OR)
                else:
                    mode = "FollowWalls"
                    print("je reviens")

            if mode == "TurnRight":
                if NOL - odoLeft < 250 :
                    NOL, NOR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(100, 100)
                    N2OL = NOL
                elif N2OL - NOL < 110 :
                    N2OL, N2OR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(100,-100)
                    N3OL = N2OL
                elif N3OL - N2OL < 400 :
                    N3OL, N3OR = mybot_ctrl.get_front_encoders()
                    mybot.powerboard.set_speed(spd_lin, spd_lin)
                else:
                    mode = "FollowWalls"
                    print("je reviens")

            t0loop = time.time()
            if (time.time() - t_start) > duration_max:
                break # max time reached , escape the loop ...

            # end of loop
            t1loop = time.time()
            dt_sleep = loop_iter_time - (t1loop - t0loop)
            if (dt_sleep > 0):
                time.sleep(dt_sleep) # wait to have perfect loop duration
            else:
                print ("too much computation in this loop, increase loop_iter_time or simplify computation ... ") 

