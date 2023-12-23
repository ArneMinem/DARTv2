# import sys
# sys.path.append("./drivers_v2")

import drivers_v2.drivers_v2 as drv2
import drivers_v2.dartv2_control as dartv2_control
import time
import numpy as np

class Cmd :
    
    def __init__(self, deg_filt, Kp, Kd, lim_dfront, dwall, v, w, p, max_dwall) :
        
        self.mybot = drv2.DartV2DriverV2()
        self.mybot_ctrl = dartv2_control.DartV2Control(self.mybot)
        
        self.tpt = 300
        self.D_roue = 0.125
        self.tpr = 930
        
        self.v = v
        self.w = w
        self.p = p
        
        self.deg_filt = deg_filt
        
        self.sf = [self.mybot.sonars.read_4_sonars()[0]] * self.deg_filt
        self.sl = [self.mybot.sonars.read_4_sonars()[1]] * self.deg_filt
        self.sre = [self.mybot.sonars.read_4_sonars()[2]] * self.deg_filt
        self.sri = [self.mybot.sonars.read_4_sonars()[3]] * self.deg_filt
        
        # self.mybot.imu.fast_heading_calibration(1792, 4256 ,-4830, -2526)
        # self.mybot.imu.fast_heading_calibration(-1000, 999, -1000, 998)
        self.mybot.imu.fast_heading_calibration(-363, 2843, -4162, -1107)

        
        self.lim_dfront = lim_dfront
        self.dwall = dwall
        self.max_dwall = max_dwall
        self.Kp = Kp
        self.Kd = Kd
                
    def wait(self, t) :
        self.mybot.powerboard.set_speed(0, 0)
        time.sleep(t)
        
    def forward_d(self, d) :
        n_t = self.tpt * d / ( np.pi * self.D_roue) / 100
        n = 0
        
        odoL0, odoR0 = self.mybot_ctrl.get_front_encoders()
        
        while n < n_t :
            self.mybot.powerboard.set_speed(self.v, self.v)
            odoL, odoR = self.mybot_ctrl.get_front_encoders()
            n = abs(odoL - odoL0)
            print("Nombre de ticks restants = ", n_t - n)
            
        self.mybot.powerboard.set_speed(0, 0)
        
    def turn_odo(self, a) :
        n_t = self.tpr * abs(a) / 360
        n = 0
        
        odoL0, odoR0 = self.mybot_ctrl.get_front_encoders()
        
        while n < n_t :
            
            if a < 0 : # ie tourner à droite
                self.mybot.powerboard.set_speed(self.w, -self.w)
            else :
                self.mybot.powerboard.set_speed(-self.w, self.w)
                
            odoL, odoR = self.mybot_ctrl.get_front_encoders()
            n = abs(odoL - odoL0)
            print("Nombre de ticks restants = ", n_t - n)
        
        self.mybot.powerboard.set_speed(0, 0)
        
    def turn_comp(self, target) : # p for precision
        s = target
        mag = self.mybot.imu.read_mag_raw()
        target = (self.mybot.imu.heading(mag[0],mag[1]) * 180.0 / np.pi + target)%360
        e = np.inf
        
        while abs(e) > self.p :
            if s < 0 :
                self.mybot.powerboard.set_speed(-self.w, self.w)
            else :
                self.mybot.powerboard.set_speed(self.w, -self.w)
                
            mag = self.mybot.imu.read_mag_raw()
            e = (target - self.mybot.imu.heading(mag[0],mag[1]) * 180.0 / np.pi)%360
            print("Ecart de cap = ", e)
        
        self.mybot.powerboard.set_speed(0, 0)

            
    def filt_s(self) :
        d_f, d_l, d_re, d_ri = self.mybot.sonars.read_4_sonars()  # distance in cm
        print("d_f = ", d_f)

        # On ajoute la mesure à notre liste
        # On elimine la première variable de-631,3170,-4489,-900
        if 0 <= d_f:
            self.sf.append(d_f)
            self.sf.pop(0)
            
        if 0 <= d_l:
            self.sl.append(d_l)
            self.sl.pop(0)

        if 0 <= d_re:
            self.sre.append(d_re)
            self.sre.pop(0)

        if 0 <= d_ri:
            self.sri.append(d_ri)
            self.sri.pop(0)


        d_f_filt = np.average(self.sf)
        d_l_filt = np.average(self.sl)
        d_re_filt = np.average(self.sre)
        d_ri_filt = np.average(self.sri)
        
        return d_f_filt, d_l_filt, d_re_filt, d_ri_filt

    def mission(self, t_max) :
        nb_turns = 0
        mode = "FW"
        t_start = time.time()
        while nb_turns != 12 :
            if mode == "FW" :
                Cmd.wait(self, 1)
                mode = Cmd.followWalls(self)
                print(mode)
            elif mode == "L" :
                Cmd.wait(self, 1)
                mode = Cmd.turnL(self)
                print(mode)
                nb_turns += 1
            else :
                Cmd.wait(self, 1)
                mode = Cmd.turnR(self)
                print(mode)
                nb_turns += 1
            
            if (time.time() - t_start) > t_max:
                break # max time reached , escape the loop ...
        else :
            exit
                
    def followWalls(self) :
        derivOK = False
        Err0 = 0
        d_f_filt, d_l_filt, d_re_filt, d_ri_filt = Cmd.filt_s(self)
        
        while d_f_filt > self.lim_dfront :
            
            if d_l_filt < self.max_dwall :
                print("go_right")
                err_l = self.dwall - d_l_filt
                if derivOK :
                    derivErr = err_l - Err0
                    dv = self.Kp * err_l + self.Kd * derivErr
                else :
                    dv = self.Kp * err_l

                Err0 = err_l
                derivOK = True
                err_l = self.dwall - d_l_filt

                self.mybot.powerboard.set_speed(self.v + dv, self.v - dv)
            
            elif d_ri_filt < self.max_dwall :
                
                print("go_left")
                err_r = self.dwall - d_ri_filt
                if derivOK :
                    derivErr = err_r - Err0
                    dv = self.Kp * err_r + self.Kd * derivErr
                else :
                    dv = self.Kp * err_r

                Err0 = err_r
                derivOK = True
                err_r = self.dwall - d_ri_filt

                self.mybot.powerboard.set_speed(self.v - dv, self.v + dv)
            
            else :
                self.mybot.powerboard.set_speed(self.v, self.v)
            
            d_f_filt, d_l_filt, d_re_filt, d_ri_filt = Cmd.filt_s(self)
            print("d_f_filt = ", d_f_filt)
        
        if d_l_filt > self.max_dwall :
            print("modechange L")
            return "L"
            
        else :
            print("modechange R")
            return "R"
            
    def turnL(self) :
        Cmd.turn_comp(self, -90)
        return "FW"
        
    def turnR(self) :
        Cmd.turn_comp(self, 90)
        return "FW"