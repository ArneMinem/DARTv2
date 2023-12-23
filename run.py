""" Script de lancement du robot """

from Missions import *

if __name__ == "__main__" :
    deg_filt = 6
    Kp = 2
    Kd = 110
    p = 10
    v = 80
    w = 90
    lim_dfront = 40
    dwall = 24  # measured
    max_dwall = 70
    
    ctrl = Cmd(deg_filt, Kp, Kd, lim_dfront, dwall, v, w, p, max_dwall)
    ctrl.mission(180)