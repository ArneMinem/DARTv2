""" Script de lancement du robot """

import drivers_v2.drivers_v2 as drv2
import sys
import time
import numpy as np
import drivers_v2.dartv2_control as dartv2_control
import drivers_v2.filt as filter
import drivers_v2.mission as mission


if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot_ctrl = dartv2_control.DartV2Control(mybot)
    mission.mvmt.followWalls(mybot, 100, 60)

    mybot.end()