#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    Z = 0.7

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    for cf in allcfs.crazyflies:
        cf.takeoff(targetHeight=Z, duration=1.0+Z)
        # timeHelper.sleep(1.0)
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        Z += 0.2
    timeHelper.sleep(1.0)
    
    # allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    # timeHelper.sleep(1.5+Z)
    # for cf in allcfs.crazyflies:
    #    pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
    #    cf.goTo(pos, 0, 1.0)

    print('press button to continue...')
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)


if __name__ == '__main__':
    main()
