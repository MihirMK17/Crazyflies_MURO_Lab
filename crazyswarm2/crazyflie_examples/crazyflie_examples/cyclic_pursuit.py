#!/usr/bin/env python

from crazyflie_py import Crazyswarm

def rotate(timeHelper, cf, rate=100.0):
    start_time = timeHelper.time()

    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > 8.0:
            break

        cf.cmdHover(0.0, 0.0, 2.0, 1.0)
        timeHelper.sleepForRate(rate)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]

    cf.takeoff(targetHeight=1.0, duration=2.5)
    timeHelper.sleep(2.5)

    rotate(timeHelper, cf, rate=100.0)

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=2.0)
    timeHelper.sleep(2.0)

if __name__ == '__main__':
    main()

# def crazyflie_dynamics(cfs, K, velocity, n):
#     cfs = np.reshape(cfs, (n, 3))

#     x_dot = velocity*np.cos(cfs[:, 2])
#     y_dot = velocity*np.sin(cfs[:, 2])

#     omega = []
#     for i in range(n):
#         dx = cfs[(i+1)%n, 0] - cfs[i, 0]
#         dy = cfs[(i+1)%n, 1] - cfs[i, 1]
#         dtheta = np.arctan2(dy, dx)
#         if dtheta < 0:
#             dtheta += (2 * np.pi)
#             alpha = dtheta - cfs[i,2]
#             if alpha < -np.pi:
#                 alpha += (2 * np.pi)
#             elif alpha > np.pi:
#                 alpha -= (2 * np.pi)
#             omega.append(K*alpha)
#         omega = np.array(omega)

#     theta_dot = K*omega
#     state_vector_dot = np.array([x_dot, y_dot, theta_dot])

#     return state_vector_dot.flatten()

# def adjust_angle(cfs, n):
#     for i in range(n):
#         while cfs[(3*i)+2] > 2*np.pi:
#             cfs[(3*i)+2] -= 2*np.pi
#         while cfs[(3*i)+2] < 2*np.pi:
#             cfs[(3*i)+2] += 2*np.pi
#     return cfs

# def rk4(dynamics, cfs, K, velocity, dt):
#     K1 = dynamics(cfs, K, velocity)
#     K2 = dynamics(cfs + (K1*0.5*dt), K, velocity)
#     K3 = dynamics(cfs + (K2*0.5*dt), K, velocity)
#     K4 = dynamics(cfs + (K3*dt), K, velocity)

#     state_vector_slope = (K1 + (2*K2) + (2*K3) + K4)/6

#     return state_vector_slope

# def cyclic_pursuit(allcfs, timeHelper, rate, K, velocity, cfs, n):
#     while not timeHelper.isShutdown():
#         dcfs = rk4(crazyflie_dynamics, cfs, K, velocity, dt=1/rate)
#         cfs = cfs + (1/rate)*dcfs
#         cfs = adjust_angle(cfs, n)
#         cfs = np.reshape(cfs, (n, 3))

#         for cf,i in enumerate(allcfs):
#             pos = np.array(cfs[i, 0], cfs[i, 1], cf.initialPosition[2])
#             cf.cmdPosition(pos, cfs[i,2])
    
#         timeHelper.sleepForRate(rate)
#         cfs = cfs.flatten()

# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     allcfs = swarm.allcfs

#     rate = 100.0 # Hz
#     Z = 1.0 
#     K = 0.25
    
#     velocity = 1
#     initial_thetas = np.array([0.0 for cf in allcfs])
#     n = len(initial_thetas)

#     xs = np.array([allcfs.crazyflies[i].initialPosition[0] for i in range(len(initial_thetas))])
#     ys = np.array([allcfs.crazyflies[i].initialPosition[1] for i in range(len(initial_thetas))])
#     cfs = np.vstack((np.vstack((xs, ys)), initial_thetas)).T
#     cfs = cfs.flatten()

#     targetHeights = np.array([Z + i for i in range(n)])

#     allcfs.takeoff(targetHeights, duration = 2.0)
#     timeHelper.sleep(2.5)

#     cyclic_pursuit(allcfs.crazyflies, timeHelper, rate, K, velocity, cfs, n)
    
#     for cf in allcfs.crazyflies:
#         cf.notifySetpointsStop()
#         cf.land(targetHeight=0.03, duration=Z+1.0)
#         Z -= 1.0
#     timeHelper.sleep(Z+2.0)

