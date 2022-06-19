import numpy as np

current_xy = np.array([1, 2])
waypoints = [[2,3],[3,4],[1,2]]
print(current_xy - np.array(waypoints)[:, :2])
print(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))

crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))
print(np.argmin(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1)))


# steer_expect = 4
# print((steer_expect+np.pi)%(2*np.pi)-np.pi)
# if steer_expect > np.pi:
#     steer_expect -= 2 * np.pi
# if steer_expect < - np.pi:
#     steer_expect += 2 * np.pi
# print(steer_expect)