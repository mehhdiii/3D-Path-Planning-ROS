#! /usr/bin/env python

import numpy as np
import time


def potential_field(obstacles, start, goal):
    '''
    start: np array of [x, y, z, phi]
    goal: np array of [x, y, z, phi]
    
    obstacles: np matrix of the form: 
    [x, y, z, radius; 
     x2, y2, z2, radius2;
     ...;
     ...]
    '''
    start_time = time.time()
    way_points = np.array([start])
    # way_points = np.vstack([way_points, start])

    F_attr = 0.5
    F_rep = 0.5
    omega = 1
    rho0 = 3
    current_pose = np.copy(start)
    mu = 0.11
    # print('goal=', goal)
    dgoalstar = np.linalg.norm(start[0:3]-goal[0:3])
    ITERa = 1000
    i = 0
    while True:
        i+=1
        x = current_pose[0]
        y = current_pose[1]
        z = current_pose[2]
        phi = current_pose[3]
        # print('x, y, z =', x, y, z, '\n')

        #calculate distance to goal: 
        dgoal = np.linalg.norm(current_pose[:3] - goal[:3])
        # print('dis to goal: ', dgoal)
        #calculate attractive force: 
        if (dgoal<dgoalstar):
            x_attr = F_attr*(x - goal[0])
            y_attr = F_attr*(y - goal[1])
            z_attr = F_attr*(z-goal[2])
        else:
            x_attr = dgoalstar*F_attr*(x - goal[0])/dgoal
            y_attr = dgoalstar*F_attr*(y - goal[1])/dgoal
            z_attr = dgoalstar*F_attr*(z - goal[2])/dgoal
        
        x_rep = 0.0
        y_rep = 0.0
        z_rep = 0.0 

        #find repulsive force: 
        for indx in range(1, len(obstacles)):
            obsx = obstacles[indx, 0] 
            obsy = obstacles[indx, 1] 
            obsz = obstacles[indx, 2] 
            obsr = obstacles[indx, 3] 

            phi = np.arctan2(obsy-y,  obsx-x)
            disxy = np.linalg.norm(np.array([obsx, obsy]) - np.array([x, y]))
            theta = np.arctan2(disxy, np.abs(obsz-z))
            obs_distance = np.linalg.norm(np.array([obsx, obsy, obsz])- np.array([x, y, z]) )
            
            #distance to reach obstacle:
            di = obs_distance - obsr

            #closest point coordinates: 
            xCl = x + di*np.cos(phi)*np.sin(theta)
            yCl = y + di*np.sin(phi)*np.sin(theta)
            zCl = z + di*np.cos(theta)

            #if obstacle is near enough, then add its effects: 
            if di<=rho0:
                # xnew = x+obs_distance*np.cos(phi)*np.sin(theta)
                # ynew = y+obs_distance*np.sin(phi)*np.sin(theta)
                # znew = z+obs_distance*np.cos(theta)

                x_rep += F_rep*(1.0/rho0-1.0/di)*(x-xCl)/(di**3)
                y_rep += F_rep*(1.0/rho0-1.0/di)*(y-yCl)/(di**3)
                z_rep += F_rep*(1.0/rho0-1.0/di)*(z-zCl)/(di**3)
        
        xTotal = x_attr+x_rep
        yTotal = y_attr+y_rep
        zTotal = z_attr+z_rep
        gradpot = np.array([-xTotal, -yTotal, -zTotal])
        print(xTotal, yTotal, zTotal)
        current_pose[0] += mu*gradpot[0]
        current_pose[1] += mu*gradpot[1]
        # current_pose[2] += mu*gradpot[2]
        # print(current_pose)
        # print('xrep, xattr = ', x_rep, x_attr)
        #calculate phi
        current_pose[3]  = phi
        # print(start)
        # print(np.linalg.norm(current_pose[:3] - goal[:3]))
        way_points = np.vstack([way_points, current_pose])
        # print(np.linalg.norm(current_pose[:3] - goal[:3]))
        if gradpot[0] < 0.00001 and gradpot[1] < 0.00000001:
            break
    
    end_time = time.time()
    runtime = end_time - start_time
    
    return way_points
    
# 2D potential field
# def potential_field(obstacles, start, goal):
#     '''
#     start: np array of [x, y, z, phi]
#     goal: np array of [x, y, z, phi]
    
#     obstacles: np matrix of the form: 
#     [x, y, z, radius; 
#      x2, y2, z2, radius2;
#      ...;
#      ...]
#     '''
#     start_time = time.time()
#     way_points = np.array([start])
#     # way_points = np.vstack([way_points, start])

#     F_attr = 0.5
#     F_rep = 0.5
#     omega = 1
#     rho0 = 0.1
#     current_pose = np.copy(start)
#     mu = 0.11
#     # print('goal=', goal)
#     dgoalstar = np.linalg.norm(start[0:2]-goal[0:2])
#     ITERa = 1000
#     i = 0
#     while True:
#         i+=1
#         x = current_pose[0]
#         y = current_pose[1]
#         z = current_pose[2]
#         phi = current_pose[3]
#         # print('x, y, z =', x, y, z, '\n')

#         #calculate distance to goal: 
#         dgoal = np.linalg.norm(current_pose[:2] - goal[:2])
#         # print('dis to goal: ', dgoal)
#         #calculate attractive force: 
#         if (dgoal<dgoalstar):
#             x_attr = F_attr*(x - goal[0])
#             y_attr = F_attr*(y - goal[1])
#             # z_attr = F_attr*(z-goal[2])
#         else:
#             x_attr = dgoalstar*F_attr*(x - goal[0])/dgoal
#             y_attr = dgoalstar*F_attr*(y - goal[1])/dgoal
#             # z_attr = dgoalstar*F_attr*(z - goal[2])/dgoal
        
#         x_rep = 0.0
#         y_rep = 0.0
#         # z_rep = 0.0 

#         #find repulsive force: 
#         for indx in range(1, len(obstacles)):
#             obsx = obstacles[indx, 0] 
#             obsy = obstacles[indx, 1] 
#             obsz = obstacles[indx, 2] 
#             obsr = obstacles[indx, 3] 

#             phi = np.arctan2(obsy-y,  obsx-x)
#             disxy = np.linalg.norm(np.array([obsx, obsy]) - np.array([x, y]))
#             # theta = np.arctan2(disxy, np.abs(obsz-z))
#             obs_distance = np.linalg.norm(np.array([obsx, obsy])- np.array([x, y]) )
            
#             #distance to reach obstacle:
#             di = obs_distance - obsr

#             #closest point coordinates: 
#             xCl = x + di*np.cos(phi)
#             yCl = y + di*np.sin(phi)
#             # zCl = z + di*np.cos(theta)

#             #if obstacle is near enough, then add its effects: 
#             if di<=rho0:
#                 # xnew = x+obs_distance*np.cos(phi)*np.sin(theta)
#                 # ynew = y+obs_distance*np.sin(phi)*np.sin(theta)
#                 # znew = z+obs_distance*np.cos(theta)

#                 x_rep += F_rep*(1/rho0-1/di)*(x-xCl)/(di**3)
#                 y_rep += F_rep*(1/rho0-1/di)*(y-yCl)/(di**3)
#                 # z_rep += F_rep*(1/rho0-1/di)*(z-zCl)/(di**3)
        
#         xTotal = x_attr+x_rep
#         yTotal = y_attr+y_rep
#         # zTotal = z_attr+z_rep
#         gradpot = np.array([-xTotal, -yTotal])
#         print(xTotal, yTotal)
#         current_pose[0] += mu*gradpot[0]
#         current_pose[1] += mu*gradpot[1]
#         # current_pose[2] += mu*gradpot[2]
#         # print(current_pose)
#         # print('xrep, xattr = ', x_rep, x_attr)
#         #calculate phi
#         current_pose[3]  = phi
#         # print(start)
#         # print(np.linalg.norm(current_pose[:3] - goal[:3]))
#         way_points = np.vstack([way_points, current_pose])
#         print(np.linalg.norm(current_pose[:2] - goal[:2]))
#         if np.linalg.norm(current_pose[:2] - goal[:2]) < 2 :
#             break
    
#     end_time = time.time()
#     runtime = end_time - start_time
    
#     return way_points

# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d


# def test():
#     fig = plt.figure()
#     ax = fig.gca(projection='3d')
#     ax.set_aspect("equal")

#     start = np.array([3.0, -9.0, 0.5, 0.0])
#     goal = np.array([9.0-2,6.0-2,0.5, 0.0])
#     obs = {'unit_box_19': [6.510105999911615, 14.11810246300535, 0.4816117021393038, 3], 'Workshop': [0.0, 0.0, 0.0, 3], 'unit_box_13': [-2.0723700000696454, 7.67139999996255, 0.6310899999769928, 3], 'unit_box_12': [-6.984011874291508, 7.786550678555996, 0.6717552130645029, 3], 'unit_box_11': [6.949896094985431, -8.244770007432976, 1.4084150321643334, 3], 'unit_box_10': [-2.78169004086536, -7.236039591991761, 0.4999952658892948, 3], 'unit_box_17': [-1.142743954868383, -0.001131999999999063, 0.49426892632166897, 3], 'unit_box_16': [7.984929999996768, -2.9480900000071584, 0.49999999999018374, 3], 'unit_box_15': [7.989894303702401, -1.586219347139821, 1.0934650155788357, 3], 'unit_box_14': [7.228823122645085, 1.4886698344249487, 0.4999909847055377, 3], 'unit_box': [2.4337964117312074, 7.83558603477884, 1.0529609802425326, 3], 'ground_plane': [0.0, 0.0, 0.0, 3], 'washer': [3.987690002000069, 3.648389995000043, -0.0010001297757312032, 3], 'Untitled': [6.342669999999999, 3.7532000000000005, 0.0, 3], 'unit_box_22': [10.2172014235736, 6.058075457224646, 1.041405071561681, 3], 'unit_box_20': [8.167310004845502, 11.91933899993823, 0.4903299999886588, 3], 'unit_box_21': [11.3332245515462, 7.042912375159528, 0.4795569802569625, 3], 'unit_box_9': [8.118099142058465, -5.954190918495212, 1.1306115684244764, 3], 'unit_box_8': [-6.232100000002085, 6.63190999999197, 0.49999999999018085, 3], 'unit_box_3': [-7.436193700245216, 0.46081183019674016, 0.49999537446528736, 3], 'unit_box_2': [-7.4736599332070766, 4.172744568067995, 0.4999178025525813, 3], 'unit_box_1': [-3.9919352087911, 7.606427760015934, 1.332034949182781, 3], 'unit_box_0': [4.579091173578661, 7.820409646311897, 0.49999489116894685, 3], 'unit_box_7': [1.3383275483507033, -8.116782037653271, 0.7834249672582764, 3], 'unit_box_6': [7.972369999996768, 4.67573999999262, 0.49999999999018374, 3], 'unit_box_5': [-7.514743932981141, -6.62544089596216, 0.49999532129780405, 3], 'unit_box_4': [-7.4843699998820545, -3.186385281910996, 0.49997638937463346, 3], 'RAs_room': [12.04953, 12.048200000000001, 0.0, 3], 'unit_box_18': [4.972430999944121, 11.145721999973114, 0.499999999983468, 3]}
#     for o in obs.keys():
#         obs[o][3] = 1
#     obstacles = np.array(obs.values())
#     goals = potential_field(obstacles, start, goal)
#     ax.plot([start[0]], [start[1]], [start[2]], 'ro')
#     ax.plot([goal[0]], [goal[1]], [goal[2]], 'go')
#     ax.plot(goals[:, 0], goals[:, 1], goals[:, 2], linestyle='-',
#      linewidth=2)
#     for ob in obstacles:
#         u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#         r = ob[3]
#         x = ob[0]+r*np.cos(u)*np.sin(v)
#         y = ob[1]+r*np.sin(u)*np.sin(v)
#         z = ob[2]+r*np.cos(v)
#         ax.plot_wireframe(x, y, z, color="k")
#         # ax.plot(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], 'b*')
    
#     ax.set_xlabel('X axis')
#     ax.set_ylabel('Y axis')
#     ax.set_zlabel('Z axis')
#     plt.show()

# test()




   
















