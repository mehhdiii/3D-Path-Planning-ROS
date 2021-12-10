#! /usr/bin/env python

import numpy as np
import time

def potential_field(obstacles, start, goal):
    start_time = time.time()
    way_points = np.array([start])
    # way_points = np.vstack([way_points, start])

    F_attr = 0.5
    F_rep = 0.5
    omega = 1
    rho0 = 1
    current_pose = np.array(start)
    mu = 0.11
    # print('goal=', goal)
    dgoalstar = np.linalg.norm(start-goal)
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
            z_attr = dgoalstar*F_attr*(z-goal[2])/dgoal
        
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

                x_rep += F_rep*(1/rho0-1/di)*(x-xCl)/(di**3)
                y_rep += F_rep*(1/rho0-1/di)*(y-yCl)/(di**3)
                z_rep += F_rep*(1/rho0-1/di)*(z-zCl)/(di**3)

        xTotal = x_attr+x_rep
        yTotal = y_attr+y_rep
        zTotal = z_attr+z_rep
        gradpot = np.array([-xTotal, -yTotal, -zTotal])
        current_pose[0] += mu*gradpot[0]
        current_pose[1] += mu*gradpot[1]
        current_pose[2] += mu*gradpot[2]
        # print('xrep, xattr = ', x_rep, x_attr)
        #calculate phi
        current_pose[3]  = 0
        
        way_points = np.vstack([way_points, current_pose])
        
        if np.linalg.norm(current_pose[:3] - goal[:3]) < 0.1 or i>ITERa:
            break
    
    end_time = time.time()
    runtime = end_time - start_time
    
    return way_points
    









   
















