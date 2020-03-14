#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 24 16:59:06 2019

@author: kartik
"""
from mj_kinova_maml import Kinova_MJ
import sys
import time
import random
if  '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path : sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os

def main(args):
    
    sim = Kinova_MJ()
    root = '/home/kartik/arm_tracking/meta_learning/MAML-Pytorch-arm/data_13'
    all_poses = []
    for task in range(args.task):
        p = np.random.rand()*0.2 - 0.1      #gives random float between [-0.1,0.1)    
        q = np.random.rand()*4 - 2          #gives random float between [-2,2)
        directory = os.path.join(root,str(task))
        if not os.path.exists(directory):
            os.makedirs(directory)
        task_poses = []
        for sample in range(args.samples):
            true_pose_arm = np.random.rand(7)*(2*6.28319) -  6.28319
            true_pose = list(true_pose_arm )+[2,2,2]
            true_pose_org = true_pose.copy()
            true_pose[6] = p*true_pose[6] + q
            image = sim.run_mujoco(true_pose)
            plt.imsave(os.path.join(root,str(task),str(sample)),image)
            task_poses.append([sample]+true_pose_org+[true_pose[6]]) 

            all_poses.append([task]+[sample]+true_pose_org+[true_pose[6]])               
            if (sample%500)==499:
                print('Task: {} Sample: {}'.format(task,sample))
                np.savetxt(os.path.join(root,str(task),'jointInfo.csv'),np.array(task_poses))
        np.savetxt(os.path.join(root,'allJointInfo.csv'),np.array(all_poses))
        

if __name__ == '__main__' :
    
    argparse = argparse.ArgumentParser()
    argparse.add_argument('--task', type=int, help = 'total number of tasks', default = 10000)
    argparse.add_argument('--samples', type = int, help = 'number of samples per task', default = 1000)
    
    args = argparse.parse_args()
    
    main(args)
