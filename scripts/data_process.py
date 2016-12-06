#!/usr/bin/env python

import rospy
import sys
import math
import time
import numpy                 as np

from numpy                   import genfromtxt
from sklearn.cluster         import KMeans
from sklearn                 import mixture
from geometry_msgs.msg       import Twist
from turtlesim.msg           import Pose
from std_msgs.msg            import String
from nav_msgs.msg            import Odometry

# ----------------------------------------------------------------------
# Configuration Constants 
# ----------------------------------------------------------------------
# TODO
# Sphero speed 
SPD = 50 

# TODO
# For simulation ---------------------------------------------------
# pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# For Sphero -------------------------------------------------------
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
group_counter = dict((el,0) for el in index)
group_theta = dict((el,0) for el in index)
group_ts = dict((el,0) for el in index)
kmeans = KMeans(random_state=0)
gmm_model = mixture.GaussianMixture(covariance_type='full')
last_group = 0
last_state = 0
previous_state = dict((el,0) for el in index)
f = open('./data_file/data_process-' + str(time.time()) + '.txt', 'w+')

def init_model():
    global index
    global group_counter
    global group_theta
    global kmeans
    global gmm_model

    # result storage
    index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
    group_counter = dict((el,0) for el in index)
    group_theta = dict((el,0) for el in index)

    # data preprocess
    my_data = genfromtxt('./data_file/path-2016-12-06_10:18:46.txt', delimiter=',')

    dataSet = np.array(my_data)
    pos, timestamps, dirc = np.split(my_data, [2, 3], axis = 1)

    # TODO: cross-validation
    # issue: unsupervised learning -> how to get label
    # basic trade-off between # of clusters:
    # more clusters: accrate but some critical states may be missed and lost the entire trace
    # less clusters: easy to move forward, but in less accurate way
    # k: number of clusters
    k = 16
    # get init means for Gaussian Mixture Model
    kmeans.set_params(n_clusters = k)
    kmeans.fit(pos)

    # fit a Gaussian Mixture Model
    gmm_model.set_params(n_components = k, means_init=kmeans.cluster_centers_)
    gmm_model.fit(pos)

    # cluster the origin data to get group orientation
    clustring_result = gmm_model.predict(pos)

    # get timestamp and direction for each cluster
    index_counter = 0

    # sum up all the timestamps/directions inside one cluster
    for x in np.nditer(clustring_result):
        group_number = x.item(0)
        group_counter[group_number] = group_counter[group_number] + 1
        group_theta[group_number] = group_theta[group_number] + dirc.item(index_counter)
        group_ts[group_number] = group_ts[group_number] + timestamps.item(index_counter)
        index_counter = index_counter + 1

    # get the average timestamp for each cluster
    f.write("clustering info: \n") 
    f.write("cluster timestamp: \n") 
    for key, value in group_ts.iteritems():
        f.write("cluster number: " + str(key) + "\n")
        f.write("# of elements in cluster: " + str(group_counter[key]) + "\n")
        if group_counter[key] == 0:
            value = value
        else:
            value = value / group_counter[key]
        group_ts[key] = value
        f.write("cluster timestamp: " + str(value) + "\n")

    # get the average direction for each cluster
    f.write("\n\n") 
    f.write("cluster theta: \n") 
    for key, value in group_theta.iteritems():
        f.write("cluster number: " + str(key) + "\n")
        f.write("# of elements in cluster: " + str(group_counter[key]) + "\n")
        if group_counter[key] == 0:
            value = value
        else:
            value = value / group_counter[key]
        group_theta[key] = value
        f.write("cluster theta: " + str(value) + "\n")
    f.write("\n")
    f.write("\n")


def myCallback(data):
    global SPD
    global group_theta
    global group_ts
    global gmm_model
    global last_state
    global last_group
    global pub

    # TODO
    # For simulation
    # PosX = data.x
    # PosY = data.y
    # PosT = data.theta
    # For Sphero 
    PosX = data.pose.pose.position.x
    PosY = data.pose.pose.position.y
    PosT = math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x)

    pos_group = gmm_model.predict([PosX, PosY]).reshape(1, -1).item(0)
    new_ts = group_ts[pos_group]
    last_ts = group_ts[last_group]

    if last_group == 0:
        last_ts = 0
    # choose group with timestamp checking
    # TODO: improve state transit
    if new_ts < last_ts:
        pos_group = last_group
    else:
        last_group = pos_group

    # get new target theta
    new_theta = group_theta[pos_group]

    newTwist = createTwist(SPD * math.cos(new_theta), SPD * math.sin(new_theta))  

    # TODO: improve in-state control
    # easy-fix if the robot support immediate direction change
    # in-state checking
    if last_state == pos_group:
        newTwist = createTwist(SPD * math.cos(PosT), SPD * math.sin(PosT))  
    else:
        if abs(new_theta - PosT) < 0.1:
            last_state = pos_group

    print ("Publish new twist")    
    pub.publish(newTwist)

    # TODO: add stop point

    print (rospy.get_caller_id() + ' heard currentPos' + "\n")
    print (data)
    print ("\n\n")
    print ("In group: " + str(pos_group) + ' and make head turn to: ' + str(new_theta) + "\n")
    print ("\n\n")

    f.write(rospy.get_caller_id() + ' heard currentPos' + "\n")
    f.write('x: ' + str(PosX) + ',' + 'y: ' + str(PosY) + ',' + 'theta: ' +  str(PosT) +"\n" )
    f.write("In group: " + str(pos_group) + ' and make head turn to: ' + str(new_theta) + "\n")
    f.write("\n")
    f.write("\n")

def createTwist(lx, ly): 
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = ly
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = 0
    return newTwist

def DataProcess():
    init_model()
    rospy.init_node('data_process', anonymous=True)

    # TODO
    # For simulation
    # rospy.Subscriber('turtle1/pose', Pose, myCallback)
    # For Sphero 
    rospy.Subscriber('odom', Odometry, myCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    DataProcess()

