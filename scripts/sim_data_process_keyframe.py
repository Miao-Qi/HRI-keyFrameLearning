#!/usr/bin/env python

import rospy
import sys
import math
import time
import itertools

import numpy                 as np
import matplotlib.pyplot     as plt
import matplotlib            as mpl

from scipy                   import linalg
from numpy                   import genfromtxt
from sklearn.cluster         import KMeans
from sklearn                 import mixture
from geometry_msgs.msg       import Twist
from turtlesim.msg           import Pose
from std_msgs.msg            import String
from nav_msgs.msg            import Odometry

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
group_counter = dict((el,0) for el in index)
group_theta = dict((el,0) for el in index)
group_ts = dict((el,0) for el in index)
group_theta_ts_max = dict((el,0) for el in index)
group_ts_max = dict((el,-9999) for el in index)
kmeans = KMeans(random_state=0)
gmm_model = mixture.GaussianMixture(covariance_type='full')
last_group = 0
last_state = 0
previous_state = dict((el,0) for el in index)
f = open('data_process-' + str(time.time()) + '.txt', 'w+')
last_x = -999999
last_y = -999999

color_iter = itertools.cycle(['navy', 'c', 'cornflowerblue', 'gold',
                              'darkorange'])

def plot_results(X, Y, means, covariances, index, title):
    splot = plt.subplot(1, 1, 1 + index)
    for i, (mean, covar, color) in enumerate(zip(
            means, covariances, color_iter)):
        v, w = linalg.eigh(covar)
        v = 2. * np.sqrt(2.) * np.sqrt(v)
        u = w[0] / linalg.norm(w[0])
        # as the DP will not use every component it has access to
        # unless it needs it, we shouldn't plot the redundant
        # components.
        if not np.any(Y == i):
            continue
        plt.scatter(X[Y == i, 0], X[Y == i, 1], .8, color=color)

        # Plot an ellipse to show the Gaussian component
        angle = np.arctan(u[1] / u[0])
        angle = 180. * angle / np.pi  # convert to degrees
        ell = mpl.patches.Ellipse(mean, v[0], v[1], 180. + angle, color=color)
        ell.set_clip_box(splot.bbox)
        ell.set_alpha(0.5)
        splot.add_artist(ell)

    plt.xlim(0., 10.)
    plt.ylim(0., 16.)
    plt.title(title)
    plt.xticks(())
    plt.yticks(())


def init_model():
    global index
    global group_counter
    global group_theta
    global kmeans
    global gmm_model
    global last_x
    global last_y

    # result storage
    index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
    group_counter = dict((el,0) for el in index)
    group_theta = dict((el,0) for el in index)

    # data preprocess
    # read from keyframe record
    raw_data = genfromtxt('path-2016-12-07_13:19:37-keyframe.txt', delimiter=',')
    raw_data_set = np.array(raw_data)

    # separate input array
    pos, timestamps, dirc = np.split(raw_data, [2, 3], axis = 1)

    # sample generation parameters
    STEP = 0.015
    CORNER_CLUSTER_SIZE = 40
    np.random.seed(0)
    raw_m, raw_n = raw_data_set.shape

    # final result for sample generation
    X = np.zeros((0, 2))
    Theta = np.zeros((0, 1))
    timestamps = np.zeros((0, 1))

    # total timestamp
    global_timestamp = 0

    # iterate the input file to generate sample with noise
    for line in range(raw_m - 1):
        # get parameters from the key points
        start_x = raw_data_set.item(((line)*raw_n) )
        start_y = raw_data_set.item(((line)*raw_n) + 1)
        start_time = raw_data_set.item(((line)*raw_n) + 2)

        end_x = raw_data_set.item(((line + 1)*raw_n))
        end_y = raw_data_set.item(((line + 1)*raw_n) + 1)
        end_time = raw_data_set.item(((line + 1)*raw_n) + 2)
        line_theta = math.atan2((end_y - start_y),(end_x - start_x))

        # form corner cluster
        CONER = np.zeros((CORNER_CLUSTER_SIZE, 2))
        CONER_thetas = np.zeros((CORNER_CLUSTER_SIZE, 1))
        CONER_timestamp = np.zeros((CORNER_CLUSTER_SIZE, 1))
        for i in range(CORNER_CLUSTER_SIZE):
            CONER[i, 0] = start_x + np.random.normal(0, 0.008)
            CONER[i, 1] = start_y + np.random.normal(0, 0.008)
            CONER_thetas[i] = line_theta
            CONER_timestamp[i] = global_timestamp
            global_timestamp = global_timestamp + STEP

        # form line samples
        n_samples = (end_time - start_time) / STEP
        x_step = (end_x - start_x) / n_samples
        y_step = (end_y - start_y) / n_samples
        LINE = np.zeros((n_samples, 2))
        LINE_thetas = np.zeros((n_samples, 1))
        LINE_timestamp = np.zeros((n_samples, 1))
        if start_x != end_x :
            ratio = (end_y - start_y) / (end_x - start_x)
        else:
            ratio = 1

        # form line samples
        for i in range(LINE.shape[0]):
            LINE[i, 0] = start_x + (x_step * i) + np.random.normal(0, 0.008)
            LINE[i, 1] = start_y + (y_step * i) + np.random.normal(0, 0.008) * ratio
            LINE_thetas[i] = line_theta
            LINE_timestamp[i] = global_timestamp
            global_timestamp = global_timestamp + STEP

        # concatenate into one array
        pos_coner_line = np.concatenate((CONER, LINE), axis=0)
        pos_coner_line_theta = np.concatenate((CONER_thetas, LINE_thetas), axis=0)
        pos_coner_line_timestamp = np.concatenate((CONER_timestamp, LINE_timestamp), axis=0)
        X = np.concatenate((X, pos_coner_line), axis=0)
        Theta = np.concatenate((Theta, pos_coner_line_theta), axis=0)
        timestamps = np.concatenate((timestamps, pos_coner_line_timestamp), axis=0)

    dirc = Theta

    k = 16
    # get init means for Gaussian Mixture Model
    kmeans.set_params(n_clusters = k)
    kmeans.fit(X)

    # fit a Gaussian Mixture Model
    # Defaul init_params: kmeans
    gmm_model.set_params(n_components = k)
    gmm_model.fit(X)

    # cluster the origin data to get group orientation
    clustring_result = gmm_model.predict(X)

    # plot results
    plot_results(X, clustring_result, gmm_model.means_, gmm_model.covariances_, 0, 'Expectation-maximization')
    plt.show()

    # get timestamp and direction for each cluster
    index_counter = 0

    # sum up all the timestamps/directions inside one cluster
    for x in np.nditer(clustring_result):
        group_number = x.item(0)
        group_counter[group_number] = group_counter[group_number] + 1
        group_theta[group_number] = group_theta[group_number] + dirc.item(index_counter)
        group_ts[group_number] = group_ts[group_number] + timestamps.item(index_counter)
        if group_ts_max[group_number] < timestamps.item(index_counter):
            group_theta_ts_max[group_number] = dirc.item(index_counter)
            group_ts_max[group_number] = timestamps.item(index_counter)
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
            if abs(group_theta_ts_max[key] - value) > 0.3:
                value = group_theta_ts_max[key]
        group_theta[key] = value
        f.write("cluster theta: " + str(value) + "\n")
    f.write("\n")
    f.write("\n")

def myCallback(data):
    global SPD
    global group_theta
    global gmm_model
    global last_state
    global last_group
    global pub
    global last_x
    global last_y

    PosX = data.x
    PosY = data.y
    PosT = data.theta

    pos_group = gmm_model.predict([PosX, PosY]).reshape(1, -1).item(0)
    new_ts = group_ts[pos_group]
    last_ts = group_ts[last_group]

    if last_group == 0:
        last_ts = 0
    # choose group with timestamp checking
    if new_ts < last_ts:
        pos_group = last_group
    else:
        last_group = pos_group

    # get new target theta
    new_theta = group_theta[pos_group]

    newTwist = createTwist(0.01, new_theta - data.theta)

    # in-state checking
    if last_state == pos_group:
        newTwist = createTwist(0.5, 0)
    else:
        if abs(new_theta - data.theta) < 0.02:
            last_state = pos_group

    distance = (data.x - last_x) * (data.x - last_x) + (data.y - last_y) * (data.y - last_y)
    if distance < 0.1:
        newTwist = createTwist(0.0, 0.0)

    print ("Publish new twist")    
    pub.publish(newTwist)

    print (rospy.get_caller_id() + ' heard currentPos' + "\n")
    print (data)
    print ("\n\n")
    print ('Destination position: (' + str(last_x) + ', ' + str(last_y) +")\n" )
    print ('Distance to Destination: ' + str(distance) +"\n" )
    print ("In group: " + str(pos_group) + ' and make head turn to: ' + str(new_theta) + "\n")
    print ("\n\n")

    f.write(rospy.get_caller_id() + ' heard currentPos' + "\n")
    f.write('x: ' + str(data.x) + ',' + 'y: ' + str(data.y) + ',' + 'theta: ' +  str(data.theta) +"\n" )
    f.write('Destination positino: (' + str(last_x) + ', ' + str(last_y) +")\n" )
    f.write('Distance to Destination: ' + str(distance) +"\n" )
    f.write("In group: " + str(pos_group) + ' and make head turn to: ' + str(new_theta) + "\n")
    f.write("\n")
    f.write("\n")
 
def createTwist(lx, az): 
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = 0
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = az
    return newTwist

def DataProcess():
    init_model()
    rospy.init_node('data_process', anonymous=True)

    rospy.Subscriber('turtle1/pose', Pose, myCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    f.close()

if __name__ == '__main__':
    DataProcess()

