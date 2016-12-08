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

# ----------------------------------------------------------------------
# Configuration Constants 
# ----------------------------------------------------------------------
# Sphero speed 
SPD = 60 
# Position error threshold 
THR = 0.05

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
k = 16
group_counter = dict((el,0) for el in range(k))
group_x = dict((el,0) for el in range(k))
group_y = dict((el,0) for el in range(k))
group_theta = dict((el,0) for el in range(k))
group_ts = dict((el,0) for el in range(k))
group_theta_ts_max = dict((el,0) for el in range(k))
group_ts_max = dict((el,-9999) for el in range(k))
kmeans = KMeans(random_state=0)
gmm_model = mixture.GaussianMixture(covariance_type='full')
# dpgmm_model = mixture.BayesianGaussianMixture(n_components=k, covariance_type = 'full', weight_concentration_prior = 1e+3,
# weight_concentration_prior_type = 'dirichlet_process', 
# mean_precision_prior = 1e-2, covariance_prior=1e0 * np.eye(2),
# init_params = 'kmeans', random_state=2)
last_group = 0
last_state = 0
previous_state = dict((el,0) for el in range(k))
f = open('./data/data_process-' + str(time.time()) + '.txt', 'w+')

curTheta = 0
curTwist = Twist()  

autoPositioning = True

pos = []
timestamps = [] 
dirc = []

last_x = -999999
last_y = -999999

mean_pos = []
counter = 0
slept = False

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

    plt.xlim(0., 20.)
    plt.ylim(0., 20.)
    plt.title(title)
    plt.xticks(())
    plt.yticks(())

# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------
def createTwist(lx, ly): 
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = ly
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = 0
    return newTwist

def CalcHeading (src, dst) : 
  height = dst[1] - src[1] 
  base   = dst[0] - src[0] 
  return math.atan2(height, base) 

def IsCloseEnough (src, dst) : 
  global THR
  height = dst[1] - src[1] 
  base   = dst[0] - src[0] 
  dist   = math.sqrt(math.pow(height, 2) + math.pow(base, 2))

  return (dist < THR) 

# ----------------------------------------------------------------------
# Callback functions
# ----------------------------------------------------------------------

def init_model():
    global index
    global group_x
    global group_y
    global group_counter
    global group_theta
    global kmeans
    global gmm_model
    global pos 
    global timestamps 
    global dirc 
    global last_x
    global last_y
    global k

    # data preprocess
    my_data = genfromtxt('./data/frames.txt', delimiter=',')

    dataSet = np.array(my_data)
    pos, timestamps, dirc = np.split(my_data, [2, 3], axis = 1)

    m, n = pos.shape
    last = pos[m - 1,[0,1]] 
    last_x = last.item(0)
    last_y = last.item(1)
    # get the average timestamp for each cluster
    f.write("last position info: \n") 
    f.write("postion in file: " + str(last) + "\n") 
    f.write("last_x: " + str(last_x) + "last_y: " + str(last_y) + "\n") 

    kmeans.set_params(n_clusters = k)
    kmeans.fit(pos)

    # fit a Gaussian Mixture Model
    # gmm_model.set_params(n_components = k, means_init=kmeans.cluster_centers_)
    # Defaul init_params: kmeans
    gmm_model.set_params(n_components = k)
    gmm_model.fit(pos)
    # dpgmm_model.fit(pos)

    # cluster the origin data to get group orientation
    clustring_result = gmm_model.predict(pos)
    # clustring_result = dpgmm_model.predict(pos)

    # plot result
    # plot_results(pos, clustring_result, dpgmm_model.means_, dpgmm_model.covariances_, 0, 'Expectation-maximization')
    plot_results(pos, clustring_result, gmm_model.means_, gmm_model.covariances_, 0, 'Expectation-maximization')
    plt.show()

    # get timestamp and direction for each cluster
    index_counter = 0

    # sum up all the timestamps/positions inside one cluster
    for x in np.nditer(clustring_result):
        group_number = x.item(0)
        group_counter[group_number] = group_counter[group_number] + 1
        group_theta[group_number] = group_theta[group_number] + dirc.item(index_counter)
        group_ts[group_number] = group_ts[group_number] + timestamps.item(index_counter)
        group_x[group_number] = group_x[group_number] + pos[index_counter][0] 
        group_y[group_number] = group_y[group_number] + pos[index_counter][1] 
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
            # if the maximum theta and the average theta differs a lot, 
            # the cluster should be at a turning corner 
            # may not be true for Sphero, since there are usually multiple 
            # clusters formed around each corner 
            # if abs(group_theta_ts_max[key] - value) > 0.3:
            #     value = group_theta_ts_max[key]
        group_ts[key] = value
        f.write("cluster timestamp: " + str(value) + "\n")

    # get the average position for each cluster
    f.write("\n\n") 
    f.write("cluster mean position x: \n") 
    for key, value in group_x.iteritems():
        f.write("cluster number: " + str(key) + "\n")
        f.write("# of elements in cluster: " + str(group_counter[key]) + "\n")
        if group_counter[key] == 0:
            value = value
        else:
            value = value / group_counter[key]
        group_x[key] = value
        f.write("cluster mean position x: " + str(value) + "\n")

    f.write("\n\n") 
    f.write("cluster mean position y: \n") 
    for key, value in group_y.iteritems():
        f.write("cluster number: " + str(key) + "\n")
        f.write("# of elements in cluster: " + str(group_counter[key]) + "\n")
        if group_counter[key] == 0:
            value = value
        else:
            value = value / group_counter[key]
        group_y[key] = value
        f.write("cluster mean position y: " + str(value) + "\n")

    # print out the mean position of each clustere 
    for key, value in group_x.iteritems() : 
      print ( "Mean of cluster # " + str(key) + ": " + str(value) + ", " + str(group_y[key]) ) 
      print ( "Time of cluster # " + str(key) + ": " + str(group_ts[key]) ) 

    # get the direction of each cluster based on mean positions 
    # f.write("\n\n") 
    # f.write("cluster theta: \n") 
    # for key, value in group_theta.iteritems():
    #     f.write("cluster number: " + str(key) + "\n")

    #     # find the next key based on time stamps 
    #     next_key = 0
    #     next_key_found = False
    #     for k, v in group_ts.iteritems () : 
    #       if v > group_ts[key] and v < group_ts[next_key] : 
    #         next_key = k
    #         next_key_found = True

    #     if next_key_found : 
    #       print ( str( key )  + " --> " + str( next_key ) ) 
    #       group_theta[key] = CalcHeading ( [group_x[key], group_y[key]], [group_x[next_key], group_y[next_key]])  
    #     else : 
    #       group_theta[key] = CalcHeading ( [group_x[key], group_y[key]], [pos[-1][0], pos[-1][1]] ) 

    #     f.write("cluster theta: " + str(value) + "\n")

    # get an array of cluster mean points, sorted by timestamps
    # print ( group_ts ) 
    global mean_pos 
    group_ts_tmp = group_ts
    min_ts   = -1 
    min_key  = -1

    while len(mean_pos) < k : 

      next_key = group_ts_tmp.keys()[0]
      for key, value in group_ts_tmp.iteritems () : 
        if value > min_ts and value < group_ts_tmp[next_key] : 
          # print ( str(next_key) + " --> " + str(key) ) 
          next_key = key 

      mean_pos.append( [group_x[next_key], group_y[next_key]] ) 
      min_ts  = group_ts_tmp[next_key] 
      min_key = next_key
      group_ts_tmp.pop(min_key, None) 

    mean_pos.append(pos[-1]) 

    print ( mean_pos ) 

    f.write("\n")
    f.write("\n")

def myCallback(data):
    global SPD
    global pub
    global curTheta
    global curTwist
    global mean_pos 
    global counter
    global slept

    if counter >= len(mean_pos) : 
      print("Navigation completed\n") 
      curTwist = createTwist(0, 0) 
      pub.publish(curTwist) 
      return 

    curPosX = data.pose.pose.position.x
    curPosY = data.pose.pose.position.y
    dstPosX = mean_pos[counter][0] 
    dstPosY = mean_pos[counter][1]
    print ("Current position: " + str(curPosX) + ", " + str(curPosY) + "\n") 
    print ("Heading position: " + str(dstPosX) + ", " + str(dstPosY) + "\n") 

    if IsCloseEnough([curPosX, curPosY], [dstPosX, dstPosY]) : 
      counter  = counter + 1 
      curTwist = createTwist(0, 0) 
      print("We are arriving at point # " + str(counter) + "\n") 
      pub.publish(curTwist)
      if not slept : 
        rospy.sleep(0.5)
      slept = True 
    else : 
      curTheta = CalcHeading([curPosX, curPosY], [dstPosX, dstPosY]) 
      curTwist = createTwist(SPD * math.cos(curTheta), SPD * math.sin(curTheta))  
      pub.publish(curTwist)
      slept = False

# def myCallback(data):
#     global SPD
#     global group_theta
#     global group_ts
#     global gmm_model
#     global last_state
#     global last_group
#     global pub
#     global curTheta
#     global curTwist
#     global autoPositioning
#     global last_x
#     global last_y
# 
#     # For Sphero 
#     PosX = data.pose.pose.position.x
#     PosY = data.pose.pose.position.y
#     PosT = math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x)
# 
#     # Initial positioning 
#     if autoPositioning : 
#       print ( "Initial positioning ... \n" ) 
#       if ( not IsCloseEnough([PosX, PosY], [pos[0][0], pos[0][1]]) ) : 
#         print ( "Current position: " + str(PosX) + ", " + str(PosY) + "\n" ) 
#         print ( "Heading position: " + str(pos[0][0]) + ", " + str(pos[0][1]) + "\n" ) 
#         curTheta = CalcHeading( [PosX, PosY], [pos[0][0], pos[0][1]] ) 
#         curTwist = createTwist( SPD * math.cos(curTheta), SPD * math.sin(curTheta) )  
#         pub.publish( curTwist )
#         return 
#       else : 
#         print ( "Initial positioning complete \n") 
#         pub.publish(createTwist(0, 0)) 
#         rospy.sleep(1) 
#         autoPositioning = False 
#         return 
# 
#     pos_group = gmm_model.predict([PosX, PosY]).reshape(1, -1).item(0)
#     print(pos_group) 
#     new_ts = group_ts[pos_group]
#     last_ts = group_ts[last_group]
# 
#     # if last_group == 0:
#     #     last_ts = 0
#     # # choose group with timestamp checking
#     # if new_ts < last_ts:
#     #     pos_group = last_group
#     # else:
#     #     last_group = pos_group
# 
#     # get new target theta
#     new_theta = group_theta[pos_group]
# 
#     curTheta = new_theta
#     curTwist = createTwist(SPD * math.cos(curTheta), SPD * math.sin(curTheta))  
# 
#     distance = (PosX - last_x) * (PosX - last_x) + (PosY - last_y) * (PosY - last_y)
#     if distance < 0.1:
#         newTwist = createTwist(0.0, 0.0)
# 
#     print ("Publish new twist")    
#     pub.publish(curTwist)
# 
#     f.write(rospy.get_caller_id() + ' heard currentPos' + "\n")
#     f.write('x: ' + str(PosX) + ',' + 'y: ' + str(PosY) + ',' + 'theta: ' +  str(PosT) +"\n" )
#     f.write('Destination position: (' + str(last_x) + ', ' + str(last_y) +")\n" )
#     f.write('Distance to Destination: ' + str(distance) +"\n" )
#     f.write("In group: " + str(pos_group) + ' and make head turn to: ' + str(new_theta) + "\n")
#     f.write("\n")
#     f.write("\n")

def DataProcess():
    init_model()
    rospy.init_node('data_process', anonymous=True)

    # For Sphero 
    rospy.Subscriber('odom', Odometry, myCallback)

    rospy.spin()

if __name__ == '__main__':
    DataProcess()

