#!/usr/bin/env python

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

index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
group_counter = dict((el,0) for el in index)
group_theta = dict((el,0) for el in index)
group_ts = dict((el,0) for el in index)
kmeans = KMeans(random_state=0)
gmm_model = mixture.GaussianMixture(covariance_type='full')
last_group = 0
last_state = 0
previous_state = dict((el,0) for el in index)
f = open('model_info-' + str(time.time()) + '.txt', 'w+')

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


# result storage
index = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32]
group_counter = dict((el,0) for el in index)
group_theta = dict((el,0) for el in index)

# data preprocess
my_data = genfromtxt('path-2016-12-06_16:37:41.txt', delimiter=',')
dataSet = np.array(my_data)
pos, timestamps, dirc = np.split(my_data, [2, 3], axis = 1)

k = 16
# get init means for Gaussian Mixture Model
kmeans.set_params(n_clusters = k)
kmeans.fit(pos)

# fit a Gaussian Mixture Model
# gmm_model.set_params(n_components = k, means_init=kmeans.cluster_centers_)
# Defaul init_params: kmeans
gmm_model.set_params(n_components = k)
gmm_model.fit(pos)

# cluster the origin data to get group orientation
clustring_result = gmm_model.predict(pos)

# plot results
plot_results(pos, clustring_result, gmm_model.means_, gmm_model.covariances_, 0, 'Expectation-maximization')
plt.show()

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


