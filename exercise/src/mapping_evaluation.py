#!/usr/bin/env python
# This script evaluates the mapping project by comparing the ground-truth map
# with the current map. Instead of the actual f-score, this script calculates
# an f-score wrt. to the positive and negative mapped cells, normalized to the
# number of ground-truth cells

import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid

map_gt = None
stamps = []
scores = []
best_time = -1.
best_score = -1.

def map_storage(msg):
    global map_gt
    rospy.loginfo("Received ground-truth map")
    map_gt = msg

def map_eval(msg):
    global map_gt, best_score, best_time, stamps, scores
    if map_gt is None:
        rospy.loginfo("Ground-truth map not yet received")
        return
    rospy.loginfo("Evaluate maps")
    stamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    stamps.append(stamp)
    # get the score

    # f score does not work, because mapping with known poses is always perfect
    score = np.sum((msg.data > 50) & (map_gt.data > 50))
    TP = float(sum([v > 50 and v_gt > 50 for v, v_gt in zip(msg.data, map_gt.data)]))
    #FP = float(sum([v > 50 and v_gt < 50 for v, v_gt in zip(msg.data, map_gt.data)]))
    #FN = float(sum([v < 50 and v_gt > 50 for v, v_gt in zip(msg.data, map_gt.data)]))
    TN = float(sum([v < 50 and v_gt < 50 for v, v_gt in zip(msg.data, map_gt.data)]))
    #precision = TP / (TP + FP)
    #recall = TP / (TP + FN)
    #score = 2. * (precision * recall) / (precision + recall)
    #rospy.loginfo("precision: %0.2f, recall: %0.2f, f-score: %0.2f" % (precision, recall, score))

    # calculate instead the fill rate as f score
    PGT = float(sum([v_gt > 50 for v_gt in map_gt.data]))
    NGT = float(sum([v_gt < 50 for v_gt in map_gt.data]))
    rospy.loginfo("TP: %0.2f, TN: %0.2f, PGT: %0.2f, NGT: %0.2f" % (TP, TN, PGT, NGT))
    score = 2. * (TP / PGT * TN / NGT) / (TP / PGT + TN / NGT)
    scores.append(score)
    
    # plot the figure
    plt.cla()
    plt.plot(stamps, scores, '-*')
    plt.xlim((0., stamp))
    plt.ylim((0., 1.))
    if score > best_score:
        best_score = score
        best_time = stamp
    plt.title("Best f-score %0.2f at %0.2f sec." % (best_score, best_time))
    plt.draw()
    plt.pause(1e-9)

if __name__ == '__main__':
    rospy.init_node("mapping_evaluation")
    rospy.Subscriber("/map", OccupancyGrid, map_eval)
    rospy.Subscriber("/gt/map", OccupancyGrid, map_storage)
    plt.ion()
    plt.show()
    rospy.spin()
