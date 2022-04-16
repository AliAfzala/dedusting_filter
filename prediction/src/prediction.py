#!/usr/bin/env python
import numpy as np
import time
import rospy
from visualization_msgs.msg import Marker
from lidar_sensing.msg import dedusting
import tensorflow as tf
from geometry_msgs.msg import Point


class RosTensorFlow():
    def __init__(self):
        self._model = tf.keras.models.load_model('/home/ali/AutonmousExcavator/models/DNN/TrainingResults_2022_04_11-07:46:07_PM')
        self._sub = rospy.Subscriber('feature_topics',dedusting, self.callback, queue_size=5)
        self._pub = rospy.Publisher('visualization_topics', Marker, queue_size=1)

    def callback(self, dedusting_features=dedusting()):
        features = np.stack((np.asarray(dedusting_features.voxel_mean_intensity),
                     np.asarray(dedusting_features.voxel_std_intensity),
                     np.asarray(dedusting_features.voxel_numberOverDis),
                     np.asarray(dedusting_features.voxel_eigen3OverEigen1),
                     np.asarray(dedusting_features.voxel_eigen2OverEigen1),
                     np.asarray(dedusting_features.voxel_eigen1OverSumEigen)), axis=-1)
        
        prediction = tf.round(self._model(features,training=False)).numpy()

        marker = Marker()
        marker.header.frame_id = "raw_data"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for _point in dedusting_features.point:
            temp = Point()
            temp.x = _point.x 
            temp.y = _point.y 
            temp.z = _point.z 
            marker.points.append(temp)


        self._pub.publish(marker)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    tensor = RosTensorFlow()
    tensor.main()