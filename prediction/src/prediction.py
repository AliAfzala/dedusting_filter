import numpy as np
import time
import rospy
from visualization_msgs.msg import Marker
from lidar_sensing.msg import dedusting
import tensorflow as tf
#########################################################################################################



class RosTensorFlow():
    def __init__(self):
        self._model = tf.keras.models.load_model('/home/ali/AutonmousExcavator/models/DNN/TrainingResults_2022_04_11-07:46:07_PM')
        self._sub = rospy.Subscriber('feature_topics',dedusting, self.callback, queue_size=5)
        self._pub = rospy.Publisher('visualization_topics', Marker, queue_size=5)

    def callback(self, dedusting_features=dedusting()):
        features = np.stack((np.asarray(dedusting_features.voxel_mean_intensity),
                     np.asarray(dedusting_features.voxel_std_intensity),
                     np.asarray(dedusting_features.voxel_numberOverDis),
                     np.asarray(dedusting_features.voxel_eigen3OverEigen1),
                     np.asarray(dedusting_features.voxel_eigen2OverEigen1),
                     np.asarray(dedusting_features.voxel_eigen1OverSumEigen)), axis=-1)
        
        prediction = tf.round(self._model(features,training=False)).numpy()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    tensor = RosTensorFlow()
    tensor.main()