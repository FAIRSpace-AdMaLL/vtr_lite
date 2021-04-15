#! /usr/bin/env python3

from pathlib import Path
import argparse
import random
import numpy as np
import matplotlib.cm as cm
import sys
import rospy
import cv2
import time

import tensorflow as tf 
from tensorflow.python.client import timeline

# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from nn_matcher.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nn.utils_naive import (make_matching_plot, AverageTimer, image2tensor)
import time


class NN_Matching:

    def __init__(self):

        parser = argparse.ArgumentParser(description='Image pair matching and pose evaluation with SuperGlue', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        # SuperPoint & SuperGlue parameters
        parser.add_argument('--resize', type=int, default=[300, 150], help='Resize the input image before running inference. If -1, do not resize')
        parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='outdoor', help='SuperGlue weights')
        parser.add_argument('--max_keypoints', type=int, default=500, help='Maximum number of keypoints detected by Superpoint, -1 keeps all keypoints)')
        parser.add_argument('--keypoint_threshold', type=float, default=0.01, help='SuperPoint keypoint detector confidence threshold')
        parser.add_argument('--nms_radius', type=int, default=4, help='SuperPoint Non Maximum Suppression (NMS) radius (Must be positive)')
        parser.add_argument('--sinkhorn_iterations', type=int, default=20, help='Number of Sinkhorn iterations performed by SuperGlue')
        parser.add_argument('--match_threshold', type=float, default=0.1, help='SuperGlue match threshold')
        parser.add_argument('--viz', type=bool, default=True, help='Use faster image visualization with OpenCV instead of Matplotlib')
        parser.add_argument('--viz_extension', type=str, default='png', choices=['png', 'pdf'], help='Visualization file extension. Use pdf for highest-quality.')
        parser.add_argument('--force_cpu', default=False, help='Force pytorch to run in CPU mode.')
        parser.add_argument('--descriptor_only', type=bool, default=True, help='Superpoint descriptor only + NN matcher.')
        parser.add_argument('--superpoint', choices={'official', 'dark'}, default='official', help='SuperPoint weights')
        parser.add_argument('--mask', type=float, default=0.65, help='Create a mask to get ride of ground.')
        # V-T&R parameters
        parser.add_argument('--maxVerticalDifference', type=int, default=10)
        parser.add_argument('--numBins', type=int, default=41) # 73 / 41
        parser.add_argument('--granlarity', type=int, default=20)
        parser.add_argument('--panorama', type=bool, default=False, help='use fisheye camera.') # [720, 180]  / [640, 350]
        # Model Path
        parser.add_argument('--model_path', type=str, default='./nn_models', help='Path to the TF models')
        parser.add_argument('--model_name', type=str, default='gamma4', help='name of the TF model')
        parser.add_argument('--trace', type=bool, default=False)

        self.args = parser.parse_args()
        print(self.args)

        rospy.init_node('nn_image_matcher', anonymous=True)

        # Load the SuperPoint and SuperGlue models
        self.load_tf_model()


        self.nn_matching_srv = rospy.Service('/vtr_lite/nn_matcher', NNImageMatching, self.matching_pair)
        self.matched_feats_pub = rospy.Publisher('/vtr_lite/matched_features', Image, queue_size = 1)

        print("Ready to localize the robot!")
        rospy.spin()

    def load_tf_model(self):

        weights_name = self.args.model_name
        weights_root_dir = Path(self.args.model_path)
        weights_root_dir.mkdir(parents=True, exist_ok=True)
        weights_dir = Path(weights_root_dir, weights_name)

        self.graph = tf.compat.v1.Graph()
        self.sess = tf.compat.v1.Session(graph=self.graph)
        tf.compat.v1.saved_model.loader.load(self.sess, [tf.compat.v1.saved_model.tag_constants.SERVING], str(weights_dir))

        self.input_img_tensor = self.graph.get_tensor_by_name('superpoint/image:0')
        self.output_prob_nms_tensor = self.graph.get_tensor_by_name('superpoint/prob_nms:0')
        self.output_desc_tensors = self.graph.get_tensor_by_name('superpoint/descriptors:0')

        print("DarkPoint model is loaded from {}.".format(weights_dir))


    def preprocess_image(self, img, img_size):
        img = cv2.resize(img, img_size)

        img_ori = img.copy()
        img = np.expand_dims(img, -1)
        img = img.astype(np.float32)
        img_preprocessed = img / 255.

        return img_preprocessed, img_ori


    def building_histogram(self, kpts0, kpts1):
        # histogram = np.zeros(self.args.numBins, dtype=int)

        differenceX = kpts0[:, 0] - kpts1[:, 0]
        differenceY = kpts0[:, 1] - kpts1[:, 1]
        invaild = abs(differenceY) > self.args.maxVerticalDifference

        if self.args.panorama is True:
            differenceX[np.nonzero(differenceX>0.5*self.args.resize[0])] -= self.args.resize[0]
            differenceX[np.nonzero(differenceX<-0.5*self.args.resize[0])] += self.args.resize[0]

        differences = differenceX
        differences[invaild] = -1000000

        index = (differenceX + self.args.granlarity / 2) / self.args.granlarity + self.args.numBins / 2
        index = index[~invaild]
        # unique_index, counts_index = np.unique(index, return_counts=True)
        span = (self.args.numBins * self.args.granlarity) / 2
        # histogram, bin_edges = np.histogram(index, bins=self.args.numBins, range=(-span, span))

        return differences, []


    def matching_pair(self, req):
        timer = AverageTimer(newline=True)

        now = time.time()

        bridge = CvBridge()
        cv_img_map = bridge.imgmsg_to_cv2(req.image_map, "passthrough")
        cv_img_camera = bridge.imgmsg_to_cv2(req.image_camera, "passthrough")

        if(self.args.mask < 1.0):
            num_row = int(cv_img_map.shape[0]*self.args.mask)
            
            if self.args.panorama:
                cv_img_map = cv_img_map[-num_row:, :]
                cv_img_camera = cv_img_camera[-num_row:, :]
            else:
                cv_img_map = cv_img_map[:num_row, :]
                cv_img_camera = cv_img_camera[:num_row, :]

        img0, img0_ori = self.preprocess_image(cv_img_map, (self.args.resize[0], self.args.resize[1]))
        img1, img1_ori = self.preprocess_image(cv_img_camera, (self.args.resize[0], self.args.resize[1]))

        if self.args.trace:
            options = tf.compat.v1.RunOptions(trace_level=tf.compat.v1.RunOptions.FULL_TRACE)
            run_metadata = tf.compat.v1.RunMetadata()

        if self.args.trace:
            out0 = self.sess.run([self.output_prob_nms_tensor, self.output_desc_tensors],
                        feed_dict={self.input_img_tensor: np.expand_dims(img0, 0)},
                        options=options, run_metadata=run_metadata)
        else:
            out0 = self.sess.run([self.output_prob_nms_tensor, self.output_desc_tensors],
                        feed_dict={self.input_img_tensor: np.expand_dims(img0, 0)})

        # Create the Timeline object, and write it to a json file
        
        if self.args.trace:
            fetched_timeline = timeline.Timeline(run_metadata.step_stats)
            chrome_trace = fetched_timeline.generate_chrome_trace_format()
            with open('timeline_' + self.args.model_name + '.json', 'w') as f:
                f.write(chrome_trace)

        out1 = self.sess.run([self.output_prob_nms_tensor, self.output_desc_tensors],
                        feed_dict={self.input_img_tensor: np.expand_dims(img1, 0)})

        timer.update('nn inference') 

        keypoint_map0 = np.squeeze(out0[0])
        descriptor_map0 = np.squeeze(out0[1])
        kpts0, desc0 = self.extract_superpoint_keypoints_and_descriptors(keypoint_map0, descriptor_map0, self.args.max_keypoints)

        keypoint_map1 = np.squeeze(out1[0])
        descriptor_map1 = np.squeeze(out1[1])
        kpts1, desc1 = self.extract_superpoint_keypoints_and_descriptors(keypoint_map1, descriptor_map1, self.args.max_keypoints)

        kpts0 = kpts0[:, ::-1]
        kpts1 = kpts1[:, ::-1]
        mkpts0, mkpts1, conf, matches = self.match_descriptors(kpts0, desc0, kpts1, desc1)
        mconf = conf
        mkpts0 = np.array(mkpts0)
        mkpts1 = np.array(mkpts1)

        timer.update('matcher') 

        differences, _ = self.building_histogram(mkpts0, mkpts1)
        # print(differences)

        if self.args.viz:
            # Visualize the matches.
            color = cm.jet(mconf)
            
            if self.args.descriptor_only is False:
                text = [
                'SuperGlue',
                'Keypoints: {}:{}'.format(len(kpts0), len(kpts1)),
                'Matches: {}'.format(len(mkpts0)),
                ]
    
                # Display extra parameter info.
                k_thresh = self.matching.superpoint.config['keypoint_threshold']
                m_thresh = self.matching.superglue.config['match_threshold']
                small_text = [
                    'Keypoint Threshold: {:.4f}'.format(k_thresh),
                    'Match Threshold: {:.2f}'.format(m_thresh),
                ]
            else:
                text = [
                    'Num of Matches: {}'.format(len(mkpts0)),
                ]
                small_text = [
                    'Matcher Nearest Neighbour'
                ]

            is_saving = False
            file_name = str(time.time())
            visualization_image = make_matching_plot(
                img0_ori, img1_ori, kpts0, kpts1, mkpts0, mkpts1, color,
                text, file_name+'.png', True, True, False, 'Matches', small_text, is_saving)

            visualization_image = bridge.cv2_to_imgmsg(visualization_image, encoding="passthrough")
            self.matched_feats_pub.publish(visualization_image)

            timer.update('viz_match')

            timer.print('Finished matching pair')


        res = NNImageMatchingResponse()
        res.differences = differences.tolist()
        res.histogram = []

        return res


    def extract_superpoint_keypoints_and_descriptors(self, keypoint_map, descriptor_map,
                                                 keep_k_points=1000):

        def select_k_best(points, k):
            """ Select the k most probable points (and strip their proba).
            points has shape (num_points, 3) where the last coordinate is the proba. """
            sorted_prob = points[points[:, 2].argsort(), :2]
            start = min(k, points.shape[0])
            return sorted_prob[-start:, :]

        # Extract keypoints
        keypoints = np.where(keypoint_map > 0)
        prob = keypoint_map[keypoints[0], keypoints[1]]
        keypoints = np.stack([keypoints[0], keypoints[1], prob], axis=-1)

        keypoints = select_k_best(keypoints, keep_k_points)
        keypoints = keypoints.astype(int)

        # Get descriptors for keypoints
        desc = descriptor_map[keypoints[:, 0], keypoints[:, 1]]

        # Convert from just pts to cv2.KeyPoints
        # keypoints = [cv2.KeyPoint(p[1], p[0], 1) for p in keypoints]

        return keypoints, desc


    def match_descriptors(self, kp1, desc1, kp2, desc2, keep=0.5):
        # Match the keypoints with the warped_keypoints with nearest neighbor search
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        matches = bf.match(desc1, desc2)

        matches = sorted(matches, key=lambda x: x.distance)
        good = matches[:int(len(matches)*keep)]

        matches_idx = np.array([m.queryIdx for m in good])
        m_kp1 = [kp1[idx] for idx in matches_idx]
        matches_idx = np.array([m.trainIdx for m in good])
        m_kp2 = [kp2[idx] for idx in matches_idx]
        confidence = 1 - np.array([m.distance for m in good])

        return m_kp1, m_kp2, confidence, good

if __name__ == '__main__':
    dr = NN_Matching()
