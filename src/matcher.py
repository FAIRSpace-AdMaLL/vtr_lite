#! /usr/bin/env python3
#
# %COPYRIGHT_BEGIN%
#
#  Part of the code is reused from Magic Leap, Inc. ("COMPANY")'s repository
#  https://github.com/magicleap/SuperGluePretrainedNetwork
#
#
# %COPYRIGHT_END%

from pathlib import Path
import argparse
import random
import numpy as np
import matplotlib.cm as cm
import torch
import sys
import rospy
import cv2
import time

# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from nn_matcher.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nn.matching import Matching
from nn.utils import (make_matching_plot, AverageTimer, image2tensor)

torch.set_grad_enabled(False)

class NN_Matching:

    def __init__(self):

        parser = argparse.ArgumentParser(description='Image pair matching and pose evaluation with SuperGlue', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        # SuperPoint & SuperGlue parameters
        parser.add_argument('--input_pairs', type=str, default='assets/scannet_sample_pairs_with_gt.txt', help='Path to the list of image pairs')
        parser.add_argument('--resize', type=int, default=[300, 150] , help='Resize the input image before running inference. If -1, do not resize')
        parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='outdoor', help='SuperGlue weights')
        parser.add_argument('--max_keypoints', type=int, default=200, help='Maximum number of keypoints detected by Superpoint, -1 keeps all keypoints)')
        parser.add_argument('--keypoint_threshold', type=float, default=0.005, help='SuperPoint keypoint detector confidence threshold')
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

        self.args = parser.parse_args()
        print(self.args)

        rospy.init_node('nn_image_matcher', anonymous=True)

        # Load the SuperPoint and SuperGlue models.
        self.device = 'cuda' if torch.cuda.is_available() and not self.args.force_cpu else 'cpu'
        print('Running inference on device \"{}\"'.format(self.device))
        config = {
            'superpoint': {
                'weights': self.args.superpoint,
                'nms_radius': self.args.nms_radius,
                'keypoint_threshold': self.args.keypoint_threshold,
                'max_keypoints': self.args.max_keypoints
            },
            'superglue': {
                'weights': self.args.superglue,
                'sinkhorn_iterations': self.args.sinkhorn_iterations,
                'match_threshold': self.args.match_threshold,
            }
        }
        self.matching = Matching(config, destcriptor_only=self.args.descriptor_only).eval().to(self.device)

        self.nn_matching_srv = rospy.Service('/vtr_lite/nn_matcher', NNImageMatching, self.matching_pair)
        self.matched_feats_pub = rospy.Publisher('/vtr_lite/matched_features', Image, queue_size = 1)

        print("Ready to localize the robot!")
        rospy.spin()


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

        image0, inp0, scales0 = image2tensor(cv_img_map, self.device, self.args.resize, False)
        image1, inp1, scales1 = image2tensor(cv_img_camera, self.device, self.args.resize, False)

        # Perform the matching.
        pred = self.matching({'image0': inp0, 'image1': inp1})

        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kpts0, kpts1 = pred['keypoints0'], pred['keypoints1']

        if self.args.descriptor_only is True:
            desc0, desc1 = pred['descriptors0'], pred['descriptors1']
            desc0 = np.transpose(desc0)
            desc1 = np.transpose(desc1)
            mkpts0, mkpts1, conf, matches = self.match_descriptors(kpts0, desc0, kpts1, desc1)
            mconf = conf
            mkpts0 = np.array(mkpts0)
            mkpts1 = np.array(mkpts1)
        else:
            matches, conf = pred['matches0'], pred['matching_scores0']
            # Keep the matching keypoints.
            valid = matches > -1
            mkpts0 = kpts0[valid]
            mkpts1 = kpts1[matches[valid]]
            mconf = conf[valid]

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
                    #'SuperDarkPoint',
                    #'Keypoints: {}:{}'.format(len(kpts0), len(kpts1)),
                    'Num of Matches: {}'.format(len(mkpts0)),
                ]
                small_text = [
                    'Matcher Nearest Neighbour'
                ]

            is_saving = False
            file_name = str(time.time())
            visualization_image = make_matching_plot(
                image0, image1, kpts0, kpts1, mkpts0, mkpts1, color,
                text, file_name+'.png', True, True, False, 'Matches', small_text, is_saving)

            visualization_image = bridge.cv2_to_imgmsg(visualization_image, encoding="passthrough")
            self.matched_feats_pub.publish(visualization_image)

            timer.update('viz_match')

            timer.print('Finished matching pair')


        res = NNImageMatchingResponse()
        res.differences = differences.tolist()
        res.histogram = []

        return res

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
