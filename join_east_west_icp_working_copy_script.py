#!/usr/bin/env python3
"""
Author : 
Date   : 
Purpose: 
"""

import argparse
import os
import sys, time
import open3d as o3d
import numpy as np
import copy


# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='Rock the Casbah',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('data_dir',
                        metavar='data_dir',
                        help = 'Directory containing east and west passes of PCD to be merged')

    parser.add_argument('-e'
                        '--east_pcd',
                        metavar ='east_pcd',
                        help = 'Filename of east PCD',
                        type = str
                        required = True)

    parser.add_argument('-w'
                        '--west_pcd',
                        metavar ='west_pcd',
                        help = 'Filename of west PCD',
                        type = str,
                        required = True)

    parser.add_argument('-t',
                        '--threshold',
                        help='Movement range threshold for ICP',
                        type=float,
                        default= 10.0)

    args = parser.parse_args()

    if '/' not in args.outdir:
        args.outdir = args.outdir + '/'

    return args


# --------------------------------------------------
def main():

    east_pcd = o3d.io.read_point_cloud(args.data_dir + args.east_pcd, print_progress=True)
    west_pcd = o3d.io.read_point_cloud(args.data_dir + args.west_pcd, print_progress=True)

    print(east_pcd)
    print(west_pcd)

    def pretty_time_delta(seconds):
        sign_string = '-' if seconds < 0 else ''
        seconds = abs(int(seconds))
        days, seconds = divmod(seconds, 86400)
        hours, seconds = divmod(seconds, 3600)
        minutes, seconds = divmod(seconds, 60)
        if days > 0:
            return '%s%dd%dh%dm%ds' % (sign_string, days, hours, minutes, seconds)
        elif hours > 0:
            return '%s%dh%dm%ds' % (sign_string, hours, minutes, seconds)
        elif minutes > 0:
            return '%s%dm%ds' % (sign_string, minutes, seconds)
        else:
            return '%s%ds' % (sign_string, seconds)


    # vis_dict is used only for visualisation geometries.  It has nothing to do
    # with ICP.  I took this dict from the open3d gui (cmd-cntrl-c) so that I can
    # look at the same point of interest each time.  However, it seems to be
    # lacking some rotation information, so it doesn't setup the point of view
    # perfectly, but it's close...
    vis_dict = {
        "class_name" : "ViewTrajectory",
        "interval" : 29,
        "is_loop" : False,
        "trajectory" :
        [
            {
                "boundingbox_max" : [ 530.43841552734375, 21795.224609375, 1431.61669921875 ],
                "boundingbox_min" : [ -552.8602294921875, -3855.62890625, 422.0048828125 ],
                "field_of_view" : 60.0,
                "front" : [ 0.11439391589314821, 0.99222772945197013, -0.048971061998077971 ],
                "lookat" : [ -277.00758495926499, 4993.7530705115532, 734.56313038767337 ],
                "up" : [ 0.0015270675038333088, 0.049118972274514533, 0.99879176740076014 ],
                "zoom" : 0.059999999999999998

            }
        ],
        "version_major" : 1,
        "version_minor" : 0
    }
    vis_zoom   = vis_dict["trajectory"][0]["zoom"]
    vis_front  = vis_dict["trajectory"][0]["front"]
    vis_lookat = vis_dict["trajectory"][0]["lookat"]
    vis_up     = vis_dict["trajectory"][0]["up"]


    east_pcd.paint_uniform_color([1, 0.706, 0])         # East is yellow
    west_pcd.paint_uniform_color([0, 0.651, 0.929])     # West is blue

    def evaluate_registration(pc1, pc2, threshold=args.threshold, report_name=None):
        print('----------------------------------------------------------------------')
        print(f'Evaluation of {report_name} point clouds...')
        print(f'Using a threshold of {threshold} for evaluation')
        evaluation = o3d.pipelines.registration.evaluate_registration(
            pc1,
            pc2,
            threshold
        )
        print(evaluation)
        print('----------------------------------------------------------------------')
        print()

    def draw_registration_result(east_pcd, west_pcd, transformation):
        # This function is from the ICP tutorial
        east_pcd_temp = copy.deepcopy(east_pcd)
        west_pcd_temp = copy.deepcopy(west_pcd)
        east_pcd_temp.paint_uniform_color([1, 0.706, 0])
        west_pcd_temp.paint_uniform_color([0, 0.651, 0.929])
        east_pcd_temp.transform(transformation)
        o3d.visualization.draw_geometries([east_pcd_temp, west_pcd_temp],
                                    zoom=vis_zoom,
                                    front=vis_front,
                                    lookat=vis_lookat,
                                    up=vis_up
        )
        evaluate_registration(east_pcd_temp, west_pcd_temp, report_name="ICP'd")

    trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix, this is a transformation matrix,
                            [0,1,0,0],   # It means there is no displacement, no rotation, we enter
                            [0,0,1,0],   # This matrix is ​​the initial transformation
                            [0,0,0,1]])

    # Raw PC Evaluation
    evaluate_registration(east_pcd, west_pcd, report_name="Raw")

    ## Do ICP, print evaulation, and display it...

    print('----------------------------------------------------------------------')
    print('ICP....')
    print(f'Using a threshold of {threshold} for ICP')
    t0 = time.time()

    reg_p2p = o3d.pipelines.registration.registration_icp(
        east_pcd, west_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    t1 = time.time()
    time_string = pretty_time_delta(t1-t0)
    print(f"Time to perform ICP: {time_string}")
    print()

    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(east_pcd, west_pcd, reg_p2p.transformation)
    print('----------------------------------------------------------------------')
    print()
# --------------------------------------------------
if __name__ == '__main__':
    main()
