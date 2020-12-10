#!/usr/bin/env python3
"""
Author : Nathan Hendler, Emmanuel Gonzalez, Travis Simmons
Date   : 2020-12-09
Purpose: Merging east and west point clouds using ICP registration.
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
                        type = str,
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

    parser.add_argument('-o'
                        '--outdir',
                        metavar ='outdir',
                        help = 'Output directory',
                        type = str,
                        default='icp_registration_out')

    return parser.parse_args()


# --------------------------------------------------
def open_paint_pcd(west_path, east_path):
    args = get_args()

    east_pcd = o3d.io.read_point_cloud(args.east_pcd, print_progress=True)
    west_pcd = o3d.io.read_point_cloud(args.west_pcd, print_progress=True)

    east_pcd.paint_uniform_color([1, 0.706, 0])         # East is yellow
    west_pcd.paint_uniform_color([0, 0.651, 0.929])     # West is blue

    return west_pcd, east_pcd

# --------------------------------------------------
def icp_registration(west_pcd, east_pcd, threshold):
    trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix, this is a transformation matrix,
                             [0,1,0,0],   # It means there is no displacement, no rotation, we enter
                             [0,0,1,0],   # This matrix is ​​the initial transformation
                             [0,0,0,1]])

    reg_p2p = o3d.pipelines.registration.registration_icp(
        east_pcd, west_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    return reg_p2p


# --------------------------------------------------
def process_pcd(west_pcd_path, east_pcd_path):
    args = get_args()

    #west_pcd, east_pcd = open_paint_pcd(west_pcd_path, east_pcd_path)
    #icp_pcd = icp_registration(west_pcd, east_pcd)

    f_name = os.path.splitext(os.path.basename(west_pcd_path)[-2]).split('__')[-2]
    out_path = os.path.join(args.outdir, f_name)
    print(out_path)
    #o3d.io.write_point_cloud()


# --------------------------------------------------
def main():
    args = get_args()

    process_pcd(args.west_pcd, args.east_pcd)


# --------------------------------------------------
if __name__ == '__main__':
    main()
