#!/usr/bin/env python3
"""
Author : Nathan Hendler, Emmanuel Gonzalez, Travis Simmons, Ariyan Zarei
Date   : 2020-12-09
Purpose: Merging east and west point clouds using ICP registration.
"""

import argparse
import os
import sys, time
import open3d as o3d
import numpy as np
import copy
import json
import cv2
import math

# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='ICP point cloud merging',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-e',
                        '--east_pcd',
                        metavar='east_pcd',
                        help='Filename of east PCD',
                        type=str,
                        required=True)

    parser.add_argument('-w',
                        '--west_pcd',
                        metavar='west_pcd',
                        help='Filename of west PCD',
                        type=str,
                        required=True)

    parser.add_argument('-t',
                        '--threshold',
                        help='Movement range threshold for ICP',
                        type=float,
                        default=10.0)

    parser.add_argument('-o',
                        '--outdir',
                        metavar ='outdir',
                        help = 'Output directory',
                        type = str,
                        default='icp_registration_out')

    return parser.parse_args()


# --------------------------------------------------
def open_paint_pcd(west_path, east_path):
    east_pcd = o3d.io.read_point_cloud(east_path, print_progress=True)
    west_pcd = o3d.io.read_point_cloud(west_path, print_progress=True)

    east_pcd.paint_uniform_color([255, 0, 0])
    west_pcd.paint_uniform_color([0, 255, 0])

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

# -------------------------------------------------- Added by Ariyan 
def correct_png(img, is_east, meta_path):

    with open(meta_path, 'r') as f:

        # Open metadata and pull scan direction.
        meta_dict = json.load(f)
        scan_direction = str(meta_dict['lemnatec_measurement_metadata']['sensor_variable_metadata']['current setting Scan direction (automatically set at runtime)'])

        # Correct array's orientation.
        img = np.array(img)

        if scan_direction == '0':

            if is_east:
                img = np.rot90(img, 1)
            else:
                img = np.rot90(img, 1)
                img = np.flipud(img)

        elif scan_direction == '1':

            if is_east:
                img = np.rot90(img, 3)
                img = np.flipud(img)
            else:
                img = np.rot90(img, 3)
               
    return img

# -------------------------------------------------- Added by Ariyan 
def merge_png_files(west_pcd_path,east_pcd_path,T):
    west_png_path = west_pcd_path.replace('.ply','_g.png')
    east_png_path = east_pcd_path.replace('.ply','_g.png')
    metadata_path = east_pcd_path.replace('__Top-heading-east_0.ply','_metadata.json')

    img_e = cv2.imread(east_png_path)
    img_e = cv2.resize(img_e,(int(img_e.shape[1]/5),int(img_e.shape[0]/5)))
    img_e = cv2.normalize(img_e, None, 255,0, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    img_w = cv2.imread(west_png_path)
    img_w = cv2.resize(img_w,(int(img_w.shape[1]/5),int(img_w.shape[0]/5)))
    img_w = cv2.normalize(img_w, None, 255,0, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    img_e_corrected = correct_png(img_e,True,metadata_path)
    img_w_corrected = correct_png(img_w,False,metadata_path)

    merged_frame = np.zeros((img_w_corrected.shape[0],img_w_corrected.shape[1]+int(T*img_e_corrected.shape[1]),3))
    merged_frame[:,:img_w_corrected.shape[1],:] = img_w_corrected
    merged_frame[:,int(T*img_e_corrected.shape[1]):,:] = img_e_corrected
    merged_frame = cv2.normalize(merged_frame, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)

    merged_frame = cv2.rotate(merged_frame, cv2.ROTATE_90_CLOCKWISE)

    cv2.imwrite(metadata_path.replace('_metadata.json','_merged_east_west.png'),merged_frame)


# -------------------------------------------------- Edited by Ariyan
def process_pcd(west_pcd_path, east_pcd_path):
    args = get_args()

    west_pcd, east_pcd = open_paint_pcd(west_pcd_path, east_pcd_path)

    icp_pcd = icp_registration(west_pcd, east_pcd, args.threshold)

    f_name = os.path.splitext(os.path.basename(west_pcd_path))[-2].split('__')[-2] + '_icp_merge.ply'
    out_path = os.path.join(args.outdir, f_name)

    west_transform = west_pcd.transform(icp_pcd.transformation)
    out_pcd = west_transform + east_pcd

    o3d.io.write_point_cloud(out_path, out_pcd)

    # added by Ariyan:

    mins_w = np.min(np.array(west_pcd.points),axis=0)
    maxs_w = np.max(np.array(west_pcd.points),axis=0)
    
    mins_e = np.min(np.array(east_pcd.points),axis=0)
    maxs_e = np.max(np.array(east_pcd.points),axis=0)

    translation = abs(mins_e[1]-mins_w[1])/(maxs_e[1]-mins_e[1])

    merge_png_files(west_pcd_path,east_pcd_path,translation)



# --------------------------------------------------
def main():
    args = get_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    process_pcd(args.west_pcd, args.east_pcd)


# --------------------------------------------------
if __name__ == '__main__':
    main()
