import argparse
import os
from glob import glob

import cv2
import numpy as np 
import yaml

# Image Parameters
IMG_HEIGHT = 3648
IMG_WIDTH = 5472

# Calibration Parameters
PATTERN_SIZE = (8,5)
GAUSSIAN_WINDOW = (5,5)
REPROJ_ACCURACY = 0.1
MAX_ITER = 30
INTRINSIC_VECTOR_LEN = 9
DISTORTION_VECTOR_LEN = 4

class CameraCalibration(object):
    """Camera Calilbration class for Intrinsic camera calibration
    """
    def __init__(self):
        """Constructs a Camera Calibration Class 
        """
        self.pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
        self.pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
        self.img_pixel_points = []
        self.object_world_points = []
        self.camera_intrinsics = np.zeros(INTRINSIC_VECTOR_LEN)
        self.lens_distort_coeff = np.zeros(DISTORTION_VECTOR_LEN)

    def find_corners(self, img, count, args):
        """ Find the Checkerboard Square corners 

        Args:
            img (cv2 image): Image with the Checkerboard 
            count (int): Count of the Images with the Calibration Checkerboard 
            args (Optional Args): Optional args with the 
        """
        # found is boolean for the corners 
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCorners(gray_img, PATTERN_SIZE, \
                                                   flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        # print(corners)
        if found is True:
            # print("Lets check if found is true")
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, MAX_ITER, REPROJ_ACCURACY)
            cv2.cornerSubPix(gray_img, corners, GAUSSIAN_WINDOW, (-1, -1), term)
            self.img_pixel_points.append(corners.reshape(1, -1, 2))
            self.object_world_points.append(self.pattern_points.reshape(1, -1, 3))
        
            if args.calibration_debug_dir:
                debug_img = img.copy()
                cv2.drawChessboardCorners(debug_img, PATTERN_SIZE, corners, found)
                debug_filename = os.path.join(args.calibration_debug_dir, 'debug-img-%d.jpg' % (count))
                cv2.imwrite(debug_filename, debug_img)

    def calibrate(self):
        """ Gives the calibration parameters i.e the camera intrinsics and the distortion coefficients 
        """
        ret, camera_intrinsics, lens_distort_coeff, rvecs, tvecs = cv2.calibrateCamera(self.object_world_points, \
                                                                self.img_pixel_points, (IMG_WIDTH, IMG_HEIGHT), None, None)
        print("Root-Mean sqaure value of the reprojection error", ret)
        print("Camera Intrinsic Matrix:\n", camera_intrinsics)
        print("Lens Distortion Coefficients:\n", lens_distort_coeff.ravel())
        return ret, camera_intrinsics, lens_distort_coeff, rvecs, tvecs
      
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Calibrate camera with a sequence of chessboard images.')
    parser.add_argument('-dbd', '--calibration_debug_dir', type = str, help='path to calibration debug directory', default=None)
    parser.add_argument('-cdpath','--calibration_data_path', type = str, help='path to the data directory')
    parser.add_argument('-cyaml','--camera_calibration_yaml', type=str, help='yaml to store the calibrated camera parameters')
    args = parser.parse_args()

    camera_calib = CameraCalibration()
    count = 0
    if args.calibration_data_path: 
        for file in os.listdir(args.calibration_data_path):
            img = cv2.imread(str(args.calibration_data_path + file))
            count += 1
            camera_calib.find_corners(img,count,args)
    
    ret, camera_intrinsics, lens_distort_coeff, rvecs, tvecs = camera_calib.calibrate()
    calibration = {'camera_matrix': camera_intrinsics.tolist(), 'dist_coefs': lens_distort_coeff.tolist()}
    if args.camera_calibration_yaml: 
        with open(args.camera_calibration_yaml, 'w') as fw:
            yaml.dump(calibration, fw)

