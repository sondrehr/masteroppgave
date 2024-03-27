#!/usr/bin/python3  

import numpy as np
import cv2
import glob
from tqdm import tqdm

def calibrate_camera():
    chessboard_size = (9, 7)

    object_points = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    object_points[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    object_points_list = []
    image_points_list = [] 

    image_files = glob.glob('src/precision_landing/calib_imgs/*.png')

    for image_file in tqdm(image_files):
        image = cv2.imread(image_file)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret == True:
            object_points_list.append(object_points)
            image_points_list.append(corners)
            
    print("Calibrating the camera...")

    ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(
        object_points_list, image_points_list, gray.shape[::-1], None, None)
    
    print("\n\n\nCamera matrix: \n" + str(camera_matrix))
    print("Distortion coefficients: \n" + str(distortion_coefficients))
    
    data = {
        "image_width": image.shape[1],
        "image_height": image.shape[0],
        "camera_name": "cam0",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.tolist()
        },
        "distortion_model": "equidistant",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 4,
            "data":  distortion_coefficients.tolist()
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [camera_matrix[0,0], 0, camera_matrix[0,2], 0, 0, camera_matrix[1,1], camera_matrix[1,2], 0, 0, 0, 1, 0]
        }
    }

    # write to opencv-yaml file
    fs = cv2.FileStorage('src/precision_landing/params.yaml', cv2.FILE_STORAGE_WRITE)
    fs.write("Camera_Matrix", camera_matrix)
    fs.write("Distortion_Coefficients", distortion_coefficients)
    fs.release()

    # write to blackfly-yaml file
    with open('src/precision_landing/blackfly.yaml', 'w') as outfile:
        outfile.write("image_width: " + str(image.shape[1]) + "\n")
        outfile.write("image_height: " + str(image.shape[0]) + "\n")
        outfile.write("camera_name: cam0\n")
        outfile.write("camera_matrix:\n")
        outfile.write("  rows: 3\n")
        outfile.write("  cols: 3\n")
        outfile.write("  data: [")
        for i in range(3):
            for j in range(3):
                outfile.write(str(camera_matrix[i][j]))
                if i == 2 and j == 2:
                    outfile.write("]\n")
                else:
                    outfile.write(", ")
        outfile.write("distortion_model: equidistant\n")
        outfile.write("distortion_coefficients:\n")
        outfile.write("  rows: 1\n")
        outfile.write("  cols: 4\n")
        outfile.write("  data: [")
        for i in range(5):
            outfile.write(str(distortion_coefficients[0][i]))
            if i == 4:
                outfile.write("]\n")
            else:
                outfile.write(", ")
        outfile.write("rectification_matrix:\n")
        outfile.write("  rows: 3\n")
        outfile.write("  cols: 3\n")
        outfile.write("  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]\n")
        outfile.write("projection_matrix:\n")
        outfile.write("  rows: 3\n")
        outfile.write("  cols: 4\n")
        outfile.write("  data: [")
        for i in range(3):
            for j in range(4):
                outfile.write(str(data["projection_matrix"]["data"][i*4+j]))
                if i == 2 and j == 3:
                    outfile.write("]\n")
                else:
                    outfile.write(", ")



if __name__ == "__main__":
    calibrate_camera()