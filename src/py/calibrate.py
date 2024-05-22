#!/usr/bin/python3  

import numpy as np
import cv2
import glob
from tqdm import tqdm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

    print("Image size: ", image.shape[::-1])
            
    print("Calibrating the camera...")

    #Calibrate the camera getting the std deviation
    ret, camera_matrix, distortion_coefficients, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.calibrateCameraExtended(
        object_points_list, image_points_list, gray.shape[::-1], None, None)
    
    #Obtain 3D scatter plot of the calibration points
    ##############################################################################################################
    scatter_points = []
    origin = np.array([0, 0, 0]).reshape(1, 1, 3)
    scatter_points.append(origin)
    for i in range(len(object_points_list)):
        R = cv2.Rodrigues(rvecs[i])[0]
        t = tvecs[i]
        T = np.zeros((4, 4))
        T[:3, :3] = R
        T[:3, 3] = t.reshape(3)
        T[3, 3] = 1

        scatter_points.append(cv2.perspectiveTransform(object_points_list[i].reshape(-1, 1, 3), T))

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    for i in range(len(scatter_points)):
        ax.scatter(scatter_points[i][:,0,0], scatter_points[i][:,0,1], scatter_points[i][:,0,2])
    plt.show()


    #Get the reprojection errors
    ##############################################################################################################
    total_error = 0
    per_image_errors = []
    all_errors = []

    test = []

    for i in range(len(object_points_list)):
        image_points, _ = cv2.projectPoints(object_points_list[i], rvecs[i], tvecs[i], camera_matrix, distortion_coefficients)
        error = cv2.norm(image_points_list[i], image_points, cv2.NORM_L2) / len(image_points)
        total_error += error
        per_image_errors.append(error)


        error_test = (image_points_list[i] - image_points).reshape(1, -1)
        error_squared = np.square(error_test)
        error_squared_sum = np.sum(error_squared)
        error_root = np.sqrt(error_squared_sum)
        
        print("Error        : ", error_root/len(image_points))
        print("Error l2     : ", error)

        error_rms = np.sqrt(np.sum(error_squared)/len(image_points))

        print("Error rms    : ", error_rms)
        print("perViewErrors: ", perViewErrors[i][0])

        for j in range(len(image_points)):
            all_errors.append(cv2.norm(image_points_list[i][j], image_points[j], cv2.NORM_L2))
        

    ## Per point
    print("Mean reprojection error per point: ", np.mean(all_errors))
    print("std deviation per point: ", np.std(all_errors))
    print("Max per point error: ", np.max(all_errors))

    ## Per image
    print("Mean reprojection error per image: ", total_error / len(object_points_list))
    print("std deviation per image: ", np.std(per_image_errors))
    print("Max per image error: ", np.max(per_image_errors))

    #Get the perViewErrors
    ##############################################################################################################
    print("Mean per view error: ", np.mean(perViewErrors))
    print("std deviation per view error: ", np.std(perViewErrors))
    print("Max per view error: ", np.max(perViewErrors)) 

    plt.plot(perViewErrors)
    plt.show()
    

    # Save the camera matrix and distortion coefficients to a file
    ##############################################################################################################
    print("\n\n\nCamera matrix: \n" + str(camera_matrix))
    print("Distortion coefficients: \n" + str(distortion_coefficients))

    print("\n\n\nStandard deviation of intrinsics: \n" + str(stdDeviationsIntrinsics))
    
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