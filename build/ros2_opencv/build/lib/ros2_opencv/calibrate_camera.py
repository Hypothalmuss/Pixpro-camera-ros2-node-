import cv2
import numpy as np
import glob
import yaml

def calibrate_camera():
    # Termination criteria for the iterative algorithm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (6,5,0)
    objp = np.zeros((6*9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images
    objpoints = []
    imgpoints = []

    # Initialize gray to avoid UnboundLocalError
    gray = None

    # Get calibration images
    images = glob.glob('calibration_image_*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (9, 6), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Ensure that at least one image is processed successfully
    if objpoints and imgpoints:
        # Calibrate the camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Save the calibration result
        calibration_data = {'camera_matrix': mtx.tolist(), 'dist_coeff': dist.tolist()}
        with open('camera_calibration.yaml', 'w') as f:
            yaml.dump(calibration_data, f)
        print("Calibration successful. Parameters saved to 'camera_calibration.yaml'.")
    else:
        print("No corners found in images. Calibration unsuccessful.")

if __name__ == '__main__':
    calibrate_camera()

