import cv2
from cv2 import aruco
import numpy as np


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    nada, rvecs, tvecs = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
    return rvecs, tvecs


cap = cv2.VideoCapture(0)
camera_matrix = np.array([[644.60188395, 0., 310.97994546], 
                 [0., 602.01153122, 261.3654111 ], 
                 [0., 0., 1.]], dtype = np.float32)
# distortion_matrix = np.array([-2.70955297e-01, 1.20785082e+00, -2.56368967e-03, -1.96688242e-03, -2.32200070e+00])
distortion_matrix = np.zeros((5,1))

dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

while True:
    ret, frame = cap.read()

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

    if len(markerCorners) > 0:

        for corners in markerCorners:

            (topLeft, topRight, bottomRight, bottomLeft) = corners[0]

            # Marker corners
            tR = (int(topRight[0]), int(topRight[1]))
            bR = (int(bottomRight[0]), int(bottomRight[1]))
            bL = (int(bottomLeft[0]), int(bottomLeft[1]))
            tL = (int(topLeft[0]), int(topLeft[1]))

            # print(tR, bR, bL, tL) 1.16, 0,93
    
            pose = my_estimatePoseSingleMarkers(corners[0], marker_size=6, mtx=camera_matrix, distortion=distortion_matrix)

            (rvec, tvec) = pose
            x = round(tvec[0][0], 2)
            y = round(tvec[1][0], 2)
            z = round(tvec[2][0], 2)

            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z}')

            for corner in corners[0]:
                cv2.circle(frame, (int(corner[0]), int(corner[1])), 4, (255, 0, 0), -1)

    cv2.imshow('image', frame)

    if cv2.waitKey(1)&0xFF == ord("q"):
        break

