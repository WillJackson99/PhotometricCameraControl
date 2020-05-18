import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

import CaptureCode

workdir = "./workdir/"
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(13, 9, 1, .8, aruco_dict)
imboard = board.draw((2000, 2000))
cv2.imwrite(workdir + "chessboard.tiff", imboard)


'''
order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
images = images[order]



im = PIL.Image.open(images[0])
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(im)
#ax.axis('off')
plt.show()
'''
def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        #print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")
    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors



def pixelList(images):
    maxList = []
    for im in images:
        x = cv2.imread(im)
        gray = cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (131,131), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        maxList.append(maxLoc)
        
    return maxList

def calibrate():

    #=====================================--Intrinsic--=================================================================
    intrinsic = "/home/pi/gphoto/PhotometricCameraControl/workdir/data2/"
    images = np.array([intrinsic + f for f in os.listdir(intrinsic) if f.endswith(".JPG") ]) 
    images.sort()

    allCorners,allIds,imsize= read_chessboards(images)
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)#Only using mtx variable
    #=====================================--Extrinsic--=================================================================
    lights_location = "/home/pi/gphoto/PhotometricCameraControl/workdir/LIGHT/"
    map( os.unlink, (os.path.join( lights_location,f) for f in os.listdir(lights_location)) )#Deletes all files in the light direction

    os.chdir(lights_location)
    CaptureCode.captureCycle()
    
    datadir = lights_location
    images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".JPG") ]) 
    images.sort()
    print(images)
    

    allCorners,allIds,imsize= read_chessboards(images)
    ret, a, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
    #============================================================================================================

    pixels = np.array(pixelList(images))
    #pixels = np.array([[3325,2279],[2857,2122],[3333,2613],[3131,2790],[3130,2135],[2829,2802],[2633,2589],[2671,2307]])
    #print("pixels")
    #print(pixels)
    homoPixels = np.empty([3,8])
    homoPixels = cv2.convertPointsToHomogeneous(pixels)
    #print("Homogeneous coordinates")
    #print(homoPixels)

    
    R = np.empty([3,3])
    cv2.Rodrigues(rvecs[0],R)#The homogeneous coordinates
    #print("Rotation matrix")
    #print(R)#The rotation matrix

    N = np.array([np.cross(R[:,0], R[:,1])])
    #print("Plane normal")
    #print(N)

    Z = np.column_stack((R[:,0], R[:,1], tvecs[0]))
    H = np.empty([3,3])
    H = np.dot(mtx,Z)#Get the homography matrix
    #print("H matrix")
    #print(H)
    HI = np.linalg.inv(H) 
    
    
    Lights = np.empty((0,3),int)

    for i in range(0,8):
        #print("M matrix")
        #print(homoPixels[0].transpose())
        m = np.dot(HI,homoPixels[i].transpose())#fix change to invert rather than transpose 
        m=np.column_stack((m[0]/m[2],m[1]/m[2],1))#World position of point, normalize around Z
        #print(m)

        P = np.dot(R,m.transpose())
        P = np.add(P,tvecs[0])#Camera coordinates of position
        #print("The HOLY P POSITION")
        #print(P)

        #print("incident")
        L = P#Incident line fro camera to point
        #print(L)

        #print("Reflection hopefuly")

        #print(L)
        #print(N.T)
        #print(Ref)
        L1= np.squeeze(np.asarray(L))
        N1= np.squeeze(np.asarray(N.T))
        Ref = (2*(N1*L1)*N1)-L1 #Get the reflection
        Ref = Ref / np.linalg.norm(Ref)
        #print(Ref)
        Lights = np.append(Lights, np.array([Ref]),axis=0)

    print(Lights)
    os.chdir("/home/pi/gphoto/PhotometricCameraControl/")
    np.savetxt("lights.txt", Lights, fmt="%s")
    

