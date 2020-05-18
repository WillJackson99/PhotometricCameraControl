from __future__ import print_function

import cali
import CaptureCode

import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

from time import sleep
from datetime import datetime
from sh import gphoto2 as gp
import signal, os, subprocess
import RPi.GPIO as GPIO
import time
import sh

from rps import RPS
import psutil



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIOpins = [27,17,22,23,4,14,15,18]
for i in range(0,8):
    GPIO.setup(GPIOpins[i], GPIO.OUT)
    GPIO.output(GPIOpins[i], GPIO.LOW)


def process():
    METHOD = RPS.L2_SOLVER 
    DATA_FOLDERNAME = CaptureCode.captureData()
    LIGHT_FILENAME = '/home/pi/gphoto/PhotometricCameraControl/lights.txt'
    MASK_FILENAME = '/home/pi/gphoto/PhotometricCameraControl/data/mask2.png'

    
    rps = RPS()
    rps.load_mask(filename=MASK_FILENAME)    # Load mask image
    rps.load_lighttxt(filename=LIGHT_FILENAME)
    rps.load_images(foldername=DATA_FOLDERNAME,ext='JPG')
    os.chdir(DATA_FOLDERNAME)
    rps.solve(METHOD) 
    rps.save_normalmap(filename="./est_normal")
    psutil.disp_normalmap(normal=rps.N, height=rps.height, width=rps.width)


def menu():
    print("MENU \n 1.Calibrate \n 2.Capture & Process \n 3.EXIT")
    choice = str(input())

    if choice == "1":
        print("Calibrating")
        cali.calibrate()
        menu()

    if choice == "2":
        print("Capturing")
        process()
        menu()

    if choice == "3":
        print("EXITING")

menu()




