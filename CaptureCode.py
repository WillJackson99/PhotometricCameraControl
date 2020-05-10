from time import sleep
from datetime import datetime
from sh import gphoto2 as gp
import signal, os, subprocess
import RPi.GPIO as GPIO
import time

#kill camera app first

def killgphoto2Process():
    p = subprocess.Popen(['ps', '-A'],stdout = subprocess.PIPE)
    out, err = p.communicate()
    #search for kill line
    for line in out.splitlines():
        if b'gvfsd-gphoto2' in line:
            pid = int(line.split(None, 1) [0])
            os.kill(pid, signal.SIGKILL)



clearCommand = ["--folder","/store_00020001/DCIM/100CANON", \
                "-R", "--delete-all-files"]

triggerCommand = ["--trigger-capture"]

downloadCommand = ["--get-all-files"]


def captureImages(i):
    try:
        gp(triggerCommand)
        sleep(0.2)
        GPIO.output(GPIOpins[i], GPIO.LOW)
        sleep(0.25)
        gp(downloadCommand)
        gp(clearCommand)
    except:
        for i in range(0,8):
            GPIO.output(GPIOpins[i], GPIO.LOW)
            
def createSaveFolder(save_location):
    try:
        os.makedirs(save_location)
    except:
        print("failed dir init")
    os.chdir(save_location)

def renameFiles():
    ID = 0
    for filename in os.listdir("."):
        if len(filename) < 13:
            if filename.endswith(".JPG"):
                os.rename(filename,(str(ID)+".JPG"))
                print("Renamed the JPG")
            elif filename.endswith(".CR2"):
                os.rename(filename,(str(ID)+".CR2"))
        ID += 1

#================================--Program Start--================================
killgphoto2Process()
GPIOpins = [27,17,22,23,4,14,15,18]
#createSaveFolder()

def captureCycle():
    gp(clearCommand)
    GPIOpins = [27,17,22,23,4,14,15,18]

    for i in range(0,8):
        GPIO.output(GPIOpins[i], GPIO.HIGH)
        captureImages(i)
        
    renameFiles()
    
def captureData():
    
    shot_time = datetime.now().strftime("%Y-%m-%d@%H:%M:%S")
    picID = "piShots"
    folder_name = shot_time + picID
    save_location = "/home/pi/gphoto/PhotometricCameraControl/data/" + folder_name
    createSaveFolder(save_location)
    captureCycle()

    return(save_location + '/')
    
