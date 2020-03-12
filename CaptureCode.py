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

shot_date = datetime.now().strftime("%Y-%m-%d")
shot_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

picID = "Photometric"

clearCommand = ["--folder","/store__00020001/DCIM/100CANON", \
                "-R", "--delete-all-files"

triggerCommand = ["--trigger-capture"]

downloadCommand = ["--get-all-files"]

folder_name = shot_date + picID
save_location = "/home/pi/Desktop/gphoto" + folder_name

def captureImages():
    gp(triggerCommand)
    sleep(1)
    gp(downloadCommand)
    gp(clearCommand)

def createSaveFolder():
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
                os.rename(filename,(shot_time +"-"+str(ID)+".JPG"))
                print("Renamed the JPG")
            elif filename.endswith(".CR2"):
                os.rename(filename,(shot_time +"-"+str(ID)+".CR2"))
        ID += 1

#================================--Program Start--================================
killgphoto2Process()
gp(clearCommand)
createSaveFolder()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIOpins = [17,27,22,18]
for i in range(0,4):
    GPIO.setup(GPIOpins[i], GPIO.OUT)

for i in range(0,4):
    GPIO.output(GPIOpins[i], GPIO.HIGH)
    captureImages()
    GPIO.output(GPIOpins[i], GPIO.LOW)

renameFiles()
