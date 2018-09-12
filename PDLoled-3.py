import time
import math

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Alternatively you can specify a software SPI implementation by providing
# digital GPIO pin numbers for all the required display pins.  For example
# on a Raspberry Pi with the 128x32 display you might use:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, dc=DC, sclk=18, din=25, cs=22)
import subprocess
import glob
import os

def readlastline():
    list_of_files = glob.glob('/home/pi/pdlpi/drips/*.csv') # * means all if need specific format then *.csv
    latest_file = max(list_of_files, key=os.path.getctime)
    #print latest_file
    line = subprocess.check_output(['tail', '-1', latest_file])
    #print line
    return line

def splitline(inputline):
    latestsplitline = inputline.split(",")
    return latestsplitline

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
#font = ImageFont.load_default()

# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
font = ImageFont.truetype('/home/pi/pdlpi/visitor1.ttf', 11)

global currentlines

heartbeat = 0
def showheartbeat():
    if heartbeat == 0:
        draw.text((x, top+25),  str("                  " + str("*")),  font=font, fill=255) #draw page number
        heartbeat += 1
    if heartbeat == 1:
        draw.text((x, top+25),  str("                    " + str(" ")),  font=font, fill=255) #draw page number
        heartbeat = 0

def getlastlogline():
    global currentlines
    splitline(readlastline())

    ##"date,time,temperature,humidity,logsize,freespace,acceleration_x,acceleration_y,acceleration_z,mag_x,mag_y,mag_z,compassdir,cputemp,lux_lr,lux_hr,lux_hr2,gpstime,gpslat,gpslon,gpsalt,gpsqual,gpsats,gpsspeed,looptime"
    currentlines = splitline(readlastline())

def displaylines(line1, line2, line3, line4, page):
    global currentlines
    ##clear the SCREEN
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    draw.text((x, top),       str(line1),  font=font, fill=255)
    draw.text((x, top+8),     str(line2),  font=font, fill=255)
    draw.text((x, top+16),    str(line3),  font=font, fill=255)
    if page != "":
        draw.text((x, top+25),  str("                  [" + str(page) + "]"),  font=font, fill=255) #draw page number
    draw.text((x, top+25),    str(line4),  font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()


displaylines("_____ ______    ", "|  _  |    \|  |   ", "|   __|  |  |  |__ ", "|__|  |____/|_____|","")
time.sleep(5)

pageindex = [{"this","is","pdl"},{"this","is","other"}]

countpage = 0
def countpage():
    global countpage
    countpage += 1
    if countpage >= 10:
        return "poo"

def modfilesize(fsinput):
    fsinput = math.trunc(float(fsinput))
    if fsinput > 1024:
        return str(fsinput/1024) + "Mb"
    else:
        return str(fsinput) + "Kb"

page2 = 0
while True:

    #multiple pages that are cyced through automatically. show current page and total in bottom right corner. also show heartbeats pulse.
    #for pages in pageindex:
        #print pages
    getlastlogline() # get the last line from csv
    try:
        displine1 = (str(currentlines[17] + " Co:" + str(math.trunc(float(currentlines[12])))))
    except:
        displine1 = "Missing info"

    try:
        displine2 = (str("La:" + currentlines[18]))
    except:
        displine2 = "Missing info"

    try:
        displine3 = (str("Lo:" + currentlines[19]))
    except:
        displine3 = "Missing info"

    try:
        #fs = math.trunc(float(currentlines[4]))
        displine4 = ("T:" + str(currentlines[2] + " LS:" + str(modfilesize(currentlines[4]))))
    except:
        displine4 = "Missing info"

    #print displine1 + "\n" + displine2 + "\n" + displine3 + "\n" + displine4 + "\n" + str(page2) #print with page number
    print displine1 + "\n" + displine2 + "\n" + displine3 + "\n" + displine4 + "\n"  #print without page number
    #displaylines(displine1, displine2, displine3, displine4, page2)
    displaylines(displine1, displine2, displine3, displine4, "")

    time.sleep(.5)
    page2 += 1 #page counter
