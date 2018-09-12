#!/usr/bin/python

##oled adafruit library installed. use Baby Blocks by Colorful Typhoon font from dafont.com/bitmap for the boot logo screen
##then use nice clear monospaced font for system info text. heartbeat asterisk!

#use and log gps time AND system time, and carry on logging without gps time if it fails

#oled or touch screen display? i^2c#!/usr/bin/python

#use and log gps time AND system time, and carry on logging without gps time if it fails

#oled or touch screen display? i^2c
#wifi?
#sql sqlite or csv - webgui if wifi?
#ws2812b status led / just rgb led / gps status, logging status, power status
#button for wifi on or something else
#button to zero all the sensors such as compass and gyro/accel, so that if mounted on uneven surface it will still be all good
#plotly upload
#custom server upload? mysql php etc, or flask? !!!!!!!!
#work out time taken to take reading in total (for optimisation and debugging only)
##pi cpu temp
#add some params as well as a config file for headless start
#log battery level% and voltage and do safe shutdown and notify of low batt
###SMARTPHONE APP - BLUETOOTH?
##log potholes for drivers or cyclists - press a button to confirm

######make slower things like the dht11 run only once every 10 loops (for example) so as to not slow down the GPS and other sensor readings

##MAKE THIS AN ACTUAL THING?

#############HARDWARE IDEAS
#make a little shield for the sensors - dont bother having headers for every single pin, just the i^2c and spi and power and shit at one end, then use some GPIOs at the other end for LEDs or something
#save power by disabling hdmi, onboard led, software
#########!!!!!!!!!!!!1 cron job every minute which checks that PDLPi is still running [ps-aux | grep python PDLPi.py]and if it isn't then launches it

#import libraries
from threading import Thread #for background threading the notification led
import time
import datetime
import sys
import Adafruit_DHT
import smbus
import math
import csv
import os
import Adafruit_LSM303 #accelerometer lib
import serial
import pynmea2
from math import radians, cos, sin, asin, sqrt

##oled libraries ###doing oled separately now in its own python cron
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
RST = None     # on the PiOLED this pin isnt used
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

##init display
disp.begin()

# Clear display.
disp.clear()
disp.display()

##more OLEd setup
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
font = ImageFont.load_default()

draw.text((x, top),       str("      Welcome"),   font=font, fill=255)
draw.text((x, top+8),     str("              to"),   font=font, fill=255)
draw.text((x, top+16),    str("                 PDL!"),   font=font, fill=255)
draw.text((x, top+25),    str("v0.8"),   font=font, fill=255)
# Display image.
disp.image(image)
disp.display()
time.sleep(5)


#enable loggers
plotly = 0
temphum = 1
gyro = 0 #not connected
gps = 1 #don't allow this to be disabled? daft logging withhout location data, though maybe still allow to monitor environment over time, rather than env over space
lux = 1 #BH1750FVI
logsizelog = 1
logfreespace = 1
logaccel = 1 #LSM303
logcputemp = 1
looptime = 1

#configure settings
debugdisplay = 1
loginterval = 0.01 #minimum of one I recon, so as to not overload the dht11 sensor (and maybe other sensors) - work out how to run something in a background thread in python
dhtintervalV = 20
gpsport = "/dev/ttyAMA0"  # Raspberry Pi GPIO serial


#set up variables
logstring = ""
currlogsize = 0

#record start time of script
starttime = datetime.datetime.now()
#start time in format to write to log file
logstart = time.strftime("%Y-%m-%d,%H-%M-%S")

####bh1750 functions
class BH1750():
    """ Implement BH1750 communication. """
    # Define some constants from the datasheet
    POWER_DOWN = 0x00 # No active state
    POWER_ON   = 0x01 # Power on
    RESET      = 0x07 # Reset data register value
    # Start measurement at 4lx resolution. Time typically 16ms.
    CONTINUOUS_LOW_RES_MODE = 0x13
    # Start measurement at 1lx resolution. Time typically 120ms
    CONTINUOUS_HIGH_RES_MODE_1 = 0x10
    # Start measurement at 0.5lx resolution. Time typically 120ms
    CONTINUOUS_HIGH_RES_MODE_2 = 0x11
    # Start measurement at 1lx resolution. Time typically 120ms
    # Device is automatically set to Power Down after measurement.
    ONE_TIME_HIGH_RES_MODE_1 = 0x20
    # Start measurement at 0.5lx resolution. Time typically 120ms
    # Device is automatically set to Power Down after measurement.
    ONE_TIME_HIGH_RES_MODE_2 = 0x21
    # Start measurement at 1lx resolution. Time typically 120ms
    # Device is automatically set to Power Down after measurement.
    ONE_TIME_LOW_RES_MODE = 0x23

    def __init__(self, bus, addr=0x23):
        self.bus = bus
        self.addr = addr
        self.power_down()
        self.set_sensitivity()

    def _set_mode(self, mode):
        self.mode = mode
        self.bus.write_byte(self.addr, self.mode)

    def power_down(self):
        self._set_mode(self.POWER_DOWN)

    def power_on(self):
        self._set_mode(self.POWER_ON)

    def reset(self):
        self.power_on() #It has to be powered on before resetting
        self._set_mode(self.RESET)

    def cont_low_res(self):
        self._set_mode(self.CONTINUOUS_LOW_RES_MODE)

    def cont_high_res(self):
        self._set_mode(self.CONTINUOUS_HIGH_RES_MODE_1)

    def cont_high_res2(self):
        self._set_mode(self.CONTINUOUS_HIGH_RES_MODE_2)

    def oneshot_low_res(self):
        self._set_mode(self.ONE_TIME_LOW_RES_MODE)

    def oneshot_high_res(self):
        self._set_mode(self.ONE_TIME_HIGH_RES_MODE_1)

    def oneshot_high_res2(self):
        self._set_mode(self.ONE_TIME_HIGH_RES_MODE_2)

    def set_sensitivity(self, sensitivity=69):
        """ Set the sensor sensitivity.
            Valid values are 31 (lowest) to 254 (highest), default is 69.
        """
        if sensitivity < 31:
            self.mtreg = 31
        elif sensitivity > 254:
            self.mtreg = 254
        else:
            self.mtreg = sensitivity
        self.power_on()
        self._set_mode(0x40 | (self.mtreg >> 5))
        self._set_mode(0x60 | (self.mtreg & 0x1f))
        self.power_down()

    def get_result(self):
        """ Return current measurement result in lx. """
        data = self.bus.read_word_data(self.addr, self.mode)
        count = data >> 8 | (data&0xff)<<8
        mode2coeff =  2 if (self.mode & 0x03) == 0x01 else 1
        ratio = 1/(1.2 * (self.mtreg/69.0) * mode2coeff)
        return ratio*count

    def wait_for_result(self, additional=0):
        basetime = 0.018 if (self.mode & 0x03) == 0x03 else 0.128
        time.sleep(basetime * (self.mtreg/69.0) + additional)

    def do_measurement(self, mode, additional_delay=0):
        """
        Perform complete measurement using command
        specified by parameter mode with additional
        delay specified in parameter additional_delay.
        Return output value in Lx.
        """
        self.reset()
        self._set_mode(mode)
        self.wait_for_result(additional=additional_delay)
        return self.get_result()

    def measure_low_res(self, additional_delay=0):
        return self.do_measurement(self.ONE_TIME_LOW_RES_MODE, additional_delay)

    def measure_high_res(self, additional_delay=0):
        return self.do_measurement(self.ONE_TIME_HIGH_RES_MODE_1, additional_delay)

    def measure_high_res2(self, additional_delay=0):
        return self.do_measurement(self.ONE_TIME_HIGH_RES_MODE_2, additional_delay)

####bh1750 functions end

##gps speed functions
def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles #####add an option for mph instead of kph
    return c * r

def getGPSspeed(startt, endt, slat, slon, elat, elon):
	FMT = '%H:%M:%S' #time format
	gpstimediff = datetime.datetime.strptime(endt, FMT) - datetime.datetime.strptime(startt, FMT) #get time between gps points
	gpstimediff = int(gpstimediff.seconds)
	gpsdistance = haversine(slat, slon, elat, elon)
	gpsspeed = (gpsdistance / gpstimediff) * 2236.94
	return gpsspeed

##gps speed functions end
##############################functions


def doLEDNotify(style):
	##this needs to run as a background thread so any flashing doesn't take up cpu time in the loop
	##off as a style!
	#if style == "warning":
		#background_thread = Thread(target=ledfunctionname, args=(ledfunctionagrs,))
		#background_thread.start()
		#do a warning led seq
	return

def getLux():
	bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
	sensor = BH1750(bus)
	luxmeasurements = ""

	#print "Sensitivity: {:d}".format(sensor.mtreg)

	for measurefunc, name in [(sensor.measure_low_res, "Low Res "),
								(sensor.measure_high_res, "HighRes "),
								(sensor.measure_high_res2, "HighRes2")]:
	#	print "{} Light Level : {:3.2f} lx".format(name, measurefunc())
		luxmeasurements += "{},".format(measurefunc())
	sensor.set_sensitivity((sensor.mtreg + 10) % 255)

	return luxmeasurements

##get pi cpu temperature
def getCPUtemperature():
	res = os.popen('vcgencmd measure_temp').readline()
	return(res.replace("temp=","").replace("'CC\n",""))
	temp1=int(float(getCPUtemperature()))
	print temp1,"C"
	updateOLED("CPU Temp: " + str(temp1), 1)
	return temp1

##get LSM303DLHC accelerometer readings
def getAccelerometer():
	accel, mag = lsm303.read()
    # Grab the X, Y, Z components from the reading and print them out.
	accel_x, accel_y, accel_z = accel
	mag_x, mag_z, mag_y = mag
	compassdir = math.atan2(mag_x,mag_y)*180/math.pi
	#print('Accel X={0}, Accel Y={1}, Accel Z={2}, Mag X={3}, Mag Y={4}, Mag Z={5}'.format(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z))
	currentaccel = str(accel_x) + "," + str(accel_y) + "," + str(accel_z) + "," + str(mag_x) + "," + str(mag_y) + "," + str(mag_z) + "," + str(compassdir)
	return currentaccel

#function to get fs free space
def get_fs_freespace():
    #"Get the free space of the filesystem containing pathname"
    stat= os.statvfs('/')
    # use f_bfree for superuser, or f_bavail if filesystem
    # has reserved space for superuser
    return stat.f_bfree*stat.f_bsize

# Return CPU temperature as a character string
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=","").replace("'C\n",""))

def writetolog(stringtolog):
	with open(("/home/pi/pdlpi/drips/" + logstart + ".csv"), 'a') as f:
		f.write(stringtolog + "\n")
		f.close()

def getlogsize():
	curlogsize = os.path.getsize(("/home/pi/pdlpi/drips/" + logstart + ".csv"))
	return curlogsize

def readLine(gpsport):
    s = ""
    while True:
        ch = gpsport.read()
        s += ch
        if ch == '\r':
            return s

def updateOLED(textline, lineno):
    disp.clear()
    disp.display()
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    if lineno == 1:
        draw.text((x, top),       str(textline),   font=font, fill=255)
    if lineno == 2:
        draw.text((x, top+8),     str(textline),   font=font, fill=255)
    if lineno == 3:
        draw.text((x, top+16),    str(textline),   font=font, fill=255)
    if lineno == 4:
        draw.text((x, top+25),    str(textline),   font=font, fill=255)
    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(.1)

def readGPS():
    #time.sleep(1)
    ser.write("A")
    try:
        rcv = readLine(ser)
#    print "received:", rcv
	nmeacode =  pynmea2.parse(rcv)
#    print pynmea2.parse(rcv)
	gpstime = nmeacode.timestamp
	gpslat = nmeacode.latitude
	gpslatdir = nmeacode.lat_dir
	gpslon = nmeacode.longitude
	gpslondir = nmeacode.lon_dir
	gpsqual = nmeacode.gps_qual
	gpsats = nmeacode.num_sats
	gpsalt = nmeacode.altitude
	#print gpstime, gpslat, gpslatdir, gpslon, gpslondir, gpsalt, gpsqual, gpsats
	#gpsval = gpstime + "," + gpslat + "," + gpslatdir + "," + gpslon + "," + gpslondir + "," + gpsalt + "," + gpsqual + "," + gpsats
	#gpsval = (gpstime, gpslat, gpslatdir, gpslon, gpslondir, gpsalt, gpsqual, gpsats)
	#print nmeacode.latitude, nmeacode.latitude_minutes, nmeacode.latitude_seconds
	#print nmeacode.longitude, nmeacode.longitude_minutes, nmeacode.longitude_seconds
	return str(gpstime), str(gpslat), str(gpslon), str(gpsalt), str(gpsqual), str(gpsats)
	# return gpsval

    except:
	pass
#        print "signal failure"

##############################functions


##run once at script start
ser = serial.Serial(gpsport, baudrate = 9600) #start gps com port
#record disk space at the start time
startdiskspace = (float(get_fs_freespace()) * 0.000001) #get the free disk space when the script is started


#write out column names in the log file once
columntitles = "date,time"
if (temphum == 1):
	columntitles += ",temperature,humidity"
if (logsizelog == 1):
	columntitles += ",logsize"
if (logfreespace == 1):
	columntitles += ",freespace"
if (logaccel == 1):
	columntitles += ",acceleration_x,acceleration_y,acceleration_z,mag_x,mag_y,mag_z,compassdir"
if (logcputemp == 1):
	columntitles += ",cputemp"
if (lux == 1):
	columntitles += ",lux_lr,lux_hr,lux_hr2"
if (gps == 1):
	columntitles += ",gpstime,gpslat,gpslon,gpsalt,gpsqual,gpsats,gpsspeed"
if (looptime == 1):
	columntitles += ",looptime"
writetolog(columntitles)
print columntitles #show the column titles in the cli

dht11counter = 0
olddht11logstring = "0,0"
#main logging loop##############################################################################################################################
try:
	gpscounter = 0 #counter to see if we're on the start or end lat lon value for GPS speed calculations
	itercounter = 0 #counter for number of iterations
	prevgps = () #set up the previous gps var
	while True:
		if (looptime == 1):
			looptimelog = time.time()

		#log timestamp always
		logstring += str(time.strftime("%Y-%m-%d,%H:%M:%S")) + ","

		#if the dht11 is enabled
		if (temphum == 1):
			if (dht11counter >= dhtintervalV):
				humidity, temperature = Adafruit_DHT.read_retry(11, 4)
				logstring += str(temperature) + "," + str(humidity)
				olddht11logstring = str(temperature) + "," + str(humidity)
				dht11counter = 0
			else:
				logstring += olddht11logstring
				pass
		dht11counter = dht11counter + 1


		#log the current size of the log file (in bytes I think, convert to kb)
		if (logsizelog == 1):
			currlogsize = (float(getlogsize()) / 1000) #convert to kb
			logstring += "," + str(currlogsize)

		if (logfreespace == 1):
			#freespace = (float(get_fs_freespace()) * 0.000001) # convert to mb - get a reading of the free disk space each time
			freespace = (float(startdiskspace) - float(currlogsize)) #calculated rather than measured
			logstring += "," + str(freespace)

		if (logaccel == 1):
			lsm303 = Adafruit_LSM303.LSM303() # Create a LSM303 instance. (MAYBE MOVE THIS TO THE GetAccelerometer FUNCTION ABOVE?)
			logstring += "," + getAccelerometer()

		if (logcputemp == 1):
			cputemp = getCPUtemperature()
			logstring += "," + cputemp

		#if the BH1750 lux sensor is enabled
		if (lux == 1):
			luxvalue = str(getLux())
			logstring += "," + luxvalue


		oldgpsspeed = 0
		if (gps == 1): #if the gps is enabled

			currgps = readGPS()
			if type(currgps) is tuple: #if its a valid gps nmea signal
				prevgps = str(currgps[0] + "," + currgps[1] + "," + currgps[2] + "," + currgps[3] + "," + currgps[4] + "," + currgps[5])
				gpsvalue = currgps[0] + "," + currgps[1] + "," + currgps[2] + "," + currgps[3] + "," + currgps[4] + "," + currgps[5]
				logstring += str(gpsvalue) + "," #log the formatted gps string

				##begin the gps speed calc here with dummy values to get the proper smooth speed between pints rather than between pairs of points

				if (gpscounter == 0): #store lat,lon,time 1
					t1 = currgps[0]
					la1 = currgps[1]
					lo1 = currgps[2]
					gpscounter = 1
					logstring += str(oldgpsspeed)

				elif (gpscounter == 1):  #store lat,lon,time 2
					t2 = currgps[0]
					la2 = currgps[1]
					lo2 = currgps[2]
					gpscounter = 0

					#####GPS SPEED CALCULATORRRRRRRRRRRRRRRRR
					#once we have both values, calculate the speed
#					currgpsspeed = getGPSspeed(t1, t2, float(la1), float(lo1), float(la2), float(lo2))
#					logstring += str(currgpsspeed) + "," #log the speed if there is any
#					oldgpsspeed = str(currgpsspeed) + "," #log the speed if there is any

			else: #if the current string isn't a tuple, which normal NMEA strings are, then just reuse the previous GPS values. use this to determine when we haven't seen any satellites yet, and then slow down the rate of sampling the sensors
				gpsvalue = prevgps #return the previous value if it doesn't return anything new
				logstring += str(gpsvalue) + "," + str(oldgpsspeed) + ","

		if (looptime == 1):
			logstring += str((time.time() - looptimelog))

		#write to log file
		writetolog(logstring)
		if (debugdisplay == 1):
			print logstring
		logstring = "" #clear the log string ready for the next reading
		#wait for the log interval
		time.sleep(loginterval)


		itercounter = itercounter + 1
		if (itercounter >= 30): ##show the column title every 10 rows
			print columntitles
			itercounter = 0



except KeyboardInterrupt:
	print " PDLPi forcefully stopped."
	print "Ran for: " + str(datetime.datetime.now() - starttime)
	print "Created " + str(currlogsize) + "Kb of logs"
	pass
