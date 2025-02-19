import time		# idek if i use this lol
import datetime		# Used to name file
import adafruit_gps	# ~Parses the GPS~
import csv

_IsWriteF = False	# Global variable to tell deviceWrite whether there is an open file
_writeF = None		# Global variable to tell deviceWrite what file to write to

# A fancy shamncy function to open a text file and give data headers to work with
def startRecord():
	global _writeF			# imports the global variables?
	global _IsWriteF		#  ^^^^^^^
	_writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".txt","w")	# Opens file and names it the current year,month,day,hour,and second - assigns to _writeF
	_IsWriteF = True		# Tells the other functions we have an open file
	Tempstr = "Time"		# Begins the string to print into file
	Tempstr += "\t\t\tLatitude\tLongitude\tAltitude"	# Continues string
	Tempstr += "\tSpeed (km/hr)"				# Continues string
	Tempstr += "\taccX (m/s^s)"				# Continues string
	Tempstr += "\taccY (m/s^s)"				# Continues string
	Tempstr += "\taccZ (m/s^s)"				# Continues string
	Tempstr += "\tangleX (m/s^s)"				# Continues string
	Tempstr += "\tangleY (m/s^s)"				# Continues string
	Tempstr += "\tangleZ (m/s^s)"				# ^^^^^^
	Tempstr += "\tSatellites\n"				# We get it at this point
	_writeF.write(Tempstr)					# Writes that string to the file yippee!

# Function to close file when were all done
def endRecord():
	global _writeF		# Imports variables
	global _IsWriteF	# Imports variables
	_IsWriteF = False	# NOT writing anymore
	_writeF.close()		# File is closed!

# This is the function of all time- it actually pulls the data from sensors and prints it to terminal and file
def deviceWrite(gps, device, prt):	# Pass in gps and device objects and True/False on whether to print to console
	if prt:				# IF print is true

		# Huge long print statement... basically gps."insert your thing here" calls a gps value
		# and device.getDeviceData("insert your thing here") calls imu data
		# theres also some formatting in here
		print("Latitude:"+str(gps.latitude)
			, "Longitude:"+str(gps.longitude)
			, "Altitude:" +str(gps.altitude_m)
			, "Speed:" +str(gps.speed_kmh)
			, "Satellites:" + str(gps.satellites)
		#	, "\r\n"
			, "AccX:" +str(device.getDeviceData("accX"))
			, "AccY:" +str(device.getDeviceData("accY"))
			, "AccZ:" +str(device.getDeviceData("accZ"))
			, "Roll:" +str(device.getDeviceData("angleX"))
			, "Pitch?:" +str(device.getDeviceData("angleY"))
			, "Yaw?:" + str(device.getDeviceData("angleZ"))
			, "\r\n"
    		)

	# If that lovely file is open then were gonna write to it
	if (_IsWriteF):

		# Another super long string, same data as before but shoved into a big string
		Tempstr = str("{}/{}/{} {:02}:{:02}:{:02}".format(
				gps.timestamp_utc.tm_mon,
				gps.timestamp_utc.tm_mday,
				gps.timestamp_utc.tm_year,
				gps.timestamp_utc.tm_hour,
				gps.timestamp_utc.tm_min,
				gps.timestamp_utc.tm_sec,
				))
		Tempstr += "\t" +str(gps.latitude)
		Tempstr += "\t"+str(gps.longitude)
		Tempstr += "\t"+str(gps.altitude_m)
		Tempstr += "\t\t"+str(gps.speed_kmh)
		Tempstr += "\t\t"+str(device.getDeviceData("accX"))
		Tempstr += "\t\t"+str(device.getDeviceData("accY"))
		Tempstr += "\t\t"+str(device.getDeviceData("accZ"))
		Tempstr += "\t\t"+str(device.getDeviceData("angleX"))
		Tempstr += "\t\t"+str(device.getDeviceData("angleY"))
		Tempstr += "\t\t"+str(device.getDeviceData("angleZ"))
		Tempstr += "\t\t" +str(gps.satellites)
		Tempstr += "\r\n"
		_writeF.write(Tempstr)	# Writing string to file

# EZPZ function to write list data to a csv file
def csvWrite(data, title, headers):
	with open(title+str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".csv", mode='w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)
	#print("CSV written")



# Drive safe
# Soli deo gloria
#    (()__(()
#    /       \
#   ( /    \  \
#    \ o o    /
#    (_()_)__/ \
#    / _,==.____ \
#   (   |--|      )
#   /\_.|__|'-.__/\_
#  / (        /     \
#  \  \      (      /
#   )  '._____)    /
#(((____.--(((____/

