import time
import datetime
import adafruit_gps

_IsWriteF = False
_writeF = None

def startRecord():
	global _writeF
	global _IsWriteF
	_writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".txt","w")
	_IsWriteF = True
	Tempstr = "Time"
	Tempstr += "\t\t\tLatitude\tLongitude\tAltitude"
	Tempstr += "\tSpeed (km/hr)"
	Tempstr += "\taccX (m/s^s)"
	Tempstr += "\taccY (m/s^s)"
	Tempstr += "\taccZ (m/s^s)"
	Tempstr += "\tangleX (m/s^s)"
	Tempstr += "\tangleY (m/s^s)"
	Tempstr += "\tangleZ (m/s^s)"
	Tempstr += "\tSatellites\n"
	_writeF.write(Tempstr)

def endRecord():
	global _writeF
	global _IsWriteF
	_IsWriteF = False
	_writeF.close()

def deviceWrite(gps, device, prt):
	if prt:
		print("Latitude:"+str(gps.latitude)
			, "Longitude:"+str(gps.longitude)
			, "Altitude:" +str(gps.altitude_m)
			, "\tSpeed:" +str(gps.speed_kmh)
			, "Satellites:" + str(gps.satellites)
			, "\r\n"
			, "AccX:" +str(device.getDeviceData("accX"))
			, "AccY:" +str(device.getDeviceData("accY"))
			, "AccZ:" +str(device.getDeviceData("accZ"))
			, "Roll:" +str(device.getDeviceData("gyroX"))
			, "Pitch?:" +str(device.getDeviceData("gyroY"))
			, "Yaw?:" + str(device.getDeviceData("gyroZ"))
			, "\r\n"
    		)

	if (_IsWriteF):    #Record data
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
		_writeF.write(Tempstr)
