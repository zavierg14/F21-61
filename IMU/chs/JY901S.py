# coding:UTF-8
"""
    Test file
"""
import time
import datetime
import platform
import struct
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

welcome = """
    Welcome to the Wit-Motoin sample program
"""
_writeF = None                    #写文件  Write file
_IsWriteF = False                 #写文件标识    Write file identification

def readConfig(device):
    """
        Example of reading configuration information
    :param device:  Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  # Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("Return result：" + str(tVals))
    else:
        print("No Return")
    tVals = device.readReg(0x23,2)  #  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("Return Result：" + str(tVals))
    else:
        print("No Return")

def setConfig(device):
    """
    Example setting configuration information
    :param device: Device model
    :return:
    """
    device.unlock()                #  unlock
    time.sleep(0.1)                #  Sleep 100ms
    device.writeReg(0x03, 6)       #  Set the transmission back rate to 10HZ
    time.sleep(0.1)                #  Sleep 100ms
    device.writeReg(0x23, 0)       #  Set the installation direction: horizontal and vertical
    time.sleep(0.1)                #  Sleep 100ms
    device.writeReg(0x24, 0)       #  Set the installation direction: nine axis, six axis
    time.sleep(0.1)                #  Sleep 100ms
    device.save()                  #  Save

def AccelerationCalibration(device):
    """
    Acceleration calibration
    :param device: Device model
    :return:
    """
    device.AccelerationCalibration()                 # Acceleration calibration
    print("Acceleration Calibration Completed")

def FiledCalibration(device):
    """
    Magnetic field calibration
    :param device: Device model
    :return:
    """
    device.BeginFiledCalibration()                   #   Starting field calibration
    if input("Please rotate the three acex slowly around the XYZ acis and then the calibration is completed (Y/N)？").lower()=="y":
        device.EndFiledCalibration()                 #   End field calibration
        print("End of field calibration")

def onUpdate(deviceModel):
    """
    Data update event
    :param deviceModel:  Device model
    :return:
    """
    print("Chiptime:" + str(deviceModel.getDeviceData("Chiptime"))
         , " Temperature:" + str(deviceModel.getDeviceData("temperature"))
         , " Acceleration：" + str(deviceModel.getDeviceData("accX")) +","+  str(deviceModel.getDeviceData("accY")) +","+ str(deviceModel.getDeviceData("accZ"))
         ,  " Angular Velocity:" + str(deviceModel.getDeviceData("gyroX")) +","+ str(deviceModel.getDeviceData("gyroY")) +","+ str(deviceModel.getDeviceData("gyroZ"))
         , " Angle:" + str(deviceModel.getDeviceData("angleX")) +","+ str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ"))
        , " Magnetic Field:" + str(deviceModel.getDeviceData("magX")) +","+ str(deviceModel.getDeviceData("magY"))+","+ str(deviceModel.getDeviceData("magZ"))
        , " Position:" + str(deviceModel.getDeviceData("lon")) + " latitude:" + str(deviceModel.getDeviceData("lat"))
        , " Yaw and Speed:" + str(deviceModel.getDeviceData("Yaw")) + " ground speed:" + str(deviceModel.getDeviceData("Speed"))
         , " Four Elements:" + str(deviceModel.getDeviceData("q1")) + "," + str(deviceModel.getDeviceData("q2")) + "," + str(deviceModel.getDeviceData("q3"))+ "," + str(deviceModel.getDeviceData("q4"))
          )
    if (_IsWriteF):    # Record data
        Tempstr = " " + str(deviceModel.getDeviceData("Chiptime"))
        Tempstr += "\t"+str(deviceModel.getDeviceData("accX")) + "\t"+str(deviceModel.getDeviceData("accY"))+"\t"+ str(deviceModel.getDeviceData("accZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("angleX")) +"\t" + str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("temperature"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("magX")) +"\t" + str(deviceModel.getDeviceData("magY")) +"\t"+ str(deviceModel.getDeviceData("magZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("lon")) + "\t" + str(deviceModel.getDeviceData("lat"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("Yaw")) + "\t" + str(deviceModel.getDeviceData("Speed"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q1")) + "\t" + str(deviceModel.getDeviceData("q2"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q3")) + "\t" + str(deviceModel.getDeviceData("q4"))
        Tempstr += "\r\n"
        _writeF.write(Tempstr)

def startRecord():
    """
    Start recording data
    :return:
    """
    global _writeF
    global _IsWriteF
    _writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")    #新建一个文件
    _IsWriteF = True                                                                        #标记写入标识
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\tlon\tlat"
    Tempstr += "\tYaw\tSpeed"
    Tempstr += "\tq1\tq2\tq3\tq4"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    print("Start Recording Data")

def endRecord():
    """
    End record data
    :return:
    """
    global _writeF
    global _IsWriteF
    _IsWriteF = False             #  Tag cannot write the identity
    _writeF.close()               #  Close file
    print("End of recording data")

if __name__ == '__main__':

    print(welcome)
    """
       Initialize a device model
    """
    device = deviceModel.DeviceModel(
        "My Jy901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = "/dev/ttyAMA0"   #   Set serial port
    else:
        device.serialConfig.portName = "COM17"          #   Set serial port
    device.serialConfig.baud = 9600                     #  Set baud rate
    device.openDevice()                                 #   Open serial port
    readConfig(device)                                  # Read configuration information
    device.dataProcessor.onVarChanged.append(onUpdate)  # Data update event

    startRecord()                                       #     Start recording data
    input()
    device.closeDevice()
    endRecord()                                         # End record data
