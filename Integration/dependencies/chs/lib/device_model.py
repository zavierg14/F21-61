# coding:UTF-8
import threading
import _thread
import time
import struct
import serial
from serial import SerialException
'''
    Serial Port configuration
'''


class SerialConfig:
    # port
    portName = '/dev/ttyAMA0'

    # Baud rate
    baud = 115200

'''
Device model
'''


class DeviceModel:
    # Device Name
    deviceName = "My Device"

    # Device ID
    ADDR = 0x50

    # Device Data Dictionary
    deviceData = {}

    # Whether it is stuck or not
    isOpen = False

    # Serial Port
    serialPort = None

    # Serial Port Configuration
    serialConfig = SerialConfig()

    # Update Trigger
    dataUpdateListener = ""

    # Data parser
    dataProcessor = None

    # Protocol Parser
    protocolResolver = None

    def __init__(self, deviceName, protocolResolver, dataProcessor, dataUpdateListener):
        #print("Initialize Device Module")
        self.deviceName = deviceName
        self.protocolResolver = protocolResolver
        self.dataProcessor = dataProcessor
        self.dataUpdateListener = dataUpdateListener
        # _thread.start_new_thread(self.readDataTh, ("Data-Received-Thread", 10, ))

    def setDeviceData(self, key, value):
        """
        Set Device Data
        :param key: Data Key
        :param value: Data Value
        :return: No return
        """
        self.deviceData[key] = value

    def getDeviceData(self, key):
        """
        Get device data
        :param key: Data key
        :return: Returns the data value. If the data key does not exist, it returns None
        """
        if ( key in self.deviceData):
            return self.deviceData[key]
        else:
            return None

    def removeDeviceData(self, key):
        """
        Delete Device Data
        :param key: Data key
        :return: No return
        """
        del self.deviceData[key]

    def readDataTh(self, threadName, delay):
        """
        Read Data
        :return:
        """
        #print("Start" + threadName)
        while True:
            # if the serial port is open
            if self.isOpen:
                try:
                    tlen = self.serialPort.inWaiting()
                    if (tlen>0):
                        data = self.serialPort.read(tlen)
                        self.onDataReceived(data)
                except Exception as ex:
                    print(ex)
            else:
                time.sleep(0.1)
                #print("Pause")
                break

    def openDevice(self):
        """
        Open
        :return: No return
        """

        # close the port first
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(self.serialConfig.portName, self.serialConfig.baud, timeout=0.5)
            self.isOpen = True
            t = threading.Thread(target=self.readDataTh, args=("",10,))          # Start a thread to recieve data
            t.start()
            #print("Open Succeeded")
        except SerialException:
            print("Open" + self.serialConfig.portName + self.serialConfig.baud + "failed")

    def closeDevice(self):
        """
        Turn off the device
        :return: No return
        """
        if self.serialPort is not None:
            self.isOpen = False
            self.serialPort.close()
            #print("The port is closed")
        self.isOpen = False
        #print("The device is turned off")

    def onDataReceived(self, data):
        """
        When recieving
        :param data: recieved
        :return: No return
        """
        if self.protocolResolver is not None:
            self.protocolResolver.passiveReceiveData(data, self)

    def get_int(self,dataBytes):
        """
        intconverter   = C# BitConverter.ToInt16
        :param dataBytes: array
        :return:
        """
        #return -(data & 0x8000) | (data & 0x7fff)
        return  int.from_bytes(dataBytes, "little", signed=True)

    def get_unint(self,dataBytes):
        """
        int conversion to unsigned
        :param data:
        :return:
        """
        return  int.from_bytes(dataBytes, "little")


    def sendData(self, data):
        """
        send data
        :return: whther the data was sent successfully
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data, self)

    def readReg(self, regAddr,regCount):
        """
        Read Register
        :param regAddr: register address
        :param regCount: number of registers
        :return:
        """
        if self.protocolResolver is not None:
            return self.protocolResolver.readReg(regAddr,regCount, self)
        else:
            return none

    def writeReg(self, regAddr,sValue):
        """
        Write register
        :param regAddr: register address
        :param sValue: write value
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.writeReg(regAddr,sValue, self)

    def unlock(self):
        """
        Unlock
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.unlock(self)

    def save(self):
        """
        Save
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.save(self)

    def AccelerationCalibration(self):
        """
        Acceleration Calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.AccelerationCalibration(self)

    def BeginFiledCalibration(self):
        """
        Start magnetic field calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.BeginFiledCalibration(self)

    def EndFiledCalibration(self):
        """
        EndMagnetic Field
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.EndFiledCalibration(self)

    def sendProtocolData(self, data):
        """
        Send Data with Protocol
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data)


