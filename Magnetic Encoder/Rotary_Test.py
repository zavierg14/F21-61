import smbus2
import time

AS5600_ADDR = 0x36
RAW_ANGLE_REG = 0x0C

def read_angle():
    bus = smbus2.SMBus(1)
    data = bus.read_i2c_block_data(AS5600_ADDR, RAW_ANGLE_REG, 2)
    bus.close()

    raw_angle = (data[0] << 8) | data[1]

    angle = (raw_angle / 4096.0) * 360.0
    return angle

if __name__ == "__main__":
    try:
        while True:
            angle = read_angle()
            print(f"Angle: {angle:.2f} deg")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting")
