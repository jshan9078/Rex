import smbus2
import time

class MPU6050:
    MPU6050_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus_number=1):
        # Initialize I2C bus
        self.bus = smbus2.SMBus(bus_number)
        # Wake up MPU6050 as it starts in sleep mode
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def read_accel(self):
        accel_x = self.read_word(self.ACCEL_XOUT_H)
        accel_y = self.read_word(self.ACCEL_XOUT_H + 2)
        accel_z = self.read_word(self.ACCEL_XOUT_H + 4)
        return {'x': accel_x, 'y': accel_y, 'z': accel_z}

    def read_gyro(self):
        gyro_x = self.read_word(self.GYRO_XOUT_H)
        gyro_y = self.read_word(self.GYRO_XOUT_H + 2)
        gyro_z = self.read_word(self.GYRO_XOUT_H + 4)
        return {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}

    def read_all(self):
        # Get both accelerometer and gyroscope data
        accel = self.read_accel()
        gyro = self.read_gyro()
        return {'accel': accel, 'gyro': gyro}

# Usage Example:
if __name__ == "__main__":
    mpu = MPU6050()
    
    while True:
        data = mpu.read_all()
        print(f"Accelerometer: {data['accel']}")
        print(f"Gyroscope: {data['gyro']}")
        time.sleep(1)
