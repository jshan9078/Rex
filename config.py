# config.py

# Motor Driver Pin Configurations
MOTOR_A_PINS = (5, 6, 12)  # Motor A: AIN1=GPIO 5, AIN2=GPIO 6, PWMA=GPIO 12 (PWM0)
MOTOR_B_PINS = (23, 24, 13)  # Motor B: BIN1=GPIO 23, BIN2=GPIO 24, PWMB=GPIO 13 (PWM1)

# Motor Driver Standby Pin (if used, but it might not be needed based on the comment)
MOTOR_DRIVER_STBY = 7  # GPIO 7 (CE1)

# Ultrasonic Sensor Pin Configurations
ULTRASONIC_TRIG = 17  # GPIO 17
ULTRASONIC_ECHO = 4   # GPIO 4

# MPU6050 Pin Configurations (I2C)
MPU_SDA = 2  # GPIO 2 (SDA)
MPU_SCL = 3  # GPIO 3 (SCL)
MPU_VCC = 1  # 3.3V power

# Power Pins
MOTOR_DRIVER_VCC = 2  # 5V power for motor driver
ULTRASONIC_VCC = 4  # 5V power for ultrasonic sensor
MOTOR_DRIVER_GND = 6  # Ground pin for motor driver
ULTRASONIC_GND = 9  # Ground pin for ultrasonic sensor
MOTOR_DRIVER_BATTERY_GND = 14  # Ground pin for motor driver battery

# PWM Frequency for Motors
PWM_FREQUENCY = 1000  # Hz

# Other Motor Configurations
MOTOR_MAX_SPEED = 100  # Maximum motor speed (percentage)
MOTOR_MIN_SPEED = 0  # Minimum motor speed (percentage)
