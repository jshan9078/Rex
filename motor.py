import RPi.GPIO as GPIO
import time
from config import MOTOR_A_PINS, MOTOR_B_PINS, PWM_FREQUENCY

class MotorDriver:
    def __init__(self, in1, in2, pwm_pin, pwm_freq=1000):
        """
        Initializes the motor driver for one motor.

        :param in1: GPIO pin connected to IN1 (direction control)
        :param in2: GPIO pin connected to IN2 (direction control)
        :param pwm_pin: GPIO pin connected to PWMA or PWMB (PWM control)
        :param pwm_freq: PWM frequency (default: 1000 Hz)
        """
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        # Setup PWM
        self.pwm = GPIO.PWM(self.pwm_pin, pwm_freq)
        self.pwm.start(0)  # Start with PWM off

    def set_speed(self, speed):
        """
        Sets the motor speed.

        :param speed: Motor speed (0-100), positive for forward, negative for reverse
        """
        if speed > 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

        self.pwm.ChangeDutyCycle(abs(speed))

    def stop(self):
        """
        Stops the motor.
        """
        self.set_speed(0)

    def cleanup(self):
        """
        Clean up GPIO resources.
        """
        self.stop()
        self.pwm.stop()
        GPIO.cleanup()


class DualMotorDriver:
    def __init__(self, motor_a_pins, motor_b_pins):
        """
        Initializes the driver for two motors.

        :param motor_a_pins: Tuple (in1, in2, pwma) for Motor A
        :param motor_b_pins: Tuple (in1, in2, pwmb) for Motor B
        """
        self.motor_a = MotorDriver(*motor_a_pins)
        self.motor_b = MotorDriver(*motor_b_pins)

    def set_motor_a_speed(self, speed):
        self.motor_a.set_speed(speed)

    def set_motor_b_speed(self, speed):
        self.motor_b.set_speed(speed)

    def stop_all(self):
        self.motor_a.stop()
        self.motor_b.stop()

    def cleanup(self):
        self.motor_a.cleanup()
        self.motor_b.cleanup()


# Test Main App
if __name__ == "__main__":
    # Define the pin connections for Motor A and Motor B
    motor_a_pins = MOTOR_A_PINS
    motor_b_pins = MOTOR_B_PINS

    # Create an instance of the DualMotorDriver
    motors = DualMotorDriver(motor_a_pins, motor_b_pins)

    try:
        # Test Motor A forward at 50% speed
        print("Motor A forward at 50% speed")
        motors.set_motor_a_speed(50)
        time.sleep(2)

        # Test Motor A backward at 75% speed
        print("Motor A backward at 75% speed")
        motors.set_motor_a_speed(-75)
        time.sleep(2)

        # Test Motor B forward at 60% speed
        print("Motor B forward at 60% speed")
        motors.set_motor_b_speed(60)
        time.sleep(2)

        # Test Motor B backward at 40% speed
        print("Motor B backward at 40% speed")
        motors.set_motor_b_speed(-40)
        time.sleep(2)

        # Stop all motors
        print("Stopping all motors")
        motors.stop_all()

    finally:
        # Cleanup GPIO resources
        print("Cleaning up GPIO")
        motors.cleanup()
