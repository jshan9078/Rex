import RPi.GPIO as GPIO
import time
from config import MOTOR_A_PINS, MOTOR_B_PINS, MOTOR_C_PINS, MOTOR_D_PINS, PWM_FREQUENCY

class MotorDriver:
    pwm_instances = {}  # Class-level dictionary to store PWM instances by pin

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

        # Setup GPIO pins for direction control
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        # Setup PWM only if it hasn't been set up already
        if pwm_pin not in MotorDriver.pwm_instances:
            GPIO.setup(self.pwm_pin, GPIO.OUT)
            MotorDriver.pwm_instances[pwm_pin] = GPIO.PWM(self.pwm_pin, pwm_freq)
            MotorDriver.pwm_instances[pwm_pin].start(0)  # Start PWM with 0% duty cycle

        self.pwm = MotorDriver.pwm_instances[pwm_pin]

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
        # Do not stop or clean up PWM if it's shared by multiple motors
        GPIO.cleanup([self.in1, self.in2])


class DualMotorDriver:
    def __init__(self, motor_a_pins, motor_b_pins):
        """
        Initializes the driver for two motors.

        :param motor_a_pins: Tuple (in1, in2, pwma) for Motor A
        :param motor_b_pins: Tuple (in1, in2, pwmb) for Motor B
        """
        self.motor_a = MotorDriver(*motor_a_pins)
        self.motor_b = MotorDriver(*motor_b_pins)
        GPIO.setup(26, GPIO.OUT)
        GPIO.output(26, GPIO.HIGH)

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


class QuadMotorDriver:
    def __init__(self, side1_pins, side2_pins):
        """
        Initializes the driver for four motors (two per side).

        :param side1_pins: Tuple of tuples ((in1, in2, pwm_pin) for Motor A, (in1, in2, pwm_pin) for Motor B) for the first side
        :param side2_pins: Tuple of tuples ((in1, in2, pwm_pin) for Motor C, (in1, in2, pwm_pin) for Motor D) for the second side
        """
        print("Initializing Motors!")
        self.side1 = DualMotorDriver(side1_pins[0], side1_pins[1])  # Motor A and Motor B
        self.side2 = DualMotorDriver(side2_pins[0], side2_pins[1])  # Motor C and Motor D

    def set_side1_speed(self, speed):
        self.side1.set_motor_a_speed(speed)
        self.side1.set_motor_b_speed(speed)

    def set_side2_speed(self, speed):
        self.side2.set_motor_a_speed(speed)
        self.side2.set_motor_b_speed(speed)

    def stop_all(self):
        self.side1.stop_all()
        self.side2.stop_all()

    def cleanup(self):
        self.side1.cleanup()
        self.side2.cleanup()


# Test Main App
if __name__ == "__main__":
    # Define the pin connections for Motors A, B, C, and D
    side1_pins = ((MOTOR_A_PINS[0], MOTOR_A_PINS[1], MOTOR_A_PINS[2]),
                  (MOTOR_B_PINS[0], MOTOR_B_PINS[1], MOTOR_B_PINS[2]))
    side2_pins = ((MOTOR_C_PINS[0], MOTOR_C_PINS[1], MOTOR_C_PINS[2]),
                  (MOTOR_D_PINS[0], MOTOR_D_PINS[1], MOTOR_D_PINS[2]))

    # Create an instance of the QuadMotorDriver
    motors = QuadMotorDriver(side1_pins, side2_pins)

    try:
        # Test Side 1 forward at 50% speed
        print("Side 1 forward at 50% speed")
        motors.set_side1_speed(50)
        time.sleep(2)

        # Test Side 1 backward at 75% speed
        print("Side 1 backward at 75% speed")
        motors.set_side1_speed(-75)
        time.sleep(2)

        # Test Side 2 forward at 60% speed
        print("Side 2 forward at 60% speed")
        motors.set_side2_speed(60)
        time.sleep(2)

        # Test Side 2 backward at 40% speed
        print("Side 2 backward at 40% speed")
        motors.set_side2_speed(-40)
        time.sleep(2)

        # Stop all motors
        print("Stopping all motors")
        motors.stop_all()

    finally:
        # Cleanup GPIO resources
        print("Cleaning up GPIO")
        motors.cleanup()
