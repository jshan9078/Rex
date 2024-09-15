# import time
# from motor_driver import QuadMotorDriver
# from mpu_driver import MPU6050

# # Dummy function to simulate receiving directions from a database
# def get_directions_from_database():
#     return [
#         ("right", 90),    # Turn 90 degrees right
#         ("straight", 50), # Move straight for 50 meters
#         ("left", 90),     # Turn 90 degrees left
#         ("straight", 20)  # Move straight for 20 meters
#     ]

# # Constants for motor control
# STRAIGHT_SPEED = 50  # Speed percentage for moving straight
# TURN_SPEED = 50      # Speed percentage for turning
# TURN_ANGLE_THRESHOLD = 90  # Angle in degrees for a right/left turn
# DISTANCE_THRESHOLD = 0.1  # Distance in meters before checking if command is complete

# # Proportional controller constant for straight movement
# KP = 10  # Adjust this gain for gyroscope feedback control

# # Gyroscope integration helper to track angle over time
# def integrate_gyro_z(mpu, duration):
#     """Integrate the gyroscope's Z-axis (angular velocity) to calculate the change in angle over the specified duration."""
#     initial_time = time.time()
#     total_angle = 0.0

#     while time.time() - initial_time < duration:
#         gyro_data = mpu.read_gyro()
#         gyro_z = gyro_data['z']  # Gyro Z-axis measures yaw (rotation around the vertical axis)
        
#         # Estimate angle change from angular velocity
#         delta_time = time.time() - initial_time
#         total_angle += gyro_z * delta_time
#         initial_time = time.time()

#     return total_angle

# # Function to turn until a specific angle is reached using the gyroscope
# def turn(motors, mpu, target_angle):
#     """Turns the robot by the target angle (positive for right, negative for left) using gyroscope feedback."""
#     total_turned_angle = 0.0
#     initial_gyro_data = mpu.read_gyro()
    
#     if target_angle > 0:  # Turn right
#         motors.set_side1_speed(TURN_SPEED)
#         motors.set_side2_speed(-TURN_SPEED)
#     else:  # Turn left
#         motors.set_side1_speed(-TURN_SPEED)
#         motors.set_side2_speed(TURN_SPEED)

#     # Loop until the total turned angle matches the target angle
#     while abs(total_turned_angle) < abs(target_angle):
#         gyro_z = mpu.read_gyro()['z']
#         elapsed_time = 0.1  # Interval between gyro readings
#         total_turned_angle += gyro_z * elapsed_time

#         time.sleep(elapsed_time)

#     # Stop the motors once the target angle is reached
#     motors.stop_all()
#     print(f"Turned {total_turned_angle} degrees (target was {target_angle} degrees).")

# # Main function to run the superloop
# def main():
#     side1_pins = ((MOTOR_A_PINS[0], MOTOR_A_PINS[1], MOTOR_A_PINS[2]),
#                   (MOTOR_B_PINS[0], MOTOR_B_PINS[1], MOTOR_B_PINS[2]))
#     side2_pins = ((MOTOR_C_PINS[0], MOTOR_C_PINS[1], MOTOR_C_PINS[2]),
#                   (MOTOR_D_PINS[0], MOTOR_D_PINS[1], MOTOR_D_PINS[2]))

#     # Motor and MPU objects
#     motors = QuadMotorDriver(side1_pins, side2_pins)
#     mpu = MPU6050()

#     try:
#         directions = get_directions_from_database()
#         for direction, value in directions:
#             if direction == "right":
#                 print(f"Turning right {value} degrees")
#                 turn(motors, mpu, value)  # Use gyro to turn right by specified degrees

#             elif direction == "left":
#                 print(f"Turning left {value} degrees")
#                 turn(motors, mpu, -value)  # Use gyro to turn left by specified degrees

#             elif direction == "straight":
#                 print(f"Moving straight for {value} meters")
#                 initial_gyro = mpu.read_gyro()['z']  # Get the initial heading from the gyroscope
#                 distance_traveled = 0.0
#                 prev_time = time.time()

#                 # Move straight while tracking the distance
#                 motors.set_side1_speed(STRAIGHT_SPEED)
#                 motors.set_side2_speed(STRAIGHT_SPEED)

#                 while distance_traveled < value:
#                     current_time = time.time()
#                     elapsed_time = current_time - prev_time
#                     prev_time = current_time

#                     # Read accelerometer data (X-axis for forward/backward movement)
#                     accel_data = mpu.read_accel()
#                     accel_x = accel_data['x'] * 9.81  # Convert g-force to m/s²

#                     # Integrate acceleration to calculate the distance traveled
#                     distance_traveled += 0.5 * accel_x * (elapsed_time ** 2)

#                     # Gyroscope correction (Z-axis for rotation)
#                     gyro_z = mpu.read_gyro()['z']
#                     error = initial_gyro - gyro_z
#                     correction = KP * error

#                     # Apply correction to motors
#                     motors.set_side1_speed(STRAIGHT_SPEED - correction)
#                     motors.set_side2_speed(STRAIGHT_SPEED + correction)

#                     time.sleep(0.1)  # Small delay for loop iteration

#                 # Stop motors after traveling the specified distance
#                 motors.stop_all()
#                 print(f"Reached {value} meters.")

#     finally:
#         # Cleanup GPIO and other resources
#         print("Cleaning up...")
#         motors.cleanup()

# if __name__ == "__main__":
#     main()

import time
from motor import QuadMotorDriver
from mpu import MPU6050
from config import MOTOR_A_PINS, MOTOR_B_PINS, MOTOR_C_PINS, MOTOR_D_PINS, PWM_FREQUENCY

from supabase import create_client, Client

# Initialize Supabase client
url = "https://pbggiqsajdewevfcxcny.supabase.co/"
key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InBiZ2dpcXNhamRld2V2ZmN4Y255Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3MjYyOTQ3MTEsImV4cCI6MjA0MTg3MDcxMX0.ZuJeJ5h9Y7u5UITpTQERhbl1F36WG8N1eaO4sdcHJAM"
supabase: Client = create_client(url, key)

def fetch_latest_instructions():
    response = supabase \
        .from_("instructions") \
        .select("instructions") \
        .order("created_at", desc=True) \
        .limit(1) \
        .execute()

    if response.data and len(response.data) > 0:
        instructions_array = response.data[0]['instructions']
        return [(direction, int(distance)) for direction, distance in (instruction.split(',') for instruction in instructions_array)]
    else:
        return []

# Example usage
latest_unfinished_row = fetch_latest_instructions()
print(latest_unfinished_row)

# Dummy function to simulate receiving directions from a database
def get_directions_from_database():
    return latest_unfinished_row
    """
    return [
        ("straight", 16),
        ("right", 90),    # Turn 90 degrees right
        ("straight", 10), # Move straight for 50 meters
        ("left", 90),     # Turn 90 degrees left
        ("straight", 1)  # Move straight for 20 meters
    ]
    """

# Constants for motor control
STRAIGHT_SPEED = 100  # Speed percentage for moving straight
TURN_SPEED = 100      # Speed percentage for turning
TURN_ANGLE_THRESHOLD = 90  # Angle in degrees for a right/left turn
DISTANCE_THRESHOLD = 0.1  # Distance in meters before checking if command is complete

# Proportional controller constant for straight movement
KP = 1.5 # Adjust this gain for gyroscope feedback control

# Gyroscope integration helper to track yaw angle over time (X-axis for yaw)
def integrate_gyro_x(mpu, duration):
    """Integrate the gyroscope's X-axis (angular velocity) to calculate the change in yaw over the specified duration."""
    total_angle = 0
    start_time = time.time()

    while (time.time() - start_time) < duration:
        gyro_data = mpu.read_gyro()
        angular_velocity_x = gyro_data['x']  # Angular velocity in degrees per second (X-axis for yaw)
        elapsed_time = time.time() - start_time

        # Calculate the change in angle
        total_angle += angular_velocity_x * elapsed_time

        # Sleep for a small time interval
        time.sleep(0.01)

    return total_angle

# Accelerometer integration helper to track distance using Z-axis acceleration
def integrate_accel_z(mpu, duration):
    """Integrate the accelerometer's Z-axis (forward acceleration) to calculate the distance traveled over the specified duration."""
    total_velocity = 0
    total_distance = 0
    start_time = time.time()

    while (time.time() - start_time) < duration:
        accel_data = mpu.read_accel()
        acceleration_z = accel_data['z'] * 9.81  # Convert g to m/s² (Z-axis for forward acceleration)
        elapsed_time = time.time() - start_time

        # Integrate acceleration to get velocity
        total_velocity += acceleration_z * elapsed_time

        # Integrate velocity to get distance
        total_distance += total_velocity * elapsed_time

        # Sleep for a small time interval
        time.sleep(0.01)

    return total_distance

def turn(mpu, motors, direction, target_angle):
    """Turn the vehicle left or right by a specified angle using gyroscope feedback."""
    # motors.stop_all()
    motors.set_side1_speed(-TURN_SPEED if direction == "right" else TURN_SPEED)
    motors.set_side2_speed(TURN_SPEED if direction == "right" else -TURN_SPEED)
    total_angle = 0
    gyro_data = None
    while abs(total_angle) < target_angle:
        try:
            gyro_data = mpu.read_gyro()
        except:
            print("GYRO read failed")
        angular_velocity_x = gyro_data['x']  # Gyroscope X-axis for yaw
        print("Hello")
        total_angle += angular_velocity_x * 0.0168  # Approximate angle change
        time.sleep(0.01)
    # Stop motors after turning
    motors.stop_all()

def drive_straight(mpu, motors, target_distance):
    """Drive the vehicle straight for a specified distance using accelerometer and gyroscope feedback."""
    total_distance = 0
    gyro_data = None
    accel_data = None
    while total_distance < target_distance:
        try:
            gyro_data = mpu.read_gyro()
            accel_data = mpu.read_accel()
        except:
            print("failed to read from mpu")

        accel_mag = accel_data['x'] ** 2 + accel_data['y'] ** 2 + accel_data['z'] ** 2
        if accel_mag > 14:
            motors.stop_all()
            while True:
                pass
        # Use gyroscope X-axis (yaw) for feedback to correct the heading
        angular_velocity_x = gyro_data['x']  # X-axis is for yaw (turning)
        correction = KP * angular_velocity_x
        # print(f"angular velocity is {angular_velocity_x} and correction val is {correction}")
        # Set motor speeds with correction to maintain straight movement
        speed1 = STRAIGHT_SPEED - correction
        speed2 = STRAIGHT_SPEED + correction - 26.5

        if speed1 > 100:
            speed1 = 100
        if speed1 < 0:
            speed1 = 0
        if speed2 < 0:
            speed2 = 0
        if speed2 > 100:
            speed2 = 100
        # print(f"speed 1 and 2 are {speed1} and {speed2}")
        motors.set_side1_speed(speed1)
        motors.set_side2_speed(speed2)

        # Integrate Z-axis acceleration to calculate distance traveled
        acceleration_z = accel_data['z'] * 9.81  # Convert g to m/s²
        velocity_z = acceleration_z * 0.01  # Approximate velocity over small time interval
        total_distance += velocity_z * 0.5  # Approximate distance
        # print(f"total distance travelled is {total_distance}")
        time.sleep(0.01)

    # Stop motors after driving straight
    motors.stop_all()

# Main control loop
def main():
    motors = QuadMotorDriver(((MOTOR_A_PINS[0], MOTOR_A_PINS[1], MOTOR_A_PINS[2]), 
                              (MOTOR_B_PINS[0], MOTOR_B_PINS[1], MOTOR_B_PINS[2])),
                             ((MOTOR_C_PINS[0], MOTOR_C_PINS[1], MOTOR_C_PINS[2]), 
                              (MOTOR_D_PINS[0], MOTOR_D_PINS[1], MOTOR_D_PINS[2])))
    mpu = MPU6050()

    directions = get_directions_from_database()

    try:
        for direction, value in directions:
            if direction == "straight":
                print(f"Driving straight for {value} meters")
                drive_straight(mpu, motors, value*1.3)
            elif direction == "left" or direction == "right":
                print(f"Turning {direction} by 90 degrees")
                turn(mpu, motors, direction, 83)
            time.sleep(1)
    finally:
        # Cleanup resources
        motors.cleanup()

if __name__ == "__main__":
    main()