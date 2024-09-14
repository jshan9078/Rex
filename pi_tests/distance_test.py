# Getting the libraries we need
from gpiozero import DistanceSensor
from time import sleep

# Initialize ultrasonic sensor
sensor = DistanceSensor(trigger=17, echo=4)

while True:
	# Wait 2 seconds
	sleep(0.1)

	# But we want it in centimetres
	distance = sensor.distance * 100

	# Print the information to the screen
	print("Distance: {} cm".format(distance))
