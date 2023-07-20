from motor_test import test_motor
import time
from pymavlink import mavutil


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)
        step += 0.2


def forward(percent):
	return [0, 0, 1 * percent, 1 * percent, 0, 0]

def backward(percent):
	return [0, 0, 1 * percent, 1 * percent, 0, 0]

def up(percent):
	return [0, 0, 0, 0, 1 * percent, 1 * percent ]

def down(percent):
	return [0, 0, 0, 0, -1 * percent, -1 * percent ]

def right(percent):
	return [0, 0, 1 * percent, 1 * percent, 0, 0]

def left(percent):
	return [0, 0, -1 * percent, -1 * percent, 0, 0]

def donut(percent):
	return [1 * percent, 0, 0.5 * percent, 0.5 * percent, 0, 0]

def circle(percent):
	return [1 * percent, 0.5 * percent, 1 * percent, 0.5 * percent, 0, 0]

def stop():
	return [0, 0, 0, 0, 0, 0]

if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    run_motors_timed(mav_connection, seconds=5, motor_settings=[100,-100 ,100 ,-100, 0, 0])
    run_motors_timed(mav_connection, seconds=5, motor_settings=[-100,100 ,-100 ,100, 0, 0])
    # stop
    run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 0, 0, 0, 0, 0])

    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)
