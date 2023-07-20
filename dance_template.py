from motor_test import test_motor
import time
from pymavlink import mavutil

## Globals
turn_time = 5
power = 100

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
            test_motor(
                mav_connection=mav_connection, motor_id=i, power=motor_settings[i]
            )
        time.sleep(0.2)
        step += 0.2


def forward(mav_connection, percent: int, t: int):
    run_motors_timed(
        mav_connection, t, [-1 * percent, -1 * percent, 1 * percent, 1 * percent, 0, 0]
    )


def backward(mav_connection, percent: int, t: int):
    run_motors_timed(
        mav_connection, t, [1 * percent, 1 * percent, -1 * percent, -1 * percent, 0, 0]
    )


def up(mav_connection, percent: int, t: int):
    run_motors_timed(mav_connection, t, [0, 0, 0, 0, 1 * percent, 1 * percent])


def down(mav_connection, percent: int, t: int):
    run_motors_timed(mav_connection, t, [0, 0, 0, 0, -1 * percent, -1 * percent])


def right(mav_connection, percent: int, t: int):
    run_motors_timed(mav_connection, t, [1 * percent, -1 * percent, 1 * percent, -1 * percent, 0, 0])


def left(mav_connection, percent: int, t: int):
    run_motors_timed(mav_connection, t, [-1 * percent, 1 * percent, -1 * percent, 1 * percent, 0, 0])


def donut(mav_connection, percent: int):
    donut_time = 15
    run_motors_timed(
        mav_connection, donut_time, [1 * percent, 0, 0.5 * percent, 0.5 * percent, 0, 0]
    )


def circle(mav_connection):
    circle_time = 60
    percent = 100
    run_motors_timed(
        mav_connection,
        circle_time,
        [1 * percent, 0.5 * percent, 1 * percent, 0.5 * percent, 0, 0],
    )


def stop(mav_connection):
    stop_time = 5
    run_motors_timed(
        mav_connection, seconds=stop_time, motor_settings=[0, 0, 0, 0, 0, 0]
    )

def trace_m(mav_connection, time):
    # time = time to trace each line, i.e. the time from bottom of | to the top
    pass

def trace_i(mav_connection, time):
    # starts at the bottom
    right(mav_connection, power, time / 2)
    forward(mav_connection, power, time)
    backward(mav_connection, power, time)
    right(mav_connection, power, time / 2)
    pass

def trace_t(mav_connection, time):
    # starts at the bottom of the outer rectangle
    right(mav_connection, power, time / 2)
    forward(mav_connection, power, time)
    left(mav_connection, power, time / 2)
    right(mav_connection, power, time)
    '''
    for this choreography, we chose to end T in the top right
    to make the "MIT" more clear. Generally, it would be better
    practice to start and end in consistent places.
    If we revists this function, we will add a:
    backward(mav_connection, power, time)
    '''
    pass


def trace_mit(mav_connection, time: int):
    trace_m(mav_connection, time)
    right(mav_connection, power, 3) # m ends at the bottom right
    trace_i(mav_connection, time)
    trace_t(mav_connection, time)


def main():
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
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
    # dance moves here:
    # forward(mav_connection, power, 10)
    # backward(mav_connection, power, 11)
    # right(mav_connection, power, 10)
    # left(mav_connection, power, 11)
    # donut(mav_connection, 100)
    # circle(mav_connection)
    trace_t(mav_connection, 20)

    # stop
    stop(mav_connection)

    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)


if __name__ == "__main__":
    main()
