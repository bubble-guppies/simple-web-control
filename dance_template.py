from motor_test import test_motor
import time
from pymavlink import mavutil

## Globals
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
    # Error checking
    for i, power in enumerate(motor_settings):
        if not isinstance(power, int):
            try:
                motor_settings[i] = int(power)
            except:
                motor_settings[i] = 0
        if power > 100:
            motor_settings[i] = 100
        elif power < -100:
            motor_settings[i] = -100

    if not isinstance(seconds, int):
        try:
            seconds = int(round(seconds))
        except:
            seconds = 0

    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(
                mav_connection=mav_connection, motor_id=i, power=motor_settings[i]
            )
        time.sleep(0.2)
        step += 0.2


def forward(mav_connection, percent: int, t: int):
    """Move forward, using 4 thrusters.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(
        mav_connection, t, [-1 * percent, -1 * percent, 1 * percent, 1 * percent, 0, 0]
    )


def backward(mav_connection, percent: int, t: int):
    """Move backward, using 4 thrusters.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(
        mav_connection, t, [1 * percent, 1 * percent, -1 * percent, -1 * percent, 0, 0]
    )


def up(mav_connection, percent: int, t: int):
    """Move up.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(mav_connection, t, [0, 0, 0, 0, 1 * percent, 1 * percent])


def down(mav_connection, percent: int, t: int):
    """Move down.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(mav_connection, t, [0, 0, 0, 0, -1 * percent, -1 * percent])


def right(mav_connection, percent: int, t: int):
    """Move to the right.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(
        mav_connection, t, [1 * percent, -1 * percent, 1 * percent, -1 * percent, 0, 0]
    )


def left(mav_connection, percent: int, t: int):
    """Move to the left

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of maximum power to set the thrusters to
        t (int): the amount of time to apply the forces for
    """
    run_motors_timed(
        mav_connection, t, [-1 * percent, 1 * percent, -1 * percent, 1 * percent, 0, 0]
    )


def donut(mav_connection, percent: int):
    """Completes a donut. Currently untested."""
    donut_time = 15
    run_motors_timed(
        mav_connection, donut_time, [1 * percent, 0, 0.5 * percent, 0.5 * percent, 0, 0]
    )


def circle(mav_connection):
    """Completes a circle. Currently untested."""
    circle_time = 60
    percent = 100
    run_motors_timed(
        mav_connection,
        circle_time,
        [1 * percent, 0.5 * percent, 1 * percent, 0.5 * percent, 0, 0],
    )


def stop(mav_connection, time: int = 5):
    """Stops the AUV by setting all thrusters to 0.

    Args:
        mav_connection: the connection to the ROV
        time (int, optional): amount of time to stop for. Defaults to 5.
    """
    run_motors_timed(mav_connection, seconds=time, motor_settings=[0, 0, 0, 0, 0, 0])


def cw_turn(mav_connection, percent: int, time: int):
    """Turn clockwise by using one thruster. The angle of the turn depends on power and time.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of full power to use
        time (int): the time to turn for
    """
    run_motors_timed(
        mav_connection, seconds=time, motor_settings=[1 * percent, 0, 0, 0, 0, 0]
    )


def ccw_turn(mav_connection, percent: int, time: int):
    """Turn counterclockwise by using one thruster. The angle of the turn depends on power and time.

    Args:
        mav_connection: the connection to the ROV
        percent (int): the percentage of full power to use
        time (int): the time to turn for
    """
    run_motors_timed(
        mav_connection, seconds=time, motor_settings=[0, 1 * percent, 0, 0, 0, 0]
    )


def trace_m(mav_connection, time):
    """Traces a 'M' in the QGroundcontrol/ardupilot simulation environment. Starts at the bottom left leg facing up, ends facing the same way at the bottom right leg.
    Currently uses hardcoded turn durations.

    Args:
        mav_connection: the connection to the ROV
        time: the time for each section; controls size of the 'M'
    """
    forward(mav_connection, power, time)  # left leg
    cw_turn(mav_connection, 55, 9)
    forward(mav_connection, power, time * 0.5)  # down right
    ccw_turn(mav_connection, 50, 6)
    forward(mav_connection, power, time * 0.5)  # up right
    cw_turn(mav_connection, 55, 9)
    stop(mav_connection, 2)
    forward(mav_connection, power, time)  # right leg
    cw_turn(mav_connection, 55, 13)  # end facing the same way we started


def trace_i(mav_connection, time):
    """Traces a 'I' in the QGroundcontrol/ardupilot simulation environment. Starts and ends by moving a bit to the side.

    Args:
        mav_connection: the connection to the ROV
        time: the time for each section; controls size of the 'I'
    """
    right(mav_connection, power, time / 2)
    left(mav_connection, power, 1)  # cancel momentum before going up
    forward(mav_connection, power, time)
    backward(mav_connection, power, time)
    right(mav_connection, power, time / 2)


def trace_t(mav_connection, time):
    """Traces a 'T' in the QGroundcontrol/ardupilot simulation environment. Starts at the bottom left of the bounding rectangle.

    Args:
        mav_connection: the connection to the ROV
        time: the time for each section; controls size of the 'T'
    """
    right(mav_connection, power, time / 2)
    forward(mav_connection, power, time)
    backward(mav_connection, power, 1)
    left(mav_connection, power, time * 0.75)
    right(mav_connection, power, time * 1.2)
    """
    for this choreography, we chose to end T in the top right
    to make the "MIT" more clear. Generally, it would be better
    practice to start and end in consistent places.
    If we revists this function, we will add a:
    backward(mav_connection, power, time)
    """


def trace_mit(mav_connection):
    """Traces out "MIT" in the QGroundControl/ardupilot simulation environment

    Args:
        mav_connection: the connection to the ROV
    """
    trace_m(mav_connection, 20)
    right(mav_connection, power, 3)  # m ends at the bottom right
    trace_i(mav_connection, 15)
    trace_t(mav_connection, 15)


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
    cw_turn(mav_connection, 50, 7)  # rotate to north
    trace_mit(mav_connection)

    stop(mav_connection)  # stop

    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)


if __name__ == "__main__":
    main()
