"""
Bubble Guppies Python Server
Creates a server to listen for commands to send to the AUV.
"""
# https://pythonprogramming.net/python-binding-listening-sockets/
import dance_template
import socket
import sys
from motor_test import test_motor
from pymavlink import mavutil
import numpy as np


def create_server(HOST: str = "", PORT: int = 5000):
    """Creates a server on HOST using PORT.

    Args:
        HOST (str, optional): the server HOST name. Defaults to "".
        PORT (int, optional): the server PORT. Defaults to 5000.

    Returns:
        conn: the opened socket
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(0)

    print(f"Trying to bind to {PORT}...")
    try:
        s.bind((HOST, PORT))

    except socket.error as msg:
        print(f"Bind failed. Error: {msg}")
        sys.exit()

    print("Socket bind complete")

    s.listen(1)  # listens for one client
    s.settimeout(60)  # times out if no client connects within 60 seconds

    conn, addr = s.accept()

    print("Connected with " + addr[0] + ":" + str(addr[1]))
    return conn


def parse_message(message: str) -> tuple[str, list, int]:
    """Parses a string input into a command that the ROV can execute.

    Args:
        reply (str): the string to parse, e.g. "forward,10,100" to move forward at full power for 10 seconds

    Returns:
        Tuple[str, list, int]: the reply to the user, the thrusters list, and the amount of time to execute the command for
    """
    stop_time = 0.1  # the duration to stop for if message can't be interpreted
    default_power = 100  # default value for power
    power = default_power
    default_time = 5  # default duration

    # The map of command strings to thruster arrays.
    command_map = {
        "forward": np.array([-1, -1, 1, 1, 0, 0]),
        "backward": np.array([1, 1, -1, -1, 0, 0]),
        "left": np.array([-1, 1, -1, 1, 0, 0]),
        "right": np.array([1, -1, 1, -1, 0, 0]),
        "clockwise": np.array([1, -1, -1, 1, 0, 0]),
        "counterclockwise": np.array([-1, 1, 1, -1, 0, 0]),
        "stop": np.array([0, 0, 0, 0, 0, 0]),
    }

    # try to convert the raw message into a command
    try:
        message = message.strip().lower()  # filter message
        message = message.split(",")
        command = message[0]

        if command not in command_map.keys():
            reply = f"Command not found! Executing 'stop' for {stop_time} seconds.\n"
            command = "stop"
            time = stop_time

        else:
            # user only specifies command
            if len(message) == 1:
                time = default_time
                power = default_power
            # user specifies command and time
            elif len(message) == 2:
                time = int(message[1])
                power = default_power
            # user specifies command, power, and time
            elif len(message) == 3:
                power = int(message[1])
                time = int(message[2])
            else:
                # if user inputs more values than intended
                pass
            reply = f"Recieved message. Executing '{command}' at {power}% power for {time} seconds.\n"

    except Exception as e:
        # Default values if there is an error interpreting the command.
        reply = f"Error interpreting command: {e}.\n"
        command = "stop"
        time = stop_time

    thrusters = command_map[command] * power
    thrusters = thrusters.tolist()

    return (reply, thrusters, time)


def execute_command(mav_connection, thrusters: list, time: int):
    """Executes the given command by arming the ROV and running the motor.

    Args:
        mav_connection: the MAV connection
        thrusters (list): the list of thruster values, e.g. [100, 100, 100, 100, 0, 0]
        time (int): the number of seconds to run the motors for
    """
    if len(thrusters) != 6:
        # exit without executing the command
        print(f"Cannot execute command, invalid input for thrusters!")
        return
    # ensure that time is an int
    try:
        time = int(time)
    except TypeError as e:
        print(
            f"Issue while executing command. Time could not be converted to an integer: {e}.\nSetting time to 0."
        )
        time = 0

    dance_template.arm_rov(mav_connection)
    dance_template.run_motors_timed(mav_connection, time, thrusters)


def main():
    ####
    # Initialize ROV
    ####
    mav_connection = dance_template.mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    mav_connection.wait_heartbeat()

    # Create server
    HOST = ""
    PORT = 5000
    conn = create_server(HOST, PORT)

    with conn:
        conn.send(
            str.encode(
                "Enter a ROV command, in one of these forms:\n\
'<command>,<power>,<time>'\n\
'<command>,<time>'\n\
'<command>'\n"
            )
        )
        while True:
            data = conn.recv(2048)
            if not data:
                break
            decoded = data.decode("utf-8")
            (reply, thrusters, time) = parse_message(decoded)
            conn.send(reply.encode())
            execute_command(mav_connection, thrusters, time)

    print(
        "Exited while loop, likely due to client disconnecting. Closing the socket and disarming ROV."
    )
    conn.close()
    dance_template.disarm_rov(mav_connection)


if __name__ == "__main__":
    main()
