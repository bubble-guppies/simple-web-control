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


def create_server(HOST="", PORT=5000) -> socket:
    """Creates a server on HOST and PORT. Returns the socket.

    Returns:
        socket: the connected socket
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(0)

    print(f"Trying to bind to {PORT}...")
    try:
        s.bind((HOST, PORT))

    except socket.error as msg:
        print("Bind failed. Error Code : " + str(msg[0]) + " Message " + msg[1])
        sys.exit()

    print("Socket bind complete")

    s.listen(1)  # listens for one client
    s.settimeout(60)  # times out if no client connects within 60 seconds

    conn, addr = s.accept()

    print("Connected with " + addr[0] + ":" + str(addr[1]))
    return conn


def parse_message(mav_connection, message: str):
    """Parses message string into a command that the AUV runs.

    Args:
        mav_connection: mav_connection
        message (str): the message string
    """
    command_map = {
        "forward": np.array([-1, -1, 1, 1, 0, 0]),
        "backward": np.array([1, 1, -1, -1, 0, 0]),
        "left": np.array([-1, 1, -1, 1, 0, 0]),
        "right": np.array([1, -1, 1, -1, 0, 0]),
        "clockwise": np.array([1, 0, 0, 0, 0, 0]),
        "counterclockwise": np.array([0, 1, 0, 0, 0, 0]),
        "stop": np.array([0, 0, 0, 0, 0, 0]),
    }
    # try to split msg string into command
    try:
        message = message.strip().lower()
        message = message.split(",")
        command = message[0]
        # user only specifies command
        if len(message) == 1:
            time = 5
            power = 100
        # user specifies command and time
        elif len(message) == 2:
            time = int(message[1])
            power = 100
        # user specifies command, time, and power
        elif len(message) == 3:
            power = int(message[1])
            time = int(message[2])
        else:
            # if user inputs more values than intended
            pass

        thrusters = command_map[command] * power
        print(f"{command = }, {power = }, {time = }")

    except Exception as e:
        # Default values if the command couldn't be found
        print("Command not found! Error: ", e)
        time = 5
        thrusters = [0, 0, 0, 0, 0, 0]

    print(f"{time = }, {thrusters = }")
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
            # parse_message(mav_connection, decoded)
            parse_message(mav_connection, decoded)

    print(
        "Exited while loop, likely due to client disconnecting. Closing the socket and disarming ROV."
    )
    conn.close()
    dance_template.disarm_rov(mav_connection)


if __name__ == "__main__":
    main()
