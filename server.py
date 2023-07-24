# open a socket
# listen on that socket, analog to NC
# if you get 100, send forward 100

# https://pythonprogramming.net/python-binding-listening-sockets/
import dance_template
import socket
import sys
from motor_test import test_motor
import time
from pymavlink import mavutil
import numpy as np


def create_server():
    """ """
    HOST = ""
    PORT = 8678
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(0)

    try:
        s.bind((HOST, PORT))

    except socket.error as msg:
        print("Bind failed. Error Code : " + str(msg[0]) + " Message " + msg[1])
        sys.exit()

    print("Socket bind complete")

    s.listen(1)
    s.settimeout(10)

    conn, addr = s.accept()

    print("Connected with " + addr[0] + ":" + str(addr[1]))
    return conn

def parse_message(mav_connection, reply: str):
    values_dict = {
        "forward": np.array([-1, -1, 1, 1, 0, 0]),
        "backward": np.array([1, 1, -1, -1, 0, 0]),
        "left": np.array([-1, 1, -1, 1, 0, 0]),
        "right": np.array([1, -1, 1, -1, 0, 0]),
        "clockwise": np.array([1, 0, 0, 0, 0, 0]),
        "counterclockwise": np.array([0, 1, 0, 0, 0, 0]),
        "stop": np.array([0, 0, 0, 0, 0, 0])
    }
    try:
        reply = reply.strip()
        reply = reply.split()
        command = reply[0]
        # user only specifies command
        if len(reply)==1:
            time = 5
        elif len(reply)==2:
            time = int(reply[1])
            power = 1
        elif len(reply) == 3:
            power = int(reply[1])
            time = int(reply[2])
        else:
            # if user inputs more values than intended
            pass

        if(command == "stop"):
            pause(mav_connection)
        thrusters = values_dict[command.strip()] * power
        print(f"{command = }, {power = }, {time = }")
    except Exception as e:
        # Default values if the command couldn't be found
        print("Command not found! Error: ", e)
        time = 5
        thrusters = [0, 0, 0, 0, 0, 0]

    print(f"{time = }, {thrusters = }")
    dance_template.arm_rov(mav_connection)
    dance_template.run_motors_timed(mav_connection, time, thrusters)

def pause(mav_connection):
    dance_template.stop(mav_connection, 0.1)


def main():
    ####
    # Initialize ROV
    ####
    mav_connection = dance_template.mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    mav_connection.wait_heartbeat()

    # Create server
    conn = create_server()

    with conn:
        conn.send(
            str.encode(
                "Type in a number between -100 and 100 to represent thruster force. \n"
            )
        )
        while True:
            data = conn.recv(2048)
            if not data:
                break
            decoded = data.decode("utf-8")
            # parse_message(mav_connection, decoded)
            parse_message(mav_connection, decoded)

    conn.close()

    dance_template.disarm_rov(mav_connection)

    print("exited while loop")


if __name__ == "__main__":
    main()
"""
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    ???
"""
