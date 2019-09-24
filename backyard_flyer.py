import argparse
import time
from enum import Enum

import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        This function checks if the flight has arrived to the desired position
            1. Take off position check
            2. Waypoints position check
        """
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]
            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                # add the map and desired path to the waypoint list
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            # calculate the distance between the target position and my local position now.
            # By calculating the Euclidean distant between north and east
            # Take care of the relaxation factor it affects the flight smoothness
            if np.linalg.norm(self.target_position[:2] - (self.local_position[:2])) < 1.0:
                # Check if you finished all the waypoints
                if len(self.all_waypoints) == 0:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
                else:
                    self.waypoint_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if (self.global_position[2] - self.global_home[2] < 0.1) and abs(
                    self.local_position[2]
            ) < 0.01:
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        The flight has three states that motivates the flight to go to the transition:
            1. Manual control
            2. ARMing
            3. Disarming
        """
        if not self.in_mission:
            return
        # Arm the flight
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        # Takeoff
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        # return back to manual control
        elif self.flight_state == States.DISARMING:
            if (not self.armed) and (not self.guided):
                self.manual_transition()

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        :return: waypoints to fly
        """
        # provide sequence of waypoints
        square = [(10, 0, 5, 0), (10, 10, 5, 0), (0, 10, 5, 0), (0, 0, 5, 0)]
        # Return waypoints to fly a box
        return square

    def arming_transition(self):
        """
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        # Take control of the drone
        self.take_control()
        # Pass an arming command
        self.arm()
        # Set the current location to be the home position
        self.set_home_position(
            self.global_position[0], self.global_position[1], self.global_position[2]
        )
        # Transition to the ARMING state
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        # Set target_position altitude to 3.0m
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        # Command a takeoff to 3.0m
        self.takeoff(target_altitude)
        # Transition to the TAKEOFF state
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        # Command the next waypoint position by removing the current waypoint from the All_waypoint list
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        # *(tuple) --> converts the tuple to numbers that fit what the function need
        self.cmd_position(*self.target_position)
        # Transition to WAYPOINT state
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        # Command the drone to land
        self.land()
        # Transition to the LANDING state
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        # Command the drone to disarm
        self.disarm()
        # After disarming you should release the control
        self.release_control()
        # Transition to the DISARMING state
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host", type=str, default="127.0.0.1", help="host address, i.e. '127.0.0.1'"
    )
    args = parser.parse_args()

    conn = MavlinkConnection(
        "tcp:{0}:{1}".format(args.host, args.port), threaded=False, PX4=False
    )
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
