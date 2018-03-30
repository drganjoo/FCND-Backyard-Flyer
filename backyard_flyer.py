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

class EventHandler:
    def __init__(self, planner, msg_id):
        self.planner = planner
        self.managed_states = set()
        self.planner.register_callback(msg_id, self.event_callback)

    def register(self, state):
        if isinstance(state, list):
            for s in state:
                self.managed_states.add(s) 
        else:
            self.managed_states.add(state)

    def event_callback(self):
        if self.planner.flight_state in self.managed_states:
            self.planner.invoke_state()

class StateNode:
    def __init__(self, event_handler, pre_condition, transition):
        self.event_handler = event_handler
        self.pre_condition = pre_condition
        self.transition = transition
        
class BoxPath:
    def __init__(self):
        self.all_waypoints = self.calculate_box()
        self.next_index = 0
        self.current_target = []

    def calculate_box(self):
        # N, E, Alt, Heading
        return np.array([[10.0, 0.0, 3.0, 0.0], [10.0, 10.0, 3.0, 0.0], [0.0, 10.0, 3.0, 0.0], [0.0, 0.0, 3.0, 0.0]])

    @property
    def current(self):
        return self.current_target

    def get_next(self):
        if self.next_index < len(self.all_waypoints):
            next_waypoint = self.all_waypoints[self.next_index]
            self.next_index += 1
        else:
            next_waypoint = []

        self.current_target = next_waypoint
        return next_waypoint

    def has_reached_target(self, local_position):
        if len(self.current_target) == 0:
            return True

        return False

class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        self.in_mission = False

        self.takeoff_altitude = 3.0
        self.heartbeat_handler = EventHandler(self, MsgID.STATE)
        self.local_pos_handler = EventHandler(self, MsgID.LOCAL_POSITION)

        self.path_planner = BoxPath()

        # create state diagram and set the initial state
        self.flight_state, self.state_diagram = self.create_state_diagram()

    def create_state_diagram(self):
        # each state in the diagram has a pre-condition that checks if the state
        # is complete, has a transition function and next state
        state_diagram = {}

        state_diagram[States.MANUAL] = StateNode(self.heartbeat_handler, None, self.arming_transition)
        state_diagram[States.ARMING] = StateNode(self.heartbeat_handler, None, self.takeoff_transition)
        state_diagram[States.TAKEOFF] = StateNode(self.local_pos_handler, self.has_reached_altitude, self.waypoint_transition)
        state_diagram[States.WAYPOINT] = StateNode(self.local_pos_handler, self.has_waypoint_reached, self.waypoint_transition)

        # register each state node with the respective event handler
        for state, node in state_diagram.items():
            node.event_handler.register(state)

        return States.MANUAL, state_diagram

    def invoke_state(self):
        state_node = self.state_diagram[self.flight_state]

        if state_node.pre_condition != None:
            pre_condition_ok = state_node.pre_condition()
        else:
            pre_condition_ok = True

        if pre_condition_ok:
            state_node.transition()

    def has_reached_altitude(self):
        altitude = -1.0 * self.local_position[2]
        return altitude > 0.95 * self.takeoff_altitude

    def has_waypoint_reached(self):
        current = self.path_planner.current
        if len(current) == 0:
            self.landing_transition()
        else:
            # distance = square root of (x2-x1) + (y2-y1)
            distance = ((current[0] - self.local_position[0]) ** 2 + (current[1] - self.local_position[1]) ** 2) ** 0.5
            return distance < 1
                
    # def velocity_callback(self):
    #     """
    #     TODO: Implement this method

    #     This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
    #     """
    #     pass

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.take_control()
        self.arm()
        self.set_home_position(*self.global_position)
        self.in_mission = True
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.takeoff(self.takeoff_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        next_waypoint = self.path_planner.get_next()
        if len(next_waypoint) > 0:
            self.cmd_position(*next_waypoint)
            self.flight_state = States.WAYPOINT
            
            print("transit to waypoint: ", next_waypoint)

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

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
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
