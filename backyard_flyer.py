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

class BoxPath:
    """The path drone is suppose to follow is represented by BoxPath. Additionally
    It is used by the BackyadFlier to get the next waypoint and to figure out if 
    the current waypoint has been reached or not"""
    class WayPointResult(Enum):
        NOTREACHED = 0
        REACHED = 1
        PATH_COMPLETE = 2

    def __init__(self):
        self.all_waypoints = self.calculate_box()
        self.current_target = []

    def calculate_box(self):
        # N, E, Alt, Heading
        distance = 10.0
        altitude = 3.0
        
        return [[distance, 0.0, altitude, 0.0], 
                [distance, distance, altitude, 0.0], 
                [0.0, distance, altitude, 0.0], 
                [0.0, 0.0, altitude, 0.0]]

    def get_next(self):
        if self.all_waypoints:
            next_waypoint = self.all_waypoints.pop(0)
        else:
            next_waypoint = []

        self.current_target = next_waypoint
        return next_waypoint

    def is_close_to_current(self, local_position):
        if not self.current_target:
            return BoxPath.WayPointResult.PATH_COMPLETE
        else:
            # distance = square root of (x2-x1) + (y2-y1)
            distance = ((self.current_target[0] - local_position[0]) ** 2 
                        + (self.current_target[1] - local_position[1]) ** 2) ** 0.5
            if distance < 1:
                return BoxPath.WayPointResult.REACHED

            return BoxPath.WayPointResult.NOTREACHED

class StateDiagram:
    """Represents a state diagram. Different flight states are handled behind
    different callbacks. e.g. when MsgID.State is called, MANUAL, ARMING, DISARMING
    are checked. Behind MsgID.LocalPosition WAYPOINT flight state is checked. This class
    helps to consolidate these checks in one place.
    A dictionary is maintained for each callback. Each key in the dictionary is the 
    possible flight state and the value holds (a) a condition function to call and (b)
    another dictionary that tells which transition function to call behind different
    return values of the condition function."""
    class StateNode:
        def __init__(self, condition_fn, result_map):
            self.condition_fn = condition_fn
            self.result_map = result_map

    def __init__(self, drone):
        self.drone = drone
        self.event_to_state = {}
        drone.register_callback(MsgID.ANY, self.callback)

    def callback(self, name):
        # in case there is a state node that would like to work when the 'name' message
        # arrives, call the condition function, check the return value against the possible
        # transition functions and in case the return value matches one of the transition 
        # function, call it
        if name in self.event_to_state:
            state_diagram = self.event_to_state[name]

            if self.drone.flight_state in state_diagram:
                state_node = state_diagram[self.drone.flight_state]

                # in case there is no condition function, just call the transition
                # function directly
                if state_node.condition_fn is None:
                    fn = state_node.result_map[True]
                    fn()
                else:
                    result = state_node.condition_fn()
                    if result in state_node.result_map:
                        state_node.result_map[result]()

    def add(self, flight_state, msg_id, condition_fn, *result_transition):
        if msg_id not in self.event_to_state:
            self.event_to_state[msg_id] = {}

        state_diagram = self.event_to_state[msg_id]

        # warn, in case a given event handler already has a state node for the 
        # particular flight_state e.g MsgID.State already has work defined for
        # Manual state, warn the user
        if flight_state in state_diagram:
            print("\x1b[32m;State {0} already has a node attached to it".format(flight_state))

        result_map = {}

        # in case transition function is to be called only on True / False of the
        # function then put an entry for True->Transition function
        if condition_fn is None or len(result_transition) == 1:
            result_map[True] = result_transition[0]
        else:
            result_map = {}
            for i in range(0, len(result_transition), 2):
                result_map[result_transition[i]] = result_transition[i+1]

        state_diagram[flight_state] = self.StateNode(condition_fn, result_map)


class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        self.in_mission = False
        self.takeoff_altitude = 3.0
        self.path_planner = BoxPath()

        # create state diagram and set the initial state
        self.flight_state, self.state_diagram = self.create_state_diagram()

    def create_state_diagram(self):
        # each state in the diagram has a condition that checks if the state
        # work is complete and has a transition function
        state_diagram = StateDiagram(self)

        state_diagram.add(States.MANUAL, MsgID.STATE, None, self.arming_transition)
        # transition to TAKEOFF, if drone is armed and in ARMING state
        state_diagram.add(States.ARMING, MsgID.STATE, lambda: self.armed, 
                                self.takeoff_transition)

        # when the drone reaches the given take off altitude, switch to waypoint
        state_diagram.add(States.TAKEOFF, MsgID.LOCAL_POSITION, 
                                self.has_reached_altitude, self.waypoint_transition)

        # when one waypoint has been reached, move to the next one BUT if there
        # are no more waypoints then go to landing transition                        
        state_diagram.add(States.WAYPOINT, MsgID.LOCAL_POSITION, self.has_waypoint_reached, 
                                BoxPath.WayPointResult.REACHED, self.waypoint_transition,
                                BoxPath.WayPointResult.PATH_COMPLETE, self.landing_transition)

        # when the drone has landed, go to disarm transition                                
        state_diagram.add(States.LANDING, MsgID.LOCAL_VELOCITY, self.has_landed, 
                                self.disarming_transition)

        # when drone has disarmed, go to manual mode                        
        state_diagram.add(States.DISARMING, MsgID.STATE, self.has_disarmed, 
                                self.manual_transition)

        return States.MANUAL, state_diagram

    def has_disarmed(self):
        # print('has drone disarmed', self.armed, self.guided)
        return not (self.armed or self.guided)

    def has_reached_altitude(self):
        altitude = -1.0 * self.local_position[2]
        return altitude > 0.95 * self.takeoff_altitude

    def has_landed(self):
        altitude = -1.0 * self.local_position[2]
        return altitude < 0.5 and self.local_velocity[2] < 0.1

    def has_waypoint_reached(self):
        return self.path_planner.is_close_to_current(self.local_position)

    def arming_transition(self):
        if not self.armed:
            self.arm()

        self.take_control()
        self.set_home_position(*self.global_position)
        self.in_mission = True
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.takeoff(self.takeoff_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        next_waypoint = self.path_planner.get_next()
        if next_waypoint:
            self.cmd_position(*next_waypoint)
            self.flight_state = States.WAYPOINT
            
            print("transit to waypoint: ", next_waypoint)

    def landing_transition(self):
        # make sure the drone has stopped moving and then land
        if ((self.local_velocity[0] + self.local_velocity[1]) ** 2.0) ** 0.5 < .1:
            self.land()
            self.flight_state = States.LANDING

    def disarming_transition(self):
        print('disarming')
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print('Manual transition')
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        
    def start(self):
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
