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
    class WayPointResult(Enum):
        NOTREACHED = 0
        REACHED = 1
        PATH_COMPLETE = 2

    def __init__(self):
        self.all_waypoints = self.calculate_box()
        self.next_index = 0
        self.current_target = []

    def calculate_box(self):
        # N, E, Alt, Heading
        distance = 10.0
        altitude = 3.0
        
        return np.array([[distance, 0.0, altitude, 0.0], 
                        [distance, distance, altitude, 0.0], 
                        [0.0, distance, altitude, 0.0], 
                        [0.0, 0.0, altitude, 0.0]])

    def get_next(self):
        if self.next_index < len(self.all_waypoints):
            next_waypoint = self.all_waypoints[self.next_index]
            self.next_index += 1
        else:
            next_waypoint = np.array([])

        self.current_target = next_waypoint
        return next_waypoint

    def is_close_to_current(self, local_position):
        if self.current_target.size == 0:
            return BoxPath.WayPointResult.PATH_COMPLETE
        else:
            # distance = square root of (x2-x1) + (y2-y1)
            distance = ((self.current_target[0] - local_position[0]) ** 2 
                        + (self.current_target[1] - local_position[1]) ** 2) ** 0.5
            if distance < 1:
                return BoxPath.WayPointResult.REACHED

            return BoxPath.WayPointResult.NOTREACHED

class StateDiagram:
    class StateNode:
        def __init__(self, condition_fn, result_map):
            self.condition_fn = condition_fn
            self.result_map = result_map

    def __init__(self, drone):
        self.drone = drone
        self.event_to_state = {}
        drone.register_callback(MsgID.ANY, self.callback)

    def callback(self, name):
        if name in self.event_to_state:
            state_diagram = self.event_to_state[name]

            if self.drone.flight_state in state_diagram:
                # only in manual state the control algorithm doesn't have to be
                # in guided mode otherwise we make sure not to call the state flow
                # in case the drone is not in guided mode
                if self.drone.flight_state != States.MANUAL and not self.drone.guided:
                    return

                state_node = state_diagram[self.drone.flight_state]

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
        # Manual state
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
        # each state in the diagram has a pre-condition that checks if the state
        # is complete, has a transition function and next state
        state_diagram = StateDiagram(self)
        state_diagram.add(States.MANUAL, MsgID.STATE, None, self.arming_transition)
        state_diagram.add(States.ARMING, MsgID.STATE, None, self.takeoff_transition)
        state_diagram.add(States.TAKEOFF, MsgID.LOCAL_POSITION, 
                                self.has_reached_altitude, self.waypoint_transition)
        state_diagram.add(States.WAYPOINT, MsgID.LOCAL_POSITION, self.has_waypoint_reached, 
                                BoxPath.WayPointResult.REACHED, self.waypoint_transition,
                                BoxPath.WayPointResult.PATH_COMPLETE, self.landing_transition)
        state_diagram.add(States.LANDING, MsgID.LOCAL_VELOCITY, self.has_landed, 
                                self.disarming_transition)
        state_diagram.add(States.DISARMING, MsgID.STATE, None, 
                                self.end_transition)

        return States.MANUAL, state_diagram

    def has_reached_altitude(self):
        altitude = -1.0 * self.local_position[2]
        return altitude > 0.95 * self.takeoff_altitude

    def has_landed(self):
        altitude = -1.0 * self.local_position[2]
        if altitude < 0.5 and self.local_velocity[2] < 0.1:
            print("Landed")
            self.flight_state = States.DISARMING            

    def has_waypoint_reached(self):
        return self.path_planner.is_close_to_current(self.local_position)

    def arming_transition(self):
        if not self.armed:
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
        if next_waypoint.size > 0:
            self.cmd_position(*next_waypoint)
            self.flight_state = States.WAYPOINT
            
            print("transit to waypoint: ", next_waypoint)

    def landing_transition(self):
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def end_transition(self):
        self.release_control()
        self.connection.stop()
        
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
