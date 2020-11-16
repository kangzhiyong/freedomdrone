import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, a_star_for_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from sampling import Sampler
from shapely.geometry import LineString
import networkx as nx
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.polygons = []

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING

        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        data = np.genfromtxt('colliders.csv', delimiter=',', dtype='str', max_rows=1)
        _, lat0 = data[0].strip().split(' ')
        _, lon0 = data[1].strip().split(' ')
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(float(lon0), float(lat0), 0)
        # TODO: retrieve current global position
        curr_glb_pos = [self._longitude, self._latitude, self._altitude]
        # TODO: convert to current local position using global_to_local()
        self._north, self._east, self._down = global_to_local(curr_glb_pos, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        sampler = Sampler(data, SAFETY_DISTANCE)
        self.polygons = sampler._polygons
        nodes = sampler.sample(300)
        print('nodes_len: ', len(nodes))

        g = self.create_graph(nodes, 10)
        print('graph_edgs: ', len(g.edges))
        start = self.local_position
        goal = global_to_local([-122.396428, 37.795128, TARGET_ALTITUDE], self.global_home)

        start = self.find_closest_node(g.nodes, start)
        goal = self.find_closest_node(g.nodes, goal)
        path, cost = a_star_for_graph(g, heuristic, start, goal)
        print('a_star_path: ', path)
        path = self.prune_path(path)
        print('prune_path: ' ,path)
        if len(path) > 0:
            # Convert path to waypoints
            waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
            # Set self.waypoints
            self.waypoints = waypoints
            # TODO: send waypoints to sim (this is just for visualization of waypoints)
            self.send_waypoints()

    def start(self):
        #self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        #Sself.stop_log()

    def can_connect(self, n1, n2):
        # Determine whether there is a feasible route between two nodes
        l = LineString([n1, n2])
        h = min(n1[2], n2[2])
        for p in self.polygons:
            if p.crosses(l) and p.height >= h:
                return False
        return True

    def create_graph(self, nodes, k):
        # Create a feasible roadmap, use KDTree to find the k nearest neighbors, detect collisions, and build a roadmap
        g = nx.Graph()
        tree = KDTree(nodes)
        for n1 in nodes:
            # for each node connect try to connect to k nearest nodes
            idxs = tree.query([n1], k, return_distance=False)[0]
            for idx in idxs:
                n2 = nodes[idx]
                if n2 == n1:
                    continue
                if self.can_connect(n1, n2):
                    g.add_edge(n1, n2, weight=1)
        return g

    def bresenham_check(self, p1, p2):
        # Use bresenham algorithm to determine whether there is a feasible route between p1 and p2
        x1, y1, _ = p1
        x2, y2, _ = p2
        cells = []

        dx = x2 - x1
        dy = y2 - y1
        x = x1
        y = y1
        eps = 0
        if dx == 0:
            return self.can_connect(p1, p2)

        m = dy / dx
        # region 1
        if 0 <= m <= 1 and x1 < x2:
            for x in range(x1, x2):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dy
                if ((eps << 1) >= dx):
                    y += 1
                    eps -= dx
        # region 2
        elif m > 1 and y1 < y2:
            for y in range(y1, y2):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dx
                if ((eps << 1) >= dy):
                    x += 1
                    eps -= dy
        # region 3
        elif m < -1 and y1 < y2:
            for y in range(y1, y2):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dx
                if ((eps << 1) <= -dy):
                    x -= 1
                    eps += dy
        # region 4
        elif -1 <= m <= 0 and x2 < x1:
            for x in range(x1, x2, -1):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dy
                if ((eps << 1) >= -dx):
                    y += 1
                    eps += dx
        # region 5
        elif 0 < m <= 1 and x2 < x1:
            for x in range(x1, x2, -1):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dy
                if ((eps << 1) <= dx):
                    y -= 1
                    eps -= dx
        # region 6
        elif m > 1 and y2 < y1:
            for y in range(y1, y2, -1):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dx
                if ((eps << 1) <= dy):
                    x -= 1
                    eps -= dy
        # region 7
        elif m < -1 and y2 < y1:
            for y in range(y1, y2, -1):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dx
                if ((eps << 1) >= -dy):
                    x += 1
                    eps += dy
        # region 8
        elif -1 <= m < 0 and x1 < x2:
            for x in range(x1, x2):
                if not self.can_connect(p1, (x, y, p2[2])):
                    return False
                cells.append((x, y))
                eps += dy
                if ((eps << 1) <= -dx):
                    y -= 1
                    eps += dx

        return True

    def prune_path(self, path):
        # Remove redundant nodes in the path
        pruned_path = [p for p in path]
        i = 0
        while i < len(pruned_path) - 2:
            p1 = pruned_path[i]
            p2 = pruned_path[i + 1]
            p3 = pruned_path[i + 2]
            # First check if p2 can be removed, if it can be checked whether p3 can be removed, if both are satisfied, remove p2
            if self.bresenham_check(p1, p2) and self.bresenham_check(p1, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path

    def find_closest_node(self, points, point):
        # Find the node closest to a given point in the graph
        start_min_dist = np.linalg.norm(np.array(point) - np.array(points), axis=1).argmin()
        near_node = np.array(points)[start_min_dist]
        return tuple(near_node)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
