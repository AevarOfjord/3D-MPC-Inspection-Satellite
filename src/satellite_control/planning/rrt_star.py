"""
RRT* (Rapidly-exploring Random Tree Star) Path Planner
"""

import math
import random
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Obstacle:
    position: np.ndarray
    radius: float


class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.cost = 0.0
        self.parent: Optional["Node"] = None

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class RRTStarPlanner:
    """RRT* (Rapidly-exploring Random Tree Star) path planner.

    Finds collision-free paths in 3D space using random sampling.
    Uses vectorized numpy operations for efficient neighbor searches.
    """

    def __init__(
        self,
        bounds_min: Tuple[float, float, float] = (-10, -10, -10),
        bounds_max: Tuple[float, float, float] = (10, 10, 10),
        step_size: float = 0.5,
        max_iter: int = 1000,
        search_radius: float = 1.0,
    ):
        self.bounds_min = np.array(bounds_min)
        self.bounds_max = np.array(bounds_max)
        self.step_size = step_size
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.node_list: List[Node] = []
        # Cached position array for vectorized operations
        self._positions: Optional[np.ndarray] = None

    def plan(
        self, start: np.ndarray, goal: np.ndarray, obstacles: List[Obstacle]
    ) -> List[np.ndarray]:
        """
        Plan a path from start to goal avoiding obstacles.
        Returns a list of waypoints (np.ndarray).
        """
        self.node_list = [Node(*start)]
        self._positions = start.reshape(1, 3).copy()  # Initialize position cache
        goal_node = Node(*goal)

        for i in range(self.max_iter):
            rnd_node = self._get_random_node(goal)
            nearest_ind = self._get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self._steer(nearest_node, rnd_node, self.step_size)

            if self._check_collision(new_node, obstacles):
                near_inds = self._find_near_nodes(new_node)

                # Connect to best parent
                new_node = self._choose_parent(new_node, near_inds, obstacles)
                if new_node:
                    self.node_list.append(new_node)
                    # Update position cache
                    self._positions = np.vstack([self._positions, new_node.position()])
                    self._rewire(new_node, near_inds, obstacles)

            # Check if close to goal
            # Optimization: Try to connect to goal every N iterations or if close
            if self._calc_dist_to_goal(self.node_list[-1], goal_node) <= self.step_size:
                if self._check_collision_line(self.node_list[-1], goal_node, obstacles):
                    # Found valid path to goal
                    # We don't stop immediately in RRT*, strictly speaking,
                    # but for real-time app we might want to return *some* path quickly.
                    # For now, let's just let it run or return best path found at end?
                    # A common RRT approach: if close, try to connect direct.
                    final_node = self._steer(
                        self.node_list[-1], goal_node, self.step_size
                    )
                    final_node.parent = self.node_list[-1]
                    final_node.cost = self.node_list[-1].cost + self._calc_dist(
                        final_node, self.node_list[-1]
                    )
                    self.node_list.append(final_node)
                    return self._generate_final_course(len(self.node_list) - 1)

        # If max_iter reached
        # Find node closest to goal? Or just fail?
        # Let's find best candidate
        last_index = self._get_best_last_index(goal_node)
        if last_index is not None:
            return self._generate_final_course(last_index)

        return []

    def _get_random_node(self, goal: np.ndarray) -> Node:
        # Goal bias
        if random.randint(0, 100) > 90:  # 10% goal bias
            return Node(*goal)

        return Node(
            random.uniform(self.bounds_min[0], self.bounds_max[0]),
            random.uniform(self.bounds_min[1], self.bounds_max[1]),
            random.uniform(self.bounds_min[2], self.bounds_max[2]),
        )

    def _steer(
        self, from_node: Node, to_node: Node, extend_length=float("inf")
    ) -> Node:
        # Vector from -> to
        diff = to_node.position() - from_node.position()
        dist = np.linalg.norm(diff)

        if dist <= extend_length:
            new_node = Node(*to_node.position())
        else:
            # Scale vector
            direction = diff / dist
            new_pos = from_node.position() + direction * extend_length
            new_node = Node(*new_pos)

        new_node.parent = from_node
        new_node.cost = from_node.cost + dist  # Rough initial cost
        return new_node

    def _get_nearest_node_index(self, node_list: List[Node], rnd_node: Node) -> int:
        """Find index of node nearest to rnd_node using vectorized operations."""
        if self._positions is None or len(self._positions) == 0:
            # Fallback to non-vectorized version
            dlist = [
                (node.x - rnd_node.x) ** 2
                + (node.y - rnd_node.y) ** 2
                + (node.z - rnd_node.z) ** 2
                for node in node_list
            ]
            return int(np.argmin(dlist))

        # Vectorized distance calculation
        target = rnd_node.position()
        diff = self._positions - target
        sq_dists = np.sum(diff * diff, axis=1)
        return int(np.argmin(sq_dists))

    def _check_collision(self, node: Node, obstacle_list: List[Obstacle]) -> bool:
        # Check node itself
        for obs in obstacle_list:
            dx = obs.position[0] - node.x
            dy = obs.position[1] - node.y
            dz = obs.position[2] - node.z
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= (obs.radius + 0.3):  # 0.3 safety margin (Sat size)
                return False  # Collision
        return True  # Safe

    def _check_collision_line(
        self, node1: Node, node2: Node, obstacle_list: List[Obstacle]
    ) -> bool:
        # Simple line check - discretize line
        p1 = node1.position()
        p2 = node2.position()
        dist = np.linalg.norm(p2 - p1)
        if dist < 0.01:
            return True

        steps = int(dist / 0.1) + 1  # Check every 10cm
        for i in range(steps + 1):
            t = i / steps
            p = p1 + (p2 - p1) * t
            temp_node = Node(*p)
            if not self._check_collision(temp_node, obstacle_list):
                return False
        return True

    def _find_near_nodes(self, new_node: Node) -> List[int]:
        """Find all nodes within search radius using vectorized operations."""
        nnode = len(self.node_list) + 1
        r = self.search_radius * math.sqrt((math.log(nnode) / nnode))
        r = min(r, self.search_radius * 5.0)  # Cap radius

        if self._positions is None or len(self._positions) == 0:
            # Fallback to non-vectorized version
            dlist = [
                (node.x - new_node.x) ** 2
                + (node.y - new_node.y) ** 2
                + (node.z - new_node.z) ** 2
                for node in self.node_list
            ]
            return [i for i, d in enumerate(dlist) if d <= r**2]

        # Vectorized distance calculation
        target = new_node.position()
        diff = self._positions - target
        sq_dists = np.sum(diff * diff, axis=1)
        r_sq = r * r
        return list(np.where(sq_dists <= r_sq)[0])

    def _choose_parent(
        self, new_node: Node, near_inds: List[int], obstacle_list: List[Obstacle]
    ):
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            # Check collision
            if self._check_collision_line(near_node, new_node, obstacle_list):
                d = self._calc_dist(near_node, new_node)
                costs.append(near_node.cost + d)
            else:
                costs.append(float("inf"))

        min_cost = min(costs)
        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def _rewire(
        self, new_node: Node, near_inds: List[int], obstacle_list: List[Obstacle]
    ):
        for i in near_inds:
            near_node = self.node_list[i]

            # Check if rewiring through new_node is shorter
            d = self._calc_dist(new_node, near_node)
            scost = new_node.cost + d

            if near_node.cost > scost:
                if self._check_collision_line(new_node, near_node, obstacle_list):
                    near_node.parent = new_node
                    near_node.cost = scost
                    # Propagate cost updates to children? (In strictly RRT* yes, but expensive in simple py list)

    def _calc_dist(self, n1: Node, n2: Node):
        return np.linalg.norm(n1.position() - n2.position())

    def _calc_dist_to_goal(self, n: Node, goal: Node):
        return self._calc_dist(n, goal)

    def _generate_final_course(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.position())
            node = node.parent
        path.append(node.position())
        return path[::-1]  # Reverse

    def _get_best_last_index(self, goal_node: Node):
        # Find node closest to goal
        disglist = [self._calc_dist_to_goal(node, goal_node) for node in self.node_list]
        minind = disglist.index(min(disglist))
        # Optional: threshold?
        return minind
