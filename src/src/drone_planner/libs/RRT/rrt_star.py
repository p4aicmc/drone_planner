#!/usr/bin/env python3
"""

Path planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)

Adapted for the Harpia planner: it subclasses the existing RRT so that the
sampling, steering and collision-checking primitives (including the
``ray_casting`` mode used against the NFZ polygons) are shared verbatim and the
two planners are drop-in replacements for each other in path_planner.py.  The
only differences are the ones that define RRT*, i.e. `choose_parent` (pick the
cheapest reachable parent inside the near-radius) and `rewire` (re-parent
neighbours through the new node when that is cheaper).

"""

import math
import os
import sys
import time

# Adiciona o diretório base ao sys.path
libs_path = os.path.join(os.path.dirname(__file__), '../.')
sys.path.append(os.path.abspath(libs_path))

from libs.RRT.rrt import RRT


class RRTStar(RRT):
    """
    Class for RRT* planning
    """

    class Node(RRT.Node):
        """RRT* node: an RRT node that also carries the cost-to-come."""

        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5,
                 max_iter=500, check_collision_mode='circle',
                 connect_circle_dist=50.0, search_until_max_iter=True,
                 max_planning_time=None, optimality_tolerance=0.01,
                 optimality_check_every=50):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList: same semantics as RRT, see rrt.py
        randArea:Random Sampling Area [min,max]
        connect_circle_dist: gamma of the shrinking near-radius
            r = connect_circle_dist * sqrt(log(n)/n), capped at expand_dis.
        search_until_max_iter: if True, keep sampling (and therefore keep
            improving the solution) until max_iter even after a first path is
            found.  This is what makes RRT* converge towards the optimum; with
            False it returns on the first connection and behaves much like RRT.
        max_planning_time: optional wall-clock budget in seconds, counted
            from the start of planning but only enforced once a solution
            exists.  RRT* is an anytime algorithm, so cutting the refinement
            short only costs path quality, never correctness.  The search for a
            first path always runs to max_iter, so the budget can never turn a
            solvable problem into a failure; the worst-case wall time is
            therefore max(time to first solution, budget) plus one check
            interval, since the budget is tested together with the optimality
            test, i.e. every optimality_check_every iterations.
            The cost per iteration grows with the tree, and it degenerates to
            O(n^2) when the sampling area is small enough that every node falls
            inside the rewire ball - which is exactly what happens on a very
            short leg, where rand_area collapses to a couple of metres.  Since
            planning runs inside a blocking service callback, this budget is
            what bounds the service latency.
        optimality_tolerance: relative gap at which the refinement stops.  The
            straight line start->goal is a lower bound on the length of any
            feasible path, so once the best solution is within this fraction of
            it, further sampling is provably unable to improve the result and
            is pure latency.  This is what keeps a trivial leg (drone already
            parked on the base) from burning the whole time budget.
        optimality_check_every: how many iterations between those checks.
            search_best_goal_node() collision-checks every node near the goal,
            so it is far too expensive to run on every iteration.
        """
        print('Initiated RRT*')
        super().__init__(
            start=start,
            goal=goal,
            obstacle_list=obstacle_list,
            rand_area=rand_area,
            expand_dis=expand_dis,
            path_resolution=path_resolution,
            goal_sample_rate=goal_sample_rate,
            max_iter=max_iter,
            check_collision_mode=check_collision_mode,
        )
        self.connect_circle_dist = connect_circle_dist
        self.search_until_max_iter = search_until_max_iter
        self.max_planning_time = max_planning_time
        self.optimality_tolerance = optimality_tolerance
        self.optimality_check_every = optimality_check_every
        # self.start / self.end are already RRTStar.Node instances: RRT.__init__
        # builds them through self.Node, which resolves on the subclass.
        self.goal_node = self.Node(goal[0], goal[1])
        print('Finished initiating RRT*')

    def planning(self, animation=False):
        """
        rrt star path planning

        animation: kept for signature compatibility with RRT; unused (the
        node runs headless).
        """
        print('Started RRT* planning')

        self.node_list = [self.start]
        deadline = None
        if self.max_planning_time is not None:
            deadline = time.perf_counter() + self.max_planning_time

        # Any feasible path is at least as long as the straight line between
        # the endpoints, so this is a lower bound on the optimum.
        lower_bound = self.calc_dist_to_goal(self.start.x, self.start.y)

        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if not self.search_until_max_iter:
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
            elif (i + 1) % self.optimality_check_every == 0:
                cost = self.best_solution_cost()
                if cost is None:
                    # No solution yet: keep searching, whatever the budget says.
                    continue

                # Refining a solution that already sits on the lower bound
                # cannot improve it, so stop paying for it.  The absolute
                # slack keeps a degenerate leg (start == goal, lower_bound 0)
                # from looping until the deadline.
                slack = lower_bound * self.optimality_tolerance + self.path_resolution
                if cost <= lower_bound + slack:
                    print(f'RRT* reached the optimality bound after {i + 1} '
                          f'iterations ({cost:.1f} m vs a {lower_bound:.1f} m '
                          f'lower bound); stopping the refinement')
                    break

                if deadline is not None and time.perf_counter() > deadline:
                    print(f'RRT* refinement budget exhausted after {i + 1} '
                          f'iterations; returning the best solution found so far')
                    break

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None  # cannot find path

    def choose_parent(self, new_node, near_inds):
        """
        Re-parent new_node to the neighbour that yields the lowest cost-to-come
        while keeping the connecting edge collision free.

        Returns the re-parented node, or None when no neighbour can be reached.
        """
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the path is blocked

        min_cost = min(costs)
        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        """Index of the cheapest node from which the goal is directly reachable."""
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            i for i, d in enumerate(dist_to_goal_list) if d <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        safe_goal_costs = [
            self.node_list[i].cost + dist_to_goal_list[i] for i in safe_goal_inds
        ]
        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    def best_solution_cost(self):
        """Length of the best start->goal path in the tree, or None if there is none."""
        best_ind = self.search_best_goal_node()
        if best_ind is None:
            return None
        node = self.node_list[best_ind]
        return node.cost + self.calc_dist_to_goal(node.x, node.y)

    def find_near_nodes(self, new_node):
        """
        Neighbours of new_node inside the shrinking ball
        r = connect_circle_dist * sqrt(log(n)/n), capped at expand_dis so the
        connection stays reachable by a single steer().
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than expand_dist
        r = min(r, self.expand_dis)
        r_squared = r ** 2
        return [
            i for i, node in enumerate(self.node_list)
            if (node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 <= r_squared
        ]

    def rewire(self, new_node, near_inds):
        """
        Re-parent every neighbour that becomes cheaper when reached through
        new_node, then push the cost update down its subtree.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                for node in self.node_list:
                    if node.parent is near_node:
                        node.parent = edge_node
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        """
        Iterative (stack based) cost propagation.  The recursive version of this
        routine blows Python's recursion limit on the multi-thousand node trees
        this planner builds.
        """
        stack = [parent_node]
        while stack:
            parent = stack.pop()
            for node in self.node_list:
                if node.parent is parent:
                    node.cost = self.calc_new_cost(parent, node)
                    stack.append(node)


def main():
    print("Start " + __file__)

    # ====Search Path with RRT*====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x, y, radius]

    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=3.0,
        check_collision_mode='circle',
    )
    path = rrt_star.planning()

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")


if __name__ == '__main__':
    main()
