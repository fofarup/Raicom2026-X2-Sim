"""空白栅格激光建图、A* 规划与前向碰撞监测。"""
from __future__ import annotations

import heapq
import math
import struct
import time
from typing import Callable

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def bresenham(a: tuple[int, int], b: tuple[int, int]):
    x0, y0 = a
    x1, y1 = b
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            break
        twice = 2 * error
        if twice >= dy:
            error, x0 = error + dy, x0 + sx
        if twice <= dx:
            error, y0 = error + dx, y0 + sy


class OccupancyMap:
    def __init__(self, size_m=6.0, resolution=0.05):
        self.resolution = resolution
        self.width = self.height = round(size_m / resolution)
        self.origin = -size_m / 2
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (int((x - self.origin) / self.resolution),
                int((y - self.origin) / self.resolution))

    def world(self, cell: tuple[int, int]) -> tuple[float, float]:
        x, y = cell
        return (self.origin + (x + 0.5) * self.resolution,
                self.origin + (y + 0.5) * self.resolution)

    def valid(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def add_ray(self, origin_xy, hit_xy):
        start, end = self.cell(*origin_xy), self.cell(*hit_xy)
        cells = list(bresenham(start, end))
        for x, y in cells[:-1]:
            if self.valid((x, y)) and self.grid[y, x] != 100:
                self.grid[y, x] = 0
        if self.valid(end):
            self.grid[end[1], end[0]] = 100

    def inflated(self, radius_m=0.28):
        blocked = self.grid >= 50
        radius = max(1, round(radius_m / self.resolution))
        inflated = blocked.copy()
        ys, xs = np.nonzero(blocked)
        for y, x in zip(ys, xs):
            y0, y1 = max(0, y - radius), min(self.height, y + radius + 1)
            x0, x1 = max(0, x - radius), min(self.width, x + radius + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            inflated[y0:y1, x0:x1] |= (xx - x) ** 2 + (yy - y) ** 2 <= radius ** 2
        return inflated

    def plan(self, start_xy, goal_xy):
        start, goal = self.cell(*start_xy), self.cell(*goal_xy)
        if not self.valid(start) or not self.valid(goal):
            return []
        blocked = self.inflated()
        # Clear the robot footprint in the planning copy. Rays can strike
        # arms/legs outside the lidar camera body; inflation of those hits can
        # otherwise form a closed ring around the start immediately after a
        # reset. The observed occupancy grid itself remains unchanged.
        footprint = max(1, round(0.30 / self.resolution))
        sx, sy = start
        y0, y1 = max(0, sy - footprint), min(self.height, sy + footprint + 1)
        x0, x1 = max(0, sx - footprint), min(self.width, sx + footprint + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        blocked[y0:y1, x0:x1] &= (
            (xx - sx) ** 2 + (yy - sy) ** 2 > footprint ** 2)
        blocked[goal[1], goal[0]] = False
        queue = [(0.0, start)]
        came_from, cost = {}, {start: 0.0}
        moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for dx, dy, step in moves:
                nxt = current[0] + dx, current[1] + dy
                if not self.valid(nxt) or blocked[nxt[1], nxt[0]]:
                    continue
                unknown_penalty = 0.25 if self.grid[nxt[1], nxt[0]] < 0 else 0.0
                candidate = cost[current] + step + unknown_penalty
                if candidate < cost.get(nxt, math.inf):
                    cost[nxt], came_from[nxt] = candidate, current
                    heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(queue, (candidate + heuristic, nxt))
        if goal not in cost:
            return []
        path, current = [goal], goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # 贪心视线压缩：保留绕障拐点，删除同一直线走廊内的反复起停点。
        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            candidate = len(path) - 1
            while candidate > anchor + 1:
                if all(self.valid(cell) and not blocked[cell[1], cell[0]]
                       for cell in bresenham(path[anchor], path[candidate])):
                    break
                candidate -= 1
            simplified.append(path[candidate])
            anchor = candidate
        result = [self.world(cell) for cell in simplified[1:]]
        if not result or result[-1] != tuple(goal_xy):
            result.append(tuple(goal_xy))
        return result


class LidarMapper:
    TOPIC = "/aima/sim/lidar/points"

    def __init__(self, node, pose_getter: Callable):
        self.node, self.pose_getter = node, pose_getter
        self.map = OccupancyMap()
        self.last_scan_at = None
        self.front_clearance = math.inf
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(PointCloud2, self.TOPIC, self._scan, qos)
        self.publisher = node.create_publisher(OccupancyGrid, "/map", 1)

    @property
    def ready(self):
        return self.last_scan_at is not None and time.monotonic() - self.last_scan_at < 0.5

    def reset(self):
        """Discard scans acquired before a MuJoCo world reset."""
        self.map = OccupancyMap()
        self.last_scan_at = None
        self.front_clearance = math.inf

    def safe_to_advance(self, clearance=0.42):
        return not self.ready or self.front_clearance >= clearance

    @staticmethod
    def _points(msg, stride=1):
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(axis in offsets for axis in "xyz") or msg.point_step < 12:
            return
        for index in range(7, msg.width * msg.height, stride):
            base = index * msg.point_step
            point = tuple(struct.unpack_from("<f", msg.data, base + offsets[axis])[0]
                          for axis in "xyz")
            if all(math.isfinite(value) for value in point):
                yield point

    def _scan(self, msg):
        now = time.monotonic()
        if self.last_scan_at is not None and now - self.last_scan_at < 0.18:
            return
        pose = self.pose_getter()
        if pose is None or msg.width == 0:
            return
        px, py, _, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        front = math.inf
        for x, y, z in self._points(msg, stride=4):
            if msg.header.frame_id == "map":
                wx, wy = x, y
                dx, dy = wx - px, wy - py
                bx, by = c * dx + s * dy, -s * dx + c * dy
            else:
                bx, by = x, y
                wx, wy = px + c * bx - s * by, py + s * bx + c * by
            horizontal_range = math.hypot(bx, by)
            # Ignore the robot's own limbs close to the pelvis-mounted sensor.
            if 0.60 <= horizontal_range <= 7.9 and 0.15 <= z <= 1.8:
                self.map.add_ray((px, py), (wx, wy))
                if bx > 0 and abs(math.atan2(by, bx)) < math.radians(25):
                    front = min(front, horizontal_range)
        self.front_clearance, self.last_scan_at = front, now
        self.publish()

    def publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.map.resolution
        msg.info.width, msg.info.height = self.map.width, self.map.height
        msg.info.origin.position.x = msg.info.origin.position.y = self.map.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map.grid.reshape(-1).astype(int).tolist()
        self.publisher.publish(msg)
