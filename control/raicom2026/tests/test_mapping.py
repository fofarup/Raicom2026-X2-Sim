import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.mapping import OccupancyMap, bresenham


class MappingTests(unittest.TestCase):
    def test_bresenham_endpoints(self):
        cells = list(bresenham((1, 2), (5, 4)))
        self.assertEqual((1, 2), cells[0])
        self.assertEqual((5, 4), cells[-1])

    def test_ray_marks_free_and_hit(self):
        grid = OccupancyMap(size_m=4, resolution=0.1)
        grid.add_ray((0, 0), (1, 0))
        self.assertEqual(100, grid.grid[grid.cell(1, 0)[1], grid.cell(1, 0)[0]])
        self.assertEqual(0, grid.grid[grid.cell(0.5, 0)[1], grid.cell(0.5, 0)[0]])

    def test_astar_routes_around_wall(self):
        grid = OccupancyMap(size_m=4, resolution=0.1)
        x = grid.cell(0, 0)[0]
        grid.grid[:, x] = 100
        gap_y = grid.cell(0, 1.2)[1]
        grid.grid[gap_y - 4:gap_y + 5, x] = 0
        route = grid.plan((-1, 0), (1, 0))
        self.assertTrue(route)
        self.assertGreater(max(y for _, y in route), 0.7)

    def test_planner_clears_inflated_robot_footprint(self):
        grid = OccupancyMap(size_m=4.0, resolution=0.05)
        # A stray return inside the occupied footprint must not trap the start.
        x, y = grid.cell(0.10, 0.0)
        grid.grid[y, x] = 100
        self.assertTrue(grid.plan((0.0, 0.0), (1.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
