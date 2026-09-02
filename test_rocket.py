import unittest
import numpy as np
from rocket import Rocket

class TestRocket(unittest.TestCase):
    def test_ballistic_motion(self):
        test_rocket = Rocket()
        test_rocket._cross_sectional_area = 0
        test_rocket._stages = [
            {"exhaust_velocity": 0, "mass_flow_rate": 0, "propellant_mass": 1, "structural_mass": 999},
            {"exhaust_velocity": 0, "mass_flow_rate": 0, "propellant_mass": 1, "structural_mass": 999},
        ]
        test_rocket._mass = 1000
        test_rocket._initial_total_mass = 1000
        test_rocket._final_dry_mass = 999

        test_rocket._velocity = np.array([0, 100])

        duration = 5
        steps = int(duration / test_rocket._dt)
        for _ in range(steps):
            test_rocket.rk4_stepper()

        expected_height = 100 * duration - 0.5 * 9.8 * duration ** 2
        self.assertAlmostEqual(test_rocket.position[1], expected_height, delta=3)

if __name__ == '__main__':
    unittest.main()