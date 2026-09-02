import unittest
import numpy as np
from rocket import Rocket

class TestRocket(unittest.TestCase):
    def test_ballistic_motion(self):
        #For this test we are eliminating thrust and drag to test the verifiable physics loop of the simulation.
        #Thrust numbers, drag numbers, etc. can change test to test, but the physics of the sim should remain
        #constant in each iteration
        test_rocket = Rocket()
        test_rocket._cross_sectional_area = 0
        #Small nonzero propellant_mass avoids an exact-equality edge case in get_stage_and_burnout's boundary
        #check (mass > stage_burnout_mass fails when they're exactly equal, which round test numbers would hit)
        test_rocket._stages = [
            {"exhaust_velocity": 0, "mass_flow_rate": 0, "propellant_mass": 1, "structural_mass": 999},
            {"exhaust_velocity": 0, "mass_flow_rate": 0, "propellant_mass": 1, "structural_mass": 999},
        ]
        test_rocket._mass = 1000
        test_rocket._initial_total_mass = 1000
        test_rocket._final_dry_mass = 999
        # Because there's no thrust, we assign a known, controlled starting velocity
        test_rocket._velocity = np.array([0, 100])
        # 5 seconds at dt=0.1s gives 50 steps
        duration = 5
        steps = int(duration / test_rocket._dt)
        for _ in range(steps):
            test_rocket.rk4_stepper()
        #Analytical formula for projectile motion under constant gravity
        expected_height = 100 * duration - 0.5 * 9.8 * duration ** 2
        #Compares the sim result against the analytical answer, with a 3m tolerance (RK4's accumulated
        #numerical error over 50 steps is far smaller than this — see notes on the Euler-based Python
        #comparison, which needed the same tolerance despite being much less accurate than RK4)
        self.assertAlmostEqual(test_rocket.position[1], expected_height, delta=3)

if __name__ == '__main__':
    unittest.main()