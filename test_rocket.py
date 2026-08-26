import unittest
import numpy as np
from rocket import Rocket

class TestRocket(unittest.TestCase):
    def test_ballistic_motion(self):
        #For this test we are eliminating thrust and drag to test the verifiable physics loop of the simulation
        #Thrust numbers, drag numbers, etc. can change test to test, but the physics of the sim should remain constant in
        #each iteration
        test_rocket = Rocket(mass=1000, dry_mass=1000, exhaust_velocity=0, mass_flow_rate=0)
        test_rocket._cross_sectional_area = 0

        #Because no thrust we assign a known, initial starting value
        test_rocket._velocity = np.array([0, 100])  #no horizontal velocity, rocket is traveling straight up at 100 m/s

        #Setting the duration of the test for 5 seconds, which will give us 50 steps since our dt is 0.1s
        duration = 5
        steps = int(duration / test_rocket._dt)
        #Passing number of steps into loop to call update for each step
        for _ in range(steps):
            test_rocket.update()

        #Expected height formula is analytical formula for projectile motion under constant gravity
        expected_height = 100 * duration - 0.5 * 9.8 * duration ** 2
        #This compares the sim result (test_rocket.position[1]) against the expected result (expected_height), with a
        #tolerance of 3 meters.
        self.assertAlmostEqual(test_rocket.position[1], expected_height, delta=3)

if __name__ == '__main__':
    unittest.main()