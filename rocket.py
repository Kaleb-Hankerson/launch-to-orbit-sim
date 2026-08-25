#This is the rocket class for the launch-to-orbit sim. It holds variables for mass,dry mass,height,velocity,
#thrust,exhaust velocity,mass flow rate, and gravity. For the purposes of this sim, exhaust velocity and mass flow rate
#are left constant, and as a byproduct, thrust is also constant. The rest will change over time, mass lowering as fuel
#is spent, velocity and acceleration increasing as mass decreases, height increasing, etc.
import numpy as np
import math

class Rocket:
    def __init__(self, mass, dry_mass, exhaust_velocity, mass_flow_rate):
        self._mass = mass
        self._dry_mass = dry_mass
        self._exhaust_velocity  = exhaust_velocity
        self._mass_flow_rate = mass_flow_rate
        self._thrust = 0.0
        self._net_force = 0.0
        self._accel = np.array([0.0,0.0])
        self._velocity = np.array([0.0,0.0])
        self._position = np.array([0.0,0.0])
        self._drag = np.array([0.0,0.0])
        self._dt = 0.1
        self._pitch_start_angle = 90.0
        self._pitch_end_angle = 45.0
        self._pitch_duration = 100.0
        self._angle = 0.0
        self._time = 0.0
        self._GRAVITY = 9.8
        self._SEA_LEVEL_DENSITY = 1.225
        self._SCALE_HEIGHT = 8500.0
        self._air_density = 0.0
        #constant for simplicity
        self._DRAG_COEFFICIENT = 0.3
        #chosen from a 2.5m diameter frontal of the rocket
        self._cross_sectional_area = 4.9
    #Thrust is found by multiplying exhaust velocity and mass flow rate.
    def calc_thrust(self):
        #Thrusts calculates while the mass is still greater than the dry mass, indicating there is still propellant
        if self._mass > self._dry_mass:
            self._thrust = self._exhaust_velocity * self._mass_flow_rate
        #If mass is no longer great than dry mass, then there must be no more propellant left, thus no thrust
        else:
            self._thrust = 0

    def calc_density(self):
        self._air_density = self._SEA_LEVEL_DENSITY * math.exp(-self._position[1] / self._SCALE_HEIGHT)

    def calc_drag(self):
        speed = np.linalg.norm(self._velocity)
        if speed > 0:
            drag_magnitude = 0.5 * self._air_density * (speed ** 2) * self._DRAG_COEFFICIENT * self._cross_sectional_area
            drag_x = -drag_magnitude * (self._velocity[0] / speed)
            drag_y = -drag_magnitude * (self._velocity[1] / speed)
            self._drag = np.array([drag_x, drag_y])
        else:
        #division by zero guard
            self._drag = np.array([0.0,0.0])

    #Net force takes the angle of the rocket and converts it to radians. It then finds the x thrust from the cos of that
    #angle multiplied against the thrust value and the y thrust from the sin of that angle multiplied against the thrust
    #value. Then the net force for x and y is found as well. The x is just that since gravity has no effect on
    #horizontal force, and y is the thrust of y minus the effects of gravity.
    def calc_net_force(self):
        angle_radians = math.radians(self._angle)
        thrust_x = self._thrust * math.cos(angle_radians)
        thrust_y = self._thrust * math.sin(angle_radians)
        net_force_x = thrust_x + self._drag[0]
        net_force_y = thrust_y - (self._mass * self._GRAVITY) + self._drag[1]
        self._net_force = np.array([net_force_x, net_force_y])

    def calc_accel(self):
        self._accel = self._net_force / self._mass
    def update_velocity(self):
        self._velocity = self._velocity + (self._accel * self._dt)
    def update_height(self):
        self._position = self._position + (self._velocity * self._dt)
    def update_mass(self):
        #Update mass while it is still above dry mass
        if self._mass > self._dry_mass:
            self._mass = self._mass - (self._mass_flow_rate * self._dt)
        #This stops the mass from dropping below the dry mass
        else:
            self._mass = self._dry_mass
    def calc_pitch_angle(self):
        if self._time < self._pitch_duration:
            self._angle = self._pitch_start_angle - (self._pitch_start_angle - self._pitch_end_angle) * (self._time / self._pitch_duration)
        else:
            self._angle = self._pitch_end_angle
    def update(self):
        self.calc_pitch_angle()
        self.calc_thrust()
        self.calc_density()
        self.calc_drag()
        self.calc_net_force()
        self.calc_accel()
        self.update_velocity()
        self.update_height()
        self.update_mass()
        self._time = self._time + self._dt

    @property
    def position(self):
        return self._position
    @property
    def velocity(self):
        return self._velocity
    @property
    def mass(self):
        return self._mass