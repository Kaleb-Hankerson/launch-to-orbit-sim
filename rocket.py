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
        # Earth-rotation initial velocity (launch site tangential speed)
        launch_latitude = 34.7  # degrees, approximate Huntsville/UAH latitude
        earth_angular_velocity = 7.292e-5  # rad/s
        earth_radius = 6.371e6  # meters
        initial_vx = earth_angular_velocity * earth_radius * math.cos(math.radians(launch_latitude))
        self._velocity = np.array([initial_vx, 0])
        self._position = np.array([0.0,0.0])
        self._dt = 0.1
        self._q = 0.0
        self._max_q = 0.0
        self._pitch_start_angle = 90.0
        self._pitch_end_angle = 45.0
        self._pitch_duration = 100.0
        self._time = 0.0
        self._GRAVITY = 9.8
        self._SEA_LEVEL_DENSITY = 1.225
        self._SCALE_HEIGHT = 8500.0
        #constant for simplicity
        self._DRAG_COEFFICIENT = 0.3
        #chosen from a 2.5m diameter frontal of the rocket
        self._cross_sectional_area = 4.9

#---------------- Start of helper methods for calc_derivatives -------------------------------------------------------
    def calc_pitch_angle(self,time):
        if time < self._pitch_duration:
            return  self._pitch_start_angle - (self._pitch_start_angle - self._pitch_end_angle) * (time / self._pitch_duration)
        else:
            return self._pitch_end_angle

    #Thrust is found by multiplying exhaust velocity and mass flow rate.
    def calc_thrust(self, mass):
        #Thrusts calculates while the mass is still greater than the dry mass, indicating there is still propellant
        if mass > self._dry_mass:
            return self._exhaust_velocity * self._mass_flow_rate
        #If mass is no longer greater than dry mass, then there must be no more propellant left, thus no thrust
        else:
            return 0

    def calc_mass_rate(self,mass):
        if mass > self._dry_mass:
            return -self._mass_flow_rate
        else:
            return 0

    def calc_density(self,position):
         return self._SEA_LEVEL_DENSITY * math.exp(-position[1] / self._SCALE_HEIGHT)

    def calc_drag(self,velocity, air_density):
        speed = np.linalg.norm(velocity)
        if speed > 0:
            drag_magnitude = 0.5 * air_density * (speed ** 2) * self._DRAG_COEFFICIENT * self._cross_sectional_area
            drag_x = -drag_magnitude * (velocity[0] / speed)
            drag_y = -drag_magnitude * (velocity[1] / speed)
            return np.array([drag_x, drag_y])
        else:
            return np.array([0.0,0.0])

    #Net force takes the angle of the rocket and converts it to radians. It then finds the x thrust from the cos of that
    #angle multiplied against the thrust value and the y thrust from the sin of that angle multiplied against the thrust
    #value. Then the net force for x and y is found as well. The x is just that since gravity has no effect on
    #horizontal force, and y is the thrust of y minus the effects of gravity.
    def calc_net_force(self, thrust, drag, angle, mass):
        angle_radians = math.radians(angle)
        thrust_x = thrust * math.cos(angle_radians)
        thrust_y = thrust * math.sin(angle_radians)
        net_force_x = thrust_x + drag[0]
        net_force_y = thrust_y - (mass * self._GRAVITY) + drag[1]
        return np.array([net_force_x, net_force_y])

    def calc_accel(self,net_force, mass):
        return net_force / mass
#----------------------- End of helper methods ------------------------------------------------------------

    #This function finds the derivates(rates of change) for position, velocity, and mass at the given state. Since it is
    #only concerned with the pass values, it can account for different states when called in the RK4 stepper.
    #It sequentially calls the helper functions in the order they are needed.
    def calc_derivatives(self, position, velocity, mass, time):
        angle = self.calc_pitch_angle(time)
        mass_rate = self.calc_mass_rate(mass)
        thrust = self.calc_thrust(mass)
        air_density = self.calc_density(position)
        drag = self.calc_drag(velocity, air_density)
        net_force = self.calc_net_force(thrust, drag, angle, mass)
        accel = self.calc_accel(net_force, mass)

        return velocity, accel, mass_rate

    def rk4_stepper(self):
        k1 = self.calc_derivatives(self._position, self._velocity, self._mass, self._time)
        half_dt = self._dt / 2
        mid_position_1 = self._position + k1[0] * half_dt
        mid_velocity_1 = self._velocity + k1[1] * half_dt
        mid_mass_1 = self._mass + k1[2] * half_dt
        mid_time_1 = self._time + half_dt

        k2 = self.calc_derivatives(mid_position_1, mid_velocity_1, mid_mass_1, mid_time_1)
        mid_position_2 = self._position + k2[0] * half_dt
        mid_velocity_2 = self._velocity + k2[1] * half_dt
        mid_mass_2 = self._mass + k2[2] * half_dt
        mid_time_2 = self._time + half_dt

        k3 = self.calc_derivatives(mid_position_2, mid_velocity_2, mid_mass_2, mid_time_2)
        end_position = self._position + k3[0] * self._dt
        end_velocity = self._velocity + k3[1] * self._dt
        end_mass = self._mass + k3[2] * self._dt
        end_time = self._time + self._dt

        k4 = self.calc_derivatives(end_position, end_velocity, end_mass, end_time)

        final_velocity_rate = (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6
        final_accel = (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6
        final_mass_rate = (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6

        self._position = self._position + final_velocity_rate * self._dt
        self._velocity = self._velocity + final_accel * self._dt
        self._mass = self._mass + final_mass_rate * self._dt
        self._time = self._time + self._dt

        #Dynamic pressure
        air_density = self.calc_density(self._position)
        speed = np.linalg.norm(self._velocity)
        self._q = 0.5 * air_density * (speed ** 2)
        if self._q > self._max_q:
            self._max_q = self._q

    @property
    def position(self):
        return self._position
    @property
    def velocity(self):
        return self._velocity
    @property
    def mass(self):
        return self._mass
    @property
    def q(self):
        return self._q
    @property
    def max_q(self):
        return self._max_q