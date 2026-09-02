#This is the rocket class for the 2D launch-to-orbit sim. Models a staged rocket (thrust, exhaust velocity,
#and mass flow rate vary per stage) under gravity, atmospheric drag, and a linear pitch program. Uses RK4
#integration via calc_derivatives()/rk4_stepper(). Tracks flight phase (a state machine), dynamic pressure/Max Q,
#and orbital elements (eccentricity, period) once orbit is detected via the vis-viva equation.

import numpy as np
import math
from enum import Enum

class FlightPhase(Enum):
    PRE_LAUNCH = 1
    POWERED_FLIGHT = 2
    COAST = 3
    ORBIT_INSERTION = 4


class Rocket:
    def __init__(self):
        self._flight_phase = FlightPhase.PRE_LAUNCH
        #Ordered list of stages, burned in sequence (index 0 first). Each stage's propellant_mass and
        #mass_flow_rate determine its burn time; structural_mass is the stage's own empty hardware weight,
        #jettisoned once that stage's propellant is exhausted (except the final stage, which stays attached).
        self._stages = [
            {"exhaust_velocity": 3000, "mass_flow_rate": 250, "propellant_mass": 38000, "structural_mass": 4000},
            {"exhaust_velocity": 3500, "mass_flow_rate": 60, "propellant_mass": 5000, "structural_mass": 500},
        ]
        self._current_stage_index = 0
        self._mass = sum(stage["propellant_mass"] + stage["structural_mass"] for stage in self._stages)
        self._initial_total_mass = self._mass
        self._final_dry_mass = self._stages[-1]["structural_mass"]
        #Earth-rotation initial velocity (launch site tangential speed)
        self._launch_latitude = 34.7  # degrees, approximate Huntsville/UAH latitude
        self._earth_angular_velocity = 7.292e-5  # rad/s
        self._earth_radius = 6.371e6  # meters
        self._GM = 3.986e14
        self._initial_vx = self._earth_angular_velocity * self._earth_radius * math.cos(math.radians(self._launch_latitude))
        self._velocity = np.array([self._initial_vx, 0])
        self._position = np.array([0.0,0.0])
        self._accel = np.array([0.0,0.0])
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
        self._eccentricity = 0.0
        self._period = 0.0

#---------------- Start of helper methods for calc_derivatives -------------------------------------------------------
    def calc_pitch_angle(self,time):
        if time < self._pitch_duration:
            return  self._pitch_start_angle - (self._pitch_start_angle - self._pitch_end_angle) * (time / self._pitch_duration)
        else:
            return self._pitch_end_angle

    #Thrust is found by multiplying exhaust velocity and mass flow rate.
    def calc_thrust(self, mass, stage, stage_burnout_mass):
        #Thrust calculates while mass is still greater than this stage's burnout threshold, indicating there is
        #still propellant in this stage
        if mass > stage_burnout_mass:
            return stage["exhaust_velocity"] * stage["mass_flow_rate"]
        #If mass is no longer greater than this stage's burnout threshold, this stage's propellant is exhausted,
        #thus no thrust
        else:
            return 0

    def calc_mass_rate(self,mass, stage, stage_burnout_mass):
        if mass > stage_burnout_mass:
            return -stage["mass_flow_rate"]
        else:
            return 0

    def get_stage_and_burnout(self, mass):
        cumulative_mass = 0
        for i, stage in enumerate(self._stages):
            cumulative_mass += stage["propellant_mass"] + stage["structural_mass"]
            stage_burnout_mass = self._initial_total_mass - cumulative_mass + stage["structural_mass"]
            if mass > stage_burnout_mass:
                return stage, stage_burnout_mass
            #If mass has dropped below everything, then rocket is in final stage's burnout
        return self._stages[-1], self._final_dry_mass

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

    #This function finds the derivatives(rates of change) for position, velocity, and mass at the given state.
    #Since it is only concerned with the pass values, it can account for different states when called in
    #the RK4 stepper.
    #It sequentially calls the helper functions in the order they are needed.
    def calc_derivatives(self, position, velocity, mass, time):
        stage, stage_burnout_mass = self.get_stage_and_burnout(mass)
        angle = self.calc_pitch_angle(time)
        mass_rate = self.calc_mass_rate(mass, stage, stage_burnout_mass)
        thrust = self.calc_thrust(mass, stage, stage_burnout_mass)
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
        self._accel = final_accel
        final_mass_rate = (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6

        self._position = self._position + final_velocity_rate * self._dt
        self._velocity = self._velocity + final_accel * self._dt
        self._mass = self._mass + final_mass_rate * self._dt
        if self._mass < self._final_dry_mass:
            self._mass = self._final_dry_mass
        self._time = self._time + self._dt

        #Stage transition: if the (mass-derived) current stage no longer matches the tracked index,
        #we've crossed a burnout boundary — jettison the previous stage's structural mass and advance.
        current_stage, current_burnout = self.get_stage_and_burnout(self._mass)
        if current_stage is not self._stages[self._current_stage_index]:
            #We've crossed into a new stage — jettison the previous stage's structure
            jettisoned_stage = self._stages[self._current_stage_index]
            self._mass -= jettisoned_stage["structural_mass"]
            self._current_stage_index += 1

        #Dynamic pressure
        air_density = self.calc_density(self._position)
        speed = np.linalg.norm(self._velocity)
        self._q = 0.5 * air_density * (speed ** 2)
        if self._q > self._max_q:
            self._max_q = self._q


        #Flight Phase
        self.calc_flight_phase(self._mass,self._position,self._velocity)

    def calc_flight_phase(self, mass, position, velocity):
        if mass > self._final_dry_mass:
            self._flight_phase = FlightPhase.POWERED_FLIGHT
        else:
            earth_centered_position = np.array([position[0], self._earth_radius + position[1]])
            distance = np.linalg.norm(earth_centered_position)
            speed = np.linalg.norm(velocity)
            semi_major_axis = self.calc_semi_major_axis(speed, distance)
            if semi_major_axis > 0:
                self._flight_phase = FlightPhase.ORBIT_INSERTION
                self._eccentricity = self.calc_eccentricity(earth_centered_position, velocity, semi_major_axis)
                self._period = self.calc_period(semi_major_axis)
            else:
                self._flight_phase = FlightPhase.COAST

    def calc_eccentricity(self, earth_centered_position, velocity, semi_major_axis):
        h = (earth_centered_position[0] * velocity[1] - (earth_centered_position[1] * velocity[0]))
        return math.sqrt(1 - (h ** 2) / (self._GM * semi_major_axis))

    def calc_period(self, semi_major_axis):
        return 2 * math.pi * math.sqrt(semi_major_axis ** 3 / self._GM)

    def calc_semi_major_axis(self,speed,distance):
        return 1 / (2 / distance - (speed**2)/self._GM)

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
    @property
    def flight_phase(self):
        return self._flight_phase
    @property
    def eccentricity(self):
        return self._eccentricity
    @property
    def period(self):
        return self._period
    @property
    def accel(self):
        return self._accel
    @property
    def stages(self):
        return self._stages