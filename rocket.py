#This is the rocket class for the launch-to-orbit sim. It holds variables for mass,dry mass,height,velocity,
#thrust,exhaust velocity,mass flow rate, and gravity. For the purposes of this sim, exhaust velocity and mass flow rate
#are left constant, and as a byproduct, thrust is also constant. The rest will change over time, mass lowering as fuel
#is spent, velocity and acceleration increasing as mass decreases, height increasing, etc.

class Rocket:
    def __init__(self, mass, dry_mass, exhaust_velocity, mass_flow_rate):
        self._mass = mass
        self._dry_mass = dry_mass
        self._exhaust_velocity  = exhaust_velocity
        self._mass_flow_rate = mass_flow_rate
        self._thrust = 0
        self._net_force = 0
        self._accel = 0
        self._velocity = 0
        self._height = 0
        self._dt = 0.1
    #Thrust is found by multiplying exhaust velocity and mass flow rate.
    def calc_thrust(self):
        #Thrusts calculates while the mass is still greater than the dry mass, indicating there is still propellant
        if self._mass > self._dry_mass:
            self._thrust = self._exhaust_velocity * self._mass_flow_rate
        #If mass is no longer great than dry mass, then there must be no more propellant left, thus no thrust
        else:
            self._thrust = 0
    #Net force takes the thrust value and subtracts the force of gravity from it to get our net force on the rocket.
    def calc_net_force(self):
        self._net_force = self._thrust - (self._mass * 9.8)
    def calc_accel(self):
        self._accel = self._net_force / self._mass
    def update_velocity(self):
        self._velocity = self._velocity + (self._accel * self._dt)
    def update_height(self):
        self._height = self._height + (self._velocity * self._dt)
    def update_mass(self):
        #Update mass while it is still above dry mass
        if self._mass > self._dry_mass:
            self._mass = self._mass - (self._mass_flow_rate * self._dt)
        #This stops the mass from dropping below the dry mass
        else:
            self._mass = self._dry_mass
    def update(self):
        self.calc_thrust()
        self.calc_net_force()
        self.calc_accel()
        self.update_velocity()
        self.update_height()
        self.update_mass()

    @property
    def height(self):
        return self._height
    @property
    def velocity(self):
        return self._velocity
    @property
    def mass(self):
        return self._mass