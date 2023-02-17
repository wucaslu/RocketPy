# -*- coding: utf-8 -*-

__author__ = "Giovani Hidalgo Ceotto, Oscar Mauricio Prada Ramirez, João Lemes Gribel Soares, Mateus Stano, Pedro Henrique Marinho Bressan, Patrick Bales, Lakshman Peri, Gautam Yarramreddy, Curtis Hu, and William Bradford"
__copyright__ = "Copyright 20XX, RocketPy Team"
__license__ = "MIT"

from abc import ABC, abstractmethod
from cmath import tan
from turtle import position

import numpy as np
from scipy import integrate

from rocketpy.Function import Function, PiecewiseFunction
from rocketpy.motors import Motor, Fluid

# @Stano
# @PBales1
# @lperi03
# @gautamsaiy
class LiquidMotor(Motor):
    def __init__(
        self,
        thrustSource,
        burnOut,
        reshapeThrustCurve=False,
        interpolationMethod="linear",
    ):

        super.__init__()
        self.tanks = []
        self.thrustSource = Function(thrustSource)
        self.burnOut = burnOut

    def evaluateMassFlowRate(self):
        total_mfr = 0
        for tank in self.tanks:
            total_mfr += tank.netMassFlowRate()
        return total_mfr

    def evaluateCenterOfMass(self):
        com = Function(0)
        total_mass = Function(0)
        for tank in self.tanks:
            com += (tank["tank"].centerOfMass() + tank[position]) * tank["tank"].mass()
            total_mass += tank["tank"].mass() + Function(lambda t: tank["tank"].netMassFlowRate.integral(t - .05, t + .05))
        com = com / total_mass
        com.setInputs("Time")
        com.setOutputs("Center of mass")
        return com


    def evaluateInertiaTensor(self):
        inertiaI = Function(0)
        inertiaZ = Function(0)
        for tank in self.tanks:
            inertia = tank["tank"].inertiaTensor()
            inertiaI += inertia[0]
            inertiaZ += inertia[1]
        inertiaI.setInputs("Time")
        inertiaI.setOutputs("Inertia tensor")
        inertiaZ.setInputs("Time")
        inertiaZ.setOutputs("Inertia tensor")
        return [inertiaI, inertiaZ]

    def addTank(self, tank, position):
        self.tanks.append({"tank": tank, "position": position})


# @gautamsaiy
class Tank(ABC):
    def __init__(self, name, tank_geometry, gas, liquid=0):
        assert isinstance(name, str)
        assert(isinstance(tank_geometry, dict))
        assert(isinstance(gas, Fluid))
        assert(isinstance(liquid, Fluid) or liquid == 0)

        self.height = sorted(tank_geometry.keys())[-1][1]
        self.tank_geometry = PiecewiseFunction(tank_geometry)
            
        self.tank_geometry.setInputs("y")
        self.tank_geometry.setOutputs("radius")

        self.tank_area = self.tank_geometry ** 2 * np.pi
        self.tank_area.setInputs("y")
        self.tank_area.setOutputs("area")

        self.tank_vol = Function(lambda x: self.tank_area.integral(0, x))
        self.tank_vol.setInputs("y")
        self.tank_vol.setOutputs("volume")

        self.gas = gas
        self.liquid = liquid


    @abstractmethod
    def mass(self):
        """Returns the total mass of liquid and gases inside the tank as a
        function of time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Mass of the tank as a function of time. Units in kg.
        """
        pass

    @abstractmethod
    def netMassFlowRate(self):
        """Returns the net mass flow rate of the tank as a function of time.
        Net mass flow rate is the mass flow rate exiting the tank minus the
        mass flow rate entering the tank, including liquids and gases.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Net mass flow rate of the tank as a function of time.
        """
        pass

    @abstractmethod
    def liquidHeight(self):
        """
        Returns the height of the uilage as a function of time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Height of the uilage as a function of time.
        """
        pass
    
    @abstractmethod
    def liquidMass():
        """
        Returns the mass of the liquid as a function of time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Mass of the liquid as a function of time.
        """
        pass

    @abstractmethod
    def gasMass():
        """
        Returns the mass of the gas as a function of time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Mass of the gas as a function of time.
        """
        pass

    @property
    def centerOfMass(self):
        """Returns the center of mass of the tank's fluids as a function of
        time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Center of mass of the tank's fluids as a function of time.
        """
        tank_liquid_A = Function(lambda x: self.tank_geometry.integral(0, x))
        tank_liquid_inside_integral = Function(lambda x: x * self.tank_geometry(x))
        tank_liquid_integral = Function(lambda x: tank_liquid_inside_integral.integral(0, x))
        tank_liquid_com = Function(lambda x: tank_liquid_integral(x) / (tank_liquid_A(x) + 1e-9))

        tank_gas_A = Function(lambda x: self.tank_geometry.integral(x, self.height))
        tank_gas_inside_integral = Function(lambda x: x * self.tank_geometry(x))
        tank_gas_integral = Function(lambda x: tank_gas_inside_integral.integral(x, self.height))
        tank_gas_com = Function(lambda x: tank_gas_integral(x) / (tank_gas_A(x) + 1e-9))


        tank_liquid_com_t = Function(lambda t: tank_liquid_com(self.liquidHeight()(t)))
        tank_gas_com_t = Function(lambda t: tank_gas_com(self.liquidHeight()(t)))

        lm = self.liquidMass()
        gm = self.gasMass()

        liquid_mass_c = Function(lambda t: tank_liquid_com_t(t) * lm(t))
        gas_mass_c = Function(lambda t: tank_gas_com_t(t) * gm(t))

        com = Function(lambda t: (liquid_mass_c(t) + gas_mass_c(t)) / (self.mass()(t) + 1e-9), "Time", "Center of Mass")
        return com

    @property
    def inertiaTensor(self):
        """
        Returns the inertia tensor of the tank's fluids as a function of
        time.

        Parameters
        ----------
        time : float
            Time in seconds.

        Returns
        -------
        Function
            Inertia tensor of the tank's fluids as a function of time.
        """
        def Ii(start, stop, density):
            r = self.tank_geometry
            rho = density
            z = Function(lambda x: x)
            Izz = Iz(start, stop, density)
            return Izz / 2 + rho * np.pi * (z ** 2 * r ** 2).integral(start, stop) / 4
    
        def Iz(start, stop, density):
            r = self.tank_geometry
            rho = density
            return (rho * np.pi / 2) * (r ** 4).integral(start, stop)

        lh = self.liquidHeight()
        return Function(lambda time: (Ii(0, lh(time), self.liquid.density), Iz(0, lh(time), self.liquid.density)))
        

# @MrGribel
# @gautamsaiy
class MassFlowRateBasedTank(Tank):
    def __init__(
        self,
        name,
        tank_geometry,
        initial_liquid_mass,
        initial_gas_mass,
        liquid_mass_flow_rate_in,
        gas_mass_flow_rate_in,
        liquid_mass_flow_rate_out,
        gas_mass_flow_rate_out,
        liquid,
        gas,
    ):
        super().__init__(name, tank_geometry, gas, liquid)
        self.initial_liquid_mass = initial_liquid_mass
        self.initial_gas_mass = initial_gas_mass

        self.liquid_mass_flow_rate_in = Function(
            liquid_mass_flow_rate_in, inputs="Time", outputs="Mass Flow Rate"
        )
        self.gas_mass_flow_rate_in = Function(
            gas_mass_flow_rate_in, inputs="Time", outputs="Mass Flow Rate"
        )
        self.liquid_mass_flow_rate_out = Function(
            liquid_mass_flow_rate_out, inputs="Time", outputs="Mass Flow Rate"
        )
        self.gas_mass_flow_rate_out = Function(
            gas_mass_flow_rate_out, inputs="Time", outputs="Mass Flow Rate"
        )

    def mass(self):
        return self.liquidMass() + self.gasMass()

    def liquidMass(self):
        liquid_flow = self.netLiquidFlowRate().integralFunction()
        return self.initial_liquid_mass + liquid_flow

    def gasMass(self):
        gas_flow = self.netGasFlowRate().integralFunction()
        return self.initial_gas_mass + gas_flow

    def netLiquidFlowRate(self):
        return self.liquid_mass_flow_rate_in - self.liquid_mass_flow_rate_out

    def netGasFlowRate(self):
        return self.gas_mass_flow_rate_in - self.gas_mass_flow_rate_out

    def netMassFlowRate(self):
        return self.netLiquidFlowRate() + self.netGasFlowRate()

    def liquidVolume(self):
        return self.liquidMass() / self.liquid.density

    def gasVolume(self):
        return self.gasMass() / self.gas.density

    def liquidHeight(self):
        liquid_volume = self.liquidVolume()
        inverse_volume = self.structure.inverse_volume.setDiscrete()
        return inverse_volume.compose(liquid_volume)

    def gasHeight(self):
        fluid_volume = self.gasVolume() + self.liquidVolume()
        inverse_volume = self.structure.inverse_volume.setDiscrete()
        return inverse_volume.compose(fluid_volume)
 


# @phmbressan
# @lperi03
# @curtisjhu
class UllageBasedTank(Tank):
    def __init__(
        self,
        name,
        tank_geometry,
        liquid,
        gas,
        ullage,
    ):
        super().__init__(name, tank_geometry, gas, liquid)
        self.ullage = Function(ullage, inputs="Time", outputs="Volume")

    def mass(self):
        return self.liquidMass() + self.gasMass()

    def netMassFlowRate(self):
        return self.mass().derivativeFunction()

    def liquidVolume(self):
        return self.structure.total_volume.item - self.ullage

    def gasVolume(self):
        return self.ullage

    def gasMass(self):
        return self.gasVolume() * self.gas.density

    def liquidMass(self):
        return self.liquidVolume() * self.liquid.density

    def liquidHeight(self):
        liquid_volume = self.liquidVolume()
        inverse_volume = self.structure.volume.inverseFunction()
        return inverse_volume.compose(liquid_volume)

    def gasHeight(self):
        fluid_volume = self.gasVolume() + self.liquidVolume()
        inverse_volume = self.structure.volume.inverseFunction()
        return inverse_volume.compose(fluid_volume)



    def liquidMass(self):
        liquid_mass = self.tank_vol.compose(self.ullageHeight) * self.liquid.density
        liquid_mass.setInputs("Time")
        liquid_mass.setOutputs("Liquid Mass")
        return liquid_mass

    def gasMass(self):
        gas_mass = self.tank_vol.compose(self.ullageHeight) * self.gas.density
        gas_mass.setInputs("Time")
        gas_mass.setOutputs("Gas Mass")
        return gas_mass




# @ompro07
# @PBales1
class MassBasedTank(Tank):
    def __init__(
        self,
        name,
        tank_geometry,
        liquid_mass,
        gas_mass,
        liquid,
        gas,
    ):
        super().__init__(name, tank_geometry, gas, liquid)
        self.liquid_mass = Function(liquid_mass, inputs="Time", outputs="Mass")
        self.gas_mass = Function(gas_mass, inputs="Time", outputs="Mass")

    def liquidMass(self):
        return self.liquid_mass

    def gasMass(self):
        return self.gas_mass

    def mass(self):
        m = self.liquidMass() + self.gasMass()
        m.setInputs("Time")
        m.setOutputs("Mass")
        return m
    
    def netMassFlowRate(self):
        m = self.mass()
        mfr = m.derivativeFunction()
        mfr.setInputs("Time")
        mfr.setOutputs("Mass Flow Rate")
        return mfr

    def liquidHeight(self):
        liquid_volume = self.liquid_mass / self.liquid.density
        tank_vol = self.tank_vol.inverseFunction()
        ullage_height = Function(lambda t: tank_vol.getValue(liquid_volume.getValue(t)))
        ullage_height.setInputs("Time")
        ullage_height.setOutputs("Ullage Height")
        return ullage_height

        

