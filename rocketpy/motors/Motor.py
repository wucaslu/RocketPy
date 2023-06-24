# -*- coding: utf-8 -*-

__author__ = "Giovani Hidalgo Ceotto, Oscar Mauricio Prada Ramirez, João Lemes Gribel Soares, Lucas Kierulff Balabram, Lucas Azevedo Pezente"
__copyright__ = "Copyright 20XX, RocketPy Team"
__license__ = "MIT"

import re
from abc import ABC, abstractmethod

try:
    from functools import cached_property
except ImportError:
    from rocketpy.tools import cached_property

import numpy as np

from rocketpy.Function import Function, funcify_method


class Motor(ABC):
    """Abstract class to specify characteristics and useful operations for
    motors. Cannot be instantiated.

    Attributes
    ----------

        Geometrical attributes:
        Motor.coordinateSystemOrientation : str
            Orientation of the motor's coordinate system. The coordinate system
            is defined by the motor's axis of symmetry. The origin of the
            coordinate system  may be placed anywhere along such axis, such as
            at the nozzle area, and must be kept the same for all other
            positions specified. Options are "nozzleToCombustionChamber" and
            "combustionChamberToNozzle".
        Motor.nozzleRadius : float
            Radius of motor nozzle outlet in meters.
        Motor.nozzlePosition : float
            Motor's nozzle outlet position in meters, specified in the motor's
            coordinate system. See `Motor.coordinateSystemOrientation` for
            more information.

        Mass and moment of inertia attributes:
        Motor.dry_mass : float
            The total mass of the motor structure, including chambers
            and tanks, when it is empty and does not contain any propellant.
        Motor.propellantInitialMass : float
            Total propellant initial mass in kg.
        Motor.totalMass : Function
            Total motor mass in kg as a function of time, defined as the sum
            of propellant and dry mass.
        Motor.propellantMass : Function
            Total propellant mass in kg as a function of time.
        Motor.totalMassFlowRate : Function
            Time derivative of propellant total mass in kg/s as a function
            of time as obtained by the thrust source.
        Motor.inertiaI : Function
            Propellant moment of inertia in kg*meter^2 with respect to axis
            perpendicular to axis of cylindrical symmetry of each grain,
            given as a function of time.
        Motor.inertiaIDot : Function
            Time derivative of inertiaI given in kg*meter^2/s as a function
            of time.
        Motor.inertiaZ : Function
            Propellant moment of inertia in kg*meter^2 with respect to axis of
            cylindrical symmetry of each grain, given as a function of time.
        Motor.inertiaZDot : Function
            Time derivative of inertiaZ given in kg*meter^2/s as a function
            of time.

        Thrust and burn attributes:
        Motor.thrust : Function
            Motor thrust force, in Newtons, as a function of time.
        Motor.totalImpulse : float
            Total impulse of the thrust curve in N*s.
        Motor.maxThrust : float
            Maximum thrust value of the given thrust curve, in N.
        Motor.maxThrustTime : float
            Time, in seconds, in which the maximum thrust value is achieved.
        Motor.averageThrust : float
            Average thrust of the motor, given in N.
        Motor.burnOutTime : float
            Total motor burn out time, in seconds. Must include delay time
            when the motor takes time to ignite. Also seen as time to end thrust
            curve.
        Motor.exhaustVelocity : float
            Propulsion gases exhaust velocity, assumed constant, in m/s.
        Motor.interpolate : string
            Method of interpolation used in case thrust curve is given
            by data set in .csv or .eng, or as an array. Options are 'spline'
            'akima' and 'linear'. Default is "linear".
    """

    def __init__(
        self,
        thrustSource,
        burnOut,
        dry_mass,
        center_of_dry_mass,
        dry_inertia,
        nozzleRadius,
        nozzlePosition=0,
        reshapeThrustCurve=False,
        interpolationMethod="linear",
        coordinateSystemOrientation="nozzleToCombustionChamber",
    ):
        """Initialize Motor class, process thrust curve and geometrical
        parameters and store results.

        Parameters
        ----------
        thrustSource : int, float, callable, string, array
            Motor's thrust curve. Can be given as an int or float, in which
            case the thrust will be considered constant in time. It can
            also be given as a callable function, whose argument is time in
            seconds and returns the thrust supplied by the motor in the
            instant. If a string is given, it must point to a .csv or .eng file.
            The .csv file shall contain no headers and the first column must
            specify time in seconds, while the second column specifies thrust.
            Arrays may also be specified, following rules set by the class
            Function. See help(Function). Thrust units are Newtons.
        burnOut : int, float
            Motor burn out time in seconds.
        dry_mass : int, float
            The total mass of the motor structure, including chambers
            and tanks, when it is empty and does not contain any propellant.
        center_of_dry_mass : int, float
            The position, in meters, of the motor's center of mass with respect
            to the motor's coordinate system when it is devoid of propellant.
            See `Motor.coordinateSystemOrientation`.
        dry_inertia : tuple, list
            Tuple or list containing the motor's dry mass inertia tensor
            components, in kg*m^2. This inertia is defined with respect to the
            the `center_of_dry_mass` position.
            Assuming e_3 is the rocket's axis of symmetry, e_1 and e_2 are
            orthogonal and form a plane perpendicular to e_3, the dry mass
            inertia tensor components must be given in the following order:
            (I_11, I_22, I_33, I_12, I_13, I_23), where I_ij is the
            component of the inertia tensor in the direction of e_i x e_j.
            Alternatively, the inertia tensor can be given as (I_11, I_22, I_33),
            where I_12 = I_13 = I_23 = 0.
        nozzleRadius : int, float, optional
            Motor's nozzle outlet radius in meters.
        nozzlePosition : int, float, optional
            Motor's nozzle outlet position in meters, in the motor's coordinate
            system. See `Motor.coordinateSystemOrientation` for details.
            Default is 0, in which case the origin of the coordinate system
            is placed at the motor's nozzle outlet.
        reshapeThrustCurve : boolean, tuple, optional
            If False, the original thrust curve supplied is not altered. If a
            tuple is given, whose first parameter is a new burn out time and
            whose second parameter is a new total impulse in Ns, the thrust
            curve is reshaped to match the new specifications. May be useful
            for motors whose thrust curve shape is expected to remain similar
            in case the impulse and burn time varies slightly. Default is
            False.
        interpolationMethod : string, optional
            Method of interpolation to be used in case thrust curve is given
            by data set in .csv or .eng, or as an array. Options are 'spline'
            'akima' and 'linear'. Default is "linear".
        coordinateSystemOrientation : string, optional
            Orientation of the motor's coordinate system. The coordinate system
            is defined by the motor's axis of symmetry. The origin of the
            coordinate system  may be placed anywhere along such axis, such as
            at the nozzle area, and must be kept the same for all other
            positions specified. Options are "nozzleToCombustionChamber" and
            "combustionChamberToNozzle". Default is "nozzleToCombustionChamber".

        Returns
        -------
        None
        """
        # Define coordinate system orientation
        self.coordinateSystemOrientation = coordinateSystemOrientation
        if coordinateSystemOrientation == "nozzleToCombustionChamber":
            self._csys = 1
        elif coordinateSystemOrientation == "combustionChamberToNozzle":
            self._csys = -1

        # Motor parameters
        self.dry_mass = dry_mass
        self.interpolate = interpolationMethod
        self.burnOutTime = burnOut
        self.nozzlePosition = nozzlePosition
        self.nozzleRadius = nozzleRadius
        self.center_of_dry_mass = center_of_dry_mass

        # Inertia tensor setup
        inertia = (*dry_inertia, 0, 0, 0) if len(dry_inertia) == 3 else dry_inertia
        self.dry_I_11 = inertia[0]
        self.dry_I_22 = inertia[1]
        self.dry_I_33 = inertia[2]
        self.dry_I_12 = inertia[3]
        self.dry_I_13 = inertia[4]
        self.dry_I_23 = inertia[5]

        # Check if thrustSource is csv, eng, function or other
        if isinstance(thrustSource, str):
            # Determine if csv or eng
            if thrustSource[-3:] == "eng":
                # Import content
                comments, desc, points = self.importEng(thrustSource)
                thrustSource = points
                self.burnOutTime = points[-1][0]

        # Create thrust function
        self.thrust = Function(
            thrustSource, "Time (s)", "Thrust (N)", self.interpolate, "zero"
        )
        if callable(thrustSource) or isinstance(thrustSource, (int, float)):
            self.thrust.setDiscrete(0, burnOut, 50, self.interpolate, "zero")

        # Reshape curve
        if reshapeThrustCurve:
            self.reshapeThrustCurve(*reshapeThrustCurve)

        # Compute thrust metrics
        self.maxThrust = np.amax(self.thrust.source[:, 1])
        maxThrustIndex = np.argmax(self.thrust.source[:, 1])
        self.maxThrustTime = self.thrust.source[maxThrustIndex, 0]
        self.averageThrust = self.totalImpulse / self.burnOutTime

    def reshapeThrustCurve(
        self, burnTime, totalImpulse, oldTotalImpulse=None, startAtZero=True
    ):
        """Transforms the thrust curve supplied by changing its total burn time
        and/or its total impulse, without altering the general shape of the
        curve. May translate the curve so that thrust starts at time equals 0,
        without any delays.

        Parameters
        ----------
        burnTime : float
            New desired burn out time in seconds.
        totalImpulse : float
            New desired total impulse.
        oldTotalImpulse : float, optional
            Specify the total impulse of the given thrust curve, overriding the
            value calculated by numerical integration. If left as None, the
            value calculated by numerical integration will be used in order to
            reshape the curve.
        startAtZero: bool, optional
            If True, trims the initial thrust curve points which are 0 Newtons,
            translating the thrust curve so that thrust starts at time equals 0.
            If False, no translation is applied.

        Returns
        -------
        None
        """
        # Retrieve current thrust curve data points
        timeArray = self.thrust.source[:, 0]
        thrustArray = self.thrust.source[:, 1]
        # Move start to time = 0
        if startAtZero and timeArray[0] != 0:
            timeArray = timeArray - timeArray[0]

        # Reshape time - set burn time to burnTime
        self.thrust.source[:, 0] = (burnTime / timeArray[-1]) * timeArray
        self.burnOutTime = burnTime
        self.thrust.setInterpolation(self.interpolate)

        # Reshape thrust - set total impulse
        if oldTotalImpulse is None:
            oldTotalImpulse = self.totalImpulse
        self.thrust.source[:, 1] = (totalImpulse / oldTotalImpulse) * thrustArray
        self.thrust.setInterpolation(self.interpolate)

        # Store total impulse
        self.totalImpulse = totalImpulse

        # Return reshaped curve
        return self.thrust

    @cached_property
    def totalImpulse(self):
        """Calculates and returns total impulse by numerical integration
        of the thrust curve in SI units.

        Parameters
        ----------
        None

        Returns
        -------
        self.totalImpulse : float
            Motor total impulse in Ns.
        """
        return self.thrust.integral(0, self.burnOutTime)

    @property
    @abstractmethod
    def exhaustVelocity(self):
        """Exhaust velocity by assuming it as a constant. The formula used is
        total impulse/propellant initial mass.

        Parameters
        ----------
        None

        Returns
        -------
        self.exhaustVelocity : float
            Constant gas exhaust velocity of the motor.
        """
        pass

    @funcify_method("Time (s)", "total mass (kg)")
    def totalMass(self):
        """Total mass of the motor as a Function of time.
        Is defined as the propellant mass plus the dry mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        Function
            Total mass as a function of time.
        """
        return self.propellantMass + self.dry_mass

    @funcify_method("Time (s)", "propellant mass (kg)")
    def propellantMass(self):
        """Total propellant mass as a Function of time.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        Function
            Total propellant mass as a function of time.
        """
        return self.totalMassFlowRate.integralFunction() + self.propellantInitialMass

    @funcify_method("Time (s)", "mass dot (kg/s)", extrapolation="zero")
    def totalMassFlowRate(self):
        """Time derivative of propellant mass. Assumes constant exhaust
        velocity. The formula used is the opposite of thrust divided by
        exhaust velocity.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        Function
            Time derivative of total propellant mass a function of time.

        See Also
        --------
        `SolidMotor.massFlowRate` :
            Numerically equivalent to `totalMassFlowRate`.
        `HybridMotor.massFlowRate` :
            Numerically equivalent to `totalMassFlowRate`.
        `LiquidMotor.massFlowRate` :
            Independent of `totalMassFlowRate` favoring more accurate
            sum of Tanks' mass flow rates.

        Notes
        -----
        This function computes the total mass flow rate of the motor by
        dividing the thrust data by a constant approximation of the exhaust
        velocity.
        This approximation of the total mass flow rate is used in the
        following manner by the child Motor classes:
            - The `SolidMotor` class uses this approximation to compute the
            grain's mass flow rate;
            - The `HybridMotor` class uses this approximation as a reference
            to the sum of the oxidizer and fuel (grains) mass flow rates;
            - The `LiquidMotor` class favors the more accurate data from the
            Tanks's mass flow rates. Therefore this value is numerically
            independent of the `LiquidMotor.massFlowRate`.
        It should be noted that, for hybrid motors, the oxidizer mass flow
        rate should not be greater than `totalMassFlowRate`, otherwise the
        grains mass flow rate will be negative, losing physical meaning.
        """
        return -1 * self.thrust / self.exhaustVelocity

    @property
    @abstractmethod
    def propellantInitialMass(self):
        """Propellant initial mass in kg.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Propellant initial mass in kg.
        """
        pass

    @funcify_method("Time (s)", "Motor center of mass (m)")
    def centerOfMass(self):
        """Position of the center of mass as a function of time. The position
        is specified as a scalar, relative to the motor's coordinate system.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        Function
            Position of the center of mass as a function of time.
        """
        mass_balance = (
            self.centerOfPropellantMass * self.propellantMass
            + self.dry_mass * self.center_of_dry_mass
        )
        return mass_balance / self.totalMass

    @property
    @abstractmethod
    def centerOfPropellantMass(self):
        """Position of the propellant center of mass as a function of time.
        The position is specified as a scalar, relative to the motor's
        coordinate system.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        Function
            Position of the propellant center of mass as a function of time.
        """
        pass

    @funcify_method("Time (s)", "Inertia I_11 (kg m²)")
    def I_11(self):
        """Inertia tensor 11 component, which corresponds to the inertia
        relative to the e_1 axis, centered at the instantaneous center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 11 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        # Propellant inertia tensor 11 component wrt propellant center of mass
        propellant_I_11 = self.propellant_I_11

        # Dry inertia tensor 11 component wrt dry center of mass
        dry_I_11 = self.dry_I_11

        # Steiner theorem the get inertia wrt motor center of mass
        propellant_I_11 += (
            self.propellantMass * (self.centerOfPropellantMass - self.centerOfMass) ** 2
        )

        dry_I_11 += self.dry_mass * (self.center_of_dry_mass - self.centerOfMass) ** 2

        # Sum of inertia components
        return propellant_I_11 + dry_I_11

    @funcify_method("Time (s)", "Inertia I_22 (kg m²)")
    def I_22(self):
        """Inertia tensor 22 component, which corresponds to the inertia
        relative to the e_2 axis, centered at the instantaneous center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 22 component at time t.

        Notes
        -----
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        # Due to symmetry, I_22 = I_11
        return self.I_11

    @funcify_method("Time (s)", "Inertia I_33 (kg m²)")
    def I_33(self):
        """Inertia tensor 33 component, which corresponds to the inertia
        relative to the e_3 axis, centered at the instantaneous center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 33 component at time t.

        Notes
        -----
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        # Propellant inertia tensor 33 component wrt propellant center of mass
        propellant_I_33 = self.propellant_I_33

        # Dry inertia tensor 33 component wrt dry center of mass
        dry_I_33 = self.dry_I_33

        # Both inertia components wrt the same axis, Steiner not needed
        return propellant_I_33 + dry_I_33

    @funcify_method("Time (s)", "Inertia I_12 (kg m²)")
    def I_12(self):
        """Inertia tensor 12 component, which corresponds to the product of
        inertia relative to axes e_1 and e_2, centered at the instantaneous
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 12 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        # Propellant inertia tensor 12 component wrt propellant center of mass
        propellant_I_12 = self.propellant_I_12

        # Dry inertia tensor 12 component wrt dry center of mass
        dry_I_12 = self.dry_I_12

        # Steiner correction not needed since the centers only move in the e_3 axis
        return propellant_I_12 + dry_I_12

    @funcify_method("Time (s)", "Inertia I_13 (kg m²)")
    def I_13(self):
        """Inertia tensor 13 component, which corresponds to the product of
        inertia relative to the axes e_1 and e_3, centered at the instantaneous
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 13 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia
        """
        # Propellant inertia tensor 13 component wrt propellant center of mass
        propellant_I_13 = self.propellant_I_13

        # Dry inertia tensor 13 component wrt dry center of mass
        dry_I_13 = self.dry_I_13

        # Steiner correction not needed since the centers only move in the e_3 axis
        return propellant_I_13 + dry_I_13

    @funcify_method("Time (s)", "Inertia I_23 (kg m²)")
    def I_23(self):
        """Inertia tensor 23 component, which corresponds to the product of
        inertia relative the axes e_2 and e_3, centered at the instantaneous
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 23 component at time t.

        Notes
        -----
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia
        """
        # Propellant inertia tensor 23 component wrt propellant center of mass
        propellant_I_23 = self.propellant_I_23

        # Dry inertia tensor 23 component wrt dry center of mass
        dry_I_23 = self.dry_I_23

        # Steiner correction not needed since the centers only move in the e_3 axis
        return propellant_I_23 + dry_I_23

    @property
    @abstractmethod
    def propellant_I_11(self):
        """Inertia tensor 11 component of the propellnat, the inertia is
        relative to the e_1 axis, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 11 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        pass

    @property
    @abstractmethod
    def propellant_I_22(self):
        """Inertia tensor 22 component of the propellnat, the inertia is
        relative to the e_2 axis, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 22 component at time t.

        Notes
        -----
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        pass

    @property
    @abstractmethod
    def propellant_I_33(self):
        """Inertia tensor 33 component of the propellnat, the inertia is
        relative to the e_3 axis, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 33 component at time t.

        Notes
        -----
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        pass

    @property
    @abstractmethod
    def propellant_I_12(self):
        """Inertia tensor 12 component of the propellnat, the product of inertia
        is relative to axes e_1 and e_2, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 12 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        pass

    @property
    @abstractmethod
    def propellant_I_13(self):
        """Inertia tensor 13 component of the propellnat, the product of inertia
        is relative to axes e_1 and e_3, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 13 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia
        """
        pass

    @property
    @abstractmethod
    def propellant_I_23(self):
        """Inertia tensor 23 component of the propellnat, the product of inertia
        is relative to axes e_2 and e_3, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 23 component at time t.

        Notes
        -----
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.
        RocketPy follows the definition of the inertia tensor as in [1], which
        includes the minus sign for all products of inertia.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia
        """
        pass

    def importEng(self, fileName):
        """Read content from .eng file and process it, in order to return the
        comments, description and data points.

        Parameters
        ----------
        fileName : string
            Name of the .eng file. E.g. 'test.eng'.
            Note that the .eng file must not contain the 0 0 point.

        Returns
        -------
        comments : list
            All comments in the .eng file, separated by line in a list. Each
            line is an entry of the list.
        description: list
            Description of the motor. All attributes are returned separated in
            a list. E.g. "F32 24 124 5-10-15 .0377 .0695 RV\n" is return as
            ['F32', '24', '124', '5-10-15', '.0377', '.0695', 'RV\n']
        dataPoints: list
            List of all data points in file. Each data point is an entry in
            the returned list and written as a list of two entries.
        """

        # Initialize arrays
        comments = []
        description = []
        dataPoints = [[0, 0]]

        # Open and read .eng file
        with open(fileName) as file:
            for line in file:
                if re.search(r";.*", line):
                    # Extract comment
                    comments.append(re.findall(r";.*", line)[0])
                    line = re.sub(r";.*", "", line)
                if line.strip():
                    if description == []:
                        # Extract description
                        description = line.strip().split(" ")
                    else:
                        # Extract thrust curve data points
                        time, thrust = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                        dataPoints.append([float(time), float(thrust)])

        # Return all extract content
        return comments, description, dataPoints

    def exportEng(self, fileName, motorName):
        """Exports thrust curve data points and motor description to.

        .eng file format. A description of the format can be found
        here: http://www.thrustcurve.org/raspformat.shtml

        Parameters
        ----------
        fileName : string
            Name of the .eng file to be exported. E.g. 'test.eng'
        motorName : string
            Name given to motor. Will appear in the description of the
            .eng file. E.g. 'Mandioca'

        Returns
        -------
        None
        """
        # Open file
        file = open(fileName, "w")

        # Write first line
        file.write(
            motorName
            + " {:3.1f} {:3.1f} 0 {:2.3} {:2.3} RocketPy\n".format(
                2000 * self.grainOuterRadius,
                1000
                * self.grainNumber
                * (self.grainInitialHeight + self.grainSeparation),
                self.propellantInitialMass,
                self.propellantInitialMass,
            )
        )

        # Write thrust curve data points
        for time, thrust in self.thrust.source[1:-1, :]:
            # time, thrust = item
            file.write("{:.4f} {:.3f}\n".format(time, thrust))

        # Write last line
        file.write("{:.4f} {:.3f}\n".format(self.thrust.source[-1, 0], 0))

        # Close file
        file.close()

        return None

    def info(self):
        """Prints out a summary of the data and graphs available about the
        Motor.

        Parameters
        ----------
        None

        Return
        ------
        None
        """
        # Print motor details
        print("\nMotor Details")
        print("Total Burning Time: " + str(self.burnOutTime) + " s")
        print(
            "Total Propellant Mass: "
            + "{:.3f}".format(self.propellantInitialMass)
            + " kg"
        )
        print(
            "Propellant Exhaust Velocity: "
            + "{:.3f}".format(self.exhaustVelocity.average(0, self.burnOutTime))
            + " m/s"
        )
        print("Average Thrust: " + "{:.3f}".format(self.averageThrust) + " N")
        print(
            "Maximum Thrust: "
            + str(self.maxThrust)
            + " N at "
            + str(self.maxThrustTime)
            + " s after ignition."
        )
        print("Total Impulse: " + "{:.3f}".format(self.totalImpulse) + " Ns")

        # Show plots
        print("\nPlots")
        self.thrust()

        return None

    @abstractmethod
    def allInfo(self):
        """Prints out all data and graphs available about the Motor.

        Parameters
        ----------
        None

        Return
        ------
        None
        """
        pass


class GenericMotor(Motor):
    def __init__(
        self,
        thrustSource,
        burnOut,
        chamberRadius,
        chamberHeight,
        chamberPosition,
        propellantInitialMass,
        nozzleRadius,
        dry_mass=0,
        center_of_dry_mass=None,
        dry_inertia=(0, 0, 0),
        nozzlePosition=0,
        reshapeThrustCurve=False,
        interpolationMethod="linear",
        coordinateSystemOrientation="nozzleToCombustionChamber",
    ):
        super().__init__(
            thrustSource,
            burnOut,
            dry_mass,
            center_of_dry_mass,
            dry_inertia,
            nozzleRadius,
            nozzlePosition,
            reshapeThrustCurve,
            interpolationMethod,
            coordinateSystemOrientation,
        )

        self.chamberRadius = chamberRadius
        self.chamberHeight = chamberHeight
        self.chamberPosition = chamberPosition
        self.propellantInitialMass = propellantInitialMass

        # Set center of mass and estimate to chamber position if not given
        self.center_of_dry_mass = (
            center_of_dry_mass if center_of_dry_mass is not None else chamberPosition
        )

    @cached_property
    def propellantInitialMass(self):
        """Calculates the initial mass of the propellant.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Initial mass of the propellant.
        """
        return self.propellantInitialMass

    @funcify_method("Time (s)", "center of mass (m)")
    def centerOfMass(self):
        """Estimates the Center of Mass of the motor as fixed in the chamber
        position. For a more accurate evaluation, use the classes SolidMotor,
        LiquidMotor or HybridMotor.

        Parameters
        ----------
        Time : float

        Returns
        -------
        Function
            Function representing the center of mass of the motor.
        """
        return self.center_of_dry_mass

    @funcify_method("Time (s)", "Inertia I_11 (kg m²)")
    def propellant_I_11(self):
        """Inertia tensor 11 component of the propellnat, the inertia is
        relative to the e_1 axis, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 11 component at time t.

        Notes
        -----
        The e_1 direction is assumed to be the direction perpendicular to the
        motor body axis.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        return self.mass * (3 * self.chamberRadius**2 + self.chamberHeight**2) / 12

    @funcify_method("Time (s)", "Inertia I_22 (kg m²)")
    def propellant_I_22(self):
        """Inertia tensor 22 component of the propellnat, the inertia is
        relative to the e_2 axis, centered at the instantaneous propellant
        center of mass.

        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 22 component at time t.

        Notes
        -----
        The e_2 direction is assumed to be the direction perpendicular to the
        motor body axis, and perpendicular to e_1.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        return self.I_11

    @funcify_method("Time (s)", "Inertia I_33 (kg m²)")
    def propellant_I_33(self):
        """Inertia tensor 33 component of the propellnat, the inertia is
        relative to the e_3 axis, centered at the instantaneous propellant
        center of mass.


        Parameters
        ----------
        t : float
            Time in seconds.

        Returns
        -------
        float
            Propellant inertia tensor 33 component at time t.

        Notes
        -----
        The e_3 direction is assumed to be the axial direction of the rocket
        motor.

        References
        ----------
        .. [1] https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        """
        return self.mass * self.chamberRadius**2 / 2

    @funcify_method("Time (s)", "Inertia I_12 (kg m²)")
    def propellant_I_12(self):
        return 0

    @funcify_method("Time (s)", "Inertia I_13 (kg m²)")
    def propellant_I_13(self):
        return 0

    @funcify_method("Time (s)", "Inertia I_23 (kg m²)")
    def propellant_I_23(self):
        return 0

    def allInfo(self):
        """Prints out all data and graphs available about the Motor.

        Parameters
        ----------
        None

        Return
        ------
        None
        """
        # Print motor details
        print("\nMotor Details")
        print("Total Burning Time: " + str(self.burnOutTime) + " s")
        print(
            "Total Propellant Mass: "
            + "{:.3f}".format(self.propellantInitialMass)
            + " kg"
        )
        print(
            "Propellant Exhaust Velocity: "
            + "{:.3f}".format(self.exhaustVelocity)
            + " m/s"
        )
        print("Average Thrust: " + "{:.3f}".format(self.averageThrust) + " N")
        print(
            "Maximum Thrust: "
            + str(self.maxThrust)
            + " N at "
            + str(self.maxThrustTime)
            + " s after ignition."
        )
        print("Total Impulse: " + "{:.3f}".format(self.totalImpulse) + " Ns")

        # Show plots
        print("\nPlots")
        self.thrust()
        self.mass()
        self.centerOfMass()
        self.I_11()
        self.I_22()
        self.I_33()
        self.I_12()
        self.I_13()
        self.I_23()


class EmptyMotor:
    """Class that represents an empty motor with no mass and no thrust."""

    # TODO: This is a temporary solution. It should be replaced by a class that
    # inherits from the abstract Motor class. Currently cannot be done easily.
    def __init__(self):
        """Initializes an empty motor with no mass and no thrust."""
        self._csys = 1
        self.dry_mass = 0
        self.nozzleRadius = 0
        self.thrust = Function(0, "Time (s)", "Thrust (N)")
        self.propellantMass = Function(0, "Time (s)", "Mass (kg)")
        self.totalMass = Function(0, "Time (s)", "Mass (kg)")
        self.totalMassFlowRate = Function(0, "Time (s)", "Mass Depletion Rate (kg/s)")
        self.burnOutTime = 1
        self.nozzlePosition = 0
        self.nozzleRadius = 0
        self.center_of_dry_mass = 0
        self.centerOfPropellantMass = Function(0, "Time (s)", "Mass (kg)")
        self.centerOfMass = Function(0, "Time (s)", "Mass (kg)")
        self.dry_I_11 = 0
        self.dry_I_22 = 0
        self.dry_I_33 = 0
        self.dry_I_12 = 0
        self.dry_I_13 = 0
        self.dry_I_23 = 0
