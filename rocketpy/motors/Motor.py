# -*- coding: utf-8 -*-

__author__ = "Giovani Hidalgo Ceotto, Oscar Mauricio Prada Ramirez, João Lemes Gribel Soares, Lucas Kierulff Balabram, Lucas Azevedo Pezente"
__copyright__ = "Copyright 20XX, RocketPy Team"
__license__ = "MIT"

import re
from abc import ABC, abstractmethod, abstractproperty

import numpy as np
from scipy import integrate

from .Function import Function


class Motor(ABC):
    """Abstract class to specify characteristics and useful operations for
    motors. Cannot be instantiated.

    Attributes
    ----------

        Geometrical attributes:
        Motor.coordinateSystemOrientation : str
            Orientation of the motor's coordinate system. The coordinate system
            is defined by the motor's axis of symmetry. The origin of the
            coordinate system  may be placed anywhere along such axis, such as at the
            nozzle area, and must be kept the same for all other positions specified.
            Options are "nozzleToCombustionChamber" and "combustionChamberToNozzle".
        Motor.nozzleRadius : float
            Radius of motor nozzle outlet in meters.
        Motor.nozzlePosition : float
            Motor's nozzle outlet position in meters. More specifically, the coordinate
            of the nozzle outlet specified in the motor's coordinate system.
            See `Motor.coordinateSystemOrientation` for more information.
        Motor.throatRadius : float
            Radius of motor nozzle throat in meters.
        Motor.grainNumber : int
            Number of solid grains.
        Motor.grainSeparation : float
            Distance between two grains in meters.
        Motor.grainDensity : float
            Density of each grain in kg/meters cubed.
        Motor.grainOuterRadius : float
            Outer radius of each grain in meters.
        Motor.grainInitialInnerRadius : float
            Initial inner radius of each grain in meters.
        Motor.grainInitialHeight : float
            Initial height of each grain in meters.
        Motor.grainInitialVolume : float
            Initial volume of each grain in meters cubed.
        Motor.grainInnerRadius : Function
            Inner radius of each grain in meters as a function of time.
        Motor.grainHeight : Function
            Height of each grain in meters as a function of time.

        Mass and moment of inertia attributes:
        Motor.grainInitialMass : float
            Initial mass of each grain in kg.
        Motor.propellantInitialMass : float
            Total propellant initial mass in kg.
        Motor.mass : Function
            Propellant total mass in kg as a function of time.
        Motor.massDot : Function
            Time derivative of propellant total mass in kg/s as a function
            of time.
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
        Motor.burnArea : Function
            Total burn area considering all grains, made out of inner
            cylindrical burn area and grain top and bottom faces. Expressed
            in meters squared as a function of time.
        Motor.Kn : Function
            Motor Kn as a function of time. Defined as burnArea divided by
            nozzle throat cross sectional area. Has no units.
        Motor.burnRate : Function
            Propellant burn rate in meter/second as a function of time.
        Motor.interpolate : string
            Method of interpolation used in case thrust curve is given
            by data set in .csv or .eng, or as an array. Options are 'spline'
            'akima' and 'linear'. Default is "linear".
    """

    def __init__(
        self,
        thrustSource,
        burnOut,
        nozzleRadius=0.0335,
        nozzlePosition=0,
        throatRadius=0.0114,
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
        nozzleRadius : int, float, optional
            Motor's nozzle outlet radius in meters. Used to calculate Kn curve.
            Optional if the Kn curve is not interesting. Its value does not impact
            trajectory simulation.
        nozzlePosition : int, float, optional
            Motor's nozzle outlet position in meters. More specifically, the coordinate
            of the nozzle outlet specified in the motor's coordinate system.
            See `Motor.coordinateSystemOrientation` for more information.
            Default is 0, in which case the origin of the motor's coordinate system
            is placed at the motor's nozzle outlet.
        throatRadius : int, float, optional
            Motor's nozzle throat radius in meters. Its value has very low
            impact in trajectory simulation, only useful to analyze
            dynamic instabilities, therefore it is optional.
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
            coordinate system  may be placed anywhere along such axis, such as at the
            nozzle area, and must be kept the same for all other positions specified.
            Options are "nozzleToCombustionChamber" and "combustionChamberToNozzle".
            Default is "nozzleToCombustionChamber".

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

        # Thrust parameters
        self.interpolate = interpolationMethod
        self.burnOutTime = burnOut

        # Check if thrustSource is csv, eng, function or other
        if isinstance(thrustSource, str):
            # Determine if csv or eng
            if thrustSource[-3:] == "eng":
                # Import content
                comments, desc, points = self.importEng(thrustSource)
                # Process description and points
                # diameter = float(desc[1])/1000
                # height = float(desc[2])/1000
                # mass = float(desc[4])
                # nozzleRadius = diameter/4
                # throatRadius = diameter/8
                # grainNumber = grainNumber
                # grainVolume = height*np.pi*((diameter/2)**2 -(diameter/4)**2)
                # grainDensity = mass/grainVolume
                # grainOuterRadius = diameter/2
                # grainInitialInnerRadius = diameter/4
                # grainInitialHeight = height
                thrustSource = points
                self.burnOutTime = points[-1][0]

        # Create thrust function
        self.thrust = Function(
            thrustSource, "Time (s)", "Thrust (N)", self.interpolate, "zero"
        )
        if callable(thrustSource) or isinstance(thrustSource, (int, float)):
            self.thrust.setDiscrete(0, burnOut, 50, self.interpolate, "zero")

        # Reshape curve and calculate impulse
        if reshapeThrustCurve:
            self.reshapeThrustCurve(*reshapeThrustCurve)
        else:
            self.evaluateTotalImpulse()

        # Define motor attributes
        # Grain and nozzle parameters
        self.nozzleRadius = nozzleRadius
        self.nozzlePosition = nozzlePosition
        self.throatRadius = throatRadius

        # Other quantities that will be computed
        self.massDot = None
        self.mass = None
        self.inertiaI = None
        self.inertiaIDot = None
        self.inertiaZ = None
        self.inertiaZDot = None
        self.maxThrust = None
        self.maxThrustTime = None
        self.averageThrust = None

        # Compute quantities
        # Thrust information - maximum and average
        self.maxThrust = np.amax(self.thrust.source[:, 1])
        maxThrustIndex = np.argmax(self.thrust.source[:, 1])
        self.maxThrustTime = self.thrust.source[maxThrustIndex, 0]
        self.averageThrust = self.totalImpulse / self.burnOutTime

        self.propellantInitialMass = None

    def reshapeThrustCurve(
        self, burnTime, totalImpulse, oldTotalImpulse=None, startAtZero=True
    ):
        """Transforms the thrust curve supplied by changing its total
        burn time and/or its total impulse, without altering the
        general shape of the curve. May translate the curve so that
        thrust starts at time equals 0, without any delays.

        Parameters
        ----------
        burnTime : float
            New desired burn out time in seconds.
        totalImpulse : float
            New desired total impulse.
        oldTotalImpulse : float, optional
            Specify the total impulse of the given thrust curve,
            overriding the value calculated by numerical integration.
            If left as None, the value calculated by numerical
            integration will be used in order to reshape the curve.
        startAtZero: bool, optional
            If True, trims the initial thrust curve points which
            are 0 Newtons, translating the thrust curve so that
            thrust starts at time equals 0. If False, no translation
            is applied.

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
            oldTotalImpulse = self.evaluateTotalImpulse()
        self.thrust.source[:, 1] = (totalImpulse / oldTotalImpulse) * thrustArray
        self.thrust.setInterpolation(self.interpolate)

        # Store total impulse
        self.totalImpulse = totalImpulse

        # Return reshaped curve
        return self.thrust

    def evaluateTotalImpulse(self):
        """Calculates and returns total impulse by numerical
        integration of the thrust curve in SI units. The value is
        also stored in self.totalImpulse.

        Parameters
        ----------
        None

        Returns
        -------
        self.totalImpulse : float
            Motor total impulse in Ns.
        """
        # Calculate total impulse
        self.totalImpulse = self.thrust.integral(0, self.burnOutTime)

        # Return total impulse
        return self.totalImpulse

    @abstractproperty
    def exhaustVelocity(self):
        """Calculates and returns exhaust velocity by assuming it
        as a constant. The formula used is total impulse/propellant
        initial mass. The value is also stored in
        self.exhaustVelocity.

        Parameters
        ----------
        None

        Returns
        -------
        self.exhaustVelocity : float
            Constant gas exhaust velocity of the motor.
        """
        pass

    @abstractmethod
    def evaluateMassDot(self):
        """Calculates and returns the time derivative of propellant
        mass by assuming constant exhaust velocity. The formula used
        is the opposite of thrust divided by exhaust velocity. The
        result is a function of time, object of the Function class,
        which is stored in self.massDot.

        Parameters
        ----------
        None

        Returns
        -------
        self.massDot : Function
            Time derivative of total propellant mas as a function
            of time.
        """
        pass

    @abstractmethod
    def evaluateCenterOfMass(self):
        """Calculates and returns the time derivative of motor center of mass.
        The result is a function of time, object of the Function class, which is stored in self.zCM.

        Parameters
        ----------
        None

        Returns
        -------
        zCM : Function
            Position of the center of mass as a function
            of time.
        """

        pass

    def evaluateMass(self):
        """Calculates and returns the total propellant mass curve by
        numerically integrating the MassDot curve, calculated in
        evaluateMassDot. Numerical integration is done with the
        Trapezoidal Rule, giving the same result as scipy.integrate.
        odeint but 100x faster. The result is a function of time,
        object of the class Function, which is stored in self.mass.

        Parameters
        ----------
        None

        Returns
        -------
        self.mass : Function
            Total propellant mass as a function of time.
        """
        # Retrieve mass dot curve data
        t = self.massDot.source[:, 0]
        ydot = self.massDot.source[:, 1]

        # Set initial conditions
        T = [0]
        y = [self.propellantInitialMass]

        # Solve for each time point
        for i in range(1, len(t)):
            T += [t[i]]
            y += [y[i - 1] + 0.5 * (t[i] - t[i - 1]) * (ydot[i] + ydot[i - 1])]

        # Create Function
        self.mass = Function(
            np.concatenate(([T], [y])).transpose(),
            "Time (s)",
            "Propellant Total Mass (kg)",
            self.interpolate,
            "constant",
        )

        # Return Mass Function
        return self.mass

    @property
    def throatArea(self):
        return np.pi * self.throatRadius**2

    @abstractmethod
    def evaluateInertia(self):
        """Calculates propellant inertia I, relative to directions
        perpendicular to the rocket body axis and its time derivative
        as a function of time. Also calculates propellant inertia Z,
        relative to the axial direction, and its time derivative as a
        function of time. Products of inertia are assumed null due to
        symmetry. The four functions are stored as an object of the
        Function class.

        Parameters
        ----------
        None

        Returns
        -------
        list of Functions
            The first argument is the Function representing inertia I,
            while the second argument is the Function representing
            inertia Z.
        """

        pass

    def importEng(self, fileName):
        """Read content from .eng file and process it, in order to
        return the comments, description and data points.

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
        """Exports thrust curve data points and motor description to
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
        """Prints out a summary of the data and graphs available about
        the Motor.

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

        return None

    def allInfo(self):
        """Prints out all data and graphs available about the Motor.

        Parameters
        ----------
        None

        Return
        ------
        None
        """
        # Print nozzle details
        print("Nozzle Details")
        print("Nozzle Radius: " + str(self.nozzleRadius) + " m")
        print("Nozzle Throat Radius: " + str(self.throatRadius) + " m")

        # Print grain details
        print("\nGrain Details")
        print("Number of Grains: " + str(self.grainNumber))
        print("Grain Spacing: " + str(self.grainSeparation) + " m")
        print("Grain Density: " + str(self.grainDensity) + " kg/m3")
        print("Grain Outer Radius: " + str(self.grainOuterRadius) + " m")
        print("Grain Inner Radius: " + str(self.grainInitialInnerRadius) + " m")
        print("Grain Height: " + str(self.grainInitialHeight) + " m")
        print("Grain Volume: " + "{:.3f}".format(self.grainInitialVolume) + " m3")
        print("Grain Mass: " + "{:.3f}".format(self.grainInitialMass) + " kg")

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
        self.massDot()
        self.grainInnerRadius()
        self.grainHeight()
        self.burnRate()
        self.burnArea()
        self.Kn()
        self.inertiaI()
        self.inertiaIDot()
        self.inertiaZ()
        self.inertiaZDot()

        return None


class EmptyMotor:
    """Class that represents an empty motor with no mass and no thrust."""

    # TODO: This is a temporary solution. It should be replaced by a class that
    # inherits from the abstract Motor class. Currently cannot be done easily.
    def __init__(self):
        """Initializes an empty motor with no mass and no thrust."""
        self._csys = 1
        self.nozzleRadius = 0
        self.thrust = Function(0, "Time (s)", "Thrust (N)")
        self.mass = Function(0, "Time (s)", "Mass (kg)")
        self.massDot = Function(0, "Time (s)", "Mass Depletion Rate (kg/s)")
        self.burnOutTime = 1
        self.nozzlePosition = 0
        self.centerOfMass = Function(0, "Time (s)", "Mass (kg)")
        self.inertiaZ = Function(0, "Time (s)", "Moment of Inertia Z (kg m²)")
        self.inertiaI = Function(0, "Time (s)", "Moment of Inertia I (kg m²)")
        self.inertiaZDot = Function(0, "Time (s)", "Propellant Inertia Z Dot (kgm²/s)")
        self.inertiaIDot = Function(0, "Time (s)", "Propellant Inertia I Dot (kgm²/s)")
