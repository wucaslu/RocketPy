# -*- coding: utf-8 -*-
__author__ = "Franz Masatoshi Yuri, Lucas Kierulff Balabram, Guilherme Fernandes Alves"
__copyright__ = "Copyright 20XX, RocketPy Team"
__license__ = "MIT"
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from scipy.integrate import solve_ivp

from .Environment import Environment
from .Function import Function

# Parachutes calculations


def compute_CdS_from_drop_test(
    terminal_velocity, rocket_mass, air_density=1.225, g=9.80665
):
    """Returns the parachute's CdS calculated through its final speed, air
    density in the landing point, the rocket's mass and the force of gravity
    in the landing point.

    Parameters
    ----------
    terminal_velocity : float
        Rocket's speed in m/s when landing.
    rocket_mass : float
        Rocket's dry mass in kg.
    air_density : float, optional
        Air density, in kg/m^3, right before the rocket lands. Default value is 1.225.
    g : float, optional
        Gravitational acceleration experienced by the rocket and parachute during
        descent in m/s^2. Default value is the standard gravity, 9.80665.

    Returns
    -------
    CdS : float
        Number equal to drag coefficient times reference area for parachute.

    """

    return 2 * rocket_mass * g / ((terminal_velocity**2) * air_density)


# TODO: Needs tests
def calculateEquilibriumAltitude(
    rocket_mass,
    CdS,
    z0,
    v0=0,
    env=None,
    eps=1e-3,
    max_step=0.1,
    seeGraphs=True,
    g=9.80665,
    estimated_final_time=10,
):
    """Returns a dictionary containing the time, altitude and velocity of the
    system rocket-parachute in which the terminal velocity is reached.


    Parameters
    ----------
    rocket_mass : float
        Rocket's mass in kg.
    CdS : float
        Number equal to drag coefficient times reference area for parachute.
    z0 : float
        Initial altitude of the rocket in meters.
    v0 : float, optional
        Rocket's initial speed in m/s. Must be negative
    env : Environment, optional
        Environmental conditions at the time of the launch.
    eps : float, optional
        acceptable error in meters.
    max_step: float, optional
        maximum allowed time step size to solve the integration
    seeGraphs : boolean, optional
        True if you want to see time vs altitude and time vs speed graphs,
        False otherwise.
    g : float, optional
        Gravitational acceleration experienced by the rocket and parachute during
        descent in m/s^2. Default value is the standard gravity, 9.80665.
    estimated_final_time: float, optional
        Estimative of how much time (in seconds) will spend until vertical terminal
        velocity is reached. Must be positive. Default is 10. It can affect the final
        result if the value is not high enough. Increase the estimative in case the
        final solution is not founded.


    Returns
    -------
    altitudeFunction: Function
        Altitude as a function of time. Always a Function object.
    velocityFunction:
        Vertical velocity as a function of time. Always a Function object.
    final_sol : dictionary
        Dictionary containing the values for time, altitude and speed of
        the rocket when it reaches terminal velocity.
    """
    final_sol = {}

    if not v0 < 0:
        print("Please set a valid negative value for v0")
        return None

    # TODO: Improve docs
    def check_constant(f, eps):
        """_summary_

        Parameters
        ----------
        f : array, list
            _description_
        eps : float
            _description_

        Returns
        -------
        int, None
            _description_
        """
        for i in range(len(f) - 2):
            if abs(f[i + 2] - f[i + 1]) < eps and abs(f[i + 1] - f[i]) < eps:
                return i
        return None

    if env == None:
        environment = Environment(
            railLength=5.0,
            latitude=0,
            longitude=0,
            elevation=1000,
            date=(2020, 3, 4, 12),
        )
    else:
        environment = env

    # TODO: Improve docs
    def du(z, u):
        """_summary_

        Parameters
        ----------
        z : float
            _description_
        u : float
            velocity, in m/s, at a given z altitude

        Returns
        -------
        float
            _description_
        """
        return (
            u[1],
            -g + environment.density(z) * ((u[1]) ** 2) * CdS / (2 * rocket_mass),
        )

    u0 = [z0, v0]

    us = solve_ivp(
        fun=du,
        t_span=(0, estimated_final_time),
        y0=u0,
        vectorized=True,
        method="LSODA",
        max_step=max_step,
    )

    constant_index = check_constant(us.y[1], eps)

    # TODO: Improve docs by explaining what is happening below with constant_index
    if constant_index is not None:
        final_sol = {
            "time": us.t[constant_index],
            "altitude": us.y[0][constant_index],
            "velocity": us.y[1][constant_index],
        }

    altitudeFunction = Function(
        source=np.array(list(zip(us.t, us.y[0])), dtype=np.float64),
        inputs="Time (s)",
        outputs="Altitude (m)",
        interpolation="linear",
    )

    velocityFunction = Function(
        source=np.array(list(zip(us.t, us.y[1])), dtype=np.float64),
        inputs="Time (s)",
        outputs="Vertical Velocity (m/s)",
        interpolation="linear",
    )

    if seeGraphs:
        altitudeFunction()
        velocityFunction()

    return altitudeFunction, velocityFunction, final_sol


def create_dispersion_dictionary(filename):
    """Creates a dictionary with the rocket data provided by a .csv file.
    File should be organized in four columns: attribute_class, parameter_name,
    mean_value, standard_deviation. The first row should be the header.
    It is advised to use ";" as separator, but "," should work on most of cases.
    Parameters
    ----------
    filename : string
        String with the path to the .csv file.
    Returns
    -------
    dictionary
        Dictionary with all rocket data to be used in dispersion analysis.
    """
    try:
        file = np.genfromtxt(
            filename, usecols=(1, 2, 3), skip_header=1, delimiter=";", dtype=str
        )
    except:
        print(
            "Error: The delimiter should be ';'. Using ',' instead, be aware that some resources might not work as expected. Please consider changing the delimiter to ';'."
        )
        file = np.genfromtxt(
            filename, usecols=(1, 2, 3), skip_header=1, delimiter=",", dtype=str
        )
    analysis_parameters = dict()
    for row in file:
        if row[0] != "":
            if row[2] == "":
                try:
                    analysis_parameters[row[0].strip()] = float(row[1])
                except:
                    analysis_parameters[row[0].strip()] = eval(row[1])
            else:
                try:
                    analysis_parameters[row[0].strip()] = (float(row[1]), float(row[2]))
                except:
                    analysis_parameters[row[0].strip()] = ""
    return analysis_parameters


# Geodesic calculations functions


def calculateEarthRadius(lat, datum="WGS84"):
    """Simple function to calculate the Earth Radius at a specific latitude
    based on ellipsoidal reference model (datum). The earth radius here is
    assumed as the distance between the ellipsoid's center of gravity and a
    point on ellipsoid surface at the desired
    Pay attention: The ellipsoid is an approximation for the earth model and
    will obviously output an estimate of the perfect distance between earth's
    relief and its center of gravity.

    Parameters
    ----------
    lat : float
        latitude in which the Earth radius will be calculated
    datum : string, optional
        The desired reference ellipsoid model, the following options are
        available: "SAD69", "WGS84", "NAD83", and "SIRGAS2000". The default
        is "WSG84", then this model will be used if the user make some
        typing mistake.

    Returns
    -------
    float:
        Earth Radius at the desired latitude in meters
    """
    # Select the desired datum (i.e. the ellipsoid parameters)
    if datum == "SAD69":
        semiMajorAxis = 6378160.0
        flattening = 1 / 298.25
    elif datum == "SIRGAS2000":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563
    elif datum == "NAD83":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257024899
    else:
        # Use WGS84 as default
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563

    # Calculate the semi minor axis length
    semiMinorAxis = semiMajorAxis * (1 - flattening)

    # Convert latitude to radians
    lat = lat * np.pi / 180

    # Calculate the Earth Radius in meters
    eRadius = np.sqrt(
        (
            (np.cos(lat) * (semiMajorAxis**2)) ** 2
            + (np.sin(lat) * (semiMinorAxis**2)) ** 2
        )
        / ((np.cos(lat) * semiMajorAxis) ** 2 + (np.sin(lat) * semiMinorAxis) ** 2)
    )

    # Convert latitude back to degrees
    lat = lat * 180 / np.pi

    return eRadius


def Haversine(lat0, lon0, lat1, lon1, eRadius=6.3781e6):
    """Returns the distance between two points in meters.
    The points are defined by their latitude and longitude coordinates.

    Parameters
    ----------
    lat0 : float
        Latitude of the first point, in degrees.
    lon0 : float
        Longitude of the first point, in degrees.
    lat1 : float
        Latitude of the second point, in degrees.
    lon1 : float
        Longitude of the second point, in degrees.
    eRadius : float, optional
        Earth's radius in meters. Default value is 6.3781e6.

    Returns
    -------
    float
        Distance between the two points in meters.

    """
    lat0_rad = math.radians(lat0)
    lat1_rad = math.radians(lat1)
    delta_lat_rad = math.radians(lat1 - lat0)
    delta_lon_rad = math.radians(lon1 - lon0)

    a = (
        math.sin(delta_lat_rad / 2) ** 2
        + math.cos(lat0_rad) * math.cos(lat1_rad) * math.sin(delta_lon_rad / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return eRadius * c


def invertedHaversine(lat0, lon0, distance, bearing, eRadius=6.3781e6):
    """Returns a tuple with new latitude and longitude coordinates considering
    a displacement of a given distance in a given direction (bearing compass)
    starting from a point defined by (lat0, lon0). This is the opposite of
    Haversine function.

    Parameters
    ----------
    lat0 : float
        Origin latitude coordinate, in degrees.
    lon0 : float
        Origin longitude coordinate, in degrees.
    distance : float
        Distance from the origin point, in meters.
    bearing : float
        Azimuth (or bearing compass) from the origin point, in degrees.
    eRadius : float, optional
        Earth radius, in meters. Default value is 6.3781e6.
        See the utilities.calculateEarthRadius() function for more accuracy.

    Returns
    -------
    lat1 : float
        New latitude coordinate, in degrees.
    lon1 : float
        New longitude coordinate, in degrees.

    """

    # Convert coordinates to radians
    lat0_rad = np.deg2rad(lat0)
    lon0_rad = np.deg2rad(lon0)

    # Apply inverted Haversine formula
    lat1_rad = math.asin(
        math.sin(lat0_rad) * math.cos(distance / eRadius)
        + math.cos(lat0_rad) * math.sin(distance / eRadius) * math.cos(bearing)
    )

    lon1_rad = lon0_rad + math.atan2(
        math.sin(bearing) * math.sin(distance / eRadius) * math.cos(lat0_rad),
        math.cos(distance / eRadius) - math.sin(lat0_rad) * math.sin(lat1_rad),
    )

    # Convert back to degrees and then return
    lat1_deg = np.rad2deg(lat1_rad)
    lon1_deg = np.rad2deg(lon1_rad)

    return lat1_deg, lon1_deg


def decimalDegreesToArcSeconds(angle):
    """Function to convert an angle in decimal degrees to deg/min/sec.
     Converts (°) to (° ' ")

    Parameters
    ----------
    angle : float
        The angle that you need convert to deg/min/sec. Must be given in
        decimal degrees.

    Returns
    -------
    deg: float
        The degrees.
    min: float
        The arc minutes. 1 arc-minute = (1/60)*degree
    sec: float
        The arc Seconds. 1 arc-second = (1/3600)*degree
    """

    if angle < 0:
        signal = -1
    else:
        signal = 1

    deg = (signal * angle) // 1
    min = abs(signal * angle - deg) * 60 // 1
    sec = abs((signal * angle - deg) * 60 - min) * 60

    return deg, min, sec


def geodesicToUtm(lat, lon, datum="WGS84"):
    """Function which converts geodesic coordinates, i.e. lat/lon, to UTM
    projection coordinates. Can be used only for latitudes between -80.00°
    and 84.00°

    Parameters
    ----------
    lat : float
        The latitude coordinates of the point of analysis, must be contained
        between -80.00° and 84.00°
    lon : float
        The longitude coordinates of the point of analysis, must be contained
        between -180.00° and 180.00°
    datum : string
        The desired reference ellipsoid model, the following options are
        available: "SAD69", "WGS84", "NAD83", and "SIRGAS2000". The default
        is "WGS84", then this model will be used if the user make some
        typing mistake

    Returns
    -------
    x: float
        East coordinate in meters, always positive
    y:
        North coordinate in meters, always positive
    utmZone: int
        The number of the UTM zone of the point of analysis, can vary between
        1 and 60
    utmLetter: string
        The letter of the UTM zone of the point of analysis, can vary between
        C and X, omitting the letters "I" and "O"
    NorthSouthHemisphere: string
        Returns "S" for southern hemisphere and "N" for Northern hemisphere
    EastWestHemisphere: string
        Returns "W" for western hemisphere and "E" for eastern hemisphere
    """

    # Calculate the central meridian of UTM zone
    if lon != 0:
        # signal = 1 for positive longitude and -1 for negative longitude
        signal = lon / abs(lon)
        if signal > 0:
            aux = lon - 3
            aux = aux * signal
            div = aux // 6
            lon_mc = div * 6 + 3
            EastWestHemisphere = "E"  # Eastern hemisphere
        else:
            aux = lon + 3
            aux = aux * signal
            div = aux // 6
            lon_mc = (div * 6 + 3) * signal
            EastWestHemisphere = "W"  # Western hemisphere
    else:
        # If the longitude is zero, the central meridian receives number 3
        lon_mc = 3
        EastWestHemisphere = "W|E"

    # Select the desired datum (i.e. the ellipsoid parameters)
    # TODO: Create separate function that returns the ellipsoid parameters
    if datum == "SAD69":
        semiMajorAxis = 6378160.0
        flattening = 1 / 298.25
    elif datum == "SIRGAS2000":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563
    elif datum == "NAD83":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257024899
    else:
        # WGS84
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563

    # Evaluate the S/N hemisphere and determine the N coordinate at the Equator
    if lat < 0:
        N0 = 10000000
        NorthSouthHemisphere = "S"
    else:
        N0 = 0
        NorthSouthHemisphere = "N"

    # Convert the input lat and lon to radians
    lat = lat * np.pi / 180
    lon = lon * np.pi / 180
    lon_mc = lon_mc * np.pi / 180

    # Evaluate reference parameters
    K0 = 1 - 1 / 2500
    e2 = 2 * flattening - flattening**2
    e2lin = e2 / (1 - e2)

    # Evaluate auxiliary parameters
    A = e2 * e2
    B = A * e2
    C = np.sin(2 * lat)
    D = np.sin(4 * lat)
    E = np.sin(6 * lat)
    F = (1 - e2 / 4 - 3 * A / 64 - 5 * B / 256) * lat
    G = (3 * e2 / 8 + 3 * A / 32 + 45 * B / 1024) * C
    H = (15 * A / 256 + 45 * B / 1024) * D
    I = (35 * B / 3072) * E

    # Evaluate other reference parameters
    n = semiMajorAxis / ((1 - e2 * (np.sin(lat) ** 2)) ** 0.5)
    t = np.tan(lat) ** 2
    c = e2lin * (np.cos(lat) ** 2)
    ag = (lon - lon_mc) * np.cos(lat)
    m = semiMajorAxis * (F - G + H - I)

    # Evaluate new auxiliary parameters
    J = (1 - t + c) * ag * ag * ag / 6
    K = (5 - 18 * t + t * t + 72 * c - 58 * e2lin) * (ag**5) / 120
    L = (5 - t + 9 * c + 4 * c * c) * ag * ag * ag * ag / 24
    M = (61 - 58 * t + t * t + 600 * c - 330 * e2lin) * (ag**6) / 720

    # Evaluate the final coordinates
    x = 500000 + K0 * n * (ag + J + K)
    y = N0 + K0 * (m + n * np.tan(lat) * (ag * ag / 2 + L + M))

    # Convert the output lat and lon to degrees
    lat = lat * 180 / np.pi
    lon = lon * 180 / np.pi
    lon_mc = lon_mc * 180 / np.pi

    # Calculate the UTM zone number
    utmZone = int((lon_mc + 183) / 6)

    # Calculate the UTM zone letter
    letters = "CDEFGHJKLMNPQRSTUVWXX"
    utmLetter = letters[int(80 + lat) >> 3]

    return x, y, utmZone, utmLetter, NorthSouthHemisphere, EastWestHemisphere


def utmToGeodesic(x, y, utmZone, NorthSouthHemisphere, datum):
    """Function to convert UTM coordinates to geodesic coordinates
    (i.e. latitude and longitude). The latitude should be between -80°
    and 84°

    Parameters
    ----------
    x : float
        East UTM coordinate in meters
    y : float
        North UTM coordinate in meters
    utmZone : int
        The number of the UTM zone of the point of analysis, can vary between
        1 and 60
    NorthSouthHemisphere : string
        Equals to "S" for southern hemisphere and "N" for Northern hemisphere
    datum : string
        The desired reference ellipsoid model, the following options are
        available: "SAD69", "WGS84", "NAD83", and "SIRGAS2000". The default
        is "WGS84", then this model will be used if the user make some
        typing mistake

    Returns
    -------
    lat: float
        latitude of the analyzed point
    lon: float
        latitude of the analyzed point
    """

    if NorthSouthHemisphere == "N":
        y = y + 10000000

    # Calculate the Central Meridian from the UTM zone number
    centralMeridian = utmZone * 6 - 183  # degrees

    # Select the desired datum
    if datum == "SAD69":
        semiMajorAxis = 6378160.0
        flattening = 1 / 298.25
    elif datum == "SIRGAS2000":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563
    elif datum == "NAD83":
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257024899
    else:
        # WGS84
        semiMajorAxis = 6378137.0
        flattening = 1 / 298.257223563

    # Calculate reference values
    K0 = 1 - 1 / 2500
    e2 = 2 * flattening - flattening**2
    e2lin = e2 / (1 - e2)
    e1 = (1 - (1 - e2) ** 0.5) / (1 + (1 - e2) ** 0.5)

    # Calculate auxiliary values
    A = e2 * e2
    B = A * e2
    C = e1 * e1
    D = e1 * C
    E = e1 * D

    m = (y - 10000000) / K0
    mi = m / (semiMajorAxis * (1 - e2 / 4 - 3 * A / 64 - 5 * B / 256))

    # Calculate other auxiliary values
    F = (3 * e1 / 2 - 27 * D / 32) * np.sin(2 * mi)
    G = (21 * C / 16 - 55 * E / 32) * np.sin(4 * mi)
    H = (151 * D / 96) * np.sin(6 * mi)

    lat1 = mi + F + G + H
    c1 = e2lin * (np.cos(lat1) ** 2)
    t1 = np.tan(lat1) ** 2
    n1 = semiMajorAxis / ((1 - e2 * (np.sin(lat1) ** 2)) ** 0.5)
    qc = (1 - e2 * np.sin(lat1) * np.sin(lat1)) ** 3
    r1 = semiMajorAxis * (1 - e2) / (qc**0.5)
    d = (x - 500000) / (n1 * K0)

    # Calculate other auxiliary values
    I = (5 + 3 * t1 + 10 * c1 - 4 * c1 * c1 - 9 * e2lin) * d * d * d * d / 24
    J = (
        (61 + 90 * t1 + 298 * c1 + 45 * t1 * t1 - 252 * e2lin - 3 * c1 * c1)
        * (d**6)
        / 720
    )
    K = d - (1 + 2 * t1 + c1) * d * d * d / 6
    L = (5 - 2 * c1 + 28 * t1 - 3 * c1 * c1 + 8 * e2lin + 24 * t1 * t1) * (d**5) / 120

    # Finally calculate the coordinates in lat/lot
    lat = lat1 - (n1 * np.tan(lat1) / r1) * (d * d / 2 - I + J)
    lon = centralMeridian * np.pi / 180 + (K + L) / np.cos(lat1)

    # Convert final lat/lon to Degrees
    lat = lat * 180 / np.pi
    lon = lon * 180 / np.pi

    return lat, lon
