"""Defines the StochasticRocket class."""

import warnings
import numpy as np
from random import choice

from rocketpy.control.controller import _Controller
from rocketpy.motors.motor import EmptyMotor, GenericMotor, Motor
from rocketpy.motors.solid_motor import SolidMotor
from rocketpy.rocket.aero_surface import (
    AirBrakes,
    EllipticalFins,
    NoseCone,
    RailButtons,
    Tail,
    TrapezoidalFins,
)
from rocketpy.rocket.components import Components
from rocketpy.rocket.parachute import Parachute
from rocketpy.rocket.rocket import Rocket
from rocketpy.stochastic.stochastic_generic_motor import StochasticGenericMotor
from rocketpy.stochastic.stochastic_motor_model import StochasticMotorModel

from .stochastic_aero_surfaces import (
    StochasticEllipticalFins,
    StochasticNoseCone,
    StochasticRailButtons,
    StochasticTail,
    StochasticTrapezoidalFins,
)
from .stochastic_model import StochasticModel
from .stochastic_parachute import StochasticParachute
from .stochastic_solid_motor import StochasticSolidMotor

# TODO: Private methods of this class should be double underscored


class StochasticRocket(StochasticModel):
    """A Stochastic Rocket class that inherits from StochasticModel.

    See Also
    --------
    :ref:`stochastic_model` and :class:`Rocket <rocketpy.Rocket>`

    Attributes
    ----------
    obj : Rocket
        The Rocket object to be used as a base for the Stochastic rocket.
    motors : Components
        A Components instance containing all the motors of the rocket.
    aerodynamic_surfaces : Components
        A Components instance containing all the aerodynamic surfaces of the
        rocket.
    rail_buttons : Components
        A Components instance containing all the rail buttons of the rocket.
    parachutes : list of StochasticParachute
        A list of StochasticParachute instances containing all the parachutes of
        the rocket.
    radius : tuple, list, int, float
        The radius of the rocket.
    mass : tuple, list, int, float
        The mass of the rocket.
    inertia_11 : tuple, list, int, float
        The inertia of the rocket around the x axis.
    inertia_22 : tuple, list, int, float
        The inertia of the rocket around the y axis.
    inertia_33 : tuple, list, int, float
        The inertia of the rocket around the z axis.
    inertia_12 : tuple, list, int, float
        The inertia of the rocket around the xy axis.
    inertia_13 : tuple, list, int, float
        The inertia of the rocket around the xz axis.
    inertia_23 : tuple, list, int, float
        The inertia of the rocket around the yz axis.
    power_off_drag : list
        The power off drag of the rocket.
    power_on_drag : list
        The power on drag of the rocket.
    power_off_drag_factor : tuple, list, int, float
        The power off drag factor of the rocket.
    power_on_drag_factor : tuple, list, int, float
        The power on drag factor of the rocket.
    center_of_mass_without_motor : tuple, list, int, float
        The center of mass of the rocket without the motor.
    coordinate_system_orientation : list[str]
        The orientation of the coordinate system of the rocket. This attribute
        can not be a randomized.
    """

    def __init__(
        self,
        rocket,
        radius=None,
        mass=None,
        inertia_11=None,
        inertia_22=None,
        inertia_33=None,
        inertia_12=None,
        inertia_13=None,
        inertia_23=None,
        power_off_drag=None,
        power_on_drag=None,
        power_off_drag_factor=(1, 0),
        power_on_drag_factor=(1, 0),
        center_of_mass_without_motor=None,
    ):
        """Initializes the Stochastic Rocket class.

        See Also
        --------
        :ref:`stochastic_model`

        Parameters
        ----------
        rocket : Rocket
            The Rocket object to be used as a base for the Stochastic rocket.
        radius : int, float, tuple, list, optional
            The radius of the rocket.
        mass : int, float, tuple, list, optional
            The mass of the rocket.
        inertia_11 : int, float, tuple, list, optional
            The inertia of the rocket around the x axis.
        inertia_22 : int, float, tuple, list, optional
            The inertia of the rocket around the y axis.
        inertia_33 : int, float, tuple, list, optional
            The inertia of the rocket around the z axis.
        inertia_12 : int, float, tuple, list, optional
            The inertia of the rocket around the xy axis.
        inertia_13 : int, float, tuple, list, optional
            The inertia of the rocket around the xz axis.
        inertia_23 : int, float, tuple, list, optional
            The inertia of the rocket around the yz axis.
        power_off_drag : list, optional
            The power off drag of the rocket.
        power_on_drag : list, optional
            The power on drag of the rocket.
        power_off_drag_factor : int, float, tuple, list, optional
            The power off drag factor of the rocket. This represents a factor
            that multiplies the power off drag curve.
        power_on_drag_factor : int, float, tuple, list, optional
            The power on drag factor of the rocket. This represents a factor
            that multiplies the power on drag curve.
        center_of_mass_without_motor : int, float, tuple, list, optional
            The center of mass of the rocket without the motor.
        """
        # TODO: mention that these factors are validated differently
        self._validate_1d_array_like("power_off_drag", power_off_drag)
        self._validate_1d_array_like("power_on_drag", power_on_drag)
        super().__init__(
            obj=rocket,
            radius=radius,
            mass=mass,
            I_11_without_motor=inertia_11,
            I_22_without_motor=inertia_22,
            I_33_without_motor=inertia_33,
            I_12_without_motor=inertia_12,
            I_13_without_motor=inertia_13,
            I_23_without_motor=inertia_23,
            power_off_drag=power_off_drag,
            power_on_drag=power_on_drag,
            power_off_drag_factor=power_off_drag_factor,
            power_on_drag_factor=power_on_drag_factor,
            center_of_mass_without_motor=center_of_mass_without_motor,
            coordinate_system_orientation=None,
        )
        self.motors = Components()
        self.aerodynamic_surfaces = Components()
        self.rail_buttons = Components()
        self.parachutes = []
        
        # Controllers data initialization
        self._controllers = []

        # AirBrakes data initialization
        self.air_brakes = []

    def add_motor(self, motor, position=None):
        """Adds a stochastic motor to the stochastic rocket. If a motor is
        already present, it will be replaced.

        Parameters
        ----------
        motor : StochasticMotor or Motor
            The motor to be added to the stochastic rocket.
        position : tuple, list, int, float, optional
            The position of the motor.
        """
        # checks if there is a motor already
        if len(self.motors) > 0:
            warnings.warn(
                "Only one motor can be added to the stochastic rocket. "
                "The previous motor will be replaced."
            )
            self.motors = Components()

        # checks if input is a Motor
        if not isinstance(motor, (Motor, StochasticMotorModel)):
            raise TypeError("`motor` must be a StochasticMotor or Motor type")
        if isinstance(motor, Motor):
            # TODO implement HybridMotor and LiquidMotor stochastic models
            if isinstance(motor, SolidMotor):
                motor = StochasticSolidMotor(solid_motor=motor)
            elif isinstance(motor, GenericMotor):
                motor = StochasticGenericMotor(generic_motor=motor)
        self.motors.add(motor, self._validate_position(motor, position))

    def _add_surfaces(self, surfaces, positions, type_, stochastic_type, error_message):
        """Adds a stochastic aerodynamic surface to the stochastic rocket. If
        an aerodynamic surface is already present, it will be replaced.

        Parameters
        ----------
        surfaces : StochasticAeroSurface or AeroSurface
            The aerodynamic surface to be added to the stochastic rocket.
        positions : tuple, list, int, float, optional
            The position of the aerodynamic surface.
        type_ : type
            The type of the aerodynamic surface to be added to the stochastic
            rocket.
        stochastic_type : type
            The type of the stochastic aerodynamic surface to be added to the
            stochastic rocket.
        error_message : str
            The error message to be raised if the input is not of the correct
            type.
        """
        if not isinstance(surfaces, (type_, stochastic_type)):
            raise AssertionError(error_message)
        if isinstance(surfaces, type_):
            surfaces = stochastic_type(component=surfaces)
        self.aerodynamic_surfaces.add(
            surfaces, self._validate_position(surfaces, positions)
        )
        
    def _add_controllers(self, controllers):
        """Adds a controller to the rocket.

        Parameters
        ----------
        controllers : list of Controller objects
            List of controllers to be added to the rocket. If a single
            Controller object is passed, outside of a list, a try/except block
            will be used to try to append the controller to the list.

        Returns
        -------
        None
        """
        try:
            self._controllers.extend(controllers)
        except TypeError:
            self._controllers.append(controllers)

    def add_nose(self, nose, position=None):
        """Adds a stochastic nose cone to the stochastic rocket.

        Parameters
        ----------
        nose : StochasticNoseCone or NoseCone
            The nose cone to be added to the stochastic rocket.
        position : tuple, list, int, float, optional
            The position of the nose cone.
        """
        self._add_surfaces(
            surfaces=nose,
            positions=position,
            type_=NoseCone,
            stochastic_type=StochasticNoseCone,
            error_message="`nose` must be of NoseCone or StochasticNoseCone type",
        )

    def add_trapezoidal_fins(self, fins, position=None):
        """Adds a stochastic trapezoidal fins to the stochastic rocket.

        Parameters
        ----------
        fins : StochasticTrapezoidalFins or TrapezoidalFins
            The trapezoidal fins to be added to the stochastic rocket.
        position : tuple, list, int, float, optional
            The position of the trapezoidal fins.
        """
        self._add_surfaces(
            fins,
            position,
            TrapezoidalFins,
            StochasticTrapezoidalFins,
            "`fins` must be of TrapezoidalFins or StochasticTrapezoidalFins type",
        )

    def add_elliptical_fins(self, fins, position=None):
        """Adds a stochastic elliptical fins to the stochastic rocket.

        Parameters
        ----------
        fins : StochasticEllipticalFins or EllipticalFins
            The elliptical fins to be added to the stochastic rocket.
        position : tuple, list, int, float, optional
            The position of the elliptical fins.
        """
        self._add_surfaces(
            fins,
            position,
            EllipticalFins,
            StochasticEllipticalFins,
            "`fins` must be of EllipticalFins or StochasticEllipticalFins type",
        )

    def add_tail(self, tail, position=None):
        """Adds a stochastic tail to the stochastic rocket.

        Parameters
        ----------
        tail : StochasticTail or Tail
            The tail to be added to the stochastic rocket.
        position : tuple, list, int, float, optional
            The position of the tail.
        """
        self._add_surfaces(
            tail,
            position,
            Tail,
            StochasticTail,
            "`tail` must be of Tail or StochasticTail type",
        )

    def add_parachute(self, parachute):
        """Adds a stochastic parachute to the stochastic rocket.

        Parameters
        ----------
        parachute : StochasticParachute or Parachute
            The parachute to be added to the stochastic rocket.
        """
        # checks if input is a StochasticParachute type
        if not isinstance(parachute, (Parachute, StochasticParachute)):
            raise TypeError(
                "`parachute` must be of Parachute or StochasticParachute type"
            )
        if isinstance(parachute, Parachute):
            parachute = StochasticParachute(parachute=parachute)
        self.parachutes.append(parachute)

    def add_air_brakes(
        self,
        drag_coefficient_curve,
        controller_function,
        sampling_rate,
        clamp=True,
        reference_area=None,
        initial_observed_variables=None,
        override_rocket_drag=False,
        return_controller=False,
        name="AirBrakes",
        controller_name="AirBrakes Controller",
    ):
        """Creates a new air brakes system, storing its parameters such as
        drag coefficient curve, controller function, sampling rate, and
        reference area.

        Parameters
        ----------
        drag_coefficient_curve : int, float, callable, array, string, Function
            This parameter represents the drag coefficient associated with the
            air brakes and/or the entire rocket, depending on the value of
            ``override_rocket_drag``.

            - If a constant, it should be an integer or a float representing a
              fixed drag coefficient value.
            - If a function, it must take two parameters: deployment level and
              Mach number, and return the drag coefficient. This function allows
              for dynamic computation based on deployment and Mach number.
            - If an array, it should be a 2D array with three columns: the first
              column for deployment level, the second for Mach number, and the
              third for the corresponding drag coefficient.
            - If a string, it should be the path to a .csv or .txt file. The
              file must contain three columns: the first for deployment level,
              the second for Mach number, and the third for the drag
              coefficient.
            - If a Function, it must take two parameters: deployment level and
              Mach number, and return the drag coefficient.

            .. note:: For ``override_rocket_drag = False``, at
                deployment level 0, the drag coefficient is assumed to be 0,
                independent of the input drag coefficient curve. This means that
                the simulation always considers that at a deployment level of 0,
                the air brakes are completely retracted and do not contribute to
                the drag of the rocket.

        controller_function : function, callable
            An user-defined function responsible for controlling the simulation.
            This function is expected to take the following arguments, in order:

            1. `time` (float): The current simulation time in seconds.
            2. `sampling_rate` (float): The rate at which the controller
               function is called, measured in Hertz (Hz).
            3. `state` (list): The state vector of the simulation, structured as
               `[x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]`.
            4. `state_history` (list): A record of the rocket's state at each
               step throughout the simulation. The state_history is organized as a
               list of lists, with each sublist containing a state vector. The last
               item in the list always corresponds to the previous state vector,
               providing a chronological sequence of the rocket's evolving states.
            5. `observed_variables` (list): A list containing the variables that
               the controller function returns. The initial value in the first
               step of the simulation of this list is provided by the
               `initial_observed_variables` argument.
            6. `interactive_objects` (list): A list containing the objects that
               the controller function can interact with. The objects are
               listed in the same order as they are provided in the
               `interactive_objects`

            This function will be called during the simulation at the specified
            sampling rate. The function should evaluate and change the observed
            objects as needed. The function should return None.

            .. note:: The function will be called according to the sampling rate
            specified.
        sampling_rate : float
            The sampling rate of the controller function in Hertz (Hz). This
            means that the controller function will be called every
            `1/sampling_rate` seconds.
        clamp : bool, optional
            If True, the simulation will clamp the deployment level to 0 or 1 if
            the deployment level is out of bounds. If False, the simulation will
            not clamp the deployment level and will instead raise a warning if
            the deployment level is out of bounds. Default is True.
        reference_area : float, optional
            Reference area used to calculate the drag force of the air brakes
            from the drag coefficient curve. If None, which is default, use
            rocket section area. Must be given in squared meters.
        initial_observed_variables : list, optional
            A list of the initial values of the variables that the controller
            function returns. This list is used to initialize the
            `observed_variables` argument of the controller function. The
            default value is None, which initializes the list as an empty list.
        override_rocket_drag : bool, optional
            If False, the air brakes drag coefficient will be added to the
            rocket's power off drag coefficient curve. If True, during the
            simulation, the rocket's power off drag will be ignored and the air
            brakes drag coefficient will be used for the entire rocket instead.
            Default is False.
        return_controller : bool, optional
            If True, the function will return the controller object created.
            Default is False.
        name : string, optional
            AirBrakes name, such as drogue and main. Has no impact in
            simulation, as it is only used to display data in a more
            organized matter.
        controller_name : string, optional
            Controller name. Has no impact in simulation, as it is only used to
            display data in a more organized matter.

        Returns
        -------
        air_brakes : AirBrakes
            AirBrakes object created.
        controller : Controller
            Controller object created.
        """
        reference_area = reference_area if reference_area is not None else (np.pi * self.radius**2)
        air_brakes = AirBrakes(
            drag_coefficient_curve=drag_coefficient_curve,
            reference_area=reference_area,
            clamp=clamp,
            override_rocket_drag=override_rocket_drag,
            deployment_level=0,
            name=name,
        )
        _controller = _Controller(
            interactive_objects=air_brakes,
            controller_function=controller_function,
            sampling_rate=sampling_rate,
            initial_observed_variables=initial_observed_variables,
            name=controller_name,
        )
        self.air_brakes.append(air_brakes)
        self._add_controllers(_controller)
        if return_controller:
            return air_brakes, _controller
        else:
            return air_brakes

    def set_rail_buttons(
        self,
        rail_buttons,
        lower_button_position=None,
    ):
        """Sets the rail buttons of the stochastic rocket.

        Parameters
        ----------
        rail_buttons : StochasticRailButtons or RailButtons
            The rail buttons to be added to the stochastic rocket.
        lower_button_position : tuple, list, int, float, optional
            The position of the lower button.
        """
        if not isinstance(rail_buttons, (StochasticRailButtons, RailButtons)):
            raise AssertionError(
                "`rail_buttons` must be of RailButtons or StochasticRailButtons type"
            )
        if isinstance(rail_buttons, RailButtons):
            rail_buttons = StochasticRailButtons(rail_buttons=rail_buttons)
        self.rail_buttons.add(
            rail_buttons, self._validate_position(rail_buttons, lower_button_position)
        )

    def _validate_position(self, validated_object, position):
        """Validate the position argument.

        Parameters
        ----------
        validated_object : object
            The object to which the position argument refers to.
        position : tuple, list, int, float
            The position argument to be validated.

        Returns
        -------
        tuple or list
            Validated position argument.

        Raises
        ------
        ValueError
            If the position argument does not conform to the specified formats.
        """

        if isinstance(position, tuple):
            return self._validate_tuple(
                "position",
                position,
                getattr=self._create_get_position(validated_object),
            )
        elif isinstance(position, (int, float)):
            return self._validate_scalar(
                "position",
                position,
                getattr=self._create_get_position(validated_object),
            )
        elif isinstance(position, list):
            return self._validate_list(
                "position",
                position,
                getattr=self._create_get_position(validated_object),
            )
        elif position is None:
            position = []
            return self._validate_list(
                "position",
                position,
                getattr=self._create_get_position(validated_object),
            )
        else:
            raise AssertionError("`position` must be a tuple, list, int, or float")

    def _create_get_position(self, validated_object):
        """Create a function to get the nominal position from an object.

        Parameters
        ----------
        validated_object : object
            The object to which the position argument refers to.

        Returns
        -------
        function
            Function to get the nominal position from an object. The function
            must receive two arguments.
        """

        # try to get position from object
        error_msg = (
            "`position` standard deviation was provided but the rocket does "
            f"not have the same {validated_object.obj.__class__.__name__} "
            "to get the nominal position value from."
        )
        # special case for motor stochastic model
        if isinstance(validated_object, (StochasticMotorModel)):
            if isinstance(self.obj.motor, EmptyMotor):
                raise AssertionError(error_msg)

            def get_motor_position(self_object, _):
                return self_object.motor_position

            return get_motor_position
        else:
            if isinstance(validated_object, StochasticRailButtons):

                def get_surface_position(self_object, _):
                    surfaces = self_object.rail_buttons.get_tuple_by_type(
                        validated_object.obj.__class__
                    )
                    if len(surfaces) == 0:
                        raise AssertionError(error_msg)
                    for surface in surfaces:
                        if surface.component == validated_object.obj:
                            return surface.position
                        else:
                            raise AssertionError(error_msg)

            else:

                def get_surface_position(self_object, _):
                    surfaces = self_object.aerodynamic_surfaces.get_tuple_by_type(
                        validated_object.obj.__class__
                    )
                    if len(surfaces) == 0:
                        raise AssertionError(error_msg)
                    for surface in surfaces:
                        if surface.component == validated_object.obj:
                            return surface.position
                        else:
                            raise AssertionError(error_msg)

            return get_surface_position

    def _randomize_position(self, position):
        """Randomize a position provided as a tuple or list."""
        if isinstance(position, tuple):
            return position[-1](position[0], position[1])
        elif isinstance(position, list):
            return choice(position) if position else position

    # pylint: disable=stop-iteration-return
    def dict_generator(self):
        """Special generator for the rocket class that yields a dictionary with
        the randomly generated input arguments. The dictionary is saved as an
        attribute of the class. The dictionary is generated by looping through
        all attributes of the class and generating a random value for each
        attribute. The random values are generated according to the format of
        each attribute. Tuples are generated using the distribution function
        specified in the tuple. Lists are generated using the random.choice
        function.

        Parameters
        ----------
        None

        Yields
        -------
        dict
            Dictionary with the randomly generated input arguments.
        """
        generated_dict = next(super().dict_generator())
        generated_dict["motors"] = []
        generated_dict["aerodynamic_surfaces"] = []
        generated_dict["rail_buttons"] = []
        generated_dict["parachutes"] = []
        self.last_rnd_dict = generated_dict
        yield generated_dict

    def _create_motor(self, component_stochastic_motor):
        stochastic_motor = component_stochastic_motor.component
        motor = stochastic_motor.create_object()
        position_rnd = self._randomize_position(component_stochastic_motor.position)
        self.last_rnd_dict["motors"].append(stochastic_motor.last_rnd_dict)
        self.last_rnd_dict["motors"][-1]["position"] = position_rnd
        return motor, position_rnd

    def _create_surface(self, component_stochastic_surface):
        stochastic_surface = component_stochastic_surface.component
        surface = stochastic_surface.create_object()
        position_rnd = self._randomize_position(component_stochastic_surface.position)
        self.last_rnd_dict["aerodynamic_surfaces"].append(
            stochastic_surface.last_rnd_dict
        )
        self.last_rnd_dict["aerodynamic_surfaces"][-1]["position"] = position_rnd
        return surface, position_rnd

    def _create_rail_buttons(self, component_stochastic_rail_buttons):
        stochastic_rail_buttons = component_stochastic_rail_buttons.component
        rail_buttons = stochastic_rail_buttons.create_object()
        lower_button_position_rnd = self._randomize_position(
            component_stochastic_rail_buttons.position
        )
        upper_button_position_rnd = (
            rail_buttons.buttons_distance + lower_button_position_rnd
        )
        self.last_rnd_dict["rail_buttons"].append(stochastic_rail_buttons.last_rnd_dict)
        self.last_rnd_dict["rail_buttons"][-1][
            "lower_button_position"
        ] = lower_button_position_rnd
        self.last_rnd_dict["rail_buttons"][-1][
            "upper_button_position"
        ] = upper_button_position_rnd
        return rail_buttons, lower_button_position_rnd, upper_button_position_rnd

    def _create_parachute(self, stochastic_parachute):
        parachute = stochastic_parachute.create_object()
        self.last_rnd_dict["parachutes"].append(stochastic_parachute.last_rnd_dict)
        return parachute

    def create_object(self):
        """Creates and returns a Rocket object from the randomly generated input
        arguments.

        Returns
        -------
        rocket : Rocket
            Rocket object with the randomly generated input arguments.
        """
        generated_dict = next(self.dict_generator())
        rocket = Rocket(
            radius=generated_dict["radius"],
            mass=generated_dict["mass"],
            inertia=(
                generated_dict["I_11_without_motor"],
                generated_dict["I_22_without_motor"],
                generated_dict["I_33_without_motor"],
                generated_dict["I_12_without_motor"],
                generated_dict["I_13_without_motor"],
                generated_dict["I_23_without_motor"],
            ),
            power_off_drag=generated_dict["power_off_drag"],
            power_on_drag=generated_dict["power_on_drag"],
            center_of_mass_without_motor=generated_dict["center_of_mass_without_motor"],
            coordinate_system_orientation=generated_dict[
                "coordinate_system_orientation"
            ],
        )
        rocket.power_off_drag *= generated_dict["power_off_drag_factor"]
        rocket.power_on_drag *= generated_dict["power_on_drag_factor"]

        for component_motor in self.motors:
            motor, position_rnd = self._create_motor(component_motor)
            rocket.add_motor(motor, position_rnd)

        for component_surface in self.aerodynamic_surfaces:
            surface, position_rnd = self._create_surface(component_surface)
            rocket.add_surfaces(surface, position_rnd)

        for component_rail_buttons in self.rail_buttons:
            (
                rail_buttons,
                lower_button_position_rnd,
                upper_button_position_rnd,
            ) = self._create_rail_buttons(component_rail_buttons)
            rocket.set_rail_buttons(
                upper_button_position=upper_button_position_rnd,
                lower_button_position=lower_button_position_rnd,
                angular_position=rail_buttons.angular_position,
            )

        for parachute in self.parachutes:
            parachute = self._create_parachute(parachute)
            rocket.add_parachute(
                name=parachute.name,
                cd_s=parachute.cd_s,
                trigger=parachute.trigger,
                sampling_rate=parachute.sampling_rate,
                lag=parachute.lag,
                noise=parachute.noise,
            )

        return rocket
