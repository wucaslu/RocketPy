class _RocketPrints:
    """Class that holds prints methods for Rocket class.

    Attributes
    ----------
    _RocketPrints.rocket : rocket
        Rocket object that will be used for the prints.

    """

    def __init__(self, rocket):
        """Initializes _EnvironmentPrints class

        Parameters
        ----------
        rocket: rocketpy.rocket
            Instance of the rocket class.

        Returns
        -------
        None
        """
        self.rocket = rocket

        pass

    def inertia_details(self):
        """Print inertia details.

        Returns
        -------
        None
        """
        print("\nInertia Details\n")
        print("Rocket Mass: {:.3f} kg".format(self.rocket.mass))
        print("Rocket Dry Mass: {:.3f} kg (With Motor)".format(self.rocket.dry_mass))
        print(
            "Rocket Mass: {:.3f} kg (With Propellant)".format(self.rocket.total_mass(0))
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 11: {:.3f} kg*m2".format(
                self.rocket.dry_I_11
            )
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 22: {:.3f} kg*m2".format(
                self.rocket.dry_I_22
            )
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 33: {:.3f} kg*m2".format(
                self.rocket.dry_I_33
            )
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 12: {:.3f} kg*m2".format(
                self.rocket.dry_I_12
            )
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 13: {:.3f} kg*m2".format(
                self.rocket.dry_I_13
            )
        )
        print(
            "Rocket Inertia (with motor, but without propellant) 23: {:.3f} kg*m2\n".format(
                self.rocket.dry_I_23
            )
        )

        return None

    def rocket_geometrical_parameters(self):
        """Print rocket geometrical parameters.

        Returns
        -------
        None
        """
        print("\nGeometrical Parameters\n")
        print("Rocket Maximum Radius: " + str(self.rocket.radius) + " m")
        print("Rocket Frontal Area: " + "{:.6f}".format(self.rocket.area) + " m2")
        print("\nRocket Distances")
        print(
            "Rocket Center of Dry Mass - Center of Mass without Motor: "
            + "{:.3f} m".format(
                abs(
                    self.rocket.center_of_mass_without_motor
                    - self.rocket.center_of_dry_mass_position
                )
            )
        )
        print(
            "Rocket Center of Dry Mass - Nozzle Exit: "
            + "{:.3f} m".format(
                abs(
                    self.rocket.center_of_dry_mass_position
                    - self.rocket.nozzle_position
                )
            )
        )
        print(
            "Rocket Center of Dry Mass - Center of Propellant Mass: "
            + "{:.3f} m".format(
                abs(
                    self.rocket.center_of_propellant_position(0)
                    - self.rocket.center_of_dry_mass_position
                )
            )
        )
        print(
            "Rocket Center of Mass - Rocket Loaded Center of Mass: "
            + "{:.3f} m\n".format(
                abs(
                    self.rocket.center_of_mass(0)
                    - self.rocket.center_of_dry_mass_position
                )
            )
        )

        return None

    def rocket_aerodynamics_quantities(self):
        """Print rocket aerodynamics quantities.

        Returns
        -------
        None
        """
        print("\nAerodynamics Lift Coefficient Derivatives\n")
        for surface, position in self.rocket.aerodynamic_surfaces:
            name = surface.name
            print(
                name
                + " Lift Coefficient Derivative: {:.3f}".format(surface.clalpha(0))
                + "/rad"
            )

        print("\nCenter of Pressure\n")
        for surface, position in self.rocket.aerodynamic_surfaces:
            name = surface.name
            cpz = surface.cp[2]  # relative to the user defined coordinate system
            print(
                name
                + " Center of Pressure position: {:.3f}".format(
                    position - self.rocket._csys * cpz
                )
                + " m"
            )
        print("\nStability\n")
        print(
            f"Center of Mass position (time=0): {self.rocket.center_of_mass(0):.3f} m"
        )
        print(
            "Initial Static Margin (mach=0, time=0): "
            + "{:.3f}".format(self.rocket.static_margin(0))
            + " c"
        )
        print(
            "Final Static Margin (mach=0, time=burn_out): "
            + "{:.3f}".format(
                self.rocket.static_margin(self.rocket.motor.burn_out_time)
            )
            + " c"
        )
        print(
            "Rocket Center of Mass (time=0) - Center of Pressure (mach=0): "
            + "{:.3f}".format(
                abs(self.rocket.center_of_mass(0) - self.rocket.cp_position(0))
            )
            + " m\n"
        )

        return None

    def parachute_data(self):
        """Print parachute data.

        Returns
        -------
        None
        """
        for chute in self.rocket.parachutes:
            chute.all_info()
        return None

    def all(self):
        """Prints all print methods about the Environment.

        Returns
        -------
        None
        """
        # Print inertia details
        self.inertia_details()

        # Print rocket geometrical parameters
        self.rocket_geometrical_parameters()

        # Print rocket aerodynamics quantities
        self.rocket_aerodynamics_quantities()

        # Print parachute data
        self.parachute_data()

        return None
