.. _flight-usage:

Flight Class Usage
==================

The Flight class receives ``Rocket`` and ``Environment`` instances and then
creates the flight simulation. Note that the Rocket and the Environment are
required beforehand.

Procedure
---------

Defining a Rocket in RocketPy is simple and requires a few steps:

1. Define Flight object with:

    1.1. Rocket object;

    1.2. Environment object;

    1.3. Launching Rail's inclination from the ground up in degrees;

    1.4. Launching Rail's heading in degrees. (ex.: North = 0, East = 90)

    1.5. Launching Rail's total length in meters

2. See results.

Basic example
-------------

Let's create a simple simulation:

We will define a simple Rocket and Environment for the example as these are 
required on the Flight class, so make sure you have defined yours beforehand.

Let's start by importing the classes we will need:

.. jupyter-execute::

    from rocketpy import Rocket, Environment, Flight

    import numpy as np

.. jupyter-execute::
    :hide-code:
    
    my_rocket = Rocket(
        radius=127 / 2000,
        mass=14.426,
        inertia=(6.321, 6.321, 0.034),
        power_off_drag="../data/calisto/powerOffDragCurve.csv",
        power_on_drag="../data/calisto/powerOnDragCurve.csv",
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )

    my_env = Environment(
        latitude=32.990254, 
        longitude=-106.974998, 
        elevation=1400
    )

.. seealso::

    For more information on the :class:`rocketpy.Rocket` class initialization,
    see \ :meth:`rocketpy.Rocket.__init__` method documentation.

.. seealso::

    For more information on the :class:`rocketpy.Environment` class initialization, \
    see :meth:`rocketpy.Environment.__init__` section.

Now, finally the flight simulation is as simple as following: 

.. jupyter-execute::

    my_flight = Flight(
        rocket=my_rocket,   # Rocket object
        environment=my_env, # Environment object
        inclination= 80,    # Inclination in degrees
        heading= 90,        # Heading in degrees  
        rail_length= 5.6,   # Rail length in meters
    )

That's enough to create a simulation already!


Advanced settings
-----------------

In this section we dive into the details and possibilities available when 
setting up your simulation.
First, let's see the complete explicit function. 

.. note::

    The new values presented here are all optional. It's not necessary to explicit \
    determinate them, but can be changed to fine tune your simulation for your needs. 


.. jupyter-execute::

    my_flight2 = Flight(
        rocket=my_rocket,
        environment=my_env,
        inclination= 80,   
        heading= 90,       
        rail_length= 5.6, 
        initial_solution=None,
        terminate_on_apogee=False,
        max_time=600,
        max_time_step=np.inf,
        min_time_step=0,
        rtol=1e-6,
        atol=6 * [1e-3] + 4 * [1e-6] + 3 * [1e-3],
        time_overshoot=True,
        verbose=False,
        name="Flight",
        equations_of_motion="standard" 
    )

I know it's a lot, but don't abort the mission just yet! Let's see what each 
part means.

The first itens were explained before, so let's see the new ones.

.. tip::

    Setting the ``equations_of_motion`` argument to ``solid_propulsion`` will \
    optimize the simulation for solid propulsion rockets. This will make the \
    simulation run twice as faster as the standard equations of motion. However, \
    it will only work for solid propulsion rockets.

Entries
~~~~~~~

- ``initial_solution`` - (array) it can be used to define a specific initial estate.
If the simulation starts with a stopped rocket on the launch rail, it won't be necessary to 
specify this entry. Otherwise, if the user wants to start the simulation in a different
stage of the flight, this entry should be used to define this point.
The array is defined as the example below:

.. code-block:: python

    initial_solution = [
        self.t_initial,
        x_init, y_init, z_init,
        vx_init, vy_init, vz_init,
        e0_init, e1_init, e2_init, e3_init,
        w1_init, w2_init, w3_init
    ]

- ``terminate_on_apogee`` - (boolean) It defines if the simulation should 
stop once the rocket reaches the apogee. By default it's set to False.

- ``max_time`` - (int, float) sets the maximum time in seconds in which the 
the trajectory will be simulated. By default, it's set to 600 seconds. 
**If you use this setting, you should also set the max_time_step.**

- ``max_time_step`` - (int,float) It's the maximum step size used on the integration. By default, it's set to 0.01s

- ``min_time_step`` - (int,float) It's the maximum step size used on the integration. By default, it's set to 0.01s

- ``rtol`` - (float, array) Maximum relative error tolerated on integration. By default it's 1e-03. Here is an example on how one can set it. 

.. code-block:: python

    r_tol = [
        r_tol_x_init, r_tol_y_init, r_tol_z_init,
        r_tol_vx_init, r_tol_vy_init, r_tol_vz_init,
        r_tol_e0_init, r_tol_e1_init, r_tol_e2_init, r_tol_e3_init,
        r_tol_w1_init, r_tol_w2_init, r_tol_w3_init
    ]

- ``atol`` - (float, array) Maximum absolute error tolerated on integration.

.. code-block:: python
    
    a_tol = [
        a_tol_x_init, a_tol_y_init, a_tol_z_init,
        a_tol_vx_init, a_tol_vy_init, a_tol_vz_init,
        a_tol_e0_init, a_tol_e1_init, a_tol_e2_init, a_tol_e3_init,
        a_tol_w1_init, a_tol_w2_init, a_tol_w3_init
    ]

.. seealso:: 
    
    TODO: Adds scipy docs ref

- ``time_overshoot`` - (boolean) If True, decouples ODE time step from parachute trigger functions sampling rate. The time steps can overshoot the necessary trigger function evaluation points and then interpolation is used to calculate them and feed the triggers. Can greatly improve run time in some cases. Default is True.

- ``verbose`` - (boolean) Activates verbose mode. Default is False. It gives you more detail while running the simulation, which can help identify problems, but does not affect the simulation.

- ``name`` - (String) You can name your flight so it's possible to identify it later on.

- ``equations_of_motion`` - (String) The user can choose between "standard" and
"solid_propulsion". The standard equations works on every scenario. The solid_propulsion, on the other hand
only works for solid propulsion rockets, but are optimized for this scenario. 


Visualizing Results
-------------------

The first and most direct way of getting the results is through the
:meth:`rocketpy.Flight.all_info` method.
For example:

.. code-block:: python

    my_flight.all_info()


But we know that sometimes you just need one of these results or even want to 
make a personalized result plot for your specific use. 

To help you with that, we will dive into each one of the results option and show 
you how you can get any of those data individually. 

Custom Results
~~~~~~~~~~~~~~

...

Going further
-------------

RocketPy allows for ...

