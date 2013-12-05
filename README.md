Steady State Corner Simulator
==========================

Steady State Corner Simulator v0.1

The Steady State Corner Simulator gives the best configuration for a vehicle based on the different setup options, a steady state study in corner and a basic tire model. This Steady State Corner Simulator returns which setup
configuration gets the biggest lateral force out of the tires. The tire model is a function of the vertical load of every tire.

To make this calculations I had to declare a set of initial conditions which are a vehicle speed – in order to calculate the down force –, and the lateral acceleration – so that I can calculate the lateral load transfer.

Since the vehicle speed and the lateral acceleration will depend on the maximum grip given by the tires (result of the simulator), these initial conditions will have to be changed for the result of the first iteration.
