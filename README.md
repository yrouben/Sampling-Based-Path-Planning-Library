# Sampling-Based-Path-Planning-Library


This library provides 2-D implementations of the PRM, RRT, RRT* and Informed-RRT* algorithms for sampling based path planning.
A simulator of sorts is provided in the "Sample Based Path Planning - Simulator.ipynb" file.

The PRM algorithm is implemented in the "PRMPlanner.py" file.
The RRT family of algorithms are implemented in the "RRTFamilyOfPlanners.py" file.

Running the simulator requires the ability to run IPython notebooks (i.e. jupyter).
Additionally, this software requires the installation of several python libraries, namely:

	- shapely
	- numpy
	- yaml
	- matplotlib
	- descartes
	- networkx
	- bisect