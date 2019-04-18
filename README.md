# NHVC
Nonholonomic Virtual Constraints and Optimization for Robust Robot Walking

Contact: Brent Griffin (griffb at umich dot edu)

## Overview
This is a copy of the simulation and optimization code used for the control design in "Nonholonomic Virtual Constraints and Gait Optimization for Robust Robot Walking Control" (see pdf) and our IJRR paper.

Get started by running ``./NHVC_testScript.m``, which will run a simulation and video using the first optimized controller.

``./NHVC_optScript.m`` can be used to optimize new control solutions.

## Paper
[Nonholonomic Virtual Constraints and Gait Optimization for Robust Walking Control](http://web.eecs.umich.edu/faculty/grizzle/papers/NonholonomicVirtualConstraints_IJRR_GrGr17.pdf "IJRR Paper")<br />
[Brent A. Griffin](https://www.griffb.com) and [Jessy W. Grizzle](http://web.eecs.umich.edu/faculty/grizzle/index.html)<br />

Please cite our paper if you find it useful for your research.
```
@inproceedings{GrGr17,
  author = {Griffin, Brent and Grizzle, Jessy},
  title ={Nonholonomic virtual constraints and gait optimization for robust walking control},
  journal = {The International Journal of Robotics Research},
  volume = {36},
  number = {8},
  pages = {895-922},
  year = {2017}
}
```

## Method

__Video Demonstration:__ https://youtu.be/81I0H5d0tUM

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/81I0H5d0tUM/0.jpg)](https://www.youtube.com/watch?v=81I0H5d0tUM)

__Energy Efficiency and Benchmark.__ Outdoor experiments set a new precident for walking efficiency by achieving the lowest mechanical cost of transport (MCOT) of any unsupported bipedal robot tested over rough terrain. In addition, this precedent is set at a faster speed than any other bipedal robot efficiency benchmark.
![alt text](https://github.com/griffbr/NHVC/blob/master/figure/MCOT.png "MCOT Benchmark")
<br />

__Nonholonomic Virtual Constraints (NHVC).__ Using nonholonomic outputs, it is possible to implement velocity-based posture regulation that accounts for the full dynamics of the biped, as well as a range of terrain variation.
![alt text](https://github.com/griffbr/NHVC/blob/master/figure/NHVC.png "NHVC")
<br />

__Indoor Terrain.__ MARLO walks over randomly thrown boards.
![alt text](https://github.com/griffbr/NHVC/blob/master/figure/indoor_terrain.png "Indoor Terrain")
<br />

__Outdoor Terrain.__ MARLO walks outdoors using two-contact-point feet (left) and prosthetic feet (right).
![alt text](https://github.com/griffbr/NHVC/blob/master/figure/outdoor_terrain.png "Outdoor Terrain")
<br />

Apprendix A in ``./additional_reference/dissertationGriffin160727.pdf`` provides a helpful introduction to 3D walking concepts.

## MEX Files

We have built all of the mex files for Windows and Linux (some for macOS). To build new mex files, use the MATLAB debugging tool on the line where the mex file is called and run the corresponding codegen lines in ``./simulationFiles/mexCodeGen/codegenScript.m``.

## Use

This code is available for non-commercial research purposes only.