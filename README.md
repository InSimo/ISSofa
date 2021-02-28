[![SOFA, Simulation Open-Framework Architecture](https://www.sofa-framework.org/wp-content/uploads/2013/01/SOFA_LOGO_ORANGE_2-normal.png)](https://www.sofa-framework.org/)
<br/>
     (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo


**This branch contains a version of SOFA used and maintained by [InSimo](https://www.insimo.com).**  
It is optimized for the use case of interactive biomechanical simulations.  
It is available publicly in order to facilitate contributions of improvements such as multithreading and matrix assembly back to the [main open-source version](https://github.com/sofa-framework/sofa).

## Introduction

SOFA is an open source framework primarily targeted at real-time simulation, 
with an emphasis on medical simulation.  
It is mainly intended for the research community to help foster newer 
algorithms, but can also be used as an efficient prototyping tool.  
SOFA's advanced software architecture allows:  
(1) the creation of complex and evolving simulations by combining new algorithms
    with existing algorithms;  
(2) the modification of key parameters of the simulation  such as deformable
    behavior, surface representation, solvers, constraints, collision algorithm,
    etc. by simply editing an XML file;  
(3) the synthesis of complex models from simpler ones using a scene-graph
    description;  
(4) the efficient simulation of the dynamics of interacting objects using
    abstract equation solvers; and  
(5) the comparison of various algorithms available in SOFA. 

## Installation

This version is meant to be included within a larger CMake project.  
See the [ISSofaDemo](https://github.com/InSimo/ISSofaDemo) repository for an example project with dependencies and build instructions.


## Information

### Authors
See [Authors.txt](https://github.com/InSimo/sofa/blob/issofa/Authors.txt)

### License
SOFA is LGPL, except:
- applications/projects (GPL)
- applications/tutorials (GPL)
- sub-directories with a license file specifying a different license

LGPL refers to the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2.1 of the License, or (at your option) any later 
version.

GPL refers to the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

### Contact information
contact@insimo.com
