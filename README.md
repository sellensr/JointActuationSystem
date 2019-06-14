# JointActuationSystem
Details of the design of an Actuation System for wrist motion simulation as detailed in the master's thesis by Matthew Pearson titled: An Open-Source Device for Tendon Actuated Wrist Motion. 

Further details on the design, manufacture and testing are presented in the thesis found in this repository or [THESIS LINK]

Code original to the thesis is left intact in /Code/AP Source Code and /Code/Aurora AMP Source Code. Current development is taking place in /Code/MP-Sim_AP and Qualisys_AMP

## Generalization for other joints

Original coordinates and angle naming (only two angles, flexion-extension and radial-ulnar) were specific to a system of 4 actuators and the wrist joint. Additions to the /Code structure generalize to 3 x-y-z angles within a right hand coordinate system with y positive distally and z positive vertically upwards. Rotation about the x axis thus corresponds to flexion-extension and rotation about the z axis to radial-ulnar deviation in the original work on the wrist. 

2019-04-26: Newer code for MP-Sim starts with an alpha version deceptively labelled as v2.0 to distinguish it from the original work by Matt Pearson. It is purely developmental for the moment. 
