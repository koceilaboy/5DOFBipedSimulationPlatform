# 5DOFBipedSimulationPlatform


This file provides a simulation platform for a 5 DOF biped robot. 

It is yet to be refined, so as of now, you will probably find many inefficiencies in the code.

However, this should outline the main ideas of what is needed in the development of walking gait for a bipedal robot.

The models were taken from: 

"Humanoid Walking Robot: Modeling, Inverse Dynamics, and Gain Scheduling Control" by Elvedin Kljuno and Robert L. Williams II
Can be found on https://www.hindawi.com/journals/jr/2010/278597/


Note to avoid design complexity, feedback linearization controllers were used to control the systems. Also, there is no
input torques restrictions by the joints, and the ground, thus assuming perfect world conditions. 

If you are looking to apply this algorithm to a biped, to maintain stability, check the generated inputs and make sure that the torques of your 
motors can handle the input of the controller.

Enjoy!
-KC
