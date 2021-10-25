# VulcanPath

Welcome to VulcanPath! This is a small library that I created in python that does some simple path following, spline interpolation, and trajectory creation. It is written with pygame and has a full simulator built in for the trajectory following. Currently, the spline trajectory alg is too slow to run on the robot using pyserial and arduinos.

// gif of pure pursuit alg working on trakr bot

## How this project started
I initially wanted a robot to test my FTC path follower algorithms on, so I 3d printed this tiny robot with 2 wheels that drifts a lot with simple localization and was overall pretty rudimentary in its construction.
The idea was to have 2 arduinos with radio transcievers that pass serial data back and forth, much like the way FTC robots do. The main difference with this approach, however, is that the robot controller only controls the robots wheel velocities and spits out encoder values to the driver station. Then, the driver station arduino passes all that data to a much faster computer to do all of the auton calculations, and motor powers are sent back to the robot controller.

//picture of old bot if i have one

This iteration never made it past the development stage, so it never ran autonomously. I simply needed a better and more reliable drivebase.

## Deconstructing an old RC Rover
While designing my own, better drivebase was probably the smartest option, I wasn't building this robot with mechanical resilience in mind. I needed something that worked well enough for me to test my software. Naturally, I turned to my closet to find an old RC "spy car" that I had laying around.
These things are super rad, and you can read more about how people used to hack into these toys and how the creators of it were actually really nerdy and rad, but the support and interest for this product died quickly and its really hard to find any new and useful information.
Needless to say I took all of the electronics out, hammered the motor shafts to get the magnetic encoders on, and put my own electronics in

// pictures of trakr bot before and after

Localization with this worked much better, and I have been using this bot to test all sorts of movement algs since I built it.
