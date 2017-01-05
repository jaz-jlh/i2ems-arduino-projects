#Introduction to Electromechanical Systems (I2EMS) Projects - Arduino

This repository contains code from the two projects in SYS 2048 I2EMS.

**Line Following Robot**

A built-from-scratch, line-following, delivery robot.
* Uses reflectance sensors as input to a proportional controller for line following
* Uses state machine to navigate course
* Works in conjunction with PHP and SQL to read and update a database
* Uses servo to drop off small packages

**Smart Greenhouse**
A internet-connected automated desktop greenhouse.
* Measures temperature and records to database through PHP and SQL
* Controls temperature through "bang-bang" control of heater and opening of greenhouse lid (hysteresis in control to prevent rapid switching)
* Displays current status on LCD