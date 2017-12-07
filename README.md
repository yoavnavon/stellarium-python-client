# Stellarium-Python-Client

This python desktop aplication allows you to get coordinates from [Stellarium](http://stellarium.org).

First you have to configure in Stellarium a new external telescope as client. Then, every time you press new coordinate for the telescope, those coordinates will appear in the main window of the python app.

The application was intended for comunication with an Arduino to control two motors of an equatorial mount:

- First you have to click three points in the sky in Stellarium, and click *Set Reference*. Then, the app will calculate a coordinates transformation matrix. With these, and the combination of the local coordinates of the telescope with the arduino, you can use the telescope without aligning.
- Then, with the *goto* button, the application calculates de number of steps necesary for the motors.