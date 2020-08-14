..
    Incorporation of Changelogs into Package Source Tree: REP 132
    https://www.ros.org/reps/rep-0132.html

0.1.0 (2020-08-15)
------------------
* Initial release
* environmental data publishing (humidity, temperature, pressure)
* IMU sensor data and stick events publishing
* LED Matrix control services
* median smoothing for environmental data
* nosetests to check Python modules (Sense HAT mocked)
* rostests to test ROS topic publishing and ROS services (Sense HAT emulated via `sense-emu <https://pypi.org/project/sense-emu/>`_ with pre-recorded sensor data)
* tested on Melodic (Python 2) and Noetic (Python 3)