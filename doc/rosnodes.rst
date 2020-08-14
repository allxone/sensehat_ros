ROS nodes
=========

sensehat_ros
------------
ROS node hosting the Sense HAT interface class :py:class:`sensehat_ros.host.Host`


Parameters
^^^^^^^^^^
Check :py:class:`sensehat_ros.host.Host` class constructur parameters.


Publishes
^^^^^^^^^

* \/Environmental
    type = `sensehat_ros/Environmental`_

    Sense HAT Environmental sensors data (i.e. humidity, temperature and pressure).

* \/IMU

    type = `sensehat_ros/IMU`_

    Sense HAT IMU sensor data (e.g. pitch, roll, yaw).

* \/Stick

    type = `sensehat_ros/Stick`_

    Sense HAT Stick events.


.. _`sensehat_ros/Environmental`: msg/Environmental.html
.. _`sensehat_ros/IMU`: msg/IMU.html
.. _`sensehat_ros/Stick`: msg/Stick.html


Messages and Services
^^^^^^^^^^^^^^^^^^^^^

Check `Msg/Srv API`_ 

.. _`Msg/Srv API`: index-msg.html