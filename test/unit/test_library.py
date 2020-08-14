#!/usr/bin/env python
#
# Copyright 2020, Stefano Dell'Orto
# License: BSD
#   https://raw.githubusercontent.com/allxone/sensehat_ros/master/LICENSE
#
from __future__ import print_function
PKG='sensehat_ros'

# Level 1: Library unit test
# (cfr. http://wiki.ros.org/action/show/Quality/Tutorials/UnitTesting)

import sys, os
import time
from datetime import datetime, timedelta
import unittest

try:
    from unittest.mock import patch, MagicMock
except ImportError:
    from mock import patch, MagicMock

sys.modules['sense_hat'] = MagicMock()
from sensehat_ros.host import Host


class TestHost(unittest.TestCase):

    def setUp(self):
        self.calibration = {
            'humidity': 10.0,
            'temperature_from_humidity': 20.0,
            'temperature_from_pressure': 30.0,
            'pressure': 40.0}

    def tearDown(self):
        pass

    def get_host(self, smoothing):
        return Host(
            stick_publishing = False,
            environmental_publishing = False,
            imu_publishing = False,
            calibration = self.calibration,
            smoothing = smoothing,
            register_services = False)


    @patch("sensehat_ros.host.SenseHat")
    def test_get_measureSmoothed(self, sense):
        smoothing = 5

        # Configure mocked hardware
        senseObj = sense.return_value   # to track instance methods we need to target the instance mock, not the class
        senseObj.get_humidity.side_effect = [float(i) for i in range(smoothing)]

        # Create a patched Host without requiring the SenseHat hardware
        host = self.get_host(smoothing=smoothing)

        # Read sensor values
        vals = [host._get_measure("humidity") for i in range(smoothing)]
        smoothedVals = [i/2.0 + self.calibration["humidity"] for i in range(smoothing)]

        # Assert
        self.assertEqual(senseObj.get_humidity.call_count, smoothing)
        self.assertEqual(vals, smoothedVals)


    @patch("sensehat_ros.host.SenseHat")
    def test_get_imu(self, sense):

        values = {
            'rpy': {u'roll': 1.0, u'pitch': 2.0, u'yaw': 3.0},
            'xyz': {u'x': 1.0, u'y': 2.0, u'z': 3.0},
        }
        modeValues = { mode: values[mode[-3:]] for mode in [
            'get_orientation_radians_rpy',
            'get_orientation_degrees_rpy',
            'get_compass_raw_xyz',
            'get_gyroscope_rpy',
            'get_gyroscope_raw_xyz',
            'get_accelerometer_rpy',
            'get_accelerometer_raw_xyz']}

        # Configure mocked hardware
        senseObj = sense.return_value   # to track instance methods we need to target the instance mock, not the class
        senseObj.get_orientation_degrees.return_value = modeValues['get_orientation_degrees_rpy']
        senseObj.get_orientation_radians.return_value = modeValues['get_orientation_radians_rpy']
        senseObj.get_compass_raw.return_value = modeValues['get_compass_raw_xyz']
        senseObj.get_gyroscope.return_value = modeValues['get_gyroscope_rpy']
        senseObj.get_gyroscope_raw.return_value = modeValues['get_gyroscope_raw_xyz']
        senseObj.get_accelerometer.return_value = modeValues['get_accelerometer_rpy']
        senseObj.get_accelerometer_raw.return_value = modeValues['get_accelerometer_raw_xyz']

        # Create a patched Host without requiring the SenseHat hardware
        host = self.get_host(smoothing=0)
        
        # Read IMU values for each supported mode
        for mode in host.imu_modes.keys():
            imu = host._get_imu(mode, datetime.now())
            self.assertDictEqual({'x': imu.x, 'y': imu.y, 'z': imu.z}, values['xyz'])


    @patch("sensehat_ros.host.SenseHat")
    def test_get_environmental(self, sense):
        # Configure mocked hardware
        senseObj = sense.return_value   # to track instance methods we need to target the instance mock, not the class
        senseObj.get_humidity.return_value = 1.0
        senseObj.get_temperature_from_humidity.return_value = 2.0
        senseObj.get_temperature_from_pressure.return_value = 3.0
        senseObj.get_pressure.return_value = 4.0

        # Create a patched Host without requiring the SenseHat hardware
        host = self.get_host(smoothing=0)

        # Read sensor values
        environmental = host._get_environmental(datetime.now())
        senseObj.get_humidity.assert_called_once()
        senseObj.get_temperature_from_humidity.assert_called_once()
        senseObj.get_temperature_from_pressure.assert_called_once()
        senseObj.get_pressure.assert_called_once()

        # Assert
        self.assertEqual(environmental.humidity, self.calibration['humidity'] + 1)
        self.assertEqual(environmental.temperature_from_humidity, self.calibration['temperature_from_humidity'] + 2)
        self.assertEqual(environmental.temperature_from_pressure, self.calibration['temperature_from_pressure'] + 3)
        self.assertEqual(environmental.pressure, self.calibration['pressure'] + 4)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'TestHost', TestHost)