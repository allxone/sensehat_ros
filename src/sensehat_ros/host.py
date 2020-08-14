# Copyright 2020, Stefano Dell'Orto
# License: BSD
#   https://raw.githubusercontent.com/allxone/sensehat_ros/master/LICENSE
#
##############################################################################
# Imports
##############################################################################


import collections
from threading import Lock

import rospy
from sensehat_ros.msg import Environmental, IMU, Stick
from sensehat_ros.srv import *

try:
    from sense_hat import SenseHat
except:
    # Fallback to sense-emu if sense-hat is not available (can be used both on x86 and ARM)
    from sense_emu import SenseHat


class Host(object):
    """Main class used by the ROS node supporting dialogue between the ROS framework and the Sense HAT hardware.
        
    :param rotation: control Sense HAT LED matrix orientation. Defaults to 0.
    :type rotation: int, optional
    :param low_light: enable Sense HAT low-light mode. Defaults to False.
    :type low_light: bool, optional
    :param calibration: linear fixing for environmental readings (ex. {"humidity": -20.0} ). Defaults to {}.
    :type calibration: dict, optional
    :param smoothing: number of observations to be used for median smoothing (0 = smoothing disabled). Smoothing is applied only to environmental data (humidity, temperature and pressure). Defaults to 0.
    :type smoothing: int, optional
    :param register_services: control ROS services registration. Defaults to True.
    :type register_services: bool, optional
    :param environmental_publishing: enable automatic publishing of environmental data (rate controlled by environmental_publishing_rate). Defaults to True.
    :type environmental_publishing: bool, optional
    :param environmental_publishing_rate: environmental data publication frequency in seconds. Defaults to 12 (5 times a minute).
    :type environmental_publishing_rate: float, optional
    :param imu_publishing: enable automatic publishing of IMU data (rate controlled by imu_publishing_rate). Defaults to False.
    :type imu_publishing: bool, optional
    :param imu_publishing_mode: specify the Sense HAT API function to be used to get x,y,z/roll,pitch,yaw. Valid values are: get_orientation_radians_rpy, get_orientation_degrees_rpy, get_compass_raw_xyz, get_gyroscope_rpy, get_gyroscope_raw_xyz, get_accelerometer_rpy, get_compass_raw_xyz. Defaults to get_orientation_degrees_rpy.
    :type imu_publishing_mode: string, optional
    :param imu_publishing_rate: IMU data publication frequency in seconds. Defaults to 1 (once a second).
    :type imu_publishing_rate: float, optional
    :param stick_publishing: enable automatic publishing of stick events. Defaults to True.
    :type stick_publishing: bool, optional
    :param stick_sampling: indicates how frequently the Stick is checked for new events. Defaults to 0.2 (5 times a second).
    :type stick_sampling: float, optional
    """

    def __init__(self,
        rotation = 0,
        low_light = False,
        calibration = {},
        smoothing = 0,
        register_services = True,
        environmental_publishing = True,
        environmental_publishing_rate = 12,
        imu_publishing = False,
        imu_publishing_mode = "get_orientation_degrees_rpy",
        imu_publishing_config = "1|1|1",
        imu_publishing_rate = 1,
        stick_publishing = True,
        stick_sampling = 0.2):
        """Constructor method"""

        # Init sensor
        self.sense = SenseHat()
        self.sense.clear(0,0,0)
        self.sense.set_rotation(rotation)
        self.sense.low_light = low_light
        self.measures = {
            'humidity': self.sense.get_humidity,
            'temperature_from_humidity': self.sense.get_temperature_from_humidity,
            'temperature_from_pressure': self.sense.get_temperature_from_pressure,
            'pressure': self.sense.get_pressure,
            'compass': self.sense.get_compass,
        }
        self.imu_modes = {
            'get_orientation_radians_rpy': self.sense.get_orientation_radians,
            'get_orientation_degrees_rpy': self.sense.get_orientation_degrees,
            'get_compass_raw_xyz': self.sense.get_compass_raw,
            'get_gyroscope_rpy': self.sense.get_gyroscope,
            'get_gyroscope_raw_xyz': self.sense.get_gyroscope_raw,
            'get_accelerometer_rpy': self.sense.get_accelerometer,
            'get_accelerometer_raw_xyz': self.sense.get_accelerometer_raw,
        }

        # Init parameters
        self.stick_publishing = stick_publishing
        self.environmental_publishing = environmental_publishing
        self.imu_publishing = imu_publishing
        self.imu_publishing_mode = imu_publishing_mode
        self.stick_sampling = stick_sampling
        self.environmental_publishing_rate = environmental_publishing_rate
        self.imu_publishing_rate = imu_publishing_rate
        self.calibration = calibration
        self.register_services = register_services
        self.smoothing = smoothing
        self.history = {}
        for measure in self.measures:
            self.history[measure] = collections.deque([], maxlen = smoothing if smoothing > 0 else None) if smoothing >= 0 else None

        # Init Lock to serialize access to the LED Matrix
        self.display_lock = Lock()

        # Register publishers
        if self.stick_publishing:
            self.stick_pub = rospy.Publisher("Stick", Stick, queue_size=10)
        if self.environmental_publishing:
            self.environmental_pub = rospy.Publisher("Environmental", Environmental, queue_size=10)
        if self.imu_publishing:
            self.sense.set_imu_config(*[i=="1" for i in imu_publishing_config.split("|")])
            self.imu_pub = rospy.Publisher("IMU", IMU, queue_size=10)

        # Register services
        if self.register_services:
            self.getEnvironmentalService = rospy.Service("GetEnvironmental", GetEnvironmental, self.getEnvironmental)
            self.getHumidityService = rospy.Service("GetHumidity", GetHumidity, self.getHumidity)
            self.getTemperatureService = rospy.Service("GetTemperature", GetTemperature, self.getTemperature)
            self.getPressureService = rospy.Service("GetPressure", GetPressure, self.getPressure)
            self.getIMUService = rospy.Service("GetIMU", GetIMU, self.getIMU)
            self.getCompassService = rospy.Service("GetCompass", GetCompass, self.getCompass)
            self.emulateStick = rospy.Service("EmulateStick", EmulateStick, self.emulateStick)
            self.clearService = rospy.Service("Clear", Clear, self.clear)
            self.setPixelsService = rospy.Service("SetPixels", SetPixels, self.setPixels)
            self.switchLowLightService = rospy.Service("SwitchLowLight", SwitchLowLight, self.switchLowLight)
            self.showLetterService = rospy.Service("ShowLetter", ShowLetter, self.showLetter)
            self.showMessageService = rospy.Service("ShowMessage", ShowMessage, self.showMessage)

        rospy.loginfo("sensehat_ros initialized")


    def __del__(self):
        if self.sense:
            self.sense.clear(0,0,0)

        rospy.loginfo("sensehat_ros destroyed")


##############################################################################
# Private methods
##############################################################################

    def _get_environmental(self, timestamp):
        msg = Environmental()
        msg.header.stamp = timestamp
        msg.humidity = self._get_measure('humidity')
        msg.temperature_from_humidity = self._get_measure('temperature_from_humidity')
        msg.temperature_from_pressure = self._get_measure('temperature_from_pressure')
        msg.pressure = self._get_measure('pressure')
        return msg


    def _get_imu(self, mode, timestamp):
        msg = IMU()
        msg.header.stamp = timestamp
        msg.mode = mode

        imu = self.imu_modes[mode]()
        if mode[-3:] == 'xyz':
            msg.x, msg.y, msg.z = imu['x'], imu['y'], imu['z']
        elif mode[-3:] == 'rpy':
            msg.x, msg.y, msg.z = imu['roll'], imu['pitch'], imu['yaw']

        return msg


    def _get_measure(self, measure, disableSmooting = False):

        def _get_measure_median(smoothing):
            """Return median value from the last <smoothing> elements in the history

            Args:
                smoothing (int): values to be extracted from the history

            Returns:
                double: median value
            """
            lst = [history[-i] for i in range(1, 1 + min(smoothing, len(history)))]
            n = len(lst)
            s = sorted(lst)
            return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

        if not measure in self.measures:
            raise ValueError('Invalid measure %s' % measure)
        
        val = self.measures[measure]() + self.calibration.get(measure, 0.0)
        history = self.history[measure]
        if history is not None:
            history.append(val)

            if self.smoothing > 0 and not disableSmooting:
                val = _get_measure_median(self.smoothing)

        return val


    def _publish_stick(self, event):
        for stick_event in self.sense.stick.get_events():
            stick = Stick(direction=stick_event.direction, action=stick_event.action)
            rospy.logdebug(
                "Publishing Stick (D: %s, A: %s)",
                stick.direction,
                stick.action
            )
            self.stick_pub.publish(stick)
           

    def _publish_environmental(self, event):
        environmental = self._get_environmental(rospy.Time.now())
        rospy.logdebug(
            "Publishing Environmental (H: %s, TH: %s, TP: %s, P: %s)",
            environmental.humidity, 
            environmental.temperature_from_humidity, 
            environmental.temperature_from_pressure,
            environmental.pressure
        )
        self.environmental_pub.publish(environmental)
    

    def _publish_imu(self, event):
        imu = self._get_imu(self.imu_publishing_mode, rospy.Time.now())
        rospy.logdebug(
            "Publishing IMU (Mode: %s, X: %s, Y: %s, Z: %s)",
            imu.mode, imu.x, imu.y, imu.z,
        )
        self.imu_pub.publish(imu)


##############################################################################
# ROS service methods
##############################################################################

    def clear(self, req):
        """ROS service: sets all of the 64 LED matrix pixels to the specified RGB color and waits for the specified amount of seconds"""

        self.display_lock.acquire()
        try:
            self.sense.clear(req.colour)
            rospy.sleep(req.duration)
        finally:
            self.display_lock.release()
        return ClearResponse()


    def switchLowLight(self, req):
        """ROS service: switches on/off the LED matrix \"low light\" mode and returns the current state."""

        self.sense.low_light = not self.sense.low_light
        return SwitchLowLightResponse(low_light = self.sense.low_light)


    def setPixels(self, req):
        """ROS service: sets each of the 64 LED matrix pixels to a specific RGB color and waits for the specified amount of seconds."""

        self.display_lock.acquire()
        try:
            self.sense.set_pixels(list(zip(req.pixels_red, req.pixels_green, req.pixels_blue)))
            rospy.sleep(req.duration)
        finally:
            self.display_lock.release()
        return SetPixelsResponse()


    def showLetter(self, req):
        """ROS service: shows a letter on the LED matrix and waits for the specified amount of seconds."""

        self.display_lock.acquire()
        try:
            self.sense.show_letter(req.text, req.text_colour, req.back_colour)
            rospy.sleep(req.duration)
        finally:
            self.display_lock.release()
        return ShowLetterResponse()


    def showMessage(self, req):
        """ROS service: scrolls a text message from right to left across the LED matrix and at the specified speed, in the specified colour and background colour."""

        self.display_lock.acquire()
        try:
            self.sense.show_message(req.text, req.scroll_speed, req.text_colour, req.back_colour)
        finally:
            self.display_lock.release()
        return ShowMessageResponse()


    def getIMU(self, req):
        """ROS service: queries Sense HAT IMU sensor. Warning: not allowed when imu_publishing=True due to potential set_imu_config interference."""

        if self.imu_publishing:
            raise RuntimeError("Method getIMU not allowed when imu_publishing=True (due to potential set_imu_config interference)")

        imu = self._get_imu(req.mode, rospy.Time.now())
        rospy.logdebug(
            "Sending IMU (Mode: %s, X: %s, Y: %s, Z: %s)",
            imu.mode, imu.x, imu.y, imu.z,
        )
        return imu


    def getCompass(self, req):
        """ROS service: gets the direction of North from the magnetometer in degrees. Warning: not allowed when imu_publishing=True due to potential set_imu_config interference."""

        if self.imu_publishing:
            raise RuntimeError("Method getCompass not allowed when imu_publishing=True due to potential set_imu_config interference")

        compass = self._get_measure('compass', disableSmooting=True)
        rospy.logdebug("Sending Compass (Compass: %s)",compass)
        return compass


    def getHumidity(self, req):
        """ROS service: gets the percentage of relative humidity from the humidity sensor."""

        humidity = self._get_measure('humidity')
        rospy.logdebug("Sending Humidity (H: %s)",humidity)
        return humidity


    def getTemperature(self, req):
        """ROS service: gets the current temperature in degrees Celsius from the humidity sensor."""

        temperature = self._get_measure('temperature_from_humidity')
        rospy.logdebug("Sending Temperature (TH: %s)",temperature)
        return temperature


    def getPressure(self, req):
        """ROS service: gets the current pressure in Millibars from the pressure sensor."""

        pressure = self._get_measure('pressure')
        rospy.logdebug("Sending Pressure (P: %s)",pressure)
        return pressure


    def getEnvironmental(self, req):
        """ROS service: gets the current humidity and temperature from the humidity sensor and temperature and pressure from the pressure sensor."""

        environmental = self._get_environmental(rospy.Time.now())
        rospy.logdebug(
            "Sending Environmental (H: %s, TH: %s, TP: %s, P: %s)",
            environmental.humidity, 
            environmental.temperature_from_humidity, 
            environmental.temperature_from_pressure, 
            environmental.pressure
        )
        return environmental


    def emulateStick(self, req):
        """ROS service: remotely triggers a stick event without interacting with the Stick device."""

        if self.stick_publishing:
            rospy.loginfo(
                "Emulating Stick (D: %s, A: %s)",
                req.direction,
                req.action
            )
            stick = Stick(direction=req.direction, action=req.action)
            self.stick_pub.publish(stick)
        
        return EmulateStickResponse()


##############################################################################
# Run
##############################################################################

    def run(self):
        """Starts configured data publishing (stick, environmental, IMU) and spins waiting for service calls. Triggered by the ROS node after initialization."""

        if self.stick_publishing:
            # Clear buffer
            self.sense.stick.get_events()
            # Start publishing events
            rospy.Timer(rospy.Duration(self.stick_sampling), self._publish_stick)
            rospy.loginfo("sensehat_ros publishing stick")

        if self.environmental_publishing:
            # Start publishing events
            rospy.Timer(rospy.Duration(self.environmental_publishing_rate), self._publish_environmental)
            rospy.loginfo("sensehat_ros publishing environmental")

        if self.imu_publishing:
            # Start publishing events
            rospy.Timer(rospy.Duration(self.imu_publishing_rate), self._publish_imu)
            rospy.loginfo("sensehat_ros publishing IMU")

        rospy.spin()