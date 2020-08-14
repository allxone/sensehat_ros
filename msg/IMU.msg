# Hold data extracted from the Sense HAT IMU sensor.
# Fields x, y and z assume different meaning depending on the selected mode.
# Modes ending with "_rpy" assign (roll, pitch, yaw) values to (x, y, z). 
Header header
# Indicates the API function used to extract data from the IMU. Valid values are:
#   * get_orientation_radians_rpy
#   * get_orientation_degrees_rpy
#   * get_compass_raw_xyz
#   * get_gyroscope_rpy
#   * get_gyroscope_raw_xyz
#   * get_accelerometer_rpy
#   * get_accelerometer_raw_xyzstring mode
string mode
float64 x
float64 y
float64 z