Header header

# CarSim's AVz (deg/s):
# Vehicle yaw (body-fixed) - 
# The [sz] component of the angular velocity of the sprung mass of the vehicle.
float32 yaw_rate

# CarSim's AAz (translated rad/s^2 -> deg/s^2):
# "Vehicle yaw (body-fixed) -
# The [sz] component of the angular acceleration of the sprung mass of the vehicle."
float32 yaw_accel

# CarSim's Ax (translated g's -> m/s^2):
# "Longitudinal acceleration -
# The [x] component of the acceleration of the instant CG of the vehicle."
float32 long_accel

# CarSim's Ay (translated g's -> m/s^2):
# "Lateral acceleration -
# The [y] component of the acceleration of the instant CG of the vehicle."
float32 lat_accel
