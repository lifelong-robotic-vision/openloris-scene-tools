rosbag filter $1 ${1%.bag}-filtered.bag \
    "topic=='/device_0/sensor_1/Color_0/image/data' \
    or topic=='/device_0/sensor_1/Color_0/image/metadata' \
    or topic=='/device_0/sensor_0/Depth_0/image/data' \
    or topic=='/device_0/sensor_0/Depth_0/image/metadata' \
    or topic=='/device_0/sensor_2/Accel_0/imu/data' \
    or topic=='/device_0/sensor_2/Accel_0/imu/metadata' \
    or topic=='/device_0/sensor_2/Gyro_0/imu/data' \
    or topic=='/device_0/sensor_2/Gyro_0/imu/metadata'"