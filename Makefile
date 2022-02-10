CPP = g++

I2C_INCLUDES = -I /usr/include -I /usr/include/i2c -I ./include

camera:
	cd ../.. && catkin_make --only-pkg-with-deps socket_streamer --make-args -j2 --cmake-args -O3

imu:
	cd ../.. && catkin_make --only-pkg-with-deps socket_streamer --make-args -j2 --cmake-args -O3

all:
	make camera
	make imu