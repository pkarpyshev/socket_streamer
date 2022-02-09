CPP = g++

I2C_INCLUDES = -I /usr/include -I /usr/include/i2c -I ./include

client:
	$(CPP) -o client.o src/client.cpp $(CPP_INCLUDES) $(OPENCV_DEPS)
server:
	cd ../.. && catkin_make --only-pkg-with-deps socket_streamer

camera:
	cd ../.. && catkin_make --only-pkg-with-deps socket_streamer --make-args -j2 --cmake-args -O3

imu:
	cd ../.. && catkin_make --only-pkg-with-deps socket_streamer --make-args -j2 --cmake-args -O3

all:
	make client
	make server