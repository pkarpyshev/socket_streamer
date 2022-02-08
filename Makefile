CPP = g++

I2C_INCLUDES = -I /usr/include -I /usr/include/i2c -I ./include

client:
	$(CPP) -o client.o src/client.cpp $(CPP_INCLUDES) $(OPENCV_DEPS)
server:
	cd ../.. && catkin_make

imu:
	$(CPP) -o imu_node.o src/imu_node.cpp $(I2C_INCLUDES) -li2c

all:
	make client
	make server