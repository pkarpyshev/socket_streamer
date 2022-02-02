client:
	$(CPP) -o client.o src/client.cpp $(CPP_INCLUDES) $(OPENCV_DEPS)
server:
	cd ../.. && catkin_make

all:
	make client
	make server