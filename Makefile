CPP = g++
OPENCV_DEPS = -I /usr/local/include -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc
ROS_DEPS = -I /opt/ros/noetic/include

client: client.cpp
	$(CPP) -o client.o client.cpp $(OPENCV_DEPS)
server: server.cpp
	$(CPP) -o server.o server.cpp $(OPENCV_DEPS) $(ROS_DEPS)

all:
	make server
	make client