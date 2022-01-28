CPP = g++
CPP_INCLUDES = -I ./include -I /usr/local/include -I /opt/ros/noetic/include
OPENCV_DEPS = -L /usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lopencv_video

client:
	$(CPP) -o client.o src/client.cpp $(CPP_INCLUDES) $(OPENCV_DEPS)
server:
	$(CPP) -o server.o src/server.cpp $(CPP_INCLUDES) $(OPENCV_DEPS) $(ROS_DEPS)

all:
	make server
	make client