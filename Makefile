CPP = g++

client: client.cpp
	$(CPP) -o client.o client.cpp -I /usr/local/include -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc

server: server.cpp
	$(CPP) -o server.o server.cpp -I /usr/local/include -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc

all:
	make server
	make client