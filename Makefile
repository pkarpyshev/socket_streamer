CPP = g++

client: client.cpp
	$(CPP) -o client.o client.cpp -I /usr/local/include -lopencv_core -lopencv_highgui -lopencv_videoio

server: server.cpp
	$(CPP) -o server.o server.cpp
