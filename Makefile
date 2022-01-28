CPP = g++
CPPFLAGS=-I

client: client.cpp
	$(CPP) -o client.o client.cpp

server: server.cpp
	$(CPP) -o server.o server.cpp

all: client.cpp server.cpp
	$(CPP) all -o client.o server.o