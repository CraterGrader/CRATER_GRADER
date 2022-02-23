CXX = g++
INCLUDES = -I ../../include
CPPFLAGS = -Wall

SOURCES = main.cpp

all: firmware_update

clean:
	rm -f main.o
	rm -f firmware_update
	cd ../.. && make clean

firmware_update: main.o ../../build/bin/libvncxx.a
	$(CXX) -o firmware_update main.o -L ../../build/bin -lvncxx -lpthread

../../build/bin/libvncxx.a:
	cd ../.. && make

main.o: main.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

