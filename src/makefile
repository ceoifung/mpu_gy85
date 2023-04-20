INC_DIR = ../include
CFLAGS = -Wall -std=c++11 $(patsubst %, -I%, $(INC_DIR))

all:
	g++ $(CFLAGS) -o gy85.out testApp.cpp I2cPort.cpp Gy85.cpp

# acc:
# 	g++ -o out main.cpp Adxl345.cpp 
clean:
	rm -rf gy85.out