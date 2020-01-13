all:
	g++ example.cpp ../src/VL6180X.cpp -I../src -li2c -lJetsonGPIO -o example
