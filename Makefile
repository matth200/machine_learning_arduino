.PHONY: all clean clean-all

GCC = g++
FLAGS = -lstdc++fs -lstdc++ -std=c++17

all: bin/extract_nn

bin/extract_nn: bin/extract_neuralnetwork.o bin/m_learning.o
	$(GCC) -o $@ $^ $(FLAGS)

bin/extract_neuralnetwork.o: extract_neuralnetwork.cpp
	$(GCC) -o $@ -c $< $(FLAGS)

bin/m_learning.o: src/m_learning.cpp src/m_learning.h
	$(GCC) -o $@ -c $< $(FLAGS)

clean:
	rm -f bin/*.o

clean-all: clean
	rm -f bin/extract_nn
