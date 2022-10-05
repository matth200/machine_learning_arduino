.PHONY: all clean clean-all

GCC = g++

all: bin/extract_nn

bin/extract_nn: bin/extract_neuralnetwork.o bin/m_learning.o
	$(GCC) -o $@ $^

bin/extract_neuralnetwork.o: extract_neuralnetwork.cpp
	$(GCC) -o $@ -c $<

bin/m_learning.o: src/m_learning.cpp src/m_learning.h
	$(GCC) -o $@ -c $<

clean:
	rm -f bin/*.o

clean-all: clean
	rm -f bin/extract_nn
