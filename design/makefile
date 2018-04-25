.PHONY: all
all: main

.PHONY: main
main: main.cpp
	g++ main.cpp -o main -O0 -std=c++11 -W

.PHONY: clean
clean:
	rm -rf main

.PHONY: run
run:
	make
	./main

