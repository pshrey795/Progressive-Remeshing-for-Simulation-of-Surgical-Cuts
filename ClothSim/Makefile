CC=g++
CFLAGS= -lGL -lGLU -lglfw

run: simulation
	./simulation

simulation: main.cpp
	$(CC) -o simulation main.cpp $(CFLAGS)

clean:
	rm -f simulation
