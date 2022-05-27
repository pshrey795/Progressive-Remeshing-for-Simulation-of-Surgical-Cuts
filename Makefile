CC = g++
CFLAGS = -lGL -lGLU -lglfw
MODE = 0

run: simulation
	./simulation $(MODE)

simulation: main.cpp mesh.hpp half_edge.hpp
	$(CC) -o simulation main.cpp $(CFLAGS)

clean:
	rm -f simulation
