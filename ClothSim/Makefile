CC=g++
CFLAGS= -lGL -lGLU -lglfw3 -lX11 -lpthread -lXi -ldl
LIBS= `pkg-config --cflags --libs eigen3`

run: simulation
	./simulation

simulation: main.cpp
	$(CC) -o simulation main.cpp $(LIBS) $(CFLAGS)

clean:
	rm -f simulation