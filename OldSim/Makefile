CC=g++
CFLAGS= -lGL -lGLU -lglut

sources=main.cpp sop.h vec3d.h particle.h
objects=$(sources:.cpp=.o)

run: main.cpp
	$(CC) -o simulation main.cpp $(CFLAGS) 

%.o: %.cpp 
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o
	rm -f simulation