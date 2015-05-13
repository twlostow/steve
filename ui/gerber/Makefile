OBJS = 	decompose.o
	shape_win.o
	
CXXFLAGS = -I../../include `pkg-config sdl --cflags` -g -I../../pcbnew

all:	$(OBJS)
			g++ -o test_shapes $(OBJS) `pkg-config sdl --libs` -lGL -lSDL_gfx

clean:
	rm -f test_shapes $(OBJS)