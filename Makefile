#OBJS specifies which files to compile as part of the project
OBJS = z80.c 

#CC specifies which compiler we're using
CC = gcc

#COMPILER_FLAGS specifies the additional compilation options we're using
# -w suppresses all warnings
COMPILER_FLAGS = -ggdb `pkg-config --cflags --libs gtk+-3.0` 

#LINKER_FLAGS specifies the libraries we're linking against
LINKER_FLAGS = -lSDL2 -lSDL2_mixer `pkg-config --libs gtk+-3.0`

#OBJ_NAME specifies the name of our exectuable
OBJ_NAME = z80_emu

#This is the target that compiles our executable
all : $(OBJS)
	$(CC) $(OBJS) $(COMPILER_FLAGS) $(LINKER_FLAGS) -o $(OBJ_NAME)

.c.o:
	${CC} ${CFLAGS} -c $<

