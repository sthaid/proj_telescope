TARGETS = controller

# to read profiling outut file gmon.out:
#   gprof controller gmon.out

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
# CFLAGS_PROFILING = -pg
CFLAGS_SDL2 = $(shell sdl2-config --cflags)
CFLAGS = -g -O2 -Wall -Iutil $(CFLAGS_PROFILING) $(CFLAGS_SDL2)

SRC_CONTROLLER = main.c \
                 sky.c \
                 util/util_sdl.c \
                 util/util_sdl_predefined_panes.c \
                 util/util_jpeg.c \
                 util/util_png.c \
                 util/util_misc.c

OBJ_CONTROLLER=$(SRC_CONTROLLER:.c=.o)

DEP=$(SRC_CONTROLLER:.c=.d)

#
# build rules
#

all: $(TARGETS)

controller: $(OBJ_CONTROLLER) 
	$(CC) -pthread -lrt -lm -lreadline -lpng -ljpeg -lSDL2 -lSDL2_ttf -lSDL2_mixer $(CFLAGS_PROFILING) \
              -o $@ $(OBJ_CONTROLLER)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_CONTROLLER) $(DEP) gmon.out

