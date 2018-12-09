TARGETS = tcx

# to read profiling outut file gmon.out:
#   gprof tcx gmon.out

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
# CFLAGS_PROFILING = -pg
CFLAGS_SDL2 = $(shell sdl2-config --cflags)
CFLAGS = -g -O2 -Wall -Iutil $(CFLAGS_PROFILING) $(CFLAGS_SDL2)

SRC_TCX = main.c \
                 sky.c \
                 util/util_sdl.c \
                 util/util_sdl_predefined_panes.c \
                 util/util_jpeg.c \
                 util/util_png.c \
                 util/util_misc.c

OBJ_TCX=$(SRC_TCX:.c=.o)

DEP=$(SRC_TCX:.c=.d)

#
# build rules
#

all: $(TARGETS)

tcx: $(OBJ_TCX) 
	$(CC) -pthread -lrt -lm -lreadline -lpng -ljpeg -lSDL2 -lSDL2_ttf -lSDL2_mixer $(CFLAGS_PROFILING) \
              -o $@ $(OBJ_TCX)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_TCX) $(DEP) gmon.out

