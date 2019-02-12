TARGETS = tcx ctlr

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
CFLAGS = -g -O2 -Wall -Iutil 

util/util_sdl.o: CFLAGS += $(shell sdl2-config --cflags)
ctlr.c: CFLAGS += -I/usr/local/include/libpololu-tic-1

SRC_TCX = main.c \
          sky.c \
          tele.c \
          util/util_sdl.c \
          util/util_sdl_predefined_panes.c \
          util/util_jpeg.c \
          util/util_png.c \
          util/util_misc.c
SRC_CTLR = ctlr.c \
           util/util_misc.c

OBJ_TCX=$(SRC_TCX:.c=.o)
OBJ_CTLR=$(SRC_CTLR:.c=.o)

DEP=$(SRC_TCX:.c=.d) $(SRC_CTLR:.c=.d)

#
# build rules
#

all: $(TARGETS)

tcx: $(OBJ_TCX) 
	$(CC) -pthread -lrt -lm -lreadline -lpng -ljpeg -lSDL2 -lSDL2_ttf -lSDL2_mixer -o $@ $(OBJ_TCX)

ctlr: $(OBJ_CTLR)
	$(CC) -pthread -lrt -lm -lreadline -lpololu-tic-1 -o $@ $(OBJ_CTLR)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_TCX) $(OBJ_CTL) $(DEP)

