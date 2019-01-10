TARGETS = tcx tele_ctlr

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
CFLAGS_SDL2 = $(shell sdl2-config --cflags)
CFLAGS = -g -O2 -Wall -Wno-stringop-truncation -Iutil $(CFLAGS_SDL2)

SRC_TCX = main.c \
          sky.c \
          tele.c \
          util/util_sdl.c \
          util/util_sdl_predefined_panes.c \
          util/util_jpeg.c \
          util/util_png.c \
          util/util_misc.c
SRC_TELE_CTLR = \
          tele_ctlr.c \
          util/util_misc.c

OBJ_TCX=$(SRC_TCX:.c=.o)
OBJ_TELE_CTLR=$(SRC_TELE_CTLR:.c=.o)

DEP=$(SRC_TCX:.c=.d) \
    $(SRC_TELE_CTLR:.c=.d) 

#
# build rules
#

all: $(TARGETS)

tcx: $(OBJ_TCX) 
	$(CC) -pthread -lrt -lm -lreadline -lpng -ljpeg -lSDL2 -lSDL2_ttf -lSDL2_mixer \
              -o $@ $(OBJ_TCX)

tele_ctlr: $(OBJ_TELE_CTLR) 
	$(CC) -pthread -lrt -lm -o $@ $(OBJ_TELE_CTLR)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_TCX) $(OBJ_TELE_CTLR) $(DEP) gmon.out

