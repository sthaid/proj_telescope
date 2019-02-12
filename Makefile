TARGETS = tcx ctlr vp

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
CFLAGS = -g -O2 -Wall -Iutil 

util/util_sdl.o: CFLAGS += $(shell sdl2-config --cflags)
ctlr.o: CFLAGS += -I/usr/local/include/libpololu-tic-1 -Wno-unused-result

SRC_TCX  = tcx.c \
           tcx_sky.c \
           tcx_tele.c \
           util_sky.c \
           util/util_sdl.c \
           util/util_sdl_predefined_panes.c \
           util/util_jpeg.c \
           util/util_png.c \
           util/util_misc.c
SRC_CTLR = ctlr.c \
           util/util_misc.c
SRC_VP   = vp.c \
           util_sky.c \
           util/util_misc.c

OBJ_TCX=$(SRC_TCX:.c=.o)
OBJ_CTLR=$(SRC_CTLR:.c=.o)
OBJ_VP=$(SRC_VP:.c=.o)

DEP=$(SRC_TCX:.c=.d) $(SRC_CTLR:.c=.d) $(SRC_VP:.c=.d)

#
# build rules
#

all: $(TARGETS)

tcx: $(OBJ_TCX) 
	$(CC) -pthread -lrt -lm -lreadline -lpng -ljpeg -lSDL2 -lSDL2_ttf -lSDL2_mixer -o $@ $(OBJ_TCX)

ctlr: $(OBJ_CTLR)
	$(CC) -pthread -lrt -lm -lreadline -lpololu-tic-1 -o $@ $(OBJ_CTLR)

vp: $(OBJ_VP)
	$(CC) -pthread -lrt -lm -lreadline -lpololu-tic-1 -o $@ $(OBJ_VP)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_TCX) $(OBJ_CTL) $(OBJ_VP) $(DEP)

