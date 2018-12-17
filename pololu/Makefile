TARGETS = my_tic_test

CC = gcc
OUTPUT_OPTION=-MMD -MP -o $@
CFLAGS = -g -Wall -I/usr/local/include/libpololu-tic-1

SRC_MY_TIC_TEST = my_tic_test.c

OBJ_MY_TIC_TEST=$(SRC_MY_TIC_TEST:.c=.o)

DEP=$(SRC_MY_TIC_TEST:.c=.d)

#
# build rules
#

all: $(TARGETS)

my_tic_test: $(OBJ_MY_TIC_TEST) 
	$(CC) -pthread -lpololu-tic-1 -o $@ $(OBJ_MY_TIC_TEST)

-include $(DEP)

#
# clean rule
#

clean:
	rm -f $(TARGETS) $(OBJ_MY_TIC_TEST) $(DEP) gmon.out

