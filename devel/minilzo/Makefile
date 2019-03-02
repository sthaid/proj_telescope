test_minilzo:
	gcc -g -Wall -O2 -I../../util -lm -o test_minilzo \
            test_minilzo.c \
            ../../util/util_misc.c \
            ../../util/util_compress.c \
            ../../util/minilzo-2.10/minilzo.c

.PHONY: test_minilzo

clean:
	rm -f test_minilzo

