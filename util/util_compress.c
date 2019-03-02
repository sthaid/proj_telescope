/*
Copyright (c) 2019 Steven Haid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "minilzo-2.10/minilzo.h"

#include "util_misc.h"
#include "util_compress.h"

void compress_init(void)
{
    int rc;

    rc = lzo_init();
    if (rc != LZO_E_OK) {
        FATAL("compress_init failed\n");
    }
}

void compress(void *buff, size_t buff_len, void *cmp, size_t *cmp_len_arg)
{
    int rc;
    char wrkmem[LZO1X_1_MEM_COMPRESS];
    lzo_uint cmp_len = *cmp_len_arg;
    size_t min_required_cmp_len = (buff_len + buff_len / 16 + 64 + 3);

    if (cmp_len < min_required_cmp_len) {
        FATAL("cmp_len=%zd is less than min required %zd\n", 
              cmp_len, min_required_cmp_len);
    }

    rc = lzo1x_1_compress(buff, buff_len, cmp, &cmp_len, wrkmem);
    if (rc != LZO_E_OK) {
        FATAL("lzo1x_1_compress failed, rc=%d\n", rc);
    }

    *cmp_len_arg = cmp_len;
}

int decompress(void *cmp, size_t cmp_len, void *buff, size_t *buff_len_arg)
{
    int rc;
    lzo_uint buff_len = *buff_len_arg;

    rc = lzo1x_decompress_safe(cmp, cmp_len, buff, &buff_len, NULL);

    *buff_len_arg = buff_len;
    return rc == LZO_E_OK ? 0 : -1;
}

