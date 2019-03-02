#include <stdio.h>
#include <string.h>

#include <util_misc.h>
#include <util_compress.h>

int main()
{
    unsigned char buff[1000000];
    char cmp[2000000];
    int i, rc;
    size_t cmp_len, buff_len;
    uint64_t start_us;

    compress_init();

    for (i = 0; i < sizeof(buff); i++) {
        buff[i] = i;
    }

    start_us = microsec_timer();
    cmp_len = sizeof(cmp);
    compress(buff, sizeof(buff), cmp, &cmp_len);
    printf("compress: buff_len = %ld  cmp_len=%ld  percent=%0.1f  duration_us=%ld\n", 
           sizeof(buff), cmp_len, 100.*cmp_len/sizeof(buff), microsec_timer()-start_us);

    start_us = microsec_timer();
    buff_len = sizeof(buff);
    rc = decompress(cmp, cmp_len, buff, &buff_len);
    printf("decompress: rc=%d  buff_len=%ld  duration_us=%ld\n", 
           rc, buff_len, microsec_timer()-start_us);
    if (rc != 0 || buff_len != sizeof(buff)) {
        printf("ERROR decomp failed\n");
        return -1;
    }

    for (i = 0; i < sizeof(buff); i++) {
        if (buff[i] != (unsigned char)i) {
            printf("ERROR decomp buff[%d] = %d\n", i, buff[i]);
            return -1;
        }
    }

    printf("okay\n");
    return 0;
}
