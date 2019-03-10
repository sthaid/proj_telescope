/*
Copyright (c) 2016 Steven Haid

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

#ifndef __UTIL_CAM_H__
#define __UTIL_CAM_H__

// to be used with cam_query_ctrls_t current_value field
#define NO_CAM_VALUE 999

// to be used with cam_query_ctrls_t type field
#define CAM_CTRL_TYPE_OTHER        0
#define CAM_CTRL_TYPE_READ_WRITE   1
#define CAM_CTRL_TYPE_READ_ONLY    2
#define CAM_CTRL_TYPE_BUTTON       3
#define CAM_CTRL_TYPE_STR(x) \
    ((x) == CAM_CTRL_TYPE_OTHER      ? "OTHER" : \
     (x) == CAM_CTRL_TYPE_READ_WRITE ? "READ_WRITE" : \
     (x) == CAM_CTRL_TYPE_READ_ONLY  ? "READ_ONLY" : \
     (x) == CAM_CTRL_TYPE_BUTTON     ? "BUTTON" : \
                                       "????")

// menu strings follow this struct
typedef struct {
    int32_t max_cam_ctrl;
    struct cam_ctrl_s {
        char name[32];
        int32_t cid;
        int32_t current_value;
        int32_t minimum;
        int32_t maximum;
        int32_t step ;
        int32_t default_value;
        int16_t menu_strings_count;
        int16_t menu_strings_offset;
        int8_t  type;
        int8_t  pad[3];
    } cam_ctrl[0];
} cam_query_ctrls_t;

// prototypes
int32_t cam_initialize(int32_t req_fmt, int32_t req_width, int32_t req_height, double req_tpf,
                       int32_t *act_fmt, int32_t *act_width, int32_t *act_height, double *act_tpf);

int32_t cam_get_buff(uint8_t **buff, uint32_t *len);
void cam_put_buff(uint8_t * buff);

int32_t cam_ctrls_get_all(cam_query_ctrls_t *qc, int32_t *qclen);
int32_t cam_ctrls_set_all_to_default(void);
int32_t cam_ctrls_get(int32_t cid, int32_t *cid_value);
int32_t cam_ctrls_set(int32_t cid, int32_t cid_value);
int32_t cam_ctrls_incr_decr(int32_t cid, bool incr_flag);

#endif
