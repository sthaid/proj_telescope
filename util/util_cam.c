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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <pthread.h>

#include "util_cam.h"
#include "util_misc.h"

//
// notes:
//
// v4l2-ctl is useful to determine webcam capabilities
//  - yum install v4l-utils
//  - v4l2-ctl --help-all
//  - v4l2-ctl --list-formats-ext
//
// sample code:
//  - https://gist.github.com/Circuitsoft/1126411  print capabilities
//
// https://help.ubuntu.com/community/Webcam#VLC
//

//
// defines
//

#define WC_VIDEO   "/dev/video"   // base name
#define MAX_BUFMAP 16

#define PIXEL_FMT_CHARS(x)  (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, ((x) >> 24) & 0xff

//
// typedefs
//

typedef struct {
    void  * addr;
    int32_t length;
} bufmap_t;

//
// variables
//

static int32_t   cam_fd = -1;
static bufmap_t  bufmap[MAX_BUFMAP];
static bool      cam_exit_handler_registered;

//
// prototypes
//

static void cam_exit_handler(void);
static void cam_ctrls_reset(void);
static int32_t cam_ctrls_query(void);

// -----------------  CAM INIT  ----------------------------------------------------

int32_t cam_initialize(int32_t req_fmt, int32_t req_width, int32_t req_height, double req_tpf,
                       int32_t *act_fmt, int32_t *act_width, int32_t *act_height, double *act_tpf)
{
    struct v4l2_capability     cap;
    struct v4l2_cropcap        cropcap;
    struct v4l2_crop           crop;
    struct v4l2_format         format;
    struct v4l2_streamparm     streamparm;
    struct v4l2_requestbuffers reqbuf;
    struct v4l2_buffer         buffer;
    enum   v4l2_buf_type       buf_type;
    int32_t                    i;

    // print args
    INFO("req_fmt='%c%c%c%c' req_width=%d req_height=%d req_tpf=%f\n",
         PIXEL_FMT_CHARS(req_fmt), req_width, req_height, req_tpf);

    // preset returns to invalid
    *act_fmt = 0;
    *act_width = 0;
    *act_height = 0;
    *act_tpf = 0;

    // reset cam_ctrls; a subsequent call to cam_ctrls_query will re-initialize 
    cam_ctrls_reset();

    // if cam_fd is open then this is a re-initialize call,
    // so start by closing cam_fd
    if (cam_fd != -1) {
        buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &buf_type);
        close(cam_fd);
        cam_fd = -1;
    }

    // open webcam, try devices /dev/video0,1,...
    for (i = 0; i < 10; i++) {
        char devpath[100];
        sprintf(devpath, "%s%d", WC_VIDEO, i);
        cam_fd = open(devpath, O_RDWR|O_NONBLOCK);
        if (cam_fd < 0) {
            DEBUG("open failed %s %s\n",  devpath, strerror(errno));
            continue;
        }
        break;
    }
    if (cam_fd < 0) {
        goto error;
    }

    // get and verify capability
    if (ioctl(cam_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        ERROR("ioctl VIDIOC_QUERYCAP %s\n", strerror(errno));
        goto error;
    }
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        ERROR("no cap V4L2_CAP_VIDEO_CAPTURE\n");
        goto error;
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == 0) {
        ERROR("no cap V4L2_CAP_STREAMING\n");
        goto error;
    }

    // get VIDEO_CAPTURE format type, and
    // set pixel format to (MJPEG,width,height)
    bzero(&format, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_G_FMT, &format) < 0) {
        ERROR("ioctl VIDIOC_G_FMT %s\n", strerror(errno));
        goto error;
    }
    format.fmt.pix.pixelformat = req_fmt;
    format.fmt.pix.width       = req_width;
    format.fmt.pix.height      = req_height;
    if (ioctl(cam_fd, VIDIOC_S_FMT, &format) < 0) {
        ERROR("ioctl VIDIOC_S_FMT %s\n", strerror(errno));
        goto error;
    }
    DEBUG("VIDIOC_S_FMT ret: fmt='%c%c%c%c' width=%d height=%d\n",
          PIXEL_FMT_CHARS(format.fmt.pix.pixelformat), 
          format.fmt.pix.width , format.fmt.pix.height);

    // get crop capabilities; 
    // if success then set crop to default
    bzero(&cropcap,sizeof(cropcap));
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_CROPCAP, &cropcap) == 0) {
        bzero(&crop, sizeof(crop));
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;
        if (ioctl(cam_fd, VIDIOC_S_CROP, &crop) < 0) {
            if (errno == EINVAL || errno == ENOTTY) {
                DEBUG("crop not supported\n");
            } else {
                ERROR("ioctl VIDIOC_S_CROP, %s\n", strerror(errno));
                goto error;
            }
        }
    }

    // set frames per sec
    bzero(&streamparm, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator   = 1;
    streamparm.parm.capture.timeperframe.denominator = lrint(1 / req_tpf);
    if (ioctl(cam_fd, VIDIOC_S_PARM, &streamparm) < 0) {
        ERROR("ioctl VIDIOC_S_PARM, %s\n", strerror(errno));
        goto error;
    }
    DEBUG("VIDIOC_S_PARM ret: timeperframe=%f   (%d / %d)\n",
          (double)streamparm.parm.capture.timeperframe.numerator / streamparm.parm.capture.timeperframe.denominator,
          streamparm.parm.capture.timeperframe.numerator,
          streamparm.parm.capture.timeperframe.denominator);

    // request memory mapped buffers
    bzero(&reqbuf, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = MAX_BUFMAP;
    if (ioctl (cam_fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
        ERROR("ioctl VIDIOC_REQBUFS %s\n", strerror(errno));
        goto error;
    }

    // verify we got all the buffers requested
    if (reqbuf.count != MAX_BUFMAP) {
        ERROR("got wrong number of frames, requested %d, actual %d\n",
              MAX_BUFMAP, reqbuf.count);
        goto error;
    }

    // memory map each of the buffers
    for (i = 0; i < MAX_BUFMAP; i++) {
        bzero(&buffer,sizeof(struct v4l2_buffer));
        buffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index  = i;
        if (ioctl (cam_fd, VIDIOC_QUERYBUF, &buffer) < 0) {
            ERROR("ioctl VIDIOC_QUERYBUF index=%d %s\n", i, strerror(errno));
            goto error;
        }
        bufmap[i].addr = mmap(NULL, buffer.length,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED,           
                              cam_fd, buffer.m.offset);
        bufmap[i].length = buffer.length;

        if (bufmap[i].addr == MAP_FAILED) {
            ERROR("mmap failed, %s\n", strerror(errno));
            goto error;
        }
    }

    // give the buffers to driver
   for (i = 0; i < MAX_BUFMAP; i++) {
        bzero(&buffer,sizeof(struct v4l2_buffer));
        buffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index  = i;
        if (ioctl(cam_fd, VIDIOC_QBUF, &buffer) < 0) {
            ERROR("ioctl VIDIOC_QBUF index=%d %s\n", i, strerror(errno));
            goto error;
        }
    }

    // enable capture
    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &buf_type) < 0) {
        ERROR("ioctl VIDIOC_STREAMON %s\n", strerror(errno));
        goto error;
    }

    // query controls
    if (cam_ctrls_query() < 0) {
        ERROR("cam_ctrls_query failed\n");
        goto error;
    }

    // register exit handler
    if (!cam_exit_handler_registered) {
        atexit(cam_exit_handler);
        cam_exit_handler_registered = true;
    }

    // set return values
    *act_fmt    = format.fmt.pix.pixelformat;
    *act_width  = format.fmt.pix.width;
    *act_height = format.fmt.pix.height;
    *act_tpf    = (double)streamparm.parm.capture.timeperframe.numerator / 
                  streamparm.parm.capture.timeperframe.denominator;
    INFO("act_fmt='%c%c%c%c' act_width=%d act_height=%d act_tpf=%f\n",
         PIXEL_FMT_CHARS(*act_fmt), *act_width, *act_height, *act_tpf);

    // return success
    INFO("success\n");
    return 0;

error:
    // error
    if (cam_fd > 0) {
        close(cam_fd);
        cam_fd = -1;
    }
    return -1;
}

static void cam_exit_handler(void) 
{
    enum v4l2_buf_type buf_type;

    if (cam_fd == -1) {
        return;
    }

    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMOFF, &buf_type) < 0) {
        WARN("ioctl VIDIOC_STREAMOFF %s\n", strerror(errno));
    }

    close(cam_fd);
}

// -----------------  CAM VIDEO STREAMING  -----------------------------------------

int32_t cam_get_buff(uint8_t **buff, uint32_t *len)
{
    int64_t duration = 0;

    static int32_t max_buffer_avail;
    static struct v4l2_buffer buffer_avail[MAX_BUFMAP];

    // if not initialized then return error
    if (cam_fd == -1) {
        return -1;
    }
 
try_again:
    // dequeue buffers until no more available
    while (true) {
        struct v4l2_buffer buffer;

        // dequeue camera buffer, 
        // if no buffers available then break out of this loop
        bzero(&buffer, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cam_fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) {
                break;
            } else {
                ERROR("ioctl VIDIOC_DQBUF failed, %s\n", strerror(errno));
                return -1;
            }
        }

        // debug print
        DEBUG("GET: index=%d addr=%p length=%d flags=0x%x\n", 
              buffer.index, bufmap[buffer.index].addr, bufmap[buffer.index].length,  buffer.flags);

        // if error flag is set then requeue the buffer and break out of this loop
        // so that the duration check is made below
        if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
            WARN("V4L2_BUF_FLAG_ERROR is set, index=%d flags=0x%x\n", 
                 buffer.index, buffer.flags);
            cam_put_buff(bufmap[buffer.index].addr);
            break;
        }

        // save buffer at end of buffer_avail array
        if (max_buffer_avail >= MAX_BUFMAP) {
            ERROR("max_buffer_avail = %d\n", max_buffer_avail);
            return -1;
        }
        buffer_avail[max_buffer_avail++] = buffer;
    }

    // if no buffers are available then 
    //   if it has been more than 2 seconds then 
    //     return error
    //   else
    //     delay and try again
    //   endif
    // endif
    if (max_buffer_avail == 0) {
        if (duration > 10000000) {
            ERROR("cam not responding\n");
            return -1;
        } else {
            usleep(1000);
            duration += 1000;
            goto try_again;
        }
    }
  
    // if this routine is now holding more than 3 then
    // discard all but the newest
    if (max_buffer_avail > 3) {
        int32_t i;
        WARN("discarding buffs because holding=%d is greater than 3\n", max_buffer_avail);
        for (i = 0; i < max_buffer_avail-1; i++) {
            cam_put_buff(bufmap[buffer_avail[i].index].addr);
        }

        buffer_avail[0] = buffer_avail[max_buffer_avail-1];
        max_buffer_avail = 1;
    }

    // return the oldest in buffer_avail
    *buff = (uint8_t*)bufmap[buffer_avail[0].index].addr;
    *len =  buffer_avail[0].bytesused;

    // shift the remaining, slot 0 needs to be the oldest
    max_buffer_avail--;
    memmove(&buffer_avail[0], &buffer_avail[1], max_buffer_avail*sizeof(struct v4l2_buffer));

    // return success
    return 0;
}

void cam_put_buff(uint8_t * buff)   
{
    struct v4l2_buffer buffer;
    int32_t i, buff_idx;

    // if not initialized then return
    if (cam_fd == -1) {
        return;
    }

    // find the buff_idx
    for (i = 0; i < MAX_BUFMAP; i++) {
        if (bufmap[i].addr == buff) {
            break;
        }
    }
    if (i == MAX_BUFMAP) {
        ERROR("invalid buff addr %p\n", buff);
    }
    buff_idx = i;

    // debug print
    DEBUG("PUT: index=%d addr=%p length=%d\n",
           buff_idx, bufmap[buff_idx].addr, bufmap[buff_idx].length);

    // give the buffer back to the driver
    bzero(&buffer,sizeof(struct v4l2_buffer));
    buffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = bufmap[buff_idx].length;
    buffer.index  = buff_idx;
    if (ioctl(cam_fd, VIDIOC_QBUF, &buffer) < 0) {
        ERROR("ioctl VIDIOC_QBUF index=%d %s\n", buff_idx, strerror(errno));
    }
}

// -----------------  CAM CONTROLS  ------------------------------------------------

// references
//   https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/control.html
//   https://www.linuxtv.org/downloads/v4l-dvb-apis-old/vidioc-queryctrl.html#control-flags

// defines
#define READABLE(f) \
        (((f) &  \
          (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_INACTIVE | V4L2_CTRL_FLAG_WRITE_ONLY)) \
         == 0)
#define WRITEABLE(f) \
        (((f) &  \
          (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_GRABBED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE | \
           V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_HAS_PAYLOAD)) \
         == 0)
#define LOCK   do { pthread_mutex_lock(&cam_ctrls_mutex); } while (0)
#define UNLOCK do { pthread_mutex_unlock(&cam_ctrls_mutex); } while (0)

// variables
static char query_ctrl_buff[10000];
static cam_query_ctrls_t * query_ctrl = (void*)query_ctrl_buff;
static int32_t query_ctrl_len;
static pthread_mutex_t cam_ctrls_mutex = PTHREAD_MUTEX_INITIALIZER;

// prototypes
static char * flags_str(int32_t flags) __attribute__ ((unused));
static char * type_str(int32_t type) __attribute__ ((unused));
static void debug_print_query_ctrl(void);

// - - - - CAM CTRLS INIT  - - - - 

// clear query_ctrl variable; 
// a subsequent call to cam_ctrls_query will be needed to re-init the query_ctrl var
static void cam_ctrls_reset(void) 
{
    LOCK;
    memset(query_ctrl_buff,0,sizeof(query_ctrl_buff));
    query_ctrl_len = 0;
    UNLOCK;
}

// query controls and store result variable 'query_ctrl"
static int32_t cam_ctrls_query(void)
{
    int32_t rc;
    struct v4l2_queryctrl queryctrl;
    char strings[8102];
    int32_t strings_offset, strings_offset_save, strings_count;

    // lock cam_ctrls mutex
    LOCK;

    // init
    memset(query_ctrl_buff, 0, sizeof(query_ctrl_buff));
    memset(strings,0,sizeof(strings));
    strings_count = 0;
    strings_offset = 0;

    // issue VIDIOC_QUERYCTRL until all controls are queried
    DEBUG("VIDIOC_QUERYCTRL ...\n");
    memset(&queryctrl, 0, sizeof(queryctrl));
    while (true) {
        // issue the QUERYCTRL ioctl
        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        rc = ioctl(cam_fd, VIDIOC_QUERYCTRL, &queryctrl);
        if (rc < 0) {
            break;
        }

        // if this control is not relevant then continue
        if ((queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) ||
            (queryctrl.flags & V4L2_CTRL_FLAG_INACTIVE)) 
        {
            continue;  
        }

        // debug print the ctrl
        DEBUG("%32s (0x%x) %s mmsd=(%d %d %d %d) (%s%s) %s\n", 
             queryctrl.name,
             queryctrl.id,
             type_str(queryctrl.type),
             queryctrl.minimum,
             queryctrl.maximum,
             queryctrl.step,
             queryctrl.default_value,
             READABLE(queryctrl.flags) ? "R" : "",
             WRITEABLE(queryctrl.flags) ? "W" : "",
             flags_str(queryctrl.flags));

        // store the ctrl in 'query_ctrl variable'
        struct cam_ctrl_s *x = &query_ctrl->cam_ctrl[query_ctrl->max_cam_ctrl++];
        strncpy(x->name, (char*)queryctrl.name, sizeof(x->name));
        x->cid                 = queryctrl.id;
        x->current_value       = NO_CAM_VALUE;
        x->minimum             = queryctrl.minimum;
        x->maximum             = queryctrl.maximum;
        x->step                = queryctrl.step;
        x->default_value       = queryctrl.default_value;
        x->menu_strings_count  = 0;
        x->menu_strings_offset = 0;

        x->type = CAM_CTRL_TYPE_OTHER;
        if (queryctrl.type == V4L2_CTRL_TYPE_BUTTON) {
            x->type = CAM_CTRL_TYPE_BUTTON;
        } else if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER ||
                   queryctrl.type == V4L2_CTRL_TYPE_MENU ||
                   queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN)
        {
            if (READABLE(queryctrl.flags) && WRITEABLE(queryctrl.flags)) {
                x->type = CAM_CTRL_TYPE_READ_WRITE;
            } else if (READABLE(queryctrl.flags)) {
                x->type = CAM_CTRL_TYPE_READ_ONLY;
            }
        }

        // if there are menu strings then query the strings and
        // add them to the 'strings' buffer
        strings_offset_save = strings_offset;
        strings_count = 0;
        if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
            struct v4l2_querymenu querymenu;
            char name[100];

            memset(&querymenu, 0, sizeof(querymenu));
            querymenu.id = queryctrl.id;
            for (querymenu.index = queryctrl.minimum;
                 querymenu.index <= queryctrl.maximum;
                 querymenu.index++) 
            {
                rc = ioctl(cam_fd, VIDIOC_QUERYMENU, &querymenu);
                if (rc == 0) {
                    strcpy(name, (char*)querymenu.name);
                } else {
                    sprintf(name, "Invalid");
                }
                DEBUG("%32s menu index=%d name=%s\n", "", querymenu.index, name);

                strcat(&strings[strings_offset], name);
                strings_offset += strlen(name) + 1;
                strings_count++;
            }
        }

        // if there are menu strings then update the query_ctrl variable
        // to include the number of menu strings and the offset to the first
        // of the menu strings
        x->menu_strings_count  = strings_count;
        x->menu_strings_offset = (strings_count ? strings_offset_save : 0);
    }

    // append strings to query_ctrl
    memcpy(&query_ctrl->cam_ctrl[query_ctrl->max_cam_ctrl],
           strings,
           strings_offset);

    // set query_ctrl_len
    query_ctrl_len = offsetof(cam_query_ctrls_t, cam_ctrl[query_ctrl->max_cam_ctrl]) + strings_offset;

    // debug print query_ctrl
    debug_print_query_ctrl();

#if 0
    // the basic queryctrl is sufficient for what I amd doing
    struct v4l2_query_ext_ctrl query_ext_ctrl;
    DEBUG("VIDIOC_QUERY_EXT_CTRL ...\n");
    memset(&query_ext_ctrl, 0, sizeof(query_ext_ctrl));
    while (true) {
        query_ext_ctrl.id |= (V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND);
        rc = ioctl(cam_fd, VIDIOC_QUERY_EXT_CTRL, &query_ext_ctrl);
        if (rc < 0) {
            break;
        }

        if ((query_ext_ctrl.flags & V4L2_CTRL_FLAG_DISABLED) ||
            (query_ext_ctrl.flags & V4L2_CTRL_FLAG_INACTIVE)) 
        {
            continue;  
        }

        DEBUG("%32s (0x%x) %s mmsd=(%lld %lld %lld %lld) (%s%s) %s\n", 
             query_ext_ctrl.name,
             query_ext_ctrl.id,
             type_str(query_ext_ctrl.type),
             query_ext_ctrl.minimum,
             query_ext_ctrl.maximum,
             query_ext_ctrl.step,
             query_ext_ctrl.default_value,
             READABLE(query_ext_ctrl.flags) ? "R" : "",
             WRITEABLE(query_ext_ctrl.flags) ? "W" : "",
             flags_str(query_ext_ctrl.flags));
    }
#endif

    // unlock cam_ctrls mutex
    UNLOCK;

    return 0;
}

static char * flags_str(int32_t flags)
{
    static char str[1000];

    #define CHECK_FLAG(flg) if (flags & (V4L2_CTRL_FLAG_##flg)) strcat(str, #flg " ");

    str[0] = '\0';
    CHECK_FLAG(DISABLED);
    CHECK_FLAG(GRABBED);
    CHECK_FLAG(READ_ONLY);
    CHECK_FLAG(UPDATE);
    CHECK_FLAG(INACTIVE);
    CHECK_FLAG(SLIDER);
    CHECK_FLAG(WRITE_ONLY);
    CHECK_FLAG(VOLATILE);
    CHECK_FLAG(HAS_PAYLOAD);
    CHECK_FLAG(EXECUTE_ON_WRITE);
    CHECK_FLAG(MODIFY_LAYOUT);

    return str;
}

static char * type_str(int32_t type)
{
    static char str[100];
    #define CHECK_TYPE(t) if (type == V4L2_CTRL_TYPE_##t) return #t;

    CHECK_TYPE(INTEGER);
    CHECK_TYPE(BOOLEAN);
    CHECK_TYPE(MENU);
    CHECK_TYPE(BUTTON);
    CHECK_TYPE(INTEGER64);
    CHECK_TYPE(CTRL_CLASS);
    CHECK_TYPE(STRING);
    CHECK_TYPE(BITMASK);
    CHECK_TYPE(INTEGER_MENU);

    sprintf(str, "V4L2_CTRL_TYPE_%d", type);
    return str;
}

static void debug_print_query_ctrl(void)
{
    int32_t i, j;
    char *strings, *s, *menu_strings[100];

    strings = (char*)&query_ctrl->cam_ctrl[query_ctrl->max_cam_ctrl];

    INFO("--- QUERY CTRL TABLE (len=%d) ---\n", query_ctrl_len);

    for (i = 0; i < query_ctrl->max_cam_ctrl; i++) {
        struct cam_ctrl_s *x = &query_ctrl->cam_ctrl[i];

        s = strings + x->menu_strings_offset;
        for (j = 0; j < x->menu_strings_count; j++) {
            menu_strings[j] = s;
            s += strlen(s) + 1;
        }

        INFO("%32s cid=%8x type=%s mmsd=%d,%d,%d,%d strings=%d\n",
             x->name,
             x->cid,
             CAM_CTRL_TYPE_STR(x->type),
             x->minimum,
             x->maximum,
             x->step,
             x->default_value,
             x->menu_strings_count);
        for (j = 0; j < x->menu_strings_count; j++) {
            INFO("%42s %s\n", "", menu_strings[j]);
        }
    }

    INFO("------------------------\n");
}

// - - - - CAM CTRLS API - - - - 

// note - qclen is in/out
int32_t cam_ctrls_get_all(cam_query_ctrls_t *qc, int32_t *qclen)
{
    int32_t i, rc;

    // lock cam_ctrls mutex
    LOCK;

    // if query_ctrl is not valid then return error
    if (query_ctrl->max_cam_ctrl == 0 || query_ctrl_len == 0) {
        UNLOCK;
        return -1;
    }

    // if caller's buffer is not big enough return error
    if (*qclen < query_ctrl_len) {
        ERROR("caller's bufflen %d too small, required=%d\n", *qclen, query_ctrl_len);
        UNLOCK;
        return -1;
    }

    // copy query_ctrl to caller's buffer
    memcpy(qc, query_ctrl, query_ctrl_len);

    // get all values that can be read
    for (i = 0; i < qc->max_cam_ctrl; i++) {
        struct cam_ctrl_s *x = &qc->cam_ctrl[i];
        if (x->type == CAM_CTRL_TYPE_READ_WRITE ||
            x->type == CAM_CTRL_TYPE_READ_ONLY)
        {
            rc = cam_ctrls_get(x->cid, &x->current_value);
            if (rc != 0) {
                UNLOCK;
                return rc;
            }
        }
    }

    // also return qclen
    *qclen = query_ctrl_len;

    // unlock cam_ctrls mutex
    UNLOCK;

    // return success
    return 0;
}

int32_t cam_ctrls_set_all_to_default(void)
{
    int32_t i, rc;

    INFO("resetting all ctrls to default values\n");

    // lock cam_ctrls mutex
    LOCK;

    // if query_ctrl is not valid then return error
    if (query_ctrl->max_cam_ctrl == 0 || query_ctrl_len == 0) {
        UNLOCK;
        return -1;
    }

    // loop over all controls and set those of READ_WRITE type to default_value
    for (i = 0; i < query_ctrl->max_cam_ctrl; i++) {
        struct cam_ctrl_s *x = &query_ctrl->cam_ctrl[i];
        if (x->type != CAM_CTRL_TYPE_READ_WRITE) {
            continue;
        }
        rc = cam_ctrls_set(x->cid, x->default_value);
        if (rc != 0) {
            UNLOCK;
            return rc;
        }
    }

    // unlock cam_ctrls mutex
    UNLOCK;

    // success
    return 0;
}

int32_t cam_ctrls_get(int32_t cid, int32_t *cid_value)
{
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));
    control.id = cid;
    if (ioctl(cam_fd, VIDIOC_G_CTRL, &control) != 0) {
        *cid_value = NO_CAM_VALUE;
        ERROR("failed get of cid 0x%x, %s\n", cid, strerror(errno));
        return -1;
    }

    *cid_value = control.value;
    return 0;
}

int32_t cam_ctrls_set(int32_t cid, int32_t cid_value)
{
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));
    control.id = cid;
    control.value = cid_value;
    if (ioctl(cam_fd, VIDIOC_S_CTRL, &control) != 0) {
        ERROR("failed set of cid 0x%x to %d, %s\n", cid, cid_value, strerror(errno));
        return -1;
    }

    return 0;
}

int32_t cam_ctrls_incr_decr(int32_t cid, bool incr_flag)
{
    struct cam_ctrl_s *x = NULL;
    int32_t cid_value, rc, i;

    // lock cam_ctrls mutex
    LOCK;

    // if query_ctrl is not valid then return error
    if (query_ctrl->max_cam_ctrl == 0 || query_ctrl_len == 0) {
        UNLOCK;
        return -1;
    }

    // search the query_ctrl table for the cid
    for (i = 0; i < query_ctrl->max_cam_ctrl; i++) {
        x = &query_ctrl->cam_ctrl[i];
        if (x->cid == cid) {
            break;
        }
    }
    if (i == query_ctrl->max_cam_ctrl) {
        ERROR("cid 0x%x not found in query_ctrl\n", cid);
        UNLOCK;
        return -1;  
    }

    // the cid must be READ_WRITE
    if (x->type != CAM_CTRL_TYPE_READ_WRITE) {
        ERROR("cid 0x%x must be READ_WRITE\n", cid);
        UNLOCK;
        return -1;
    }

    // we need the menu strings
    char *strings, *s, *menu_strings[100];
    strings = (char*)&query_ctrl->cam_ctrl[query_ctrl->max_cam_ctrl];
    s = strings + x->menu_strings_offset;
    for (i = 0; i < x->menu_strings_count; i++) {
        menu_strings[i] = s;
        s += strlen(s) + 1;
    }

    // get the cid_value,
    rc = cam_ctrls_get(cid, &cid_value);
    if (rc < 0) {
        UNLOCK;
        return -1;
    }

try_again:
    // increment or decrement the cid_value, if new value is out of range then return
    if (incr_flag) {
        cid_value += x->step;
    } else {
        cid_value -= x->step;
    }
    if (cid_value < x->minimum || cid_value > x->maximum) {
        UNLOCK;
        return -1;
    }

    // check if new value is associated with an "Invalid" menu string;
    // if so then try to incr/decr again
    if (x->menu_strings_count > 0 &&
        cid_value > 0 && cid_value < x->menu_strings_count &&
        strcmp(menu_strings[cid_value], "Invalid") == 0) 
    {
        goto try_again;
    }

    // set the control
    rc = cam_ctrls_set(cid, cid_value);
    if (rc < 0) {
        UNLOCK;
        return -1;
    }

    // unlock cam_ctrls mutex
    UNLOCK;

    // return success
    return 0;
}

