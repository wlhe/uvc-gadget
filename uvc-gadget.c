/*
 * UVC gadget test application
 *
 * Copyright (C) 2010 Ideas on board SPRL <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include "../drivers/usb/gadget/uvc.h"

#define clamp(val, min, max) ({                 \
        typeof(val) __val = (val);              \
        typeof(min) __min = (min);              \
        typeof(max) __max = (max);              \
        (void) (&__val == &__min);              \
        (void) (&__val == &__max);              \
        __val = __val < __min ? __min: __val;   \
        __val > __max ? __max: __val; })

#define ARRAY_SIZE(a)	((sizeof(a) / sizeof(a[0])))

struct uvc_device
{
    int fd;

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;

    int control;

    unsigned int fcc;
    unsigned int width;
    unsigned int height;

    void **mem;
    unsigned int nbufs;
    unsigned int bufsize;

    unsigned int bulk;
    uint8_t color;
    unsigned int imgsize;
    void *imgdata;
};

static struct uvc_device *
uvc_open(const char *devname)
{
    struct uvc_device *dev;
    struct v4l2_capability cap;
    int ret;
    int fd;

    fd = open(devname, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        printf("v4l2 open failed: %s (%d)\n", strerror(errno), errno);
        return NULL;
    }

    printf("open succeeded, file descriptor = %d\n", fd);

    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        printf("unable to query device: %s (%d)\n", strerror(errno),
            errno);
        close(fd);
        return NULL;
        }

    printf("device is %s on bus %s\n", cap.card, cap.bus_info);

    dev = malloc(sizeof *dev);
    if (dev == NULL) {
        close(fd);
        return NULL;
    }

    memset(dev, 0, sizeof *dev);
    dev->fd = fd;

    return dev;
}

static void
uvc_close(struct uvc_device *dev)
{
    close(dev->fd);
    free(dev->imgdata);
    free(dev->mem);
    free(dev);
}

/* ---------------------------------------------------------------------------
 * Video streaming
 */

static void
uvc_video_fill_buffer(struct uvc_device *dev, struct v4l2_buffer *buf)
{
    unsigned int bpl;
    unsigned int i;

    switch (dev->fcc) {
    case V4L2_PIX_FMT_YUYV:
        /* Fill the buffer with video data. */
        bpl = dev->width * 2;
        for (i = 0; i < dev->height; ++i)
            memset(dev->mem[buf->index] + i*bpl, dev->color++, bpl);

        buf->bytesused = bpl * dev->height;
        break;

    case V4L2_PIX_FMT_MJPEG:
        memcpy(dev->mem[buf->index], dev->imgdata, dev->imgsize);
        buf->bytesused = dev->imgsize;
        break;
    }
}

static int
uvc_video_process(struct uvc_device *dev)
{
    struct v4l2_buffer buf;
    int ret;

    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;

    if ((ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf)) < 0) {
        printf("Unable to dequeue buffer: %s (%d).\n", strerror(errno),
            errno);
        return ret;
    }

    uvc_video_fill_buffer(dev, &buf);

    if ((ret = ioctl(dev->fd, VIDIOC_QBUF, &buf)) < 0) {
        printf("Unable to requeue buffer: %s (%d).\n", strerror(errno),
            errno);
        return ret;
    }

    return 0;
}

static int
uvc_video_reqbufs(struct uvc_device *dev, int nbufs)
{
    struct v4l2_requestbuffers rb;
    struct v4l2_buffer buf;
    unsigned int i;
    int ret;

    for (i = 0; i < dev->nbufs; ++i)
        munmap(dev->mem[i], dev->bufsize);

    free(dev->mem);
    dev->mem = 0;
    dev->nbufs = 0;

    memset(&rb, 0, sizeof rb);
    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        printf("Unable to allocate buffers: %s (%d).\n",
            strerror(errno), errno);
        return ret;
    }

    printf("%u buffers allocated.\n", rb.count);

    /* Map the buffers. */
    dev->mem = malloc(rb.count * sizeof dev->mem[0]);

    for (i = 0; i < rb.count; ++i) {
        memset(&buf, 0, sizeof buf);
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
        if (ret < 0) {
            printf("Unable to query buffer %u: %s (%d).\n", i,
                strerror(errno), errno);
            return -1;
        }
        printf("length: %u offset: %u\n", buf.length, buf.m.offset);

        dev->mem[i] = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, buf.m.offset);
        if (dev->mem[i] == MAP_FAILED) {
            printf("Unable to map buffer %u: %s (%d)\n", i,
                strerror(errno), errno);
            return -1;
        }
        printf("Buffer %u mapped at address %p.\n", i, dev->mem[i]);
    }

    dev->bufsize = buf.length;
    dev->nbufs = rb.count;

    return 0;
}

static int
uvc_video_stream(struct uvc_device *dev, int enable)
{
    struct v4l2_buffer buf;
    unsigned int i;
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int ret;

    if (!enable) {
        printf("Stopping video stream.\n");
        ioctl(dev->fd, VIDIOC_STREAMOFF, &type);
        return 0;
    }

    printf("Starting video stream.\n");

    for (i = 0; i < dev->nbufs; ++i) {
        memset(&buf, 0, sizeof buf);

        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;

        uvc_video_fill_buffer(dev, &buf);

        printf("Queueing buffer %u.\n", i);
        if ((ret = ioctl(dev->fd, VIDIOC_QBUF, &buf)) < 0) {
            printf("Unable to queue buffer: %s (%d).\n",
                strerror(errno), errno);
            break;
        }
    }

    ioctl(dev->fd, VIDIOC_STREAMON, &type);
    return ret;
}

static int
uvc_video_set_format(struct uvc_device *dev)
{
    struct v4l2_format fmt;
    int ret;

    printf("Setting format to 0x%08x %ux%u\n",
        dev->fcc, dev->width, dev->height);

    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = dev->width;
    fmt.fmt.pix.height = dev->height;
    fmt.fmt.pix.pixelformat = dev->fcc;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (dev->fcc == V4L2_PIX_FMT_MJPEG)
        fmt.fmt.pix.sizeimage = dev->imgsize * 1.5;

    if ((ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt)) < 0)
        printf("Unable to set format: %s (%d).\n",
            strerror(errno), errno);

    return ret;
}

static int
uvc_video_init(struct uvc_device *dev __attribute__((__unused__)))
{
    return 0;
}

/* ---------------------------------------------------------------------------
 * Request processing
 */

struct uvc_frame_info
{
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info
{
    unsigned int fcc;
    const struct uvc_frame_info *frames;
};

static const struct uvc_frame_info uvc_frames_yuyv[] = {
    {  640, 360, { 666666, 10000000, 50000000, 0 }, },
    { 1280, 720, { 50000000, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_frame_info uvc_frames_mjpeg[] = {
    {  640, 360, { 666666, 10000000, 50000000, 0 }, },
    { 1280, 720, { 50000000, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_format_info uvc_formats[] = {
    { V4L2_PIX_FMT_YUYV, uvc_frames_yuyv },
    { V4L2_PIX_FMT_MJPEG, uvc_frames_mjpeg },
};

static void
uvc_fill_streaming_control(struct uvc_device *dev,
               struct uvc_streaming_control *ctrl,
               int iframe, int iformat)
{
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    unsigned int nframes;

    if (iformat < 0)
        iformat = ARRAY_SIZE(uvc_formats) + iformat;
    if (iformat < 0 || iformat >= (int)ARRAY_SIZE(uvc_formats))
        return;
    format = &uvc_formats[iformat];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    if (iframe < 0)
        iframe = nframes + iframe;
    if (iframe < 0 || iframe >= (int)nframes)
        return;
    frame = &format->frames[iframe];

    memset(ctrl, 0, sizeof *ctrl);

    ctrl->bmHint = 1;
    ctrl->bFormatIndex = iformat + 1;
    ctrl->bFrameIndex = iframe + 1;
    ctrl->dwFrameInterval = frame->intervals[0];
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
        ctrl->dwMaxVideoFrameSize = dev->imgsize;
        break;
    }
    ctrl->dwMaxPayloadTransferSize = 512;	/* TODO this should be filled by the driver. */
    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

static void
uvc_events_process_standard(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                struct uvc_request_data *resp)
{
    printf("standard request\n");
    (void)dev;
    (void)ctrl;
    (void)resp;
}

static void
uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
               struct uvc_request_data *resp)
{
    printf("control request (req %02x cs %02x)\n", req, cs);
    (void)dev;
    (void)resp;
}

static void
uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs,
                 struct uvc_request_data *resp)
{
    struct uvc_streaming_control *ctrl;

    printf("streaming request (req %02x cs %02x)\n", req, cs);

    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
        return;

    ctrl = (struct uvc_streaming_control *)&resp->data;
    resp->length = sizeof *ctrl;

    switch (req) {
    case UVC_SET_CUR:
        dev->control = cs;
        resp->length = 34;
        break;

    case UVC_GET_CUR:
        if (cs == UVC_VS_PROBE_CONTROL)
            memcpy(ctrl, &dev->probe, sizeof *ctrl);
        else
            memcpy(ctrl, &dev->commit, sizeof *ctrl);
        break;

    case UVC_GET_MIN:
    case UVC_GET_MAX:
    case UVC_GET_DEF:
        uvc_fill_streaming_control(dev, ctrl, req == UVC_GET_MAX ? -1 : 0,
                       req == UVC_GET_MAX ? -1 : 0);
        break;

    case UVC_GET_RES:
        memset(ctrl, 0, sizeof *ctrl);
        break;

    case UVC_GET_LEN:
        resp->data[0] = 0x00;
        resp->data[1] = 0x22;
        resp->length = 2;
        break;

    case UVC_GET_INFO:
        resp->data[0] = 0x03;
        resp->length = 1;
        break;
    }
}

static void
uvc_events_process_class(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
             struct uvc_request_data *resp)
{
    if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
        return;

    switch (ctrl->wIndex & 0xff) {
    case UVC_INTF_CONTROL:
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8, resp);
        break;

    case UVC_INTF_STREAMING:
        uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8, resp);
        break;

    default:
        break;
    }
}

static void
uvc_events_process_setup(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
             struct uvc_request_data *resp)
{
    dev->control = 0;

    printf("bRequestType %02x bRequest %02x wValue %04x wIndex %04x "
        "wLength %04x\n", ctrl->bRequestType, ctrl->bRequest,
        ctrl->wValue, ctrl->wIndex, ctrl->wLength);

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        uvc_events_process_standard(dev, ctrl, resp);
        break;

    case USB_TYPE_CLASS:
        uvc_events_process_class(dev, ctrl, resp);
        break;

    default:
        break;
    }
}

static void
uvc_events_process_data(struct uvc_device *dev, struct uvc_request_data *data)
{
    struct uvc_streaming_control *target;
    struct uvc_streaming_control *ctrl;
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    const unsigned int *interval;
    unsigned int iformat, iframe;
    unsigned int nframes;

    switch (dev->control) {
    case UVC_VS_PROBE_CONTROL:
        printf("setting probe control, length = %d\n", data->length);
        target = &dev->probe;
        break;

    case UVC_VS_COMMIT_CONTROL:
        printf("setting commit control, length = %d\n", data->length);
        target = &dev->commit;
        break;

    default:
        printf("setting unknown control, length = %d\n", data->length);
        return;
    }

    ctrl = (struct uvc_streaming_control *)&data->data;
    iformat = clamp((unsigned int)ctrl->bFormatIndex, 1U,
            (unsigned int)ARRAY_SIZE(uvc_formats));
    format = &uvc_formats[iformat-1];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    iframe = clamp((unsigned int)ctrl->bFrameIndex, 1U, nframes);
    frame = &format->frames[iframe-1];
    interval = frame->intervals;

    while (interval[0] < ctrl->dwFrameInterval && interval[1])
        ++interval;

    target->bFormatIndex = iformat;
    target->bFrameIndex = iframe;
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        target->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
        if (dev->imgsize == 0)
            printf("WARNING: MJPEG requested and no image loaded.\n");
        target->dwMaxVideoFrameSize = dev->imgsize;
        break;
    }
    target->dwFrameInterval = *interval;

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        dev->fcc = format->fcc;
        dev->width = frame->width;
        dev->height = frame->height;

        uvc_video_set_format(dev);
        if (dev->bulk)
            uvc_video_stream(dev, 1);
    }
}

static void
uvc_events_process(struct uvc_device *dev)
{
    struct v4l2_event v4l2_event;
    struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
    struct uvc_request_data resp;
    int ret;

    ret = ioctl(dev->fd, VIDIOC_DQEVENT, &v4l2_event);
    if (ret < 0) {
        printf("VIDIOC_DQEVENT failed: %s (%d)\n", strerror(errno),
            errno);
        return;
    }

    memset(&resp, 0, sizeof resp);
    resp.length = -EL2HLT;

    switch (v4l2_event.type) {
    case UVC_EVENT_CONNECT:
    case UVC_EVENT_DISCONNECT:
        return;

    case UVC_EVENT_SETUP:
        uvc_events_process_setup(dev, &uvc_event->req, &resp);
        break;

    case UVC_EVENT_DATA:
        uvc_events_process_data(dev, &uvc_event->data);
        return;

    case UVC_EVENT_STREAMON:
        uvc_video_reqbufs(dev, 4);
        uvc_video_stream(dev, 1);
        return;

    case UVC_EVENT_STREAMOFF:
        uvc_video_stream(dev, 0);
        uvc_video_reqbufs(dev, 0);
        return;
    }

    ioctl(dev->fd, UVCIOC_SEND_RESPONSE, &resp);
    if (ret < 0) {
        printf("UVCIOC_S_EVENT failed: %s (%d)\n", strerror(errno),
            errno);
        return;
    }
}

static void
uvc_events_init(struct uvc_device *dev)
{
    struct v4l2_event_subscription sub;

    uvc_fill_streaming_control(dev, &dev->probe, 0, 0);
    uvc_fill_streaming_control(dev, &dev->commit, 0, 0);

    if (dev->bulk) {
        /* FIXME Crude hack, must be negotiated with the driver. */
        dev->probe.dwMaxPayloadTransferSize = 16 * 1024;
        dev->commit.dwMaxPayloadTransferSize = 16 * 1024;
    }


    memset(&sub, 0, sizeof sub);
    sub.type = UVC_EVENT_SETUP;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_DATA;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMON;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

/* ---------------------------------------------------------------------------
 * main
 */

static void image_load(struct uvc_device *dev, const char *img)
{
    int fd = -1;

    if (img == NULL)
        return;

    fd = open(img, O_RDONLY);
    if (fd == -1) {
        printf("Unable to open MJPEG image '%s'\n", img);
        return;
    }

    dev->imgsize = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    dev->imgdata = malloc(dev->imgsize);
    if (dev->imgdata == NULL) {
        printf("Unable to allocate memory for MJPEG image\n");
        dev->imgsize = 0;
        return;
    }

    read(fd, dev->imgdata, dev->imgsize);
    close(fd);
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -b		Use bulk mode\n");
    fprintf(stderr, " -d device	Video device\n");
    fprintf(stderr, " -h		Print this help screen and exit\n");
    fprintf(stderr, " -i image	MJPEG image\n");
}

int main(int argc, char *argv[])
{
    char *device = "/dev/video0";
    struct uvc_device *dev;
    int bulk_mode = 0;
    char *mjpeg_image = NULL;
    fd_set fds;
    int ret, opt;

    while ((opt = getopt(argc, argv, "bd:hi:")) != -1) {
        switch (opt) {
        case 'b':
            bulk_mode = 1;
            break;

        case 'd':
            device = optarg;
            break;

        case 'h':
            usage(argv[0]);
            return 0;

        case 'i':
            mjpeg_image = optarg;
            break;

        default:
            fprintf(stderr, "Invalid option '-%c'\n", opt);
            usage(argv[0]);
            return 1;
        }
    }

    dev = uvc_open(device);
    if (dev == NULL)
        return 1;

    image_load(dev, mjpeg_image);

    dev->bulk = bulk_mode;

    uvc_events_init(dev);
    uvc_video_init(dev);

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    while (1) {
        fd_set efds = fds;
        fd_set wfds = fds;

        ret = select(dev->fd + 1, NULL, &wfds, &efds, NULL);
        if (FD_ISSET(dev->fd, &efds))
            uvc_events_process(dev);
        if (FD_ISSET(dev->fd, &wfds))
            uvc_video_process(dev);
    }

    uvc_close(dev);
    return 0;
}
