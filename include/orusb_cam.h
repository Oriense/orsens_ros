/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H

// legacy reasons
#include <libavcodec/version.h>
//#if LIBAVCODEC_VERSION_MAJOR < 55
//#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
//#endif

#include <string>
#include <sstream>

extern "C"
{
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
}

typedef struct
{
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  char *image;
  int is_new;
} usb_cam_camera_image_t;

typedef enum
{
  IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR,
} usb_cam_io_method;

typedef enum
{
  PIXEL_FORMAT_YUYV, PIXEL_FORMAT_UYVY, PIXEL_FORMAT_MJPEG, PIXEL_FORMAT_YUVMONO10, PIXEL_FORMAT_RGB24
} usb_cam_pixel_format;

/*// start camera
usb_cam_camera_image_t *usb_cam_camera_start(const char* dev, usb_cam_io_method io, usb_cam_pixel_format pf,
                                             int image_width, int image_height, int framerate);
// shutdown camera
void usb_cam_camera_shutdown(void);
// grabs a new image from the camera
void usb_cam_camera_grab_image(usb_cam_camera_image_t *image);
// enables/disable auto focus
void usb_cam_camera_set_auto_focus(int value);
*/

class UsbCam
{
private:
    char *camera_dev;
    unsigned int pixelformat;
    bool monochrome;
    usb_cam_io_method io;
    int fd;
    struct buffer * buffers;
    unsigned int n_buffers;
    AVFrame *avframe_camera;
    AVFrame *avframe_rgb;
    AVCodec *avcodec;
    AVDictionary *avoptions;
    AVCodecContext *avcodec_context;
    int avframe_camera_size;
    int avframe_rgb_size;

    struct SwsContext *video_sws;

public:

    UsbCam()
    {
        io = IO_METHOD_MMAP;
        fd = -1;
        buffers = NULL;
        n_buffers = 0;
        avframe_camera = NULL;
        avframe_rgb = NULL;
        avcodec = NULL;
        avoptions = NULL;
        avcodec_context = NULL;
        avframe_camera_size = 0;
        avframe_rgb_size = 0;
        video_sws = NULL;
    }

    int init_mjpeg_decoder(int image_width, int image_height);
    void mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels);
    void process_image(const void * src, int len, usb_cam_camera_image_t *dest);
    int read_frame(usb_cam_camera_image_t *image);
    void start_capturing(void);
    void stop_capturing(void);
    void open_device(void);
    void close_device(void);

    usb_cam_camera_image_t *usb_cam_camera_start(const char* dev, usb_cam_io_method io_method,
            usb_cam_pixel_format pixel_format, int image_width, int image_height,
            int framerate, bool mono);
    void usb_cam_camera_shutdown(void);
    void usb_cam_camera_grab_image(usb_cam_camera_image_t *image);
    void usb_cam_camera_set_auto_focus(int value);

    void init_read(unsigned int buffer_size);
    void init_mmap(void);
    void init_device(int image_width, int image_height, int framerate);
    void uninit_device(void);

    void init_userp(unsigned int buffer_size);


};


#endif

