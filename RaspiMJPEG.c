/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, Silvan Melchior
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiMJPEG.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally stream a preview of current camera input wth MJPEG.
 *
 * \date 25th Nov 2013
 * \Author: Silvan Melchior
 *
 * Description
 *
 * RaspiMJPEG is an OpenMAX-Application based on the mmal-library, which is
 * comparable to and inspired by RaspiVid and RaspiStill. RaspiMJPEG can record
 * 1080p 30fps videos and 5 Mpx images into a file. But instead of showing the
 * preview on a screen, RaspiMJPEG streams the preview as MJPEG into a file.
 * The update-rate and the size of the preview are customizable with parameters
 * and independent of the image/video. Once started, the application receives
 * commands with a unix-pipe and showes its status on stdout and writes it into
 * a status-file. The program terminates itself after receiving a SIGINT or
 * SIGTERM.
 *
 * Usage information in README_RaspiMJPEG.md
 */

#define VERSION "4.2.3"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <semaphore.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

MMAL_STATUS_T status;
MMAL_COMPONENT_T *camera = 0, *jpegencoder = 0, *jpegencoder2 = 0, *h264encoder = 0, *resizer = 0;
MMAL_CONNECTION_T *con_cam_res, *con_res_jpeg, *con_cam_h264, *con_cam_jpeg;
FILE *jpegoutput_file = NULL, *jpegoutput2_file = NULL, *h264output_file = NULL, *status_file = NULL;
MMAL_POOL_T *pool_jpegencoder, *pool_jpegencoder2, *pool_h264encoder;
unsigned int tl_cnt=0, mjpeg_cnt=0, width=320, divider=5, image_cnt=0, image2_cnt=0, video_cnt=0;
unsigned int cam_setting_sharpness=0, cam_setting_contrast=0, cam_setting_brightness=50, cam_setting_saturation=0, cam_setting_iso=0, cam_setting_vs=0, cam_setting_ec=0, cam_setting_rotation=0, cam_setting_quality=85, cam_setting_raw=0, cam_setting_ce_en=0, cam_setting_ce_u=128, cam_setting_ce_v=128, cam_setting_hflip=0, cam_setting_vflip=0, cam_setting_annback=0;
char cam_setting_em[20]="auto", cam_setting_wb[20]="auto", cam_setting_ie[20]="none", cam_setting_mm[20]="average";
unsigned long int cam_setting_bitrate=17000000, cam_setting_roi_x=0, cam_setting_roi_y=0, cam_setting_roi_w=65536, cam_setting_roi_h=65536, cam_setting_ss=0;
unsigned int video_width=1920, video_height=1080, video_fps=25, MP4Box_fps=25, image_width=2592, image_height=1944;
char *jpeg_filename = 0, *jpeg2_filename = 0, *h264_filename = 0, *pipe_filename = 0, *status_filename = 0, *cam_setting_annotation = 0;
unsigned char timelapse=0, mp4box=0, running=1, autostart=1, quality=85, idle=0, capturing=0, motion_detection=0;
int time_between_pic;
time_t currTime;
struct tm *localTime;
short verbose = 0;

void cam_set_annotation();
  
void error (const char *string) {

  fprintf(stderr, "Error: %s\n", string);
  if(status_filename != 0) {
    status_file = fopen(status_filename, "w");
    if(status_file) {
      fprintf(status_file, "Error: %s", string);
      fclose(status_file);
    }
  }
  exit(1);

}

void term (int signum) {

  running = 0;

}

static void camera_control_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  if(buffer->cmd != MMAL_EVENT_PARAMETER_CHANGED) error("Camera sent invalid data");
  mmal_buffer_header_release(buffer);

}

static void jpegencoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  int bytes_written = buffer->length;
  char *filename_temp, *filename_temp2;

  if(mjpeg_cnt == 0) {
    if(!jpegoutput_file) {
      asprintf(&filename_temp, jpeg_filename, image_cnt);
      asprintf(&filename_temp2, "%s.part", filename_temp);
      jpegoutput_file = fopen(filename_temp2, "wb");
      free(filename_temp);
      free(filename_temp2);
      if(!jpegoutput_file) error("Could not open mjpeg-destination");
    }
    if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput_file);
      mmal_buffer_header_mem_unlock(buffer);
    }
    if(bytes_written != buffer->length) error("Could not write all bytes");
  }
  
  if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
    mjpeg_cnt++;
    if(mjpeg_cnt == divider) {
      fclose(jpegoutput_file);
      jpegoutput_file = NULL;
      asprintf(&filename_temp, jpeg_filename, image_cnt);
      asprintf(&filename_temp2, "%s.part", filename_temp);
      rename(filename_temp2, filename_temp);
      free(filename_temp);
      free(filename_temp2);
      image_cnt++;
      mjpeg_cnt = 0;
      cam_set_annotation();
    }
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_jpegencoder->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

static void jpegencoder2_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  int bytes_written = buffer->length;

  if(buffer->length) {
    mmal_buffer_header_mem_lock(buffer);
    bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput2_file);
    mmal_buffer_header_mem_unlock(buffer);
  }
  if(bytes_written != buffer->length) error("Could not write all bytes");
  
  if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
    fclose(jpegoutput2_file);
    if(status_filename != 0) {
      if(!timelapse) {
        status_file = fopen(status_filename, "w");
        fprintf(status_file, "ready");
        fclose(status_file);
      }
    }
    image2_cnt++;
    capturing = 0;
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_jpegencoder2->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

static void h264encoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)  {

  int bytes_written = buffer->length;

  if(buffer->length) {
    mmal_buffer_header_mem_lock(buffer);
    bytes_written = fwrite(buffer->data, 1, buffer->length, h264output_file);
    mmal_buffer_header_mem_unlock(buffer);
    if(bytes_written != buffer->length) error("Could not write all bytes");
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_h264encoder->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

void cam_set_sharpness () {
  MMAL_RATIONAL_T value = {cam_setting_sharpness, 100};
  status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SHARPNESS, value);
  if(status != MMAL_SUCCESS) error("Could not set sharpness");
}

void cam_set_contrast () {
  MMAL_RATIONAL_T value = {cam_setting_contrast, 100};
  status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_CONTRAST, value);
  if(status != MMAL_SUCCESS) error("Could not set contrast");
}

void cam_set_brightness () {
  MMAL_RATIONAL_T value = {cam_setting_brightness, 100};
  status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_BRIGHTNESS, value);
  if(status != MMAL_SUCCESS) error("Could not set brightness");
}

void cam_set_saturation () {
  MMAL_RATIONAL_T value = {cam_setting_saturation, 100};
  status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SATURATION, value);
  if(status != MMAL_SUCCESS) error("Could not set saturation");
}

void cam_set_iso () {
  status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_ISO, cam_setting_iso);
  if(status != MMAL_SUCCESS) error("Could not set ISO");
}

void cam_set_vs () {
  status = mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, cam_setting_vs);
  if(status != MMAL_SUCCESS) error("Could not set video stabilisation");
}

void cam_set_ec () {
  status = mmal_port_parameter_set_int32(camera->control, MMAL_PARAMETER_EXPOSURE_COMP, cam_setting_ec);
  if(status != MMAL_SUCCESS) error("Could not set exposure compensation");
}

void cam_set_em () {
  MMAL_PARAM_EXPOSUREMODE_T mode;
  if(strcmp(cam_setting_em, "off") == 0) mode = MMAL_PARAM_EXPOSUREMODE_OFF;
  else if(strcmp(cam_setting_em, "auto") == 0) mode = MMAL_PARAM_EXPOSUREMODE_AUTO;
  else if(strcmp(cam_setting_em, "night") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHT;
  else if(strcmp(cam_setting_em, "nightpreview") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
  else if(strcmp(cam_setting_em, "backlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
  else if(strcmp(cam_setting_em, "spotlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
  else if(strcmp(cam_setting_em, "sports") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPORTS;
  else if(strcmp(cam_setting_em, "snow") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SNOW;
  else if(strcmp(cam_setting_em, "beach") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BEACH;
  else if(strcmp(cam_setting_em, "verylong") == 0) mode = MMAL_PARAM_EXPOSUREMODE_VERYLONG;
  else if(strcmp(cam_setting_em, "fixedfps") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
  else if(strcmp(cam_setting_em, "antishake") == 0) mode = MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
  else if(strcmp(cam_setting_em, "fireworks") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
  else error("Invalid exposure mode");
  MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, mode};
  status = mmal_port_parameter_set(camera->control, &exp_mode.hdr);
  if(status != MMAL_SUCCESS) error("Could not set exposure mode");
}

void cam_set_wb () {
  MMAL_PARAM_AWBMODE_T awb_mode;
  if(strcmp(cam_setting_wb, "off") == 0) awb_mode = MMAL_PARAM_AWBMODE_OFF;
  else if(strcmp(cam_setting_wb, "auto") == 0) awb_mode = MMAL_PARAM_AWBMODE_AUTO;
  else if(strcmp(cam_setting_wb, "sun") == 0) awb_mode = MMAL_PARAM_AWBMODE_SUNLIGHT;
  else if(strcmp(cam_setting_wb, "cloudy") == 0) awb_mode = MMAL_PARAM_AWBMODE_CLOUDY;
  else if(strcmp(cam_setting_wb, "shade") == 0) awb_mode = MMAL_PARAM_AWBMODE_SHADE;
  else if(strcmp(cam_setting_wb, "tungsten") == 0) awb_mode = MMAL_PARAM_AWBMODE_TUNGSTEN;
  else if(strcmp(cam_setting_wb, "fluorescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLUORESCENT;
  else if(strcmp(cam_setting_wb, "incandescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_INCANDESCENT;
  else if(strcmp(cam_setting_wb, "flash") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLASH;
  else if(strcmp(cam_setting_wb, "horizon") == 0) awb_mode = MMAL_PARAM_AWBMODE_HORIZON;
  else error("Invalid white balance");
  MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE,sizeof(param)}, awb_mode};
  status = mmal_port_parameter_set(camera->control, &param.hdr);
  if(status != MMAL_SUCCESS) error("Could not set white balance");
}

void cam_set_mm () {
  MMAL_PARAM_EXPOSUREMETERINGMODE_T m_mode;
  if(strcmp(cam_setting_mm, "average") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
  else if(strcmp(cam_setting_mm, "spot") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
  else if(strcmp(cam_setting_mm, "backlit") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
  else if(strcmp(cam_setting_mm, "matrix") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
  else error("Invalid metering mode");
  MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {{MMAL_PARAMETER_EXP_METERING_MODE,sizeof(meter_mode)}, m_mode};
  status = mmal_port_parameter_set(camera->control, &meter_mode.hdr);
  if(status != MMAL_SUCCESS) error("Could not set metering mode");
}

void cam_set_ie () {
  MMAL_PARAM_IMAGEFX_T imageFX;
  if(strcmp(cam_setting_ie, "none") == 0) imageFX = MMAL_PARAM_IMAGEFX_NONE;
  else if(strcmp(cam_setting_ie, "negative") == 0) imageFX = MMAL_PARAM_IMAGEFX_NEGATIVE;
  else if(strcmp(cam_setting_ie, "solarise") == 0) imageFX = MMAL_PARAM_IMAGEFX_SOLARIZE;
  else if(strcmp(cam_setting_ie, "sketch") == 0) imageFX = MMAL_PARAM_IMAGEFX_SKETCH;
  else if(strcmp(cam_setting_ie, "denoise") == 0) imageFX = MMAL_PARAM_IMAGEFX_DENOISE;
  else if(strcmp(cam_setting_ie, "emboss") == 0) imageFX = MMAL_PARAM_IMAGEFX_EMBOSS;
  else if(strcmp(cam_setting_ie, "oilpaint") == 0) imageFX = MMAL_PARAM_IMAGEFX_OILPAINT;
  else if(strcmp(cam_setting_ie, "hatch") == 0) imageFX = MMAL_PARAM_IMAGEFX_HATCH;
  else if(strcmp(cam_setting_ie, "gpen") == 0) imageFX = MMAL_PARAM_IMAGEFX_GPEN;
  else if(strcmp(cam_setting_ie, "pastel") == 0) imageFX = MMAL_PARAM_IMAGEFX_PASTEL;
  else if(strcmp(cam_setting_ie, "watercolour") == 0) imageFX = MMAL_PARAM_IMAGEFX_WATERCOLOUR;
  else if(strcmp(cam_setting_ie, "film") == 0) imageFX = MMAL_PARAM_IMAGEFX_FILM;
  else if(strcmp(cam_setting_ie, "blur") == 0) imageFX = MMAL_PARAM_IMAGEFX_BLUR;
  else if(strcmp(cam_setting_ie, "saturation") == 0) imageFX = MMAL_PARAM_IMAGEFX_SATURATION;
  else if(strcmp(cam_setting_ie, "colourswap") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURSWAP;
  else if(strcmp(cam_setting_ie, "washedout") == 0) imageFX = MMAL_PARAM_IMAGEFX_WASHEDOUT;
  else if(strcmp(cam_setting_ie, "posterise") == 0) imageFX = MMAL_PARAM_IMAGEFX_POSTERISE;
  else if(strcmp(cam_setting_ie, "colourpoint") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURPOINT;
  else if(strcmp(cam_setting_ie, "colourbalance") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURBALANCE;
  else if(strcmp(cam_setting_ie, "cartoon") == 0) imageFX = MMAL_PARAM_IMAGEFX_CARTOON;
  else error("Invalid image effect");
  MMAL_PARAMETER_IMAGEFX_T imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, imageFX};
  status = mmal_port_parameter_set(camera->control, &imgFX.hdr);
  if(status != MMAL_SUCCESS) error("Could not set image effect");
}

void cam_set_ce () {
  MMAL_PARAMETER_COLOURFX_T colfx = {{MMAL_PARAMETER_COLOUR_EFFECT,sizeof(colfx)}, 0, 0, 0};
  colfx.enable = cam_setting_ce_en;
  colfx.u = cam_setting_ce_u;
  colfx.v = cam_setting_ce_v;
  status = mmal_port_parameter_set(camera->control, &colfx.hdr);
  if(status != MMAL_SUCCESS) error("Could not set exposure compensation");
}

void cam_set_rotation () {
  status = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
  if(status != MMAL_SUCCESS) error("Could not set rotation (0)");
  status = mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
  if(status != MMAL_SUCCESS) error("Could not set rotation (1)");
  status = mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
  if(status != MMAL_SUCCESS) error("Could not set rotation (2)");
}

void cam_set_flip () {
  MMAL_PARAMETER_MIRROR_T mirror = {{MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};
  if (cam_setting_hflip && cam_setting_vflip) mirror.value = MMAL_PARAM_MIRROR_BOTH;
  else if (cam_setting_hflip) mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
  else if (cam_setting_vflip) mirror.value = MMAL_PARAM_MIRROR_VERTICAL;
  status = mmal_port_parameter_set(camera->output[0], &mirror.hdr);
  if(status != MMAL_SUCCESS) error("Could not set flip (0)");
  status = mmal_port_parameter_set(camera->output[1], &mirror.hdr);
  if(status != MMAL_SUCCESS) error("Could not set flip (1)");
  status = mmal_port_parameter_set(camera->output[2], &mirror.hdr);
  if(status != MMAL_SUCCESS) error("Could not set flip (2)");
}

void cam_set_roi () {
  MMAL_PARAMETER_INPUT_CROP_T crop = {{MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)}};
  crop.rect.x = cam_setting_roi_x;
  crop.rect.y = cam_setting_roi_y;
  crop.rect.width = cam_setting_roi_w;
  crop.rect.height = cam_setting_roi_h;
  status = mmal_port_parameter_set(camera->control, &crop.hdr);
  if(status != MMAL_SUCCESS) error("Could not set sensor area");
}

void cam_set_ss () {
  status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, cam_setting_ss);
  if(status != MMAL_SUCCESS) error("Could not set shutter speed");
}

void cam_set_quality () {
  status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, cam_setting_quality);
  if(status != MMAL_SUCCESS) error("Could not set quality");
}

void cam_set_raw () {
  status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_ENABLE_RAW_CAPTURE, cam_setting_raw);
  if(status != MMAL_SUCCESS) error("Could not set raw layer");
}

void cam_set_bitrate () {
  h264encoder->output[0]->format->bitrate = cam_setting_bitrate;
  status = mmal_port_format_commit(h264encoder->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set bitrate");
}

void cam_set_annotation () {
  char *filename_temp;
  MMAL_PARAMETER_CAMERA_ANNOTATE_V2_T anno = {{MMAL_PARAMETER_ANNOTATE, sizeof(MMAL_PARAMETER_CAMERA_ANNOTATE_V2_T)}};

  if(cam_setting_annotation != 0) {
    currTime = time(NULL);
    localTime = localtime (&currTime);
    asprintf(&filename_temp, cam_setting_annotation, localTime->tm_year+1900, localTime->tm_mon+1, localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);
    anno.enable = 1;
    strcpy(anno.text, filename_temp);
    free(filename_temp);
  }
  else {
    anno.enable = 0;
  }
  anno.show_shutter = 0;
  anno.show_analog_gain = 0;
  anno.show_lens = 0;
  anno.show_caf = 0;
  anno.show_motion = 0;
  anno.black_text_background = cam_setting_annback;

  status = mmal_port_parameter_set(camera->control, &anno.hdr);
  if(status != MMAL_SUCCESS) error("Could not set annotation");
}

void start_all (void) {
  if (verbose) printf("Autostart...\n");

  MMAL_ES_FORMAT_T *format;
  int max, i;
  
  //
  // create camera
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
  if(status != MMAL_SUCCESS) error("Could not create camera");
  status = mmal_port_enable(camera->control, camera_control_callback);
  if(status != MMAL_SUCCESS) error("Could not enable camera control port");

  MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
    {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
    .max_stills_w = image_width,
    .max_stills_h = image_height,
    .stills_yuv422 = 0,
    .one_shot_stills = 1,
    .max_preview_video_w = video_width,
    .max_preview_video_h = video_height,
    .num_preview_video_frames = 3,
    .stills_capture_circular_buffer_height = 0,
    .fast_preview_resume = 0,
    .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
  };
  mmal_port_parameter_set(camera->control, &cam_config.hdr);
  
  format = camera->output[0]->format;
  format->es->video.width = video_width;
  format->es->video.height = video_height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = video_width;
  format->es->video.crop.height = video_height;
  format->es->video.frame_rate.num = 0;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[0]);
  if(status != MMAL_SUCCESS) error("Coult not set preview format");

  format = camera->output[1]->format;
  format->encoding_variant = MMAL_ENCODING_I420;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = video_width;
  format->es->video.height = video_height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = video_width;
  format->es->video.crop.height = video_height;
  format->es->video.frame_rate.num = video_fps;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[1]);
  if(status != MMAL_SUCCESS) error("Could not set video format");
  if(camera->output[1]->buffer_num < 3)
    camera->output[1]->buffer_num = 3;
  
  format = camera->output[2]->format;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = image_width;
  format->es->video.height = image_height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = image_width;
  format->es->video.crop.height = image_height;
  format->es->video.frame_rate.num = 0;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[2]);
  if(status != MMAL_SUCCESS) error("Could not set still format");
  if(camera->output[2]->buffer_num < 3)
    camera->output[2]->buffer_num = 3;

  status = mmal_component_enable(camera);
  if(status != MMAL_SUCCESS) error("Could not enable camera");

  //
  // create jpeg-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder");

  mmal_format_copy(jpegencoder->output[0]->format, jpegencoder->input[0]->format);
  jpegencoder->output[0]->format->encoding = MMAL_ENCODING_JPEG;
  jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_recommended;
  if(jpegencoder->output[0]->buffer_size < jpegencoder->output[0]->buffer_size_min)
    jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_min;
  jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_recommended;
  if(jpegencoder->output[0]->buffer_num < jpegencoder->output[0]->buffer_num_min)
    jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_min;
  status = mmal_port_format_commit(jpegencoder->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image format");
  status = mmal_port_parameter_set_uint32(jpegencoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, quality);
  if(status != MMAL_SUCCESS) error("Could not set jpeg quality");

  status = mmal_component_enable(jpegencoder);
  if(status != MMAL_SUCCESS) error("Could not enable image encoder");
  pool_jpegencoder = mmal_port_pool_create(jpegencoder->output[0], jpegencoder->output[0]->buffer_num, jpegencoder->output[0]->buffer_size);
  if(!pool_jpegencoder) error("Could not create image buffer pool");

  //
  // create second jpeg-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder2);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder 2");

  mmal_format_copy(jpegencoder2->output[0]->format, jpegencoder2->input[0]->format);
  jpegencoder2->output[0]->format->encoding = MMAL_ENCODING_JPEG;
  jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_recommended;
  if(jpegencoder2->output[0]->buffer_size < jpegencoder2->output[0]->buffer_size_min)
    jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_min;
  jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_recommended;
  if(jpegencoder2->output[0]->buffer_num < jpegencoder2->output[0]->buffer_num_min)
    jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_min;
  status = mmal_port_format_commit(jpegencoder2->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image format 2");
  status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, 85);
  if(status != MMAL_SUCCESS) error("Could not set jpeg quality 2");

  status = mmal_component_enable(jpegencoder2);
  if(status != MMAL_SUCCESS) error("Could not enable image encoder 2");
  pool_jpegencoder2 = mmal_port_pool_create(jpegencoder2->output[0], jpegencoder2->output[0]->buffer_num, jpegencoder2->output[0]->buffer_size);
  if(!pool_jpegencoder2) error("Could not create image buffer pool 2");

  //
  // create h264-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &h264encoder);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create video encoder");

  mmal_format_copy(h264encoder->output[0]->format, h264encoder->input[0]->format);
  h264encoder->output[0]->format->encoding = MMAL_ENCODING_H264;
  h264encoder->output[0]->format->bitrate = 17000000;
  h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_recommended;
  if(h264encoder->output[0]->buffer_size < h264encoder->output[0]->buffer_size_min)
    h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_min;
  h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_recommended;
  if(h264encoder->output[0]->buffer_num < h264encoder->output[0]->buffer_num_min)
    h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_min;
  h264encoder->output[0]->format->es->video.frame_rate.num = 0;
  h264encoder->output[0]->format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(h264encoder->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set video format");

  MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param2)}, 25};
  status = mmal_port_parameter_set(h264encoder->output[0], &param2.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video quantisation");

  MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_QP_P, sizeof(param3)}, 31};
  status = mmal_port_parameter_set(h264encoder->output[0], &param3.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video quantisation");

  MMAL_PARAMETER_VIDEO_PROFILE_T param4;
  param4.hdr.id = MMAL_PARAMETER_PROFILE;
  param4.hdr.size = sizeof(param4);
  param4.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
  param4.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;
  status = mmal_port_parameter_set(h264encoder->output[0], &param4.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video port format");

  status = mmal_port_parameter_set_boolean(h264encoder->input[0], MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
  if(status != MMAL_SUCCESS) error("Could not set immutable flag");

  status = mmal_port_parameter_set_boolean(h264encoder->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0);
  if(status != MMAL_SUCCESS) error("Could not set inline flag");

  //
  // create image-resizer
  //
  unsigned int height_temp = (unsigned long int)width*video_height/video_width;
  height_temp -= height_temp%16;
  status = mmal_component_create("vc.ril.resize", &resizer);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image resizer");
  
  format = resizer->output[0]->format;
  format->es->video.width = width;
  format->es->video.height = height_temp;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = width;
  format->es->video.crop.height = height_temp;
  format->es->video.frame_rate.num = 30;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(resizer->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image resizer output");
  
  status = mmal_component_enable(resizer);
  if(status != MMAL_SUCCESS) error("Could not enable image resizer");

  //
  // connect
  //
  status = mmal_connection_create(&con_cam_res, camera->output[0], resizer->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection camera -> resizer");
  status = mmal_connection_enable(con_cam_res);
  if(status != MMAL_SUCCESS) error("Could not enable connection camera -> resizer");
  
  status = mmal_connection_create(&con_res_jpeg, resizer->output[0], jpegencoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection resizer -> encoder");
  status = mmal_connection_enable(con_res_jpeg);
  if(status != MMAL_SUCCESS) error("Could not enable connection resizer -> encoder");

  status = mmal_port_enable(jpegencoder->output[0], jpegencoder_buffer_callback);
  if(status != MMAL_SUCCESS) error("Could not enable jpeg port");
  max = mmal_queue_length(pool_jpegencoder->queue);
  for(i=0;i<max;i++) {
    MMAL_BUFFER_HEADER_T *jpegbuffer = mmal_queue_get(pool_jpegencoder->queue);

    if(!jpegbuffer) error("Could not create jpeg buffer header");
    status = mmal_port_send_buffer(jpegencoder->output[0], jpegbuffer);
    if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port");
  }

  status = mmal_connection_create(&con_cam_jpeg, camera->output[2], jpegencoder2->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection camera -> encoder");
  status = mmal_connection_enable(con_cam_jpeg);
  if(status != MMAL_SUCCESS) error("Could not enable connection camera -> encoder");
  
  status = mmal_port_enable(jpegencoder2->output[0], jpegencoder2_buffer_callback);
  if(status != MMAL_SUCCESS) error("Could not enable jpeg port 2");
  max = mmal_queue_length(pool_jpegencoder2->queue);
  for(i=0;i<max;i++) {
    MMAL_BUFFER_HEADER_T *jpegbuffer2 = mmal_queue_get(pool_jpegencoder2->queue);

    if(!jpegbuffer2) error("Could not create jpeg buffer header 2");
    status = mmal_port_send_buffer(jpegencoder2->output[0], jpegbuffer2);
    if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port 2");
  }
  
  //
  // settings
  //
  cam_set_sharpness();
  cam_set_contrast();
  cam_set_brightness();
  cam_set_saturation();
  cam_set_iso();
  cam_set_vs();
  cam_set_ec();
  cam_set_em();
  cam_set_wb();
  cam_set_mm();
  cam_set_ie();
  cam_set_ce();
  cam_set_rotation();
  cam_set_flip();
  cam_set_roi();
  cam_set_ss();
  cam_set_quality();
  cam_set_raw();
  cam_set_bitrate();
  cam_set_annotation();

}


void stop_all (void) {

  mmal_port_disable(jpegencoder->output[0]);
  mmal_connection_destroy(con_cam_res);
  mmal_connection_destroy(con_res_jpeg);
  mmal_port_pool_destroy(jpegencoder->output[0], pool_jpegencoder);
  mmal_component_disable(jpegencoder);
  mmal_component_disable(camera);
  mmal_component_destroy(jpegencoder);
  mmal_component_destroy(h264encoder);
  mmal_component_destroy(camera);

}

void capt_img (void) {

  char *filename_temp;

  currTime = time(NULL);
  localTime = localtime (&currTime);
  asprintf(&filename_temp, jpeg2_filename, image2_cnt, localTime->tm_year+1900, localTime->tm_mon+1, localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);
  jpegoutput2_file = fopen(filename_temp, "wb");
  free(filename_temp);
  if(!jpegoutput2_file) error("Could not open/create image-file");
  status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_CAPTURE, 1);
  if(status != MMAL_SUCCESS) error("Could not start image capture");
  printf("Capturing image\n");
  if(status_filename != 0) {
    if(!timelapse) {
      status_file = fopen(status_filename, "w");
      fprintf(status_file, "image");
      fclose(status_file);
    }
  }
  capturing = 1;

}

int main (int argc, char* argv[]) {

  int i, max, fd, length;
  char readbuf[60];
  char *filename_temp, *filename_recording, *cmd_temp, *line = NULL;
  FILE *fp;
  char cfg_path[256] = "/etc/raspimjpeg";

  bcm_host_init();
  
  //
  // read arguments
  //

  for(i=1; i<argc; i++) {
    if(strcmp(argv[i], "--version") == 0) {
      printf("RaspiMJPEG Version ");
      printf(VERSION);
      printf("\n");
      exit(0);
    }
    else if(strcmp(argv[i], "--config") == 0) {
      i++;
      strcpy(cfg_path,argv[i]);
    }
    else if(strcmp(argv[i], "-v") == 0) {
	verbose = 1;
    }
    else if(strcmp(argv[i], "-ic") == 0) {
      i++;
      image2_cnt = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-vc") == 0) {
      i++;
      video_cnt = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-md") == 0) {
      motion_detection = 1;
    }
    else {
	printf("Supported arguments: \n");
	printf("--config [PATH]\n");
	printf("-ic [count]\n");
	printf("-vc [count]\n");
	printf("-md\n");
	printf("-v\n");
	error("Invalid arguments");
    }
  }

  //
  // read config file
  //
  if (verbose) printf("Reading config: %s\n",cfg_path);
  fp = fopen(cfg_path, "r");
  if(fp != NULL) {
	if (verbose) printf("Config opened\n");
    unsigned int len = 0;
    while((length = getline(&line, &len, fp)) != -1) {
      line[length-1] = 0;

      if(strncmp(line, "width ", 6) == 0) {
        width = atoi(line+6);
      }
      else if(strncmp(line, "quality ", 8) == 0) {
        quality = atoi(line+8);
      }
      else if(strncmp(line, "divider ", 8) == 0) {
        divider = atoi(line+8);
      }
      else if(strncmp(line, "preview_path ", 13) == 0) {
        asprintf(&jpeg_filename, "%s", line+13);
      }
      else if(strncmp(line, "image_path ", 11) == 0) {
        asprintf(&jpeg2_filename, "%s", line+11);
      }
      else if(strncmp(line, "video_path ", 11) == 0) {
        asprintf(&h264_filename, "%s", line+11);
      }
      else if(strncmp(line, "status_file ", 12) == 0) {
        asprintf(&status_filename, "%s", line+12);
      }
      else if(strncmp(line, "control_file ", 13) == 0) {
        asprintf(&pipe_filename, "%s", line+13);
      }
      else if(strncmp(line, "annotation ", 11) == 0) {
        asprintf(&cam_setting_annotation, "%s", line+11);
      }
      else if(strncmp(line, "anno_background ", 16) == 0) {
        if(strncmp(line+16, "true", 4) == 0) cam_setting_annback = 1;
      }
      else if(strncmp(line, "MP4Box ", 7) == 0) {
        if(strncmp(line+7, "true", 4) == 0) mp4box = 1;
      }
      else if(strncmp(line, "autostart ", 10) == 0) {
        if(strncmp(line+10, "idle", 4) == 0) {
          autostart = 0;
          idle = 1;
        }
      }
      else if(strncmp(line, "motion_detection ", 17) == 0) {
        if(strncmp(line+17, "true", 4) == 0) motion_detection = 1;
      }
      else if(strncmp(line, "sharpness ", 10) == 0) {
        cam_setting_sharpness = atoi(line+10);
      }
      else if(strncmp(line, "contrast ", 9) == 0) {
        cam_setting_contrast = atoi(line+9);
      }
      else if(strncmp(line, "brightness ", 11) == 0) {
        cam_setting_brightness = atoi(line+11);
      }
      else if(strncmp(line, "saturation ", 11) == 0) {
        cam_setting_saturation = atoi(line+11);
      }
      else if(strncmp(line, "iso ", 4) == 0) {
        cam_setting_iso = atoi(line+4);
      }
      else if(strncmp(line, "video_stabilisation ", 20) == 0) {
        if(strncmp(line+20, "true", 4) == 0) cam_setting_vs = 1;
      }
      else if(strncmp(line, "exposure_compensation ", 22) == 0) {
        cam_setting_ec = atoi(line+22);
      }
      else if(strncmp(line, "exposure_mode ", 14) == 0) {
        sprintf(cam_setting_em, "%s", line+14);
      }
      else if(strncmp(line, "white_balance ", 14) == 0) {
        sprintf(cam_setting_wb, "%s", line+14);
      }
      else if(strncmp(line, "metering_mode ", 14) == 0) {
        sprintf(cam_setting_mm, "%s", line+14);
      }
      else if(strncmp(line, "image_effect ", 13) == 0) {
        sprintf(cam_setting_ie, "%s", line+13);
      }
      else if(strncmp(line, "colour_effect_en ", 17) == 0) {
        if(strncmp(line+17, "true", 4) == 0) cam_setting_ce_en = 1;
      }
      else if(strncmp(line, "colour_effect_u ", 16) == 0) {
        cam_setting_ce_u = atoi(line+16);
      }
      else if(strncmp(line, "colour_effect_v ", 16) == 0) {
        cam_setting_ce_v = atoi(line+16);
      }
      else if(strncmp(line, "rotation ", 9) == 0) {
        cam_setting_rotation = atoi(line+9);
      }
      else if(strncmp(line, "hflip ", 6) == 0) {
        if(strncmp(line+6, "true", 4) == 0) cam_setting_hflip = 1;
      }
      else if(strncmp(line, "vflip ", 6) == 0) {
        if(strncmp(line+6, "true", 4) == 0) cam_setting_vflip = 1;
      }
      else if(strncmp(line, "sensor_region_x ", 16) == 0) {
        cam_setting_roi_x = strtoull(line+16, NULL, 0);
      }
      else if(strncmp(line, "sensor_region_y ", 16) == 0) {
        cam_setting_roi_y = strtoull(line+16, NULL, 0);
      }
      else if(strncmp(line, "sensor_region_w ", 16) == 0) {
        cam_setting_roi_w = strtoull(line+16, NULL, 0);
      }
      else if(strncmp(line, "sensor_region_h ", 16) == 0) {
        cam_setting_roi_h = strtoull(line+16, NULL, 0);
      }
      else if(strncmp(line, "shutter_speed ", 14) == 0) {
        cam_setting_ss = strtoull(line+14, NULL, 0);
      }
      else if(strncmp(line, "image_quality ", 14) == 0) {
        cam_setting_quality = atoi(line+14);
      }
      else if(strncmp(line, "raw_layer ", 10) == 0) {
        if(strncmp(line+10, "true", 4) == 0) cam_setting_raw = 1;
      }
      else if(strncmp(line, "video_bitrate ", 14) == 0) {
        cam_setting_bitrate = strtoull(line+14, NULL, 0);
      }
      else if(strncmp(line, "video_width ", 12) == 0) {
        video_width = atoi(line+12);
      }
      else if(strncmp(line, "video_height ", 13) == 0) {
        video_height = atoi(line+13);
      }
      else if(strncmp(line, "video_fps ", 10) == 0) {
        video_fps = atoi(line+10);
      }
      else if(strncmp(line, "MP4Box_fps ", 11) == 0) {
        MP4Box_fps = atoi(line+11);
      }
      else if(strncmp(line, "image_width ", 12) == 0) {
        image_width = atoi(line+12);
      }
      else if(strncmp(line, "image_height ", 13) == 0) {
        image_height = atoi(line+13);
      }
      else if(strncmp(line, "#", 1) == 0) {
      }
      else if(strcmp(line, "") == 0) {
      }
      else {
        printf("Unknown command in config file: %s\n", line);
        error("Invalid config file");
      }
 
    }
    if(line) free(line);
  }
  
  if (verbose) printf("Reading config - DONE\n");
  //
  // init
  //
  if(autostart) start_all();
  if(motion_detection) {
    if(system("motion") == -1) error("Could not start Motion");
  }

  //
  // run
  //
  if(autostart) {
    if(pipe_filename != 0) printf("MJPEG streaming, ready to receive commands\n");
    else printf("MJPEG streaming\n");
  }
  else {
    if(pipe_filename != 0) printf("MJPEG idle, ready to receive commands\n");
    else printf("MJPEG idle\n");
  }

  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = term;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);
  
  if(status_filename != 0) {
    status_file = fopen(status_filename, "w");
    if(!status_file) error("Could not open/create status-file");
    if(autostart) {
      if(!motion_detection) {
        fprintf(status_file, "ready");
      }
      else fprintf(status_file, "md_ready");
    }
    else fprintf(status_file, "halted");
    fclose(status_file);
  }
  
  while(running) {
    if(pipe_filename != 0) {

      fd = open(pipe_filename, O_RDONLY | O_NONBLOCK);
      if(fd < 0) error("Could not open PIPE");
      fcntl(fd, F_SETFL, 0);
      length = read(fd, readbuf, 60);
      close(fd);

      if(length) {
        if((readbuf[0]=='c') && (readbuf[1]=='a')) {
          if(readbuf[3]=='1') {
            if(!capturing) {
              status = mmal_component_enable(h264encoder);
              if(status != MMAL_SUCCESS) error("Could not enable h264encoder");
              pool_h264encoder = mmal_port_pool_create(h264encoder->output[0], h264encoder->output[0]->buffer_num, h264encoder->output[0]->buffer_size);
              if(!pool_h264encoder) error("Could not create pool");
              status = mmal_connection_create(&con_cam_h264, camera->output[1], h264encoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
              if(status != MMAL_SUCCESS) error("Could not create connecton camera -> video converter");
              status = mmal_connection_enable(con_cam_h264);
              if(status != MMAL_SUCCESS) error("Could not enable connection camera -> video converter");
              currTime = time(NULL);
              localTime = localtime (&currTime);
              if(mp4box) {
                asprintf(&filename_recording, h264_filename, video_cnt, localTime->tm_year+1900, localTime->tm_mon+1, localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);
                asprintf(&filename_temp, "%s.h264", filename_recording);
              }
              else {
                asprintf(&filename_temp, h264_filename, video_cnt, localTime->tm_year+1900, localTime->tm_mon+1, localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);
              }
              h264output_file = fopen(filename_temp, "wb");
              free(filename_temp);
              if(!h264output_file) error("Could not open/create video-file");
              status = mmal_port_enable(h264encoder->output[0], h264encoder_buffer_callback);
              if(status != MMAL_SUCCESS) error("Could not enable video port");
              max = mmal_queue_length(pool_h264encoder->queue);
              for(i=0;i<max;i++) {
                MMAL_BUFFER_HEADER_T *h264buffer = mmal_queue_get(pool_h264encoder->queue);
                if(!h264buffer) error("Could not create video pool header");
                status = mmal_port_send_buffer(h264encoder->output[0], h264buffer);
                if(status != MMAL_SUCCESS) error("Could not send buffers to video port");
              }
              mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 1);
              if(status != MMAL_SUCCESS) error("Could not start capture");
              printf("Capturing started\n");
              if(status_filename != 0) {
                status_file = fopen(status_filename, "w");
                if(!motion_detection) fprintf(status_file, "video");
                else fprintf(status_file, "md_video");
                fclose(status_file);
              }
              capturing = 1;
            }
          }
          else {
            if(capturing) {
              mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 0);
              if(status != MMAL_SUCCESS) error("Could not stop capture");
              status = mmal_port_disable(h264encoder->output[0]);
              if(status != MMAL_SUCCESS) error("Could not disable video port");
              status = mmal_connection_destroy(con_cam_h264);
              if(status != MMAL_SUCCESS) error("Could not destroy connection camera -> video encoder");
              mmal_port_pool_destroy(h264encoder->output[0], pool_h264encoder);
              if(status != MMAL_SUCCESS) error("Could not destroy video buffer pool");
              status = mmal_component_disable(h264encoder);
              if(status != MMAL_SUCCESS) error("Could not disable video converter");
              fclose(h264output_file);
              h264output_file = NULL;
              printf("Capturing stopped\n");
              if(mp4box) {
                printf("Boxing started\n");
                status_file = fopen(status_filename, "w");
                if(!motion_detection) fprintf(status_file, "boxing");
                else fprintf(status_file, "md_boxing");
                fclose(status_file);
                asprintf(&cmd_temp, "MP4Box -fps %i -add %s.h264 %s > /dev/null", MP4Box_fps, filename_recording, filename_recording);
                if(system(cmd_temp) == -1) error("Could not start MP4Box");
                asprintf(&filename_temp, "%s.h264", filename_recording);
                remove(filename_temp);
                free(filename_temp);
                free(filename_recording);
                free(cmd_temp);
                printf("Boxing stopped\n");
              }
              video_cnt++;
              if(status_filename != 0) {
                status_file = fopen(status_filename, "w");
                if(!motion_detection) fprintf(status_file, "ready");
                else fprintf(status_file, "md_ready");
                fclose(status_file);
              }
              capturing = 0;
            }
          }
        }
        else if((readbuf[0]=='i') && (readbuf[1]=='m')) {
          capt_img();
        }
        else if((readbuf[0]=='t') && (readbuf[1]=='l')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          time_between_pic = atoi(readbuf);
          if(time_between_pic) {
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "timelapse");
              fclose(status_file);
            }
            timelapse = 1;
            printf("Timelapse started\n");
          }
          else {
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
            timelapse = 0;
            printf("Timelapse stopped\n");
          }
        }
        else if((readbuf[0]=='p') && (readbuf[1]=='x')) {
          stop_all();
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[7] = 0;
          readbuf[12] = 0;
          readbuf[15] = 0;
          readbuf[18] = 0;
          readbuf[23] = 0;
          readbuf[length] = 0;
          video_width = atoi(readbuf);
          video_height = atoi(readbuf+8);
          video_fps = atoi(readbuf+13);
          MP4Box_fps = atoi(readbuf+16);
          image_width = atoi(readbuf+19);
          image_height = atoi(readbuf+24);
          start_all();
          printf("Changed resolutions and framerates\n");
        }
        else if((readbuf[0]=='a') && (readbuf[1]=='n')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          asprintf(&cam_setting_annotation, "%s", readbuf+3);
          printf("Annotation changed\n");
        }
        else if((readbuf[0]=='a') && (readbuf[1]=='b')) {
          if(readbuf[3] == '0') {
            cam_setting_annback = 0;
          }
          else {
            cam_setting_annback = 1;
          }
          printf("Annotation background changed\n");
        }
        else if((readbuf[0]=='s') && (readbuf[1]=='h')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_sharpness = atoi(readbuf);
          cam_set_sharpness();
          printf("Sharpness: %d\n", cam_setting_sharpness);
        }
        else if((readbuf[0]=='c') && (readbuf[1]=='o')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_contrast = atoi(readbuf);
          cam_set_contrast();
          printf("Contrast: %d\n", cam_setting_contrast);
        }
        else if((readbuf[0]=='b') && (readbuf[1]=='r')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_brightness = atoi(readbuf);
          cam_set_brightness();
          printf("Brightness: %d\n", cam_setting_brightness);
        }
        else if((readbuf[0]=='s') && (readbuf[1]=='a')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_saturation = atoi(readbuf);
          cam_set_saturation();
          printf("Saturation: %d\n", cam_setting_saturation);
        }
        else if((readbuf[0]=='i') && (readbuf[1]=='s')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_iso = atoi(readbuf);
          cam_set_iso();
          printf("ISO: %d\n", cam_setting_iso);
        }
        else if((readbuf[0]=='v') && (readbuf[1]=='s')) {
          if(readbuf[3]=='1') cam_setting_vs = 1;
          else cam_setting_vs = 0;
          cam_set_vs();
          printf("Changed video stabilisation\n");
        }
        else if((readbuf[0]=='r') && (readbuf[1]=='l')) {
          if(readbuf[3]=='1') cam_setting_raw = 1;
          else cam_setting_raw = 0;
          cam_set_raw();
          printf("Changed raw layer\n");
        }
        else if((readbuf[0]=='e') && (readbuf[1]=='c')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_ec = atoi(readbuf);
          cam_set_ec();
          printf("Exposure compensation: %d\n", cam_setting_ec);
        }
        else if((readbuf[0]=='e') && (readbuf[1]=='m')) {
          readbuf[length] = 0;
          sprintf(cam_setting_em, "%s", readbuf+3);
          cam_set_em();
          printf("Exposure mode changed\n");
        }
        else if((readbuf[0]=='w') && (readbuf[1]=='b')) {
          readbuf[length] = 0;
          sprintf(cam_setting_wb, "%s", readbuf+3);
          cam_set_wb();
          printf("White balance changed\n");
        }
        else if((readbuf[0]=='m') && (readbuf[1]=='m')) {
          readbuf[length] = 0;
          sprintf(cam_setting_mm, "%s", readbuf+3);
          cam_set_mm();
          printf("Metering mode changed\n");
        }
        else if((readbuf[0]=='i') && (readbuf[1]=='e')) {
          readbuf[length] = 0;
          sprintf(cam_setting_ie, "%s", readbuf+3);
          cam_set_ie();
          printf("Image effect changed\n");
        }
        else if((readbuf[0]=='c') && (readbuf[1]=='e')) {
          readbuf[4] = 0;
          readbuf[8] = 0;
          readbuf[length] = 0;
          cam_setting_ce_en = atoi(readbuf+3);
          cam_setting_ce_u = atoi(readbuf+5);
          cam_setting_ce_v = atoi(readbuf+9);
          cam_set_ce();
          printf("Colour effect changed\n");
        }
        else if((readbuf[0]=='r') && (readbuf[1]=='o')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_rotation = atoi(readbuf);
          cam_set_rotation();
          printf("Rotation: %d\n", cam_setting_rotation);
        }
        else if((readbuf[0]=='f') && (readbuf[1]=='l')) {
          if(readbuf[3] == '0') {
            cam_setting_hflip = 0;
            cam_setting_vflip = 0;
          }
          else if(readbuf[3] == '1') {
            cam_setting_hflip = 1;
            cam_setting_vflip = 0;
          }
          else if(readbuf[3] == '2') {
            cam_setting_hflip = 0;
            cam_setting_vflip = 1;
          }
          else {
            cam_setting_hflip = 1;
            cam_setting_vflip = 1;
          }
          cam_set_flip();
          printf("Flip changed\n");
        }
        else if((readbuf[0]=='r') && (readbuf[1]=='i')) {
          readbuf[8] = 0;
          readbuf[14] = 0;
          readbuf[20] = 0;
          readbuf[length] = 0;
          cam_setting_roi_x = strtoull(readbuf+3, NULL, 0);
          cam_setting_roi_y = strtoull(readbuf+9, NULL, 0);
          cam_setting_roi_w = strtoull(readbuf+15, NULL, 0);
          cam_setting_roi_h = strtoull(readbuf+21, NULL, 0);
          cam_set_roi();
          printf("Changed Sensor Region\n");
        }
        else if((readbuf[0]=='s') && (readbuf[1]=='s')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_ss = strtoull(readbuf, NULL, 0);
          cam_set_ss();
          printf("Shutter Speed: %lu\n", cam_setting_ss);
        }
        else if((readbuf[0]=='q') && (readbuf[1]=='u')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_quality = atoi(readbuf);
          cam_set_quality();
          printf("Quality: %d\n", cam_setting_quality);
        }
        else if((readbuf[0]=='b') && (readbuf[1]=='i')) {
          readbuf[0] = ' ';
          readbuf[1] = ' ';
          readbuf[length] = 0;
          cam_setting_bitrate = strtoull(readbuf, NULL, 0);
          cam_set_bitrate();
          printf("Bitrate: %lu\n", cam_setting_bitrate);
        }
        else if((readbuf[0]=='r') && (readbuf[1]=='u')) {
          if(readbuf[3]=='0') {
            stop_all();
            idle = 1;
            printf("Stream halted\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "halted");
              fclose(status_file);
            }
          }
          else {
            start_all();
            idle = 0;
            printf("Stream continued\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
          }
        }
        else if((readbuf[0]=='m') && (readbuf[1]=='d')) {
          if(readbuf[3]=='0') {
            motion_detection = 0;
            if(system("killall motion") == -1) error("Could not stop Motion");
            printf("Motion detection stopped\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
          }
          else {
            motion_detection = 1;
            if(system("motion") == -1) error("Could not start Motion");
            printf("Motion detection started\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "md_ready");
              fclose(status_file);
            }
          }
        }
      }

    }
    if(timelapse) {
      tl_cnt++;
      if(tl_cnt >= time_between_pic) {
        if(capturing == 0) {
          capt_img();
          tl_cnt = 0;
        }
      }
    }
    usleep(100000);
  }
  
  printf("SIGINT/SIGTERM received, stopping\n");
  
  //
  // tidy up
  //
  if(!idle) stop_all();

  return 0;

}
