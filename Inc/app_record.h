#ifndef APP_RECORD_H
#define APP_RECORD_H
/* Full-res COLOR JPEG snapshot to the SD card (IMG named <timestamp>.jpg). */
void record_jpeg_sd(const char *timestamp);
/* Records H264_RECORD_SECONDS of 720p H264 into <timestamp>.mp4 on the SD. */
void record_h264_sd(const char *timestamp);
#endif /* APP_RECORD_H */
