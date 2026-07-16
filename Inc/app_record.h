#ifndef APP_RECORD_H
#define APP_RECORD_H
/* COLOR JPEG snapshot to the SD card (<timestamp>.jpg).
 * height: 4:3 photo height (width derived), up to the sensor full resolution. */
void record_jpeg_sd(const char *timestamp, int height);
/* Records H264 video into <timestamp>.mp4 on the SD card.
 * height: 4:3 video height (width derived), max H264_MAX_HEIGHT. */
void record_h264_sd(const char *timestamp, int height);
#endif /* APP_RECORD_H */
