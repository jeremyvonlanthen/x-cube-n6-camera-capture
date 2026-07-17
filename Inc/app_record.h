#ifndef APP_RECORD_H
#define APP_RECORD_H
/* COLOR JPEG snapshot to the SD card (<timestamp>.jpg).
 * height: 4:3 photo height (width derived), up to the sensor full resolution. */
void record_jpeg_sd(const char *timestamp, int height);
/* Prepares the camera for H264 recording (RGB565 reconfig + VENC/encoder init +
 * double-buffer start + AE warmup). Call in RECORD_MODE_WARMUP.
 * height: 4:3 video height (width derived), max H264_MAX_HEIGHT. */
void record_camera_setup(int height);
/* Records the H264 video into <timestamp>.mp4 (uses the capture started by
 * record_camera_setup()). Call in RECORDING, right after RECORD_MODE_WARMUP.
 * height: must match record_camera_setup(); rec_duration: seconds. */
void record_h264_run(const char *timestamp, int height, int rec_duration);
#endif /* APP_RECORD_H */
