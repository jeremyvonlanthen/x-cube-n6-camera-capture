#ifndef APP_RECORD_H
#define APP_RECORD_H
/* MONO JPEG snapshot into hires_jpeg_buffer (no SD access). Call in
 * RECORD_MODE_INIT.
 * height: 4:3 photo height (width derived), up to the sensor full resolution.
 * Returns the encoded length (> 0), or <= 0 on failure. */
int record_snapshot_to_ram(int height);
/* Writes the JPEG captured by record_snapshot_to_ram() into fname on the SD
 * card. Call in MULTIMEDIA_STORAGE, once SD_CARD_INIT has succeeded.
 * Returns 0 on success. */
int record_snapshot_flush_to_sd(const char *fname);
/* Prepares the camera for H264 recording (RGB565 reconfig + VENC/encoder init +
 * double-buffer start + AE warmup). Call in VIDEO_CAPTURE.
 * height: 4:3 video height (width derived), max H264_MAX_HEIGHT. */
void setup_record_h264(int height);
/* Captures+encodes rec_duration seconds of H264 video into a RAM store (uses
 * the capture started by setup_record_h264(); no SD access). Call in
 * VIDEO_CAPTURE, right after setup_record_h264().
 * height: must match setup_record_h264(); rec_duration: seconds.
 * Returns the number of frames captured (> 0), or -1 if none were. */
int record_h264_to_ram(int height, int rec_duration);
/* Muxes the RAM store filled by record_h264_to_ram() into fname on
 * the SD card. Call in MULTIMEDIA_STORAGE, once SD_CARD_INIT has succeeded.
 * Returns 0 on success. */
int record_h264_flush_to_sd(const char *fname);
#endif /* APP_RECORD_H */
