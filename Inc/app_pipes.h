#ifndef APP_PIPES_H
#define APP_PIPES_H
/* Applies the GUI crop/decimation/downsize config to DCMIPP pipe1 & pipe2. */
void dcmipp_apply_detect_config(void);

/* One snapshot on pipe1+pipe2 using whatever config is already programmed
 * (dcmipp_apply_detect_config) -- DCMIPP is NOT reconfigured. Returns 0 on
 * success, -1 on capture timeout. */
int capture_detect_frame(void);
#endif /* APP_PIPES_H */
