# Build Options

Some features are enabled using build options or by using `app_config.h`:

- [Available streams](#available-streams)

This documentation explains those features and how to modify them.

## Available streams

You can define streams that camera application can display using IMG_STREAMS define.
For each stream you can define

- format
- size
- frame rate

1. Open [app_config.h](../Inc/app_config.h).

2. Change the `IMG_STREAMS` define for your board:

```c
#define IMG_STREAMS {{UVCL_PAYLOAD_JPEG             ,  224, 224, 30}, \
                     {UVCL_PAYLOAD_UNCOMPRESSED_YUY2, 1280, 720, 10}};
```

3. Change the `MAX_IMG_FRAME_SIZE` define for your board:

```c
#define MAX_IMG_FRAME_SIZE (1280 * 720 * 2)
```

### Constraints on IMG_STREAMS

- The maximum number of items in `IMG_STREAMS` is limited to `UVCL_MAX_STREAM_CONF_NB`, which is set to 8 by default.
- Possible values for the UVC payload are listed in the [uvcl.h](../Lib/uvcl/Inc/uvcl.h) file.
- The stride width of dcmipp must be a multiple of 16. The stride width is calculated as `width * dcmipp_capture_format_byte_per_pel`.
  For `UVCL_PAYLOAD_JPEG` or `UVCL_PAYLOAD_FB_JPEG`, use a value of 2 for `dcmipp_capture_format_byte_per_pel`.