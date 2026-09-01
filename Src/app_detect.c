/**
 ******************************************************************************
 * @file    app_detect.c
 * @brief   Statistical + frame-diff motion detector on the two detect-mode
 *          DCMIPP pipes (MVE-accelerated), ported from TB_LDS/Src/app.c
 *          (DETECTION state) -- see app_detect.h.
 ******************************************************************************
 */
#include "app_detect.h"
#include "app_shared.h"
#include "app_pipes.h"

#include <arm_mve.h>
#include <math.h>
#include <string.h>

#define MAX_GREY ((uint8_t)255)

/* ==========================================================================
 * Per-pipe statistics buffers (axisram_alloc pool, see DETECT_Init)
 * ========================================================================== */
static uint16_t width_pipe1, height_pipe1, width_pipe2, height_pipe2;
static uint32_t size_pipe1, size_pipe2;
static uint8_t *buffer_pipe1_capture, *buffer_pipe2_capture;

static uint8_t  *mean_pipe1, *mean_pipe2;
static uint16_t *accum_pipe1_mean, *accum_pipe2_mean;
static uint16_t *var_pipe1, *var_pipe2;
static uint32_t *accum_pipe1_var, *accum_pipe2_var;
static uint8_t  *std_pipe1, *std_pipe2;
static uint8_t  *detect_tmp_pipe1, *detect_tmp_pipe2;
static uint8_t  *detect_pipe1, *detect_pipe2;
static uint8_t  *frame1_pipe1, *frame1_pipe2;
static uint8_t  *frame2_pipe1, *frame2_pipe2;
static uint8_t  *detect_mvt_pipe1, *detect_mvt_pipe2;

/* Frames processed since the last calibration; gates the frame-diff
 * ("movement") sub-check in pixel_detection() until the running mean/std
 * have had a chance to settle (see pixel_detection below). */
static uint8_t nb_capture;

/* ==========================================================================
 * MVE-accelerated statistics helpers (ported verbatim from TB_LDS)
 * ========================================================================== */

/* Accumulates p_src (row_stride = SENSOR_WIDTH) into the running sum
 * (p_acc) and sum-of-squares (p_acc_var), one calibration frame at a time. */
static void stat_calibration(uint8_t *p_src, uint16_t *p_acc, uint32_t *p_acc_var, uint16_t height, uint16_t width)
{
    uint16_t row_stride = SENSOR_WIDTH;
    uint16_t last_rows = width - (width % 16);

    for (int row = 0; row < height; row++)
    {
        uint8_t  *row_src = &p_src[row * row_stride];
        uint16_t *row_acc = &p_acc[row * width];
        uint32_t *row_acc_var = &p_acc_var[row * width];

        for (int col = 0; col < last_rows; col += 16)
        {
            uint8x16_t img = vld1q_u8(&row_src[col]);
            uint16x8_t img_even = vmovlbq_u8(img);
            uint16x8_t img_odd = vmovltq_u8(img);

            uint16x8_t acc_even = vld1q_u16(&row_acc[col]);
            uint16x8_t acc_odd  = vld1q_u16(&row_acc[col + 8]);

            uint32x4_t acc_var_0 = vld1q_u32(&row_acc_var[col]);
            uint32x4_t acc_var_1 = vld1q_u32(&row_acc_var[col + 4]);
            uint32x4_t acc_var_2 = vld1q_u32(&row_acc_var[col + 8]);
            uint32x4_t acc_var_3 = vld1q_u32(&row_acc_var[col + 12]);

            acc_even = vaddq_u16(acc_even, img_even);
            acc_odd  = vaddq_u16(acc_odd, img_odd);

            acc_var_0 = vaddq_u32(acc_var_0, vmulq_u32(vmovlbq_u16(img_even), vmovlbq_u16(img_even)));
			acc_var_1 = vaddq_u32(acc_var_1, vmulq_u32(vmovltq_u16(img_even), vmovltq_u16(img_even)));
			acc_var_2 = vaddq_u32(acc_var_2, vmulq_u32(vmovlbq_u16(img_odd), vmovlbq_u16(img_odd)));
			acc_var_3 = vaddq_u32(acc_var_3, vmulq_u32(vmovltq_u16(img_odd), vmovltq_u16(img_odd)));

            vst1q_u16(&row_acc[col], acc_even);
            vst1q_u16(&row_acc[col + 8], acc_odd);

            vst1q_u32(&row_acc_var[col], acc_var_0);
            vst1q_u32(&row_acc_var[col + 4], acc_var_1);
            vst1q_u32(&row_acc_var[col + 8], acc_var_2);
            vst1q_u32(&row_acc_var[col + 12], acc_var_3);
        }

        for (int col = last_rows; col < width; col++)
        {
            row_acc[col] += row_src[col];
            row_acc_var[col] += (uint32_t)row_src[col] * row_src[col];
        }
    }
}

/* Folds the last calibration frame into the running sums and derives the
 * mean (p_dst_mean) and variance (p_dst_var) over the 32-frame window. */
static void last_stat_calibration(uint8_t *p_src, uint8_t *p_dst_mean, uint16_t *p_dst_var,
							uint16_t *p_acc, uint32_t *p_acc_var, uint16_t height, uint16_t width)
{
    uint16_t row_stride = SENSOR_WIDTH;
    uint16_t last_rows = width - (width % 16);

    for (int row = 0; row < height; row++)
    {
        uint8_t  *row_src = &p_src[row * row_stride];
        uint16_t *row_acc_mean = &p_acc[row * width];
        uint8_t  *row_dst_mean = &p_dst_mean[row * width];
        uint32_t *row_acc_var = &p_acc_var[row * width];
        uint16_t *row_dst_var  = &p_dst_var[row * width];

        for (int col = 0; col < last_rows; col += 16)
        {
            uint8x16_t img = vld1q_u8(&row_src[col]);
            uint16x8_t img_even = vmovlbq_u8(img);
            uint16x8_t img_odd = vmovltq_u8(img);

            uint16x8_t acc_even = vld1q_u16(&row_acc_mean[col]);
            uint16x8_t acc_odd  = vld1q_u16(&row_acc_mean[col + 8]);

            acc_even = vaddq_u16(acc_even, img_even);
            acc_odd  = vaddq_u16(acc_odd , img_odd);

            uint8x16_t result_mean = vdupq_n_u8(0);
			result_mean = vshrnbq_n_u16(result_mean, acc_even, 5);
			result_mean = vshrntq_n_u16(result_mean, acc_odd, 5);
			vst1q_u8(&row_dst_mean[col], result_mean);

            uint32x4_t acc_var_0 = vld1q_u32(&row_acc_var[col]);
			uint32x4_t acc_var_1 = vld1q_u32(&row_acc_var[col + 4]);
			uint32x4_t acc_var_2 = vld1q_u32(&row_acc_var[col + 8]);
			uint32x4_t acc_var_3 = vld1q_u32(&row_acc_var[col + 12]);

			acc_var_0 = vaddq_u32(acc_var_0, vmulq_u32(vmovlbq_u16(img_even), vmovlbq_u16(img_even)));
			acc_var_1 = vaddq_u32(acc_var_1, vmulq_u32(vmovltq_u16(img_even), vmovltq_u16(img_even)));
			acc_var_2 = vaddq_u32(acc_var_2, vmulq_u32(vmovlbq_u16(img_odd),  vmovlbq_u16(img_odd)));
			acc_var_3 = vaddq_u32(acc_var_3, vmulq_u32(vmovltq_u16(img_odd),  vmovltq_u16(img_odd)));

			uint32x4_t acc_even_lo = vmovlbq_u16(acc_even);
			uint32x4_t acc_even_hi = vmovltq_u16(acc_even);
			uint32x4_t acc_odd_lo  = vmovlbq_u16(acc_odd);
			uint32x4_t acc_odd_hi  = vmovltq_u16(acc_odd);

			uint32x4_t mean_sq_0 = vshrq_n_u32(vmulq_u32(acc_even_lo, acc_even_lo), 10);
			uint32x4_t mean_sq_1 = vshrq_n_u32(vmulq_u32(acc_even_hi, acc_even_hi), 10);
			uint32x4_t mean_sq_2 = vshrq_n_u32(vmulq_u32(acc_odd_lo,  acc_odd_lo),  10);
			uint32x4_t mean_sq_3 = vshrq_n_u32(vmulq_u32(acc_odd_hi,  acc_odd_hi),  10);

			uint32x4_t var_0 = vqsubq_u32(vshrq_n_u32(acc_var_0, 5), mean_sq_0);
			uint32x4_t var_1 = vqsubq_u32(vshrq_n_u32(acc_var_1, 5), mean_sq_1);
			uint32x4_t var_2 = vqsubq_u32(vshrq_n_u32(acc_var_2, 5), mean_sq_2);
			uint32x4_t var_3 = vqsubq_u32(vshrq_n_u32(acc_var_3, 5), mean_sq_3);

			uint16x8_t var_even = vdupq_n_u16(0);
			var_even = vmovnbq_u32(var_even, var_0);
			var_even = vmovntq_u32(var_even, var_1);

			uint16x8_t var_odd = vdupq_n_u16(0);
			var_odd = vmovnbq_u32(var_odd, var_2);
			var_odd = vmovntq_u32(var_odd, var_3);

			uint16x8x2_t result_var;
			result_var.val[0] = var_even;
			result_var.val[1] = var_odd;
			vst2q_u16(&row_dst_var[col], result_var);
        }

        for (int col = last_rows; col < width; col++)
        {
        	uint32_t sum_x  = (uint32_t)row_acc_mean[col] + row_src[col];
        	uint32_t sum_x2 = row_acc_var[col] + (uint32_t)row_src[col] * row_src[col];

        	uint32_t e_x2       = sum_x2 >> 5;
        	uint32_t mean_sq    = (sum_x * sum_x) >> 10;

        	row_dst_mean[col]  = (uint8_t)(sum_x >> 5);
        	row_dst_var[col]   = (e_x2 > mean_sq) ? (uint16_t)(e_x2 - mean_sq) : 0;
        }
    }
}

/* Derives the per-pixel detection threshold (std, floored at 7, saturated
 * at 255) from the variance computed by last_stat_calibration/stat_adjustment. */
static void var_to_std(uint16_t *p_var, uint8_t *p_std, uint32_t size_pipe)
{
    for (uint32_t i = 0; i < size_pipe; i++)
    {
        float k_std = sqrtf((float)p_var[i]) * 7.0f;
        if (k_std < 7.0f) k_std = 7.0f;
        p_std[i] = (k_std < 255.0f) ? (uint8_t)floorf(k_std) : MAX_GREY;
    }
}

/* Per-pixel detector: flags a pixel as "outlier" if it falls outside
 * [mean-std, mean+std] (p_tmp/p_dst, neighbour-count gated), and separately
 * flags "movement" via frame-to-frame differencing against the last two
 * frames (p_mvt) -- gated by nb_capture so it only arms once the running
 * mean/std have had a few post-calibration frames to settle. Either
 * sub-check can set *p_is_detect. */
static void pixel_detection(uint8_t *p_src, uint8_t *p_frame1, uint8_t *p_frame2, uint8_t *p_mvt, uint8_t *p_tmp ,uint8_t *p_dst, uint8_t *p_mean, uint8_t *p_std, uint16_t height, uint16_t width, uint8_t nb_voisin, bool *p_is_detect, uint8_t *nb_capture_p)
{
	uint16_t row_stride = SENSOR_WIDTH;

	for (int row = 0; row < height; row++)
	{
		uint8_t *row_src = &p_src[row * row_stride];
		uint8_t *row_frame1 = &p_frame1[row * width];
		uint8_t *row_frame2 = &p_frame2[row * width];

		uint8_t *row_tmp = &p_tmp[row * width];
		uint8_t *row_mean =&p_mean[row * width];
		uint8_t *row_std = &p_std[row * width];

		uint8_t *row_mvt = &p_mvt[row * width];

		uint8x16_t one  = vdupq_n_u8(MAX_GREY);
		uint8x16_t zero = vdupq_n_u8(0);
		uint8x16_t thresh_mvt = vdupq_n_u8(75);

		for (int col = 0; col < width; col += 16)
		{
			mve_pred16_t p = vctp8q(width - col);

			uint8x16_t pix_2 = vld1q_z_u8(&row_frame2[col], p);
			uint8x16_t pix_1 = vld1q_z_u8(&row_frame1[col], p);
			uint8x16_t pix_0 = vld1q_z_u8(&row_src[col], p);

			/* détection avec moyenne */
			uint8x16_t mean = vld1q_z_u8(&row_mean[col], p);
			uint8x16_t std = vld1q_z_u8(&row_std[col], p);

			uint8x16_t val_max = vqaddq_u8(mean, std);
			uint8x16_t val_min = vqsubq_u8(mean, std);

			mve_pred16_t cmp_up = vcmpcsq_m_u8(pix_0, val_max, p);
			mve_pred16_t cmp_down = vcmpcsq_m_u8(val_min, pix_0, p);

			mve_pred16_t detected = (mve_pred16_t)(cmp_up | cmp_down);
			uint8x16_t result = vpselq_u8(one, zero, detected);

			/* détection avec mouvement */
			uint8x16_t diff_1 = vabdq_x_u8(pix_2, pix_0, p);
			uint8x16_t diff_2 = vabdq_x_u8(pix_1, pix_0, p);

			mve_pred16_t cmp1_mvt = vcmpcsq_m_u8(diff_1, thresh_mvt, p);
			mve_pred16_t cmp2_mvt = vcmpcsq_m_u8(diff_2, thresh_mvt, p);

			mve_pred16_t detected_mvt = (mve_pred16_t)(cmp1_mvt | cmp2_mvt);
			if(detected_mvt && *nb_capture_p>=34) *p_is_detect = true;
			uint8x16_t result_mvt = vpselq_u8(one, zero, detected_mvt);


			vst1q_p_u8(&row_tmp[col], result, p);
			vst1q_p_u8(&row_mvt[col], result_mvt, p);
			vst1q_p_u8(&row_frame2[col], pix_1, p);
			vst1q_p_u8(&row_frame1[col], pix_0, p);
		}
	}

	memset(p_dst, 0, height * width);
	for(int row = 1; row < height-1; row++)
	{
		for(int col = 1; col < width-1; col++)
		{
			if(p_tmp[row*width + col] == 0) continue;

			uint16_t count = (uint16_t)p_tmp[(row-1)*width + col-1] + p_tmp[(row-1)*width + col] + p_tmp[(row-1)*width + col+1] +
							 	 	   p_tmp[row*width + col-1] + p_tmp[row*width + col] + p_tmp[row*width + col+1] +
									   p_tmp[(row+1)*width + col-1] + p_tmp[(row+1)*width + col] + p_tmp[(row+1)*width + col+1];

			if(count>nb_voisin*MAX_GREY)
			{
				p_dst[row*width + col] = MAX_GREY;
				*p_is_detect = true;
			}
		}
	}
}

/* Slowly drifts the running mean/std toward the current frame at
 * non-detected pixels (detected ones are left alone, so a lingering
 * object doesn't get absorbed into the background). */
static void stat_adjustment(uint8_t *p_src, uint8_t *p_detect, uint8_t *p_mean, uint16_t *p_var, uint8_t *p_std, uint16_t height, uint16_t width)
{
	float stat_adjust_ratio = 1.0f/(0.2f*60.0f);
	uint16_t row_stride = SENSOR_WIDTH;

	for (int row = 0; row < height; row++)
	{
		uint8_t *row_src = &p_src[row * row_stride];
		uint8_t *row_mean = &p_mean[row * width];
		uint16_t *row_var = &p_var[row*width];
		uint8_t *row_std = &p_std[row * width];
		uint8_t *row_detect = &p_detect[row * width];

		for (int col = 0; col < width; col++)
		{
			if(row_detect[col] == MAX_GREY) continue;

			float diff = row_src[col] - row_mean[col];

			row_mean[col] = (uint8_t)floorf((1-stat_adjust_ratio) * (float)row_mean[col] + stat_adjust_ratio * (float)row_src[col]);
			row_var[col] = (uint16_t)floorf((1-stat_adjust_ratio) * ((float)row_var[col] + stat_adjust_ratio * diff * diff));

			float k_std = sqrtf((float)row_var[col]) * 7.0f;
			if (k_std < 7.0f) k_std = 7.0f;
			row_std[col] = (k_std < 255.0f) ? (uint8_t)floorf(k_std) : MAX_GREY;
		}
	}
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

void DETECT_Init(void)
{
  dcmipp_get_detect_dims(&width_pipe1, &height_pipe1, &width_pipe2, &height_pipe2);
  dcmipp_get_capture_buffers(&buffer_pipe1_capture, &buffer_pipe2_capture);

  size_pipe1 = (uint32_t)width_pipe1 * height_pipe1;
  size_pipe2 = (uint32_t)width_pipe2 * height_pipe2;

  mean_pipe1       = (uint8_t  *)axisram_alloc(size_pipe1);
  mean_pipe2       = (uint8_t  *)axisram_alloc(size_pipe2);
  accum_pipe1_mean = (uint16_t *)axisram_alloc(size_pipe1 * 2);
  accum_pipe2_mean = (uint16_t *)axisram_alloc(size_pipe2 * 2);
  var_pipe1        = (uint16_t *)axisram_alloc(size_pipe1 * 2);
  var_pipe2        = (uint16_t *)axisram_alloc(size_pipe2 * 2);
  accum_pipe1_var  = (uint32_t *)axisram_alloc(size_pipe1 * 4);
  accum_pipe2_var  = (uint32_t *)axisram_alloc(size_pipe2 * 4);
  std_pipe1        = (uint8_t  *)axisram_alloc(size_pipe1);
  std_pipe2        = (uint8_t  *)axisram_alloc(size_pipe2);
  detect_tmp_pipe1 = (uint8_t  *)axisram_alloc(size_pipe1);
  detect_tmp_pipe2 = (uint8_t  *)axisram_alloc(size_pipe2);
  detect_pipe1     = (uint8_t  *)axisram_alloc(size_pipe1);
  detect_pipe2     = (uint8_t  *)axisram_alloc(size_pipe2);
  frame1_pipe1     = (uint8_t  *)axisram_alloc(size_pipe1);
  frame1_pipe2     = (uint8_t  *)axisram_alloc(size_pipe2);
  frame2_pipe1     = (uint8_t  *)axisram_alloc(size_pipe1);
  frame2_pipe2     = (uint8_t  *)axisram_alloc(size_pipe2);
  detect_mvt_pipe1 = (uint8_t  *)axisram_alloc(size_pipe1);
  detect_mvt_pipe2 = (uint8_t  *)axisram_alloc(size_pipe2);

  nb_capture = 0;
}

void DETECT_CalibrateStats(void)
{
  memset(accum_pipe1_mean, 0, size_pipe1 * 2);
  memset(accum_pipe2_mean, 0, size_pipe2 * 2);
  memset(accum_pipe1_var, 0, size_pipe1 * 4);
  memset(accum_pipe2_var, 0, size_pipe2 * 4);

  nb_capture = 0;
  while (nb_capture < 31) {
    if (capture_detect_frame() == 0) {
      stat_calibration(buffer_pipe1_capture, accum_pipe1_mean, accum_pipe1_var, height_pipe1, width_pipe1);
      stat_calibration(buffer_pipe2_capture, accum_pipe2_mean, accum_pipe2_var, height_pipe2, width_pipe2);
    }
    nb_capture++;
  }

  if (capture_detect_frame() == 0) {
    last_stat_calibration(buffer_pipe1_capture, mean_pipe1, var_pipe1, accum_pipe1_mean, accum_pipe1_var, height_pipe1, width_pipe1);
    last_stat_calibration(buffer_pipe2_capture, mean_pipe2, var_pipe2, accum_pipe2_mean, accum_pipe2_var, height_pipe2, width_pipe2);
  }

  var_to_std(var_pipe1, std_pipe1, size_pipe1);
  var_to_std(var_pipe2, std_pipe2, size_pipe2);
  nb_capture++;
}

bool DETECT_ProcessFrame(void)
{
  bool is_detect = false;

  if(capture_detect_frame() != 0)
    return false;

  pixel_detection(buffer_pipe1_capture, frame1_pipe1, frame2_pipe1, detect_mvt_pipe1, detect_tmp_pipe1,
                   detect_pipe1, mean_pipe1, std_pipe1, height_pipe1, width_pipe1, 3, &is_detect, &nb_capture);
  stat_adjustment(buffer_pipe1_capture, detect_pipe1, mean_pipe1, var_pipe1, std_pipe1, height_pipe1, width_pipe1);

  pixel_detection(buffer_pipe2_capture, frame1_pipe2, frame2_pipe2, detect_mvt_pipe2, detect_tmp_pipe2,
                   detect_pipe2, mean_pipe2, std_pipe2, height_pipe2, width_pipe2, 2, &is_detect, &nb_capture);
  stat_adjustment(buffer_pipe2_capture, detect_pipe2, mean_pipe2, var_pipe2, std_pipe2, height_pipe2, width_pipe2);

  if(nb_capture < 34) nb_capture++;

  return is_detect;
}
