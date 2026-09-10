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
static uint8_t  *voisin_count_pipe1, *voisin_count_pipe2; /* 0-9 per pixel, for extract_blocs() */
static uint32_t *stack_pipe1, *stack_pipe2; /* flood-fill scratch, see extract_blocs() */

/* Frames since last calibration; gates the movement sub-check. */
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

/* Derives the per-pixel detection threshold (std, floored at STD_FACTOR,
 * saturated at 255) from the variance computed by
 * last_stat_calibration/stat_adjustment. */
static void var_to_std(uint16_t *p_var, uint8_t *p_std, uint32_t size_pipe)
{
    for (uint32_t i = 0; i < size_pipe; i++)
    {
        float k_std = sqrtf((float)p_var[i]) * (float)STD_FACTOR;
        if (k_std < (float)STD_FACTOR) k_std = (float)STD_FACTOR;
        p_std[i] = (k_std < 255.0f) ? (uint8_t)floorf(k_std) : MAX_GREY;
    }
}

/* Flags outliers (p_dst, neighbour-count gated) and movement (frame diff,
 * nb_capture gated). p_is_detect is shared/global across both pipes;
 * p_is_detect_mouvement/p_is_detect_deviation are the same two sub-checks
 * scoped to this call. p_voisin_count gets each pixel's 3x3 outlier count. */
static void pixel_detection(uint8_t *p_src, uint8_t *p_frame1, uint8_t *p_frame2, uint8_t *p_mvt, uint8_t *p_tmp, uint8_t *p_dst,
                             uint8_t *p_mean, uint8_t *p_std, uint16_t height, uint16_t width, uint8_t nb_voisin,
                             uint8_t *p_voisin_count, bool *p_is_detect, bool *p_is_detect_mouvement,
                             bool *p_is_detect_deviation, uint8_t *nb_capture_p)
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
		uint8x16_t thresh_mvt = vdupq_n_u8(DETECT_THRESH_MVT);

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
			if(detected_mvt && *nb_capture_p>=34) { *p_is_detect = true; *p_is_detect_mouvement = true; }
			uint8x16_t result_mvt = vpselq_u8(one, zero, detected_mvt);


			vst1q_p_u8(&row_tmp[col], result, p);
			vst1q_p_u8(&row_mvt[col], result_mvt, p);
			vst1q_p_u8(&row_frame2[col], pix_1, p);
			vst1q_p_u8(&row_frame1[col], pix_0, p);
		}
	}

	memset(p_dst, 0, height * width);
	memset(p_voisin_count, 0, height * width);
	for(int row = 1; row < height-1; row++)
	{
		for(int col = 1; col < width-1; col++)
		{
			if(p_tmp[row*width + col] == 0) continue;

			uint16_t count = (uint16_t)p_tmp[(row-1)*width + col-1] + p_tmp[(row-1)*width + col] + p_tmp[(row-1)*width + col+1] +
							 	 	   p_tmp[row*width + col-1] + p_tmp[row*width + col] + p_tmp[row*width + col+1] +
									   p_tmp[(row+1)*width + col-1] + p_tmp[(row+1)*width + col] + p_tmp[(row+1)*width + col+1];

			p_voisin_count[row*width + col] = (uint8_t)(count / MAX_GREY);

			if(count>nb_voisin*MAX_GREY)
			{
				p_dst[row*width + col] = MAX_GREY;
				*p_is_detect = true;
				*p_is_detect_deviation = true;
			}
		}
	}
}

/* Max |src-frame| over the pipe. Must run before pixel_detection() ages
 * frame1/frame2 in place. Plain scalar, separate from the MVE hot path. */
static void compute_max_deltas(const uint8_t *p_src, const uint8_t *p_frame1, const uint8_t *p_frame2,
                                uint16_t height, uint16_t width,
                                uint8_t *p_delta_max_1, uint8_t *p_delta_max_2)
{
	uint16_t row_stride = SENSOR_WIDTH;
	uint8_t max1 = 0, max2 = 0;

	for (int row = 0; row < height; row++)
	{
		const uint8_t *row_src = &p_src[row * row_stride];
		const uint8_t *row_f1 = &p_frame1[row * width];
		const uint8_t *row_f2 = &p_frame2[row * width];

		for (int col = 0; col < width; col++)
		{
			uint8_t d1 = (row_src[col] > row_f1[col]) ? (uint8_t)(row_src[col] - row_f1[col]) : (uint8_t)(row_f1[col] - row_src[col]);
			uint8_t d2 = (row_src[col] > row_f2[col]) ? (uint8_t)(row_src[col] - row_f2[col]) : (uint8_t)(row_f2[col] - row_src[col]);
			if (d1 > max1) max1 = d1;
			if (d2 > max2) max2 = d2;
		}
	}

	*p_delta_max_1 = max1;
	*p_delta_max_2 = max2;
}

/* 8-connected component labelling via iterative flood-fill. p_visited is
 * scratch (caller passes detect_tmp_pipeN -- its content from this frame's
 * pixel_detection() is dead by now). p_stack is a worst-case size_pipeN
 * index stack, allocated once in DETECT_Init(). Components beyond
 * DETECT_MAX_BLOCS still get flood-filled (and count toward bbox_global)
 * but aren't added to blocs[]. */
static void extract_blocs(const uint8_t *p_dst, const uint8_t *p_src, const uint8_t *p_mean, const uint8_t *p_std,
                           const uint8_t *p_voisin_count, uint8_t *p_visited, uint32_t *p_stack,
                           uint16_t height, uint16_t width, DETECT_DeviationVoisinage_t *p_out)
{
	uint16_t row_stride = SENSOR_WIDTH; /* p_src is the raw capture buffer (SENSOR_WIDTH stride) */
	uint32_t size = (uint32_t)height * width;

	memset(p_visited, 0, size);
	p_out->bbox_global_valid = false;
	p_out->nb_blocs = 0;

	for (uint32_t start = 0; start < size; start++)
	{
		if (p_dst[start] != MAX_GREY || p_visited[start])
			continue;

		uint32_t x_min = start % width, x_max = x_min;
		uint32_t y_min = start / width, y_max = y_min;
		uint32_t sum_valeur = 0, sum_mean = 0, sum_std = 0, sum_voisin = 0, n = 0;
		uint32_t sp = 0;

		p_stack[sp++] = start;
		p_visited[start] = 1;

		while (sp > 0)
		{
			uint32_t idx = p_stack[--sp];
			uint32_t row = idx / width, col = idx % width;

			if (col < x_min) x_min = col;
			if (col > x_max) x_max = col;
			if (row < y_min) y_min = row;
			if (row > y_max) y_max = row;

			sum_valeur += p_src[row * row_stride + col];
			sum_mean   += p_mean[idx];
			sum_std    += p_std[idx];
			sum_voisin += p_voisin_count[idx];
			n++;

			for (int dr = -1; dr <= 1; dr++)
			{
				for (int dc = -1; dc <= 1; dc++)
				{
					if (dr == 0 && dc == 0) continue;

					int nr = (int)row + dr, nc = (int)col + dc;
					if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;

					uint32_t nidx = (uint32_t)nr * width + nc;
					if (p_dst[nidx] == MAX_GREY && !p_visited[nidx])
					{
						p_visited[nidx] = 1;
						p_stack[sp++] = nidx;
					}
				}
			}
		}

		if (!p_out->bbox_global_valid)
		{
			p_out->bbox_global.x_min = (uint16_t)x_min;
			p_out->bbox_global.y_min = (uint16_t)y_min;
			p_out->bbox_global.x_max = (uint16_t)x_max;
			p_out->bbox_global.y_max = (uint16_t)y_max;
			p_out->bbox_global_valid = true;
		}
		else
		{
			if (x_min < p_out->bbox_global.x_min) p_out->bbox_global.x_min = (uint16_t)x_min;
			if (y_min < p_out->bbox_global.y_min) p_out->bbox_global.y_min = (uint16_t)y_min;
			if (x_max > p_out->bbox_global.x_max) p_out->bbox_global.x_max = (uint16_t)x_max;
			if (y_max > p_out->bbox_global.y_max) p_out->bbox_global.y_max = (uint16_t)y_max;
		}

		if (p_out->nb_blocs < DETECT_MAX_BLOCS)
		{
			DETECT_Bloc_t *b = &p_out->blocs[p_out->nb_blocs++];
			b->bbox.x_min = (uint16_t)x_min;
			b->bbox.y_min = (uint16_t)y_min;
			b->bbox.x_max = (uint16_t)x_max;
			b->bbox.y_max = (uint16_t)y_max;
			b->valeur_moyenne  = (float)sum_valeur / (float)n;
			b->mean_moyen      = (float)sum_mean   / (float)n;
			b->std_moyen       = (float)sum_std    / (float)n;
			b->nb_voisin_moyen = (float)sum_voisin / (float)n;
		}
	}
}

/* Drifts running mean/std toward the current frame at non-detected pixels. */
static void stat_adjustment(uint8_t *p_src, uint8_t *p_detect, uint8_t *p_mean, uint16_t *p_var, uint8_t *p_std, uint16_t height, uint16_t width)
{
	float stat_adjust_ratio = DETECT_STAT_ADJUST_RATIO;
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

			float k_std = sqrtf((float)row_var[col]) * (float)STD_FACTOR;
			if (k_std < (float)STD_FACTOR) k_std = (float)STD_FACTOR;
			row_std[col] = (k_std < 255.0f) ? (uint8_t)floorf(k_std) : MAX_GREY;
		}
	}
}

/* Fraction (0-100) of p_detect's pixels flagged MAX_GREY. */
static float detect_percentage(const uint8_t *p_detect, uint32_t size)
{
  uint32_t count = 0;

  for (uint32_t i = 0; i < size; i++)
    if (p_detect[i] == MAX_GREY)
      count++;

  return (100.0f * (float)count) / (float)size;
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
  voisin_count_pipe1 = (uint8_t *)axisram_alloc(size_pipe1);
  voisin_count_pipe2 = (uint8_t *)axisram_alloc(size_pipe2);
  stack_pipe1 = (uint32_t *)axisram_alloc(size_pipe1 * 4);
  stack_pipe2 = (uint32_t *)axisram_alloc(size_pipe2 * 4);

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

bool DETECT_ProcessFrame(DETECT_Result_t *p_result)
{
  bool is_detect = false;
  bool is_detect_mvt_1 = false, is_detect_dev_1 = false;
  bool is_detect_mvt_2 = false, is_detect_dev_2 = false;

  if(capture_detect_frame() != 0)
    return false;

  compute_max_deltas(buffer_pipe1_capture, frame1_pipe1, frame2_pipe1, height_pipe1, width_pipe1,
                      &p_result->second_plan.mouvement.delta_max_frame_moins_1,
                      &p_result->second_plan.mouvement.delta_max_frame_moins_2);
  compute_max_deltas(buffer_pipe2_capture, frame1_pipe2, frame2_pipe2, height_pipe2, width_pipe2,
                      &p_result->premier_plan.mouvement.delta_max_frame_moins_1,
                      &p_result->premier_plan.mouvement.delta_max_frame_moins_2);

  pixel_detection(buffer_pipe1_capture, frame1_pipe1, frame2_pipe1, detect_mvt_pipe1, detect_tmp_pipe1,
                   detect_pipe1, mean_pipe1, std_pipe1, height_pipe1, width_pipe1, DETECT_NB_VOISIN_PIPE1,
                   voisin_count_pipe1, &is_detect, &is_detect_mvt_1, &is_detect_dev_1, &nb_capture);
  stat_adjustment(buffer_pipe1_capture, detect_pipe1, mean_pipe1, var_pipe1, std_pipe1, height_pipe1, width_pipe1);

  p_result->second_plan.mouvement.detecte = is_detect_mvt_1;
  p_result->second_plan.deviation_voisinage.detecte = is_detect_dev_1;
  p_result->second_plan.deviation_voisinage.pct_pipe = detect_percentage(detect_pipe1, size_pipe1);

  extract_blocs(detect_pipe1, buffer_pipe1_capture, mean_pipe1, std_pipe1, voisin_count_pipe1,
                detect_tmp_pipe1, stack_pipe1, height_pipe1, width_pipe1, &p_result->second_plan.deviation_voisinage);

  pixel_detection(buffer_pipe2_capture, frame1_pipe2, frame2_pipe2, detect_mvt_pipe2, detect_tmp_pipe2,
                   detect_pipe2, mean_pipe2, std_pipe2, height_pipe2, width_pipe2, DETECT_NB_VOISIN_PIPE2,
                   voisin_count_pipe2, &is_detect, &is_detect_mvt_2, &is_detect_dev_2, &nb_capture);
  stat_adjustment(buffer_pipe2_capture, detect_pipe2, mean_pipe2, var_pipe2, std_pipe2, height_pipe2, width_pipe2);

  p_result->premier_plan.mouvement.detecte = is_detect_mvt_2;
  p_result->premier_plan.deviation_voisinage.detecte = is_detect_dev_2;
  p_result->premier_plan.deviation_voisinage.pct_pipe = detect_percentage(detect_pipe2, size_pipe2);

  extract_blocs(detect_pipe2, buffer_pipe2_capture, mean_pipe2, std_pipe2, voisin_count_pipe2,
                detect_tmp_pipe2, stack_pipe2, height_pipe2, width_pipe2, &p_result->premier_plan.deviation_voisinage);

  if(nb_capture < 34) nb_capture++;

  return is_detect;
}

