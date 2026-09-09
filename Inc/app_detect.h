/**
 ******************************************************************************
 * @file    app_detect.h
 * @brief   Statistical + frame-diff motion detector on the two detect-mode
 *          DCMIPP pipes (MVE-accelerated), ported from TB_LDS/Src/app.c
 *          (DETECTION state).
 ******************************************************************************
 */
#ifndef APP_DETECT_H
#define APP_DETECT_H

#include <stdbool.h>
#include <stdint.h>

/* Max blobs tracked per pipe per frame; extras beyond this still count
 * toward bbox_global but aren't added to blocs[]. */
#define DETECT_MAX_BLOCS 8U

/* Detection thresholds -- single source of truth, used by app_detect.c and
 * readable from app.c for the config side of the JSON log. */
#define DETECT_THRESH_MVT        75U
#define DETECT_NB_VOISIN_PIPE1   3U
#define DETECT_NB_VOISIN_PIPE2   2U
#define DETECT_DIM_CARRE         3U
#define DETECT_STAT_ADJUST_RATIO (1.0f/(0.2f*60.0f))

typedef struct {
  uint16_t x_min, y_min, x_max, y_max;
} DETECT_BBox_t;

typedef struct {
  bool detecte;
  uint8_t delta_max_frame_moins_1;
  uint8_t delta_max_frame_moins_2;
} DETECT_Mouvement_t;

/* One connected component of flagged pixels (see extract_blocs()). */
typedef struct {
  DETECT_BBox_t bbox;
  float valeur_moyenne;
  float mean_moyen;
  float std_moyen;
  float nb_voisin_moyen;
} DETECT_Bloc_t;

typedef struct {
  bool detecte;
  float pct_pipe;
  bool bbox_global_valid;
  DETECT_BBox_t bbox_global;
  uint8_t nb_blocs;
  DETECT_Bloc_t blocs[DETECT_MAX_BLOCS];
} DETECT_DeviationVoisinage_t;

typedef struct {
  DETECT_Mouvement_t mouvement;
  DETECT_DeviationVoisinage_t deviation_voisinage;
} DETECT_PipeResult_t;

/* second_plan = pipe1, premier_plan = pipe2. Coordinates are local to the
 * pipe (post crop+downsize), not sensor coordinates. */
typedef struct {
  DETECT_PipeResult_t second_plan;
  DETECT_PipeResult_t premier_plan;
} DETECT_Result_t;

void DETECT_Init(void);
void DETECT_CalibrateStats(void);

/* One detect cycle. Returns true if movement was detected on either pipe;
 * p_result is always fully filled in either way. */
bool DETECT_ProcessFrame(DETECT_Result_t *p_result);

#endif /* APP_DETECT_H */
