#ifndef __clock_skew_estimator_h__
#define __clock_skew_estimator_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _ClockSkewEstimator ClockSkewEstimator;

/** 
 * clock_skew_estimator_new:
 * @ref_clock_name: the name of the reference clock.  All clock skew will be
 * estimated relative to the reference clock.
 *
 * Creates a new clock skew estimator.
 */
ClockSkewEstimator * clock_skew_estimator_new(const char *ref_clock_name);

void clock_skew_estimator_destroy(ClockSkewEstimator *self);

void clock_skew_estimator_add_measurement(ClockSkewEstimator *self,
        const char *clock_name, int64_t recv_time, int64_t clock_time);

/**
 * Returns: TRUE if a skew estimate is available, FALSE if not.
 */
gboolean clock_skew_estimator_get_skew(ClockSkewEstimator *self,
        const char *target_clock, int64_t *skew_time);

/**
 * Returns: a newly allocated array of clock names.  free with g_strfreev.
 */
char ** clock_skew_estimator_get_clock_names(ClockSkewEstimator *self);

#ifdef __cplusplus
}
#endif

#endif
