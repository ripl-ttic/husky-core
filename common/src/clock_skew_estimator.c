#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <glib.h>

#include <bot_core/bot_core.h>

#include "clock_skew_estimator.h"

typedef struct _clock_t skewed_clock_t;
struct _clock_t
{
    char *name;

    int64_t last_recv_time;
    int64_t last_clock_time;
    int64_t have_skew_estimate;
    int64_t have_measurement;
    double skew_estimate;
};

static skewed_clock_t * 
clock_new(const char *name)
{
    skewed_clock_t *clock = g_slice_new(skewed_clock_t);
    clock->name = strdup(name);
    clock->last_recv_time = INT64_MAX;
    clock->last_clock_time = INT64_MAX;
    clock->skew_estimate = 0;
    clock->have_skew_estimate = 0;
    clock->have_measurement = 0;
    return clock;
}

static void
clock_free(skewed_clock_t *clock)
{
    free(clock->name);
    g_slice_free(skewed_clock_t, clock);
}

// =================================

struct _ClockSkewEstimator
{
    skewed_clock_t *ref_clock;
    GHashTable * clocks;
};

static skewed_clock_t * get_clock(ClockSkewEstimator *self, const char *clock_name,
        gboolean create_if_needed);

ClockSkewEstimator * 
clock_skew_estimator_new(const char *ref_clock_name)
{
    ClockSkewEstimator *self = g_slice_new(ClockSkewEstimator);

    self->clocks = g_hash_table_new_full(g_str_hash, g_str_equal, NULL,
            (GDestroyNotify) clock_free);
    self->ref_clock = get_clock(self, ref_clock_name, TRUE);

    return self;
}

void 
clock_skew_estimator_destroy(ClockSkewEstimator *self)
{
    g_hash_table_destroy(self->clocks);
    g_slice_free(ClockSkewEstimator, self);
}

static skewed_clock_t * 
get_clock(ClockSkewEstimator *self, const char *clock_name, 
        gboolean create_if_needed)
{
    skewed_clock_t *clock = g_hash_table_lookup(self->clocks, clock_name);
    if(!clock && create_if_needed) {
        clock = clock_new(clock_name);
        g_hash_table_insert(self->clocks, clock->name, clock);
    }
    return clock;
}

void 
clock_skew_estimator_add_measurement(ClockSkewEstimator *self,
        const char *clock_name, int64_t recv_time, int64_t clock_time)
{
    skewed_clock_t *clock = get_clock(self, clock_name, TRUE);

    if(recv_time < clock->last_recv_time || 
       clock_time < clock->last_clock_time) {
        // clock went back in time.  reset estimates.

        clock->skew_estimate = 0;
        clock->have_skew_estimate = 0;
    } else if(clock != self->ref_clock && self->ref_clock->have_measurement) {
        int64_t recv_dt = recv_time - self->ref_clock->last_recv_time;

        int64_t clock_dt = clock_time - self->ref_clock->last_clock_time;

        int64_t instantaneous_skew = clock_dt - recv_dt;

        double new_skew_weight = 0.2;
        clock->skew_estimate = clock->skew_estimate * (1-new_skew_weight) + 
            instantaneous_skew * new_skew_weight;
        clock->have_skew_estimate = 1;
    }
    clock->last_clock_time = clock_time;
    clock->last_recv_time = recv_time;
    if(!clock->have_measurement)
        clock->have_measurement = 1;
}

gboolean 
clock_skew_estimator_get_skew(ClockSkewEstimator *self,
        const char *target_clock, int64_t *skew_usec)
{
    skewed_clock_t *clock = get_clock(self, target_clock, FALSE);
    if(!clock || !clock->have_skew_estimate)
        return FALSE;

    if(skew_usec)
        *skew_usec = clock->skew_estimate;
    return TRUE;
}

static int 
_clock_name_cmp(const void *A, const void *B)
{
    skewed_clock_t *clock_a = (skewed_clock_t*) A;
    skewed_clock_t *clock_b = (skewed_clock_t*) B;
    return strcmp(clock_a->name, clock_b->name);
}

char ** 
clock_skew_estimator_get_clock_names(ClockSkewEstimator *self)
{
    GList * clocks = bot_g_hash_table_get_vals(self->clocks);
    clocks = g_list_sort(clocks, _clock_name_cmp);

    int nclocks = g_list_length(clocks);
    char **result = (char**) calloc(1, (nclocks + 1) * sizeof(char*));

    int i = 0;
    for(GList *citer=clocks; citer; citer=citer->next) {
        skewed_clock_t *clock = (skewed_clock_t*) citer->data;
        result[i] = strdup(clock->name);
        i++;
    }
    g_list_free(clocks);
    return result;
}
