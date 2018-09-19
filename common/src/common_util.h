#ifndef __HR_COMMON_UTIL_H__
#define __HR_COMMON_UTIL_H__

#include <bot_core/bot_core.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * hr_common_get_unique_id
 *
 * Returns a random uint64_t
 */
    uint64_t hr_common_get_unique_id ();

    uint32_t hr_get_unique_id_32t ();

#ifdef __cplusplus
}
#endif

#endif
