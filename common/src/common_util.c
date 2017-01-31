#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <bot_core/bot_core.h>

#include "common_util.h"

uint64_t 
hr_common_get_unique_id (void)
{
    return (0xefffffffffffffff&(bot_timestamp_now()<<8)) + 256*rand()/RAND_MAX;
}

uint32_t 
hr_get_unique_id_32t (void)
{
    return (0xefffffff&(bot_timestamp_now()<<8)) + 256*rand()/RAND_MAX;
}
