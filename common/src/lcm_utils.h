/** @addtogroup LCM_Utils **/
// @{

/*
 * lcm_ipc_utils.h
 *
 *  Created on: Mar 12, 2009
 *      Author: abachrac
 */

#ifndef LCM_UTILS_H_
#define LCM_UTILS_H_

#include <glib.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <hr_lcmtypes/lcm_channel_names.h>


#ifdef __cplusplus
extern "C" {
#endif

static inline void lcm_sleep(lcm_t * lcm, double sleeptime)
{ //max time to wait (in seconds, may return sooner)
    int lcm_fileno = lcm_get_fileno(lcm);

    fd_set rfds;
    int retval;
    FD_ZERO(&rfds);
    FD_SET(lcm_fileno, &rfds);
    struct timeval tv;
    tv.tv_sec = (int) sleeptime;
    tv.tv_usec = (int) ((sleeptime - tv.tv_sec) * 1.0e6);
    retval = select(lcm_fileno + 1, &rfds, NULL, NULL, &tv);
    if (retval == -1){
      printf("select() failed!\n");
      return;
    }
    if (retval) {
      if (FD_ISSET(lcm_fileno, &rfds)) {
        lcm_handle(lcm);
      }
    }
  }

  static inline void lcm_dispatch(lcm_t * lcm)
  { //wait indefinitely for incoming events
    while (1)
      lcm_handle(lcm);
  }

    /*static inline void lcm_ipc_dispatch(lcm_t * lcm)
  { //wait indefinitely for incoming events
    while (1)
      lcm_ipc_sleep(lcm, 10);
      }*/

  /*
   * Convert bot_timestamp to a double for printing
   */
  static inline double bot_timestamp_to_double(int64_t utime)
  {

    return ((double) utime) / 1.0e6;
  }

  /*
   * Get bot_timestamp from a double value
   */
  static inline int64_t bot_timestamp_from_double(double timestamp)
  {
    return (int64_t) (timestamp * 1.0e6);
  }

  /**
   * Computes the time elapsed in seconds since the given bot_timestamp.
   * @param start_utime a bot_timestamp specifying the start of the period to measure.
   * @return the elapsed time in seconds as a double
   */
  static inline double bot_time_elapsed_secs(int64_t start_utime) {
    int64_t elapsed_utime = bot_timestamp_now() - start_utime;
    return bot_timestamp_to_double(elapsed_utime);
  }

  
#ifdef __cplusplus
}
#endif

#endif /* LCM_UTILS_H_ */

// @}
