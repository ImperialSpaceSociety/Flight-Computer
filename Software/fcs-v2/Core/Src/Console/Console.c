
#include "Console.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

static char *DLOG_LEVEL = DLOG_DEFAULT_LEVEL;

osStatus dlog(const char *format, ...) {

#ifdef COMPILE_DLOG

  osStatus ret = osOK;

  // Check if minimum message length is satisfied
  if (strnlen(format, DLOG_MAX_MSG_LEN) < 2)
    return osErrorParameter;

  // Check if message header is correct
  if (format[0] != DLOG_SOH[0])
    return osErrorParameter;

  // Check if logging level is in range
  if (format[1] < DLOG_DEBUG[1] || format[1] > DLOG_CRIT[1])
    return osErrorParameter;

  char *p = (char *)malloc(DLOG_MAX_MSG_LEN + 3);

  va_list args;
  va_start(args, format);

  // Print message only if logging level is sufficient
  if (format[1] >= DLOG_LEVEL[1]) {
    vsnprintf(p, DLOG_MAX_MSG_LEN, format + 2, args);
    int len = strnlen(p, DLOG_MAX_MSG_LEN);
    p[len] = '\r';
    p[len + 1] = '\n';
    p[len + 2] = '\0';
    ret = 0;
    //ret = osMessagePut(printQueueHandle, (uint32_t)p, CONSOLE_TIMEOUT_MS);
  }

  va_end(args);

  if (ret != osOK) {
    free(p);
  }

  return ret;

#endif
}
