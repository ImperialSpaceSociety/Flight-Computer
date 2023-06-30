
#include "cmsis_os.h"

osStatus dlog(const char *format, ...);

// Comment out to disable logging code generation
#define COMPILE_DLOG

// Max message length in bytes
#define DLOG_MAX_MSG_LEN  64

// Logging levels in increasing verbosity (and decreasing importance)
#define DLOG_CRIT   DLOG_SOH  "5"
#define DLOG_ERR    DLOG_SOH  "4"
#define DLOG_WARN   DLOG_SOH  "3"
#define DLOG_INFO   DLOG_SOH  "2"
#define DLOG_DEBUG  DLOG_SOH  "1"

// Default logging level
#define DLOG_DEFAULT_LEVEL    DLOG_DEBUG

#define CONSOLE_TIMEOUT_MS 0

#define DLOG_SOH "\001"
extern osMessageQId printQueueHandle;

