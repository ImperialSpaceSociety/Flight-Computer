
#include "cmsis_os.h"

/* Must prepend message string with one of the DLOG_XXXX
 * levels, e.g. `dlog(DLOG_CRIT "Critical error: ...")`
 * or else your message will not be printed.
 */
osStatus dlog(const char *format, ...);

// Comment out to disable logging code generation
#define COMPILE_DLOG

// Comment out to use generic `printf` for debugging purposes
//#define USE_USB_CDC

// Max message length in bytes
#define DLOG_MAX_MSG_LEN  64

// Logging levels in increasing verbosity (and decreasing importance)
#define DLOG_CRIT   DLOG_SOH  "5"
#define DLOG_ERR    DLOG_SOH  "4"
#define DLOG_WARN   DLOG_SOH  "3"
#define DLOG_INFO   DLOG_SOH  "2"
#define DLOG_DEBUG  DLOG_SOH  "1"

/* Default logging level
 * Messages below this level will not be printed.
 */
#define DLOG_DEFAULT_LEVEL    DLOG_DEBUG

/* Max amount of time that `dlog` will block for
 * if the printQueue is full. If your task is printing
 * critical messages, make sure you check the return
 * value of `dlog` to see if your request timed-out.
 *
 * The default value is 0, which assumes that the
 * debug logging frequency is low, and the queue will
 * never be full.
 */
#define CONSOLE_TIMEOUT_MS 0


// ----- Private -----
#define DLOG_SOH "\001"
extern osMessageQId printQueueHandle;

