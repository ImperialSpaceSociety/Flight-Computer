
#include "main.h"
#include "Console.h"

#include "cmsis_os.h"


void BuzzerTask(void const * argument) {

  while (1) {
    dlog(DLOG_INFO "Hello, BuzzerTask!");
    osDelay(1500);
  }

}
