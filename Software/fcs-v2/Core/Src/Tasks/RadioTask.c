
#include "main.h"
#include "Console.h"

#include "cmsis_os.h"


void RadioTask(void const * argument) {

  while (1) {
    dlog(DLOG_INFO "Hello, FRADIO!");
    osDelay(1000);
  }
}
