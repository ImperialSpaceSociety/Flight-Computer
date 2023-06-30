
#include "main.h"
#include "Console.h"

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>


void ConsoleTask(void const * argument) {

  MX_USB_DEVICE_Init();

  while (1) {

    osEvent e = osMessageGet(printQueueHandle, osWaitForever);

    if (e.status == osEventMessage) {
      char *p = e.value.p;
      CDC_Transmit_FS((uint8_t *)p, strnlen(p, DLOG_MAX_MSG_LEN));
      free(p);
    }

  }

}
