
#include "main.h"
#include "Console.h"

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>


void ConsoleTask(void const * argument) {

  MX_USB_DEVICE_Init();

  while (1) {

    osEvent e = osMessageGet(printQueueHandle, osWaitForever);

    if (e.status == osEventMessage) {
      char *p = e.value.p;

#ifdef USE_USB_CDC
      CDC_Transmit_FS((uint8_t *)p, strnlen(p, DLOG_MAX_MSG_LEN));
#else
      printf("%.*s", strnlen(p, DLOG_MAX_MSG_LEN), p);
#endif

      free(p);
    }

  }

}

/*
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart?, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
*/

