#include <string.h>
#include <stdint.h>
#include "inc/hw_uart.h"
#include "inc/hw_memmap.h"
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"

#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"

#include "DbgUart.h" 
#include "Queue/Queue.h"

#include "FreeRTOS.h"
#include "task.h"

#define UART_RX_BUFF 128
namespace TskPrint{
	const int tskPrio = 2;
	const int tskStkSize = 512;

	QueueHandle_t MbCmd;
	char msg[128];

	Queue<char, false> QUartRx(128);

	void uartRxISR(void)
	{
        //
        // Clear RX receive interrupt mask
        //
        HWREG(UART2_BASE | UART_O_ICR) |= 0x00000010;
        while(UARTCharsAvail(UART2_BASE)){
            QUartRx.En(HWREG(UART2_BASE | UART_O_DR));
        }
	}
	void UART2Init(void)
	{
	    //
	    // PA6-7 are used for UART2.
	    //
	    HWREG(SYSCTL_RCGCUART) |= 0x00000004;
	    HWREG(GPIO_PORTA_BASE | GPIO_O_AFSEL) |= 0x000000C0;    // port A[7..6] as alternate func
	    HWREG(GPIO_PORTA_BASE | GPIO_O_DEN)   |= 0x000000C0;    // port A[7..6] digital enable
	    // port A[7..6] as AF1(uart)
	    HWREG(GPIO_PORTA_BASE | GPIO_O_PCTL) = HWREG(GPIO_PORTA_BASE | GPIO_O_PCTL) & 0x00FFFFFF | 0x11000000;

	    HWREG(UART2_BASE | UART_O_CTL) = 0x00000000;            // disable UART2
	    /*
	     *BRD = 120 000 000 / (16 * 115200) = 65.1041667
	     *IBRD = 65
	     *FBRD = integer(0.1041667 * 64 + 0.5) = 7
	     */
	    HWREG(UART2_BASE | UART_O_IBRD) = 65;               // IBRD 65
	    HWREG(UART2_BASE | UART_O_FBRD) = 7;                // IBRD 7
	    // LCRH 0x60:8bits,one stop bit,no parity,fifos disable
	    HWREG(UART2_BASE | UART_O_LCRH) = 0x00000060;
	    HWREG(UART2_BASE | UART_O_CC) = 0x00000000;         //System clock
        //
        // Enable RX receive interrupt mask
        //
        HWREG(UART2_BASE | UART_O_IM) |= 0x00000010;
        IntRegister(INT_UART2_TM4C129, uartRxISR);
        IntEnable(INT_UART2_TM4C129);
	    //
	    // Enable RX, TX, and the UART.
	    //
	    HWREG(UART2_BASE | UART_O_CTL) = 0x00000301;
        HWREG(UART2_BASE | UART_O_ICR) |= 0x00000010;
	}

	void UartGetLine(char *line)
	{
	    char c;
	    bool b;
	    if(line == NULL)
	        return;

	    while(1)
	    {
	        do{
	            //
	            //Disable RX receive interrupt mask
	            //
//	            HWREG(UART2_BASE | UART_O_IM) &= ~0x00000010;
	            b = QUartRx.De(c);
	            //
	            // Enable RX receive interrupt mask
	            //
//	            HWREG(UART2_BASE | UART_O_IM) |= 0x00000010;
	        }while(!b);

	        if((*line++ = c) == '\r')
	            break;
	    }
	    *(line-1) = '\0';
	    return;
	}

	int UartWrite(char *str)
	{
		unsigned int uIdx,len;

		len = strlen(str);

		for(uIdx = 0; uIdx < len; uIdx++)
		{
			while (HWREG(UART2_BASE | UART_O_FR) & UART_FR_TXFF)
		    {
		    }
			HWREG(UART2_BASE | UART_O_DR) = str[uIdx];
		}
		return len;
	}

	void task(void *pvParameters)
	{
	    BaseType_t rtn;

	    while(true){
	        rtn=xQueuePend(MbCmd, msg, portMAX_DELAY);
    		configASSERT(rtn==pdPASS);
	        UartWrite(msg);
	    }
	}

	void Init()
	{
		BaseType_t rtn;
        /*
         *  Initialize UART port 2
         */
        UART2Init();

	    MbCmd = xQueueCreate(10, 128);
    	configASSERT(MbCmd);

	    // Create tasks
	    rtn=xTaskCreate(task, (const portCHAR *)"PrintTask",
	                tskStkSize, NULL, tskPrio, NULL);
	    configASSERT(rtn==pdPASS);
	}
}
