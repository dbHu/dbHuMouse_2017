#ifndef __DBMOUSE_H
#define __DBMOUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on DBMOUSE are active high. */
#define DBMOUSE_LED_OFF (1)
#define DBMOUSE_LED_ON  (0)

/*!
 *  @def    DBMOUSE_GPIOName
 *  @brief  Enum of GPIO names on the DBMOUSE dev board
 */
typedef enum DBMOUSE_GPIOName{
    DBMOUSE_IR_FL = 0,  // ir_fl must be 0
    DBMOUSE_IR_FR,      // order must remain no change
    DBMOUSE_IR_SL,      // cuz some routines depends on these.
    DBMOUSE_IR_SR,
    EK_TM4C1294XL_ENCRA,
    EK_TM4C1294XL_ENCRB,
    EK_TM4C1294XL_ENCLA,
    EK_TM4C1294XL_ENCLB,
    DBMOUSE_LED_0,
    DBMOUSE_LED_1,
    DBMOUSE_LED_2,
	DBMOUSE_GPIOCOUNT
} DBMOUSE_GPIOName;

/*!
 *  @def    DBMOUSE_I2CName
 *  @brief  Enum of I2C names on the DBMOUSE dev board
 */
typedef enum DBMOUSE_I2CName {
    DBMOUSE_I2C_IMU = 0,
    DBMOUSE_I2CCOUNT
} DBMOUSE_I2CName;

/*!
 *  @def    DBMOUSE_PWMName
 *  @brief  Enum of PWM names on the DBMOUSE dev board
 */
typedef enum DBMOUSE_PWMName {
    DBMOUSE_PWM_RP = 0,
    DBMOUSE_PWM_RN,
    DBMOUSE_PWM_LP,
    DBMOUSE_PWM_LN,
    DBMOUSE_PWMCOUNT
} DBMOUSE_PWMName;

/*!
 *  @def    DBMOUSE_UARTName
 *  @brief  Enum of UARTs on the DBMOUSE dev board
 */
typedef enum DBMOUSE_UARTName {
    DBMOUSE_UART_DBG = 0,
    DBMOUSE_UARTCOUNT
} DBMOUSE_UARTName;

#define UART_BUF_LEN 128

/*!
 *  @brief  Initialize board specific DMA settings
 *
 *  This function creates a hwi in case the DMA controller creates an error
 *  interrrupt, enables the DMA and supplies it with a uDMA control table.
 */
extern void DBMOUSE_initDMA(void);

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 *  This includes:
 *     - Flash wait states based on the process
 *     - Disable clock source to watchdog module
 *     - Enable clock sources for peripherals
 */
extern void DBMOUSE_initGeneral(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_PinConfig
 *  variable.
 */
extern void DBMOUSE_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void DBMOUSE_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void DBMOUSE_initPWM(void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
//extern void DBMOUSE_initSDSPI(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
//extern void DBMOUSE_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void DBMOUSE_initUART(void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
//extern void DBMOUSE_initUSB(DBMOUSE_USBMode usbMode);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
//extern void DBMOUSE_initWatchdog(void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 *
 *  A SimpleLink CC3100 device or module is required and must be connected to
 *  use the WiFi driver.
 */
//extern void DBMOUSE_initWiFi(void);

#ifdef __cplusplus
}
#endif

#endif /* __DBMOUSE_H */
