/**========================================================================
 * ?                                ABOUT
 * @author         :  N. O.
 * @email          :  
 * @repo           :  ver. 3
 * @createdOn      :  12/06/2021
 * @description    :  BMP20
 *========================================================================**/

/**========================================================================
 * *                      RISULTATI BMP20
 *   
 * Simboli: ✔ ✗  
 *   
 * ✔ Leggere Temperatura
 * ✔ Leggere Pressione 
 * ✗ Leggere Altitudine -> Valori Non Corretti
 * ✔ Leggere dai registri 
 *========================================================================**/

/*================================ Include ==============================*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

/*================================ Define & Const ==============================*/
#define BMP280_ADDR 0x76 // 7-bit address

#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_RESULT_PRESSURE 0xF7     // 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA // 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

#define BMP280_COMMAND_TEMPERATURE 0x2E
#define BMP280_COMMAND_PRESSURE0 0x25
#define BMP280_COMMAND_PRESSURE1 0x29
#define BMP280_COMMAND_PRESSURE2 0x2D
#define BMP280_COMMAND_PRESSURE3 0x31
#define BMP280_COMMAND_PRESSURE4 0x5D
#define BMP280_COMMAND_OVERSAMPLING_MAX 0xF5
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1 /**< R calibration = 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

/** No over-sampling. */
#define SAMPLING_NONE 0x00
/** 1x over-sampling. */
#define SAMPLING_X1 0x01
/** 2x over-sampling. */
#define SAMPLING_X2 0x02
/** 4x over-sampling. */
#define SAMPLING_X4 0x03
/** 8x over-sampling. */
#define SAMPLING_X8 0x04
/** 16x over-sampling. */
#define SAMPLING_X16 0x05

/** Sleep mode. */
#define MODE_SLEEP 0x00,
/** Forced mode. */
#define MODE_FORCED 0x01,
/** Normal mode. */
#define MODE_NORMAL 0x03,
/** Software reset. */
#define MODE_SOFT_RESET_CODE 0xB6

/** No filtering. */
#define FILTER_OFF 0x00
/** 2x filtering. */
#define FILTER_X2 0x01
/** 4x filtering. */
#define FILTER_X4 0x02
/** 8x filtering. */
#define FILTER_X8 0x03
/** 16x filtering. */
#define FILTER_X16 0x04

/** 1 ms standby. */
#define STANDBY_MS_1 0x00,
/** 62.5 ms standby. */
#define STANDBY_MS_63 0x01,
/** 125 ms standby. */
#define STANDBY_MS_125 0x02,
/** 250 ms standby. */
#define STANDBY_MS_250 0x03,
/** 500 ms standby. */
#define STANDBY_MS_500 0x04,
/** 1000 ms standby. */
#define STANDBY_MS_1000 0x05,
/** 2000 ms standby. */
#define STANDBY_MS_2000 0x06,
/** 4000 ms standby. */
#define STANDBY_MS_4000 0x07

/*================================ Function prototypes ==============================*/
void i2c_initialize(void);
uint8_t bmp280_initialize(void);
uint8_t getOversampling(void);
uint8_t setOversampling(uint8_t oss);
uint8_t startMeasurment(void);
uint8_t calcTemperature(double *T, double adc_T);
uint8_t calcPressure(double *P, double uP);
double sealevel(double P, double A);
double altitude(double P, double P0);
double waterBoilingPoint(double P);
char getError(void);
uint8_t getTemperatureAndPressure(double *T, double *P);
uint8_t readCalibration();
uint8_t readInt(uint8_t address, double *value);
uint8_t readUInt(uint8_t address, double *value);
uint8_t readBytes(uint8_t values[], uint8_t length);
uint8_t writeBytes(uint8_t values, uint8_t length);
uint8_t getUnPT(double *uP, double *uT);

/*================================ Global Variable ==============================*/
double dig_T1, dig_T2, dig_T3, dig_T4, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t oversampling, oversampling_t;
double t_fine;
char error;
const double P0 = 1013.25;

/*================================ Main  ==============================*/

int main()
{
    /* Initialize */
    stdio_init_all();
    printf("\n**************\n");
    printf("Inizio Inizializzazione\n");

    printf("stdio_init_all -> OK\n");

    i2c_initialize();
    printf("i2c_initialize -> OK\n");

    if (bmp280_initialize())
    {
        printf("bmp280_initialize -> OK\n");
    }
    else
    {
        printf("bmp280_initialize -> KO\n");
    }

    printf("Fine Inizializzazione\n");
    printf("**************\n\n");

    setOversampling(4);

    if (readCalibration())
    {
        printf("readCalibration -> OK\n");
    }
    else
    {
        printf("readCalibration -> KO\n");
    }

    while (1)
    {
        double T = 0, P = 0;
        uint8_t result = startMeasurment();

        if (result != 0)
        {
            printf("startMeasurment -> OK\n");
        }
        else
        {
            printf("startMeasurment -> KO\n");
        }
        if (result != 0)
        {
            sleep_ms(result);
            result = getTemperatureAndPressure(&T, &P);

            if (result != 0)
            {
                double A = altitude(P, P0);

                printf("T = %.2f degC\n", T);
                printf("P = %.2f mBar\n", P);
                printf("A = %.2f m\n", A);
                printf("Sea Level = %.2f m\n", sealevel(P, A));
                printf("Water Boiling Point = %.2f degC\n", waterBoilingPoint(P));
            }
            else
            {
                printf("Error.\n");
            }
        }
        else
        {
            printf("Error.\n");
        }
        sleep_ms(5000);
        printf("***********************************************\n");
    }
    return 0;
}

/*================================ INFO ==============================*/

/**========================================================================
 * *                        i2c_write_blocking
 * int i2c_write_blocking (i2c_inst_t *i2c, uint8_t addr,const uint8_t *src, size_t len, bool nostop)
 * 
 * Parameters:
 * i2c Either i2c0 or i2c1
 * addr Address of device to write to
 * src Pointer to data to send
 * len Length of data in bytes to send 
 * nostop If true, master retains control of the bus at the end of the transfer (no Stop is issued), and the next transfer
   will begin with a Restart rather than a Start.
 *  
 * Returns: Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.

 *========================================================================**/

/**========================================================================
 * *                         i2c_read_blocking
 *   int i2c_read_blocking (i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop)
 * 
 *   Parameters:
 *   i2c Either i2c0 or i2c1
 *   addr Address of device to read from
 *   dst Pointer to buffer to receive data
 *   len Length of data in bytes to receive
 *   nostop If true, master retains control of the bus at the end of the transfer (no Stop is issued), and the next transfer will begin with a Restart rather than a Start.
 * 
 *   Return: Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present 
 *========================================================================**/

/*================================ FUNCTION  ==============================*/

/**========================================================================
 **                           FUNCTION NAME
 *?  What does it do?
 *@param name type  
 *@param name type  
 *@return type
 *========================================================================**/

/**========================================================================
 **                           i2c_initialize
 *? initialize i2c
 *? This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
 *@param null   
 *@return void
 *========================================================================**/
void i2c_initialize(void)
{
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

/**========================================================================
 **                           bmp280_initialize
 *?  bmp280_initialize
 *?  Del buffer il primo parametro è il registro dove scrivere e il secondo è il valore da scrivere  
 *@return 1 OK, 0 KO
 *========================================================================**/
uint8_t bmp280_initialize(void)
{
    uint8_t buffer_RESET[] = {BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE};
    uint8_t buffer_MODE_NORMAL[] = {BMP280_REGISTER_CONTROL, MODE_NORMAL};
    uint8_t buffer_SAMPLING_X2[] = {BMP280_REGISTER_CONTROL, SAMPLING_X2};
    uint8_t buffer_SAMPLING_X16[] = {BMP280_REGISTER_CONTROL, SAMPLING_X16};
    uint8_t buffer_FILTER_X16[] = {BMP280_REGISTER_CONFIG, FILTER_X16};
    uint8_t buffer_STANDBY_MS_500[] = {BMP280_REGISTER_CONFIG, STANDBY_MS_500};

    uint8_t return_buffer_RESET = 0;
    uint8_t return_buffer_MODE_NORMAL = 0;
    uint8_t return_buffer_SAMPLING_X2 = 0;
    uint8_t return_buffer_SAMPLING_X16 = 0;
    uint8_t return_buffer_FILTER_X16 = 0;
    uint8_t return_buffer_STANDBY_MS_500 = 0;

    return_buffer_RESET = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_RESET, 2, false);
    sleep_ms(100);
    return_buffer_MODE_NORMAL = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_MODE_NORMAL, 2, false);
    sleep_ms(100);
    return_buffer_SAMPLING_X2 = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_SAMPLING_X2, 2, false);
    sleep_ms(100);
    return_buffer_SAMPLING_X16 = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_SAMPLING_X16, 2, false);
    sleep_ms(100);
    return_buffer_FILTER_X16 = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_FILTER_X16, 2, false);
    sleep_ms(100);
    return_buffer_STANDBY_MS_500 = i2c_write_blocking(i2c_default, BMP280_ADDR, buffer_STANDBY_MS_500, 2, false);
    sleep_ms(100);

    if (
        return_buffer_RESET != PICO_ERROR_GENERIC &&
        return_buffer_MODE_NORMAL != PICO_ERROR_GENERIC &&
        return_buffer_SAMPLING_X2 != PICO_ERROR_GENERIC &&
        return_buffer_SAMPLING_X16 != PICO_ERROR_GENERIC &&
        return_buffer_FILTER_X16 != PICO_ERROR_GENERIC &&
        return_buffer_STANDBY_MS_500 != PICO_ERROR_GENERIC)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**========================================================================
 **                            readCalibration
 *? Il BMP280 include i dati di calibrazione di fabbrica memorizzati sul dispositivo.
 *? Ogni dispositivo ha numeri diversi, questi devono essere recuperati e utilizzato nei calcoli durante le misurazioni.
 *? Recupera i dati di calibrazione dal dispositivo:
 *@param null null    
 *@return 1 se OK
 *@return 0 se KO
 *========================================================================**/
uint8_t readCalibration()
{
    if (readUInt(0x88, &dig_T1) && readInt(0x8A, &dig_T2) &&
        readInt(0x8C, &dig_T3) && readUInt(0x8E, &dig_P1) &&
        readInt(0x90, &dig_P2) && readInt(0x92, &dig_P3) &&
        readInt(0x94, &dig_P4) && readInt(0x96, &dig_P5) &&
        readInt(0x98, &dig_P6) && readInt(0x9A, &dig_P7) &&
        readInt(0x9C, &dig_P8) && readInt(0x9E, &dig_P9))
    {

        printf("dig_T1= %2.f\n", dig_T1);
        printf("dig_T2= %2.f\n", dig_T2);
        printf("dig_T3= %2.f\n", dig_T3);
        printf("dig_P1= %2.f\n", dig_P1);
        printf("dig_P2= %2.f\n", dig_P2);
        printf("dig_P3= %2.f\n", dig_P3);
        printf("dig_P4= %2.f\n", dig_P4);
        printf("dig_P5= %2.f\n", dig_P5);
        printf("dig_P6= %2.f\n", dig_P6);
        printf("dig_P7= %2.f\n", dig_P7);
        printf("dig_P8= %2.f\n", dig_P8);
        printf("dig_P9= %2.f\n", dig_P9);

        return (1);
    }
    else
    {
        return (0);
    }
}

/**========================================================================
 **                            readInt
 *? Read a signed integer (two bytes) from device
 *? Read an signed int (16 bits) from a BMP280 register
 *? address = register to start reading (plus subsequent register)
 *? value   = external variable to store data (function modifies value)
 *@param uint8_t address 
 *@param double &value  
 *@return 1 se OK
 *@return 0 se KO
 *========================================================================**/
uint8_t readInt(uint8_t address, double *value)
{
    uint8_t data[2]; //char is 4bit,1byte

    data[0] = address;
    if (readBytes(data, 2))
    {
        *value = (double)(int16_t)(((unsigned int)data[1] << 8) | (unsigned int)data[0]);
        return (1);
    }
    else
    {
        value = 0;
        return (0);
    }
}

/**========================================================================
 **                           readUInt
 *? Read a signed unsigned integer (two bytes) from device
 *? Read an signed int (16 bits) from a BMP280 register
 *? address = register to start reading (plus subsequent register)
 *? value   = external variable to store data (function modifies value)
 *@param uint8_t address 
 *@param double &value  
 *@return 1 se OK
 *@return 0 se KO
 *========================================================================**/
uint8_t readUInt(uint8_t address, double *value)
{
    unsigned char data[2]; //4bit
    data[0] = address;

    if (readBytes(data, 2))
    {
        *value = (double)(unsigned int)(((unsigned int)data[1] << 8) | (unsigned int)data[0]);
        return (1);
    }
    else
    {
        value = 0;
        return (0);
    }
}

/**========================================================================
 **                           readBytes
 *?  Read an array of bytes from device
 *?  value: external array to hold data. Put starting register in values[0].
 *?  length: number of bytes to read
 *@param uint8_t  *values
 *@param uint8_t length
 *@return returns 1 for success, 0 for fail, with read bytes in values[] array
 *========================================================================**/
uint8_t readBytes(uint8_t values[], uint8_t length)
{
    uint8_t buffer[length];
    uint8_t val = values[0];
    i2c_write_blocking(i2c_default, BMP280_ADDR, &val, 1, true); //accedi al registro da leggere
    if (i2c_read_blocking(i2c_default, BMP280_ADDR, buffer, length, false) != PICO_ERROR_GENERIC)
    {
        for (int i = 0; i < length; i++)
        {
            values[i] = buffer[i];
            //printf("values[%d] = %d\n", i, values[i]);
            //printf("buffer[%d] = %d\n", i, buffer[i]);
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

/**========================================================================
 **                           writeBytes
 *?  Write an array of bytes to device
 *? @param : values = external array of data to write. Put starting register in values[0].
 *? @param : length = number of bytes to write
 *@param uint8_t  *values
 *@param uint8_t length
 *@return returns 1 for success, 0 for fail, with write bytes in values[] array
 *========================================================================**/
uint8_t writeBytes(uint8_t values, uint8_t length)
{
    if (i2c_write_blocking(i2c_default, BMP280_ADDR, &values, length, false) != PICO_ERROR_GENERIC)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**========================================================================
 **                            getOversampling
 *?  get Oversampling
 *@return Oversampling
 *========================================================================**/
uint8_t getOversampling(void)
{
    return oversampling;
}

/**========================================================================
 **                           FUNCTION NAME
 *?  setOversampling
 *@param uint8_t oss 
 *@return 1 ok
 *========================================================================**/
uint8_t setOversampling(uint8_t oss)
{
    oversampling = oss;
    return (1);
}

/**========================================================================
 **                           startMeasurment
 *?  Begin a measurement cycle
 *?  command BMP280 to start a pressure measurement
 *?	 oversampling: 0 - 3 for oversampling value   
 *@return returns (number of ms to wait) for success, 0 for fail
 *========================================================================**/
uint8_t startMeasurment(void)

{
    uint8_t data[2], result, delay;

    data[0] = BMP280_REG_CONTROL;

    switch (oversampling)
    {
    case 0:
        data[1] = BMP280_COMMAND_PRESSURE0;
        oversampling_t = 1;
        delay = 8;
        break;
    case 1:
        data[1] = BMP280_COMMAND_PRESSURE1;
        oversampling_t = 1;
        delay = 10;
        break;
    case 2:
        data[1] = BMP280_COMMAND_PRESSURE2;
        oversampling_t = 1;
        delay = 15;
        break;
    case 3:
        data[1] = BMP280_COMMAND_PRESSURE3;
        oversampling_t = 1;
        delay = 24;
        break;
    case 4:
        data[1] = BMP280_COMMAND_PRESSURE4;
        oversampling_t = 1;
        delay = 45;
        break;
    case 16:
        data[1] = BMP280_COMMAND_OVERSAMPLING_MAX;
        oversampling_t = 1;
        delay = 80; //I cannot find any data about timings in datasheet for x16 pressure and x16 temeprature oversampling
        break;      //I guess this is enough time (maybe it can be even smaller ~60ms)
    default:
        data[1] = BMP280_COMMAND_PRESSURE0;
        delay = 9;
        break;
    }
    result = i2c_write_blocking(i2c_default, BMP280_ADDR, data, 2, false);
    //writeBytes(data, 2);
    if (result != PICO_ERROR_GENERIC) // good write?
        return (delay);               // return the delay in ms (rounded up) to wait before retrieving data
    else
        return (0); // or return 0 if there was a problem communicating with the BMP
}

/**========================================================================
 **                           getUnPT
 *?  Get the uncalibrated pressure and temperature value.
 *?  @param : uP = stores the uncalibrated pressure value.(20bit)
 *?  @param : uT = stores the uncalibrated temperature value.(20bit)
 *@param double *uP
 *@param double *uT  
 *@return result, 1 OK , 0 KO
 *========================================================================**/
uint8_t getUnPT(double *uP, double *uT)
{
    uint8_t data[6];
    uint8_t result;

    data[0] = BMP280_REG_RESULT_PRESSURE; //0xF7

    result = readBytes(data, 6); // 0xF7; xF8, 0xF9, 0xFA, 0xFB, 0xFC
    if (result)                  // good read
    {
        double factor = pow(2, 4);
        *uP = (double)(data[0] * 4096 + data[1] * 16 + data[2] / 16); //20bit UP
        *uT = (double)(data[3] * 4096 + data[4] * 16 + data[5] / 16); //20bit UT

        //printf("uT: %2.f\n", *uT);
        //printf("uP: %2.f\n", *uP);
    }
    return (result);
}

/**========================================================================
 **                        getTemperatureAndPressure
 *?  Retrieve temperature and pressure.
 *?  T = stores the temperature value in degC.
 *?  P = stores the pressure value in mBar.
 *@param double *T
 *@param double *P   
 *@return type
 *========================================================================**/
uint8_t getTemperatureAndPressure(double *T, double *P)
{
    double uT = 0;
    double uP = 0;
    char result = getUnPT(&uP, &uT);

    if (result != 0)
    {
        // calculate the temperature
        result = calcTemperature(T, uT);
        if (result)
        {
            // calculate the pressure
            result = calcPressure(P, uP);
            if (result)
                return (1);
            else
                error = 3; // pressure error ;
            return (9);
        }
        else
            error = 2; // temperature error ;
    }
    else
        error = 1;

    return (9);
}

/**========================================================================
 **                           calcTemperature
 *?  temperature calculation
 *?  T  = stores the temperature value after calculation.
 *?  uT = the uncalibrated temperature value.
 *@param double *T
 *@param double *adc_T 
 *@return 1 OK, 0 KO
 *========================================================================**/
uint8_t calcTemperature(double *T, double adc_T)
//
{
    //printf("adc_T = "); printfln(adc_T,DEC);
    double temp = 0;
    double var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2;
    double var2 = ((adc_T / 131072.0 - dig_T1 / 8192.0) * (adc_T / 131072.0 - dig_T1 / 8192.0)) * dig_T3;
    t_fine = var1 + var2;
    temp = (var1 + var2) / 5120.0;
    *T = temp;

    //printf("var1: %2.f\n", var1);
    //printf("var2: %2.f\n", var2);
    //printf("t_fine: %2.f\n", t_fine);
    //printf("T: %2.f\n", *T);

    if (temp > 100 || temp < -100)
    {
        return 0;
    }

    return (1);
}

/**========================================================================
 **                           calcPressure
 *?  Pressure calculation from uncalibrated pressure value.
 *?  P  = stores the pressure value.
 *?  uP = uncalibrated pressure value. 
 *@param double *P
 *@param double *uP   
 *@return 1 OK, 0 KO
 *========================================================================**/
uint8_t calcPressure(double *P, double uP)
{
    //char result;
    double var1, var2;
    double temp = 0;
    var1 = (t_fine / 2.0) - 64000.0;
    var2 = var1 * (var1 * dig_P6 / 32768.0); //not overflow
    var2 = var2 + (var1 * dig_P5 * 2.0);     //overflow
    var2 = (var2 / 4.0) + ((dig_P4)*65536.0);
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * dig_P1;
    temp = 1048576.0 - uP;
    temp = (temp - (var2 / 4096.0)) * 6250.0 / var1; //overflow
    var1 = dig_P9 * temp * temp / 2147483648.0;      //overflow
    var2 = temp * dig_P8 / 32768.0;
    temp = temp + (var1 + var2 + dig_P7) / 16.0;
    temp = temp / 100.0;
    if (temp > 1200.0 || temp < 800.0)
    {
        return 0;
    }
    *P = temp;
    return (1);
}

/**========================================================================
 **                           sealevel
 *? Given a pressure P (mb) taken at a specific altitude (meters),
 *? return the equivalent pressure (mb) at sea level.
 *? This produces pressure readings that can be used for weather measurements.
 *@param double P 
 *@param double A  
 *@return return the equivalent pressure (mb) at sea level
 *========================================================================**/
double sealevel(double P, double A)
{
    return (P / pow(1 - (A / 44330.0), 5.255));
}

/**========================================================================
 **                           altitude
 *?  Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
 *@param double P 
 *@param double P0 
 *@return altitude (meters) above baseline.
 *========================================================================**/
double altitude(double P, double P0)
{
    return (44330.0 * (1 - pow(P / P0, (1 / 5.255))));
}

/**========================================================================
 **                           waterBoilingPoint
 *?  Punto di elbolizione dell'acqua data la pressione
 *@param double P  
 *@return type
 *========================================================================**/
double waterBoilingPoint(double P)
{
    return (234.175 * log(P / 6.1078)) / (17.08085 - log(P / 6.1078));
}

/**========================================================================
 **                           getError
 *? If any library command fails, you can retrieve an extended
 *?	error code using this command. Errors are from the wire library: 
 *?	0 = Success
 *?	1 = Data too long to fit in transmit buffer
 *?	2 = Received NACK on transmit of address
 *?	3 = Received NACK on transmit of data
 *?	4 = Other error
 *@return error
 *========================================================================**/
char getError(void)
{
    return (error);
}
