/**========================================================================
 * ?                                ABOUT
 * @author         :  N.O.
 * @email          :  
 * @repo           :  v.7
 * @createdOn      :  24/05/2021
 * @description    :  USE LCD I2C
 *========================================================================**/

/**========================================================================
 * *                      RISULTATI LCD_16X02_I2C
 *   
 * Simboli: ✔ ✗  
 *   
 * ✔ Trasformare un float in carattere e stamparlo sul LCD 16X02 i2c
 * ✔ Manipolare gli elementi del vettore "static uint8_t *message[]"
 * ✔ Lettura temperatura Chip RP2040
 * ✔ Illuminare singoli elementi del LCD 16X02 i2c (createChar)
 * ✔ Stampare mediante bit map
 *========================================================================**/

/*================================ Include ==============================*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"

/*================================ Define & Const ==============================*/

const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;
const int LCD_ENABLE_BIT = 0x04;

// By default these LCD display drivers are on bus address 0x27
static int addr_lcd = 0x27;

// Modes for lcd_send_byte
#define LCD_CHARACTER 1
#define LCD_COMMAND 0

#define MAX_LINES 2
#define MAX_CHARS 16

#define TIME 2000

/*================================ Function prototypes ==============================*/
void i2c_initialize(void);
void i2c_write_byte(uint8_t val);
void lcd_toggle_enable(uint8_t val);
void lcd_send_byte(uint8_t val, int mode);
void lcd_clear(void);
void lcd_set_cursor(int line, int position);
static void inline lcd_char(char val);
void lcd_string(const char *s);
void lcd_init();
void createChar(uint8_t location, uint8_t charmap[]);

/*================================ Main ==============================*/

int main()
{

    /* Initialize */
    stdio_init_all();
    printf("\n**************\n");
    printf("Inizio Inizializzazione\n");

    printf("stdio_init_all -> OK\n");

    i2c_initialize();
    printf("i2c_initialize -> OK\n");

    lcd_init();
    printf("lcd_init -> OK\n");

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    printf("adc_init -> OK\n");
    printf("adc_set_temp_sensor_enabled -> OK\n");

    printf("Fine Inizializzazione\n");
    printf("**************\n\n");

    static uint8_t *message[] =
        {
            "RP2040 by", "Raspberry Pi",
            "A brand new", "microcontroller",
            "Twin core M0", "Full C SDK",
            "More power in", "your product",
            "More beans", "than Heinz!"};

    //* Il massimo numero di elementi di questo vettore sono 8, perch+ la memoria è di 8 byte
    static uint8_t my_bitmap[][8] =
        {
            // chequer
            {0x15, 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15, 0x00},
            // up arrow
            {0x04, 0x0E, 0x1F, 0x04, 0x04, 0x04, 0x00, 0x00},
            // down arrow
            {0x00, 0x00, 0x04, 0x04, 0x04, 0x1F, 0x0E, 0x04},
            // rectangle
            {0x00, 0x1F, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x00},
            // up-left arrow
            {0x1F, 0x1E, 0x1C, 0x1A, 0x11, 0x00, 0x00, 0x00},
            // up-right arrow
            {0x1F, 0x0F, 0x07, 0x0B, 0x11, 0x00, 0x00, 0x00},
            // down-left arrow
            {0x00, 0x00, 0x00, 0x11, 0x1A, 0x1C, 0x1E, 0x1F},
            // down-right arrow
            {0x00, 0x00, 0x00, 0x11, 0x0B, 0x07, 0x0F, 0x1F},
        };

    //* Il massimo numero di elementi di questo vettore sono 8, perch+ la memoria è di 8 byte
    static uint8_t my_bitmap_2[][8] =
        {
            // heart
            {0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000},
            // bell
            {0b00100, 0b01110, 0b01110, 0b01110, 0b11111, 0b00000, 0b00100, 0b00000},
            // alien
            {0b11111, 0b10101, 0b11111, 0b11111, 0b01110, 0b01010, 0b11011, 0b00000},
            //check
            {0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000, 0b00000, 0b00000},
            //speaker
            {0b00001, 0b00011, 0b01111, 0b01111, 0b01111, 0b00011, 0b00001, 0b00000},
            //sound
            {0b00001, 0b00011, 0b00101, 0b01001, 0b01001, 0b01011, 0b11011, 0b11000},
            //skull
            {0b00000, 0b01110, 0b10101, 0b11011, 0b01110, 0b01110, 0b00000, 0b00000},
            //lock
            {0b01110, 0b10001, 0b10001, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000},
        };

    uint16_t raw = 0;
    const float conversion_factor = 3.3f / (1 << 12);
    float result = 0;
    float temp_core_M0 = 0;

    //* Converto un float in una stringa per poi stamparlo sul lcd grazie alla finzione sprintf
    float pi = 113.097335529 / 36;
    char str[10];
    sprintf(str, "%f", pi);
    printf("pi: %f\n", pi);
    printf("str: %s\n", str);
    //lcd_string(str);

    int size_my_bipmap = sizeof(my_bitmap) / sizeof(my_bitmap[0]);
    for (int i = 0; i < size_my_bipmap; i++)
    {
        createChar(i, (uint8_t *)my_bitmap[i]);
    }

    while (1)
    {
        //* Routine che stampa temperatura del core sul LCD_1602_I2C
        raw = adc_read();
        result = raw * conversion_factor;
        temp_core_M0 = 27 - ((result - 0.706) / 0.001721);
        char str[10];
        sprintf(str, "%f", temp_core_M0); //converto float in stringa
        printf("Temp Core RP2040: %.2f C\n", temp_core_M0);
        lcd_set_cursor(0, 0);
        lcd_string("Temp Core: ");
        lcd_set_cursor(1, 0);
        lcd_string(str);
        sleep_ms(TIME);
        lcd_clear();

        //* Routine che stampa message sul LCD_1602_I2C
        for (int m = 0; m < sizeof(message) / sizeof(message[0]); m += MAX_LINES)
        {
            for (int line = 0; line < MAX_LINES; line++)
            {
                lcd_set_cursor(line, (MAX_CHARS / 2) - strlen(message[m + line]) / 2);
                lcd_string(message[m + line]);
                printf("m: %d, line: %d -> word: %s\n", m, line, message[m + line]);
            }
            sleep_ms(TIME - 1000);
            lcd_clear();
        }

        //* Routine che stampa my_bitmap sul LCD_1602_I2C
        for (int i = 0; i < size_my_bipmap; i++)
        {
            lcd_set_cursor(0, i);
            lcd_send_byte(i, 1);
        }
        sleep_ms(TIME);
        lcd_clear();
    }

    return 0;
}

/*================================ INFO ==============================*/
/**========================================================================
 * *                               Info LCD
 * 
 * NOTE: The panel must be capable of being driven at 3.3v NOT 5v. The Pico
 * GPIO (and therefor I2C) cannot be used at 5v.
 * 
 * You will need to use a level shifter on the I2C lines if you want to run the
 * board at 5v.
 * 
 * Connections on Raspberry Pi Pico board, other boards may vary.
 * 
 * GPIO 4 (pin 6)-> SDA on LCD bridge board
 * GPIO 5 (pin 7)-> SCL on LCD bridge board
 * 3.3v (pin 36) -> VCC on LCD bridge board
 * GND (pin 38)  -> GND on LCD bridge board
 *========================================================================**/

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
 **                           i2c_write_byte
 *?  Quick helper function for single byte transfers
 *@param uint8_t val   
 *@return void
 *========================================================================**/
void i2c_write_byte(uint8_t val)
{
#ifdef i2c_default
    i2c_write_blocking(i2c_default, addr_lcd, &val, 1, false);
#endif
}

/**========================================================================
 **                            lcd_toggle_enable
 *?  Toggle enable pin on LCD display
 *?  We cannot do this too quickly or things don't work
 *@param uint8_t val 
 *@return void
 *========================================================================**/
void lcd_toggle_enable(uint8_t val)
{
    #define DELAY_US 600
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

/**========================================================================
 **                           lcd_send_byte
 *?  The display is sent a byte as two separate nibble transfers
 *@param uint8_t val 
 *@param int mode
 *@return void
 *========================================================================**/
void lcd_send_byte(uint8_t val, int mode)
{
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    i2c_write_byte(high);
    lcd_toggle_enable(high);
    i2c_write_byte(low);
    lcd_toggle_enable(low);
}

/**========================================================================
 **                            lcd_clear
 *?  Recall lcd_send_byte
 *@param null
 *@return void
 *========================================================================**/
void lcd_clear(void)
{
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

/**========================================================================
 **                           lcd_set_cursor
 *?  go to location on LCD
 *@param int line  
 *@param int position  
 *@return void
 *========================================================================**/
void lcd_set_cursor(int line, int position)
{
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, LCD_COMMAND);
}

/**========================================================================
 **                           lcd_char
 *?  Recall lcd_send_byte
 *@param char val 
 *@return void
 *========================================================================**/
static void inline lcd_char(char val)
{
    lcd_send_byte(val, LCD_CHARACTER);
}

/**========================================================================
 **                        lcd_string   
 *?  What does it do?
 *@param const char *s 
 *@return void
 *========================================================================**/
void lcd_string(const char *s)
{
    while (*s)
    {
        lcd_char(*s++);
    }
}

/**========================================================================
 **                         lcd_init  
 *?  initialize Lcd
 *@param null  
 *@return void
 *========================================================================**/
void lcd_init()
{
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);

    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}

/**========================================================================
 **                           createChar
 *?  Crea un nuovo carattere e lo inserisce in memoria, al massimo 8
 *@param uint8_t location
 *@param uint8_t charmap[]
 *@return void
 *========================================================================**/
void createChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    lcd_send_byte((LCD_SETCGRAMADDR | (location << 3)), 0);
    for (int i = 0; i < 8; i++)
    {
        lcd_send_byte(charmap[i], 1);
    }
}
