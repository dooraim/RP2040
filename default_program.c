/**========================================================================
 * ?                                ABOUT
 * @author         :  
 * @email          :  
 * @repo           :  
 * @createdOn      :  
 * @description    :  
 *========================================================================**/

/**========================================================================
 * *                      RISULTATI 
 *   
 * Simboli: ✔ ✗  
 *   
 * 
 *========================================================================**/

/*================================ Include ==============================*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

/*================================ Define & Const ==============================*/



/*================================ Function prototypes ==============================*/


/*================================ Main  ==============================*/

int main()
{
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
