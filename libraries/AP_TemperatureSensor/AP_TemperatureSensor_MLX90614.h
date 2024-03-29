#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_MLX90614_ENABLED
    #define AP_TEMPERATURE_SENSOR_MLX90614_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_MLX90614_ENABLED

/**************************************************************************************************/
/* Definitions                                                                                    */
/**************************************************************************************************/

#define MLX90614_I2CDEFAULTADDR 0x5A    /**< Device default slave address */
#define MLX90614_BROADCASTADDR  0       /**< Device broadcast slave address */
#define MLX90614_CRC8POLY       7       /**< CRC polynomial = X8+X2+X1+1 */
#define MLX90614_XDLY           25      /**< Experimentally determined delay to prevent read

/** RAM addresses. */
#define MLX90614_RAWIR1         0x04    /**< RAM reg - Raw temperature, source #1 */
#define MLX90614_RAWIR2         0x05    /**< RAM reg - Raw temperature, source #2 */
#define MLX90614_TA             0x06    /**< RAM reg - Linearized temperature, ambient */
#define MLX90614_TOBJ1          0x07    /**< RAM reg - Linearized temperature, source #1 */
#define MLX90614_TOBJ2          0x08    /**< RAM reg - Linearized temperature, source #2 */

/** EEPROM addresses. */
#define MLX90614_TOMAX          0x00    /**< EEPROM reg - Customer dependent object temperature range maximum */
#define MLX90614_TOMIN          0x01    /**< EEPROM reg - Customer dependent object temperature range minimum */
#define MLX90614_PWMCTRL        0x02    /**< EEPROM reg - Pulse width modulation output control register */
#define MLX90614_TARANGE        0x03    /**< EEPROM reg - Customer dependent ambient temperature range */
#define MLX90614_EMISS          0x04    /**< EEPROM reg - Object emissivity register */
#define MLX90614_CONFIG         0x05    /**< EEPROM reg - Configuration register */
#define MLX90614_ADDR           0x0E    /**< EEPROM reg - SMBus address */
#define MLX90614_ID1            0x1C    /**< EEPROM reg - ID numer (w1) */
#define MLX90614_ID2            0x1D    /**< EEPROM reg - ID numer (w2) */
#define MLX90614_ID3            0x1E    /**< EEPROM reg - ID numer (w3) */
#define MLX90614_ID4            0x1F    /**< EEPROM reg - ID numer (w4) */

#define MLX90614_RFLAGCMD       0xF0    /**< Read R/W Flags register command */

/** Read flags - bitmask. */
#define MLX90614_EEBUSY         0x80    /**< R/W flag bitmask - EEProm is busy (writing/erasing) */
#define MLX90614_EE_DEAD        0x20    /**< R/W flag bitmask - EEProm double error has occurred */
#define MLX90614_INIT           0x10    /**< R/W flag bitmask - POR initialization is still ongoing */

/** R/W Error flags - bitmask. */
#define MLX90614_NORWERROR      0       /**< R/W error bitmask - No Errors */
#define MLX90614_DATATOOLONG    1       /**< R/W error bitmask - Data is too long */
#define MLX90614_TXADDRNACK     2       /**< R/W error bitmask - TX address not acknowledged */
#define MLX90614_TXDATANACK     4       /**< R/W error bitmask - TX data not acknowledged */
#define MLX90614_TXOTHER        8       /**< R/W error bitmask - Unknown error */
#define MLX90614_RXCRC          0x10    /**< R/W error bitmask - Receiver CRC mismatch */
#define MLX90614_INVALIDATA     0x20    /**< R/W error bitmask - RX/TX Data fails selection criteria */
#define MLX90614_EECORRUPT      0x40    /**< R/W error bitmask - The EEProm is likely to be corrupted */
#define MLX90614_RFLGERR        0x80    /**< R/W error bitmask - R/W flags register access error */


class AP_TemperatureSensor_MLX90614 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

    public:
    void init(void) override;

    void update() override {};


private:
   
    // Hold return value in _timer
    uint16_t _crude_value;

    // update the temperature, called at 20Hz
    void _timer(void);

    uint16_t read_data(uint8_t cmd);

   
    uint16_t read_eeprom(uint8_t address) {return read_data(address | 0x20);};

};


#endif