
/*******************************************************************************
Copyright � 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file vl6180x_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */

#include "vl6180x_platform.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define ACK_CHECK_EN true

/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  0         on success
 */
static int VL6180x_WriteMulti(VL6180xDev_t Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    LOG_FUNCTION_START("");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write I2C address
    i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);

    // write register
    i2c_master_write_byte(cmd, (index >> 8) & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, index & 0xff, ACK_CHECK_EN);

    // Data
    // Note: Needed to use i2c_master_write_byte as i2c_master_write will not expect an ack
    // after each byte
    for (int i = 0; i < count; i++)
    {
        i2c_master_write_byte(cmd, *(pdata + i), ACK_CHECK_EN);
    }

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(Dev->i2c_port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    int status = (ret == ESP_OK) ? 0 : -1;
    LOG_FUNCTION_END(status);
    return status;
}

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  0         on success
 */
static int VL6180x_ReadMulti(VL6180xDev_t Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    LOG_FUNCTION_START("");
    // I2C write
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ////// First tell the VL53L0X which register we are reading from
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    // Write register
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (index >> 8) & 0xff, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, index & 0xff, ACK_CHECK_EN));

    ////// Second, read from the register
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ( Dev->i2c_address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN));

    // Read data from register
    ESP_ERROR_CHECK(i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(Dev->i2c_port_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    int status = (ret == ESP_OK) ? 0 : -1;
    LOG_FUNCTION_END(status);
    return status;
}

/**
 * Write single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      8 bit register data
 * @return  0         on success
 */
int VL6180x_WrByte(VL6180xDev_t Dev, uint16_t index, uint8_t data)
{
    return VL6180x_WriteMulti(Dev, index, &data, 1);
}

/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  0         on success
 */
int VL6180x_WrWord(VL6180xDev_t Dev, uint16_t index, uint16_t data)
{
    uint8_t buffer[2]; // 2
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    return VL6180x_WriteMulti(Dev, index, buffer, 2);
}

/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  0         on success
 */
int VL6180x_WrDWord(VL6180xDev_t Dev, uint16_t index, uint32_t data)
{
    uint8_t buffer[4]; // 4

    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    return VL6180x_WriteMulti(Dev, index, buffer, 4);
}

/**
 * Read single byte register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 8 bit data
 * @return  0         on success
 */
int VL6180x_RdByte(VL6180xDev_t Dev, uint16_t index, uint8_t *data)
{
    return VL6180x_ReadMulti(Dev, index, data, 1);
}

/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  0         on success
 */
int VL6180x_RdWord(VL6180xDev_t Dev, uint16_t index, uint16_t *data)
{
    int status;
    uint8_t  buffer[2];

    status = VL6180x_ReadMulti(Dev, index, buffer, 2);
    *data = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;
}

/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  0         on success
 */
int VL6180x_RdDWord(VL6180xDev_t Dev, uint16_t index, uint32_t *data)
{
    int status;
    uint8_t  buffer[4];

    status = VL6180x_ReadMulti(Dev, index, buffer, 4);
    *data = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) +
             ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;
}

/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  0         on success
 */
int VL6180x_UpdateByte(VL6180xDev_t Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    int status;
    uint8_t data;

    status = VL6180x_RdByte(Dev, index, &data);

    if (status != 0)
        return status;

    data = (data & AndData) | OrData;

    return VL6180x_WrByte(Dev, index, data);
}
