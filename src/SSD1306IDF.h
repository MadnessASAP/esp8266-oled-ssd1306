/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

#ifndef SSD1306IDF_h
#define SSD1306IDF_h
#ifdef IDF_VER
#include "driver/i2c.h"
#include "sdkconfig"
#include "OLEDDisplay.h"

// TODO: Finish implementing Kconfig options
#define _I2C_PORT I2C_NUM_##CONFIG
#ifdef CONFIG_SSD1306_GEOMETRY_128_64
  #define _OLED_GEOMETRY GEOMETRY_128_64
#elif CONFIG_SSD1306_GEOMETRY_128_32
  #define _OLED_GEOMETRY GEOMETRY_128_32
#else
  #define _OLED_GEOMETRY GEOMETRY_128_64
#endif

class SSD1306IDF : public OLEDDisplay {
  private:
    uint8_t             _address;
    gpio_num_t          _sda;
    gpio_num_t          _scl;
    i2c_port_t          _port;

  public:
    SSD1306IDF(uint8_t _address, gpio_num_t _sda, gpio_num_t _scl
      OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64, i2c_port_t _port = I2C_NUM_0) {
      setGeometry(g);

      this->_address = _address;
      this->_sda = _sda;
      this->_scl = _scl;
      this->_port = _port;
    }

    bool connect() {
      i2c_config_t i2c_config = {};
      i2c_config.mode = I2C_MODE_MASTER;
      i2c_config.sda_io_num = _sda;
      i2c_config.scl_io_num = _scl;
      i2c_config.master.clk_speed = 400e3;  // 400 kHz
      if(i2c_param_config(_port, &i2c_config) != ESP_OK) return false;
      if(i2c_driver_install(_port, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) return false;
      
      return true;
    }

    void display(void) {
      #ifdef OLEDDISPLAY_DOUBLE_BUFFER
        uint8_t minBoundY = UINT8_MAX;
        uint8_t maxBoundY = 0;  
        uint8_t minBoundX = UINT8_MAX;
        uint8_t maxBoundX = 0;  
        
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (uint8_t y = 0; y < (displayHeight / 8); y++) {
          for (uint8_t x = 0; x < displayWidth; x++) {
            uint16_t pos = x + y * displayWidth;
            if (buffer[pos] != buffer_back[pos]) {
              minBoundY = min(minBoundY, y);
              maxBoundY = max(maxBoundY, y);
              minBoundX = min(minBoundX, x);
              maxBoundX = max(maxBoundX, x);
            }
            buffer_back[pos] = buffer[pos];
          }
        }  
        
        // If the minBoundY wasn't updated
        // we can savely assume that buffer_back[pos] == buffer[pos]
        // holdes true for all values of pos
        if (minBoundY == UINT8_MAX) return;  
        sendCommand(COLUMNADDR);
        sendCommand(minBoundX);
        sendCommand(maxBoundX);  
        sendCommand(PAGEADDR);
        sendCommand(minBoundY);
        sendCommand(maxBoundY);
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (_address << 1), true); // b[0] == 0 (Write Mode)
          i2c_master_write_byte(cmd, 0x40, true);            // b[7] == 1 (Continuation Mode, all data bytes) 
            for(uint8_t y = minBoundY; y <= maxBoundY; y++)
              i2c_master_write(cmd, &buffer[minBoundX + y * width()], (maxBoundX - minBoundX) + 1 , true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
      #else // No double buffering
        sendCommand(COLUMNADDR);
        sendCommand(0x0);
        sendCommand(0x7F);  
        sendCommand(PAGEADDR);
        sendCommand(0x0);  
        if (geometry == GEOMETRY_128_64) {
          sendCommand(0x7);
        } else if (geometry == GEOMETRY_128_32) {
          sendCommand(0x3);
        }  
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (_address << 1), true); // b[0] == 0 (Write Mode)
          i2c_master_write_byte(cmd, 0x40, true);            // b[7] == 1 (Continuation Mode, all data bytes) 
            for(uint8_t y = minBoundY; y <= maxBoundY; y++)
              i2c_master_write(cmd, buffer, displayBufferSize, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
      #endif
    }

  private:
	int getBufferOffset(void) {
		return 0;
	}
    inline void sendCommand(uint8_t command) __attribute__((always_inline)){
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
          i2c_master_write_byte(cmd, (_address << 1), true); // b[0] == 0 (Write Mode)
          i2c_master_write_byte(cmd, 0x80, true);
          i2c_master_write_byte(cmd, command, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
};

#else
#error "Can only be used with ESP-IDF framework"
#endif

#endif
