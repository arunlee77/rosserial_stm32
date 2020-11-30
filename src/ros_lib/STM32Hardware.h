/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#define STM32H7xx  // TODO Move it to build flag
#ifdef STM32F3xx
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#endif /* STM32F3xx */
#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif /* STM32F4xx */
#ifdef STM32F7xx
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#endif /* STM32F7xx */
#ifdef STM32H7xx
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "usbd_cdc_if.h"
#endif /* STM32F7xx */

#include "main.h"

#ifdef USE_ETHERNET_FOR_ROS
#include "RosInterface.hpp"
extern RosInterface* rosInterfacePtr;
#endif

#ifdef USE_UART_FOR_ROS
extern UART_HandleTypeDef huart3;
#endif

#ifdef USE_USB_FOR_ROS
// Temporary usage in Nucleo board
#endif

class STM32Hardware {
  protected:
#ifndef USE_ETHERNET_FOR_ROS
    const static uint16_t rbuflen = 1024;
    uint32_t rCurrentSize;
    uint8_t rbuf[rbuflen];
    uint32_t rind;

#ifdef USE_UART_FOR_ROS
    UART_HandleTypeDef *huart;
    inline uint32_t getRdmaInd(void){ return (rbuflen - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (rbuflen - 1); }
#endif
#ifdef USE_USB_FOR_ROS
    inline uint32_t getRdmaInd(void){ return (rbuflen -rCurrentSize) & (rbuflen - 1); }
#endif

    const static uint16_t tbuflen = 512;
    uint8_t tbuf[tbuflen];
    uint32_t twind, tfind;
#endif

  public:
    STM32Hardware()
#ifndef USE_ETHERNET_FOR_ROS
#ifdef USE_UART_FOR_ROS
     : huart(&huart3),
#endif
	  rCurrentSize(0), rind(0), twind(0), tfind(0)
#endif
    { }

#ifdef USE_UART_FOR_ROS
    STM32Hardware(UART_HandleTypeDef *huart_):
      huart(huart_), rind(0), twind(0), tfind(0){
    	rCurrentSize = 0;
    }
#endif

    void init(){

#ifndef USE_ETHERNET_FOR_ROS
        reset_rbuf();
#endif

    }

    void reset_rbuf(void){
#ifndef USE_ETHERNET_FOR_ROS

    	memset(tbuf, 0x00, tbuflen);
#ifdef USE_UART_FOR_ROS
        HAL_UART_Receive_DMA(huart, rbuf, rbuflen);
#endif
        rCurrentSize =0;
#endif
    }

    int read(){
#ifndef USE_ETHERNET_FOR_ROS
        int c = -1;
        if(rind != getRdmaInd()){
          c = rbuf[rind++];
          rind &= rbuflen - 1;
        }
        return c;
#else
        //return getRosInterface()->read();
        return getRosInterface()->getRxData();
#endif

    }

    void flush(void){

#ifndef USE_ETHERNET_FOR_ROS
      static bool mutex = false;

#ifdef USE_UART_FOR_ROS
      if((huart->gState == HAL_UART_STATE_READY) && !mutex){
#else
        if(!mutex){
#endif
           mutex = true;

        if(twind != tfind){
          uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
#ifdef USE_USB_FOR_ROS
          CDC_Transmit_FS(&(tbuf[tfind]), len);
#endif

#ifdef USE_UART_FOR_ROS
          HAL_UART_Transmit_DMA(huart, &(tbuf[tfind]), len);
#endif
          tfind = (tfind + len) & (tbuflen - 1);
        }
        mutex = false;
      }
#endif
    }

    void write(uint8_t* data, int length){

#ifdef USE_ETHERNET_FOR_ROS
    	//getRosInterface()->write(data, length);
    	if (!getRosInterface()->SendData(data, length) )
    	{
    		// FAiled to send data log it
    	}
#else
      uint32_t n = length;
      n = n <= tbuflen ? n : tbuflen;

      uint32_t n_tail = n <= tbuflen - twind ? n : tbuflen - twind;
      memcpy(&(tbuf[twind]), data, n_tail);
      twind = (twind + n) & (tbuflen - 1);

      if(n != n_tail){
        memcpy(tbuf, &(data[n_tail]), n - n_tail);
      }

      flush();
#endif
    }

    unsigned long time(){ return xTaskGetTickCount(); }

  protected:
};

#endif
