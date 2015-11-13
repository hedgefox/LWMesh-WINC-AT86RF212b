/**
 * \file
 *
 * \brief MAIN configuration.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID             "MY_SSID_HERE" //       /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                 "MY_PASSWORD_HERE"  //  /**< Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME        "WINC_UDP_FILE_TRANSFER"
#define MAIN_WIFI_M2M_SERVER_IP          0xC0A80066 // 192.168.0.102  --- Target IP of the receiver application on PC

#define IP           0xC0A8006E  /* 192.168.0.110 */  // IP Address of the first SAMD21 board
//#define IP           0xC0A8006F  /* 192.168.0.111 */  // IP Address of the second SAMD21 board

#define GATEWAY      0xC0A80001  // 0xAC1E01FE   /* 192.168.0.1 */

#define SUBNET       0xFFFFF00 /* 255.255.255.0 */
#define DNS				GATEWAY

#define MAIN_WIFI_M2M_SERVER_PORT_TX       (6000) 
#define MAIN_WIFI_M2M_SERVER_PORT_RX       (6001)
#define MAIN_WIFI_M2M_REPORT_INTERVAL     (1000)

#define MAIN_WIFI_M2M_BUFFER_SIZE         1460

/** UDP MAX packet count */
#define MAIN_WIFI_M2M_PACKET_COUNT        10

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
