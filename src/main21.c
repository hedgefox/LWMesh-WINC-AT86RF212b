/**
 *
 * \file
 *
 * \brief WINC1500 UDP Server Example.
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

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAMD21 Xplained Pro
 * board to test UDP server socket.<br>
 * It uses the following hardware:
 * - the SAMD21 Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500 and test UDP server.
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID                    "DEMO_AP"
 *    #define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK                     "12345678"
 *    #define MAIN_WIFI_M2M_PRODUCT_NAME        "NMCTemp"
 *    #define MAIN_WIFI_M2M_SERVER_IP           0xFFFFFFFF // "255.255.255.255"
 *    #define MAIN_WIFI_M2M_SERVER_PORT         (6666)
 *    #define MAIN_WIFI_M2M_REPORT_INTERVAL     (1000)
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 * \endcode
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 * \code
 *    -- WINC1500 UDP server example --
 *    -- SAMD21_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED : CONNECTED
 *    wifi_cb: M2M_WIFI_REQ_DHCP_CONF : IP is xxx.xxx.xxx.xxx
 *    socket_cb: bind success!
 *    socket_cb: received app message.(1)
 *    . . .
 *    socket_cb: received app message.(10)
 *    UDP Server test Complete!
 * \endcode
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "main.h"
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

#include "sys.h"
#include "phy.h"
#include "nwk.h"
#include "sysTimer.h"
#include <sio2host.h>

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 UDP server example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

	struct sockaddr_in addr_rx,addr_tx;

bool connectiook = false;

bool issending = false;
bool isreceiving = false;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Test buffer */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Socket for Rx */
static SOCKET rx_socket = -1;

/** Socket for Tx */
static SOCKET tx_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** UDP packet count */
static uint8_t packetCnt = 0;



/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

static uint8_t rx_data[APP_RX_BUF_SIZE];

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataToWinc = false;
static bool appDataFromWinc =false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static uint8_t sio_rx_length;
/*- Implementations --------------------------------------------------------*/

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

typedef struct s_msg_wifi_product_main {
	uint8_t name[9];
} t_msg_wifi_product_main;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

static t_msg_wifi_product_main msg_wifi_product_main = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};
tstrM2MIPConfig ip_client;

/** 
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	if (u8Msg == SOCKET_MSG_BIND) 
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) 
		{
			/* Prepare next buffer reception. */
			printf("socket_cb: bind success!\r\n");
			recvfrom(sock, gau8SocketTestBuffer, MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			connectiook = true;

			uint32_t val = IP;
			printf("\r\nMy IP address = %d.%d.%d.%d\r\n",
			(val & 0xFF000000) >> 24,
			(val & 0x00FF0000) >> 16,
			(val & 0x0000FF00) >> 8,
			val & 0x000000FF);

			printf("\r\nUDP Target address = 0x%08x \r\n", MAIN_WIFI_M2M_SERVER_IP);
			
			printf("Ready to serve... \r\n");

			//memcpy(appDataReqBuffer, "ready to send", 13);
			//appUartBufferPtr = strlen(appDataReqBuffer);
			//appSendData();
		} else 
		{
			printf("socket_cb: bind error!\r\n");
		}
	} else if (u8Msg == SOCKET_MSG_RECVFROM) {
		tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
 		

		if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) 
		{
			packetCnt=0;

			if (!issending)
			{
				printf("Invio via radio \r\n");
				issending = true;
			}

			
			//printf("socket_cb: received app message - (%u) - %s\r\n", packetCnt, gau8SocketTestBuffer);
			//printf("\r\nWINC RX - %s\r\n", gau8SocketTestBuffer);

			if (gau8SocketTestBuffer[0] == 0x03)
			{
				printf("ETX received \r\n");
				issending = false;
			}

			/* Prepare next buffer reception. */
			recvfrom(sock, gau8SocketTestBuffer, MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			
			appUartBufferPtr = pstrRx->s16BufferSize;
			memcpy(appDataReqBuffer, gau8SocketTestBuffer, appUartBufferPtr);
			LED_On(LED0);
			
			appDataFromWinc=true;
			for (int i = 0; i <= pstrRx->s16BufferSize; i++)
			{
				gau8SocketTestBuffer[i] = '\0';
			}
			return;
			
		} 
		else 
		{
			if (pstrRx->s16BufferSize == SOCK_ERR_TIMEOUT) 
			{
				/* Prepare next buffer reception. */
				recvfrom(sock, gau8SocketTestBuffer, MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			}
		}
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			
//			m2m_wifi_request_dhcp_client();
 			ip_client.u32DNS=_htonl(DNS);
  			ip_client.u32Gateway=_htonl(GATEWAY);
  			ip_client.u32StaticIP=_htonl(IP);
  			ip_client.u32SubnetMask=_htonl(SUBNET);
	  		m2m_wifi_set_static_ip(&ip_client);
			 wifi_connected = M2M_WIFI_CONNECTED;
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = M2M_WIFI_CONNECTED;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF : IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
	}
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)
{
	//if (appDataReqBusy || 0 == appUartBufferPtr) 
	//{
		//return;
	//}
//


appDataReq.dstAddr = 1-APP_ADDR;
appDataReq.dstEndpoint = APP_ENDPOINT;
appDataReq.srcEndpoint = APP_ENDPOINT;
//appDataReq.options = NWK_OPT_ACK_REQUEST;
appDataReq.data = appDataReqBuffer;
appDataReq.size = appUartBufferPtr;
appDataReq.confirm = appDataConf;
NWK_DataReq(&appDataReq);
appDataFromWinc=false;
	//for(uint8_t i=0;i<appUartBufferPtr;i++)
	//appDataReqBuffer[i]='\0';
appUartBufferPtr=0;
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
	printf("TIMER");
	appSendData();
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	//printf("\r\nRADIO RX - ");
 	//for (uint8_t i = 0; i < ind->size; i++) 
 	//{
 		//printf("%c", ind->data[i]);
 		//appDataReqBuffer[i]=ind->data[i];
 	//}
 	//printf("\n");

	appUartBufferPtr=ind->size;
	memcpy(appDataReqBuffer,ind->data,ind->size);	
	appDataToWinc=true;

	if (!isreceiving)
	{
		printf("Receiving data... \r\n");
		isreceiving = true;
	}
	
	if (ind->data[0] == 0x03)
	{
		printf("End receiving... \r\n");
		isreceiving = false;
	}
	
	return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	#ifdef PHY_AT86RF212
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
	#endif
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);
}

static void APP_TaskHandler(void)
{
	switch (appState) {
		case APP_STATE_INITIAL:
		{
			//appInit();
			appState = APP_STATE_IDLE;
		}
		break;

		case APP_STATE_IDLE:
		break;

		default:
		break;
	}
	
	//sio_rx_length = sio2host_rx(rx_data, APP_RX_BUF_SIZE);
	//if (sio_rx_length) {
		//for (uint16_t i = 0; i < sio_rx_length; i++) {
			//sio2host_putchar(rx_data[i]);
			//if (appUartBufferPtr == sizeof(appUartBuffer)) {
				//appSendData();
			//}
//
			//if (appUartBufferPtr < sizeof(appUartBuffer)) {
				//appUartBuffer[appUartBufferPtr++] = rx_data[i];
			//}
		//}
//
//		SYS_TimerStop(&appTimer);
		// SYS_TimerStart(&appTimer);
	//}
}


/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start function of UDP socket.
 *
 * \return program return value.
 */

void lwmesh_init(void)
{
	SYS_Init();
	appInit();
}	

int main(void)
{
	
	irq_initialize_vectors();
	#if SAMD || SAMR21 || SAML21
		system_init();
		delay_init();
	#else
		sysclk_init();
		board_init();
	#endif

	cpu_irq_enable();
	LED_On(LED0);
		
	lwmesh_init();

	tstrWifiInitParam param;
	int8_t ret;

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize socket address structure. */
	addr_rx.sin_family = AF_INET;
	addr_rx.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT_RX);
	addr_rx.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);
	
	addr_tx.sin_family = AF_INET;
	addr_tx.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT_TX);
	addr_tx.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	 m2m_wifi_enable_dhcp(0);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	packetCnt = 0;
	connectiook = false;
	printf("%x \n TX: %d\n RX: %d\n",APP_ADDR,MAIN_WIFI_M2M_SERVER_PORT_TX,MAIN_WIFI_M2M_SERVER_PORT_RX);
	while (1) 
	{
		//printf("inizio while\n");
		//if (packetCnt >= MAIN_WIFI_M2M_PACKET_COUNT)
		//{
			//printf("UDP Server test Complete!\r\n");
			//close(rx_socket);
			//rx_socket = -1;
			//break;
		//}

		m2m_wifi_handle_events(NULL);

		if (wifi_connected == M2M_WIFI_CONNECTED) 
		{
			/* Create socket for Rx UDP */

			if (rx_socket < 0) 
			{
				if ((rx_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
				{
					printf("main : failed to create RX UDP Client socket error!\r\n");
					continue;
				}

				/* Socket bind */
				bind(rx_socket, (struct sockaddr *)&addr_rx, sizeof(struct sockaddr_in));
			}




			SYS_TaskHandler();
	if(appDataFromWinc==true)
		appSendData();
		
	if(appDataToWinc == true)
	{
			if (tx_socket < 0)
		{
			if ((tx_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				printf("main : failed to create TX UDP client socket error!\r\n");
				return false;
			}
		}

		int8_t ret;

		/* Send client discovery frame. */
//		sendto(tx_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0, (struct sockaddr *)&addr, sizeof(addr));

		//ret = sendto(tx_socket, &msg_wifi_product_main, sizeof(t_msg_wifi_product_main), 0, (struct sockaddr *)&addr, sizeof(addr));
		ret = sendto(tx_socket,appDataReqBuffer, appUartBufferPtr, 0, (struct sockaddr *)&addr_tx, sizeof(addr_tx));
		for(uint8_t i=0;i<appUartBufferPtr;i++)
			appDataReqBuffer[i]='\0';
		appUartBufferPtr=0;
		if (ret == M2M_SUCCESS)
		{
	//		printf("main: message sent\r\n");
			//packetCnt += 1;
			//if (packetCnt == MAIN_WIFI_M2M_PACKET_COUNT)
			//{
				//printf("UDP Client test Complete!\r\n");
			//}
			appDataToWinc = false;

			LED_Toggle(LED0);
		}
		else
		{
			printf("main: failed to send status report error!\r\n");
		}
		
		//sio2host_putchar(ind->data[i]);
	

		
	}
		}
	}
	return 0;
}
