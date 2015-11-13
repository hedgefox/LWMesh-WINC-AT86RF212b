# LWMesh-WINC-AT86RF212b
UDP File Transfer between 2 SAM D21 XPLAINED PRO with a WINC 1500 on EXT1 and an AT86RF212b on EXT 2

To configuring the boards, we need to modify a few code lines.

Things to modify in main.h

#define MAIN_WLAN_SSID             "MY_SSID_HERE" //       /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                 "MY_PASSWORD_HERE"  //  /**< Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME        "WINC_UDP_FILE_TRANSFER"
#define MAIN_WIFI_M2M_SERVER_IP          0xC0A80066 // 192.168.0.102  --- Target IP of the receiver application on PC

#define IP           0xC0A8006E  /* 192.168.0.110 */  // IP Address of the SAMD21 board

#define MAIN_WIFI_M2M_SERVER_PORT_TX       (6000)
#define MAIN_WIFI_M2M_SERVER_PORT_RX       (6001)

Things to modify in config.h

#define APP_ADDR                0x0000        //  APP Address of this board - use 0x0000 and 0x0001 on a second board to test
