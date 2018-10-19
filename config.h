/* MQTT based house control
 * Copyright (C) 2018 Horion Dreher <horiondreher@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define set_bit(y,bit) (y|=(1<<bit))
#define clr_bit(y,bit) (y&=~(1<<bit))
#define cpl_bit(y,bit) (y^=(1<<bit))
#define tst_bit(y,bit) (y&(1<<bit))

#define F_CPU 16000000UL 

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "node/node.h"
#include "common/common.h"

#define CONFIG_DEBUG    1

#define ETH_ADDR0       0x76
#define ETH_ADDR1       0xe6
#define ETH_ADDR2       0xe2
#define ETH_ADDR3       0x18
#define ETH_ADDR4       0x3f
#define ETH_ADDR5       0x44

/* IP configuration. */
#define CONFIG_DHCP     0
#if !(CONFIG_DHCP)
#define CONFIG_IP_ADDR0 192
#define CONFIG_IP_ADDR1 168
#define CONFIG_IP_ADDR2 15
#define CONFIG_IP_ADDR3 5

#define CONFIG_NETMASK0 255
#define CONFIG_NETMASK1 255
#define CONFIG_NETMASK2 255
#define CONFIG_NETMASK3 0
#endif

/* SPI interface configuration. */
#define SPI_CONFIG_AS_MASTER 	1
#define SPI_DDR		DDRB
#define SPI_PORT	PORTB
#define SPI_PIN		PINB
#define SPI_MOSI	PB3
#define SPI_MISO	PB4
#define SPI_SCK		PB5

#define ENC28J60_SPI_SS         PB2
#define RFID_SPI_SS				PB1

/* ENC28J60 interface configuration */
#define ENC28J60_CONTROL_PORT   PORTB
#define ENC28J60_CONTROL_DDR    DDRB
#define ENC28J60_CONTROL_CS     PB2

/* Signal LED configuration */
#define CONFIG_SIGNAL_LED_PIN       PD2
#define CONFIG_SIGNAL_LED_DDR       DDRD
#define CONFIG_SIGNAL_LED_PORT      PORTD
#define CONFIG_SIGNAL_LED_INTERVAL  0.5

/* MQTT configuration. */
#define MQTT_BROKER_IP_ADDR0    192
#define MQTT_BROKER_IP_ADDR1    168
#define MQTT_BROKER_IP_ADDR2    15
#define MQTT_BROKER_IP_ADDR3    2

#define MQTT_BROKER_PORT        1883

#define MQTT_PUBLISH_PERIOD     2

#define MQTT_KEEP_ALIVE         30
#define _MQTT_CLIENT_ID         horion
#define MQTT_CLIENT_ID          "" STR(_MQTT_CLIENT_ID) ""

/* Home Configuration - Projeto Integrador */

#define LMP1_PIN				PC5
#define LMP2_PIN				PC4
#define LMP3_PIN				PC3
#define LMP4_PIN				PC2

#define LMP1_TOPIC				"horion/lmp1"
#define LMP2_TOPIC				"horion/lmp2"
#define LMP3_TOPIC				"horion/lmp3"
#define LMP4_TOPIC				"horion/lmp4"
#define LMP5_TOPIC				"horion/lmp5"
#define PORTAO_TOPIC			"horion/portao"

#define AUT_TOPIC				"horion/autenticacao"

#define LMP_ON					"on"
#define LMP_OFF					"off"	
#define GATE_OPEN				"gopen"
#define GATE_CLOSE				"gclose"		

/* MQTT node presence */
#define MQTT_NODE_PRESENCE              1
#define MQTT_NODE_PRESENCE_TOPIC        "presence/" STR(_MQTT_CLIENT_ID)
#define MQTT_NODE_PRESENCE_MSG_ONLINE   "online"
#define MQTT_NODE_PRESENCE_MSG_OFFLINE  "offline"

#define TESTE "teste"

/* TCP connections. */
typedef struct node_appstate uip_tcp_appstate_t;
#define UIP_APPCALL node_appcall

/* UDP connections. */
typedef struct node_udp_appstate uip_udp_appstate_t;
#define UIP_UDP_APPCALL node_udp_appcall

#endif
