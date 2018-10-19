/* MQTT based house control
 * Copyright (C) Horion Dreher <horiondreher@gmail.com>
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

#include "config.h"
#include "enc28j60/network.h"
#include "uip/uip.h"
#include "uip/uiparp.h"
#include "uip/timer.h"
#include "nethandler/nethandler.h"
#include "node/node.h"
#include "umqtt/mqttclient.h"
#include "spi/spi.h"
#include "mfrc522/mfrc522.h"
#include "convertion/convertion.h"

int sub_flag = 0;

//RFID Variables
uint8_t SelfTestBuffer[64];
char rfid_data[17];
int data_count=0;
int ovf=0;

char horion_rfid[17] = "C58D2583EEAAFF47";
char bruno_rfid[17] = "07D4258375EAFD67";

//ADC Variables
float temperature;
char s[5];
#define ADC_VREF_TYPE ((1<<REFS1)|(1<<REFS0)|(0<<ADLAR))

//Mqtt Variables
static struct timer periodic_timer;
static struct timer arp_timer;

/* Prototypes */
static void _interface_init(void);
#if !(CONFIG_DHCP)
	static void _ip_init();
#endif

void HexDump(uint8_t d);
unsigned int adc_converter(unsigned char adc_input);

//Temperature interrupt
ISR(TIMER0_OVF_vect){
	unsigned int valor;
	TCNT0 = 100; //Check temperature every 5 seconds
	ovf++;
	
	if(ovf==500){
		ovf=0;
		valor = adc_converter(1);
		temperature = valor/9.3;
		ftoa(temperature, s, 1);
		_mqttclient_send_data("horion/temp", s);	
	}
}

void tcpip_output(void) {
}

int main(void){
	
	uint8_t byte;
	uint8_t str[MAX_LEN];	
	_delay_ms(50);
	
	//Modules start
	spi_init();
	clock_init();
	network_init();
	uip_init();
	node_init();
	_interface_init();
	mfrc522_init();
	
	//AD Converter
 	ADMUX = 0b11000000;				 //1.1 V - Voltage ref.
 	ADCSRA = 0b10000100;
	TCCR0B |= (1<<CS02) | (1<<CS00); //Prescaler 1024
	TIMSK0 |= (1<<TOIE0);			 //Time Overflow Interrupt Enable
	
	set_bit(PORTD,4);
	_delay_ms(500);
	
#if !(CONFIG_DHCP)
	_ip_init();
#endif
	
	sei();
	
	//Cooler and lamps
	DDRC |= (_BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5));	//saídas
	
	//Gate control pins
	DDRD |= (_BV(0) | _BV(1));
	
	//Rfid Leds
	DDRD |= (_BV(3) | _BV(4));
	set_bit(DDRD, 4);
	
    while (1){
		
		/*Broker connection*/	
		nethandler_rx();
		
		if (timer_tryrestart(&periodic_timer))
		nethandler_periodic();

		if (timer_tryrestart(&arp_timer))
		uip_arp_timer();

		node_process();
		
		/*******************************/
		
		/*Topics Subscribe*/
		if(_mqttclient_is_connected() == 1 && sub_flag == 0){
			_mqttclient_subscribe(LMP1_TOPIC);
			_mqttclient_subscribe(LMP2_TOPIC);
			_mqttclient_subscribe(LMP3_TOPIC);
			_mqttclient_subscribe(LMP4_TOPIC);
			_mqttclient_subscribe(LMP5_TOPIC);
			_mqttclient_subscribe(PORTAO_TOPIC);
			_mqttclient_subscribe(AUT_TOPIC);
			sub_flag = 1;
		}else if (_mqttclient_is_connected() == 0){
			sub_flag = 0;
		}
		
		/*RFID verification */
		byte = mfrc522_request(PICC_REQALL,str);
		
		if(byte == CARD_FOUND){
			
			byte = mfrc522_get_card_serial(str);

			if(byte == CARD_FOUND){
				
				for(byte=0;byte<8;byte++)
				HexDump(str[byte]);
				
				data_count=0;
				rfid_data[16] = '\0';
				
				
				_mqttclient_send_data("horion/autenticacao", rfid_data);
				_delay_ms(1000);
			}
			else{
				_mqttclient_send_data("horion/autenticacao", "Erro");
			}
		}
		
		//Turn the cooler on if temperature>=22
		if (temperature >= 22)
			set_bit(PORTC,1);		
			
    }
	return 0;
}

unsigned int adc_converter(unsigned char adc_input){	
	
	ADMUX= adc_input | ADC_VREF_TYPE;
	_delay_us(10); // Atraso p/ estabilização da tensão em Vin
	ADCSRA|=(1<<ADSC); // Sinal de Start Conversion (ADSC) ativado
	while ((ADCSRA&(1<<ADIF))==0); // Espera final de conversão
	ADCSRA|=(1<<ADIF); // Reseta flag de interrupção
		
	return ADCW;
}

//Function to assist RFID string conversion
void HexDump(uint8_t d){
	
	uint8_t byte_dump = '0';
		
	(((d>>4)&0x0F)<=9) ? (byte_dump='0'+((d>>4)&0x0F)) : (byte_dump='A'+ ((d>>4)&0x0F)-0x0A);
	rfid_data[data_count++] = byte_dump;
	((d&0x0F)<=9) ? (byte_dump='0'+ (d&0x0F)) : (byte_dump='A'+ (d&0x0F)-0x0A);
	rfid_data[data_count++] = byte_dump;
	
}


static void _interface_init(void) {
	struct uip_eth_addr mac;

	mac.addr[0] = ETH_ADDR0;
	mac.addr[1] = ETH_ADDR1;
	mac.addr[2] = ETH_ADDR2;
	mac.addr[3] = ETH_ADDR3;
	mac.addr[4] = ETH_ADDR4;
	mac.addr[5] = ETH_ADDR5;

	uip_setethaddr(mac);

	timer_set(&periodic_timer, CLOCK_SECOND / 2);
	timer_set(&arp_timer, CLOCK_SECOND * 10);
}

#if !(CONFIG_DHCP)
static void _ip_init() {
	uip_ipaddr_t address;
	uip_ipaddr_t netmask;

	uip_ipaddr(&address, CONFIG_IP_ADDR0, CONFIG_IP_ADDR1, CONFIG_IP_ADDR2, CONFIG_IP_ADDR3);
	uip_ipaddr(&netmask, CONFIG_NETMASK0, CONFIG_NETMASK1, CONFIG_NETMASK2, CONFIG_NETMASK3);

	uip_sethostaddr(&address);
	uip_setnetmask(&netmask);
}
#endif

