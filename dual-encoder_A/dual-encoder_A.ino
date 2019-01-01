/*
* dual-encoder_A.ino
*
* Created: 23.12.2017
* Author: Lukas
*
* Based on "Efficiently Reading Quadrature With Interrupts"
* http://makeatronics.blogspot.de/2013/02/efficiently-reading-quadrature-with.html
*
*/

#include "Arduino.h"
#include "Wire.h"

#define SLAVE_ADDRESS 0x08

//volatile boolean enc_M1_interrupt = false;
//volatile boolean enc_M2_interrupt = false;

volatile int16_t enc_count_M1 = 0;	// motor 1 encoder count
volatile int16_t enc_count_M2 = 0;	// motor 2 encoder count

// stores which combination of current and previous encoder state lead to an increase or decrease of the encoder count and which are not defined
const int8_t lookup_table[] = {0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1, 0, 0, 0, 0, 0};

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

void requestEvent() {
	static byte message[6];
	
	message[0] = (enc_count_M1 >> 8) & 0xFF;	// most significant bits for motor 1 encoder count
	message[1] = enc_count_M1 & 0xFF;			// least significant bits for motor 1 encoder count
	message[2] = (enc_count_M2 >> 8) & 0xFF;	// most significant bits for motor 2 encoder count
	message[3] = enc_count_M2 & 0xFF;			// least significant bits for motor 2 encoder count

	Wire.write(message, 4);

	// reset encoder count
	enc_count_M1 = 0;
	enc_count_M2 = 0;
}

void enc_isr_M1() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b10100) >> 2);
	
	// update encoder count according to the encoder state
	enc_count_M1 = enc_count_M1 + lookup_table[enc_state & 0b1111];
	
	//enc_M1_interrupt = true;
}

void enc_isr_M2() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b101000) >> 3);
	
	// update encoder count according to the encoder state
	enc_count_M2 = enc_count_M2 - lookup_table[enc_state & 0b1111];
	
	//enc_M2_interrupt = true;
}

void setup() {
	// join i2c bus
	Wire.begin(SLAVE_ADDRESS);
	// register event
	Wire.onRequest(requestEvent);

	#ifdef DEBUG
	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo eNUMeration, others continue immediately
	#endif
	
	// prepare encoder pins
	pinMode(2, INPUT);	// interrupt pin for encoder M1
	pinMode(3, INPUT);	// interrupt pin for encoder M2
	
	attachInterrupt(digitalPinToInterrupt(2), enc_isr_M1, CHANGE);	// attach interrupt service routine enc_isr_M1 to encoder M1 interrupt pin
	attachInterrupt(digitalPinToInterrupt(3), enc_isr_M2, CHANGE);	// attach interrupt service routine enc_isr_M2 to encoder M2 interrupt pin
}

void loop() {
	/*if (enc_M1_interrupt) {
		DEBUG_PRINTLN(enc_count_M1 % 1000);
		enc_M1_interrupt = false;
	}
	
	if (enc_M2_interrupt) {
		DEBUG_PRINT("\t"); DEBUG_PRINTLN(enc_count_M2 % 1000);
		enc_M2_interrupt = false;
	}*/
}