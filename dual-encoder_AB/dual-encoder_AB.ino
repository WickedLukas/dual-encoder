/*
* dual-encoder_AB.ino
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

//volatile boolean M1_interrupt = false;
//volatile boolean M2_interrupt = false;

volatile int8_t enc_count_M1 = 0;
volatile int8_t enc_count_M2 = 0;

// stores which combination of current and previous encoder state lead to an increase or decrease of the encoder count and which are not defined
const int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void requestEvent() {
	Wire.write(enc_count_M1);
	Wire.write(enc_count_M2);
	
	enc_count_M1 = 0;
	enc_count_M2 = 0;
}

void encoder_isr_M1() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 2;
	// add the current encoder state (PIND stores the values of digital pins 0-7) 
	enc_state = enc_state | (PIND & 0b0011);
	
	// change encounter count according to encoder state
	enc_count_M1 = enc_count_M1 + lookup_table[enc_state & 0b1111];
	
	//M1_interrupt = true;
}

void encoder_isr_M2() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 2;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = enc_state | ((PIND & 0b1100) >> 2);
	
	// change encounter count according to encoder state
	enc_count_M2 = enc_count_M2 + lookup_table[enc_state & 0b1111];
	
	//M2_interrupt = true;
}

void setup() {
	// join i2c bus
	Wire.begin(SLAVE_ADDRESS);
	// register event
	Wire.onRequest(requestEvent);
	
	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo eNUMeration, others continue immediately

	attachInterrupt(digitalPinToInterrupt(0),encoder_isr_M1,CHANGE);
	attachInterrupt(digitalPinToInterrupt(1),encoder_isr_M1,CHANGE);
	attachInterrupt(digitalPinToInterrupt(2),encoder_isr_M2,CHANGE);
	attachInterrupt(digitalPinToInterrupt(3),encoder_isr_M2,CHANGE);

	digitalWrite(digitalPinToInterrupt(0), HIGH);
	digitalWrite(digitalPinToInterrupt(1), HIGH);
	digitalWrite(digitalPinToInterrupt(2), HIGH);
	digitalWrite(digitalPinToInterrupt(3), HIGH);
}

void loop() {
	/*if (M1_interrupt) {
		Serial.println(enc_count_M1 % 1000);
		M1_interrupt = false;
	}
	
	if (M2_interrupt) {
		Serial.print("\t"); Serial.println(enc_count_M2 % 1000);
		M2_interrupt = false;
	}*/
}
