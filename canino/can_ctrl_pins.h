
#pragma once

#if defined(__AVR_ATmega2560__)

#define	P_MOSI	B,2
#define	P_MISO	B,3
#define	P_SCK	B,1

#define	MCP2515_CS			B,0
#define	MCP2515_INT			D,2
#define LED2_HIGH			B,7
#define LED2_LOW			B,7

#else

#define	P_MOSI	B,3
#define	P_MISO	B,4
#define	P_SCK	B,5

#define	MCP2515_CS			B,2
#define	MCP2515_INT			D,2
#define LED2_HIGH			B,0
#define LED2_LOW			B,0

#endif

