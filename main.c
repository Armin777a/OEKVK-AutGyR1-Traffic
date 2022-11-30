/*
 * Traffic.c
 *
 * Created: 29-Oct-22 18:38:55
 * Author : Armin
 */ 



//////////////////////////////////////////////////////////////////////////
// Include directories
//////////////////////////////////////////////////////////////////////////

#include <avr/io.h>						// AVR device-specific IO definitions
#include <avr/interrupt.h>				// AVR device-specific timer / interrupt definitions





//////////////////////////////////////////////////////////////////////////
// Changeable parameters, constants, macros
//////////////////////////////////////////////////////////////////////////

#define ENABLE_DEBUG_MODE				1								// Enable's the debug mode (1 - on, 0 - off)

#define CURRENT_MICROCONTROLLER			ATMEGA64						// The current microcontroller (ATMEGA128 and ATMEGA64 are supported becouse of clock speed)


// Set the delays
#define DELAY_All_Red_Start				20								// Timing of the starting state of the modes, where all the light are red (in x*100ms)
#define DELAY_Red						50								// Timing of the main red-green light state (in x*100ms)
#define DELAY_Red_Yellow				10								// Timing of the red-yellow transition light state (in x*100ms)
#define DELAY_Pre_Green_Blink			20								// Timing of the end of the green state where the pedestrian green light starts blinking (in x*100ms)
#define DELAY_Yellow					20								// Timing of the yellow light and blinking pedestrian light (in x*100ms)
#define DELAY_Turn						30								// Timing for the turning lanes (in x*100ms)
#define DELAY_Blink						5								// Dealy between blinks (in x*100ms)
#define DELAY_All_Red					5
#define DELAY_Only_Green				DELAY_Red - DELAY_Pre_Green_Blink	// Calculated value for the state where the pedestrian lights are not yet blinking


// Set the start address of the program
#define PROGRAM_ServiceMode				50								// Service mode's address in the master table
#define PROGRAM_Normal					8								// Normal mode's address in the master table
#define PROGRAM_Normal_All_Red			7								// Normal mode's all red state's address in the master table
#define PROGRAM_Train_All_Red			25								// Train mode's all red state's address in the master table
#define PROGRAM_Train					35								// Train mode's address in the master table
#define PROGRAM_Volatile				5								// Volatile mode's address in the master table



//////////////////////////////////////////////////////////////////////////
// The Master Table
//////////////////////////////////////////////////////////////////////////

// Every state, port and pin is in here
// For debugging on the 8 LEDS
	// LED 7 - Pedestrian light green
	// LED 6 - Pedestrian light red
	// LED 5 - Train light white
	// LED 4 - Train light red1
	// LED 3 - Train light red 2
	// LED 2 - Traffic light green
	// LED 1 - Traffic light yellow
	// LED 0 - Traffic light red
/*
uint8_t MasterTable[21][11] = {
	// Data Header
	{7,	4,											2, 2, 2, 0,		0, 4, 4, 4,		255},		// 0 - PORT				+ Service Button On
	{7,	3,											4, 5, 6, 0,		0, 6, 7, 7,		255},		// 1 - PIN				+ Service Button Off
	{0,	0,											1, 0, 0, 0,		2, 0, 0, 0,		255},		// 2 - TYPE / DELAY
	
	// Train ports and pins
	{4, 5,		4, 4,		2, 7,		7, 2,		7, 1,		255},							// 3 - TRAIN
	
	// In case of overflow
	{PROGRAM_ServiceMode,	0,							0, 0, 0, 0,		0, 0, 0, 0,		255},	// 4 - BLANK
	
	// Free to use line
	{PROGRAM_ServiceMode,	DELAY_Yellow,				0, 0, 0, 0,		0, 0, 0, 0,		255},	// 5 - VOLATILE
	
	// In case of overflow
	{PROGRAM_ServiceMode,	0,							0, 0, 0, 0,		0, 0, 0, 0,		255},	// 6 - BLANK
	
	// Normal program
	{0,						DELAY_Red,					1, 0, 0, 0,		0, 1, 0, 0,		255},	// 7
	{0,						DELAY_Red_Yellow,			1, 1, 0, 0,		0, 1, 0, 0,		255},	// 8
	{0,						DELAY_Only_Green,			0, 0, 1, 0,		0, 0, 1, 0,		255},	// 9
	{0,						DELAY_Pre_Green_Blink,		0, 0, 1, 0,		0, 0, 0, 1,		255},	// 10
	{PROGRAM_Normal,		DELAY_Yellow,				0, 1, 0, 0,		0, 0, 0, 1,		255},	// 11
	
	// In case of overflow
	{PROGRAM_ServiceMode,	1,							0, 0, 0, 0,		0, 0, 0, 0,		255},	// 12 - BLANK
	
	// Service mode
	{0,						DELAY_Blink,				0, 1, 0, 0,		0, 0, 0, 0,		255},	// 13 - Yellow blinking (On)
	{PROGRAM_ServiceMode,	DELAY_Blink,				0, 0, 0, 0,		0, 0, 0, 0,		255},	// 14 - Yellow blinking (Off)
	
	// In case of overflow
	{PROGRAM_ServiceMode,	1,							0, 0, 0, 0,		0, 0, 0, 0,		255},	// 15 - BLANK
	
	// Train routine
	{0,						DELAY_All_Red,				1, 0, 0, 0,		0, 1, 0, 0,		255},	// 16 - All light red
	{0,						DELAY_Red,					1, 0, 0, 0,		0, 0, 1, 0,		255},	// 17 - Normal train routine
	{0,						DELAY_Yellow,				1, 0, 0, 0,		0, 0, 0, 1,		255},	// 18
	{PROGRAM_Train,			DELAY_Red,					1, 0, 0, 0,		0, 1, 0, 0,		255},	// 19
	
	// In case of overflow
	{PROGRAM_ServiceMode,	1,							0, 0, 0, 0,		0, 0, 0, 0,		255}	// 20 - BLANK
};
*/



// Master table for the real traffic modell
uint8_t MasterTable[53][31] = {
	// Data Header
	{7,	4,											0, 5, 5, 5,		0, 5, 5, 5,		1, 1, 1, 0,		1, 1, 1, 1,		3, 3, 3, 0,		1, 3, 3, 0,		3, 3, 3, 6,		255},	// 0 - PORT				+ Service Button On
	{7,	3,											0, 2, 3, 3,		0, 6, 7, 7,		0, 1, 2, 0,		3, 4, 5, 6,		2, 3, 4, 0,		7, 0, 1, 0,		5, 6, 7, 0,		255},	// 1 - PIN				+ Service Button Off
	{0,	0,											2, 0, 0, 0,		2, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 2 - TYPE / DELAY
	
	// Train ports and pins
	{6, 3,		6, 1,		6, 2,		5, 4,		5, 5,		255},																										// 3 - TRAIN
	
	// In case of overflow
	{PROGRAM_ServiceMode,	0,						0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 4 - BLANK

	// Free to use line
	{PROGRAM_ServiceMode,	DELAY_Yellow,			0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 5 - VOLATILE

	// In case of overflow
	{PROGRAM_ServiceMode,	0,						0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 6 - BLANK

	// Normal mode All red
	{0,						DELAY_All_Red_Start,	0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 7 - NORMAL ALL RED
	// Normal mode
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 8 - NORMAL PROGRAM
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		255},	// 9
	{0,						DELAY_Only_Green,		0, 0, 1, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 0, 1, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 10
	{0,						DELAY_Pre_Green_Blink,	0, 0, 0, 1,		0, 1, 0, 0,		1, 0, 0, 0,		0, 0, 1, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 11
	{0,						DELAY_Turn,				0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 0, 1, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 12
	{0,						DELAY_Yellow,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		255},	// 13
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 14
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		255},	// 15
	{0,						DELAY_Only_Green,		0, 1, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		0, 0, 1, 1,		255},	// 16
	{0,						DELAY_Pre_Green_Blink,	0, 1, 0, 0,		0, 0, 0, 1,		1, 0, 0, 0,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		0, 0, 1, 1,		255},	// 17
	{0,						DELAY_Turn,				0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		0, 0, 1, 1,		255},	// 18
	{0,						DELAY_Yellow,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		255},	// 19
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 20
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 21
	{0,						DELAY_Only_Green,		0, 1, 0, 0,		0, 1, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 22
	{PROGRAM_Normal,		DELAY_Yellow,			0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 23

	// In case of overflow
	{PROGRAM_ServiceMode,	0,						0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 24 - BLANK
		
	// Train mode All red
	{0,						DELAY_All_Red_Start,	0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 25 - TRAIN ALL RED
	// Train mode - release cars near the track
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 26 - TRAIN CLEAR TRAFFIC
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		255},	// 27
	{0,						DELAY_Only_Green,		0, 0, 1, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 28
	{0,						DELAY_Pre_Green_Blink,	0, 0, 0, 1,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 29
	{0,						DELAY_Yellow,			0, 0, 0, 1,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		255},	// 30
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 31
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 32
	{0,						DELAY_Turn,				0, 1, 0, 0,		0, 1, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 33
	{0,						DELAY_Yellow,			0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 34
	// Train mode program
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 35 - TRAIN PROGRAM
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		255},	// 36
	{0,						DELAY_Only_Green,		0, 1, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 1, 0,		255},	// 37
	{0,						DELAY_Pre_Green_Blink,	0, 1, 0, 0,		0, 0, 0, 1,		1, 0, 0, 0,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 1, 0,		255},	// 38
	{0,						DELAY_Yellow,			0, 1, 0, 0,		0, 0, 0, 1,		1, 0, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		255},	// 39
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 40
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		255},	// 41
	{0,						DELAY_Only_Green,		0, 0, 1, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 42
	{0,						DELAY_Pre_Green_Blink,	0, 0, 0, 1,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 1,		1, 0, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		255},	// 43
	{0,						DELAY_Yellow,			0, 0, 0, 1,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		255},	// 44
	{0,						DELAY_All_Red,			0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 45
	{0,						DELAY_Red_Yellow,		0, 1, 0, 0,		0, 1, 0, 0,		1, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 46
	{0,						DELAY_Turn,				0, 1, 0, 0,		0, 1, 0, 0,		0, 0, 1, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 47
	{PROGRAM_Train,			DELAY_Yellow,			0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		1, 0, 0, 0,		255},	// 48

	// In case of overflow
	{PROGRAM_ServiceMode,	0,						0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 49 - BLANK

	// Service mode
	{0,						DELAY_Blink,			0, 0, 0, 0,		0, 0, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		0, 1, 0, 0,		255},	// 50 - SERVICE MODE
	{PROGRAM_ServiceMode,	DELAY_Blink,			0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 51


	// In case of overflow
	{PROGRAM_ServiceMode,	0,						0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		0, 0, 0, 0,		255},	// 52 - BLANK
};




//////////////////////////////////////////////////////////////////////////
// Mostly static constants/macros
//////////////////////////////////////////////////////////////////////////

#define ATMEGA128	16													// ATmega128's clock speed in MHz
#define ATMEGA64	8													// ATmega64's clock speed in MHz

#define DATA_Traffic_Light_StartID		2								// The first light's ID (column)
#define DATA_Traffic_Light_NextID		4								// The next light's offset from the current one
#define DATA_Port_Line					0								// The line (row) where the lights ports are located
#define DATA_Pin_Line					1								// The line (row) where the lights pins are located
#define DATA_Type_Line					2								// The line (row) where the lights types are located
#define DATA_Lenght						DATA_Traffic_Light_NextID		// Alternate name for the "DATA_Traffic_Light_NextID" when using a for loop
#define MASTERTABLE_NextState			0								// The column where the next states are located
#define MASTERTABLE_Delay				1								// The column where the delays are located (in x*100ms)

#define DATA_Traffic_Red_Offset			0								// Car's red light offset from the light's ID
#define DATA_Traffic_Yellow_Offset		1								// Car's yellow light offset from the light's ID
#define DATA_Traffic_Green_Offset		2								// Car's green light offset from the light's ID
#define DATA_Traffic_Green2_Offset		3								// Car's auxiliary green light offset from the light's ID

#define DATA_Pedestrian_Red_Offset		1								// Pedestrian's red light offset from the light's ID
#define DATA_Pedestrian_Green_Offset	2								// Pedestrian's green light offset from the light's ID
#define DATA_Pedestrian_Blink_Offset	3								// Pedestrian's blinking light offset from the light's ID

#define DATA_Train						3								// Train lights and buttons line (row)
#define LIGHT_Train_White				0								// Train light white port's location in the row
#define LIGHT_Train_Red1				2								// Train light red1 port's location in the row
#define LIGHT_Train_Red2				4								// Train light red2 port's location in the row
#define TRAIN_First_Light				LIGHT_Train_White				// Alternate name for the "LIGHT_Train_White" when using with an offset
#define TRAIN_Light_Count				3								// How many train lights are there
#define TRAIN_Pin_Offset				1								// Pin's offset from the port (They are in a row)
#define DATA_Train_Button_Left_Port		6								// Train button left port's location in the row
#define DATA_Train_Button_Right_Port	8								// Train button right port's location in the row

#define DATA_Service_Button_On			0								// Service button on's line (row)
#define DATA_Service_Button_Off			1								// Service button off's line (row)
#define DATA_Service_Button_Port		0								// Service buttons port's column
#define DATA_Service_Button_Pin			1								// Service buttons pin's column

#define TRAINMANAGER_Bit_Tick			0								// Train manager's bit where the alternating value can be found
#define TRAINMANAGER_Bit_Left			1								// Train manager's bit where the button left's saved value can be found
#define TRAINMANAGER_Bit_Right			2								// Train manager's bit where the button right's saved value can be found
#define TRAINMANAGER_Bit_Is_Train		3								// Train manager's bit where the trains state can be fond (there is or there is no train)

#define PRESSED_Train_Button_Left		1								// Train button left's ID, used to make a switch-case statement more readable
#define PRESSED_Train_Button_Right		2								// Train button right's ID, used to make a switch-case statement more readable
#define PRESSED_Train_Button_None		0								// No train button pressed ID, used to make a switch-case statement more readable

#define PRESSED_Service_Button_On		1								// Service mode button on's ID, used to make a switch-case statement more readable
#define PRESSED_Service_Button_Off		2								// Service mode button off's ID, used to make a switch-case statement more readable
#define PRESSED_Service_Button_None		0								// No service mode button pressed ID, used to make a switch-case statement more readable

#define STATE_OFF						0								// The off state's ID
#define STATE_ON						1								// The on state's ID
#define STATE_BLINK						2								// The blink state's ID

#define	TYPE_CarLight					1								// The car light type's ID
#define TYPE_PedestrianLight			2								// The pedestrian light type's ID

#define HOLD_Train_Button				0								// The button manager's bit where it's saved if the train button is held down
#define HOLD_Service_Button				1								// The button manager's bit where it's saved if the service mode button is held down

#define DDR_Traffic_Light				1								// Traffic light's ddr ID, used to make a switch-case statement more readable
#define DDR_Train_Light					2								// Traibn light's ddr ID, used to make a switch-case statement more readable
#define DDR_Train_Button				3								// Train button's ddr ID, used to make a switch-case statement more readable
#define DDR_Service_Button				4								// Service mode button's ddr ID, used to make a switch-case statement more readable










//////////////////////////////////////////////////////////////////////////
// Function declarations
//////////////////////////////////////////////////////////////////////////

// Initialization
void TimerInitialization(uint8_t microcontroller);			// Timer initialization based on the clock frequency

void PortInitialization();									// Initializes all the pins from the master table
void SetDDR(uint8_t, uint8_t);								// Choses the correct setddr function
void SetDDR_TrafficLight(uint8_t);							// Sets the traffic light pins as outputs
void SetDDR_TrainLight(uint8_t);							// Sets the train light pins as outputs
void SetDDR_TrainButton(uint8_t);							// Sets the train button pins as inputs, end turns on the pull-up resistor
void SetDDR_ServiceButton(uint8_t);							// Sets the service mode button pins as inputs, end turns on the pull-up resistor



// Debugging
void DebugMode();											// Prints out the master iterator to the leds
void LED_port_init();										// LED port initialization
void LED_out(uint8_t led);									// LED controller



// Mode controllers
void	Start_NewProgram(uint8_t);							// Starts the program based on the input address
void	Start_ServiceMode();								// Starts the service mode (car lights blinking yellow)
void	Stop_ServiceMode();									// Stops the service mode and returns to the correct mode



// Traffic lights
void	Controller_TrafficLight();							// Controlls the car and pedestrian lights
void	Controller_TrafficCarLight();						// Sets the car light state from the master table
void	Controller_TrafficPedestrianLight();				// Sets the pedestrian light state from the master table

void	Set_TrafficLight_MasterTable(uint8_t);				// Sets the traffic light's state based on the mastertable's value
void	Set_State_Traffic(uint8_t, uint8_t);				// Sets tha traffic light's state based on the input (on, off or blinking)

uint8_t Get_Port_Traffic(uint8_t);							// Returns the traffic light's port from the master table based on the light's ID
uint8_t Get_Pin_Traffic(uint8_t);							// Returns the traffic light's pin from the master table based on the light's ID

void	Set_TrafficBlinkCounter(uint8_t, uint8_t);			// Sets the blink delay counter based on the light's ID and our input
void	Tick_TrafficBlinkCounter(uint8_t);					// Iterates downwards the blink delay counter
uint8_t Get_TrafficBlinkCounter(uint8_t);					// Returns the blink delay counter's current value



// Train
void	Controller_TrainLight();							// The train light's main controller, it turns on or off the lights

void	Set_TrainFromLeft();								// Starts or stops the train mode and outher functions if a train is comming or went to the left
void	Set_TrainFromRight();								// Starts or stops the train mode and outher functions if a train is comming or went to the right

uint8_t Get_Port_Train(uint8_t);							// Returns the train lights or buttons port from the master table
uint8_t Get_Pin_Train(uint8_t);								// Returns the train lights or buttons pins from the master table
void	Set_State_Train(uint8_t, uint8_t);					// Sets the train light's state based on the input  (on, off or blinking)

void	Set_TrainManager(uint8_t);							// Sets a bit to 1 in the train manager variable based on the input
void	Clear_TrainManager(uint8_t);						// Clears a bit to 0 in the train manager variable based on the input
uint8_t Get_TrainManager(uint8_t);							// Returns a bit's state in the train manager variable based on the input
uint8_t Is_TrainManager_Gone();								// Returns true or false if there are no trains on the tracks (Both left and right switches triggered)



// Transition
void	TransitionState();									// Initializes the tranisition state (Every light will turn red)

void	Clear_AllVolatile();								// Clears the volatile line (sets every state to 0)
void	Set_Next_Volatile(uint8_t);							// Sets the next state's adress in the volatile line
void	Set_Data_Volatile(uint8_t);							// Sets the light state to on or blink based on the input lightID



// Button
void	Controller_Button();								// The buttons main controller, it starts other functions based on the button presses

uint8_t Check_ButtonTrainLeft();							// Returns the left train button's current state
uint8_t Check_ButtonTrainRight();							// Returns the right train button's current state
uint8_t IsPressed_ButtonTrain();							// Returns which button is pressed if there is any

uint8_t Check_ButtonServiceOn();							// Returns the service mode on button's current state
uint8_t Check_ButtonServiceOff();							// Returns the service mode off button's current state
uint8_t IsPressed_ButtonService();							// Returns which button is pressed if there is any

void	Set_ButtonHoldManager(uint8_t);						// Sets a bit to 1 in the button hold manager variable based on the input
void	Clear_ButtonHoldManager(uint8_t);					// Clears a bit to 0 in the button hold manager variable based on the input
uint8_t Get_ButtonHoldManager(uint8_t);						// Returns a bit's state in the button hold manager variable based on the input



// Master table
uint8_t Get_Next_MasterTable();								// Returns the next state's address from the mastertable, based on the current line (masteriterator)
uint8_t Get_Delay_MasterTable();							// Returns the delay from the mastertable, based on the current line (masteriterator)
uint8_t Get_Type_MasterTable(uint8_t);						// Returns the light's type from the mastertable based on the lightID
uint8_t Get_CurrentState_MasterTable(uint8_t);				// Returns the light's current state from the mastertable, based on the current line and the lightID
uint8_t Get_Data_MasterTable(uint8_t, uint8_t);				// Returns a data from the mastertable based on the inputs (row and coulumn)
uint8_t Is_End_MasterTable(uint8_t);						// Returns true if the current lightID is the end of the master table










//////////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////////

// Ports, pins and DDRs
volatile uint8_t *Ports[8] = {0, &PORTA, &PORTB, &PORTC, &PORTD, &PORTE, &PORTF, &PORTG};
volatile uint8_t *Pins[8]  = {0, &PINA,  &PINB,  &PINC,  &PIND,  &PINE,  &PINF,  &PING};
volatile uint8_t *DDRs[8]  = {0, &DDRA,  &DDRB,  &DDRC,  &DDRD,  &DDRE,  &DDRF,  &DDRG};

// Service mode state
uint8_t ServiceMode = 0;

// Manager byte for the buttons
uint8_t ButtonHoldManager = 0;
	// 0. bit - Train button
	// 1. bit - Service button

// Train light variables
uint8_t TrainBlinkDelay = 0;			// Delay counter for the train lights
uint8_t TrainManager = 0;				// Manager byte for the train
	// 0. bit - Red separate blinking
	// 1. bit - Left button
	// 2. bit - Right button
	// 3. bit - Is there a train

// Light state iterators
uint8_t DataIterator = 0;				// Mainly used for going thru the lightIDs
uint8_t TrafficIterator = 0;			// Used for going thru lightID's different states
uint8_t TransitionDataIterator = 0;		// Used for going thru the lightIDs within the volatile line

// Master control
uint8_t CurrentDelay = 0;				// Current state's delay, it decreases downwards
uint8_t NewData = 0;					// A flag if if new data/line needs to be read
uint8_t MasterIterator = 0;				// The current line's ID










//////////////////////////////////////////////////////////////////////////
// Main function
//////////////////////////////////////////////////////////////////////////

int main(void) {
	
	PortInitialization();									// Initializes the ports
	TimerInitialization(CURRENT_MICROCONTROLLER);			// Initializes the timers
	Start_NewProgram(PROGRAM_Normal_All_Red);				// Starts the light's program

    while (1) {
		if (ENABLE_DEBUG_MODE) {							// If debug mode is enabled
			DebugMode();									//   Runs the debug mode function
		}
    }
}



//////////////////////////////////////////////////////////////////////////
// Timer1's interrupt, exactly 100ms
//////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect) {
	
	// New Data Controler
	if (NewData) {											// Of the NewData flag is set
		NewData = 0;										//   Clears the NewData flag
		CurrentDelay = Get_Delay_MasterTable();				//   Sets the current delay from the master table
	}

	// Light controllers
	Controller_TrafficLight();								// Runs the traffic light controller
	Controller_TrainLight();								// Runs the train light controller

	// Master Controller
	if (CurrentDelay <= 1) {								// If the delay counter is at it's minimum
		NewData = 1;										//   Sets the NewData flag
		
		if (Get_Next_MasterTable()) {						//   Ha a master table nextState értéke nem nulla
			MasterIterator = Get_Next_MasterTable();		//     Set the iterator to the next address from the master table
		} else {											//   If the master table's nextState's value is zero
			MasterIterator++;								//     Set the iterator to the next line
		}
	} else {												// If the delay counter is not at it's minimum
		CurrentDelay--;										//   Iterates the delay counter downwards
	}
}



//////////////////////////////////////////////////////////////////////////
// Timer0's interrupt for the buttons, around ~16-33ms (based on the micro-controller)
//////////////////////////////////////////////////////////////////////////
ISR(TIMER0_OVF_vect) {
	
	// Runs the button controller
	Controller_Button();
}










//////////////////////////////////////////////////////////////////////////
// Mode controllers
//////////////////////////////////////////////////////////////////////////

// Starts a new program based on the parameter's address
void Start_NewProgram(uint8_t modeID) {
	MasterIterator = modeID;					// Sets the masteriterator to the parameters address
	NewData = 1;								// Sets the NewData flag that there is new data
}

// Starts the service mode
void Start_ServiceMode() {
	if (!ServiceMode) {												// It only runs if it's not in service mode
		ServiceMode = 1;											// Turns on the service mode variable
		Start_NewProgram(PROGRAM_ServiceMode);						// Starts the service mode
	}
}

// Ends the service mode
void Stop_ServiceMode() {
	if (ServiceMode) {												// It only runs if it's in service mode
		ServiceMode = 0;											// Turns of the service mode variable
		if (Get_TrainManager(TRAINMANAGER_Bit_Is_Train)) {			// Which program starts depends on the train
			Start_NewProgram(PROGRAM_Train_All_Red);				// If there is a train the train mode starts
		} else {
			Start_NewProgram(PROGRAM_Normal_All_Red);				// If there is no train normal mode starts
		}
	}
}

// Triggers train mode if a train is coming from the left
void Set_TrainFromLeft() {
	if (!(Get_TrainManager(TRAINMANAGER_Bit_Left))) {
		Set_TrainManager(TRAINMANAGER_Bit_Left);
		TransitionState();
	}
}

// Triggers train mode if a train is coming from the right
void Set_TrainFromRight() {
	if (!(Get_TrainManager(TRAINMANAGER_Bit_Right))) {
		Set_TrainManager(TRAINMANAGER_Bit_Right);
		TransitionState();
	}
}










//////////////////////////////////////////////////////////////////////////
// Traffic light master controller
//////////////////////////////////////////////////////////////////////////
void Controller_TrafficLight() {
	
	DataIterator = DATA_Traffic_Light_StartID;					// Starts from the first light's id
	
	while (1) {
		switch (Get_Type_MasterTable(DataIterator)) {			// Check's the light type
			case TYPE_CarLight:
				Controller_TrafficCarLight();					// If it's a car light runs the corresponding function
				break;
			case TYPE_PedestrianLight:
				Controller_TrafficPedestrianLight();			// If it's a pedestrian light runs the corresponding function
				break;
			
			// In case of an error it switches to service mode
			default:
				Start_ServiceMode();
				break;
		}
		
		DataIterator += DATA_Traffic_Light_NextID;			// Goes to the next light's id
		
		if (Is_End_MasterTable(DataIterator)) {				// If it reached the end of the mastertable it breaks from the loop
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Car traffic light controller
//////////////////////////////////////////////////////////////////////////
void Controller_TrafficCarLight() {
	for (TrafficIterator = 0; TrafficIterator < DATA_Lenght; TrafficIterator++) {		// Goes thru a light's all 4 state
		Set_TrafficLight_MasterTable(DataIterator + TrafficIterator);					// Set's the light corresponding to the master table's state 
	}
}

//////////////////////////////////////////////////////////////////////////
// Pedestrian traffic light controller
//////////////////////////////////////////////////////////////////////////
void Controller_TrafficPedestrianLight() {
	Set_TrafficLight_MasterTable(DataIterator + DATA_Pedestrian_Red_Offset);							// Set's the red pedestrianlight corresponding to the master table's state 
	
	if (Get_Port_Traffic(DataIterator + DATA_Pedestrian_Green_Offset)) {								// Checks if the light's port can be found in the master table
		if (Get_CurrentState_MasterTable(DataIterator + DATA_Pedestrian_Green_Offset)) {				// If the light's current state is green in the master table
			Set_State_Traffic(STATE_ON, DataIterator + DATA_Pedestrian_Green_Offset);					//   It sets it to green
		} else if (Get_CurrentState_MasterTable(DataIterator + DATA_Pedestrian_Blink_Offset)) {			// If the light's current state is blinking in the master table
			if (Get_TrafficBlinkCounter(DataIterator + DATA_Pedestrian_Blink_Offset) <= 1) {			//   It checks if the light's counter reached the end
				Set_TrafficBlinkCounter(DELAY_Blink, DataIterator + DATA_Pedestrian_Blink_Offset);		//     Reset's the light's counter based on the DELAY_Blink definition
				Set_State_Traffic(STATE_BLINK, DataIterator + DATA_Pedestrian_Blink_Offset);			//     It switches the light to the opposite state
			} else {																					//   If the light's counter haven't reached the end
				Tick_TrafficBlinkCounter(DataIterator + DATA_Pedestrian_Blink_Offset);					//     Iterates the light's counter downwards
			}
		} else {																						// If its neither on or blinking
			Set_State_Traffic(STATE_OFF, DataIterator + DATA_Pedestrian_Blink_Offset);					//   It turns of the light
		}
	}
}










//////////////////////////////////////////////////////////////////////////
// Train Light Controller
//////////////////////////////////////////////////////////////////////////
void Controller_TrainLight() {

	// Train White
	if (!(Get_TrainManager(TRAINMANAGER_Bit_Is_Train))) {				// Cheks if there is no train
		if (TrainBlinkDelay <= 1) {										// Checks if the light's counter reached the end
			TrainBlinkDelay = DELAY_Blink;								//   Resets the light's counter based on the DELAY_Blink definition
			Set_State_Train(STATE_BLINK, LIGHT_Train_White);			//   It switches the white light to the opposite state
		} else {														// If the light's counter haven't reached the end
			TrainBlinkDelay--;											//   Iterates the light's counter downwars
		}
	} else {															// If there is a train
		Set_State_Train(STATE_OFF, LIGHT_Train_White);					//   Turns off the white light
	}
	
	// Train Red
	if (Get_TrainManager(TRAINMANAGER_Bit_Is_Train)) {					// Cheks if there is a train
		if (TrainBlinkDelay <= 1) {										// Checks if the light's counter reached the end
			TrainBlinkDelay = DELAY_Blink;								//   Resets the light's counter based on the DELAY_Blink definition
			if (!(Get_TrainManager(TRAINMANAGER_Bit_Tick))) {			// If the alternating red light state is false
				Set_TrainManager(TRAINMANAGER_Bit_Tick);				//   Set's the train control's bit corresponding to the alternating state
				Set_State_Train(STATE_ON, LIGHT_Train_Red1);			//   Turns on the red1 light
				Set_State_Train(STATE_OFF, LIGHT_Train_Red2);			//   Turns off the red2 light
			} else {													// If the alternating red light state is true
				Clear_TrainManager(TRAINMANAGER_Bit_Tick);				//   Clears's the train control's bit corresponding to the alternating state
				Set_State_Train(STATE_OFF, LIGHT_Train_Red1);			//   Turns off the red1 light
				Set_State_Train(STATE_ON, LIGHT_Train_Red2);			//   Turns on the red2 light
			}
		} else {														// If the light's counter haven't reached the end
			TrainBlinkDelay--;											//   Iterates the light's counter downwars
		}
	} else {															// If there is no train
		Set_State_Train(STATE_OFF, LIGHT_Train_Red1);					//   Turns off the red1 light
		Set_State_Train(STATE_OFF, LIGHT_Train_Red2);					//   Turns off the red2 light
	}
}










//////////////////////////////////////////////////////////////////////////
// Transitions from any state to all red
//////////////////////////////////////////////////////////////////////////
void TransitionState() {
	
	// Enable Train Flag if the train comes now
	if (!(Get_TrainManager(TRAINMANAGER_Bit_Is_Train))) {			// If there was no train
		Set_TrainManager(TRAINMANAGER_Bit_Is_Train);				//   Sets the train control's bit to, "there is train"
		Set_Next_Volatile(PROGRAM_Train_All_Red);					//	 Set's the volatile line's nextState value to the train's program "all red" address
	} else {														// If there is a train
		if (Is_TrainManager_Gone()) {								//   Checks if both of the train button's were pressed
			Clear_TrainManager(TRAINMANAGER_Bit_Left);				//     Clears the train control's left button bit
			Clear_TrainManager(TRAINMANAGER_Bit_Right);				//     Clears the train control's right button bit
			Clear_TrainManager(TRAINMANAGER_Bit_Is_Train);			//	   Clears the train control's there is a train flag
			Set_Next_Volatile(PROGRAM_Normal_All_Red);				//	Set's the volatile line's nextState value to the normal's program "all red" address
		}
	}
	
	// In case of service mode do nothing
	if (ServiceMode) {
		return;	
	}
	
	// Clear the volatile line
	Clear_AllVolatile();
	
	// Disable's the interrupts
	cli();
	
	// Start from the first lightID
	TransitionDataIterator = DATA_Traffic_Light_StartID;
	
	// Set the lights states in the volatile line
	//		In case of car lights:
	//			If its red, it stays red
	//			If its red and yellow, then only the yellow stays on
	//			If its yellow, it stays yellow
	//			If its green, it switches to yellow
	//		In case of pedestrian lights:
	//			If its red, it stays red
	//			If its blinking, it stays blinking
	//			If its green, it starts blinking
	while (1) {
		switch (Get_Type_MasterTable(TransitionDataIterator)) {														// Checks the light's type
			case TYPE_CarLight:																						// If it's a car light
				if (Get_CurrentState_MasterTable(TransitionDataIterator + DATA_Traffic_Green_Offset)) {				//   Check if the light's green state is on
					Set_Data_Volatile(TransitionDataIterator + DATA_Traffic_Yellow_Offset);							//     Turns on the yellow light's state in the volatile line
				} else if (Get_CurrentState_MasterTable(TransitionDataIterator + DATA_Traffic_Yellow_Offset)) {		//   Check if the light's green state is on
					Set_Data_Volatile(TransitionDataIterator + DATA_Traffic_Yellow_Offset);							//     Turns on the yellow light's state in the volatile line
				} else {																							//   If nothing was true above
					Set_Data_Volatile(TransitionDataIterator + DATA_Traffic_Red_Offset);							//     Turns on the red light's state in the volatile line
				}
				break;
				
			case TYPE_PedestrianLight:																				// If it's a pedestrian light
				if (Get_CurrentState_MasterTable(TransitionDataIterator + DATA_Pedestrian_Green_Offset)) {			//   Check if the light's green state is on
					Set_Data_Volatile(TransitionDataIterator + DATA_Pedestrian_Blink_Offset);						//     Turns on the light's blinking state in the volatile line
				} else if (Get_CurrentState_MasterTable(TransitionDataIterator + DATA_Pedestrian_Blink_Offset)) {	//   Check if the light's green state is on
					Set_Data_Volatile(TransitionDataIterator + DATA_Pedestrian_Blink_Offset);						//     Turns on the light's blinking state in the volatile line
				} else {																							//   If nothing was true above
					Set_Data_Volatile(TransitionDataIterator + DATA_Pedestrian_Red_Offset);							//     Turns on the red light's state in the volatile line
				}
				break;
			
			// In case of an error it switches to service mode
			default:
				Start_ServiceMode();	// Start's the service mode
				sei();					// Enable's the interrupts
				return;
		}
		
		// Go to the next lightID
		TransitionDataIterator += DATA_Traffic_Light_NextID;
		
		// If it reached the last cell break from the loop
		if (Is_End_MasterTable(TransitionDataIterator)) {
			break;
		}
	}
		
	// Set te MC_i to the override start state
	Start_NewProgram(PROGRAM_Volatile);
	
	// Enable's the interrupts
	sei();
}










//////////////////////////////////////////////////////////////////////////
// Button controller
//////////////////////////////////////////////////////////////////////////

// Controls the button inputs
void Controller_Button() {
	
	// Watch the state for the train button
	switch (IsPressed_ButtonTrain()) {
		case PRESSED_Service_Button_On:
			if(!Get_ButtonHoldManager(HOLD_Train_Button)) {			// While the button is held it only runs the function once
				Set_ButtonHoldManager(HOLD_Train_Button);			// Sets that the button was pressed
				Set_TrainFromLeft();								// Triggers a function, that a train is coming from the left
			}
			break;
		case PRESSED_Service_Button_Off:
			if(!Get_ButtonHoldManager(HOLD_Train_Button)) {			// While the button is held it only runs the function once
				Set_ButtonHoldManager(HOLD_Train_Button);			// Sets that the button was pressed
				Set_TrainFromRight();								// Triggers a function, that a train is coming from the right
			}
			break;
		default:
			Clear_ButtonHoldManager(HOLD_Train_Button);				// If the button is no longer pressed it clears the button manager
			break;
	}
	
	// Watch the state for the service mode button
	switch (IsPressed_ButtonService()) {
		case PRESSED_Train_Button_Left:
			if(!Get_ButtonHoldManager(HOLD_Service_Button)) {		// While the button is held it only runs the function once
				Set_ButtonHoldManager(HOLD_Service_Button);			// Sets that the button was pressed
				Start_ServiceMode();								// Start's the service mode
			}
			break;
		case PRESSED_Train_Button_Right:
			if(!Get_ButtonHoldManager(HOLD_Service_Button)) {		// While the button is held it only runs the function once
				Set_ButtonHoldManager(HOLD_Service_Button);			// Sets that the button was pressed
				Stop_ServiceMode();									// Stop's the service mode
			}
			break;
		default:
			Clear_ButtonHoldManager(HOLD_Service_Button);			// If the button is no longer pressed it clears the button manager
			break;
	}
}

// Train Button Controller
//   Returns which train button is pressed
uint8_t IsPressed_ButtonTrain() {
	if (Check_ButtonTrainLeft()) {
		return PRESSED_Train_Button_Left;
	}
	
	if (Check_ButtonTrainRight()) {
		return PRESSED_Train_Button_Right;
	}
	
	return PRESSED_Train_Button_None;
}

// Service mode button controller
//   Returns which service mode button is pressed
uint8_t IsPressed_ButtonService() {
	if (Check_ButtonServiceOn()) {
		return PRESSED_Service_Button_On;
	}
	
	if (Check_ButtonServiceOff()) {
		return PRESSED_Service_Button_Off;
	}
	
	return PRESSED_Service_Button_None;
}










//////////////////////////////////////////////////////////////////////////
// Initialization and debugging
//////////////////////////////////////////////////////////////////////////

// Timer initialization
void TimerInitialization(uint8_t clock_speed){
	
	// If the micro-controller has a 16 MHz clock
	if (clock_speed == 16) {
		// TIMER1 CTC COMPA 256 10Hz - 100ms
		TCCR1A = (0<<WGM11) | (0<<WGM10);
		TCCR1B = (0<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);
		TIMSK |= (1<<OCIE1A);
		OCR1A = 6250-1;
		
		// TIMER 0 OVFLW 256 ~61Hz - ~16ms
		TCCR0 = (0<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (1<<CS00);
		TIMSK |= (0<<OCIE0) | (1 << TOIE0);
		
	// If the micro-controller has a 8 MHz clock
	} else if (clock_speed == 8) {
		// TIMER1 CTC COMPA 256 10Hz - 100ms
		TCCR1A = (0<<WGM11) | (0<<WGM10);
		TCCR1B = (0<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);
		TIMSK |= (1<<OCIE1A);
		OCR1A = 3125-1;
		
		// TIMER 0 OVFLW 256 ~30Hz - ~33ms
		TCCR0 = (0<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (1<<CS00);
		TIMSK |= (0<<OCIE0) | (1 << TOIE0);
	}
	
	// Enable Interrupts
	sei();
}

// Port initialization based on the master table
void PortInitialization() {
	
	// This variable will go thru all the lights
	uint8_t initIterator = 0;
	
	// It set the first light's id
	DataIterator = DATA_Traffic_Light_StartID;
	
	// Traffic lights and pedestrian lights
	while (1) {
		for (initIterator = 0; initIterator < DATA_Lenght; initIterator++) {		// Each light consists of 4 states, we go thru all of them
			SetDDR(DDR_Traffic_Light, DataIterator + initIterator);					// Send's the parameter to be set as input pins
		}
		
		DataIterator += DATA_Traffic_Light_NextID;									// Goes to the next light
		
		if (Is_End_MasterTable(DataIterator)){										// If it reached the end of the mastertable it breaks from the loop
			break;
		}
	}
	
	// Train lights
	for (initIterator = 0; initIterator < TRAIN_Light_Count; initIterator++) {		// Goes thru all of the train lights
		SetDDR(DDR_Train_Light, TRAIN_First_Light + (initIterator * 2));			// Send's the parameter to be set as input pins
	}
	
	// Train Buttons
	SetDDR(DDR_Train_Button, DATA_Train_Button_Left_Port);							// Sets the left train button's pins to input and sets the pull-up resistor
	SetDDR(DDR_Train_Button, DATA_Train_Button_Right_Port);							// Sets the right train button's pins to input and sets the pull-up resistor
	
	// Service Buttons
	SetDDR(DDR_Service_Button, DATA_Service_Button_On);								// Sets the service mode on button's pins to input and sets the pull-up resistor
	SetDDR(DDR_Service_Button, DATA_Service_Button_Off);							// Sets the service mode off button's pins to input and sets the pull-up resistor
	
	if(ENABLE_DEBUG_MODE) {															// If the debug mode is enabled
		LED_port_init();															//   Initialize the LED pins on the board
	}
}

// Runs the corresponding pin initialization function based on the type parameter
void SetDDR(uint8_t type, uint8_t id) {
	switch (type) {
		case DDR_Traffic_Light:
			SetDDR_TrafficLight(id);
			return;
		case DDR_Train_Light:
			SetDDR_TrainLight(id);
			return;
		case DDR_Train_Button:
			SetDDR_TrainButton(id);
			return;
		case DDR_Service_Button:
			SetDDR_ServiceButton(id);
			return;
		default:
			return;
	}
}

// Initializes the pins as outputs for the traffic lights
void SetDDR_TrafficLight(uint8_t lightID) {
	if (Get_Port_Traffic(lightID)) {															// If there is a port based on the lightID
		*DDRs[Get_Port_Traffic(lightID)] |= (1 << Get_Pin_Traffic(lightID));					//   Sets the pins as outputs
	}
}

// Initializes the pins as outputs for the train lights
void SetDDR_TrainLight(uint8_t lightID) {
	if (Get_Port_Train(lightID)) {																// If there is a port based on the lightID
		*DDRs[Get_Port_Train(lightID)] |= (1 << Get_Pin_Train(lightID + TRAIN_Pin_Offset));		//   Sets the pins as outputs
	}
}

// Initializes the pins as inputs for the train buttons and sets the pull-up resistor
void SetDDR_TrainButton(uint8_t buttonID) {
	if (Get_Port_Train(buttonID)) {																// If there is a port based on the buttonID
		*DDRs[Get_Port_Train(buttonID)] &= ~(1 << Get_Pin_Train(buttonID + TRAIN_Pin_Offset));	//   Sets the pins as inputs
		*Ports[Get_Port_Train(buttonID)] |= (1 << Get_Pin_Train(buttonID + TRAIN_Pin_Offset));	//   Sets the pull-up resistor
	}
}

// Initializes the pins as inputs for the service mode buttons and sets the pull-up resistor
void SetDDR_ServiceButton(uint8_t buttonID) {
	if (Get_Data_MasterTable(buttonID, DATA_Service_Button_Port)) {																				// If there is a port based on the buttonID
		*DDRs[Get_Data_MasterTable(buttonID, DATA_Service_Button_Port)] &= ~(1 << Get_Data_MasterTable(buttonID, DATA_Service_Button_Pin));		//   Sets the pins as inputs
		*Ports[Get_Data_MasterTable(buttonID, DATA_Service_Button_Port)] |= (1 << Get_Data_MasterTable(buttonID, DATA_Service_Button_Pin));		//   Sets the pull-up resistor
	}
}

// Port initialization
void LED_port_init(){
	DDRB |= 0xF0;					// LED 0-3
	DDRD |= 0xF0;					// LED 4-7
}

// LED manager
void LED_out(uint8_t led){
	PORTD &= 0x0F;					// Clear the upper 4 bit
	PORTD |= led & 0xF0;			// Set the upper 4 bit
	PORTB &= 0x0F;					// Clear the upper 4 bit
	PORTB |= (led<<4) & 0xF0;		// Shift the lower 4 bits up and set it
}

// Debug mode
void DebugMode() {
	// Print's out the MasterIterator's current line to the leds
	LED_out(MasterIterator);
}










////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SUPPLEMENTARY / SERVICE FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
// Master table supplementary functions
//////////////////////////////////////////////////////////////////////////

// Returns the data from the master table corresponding to the parameters
uint8_t Get_Data_MasterTable(uint8_t line, uint8_t id) {
	return MasterTable[line][id];
}

// Returns the next program addres from the current line
uint8_t Get_Next_MasterTable() {
	return MasterTable[MasterIterator][MASTERTABLE_NextState];
}

// Returns the current line's delay from the master table
uint8_t Get_Delay_MasterTable() {
	return MasterTable[MasterIterator][MASTERTABLE_Delay];
}

// Returns the type from the master table corresponding to the lightID
uint8_t Get_Type_MasterTable(uint8_t lightID) {
	return MasterTable[DATA_Type_Line][lightID];
}

// Returns the lights state from the master table corresponding to the lightID
uint8_t Get_CurrentState_MasterTable(uint8_t lightID) {
	return MasterTable[MasterIterator][lightID];
}

// Checks if the next data is the end data (255)
uint8_t Is_End_MasterTable(uint8_t data) {
	if (MasterTable[DATA_Type_Line][data] == 255) {
		return 1;
	} else {
		return 0;
	}
}



//////////////////////////////////////////////////////////////////////////
// Traffic light supplementary functions
//////////////////////////////////////////////////////////////////////////

// Sets the state of the traffic lights based on the master table's state
void Set_TrafficLight_MasterTable(uint8_t lightID) {
	if (Get_Port_Traffic(lightID)) {											// Checks if the light's port can be found in the master table
		Set_State_Traffic(Get_CurrentState_MasterTable(lightID), lightID);		// Set's the light's state based on the master table's state
	}
}

// Sets the state of the traffic lights based on the parameter's state
void Set_State_Traffic(uint8_t newState, uint8_t lightID) {
	switch (newState) {																	// Check's the input parameter's state
		case STATE_OFF:																	// Set it to off
			*Ports[Get_Port_Traffic(lightID)] &= ~(1 << Get_Pin_Traffic(lightID));		//   Turn's the light on
			return;
		case STATE_ON:																	// Set it to on
			*Ports[Get_Port_Traffic(lightID)] |= (1 << Get_Pin_Traffic(lightID));		//   Turn's the light off
			return;
		case STATE_BLINK:																// Set it to blinking
			*Ports[Get_Port_Traffic(lightID)] ^= (1 << Get_Pin_Traffic(lightID));		//   It switches the light to the opposite state
			return;
		
		// In case of an error it switches to service mode
		default:
			Start_ServiceMode();
			return;
		
	}
}

// Returns the port from the master table corresponding to the lightID
uint8_t Get_Port_Traffic(uint8_t lightID) {
	return MasterTable[DATA_Port_Line][lightID];
}

// Returns the pin from the master table corresponding to the lightID
uint8_t Get_Pin_Traffic(uint8_t lightID) {
	return MasterTable[DATA_Pin_Line][lightID];
}

// Returns the pedestrian light's blink counter
uint8_t Get_TrafficBlinkCounter(uint8_t lightID) {
	return MasterTable[DATA_Type_Line][lightID];
}

// Sets the pedestrian light's blink counter
void Set_TrafficBlinkCounter(uint8_t newDelay, uint8_t lightID) {
	MasterTable[DATA_Type_Line][lightID] = newDelay;
}

// Iterates the pedestrian light's blink counter
void Tick_TrafficBlinkCounter(uint8_t lightID) {
	MasterTable[DATA_Type_Line][lightID]--;
}



//////////////////////////////////////////////////////////////////////////
// Train light supplementary functions
//////////////////////////////////////////////////////////////////////////

// Sets the bit in the TrainManager corresponding to the dataBit parameter
void Set_TrainManager(uint8_t dataBit) {
	TrainManager |= (1 << dataBit);
}

// Clears the bit in the TrainManager corresponding to the dataBit parameter
void Clear_TrainManager(uint8_t dataBit) {
	TrainManager &= ~(1 << dataBit);
}

// Returns the bit in the TrainManager corresponding to the dataBit parameter
uint8_t Get_TrainManager(uint8_t dataBit) {
	return TrainManager & (1 << dataBit);
}

// Checks if both of the train left and right buttons were pressed
uint8_t Is_TrainManager_Gone() {
	return Get_TrainManager(TRAINMANAGER_Bit_Left) && Get_TrainManager(TRAINMANAGER_Bit_Right);
};

// Sets the state of the train lights based on the parameter's state
void Set_State_Train(uint8_t newState, uint8_t lightID) {
	switch (newState) {																					// Checks the parameter's state
		case STATE_OFF:																					// Set it to off
			*Ports[Get_Port_Train(lightID)] &= ~(1 << Get_Pin_Train(lightID + TRAIN_Pin_Offset));		//   Turn's the light on
			return;
		case STATE_ON:																					// Set it to on
			*Ports[Get_Port_Train(lightID)] |= (1 << Get_Pin_Train(lightID + TRAIN_Pin_Offset));		//   Turn's the light off
			return;
		case STATE_BLINK:																				// Set it to blinking
			*Ports[Get_Port_Train(lightID)] ^= (1 << Get_Pin_Train(lightID + TRAIN_Pin_Offset));		//   It switches the light to the opposite state
			return;
		
		// In case of an error it switches to service mode
		default:
			Start_ServiceMode();
			return;
	}
}

// Returns the train light's ports from the master table
uint8_t Get_Port_Train(uint8_t port) {
	return MasterTable[DATA_Train][port];
}

// Returns the train light's pins from the master table
uint8_t Get_Pin_Train(uint8_t pin) {
	return MasterTable[DATA_Train][pin];
}



//////////////////////////////////////////////////////////////////////////
// Transition supplementary functions
//////////////////////////////////////////////////////////////////////////

// Clears the volatile lines
void Clear_AllVolatile() {
	// Start from the first data
	TransitionDataIterator = DATA_Traffic_Light_StartID;				// Starts from the first light's id
	
	// Reset the override state
	while (1) {
		MasterTable[PROGRAM_Volatile][TransitionDataIterator] = 0;		// Clears the state in the master table's volatile line
		TransitionDataIterator++;										// Advences the iterator to the next state
		
		if (Is_End_MasterTable(TransitionDataIterator)) {				// If it reached the end of the mastertable it breaks from the loop
			break;
		}
	}
}

// Sets the next state value in the volatile line
void Set_Next_Volatile(uint8_t nextState) {
	MasterTable[PROGRAM_Volatile][MASTERTABLE_NextState] = nextState;
}

// Sets the light state data in the volatile line
void Set_Data_Volatile(uint8_t data) {
	MasterTable[PROGRAM_Volatile][data] = 1;
}



//////////////////////////////////////////////////////////////////////////
// Button manager/service functions
//////////////////////////////////////////////////////////////////////////

// Sets the bit in the ButtonHoldManager corresponding to the button parameter
void Set_ButtonHoldManager(uint8_t buttonID) {
	ButtonHoldManager |= (1 << buttonID);
}

// Clears the bit in the ButtonHoldManager corresponding to the button parameter
void Clear_ButtonHoldManager(uint8_t buttonID) {
	ButtonHoldManager &= ~(1 << buttonID);
}

// Returns the bit in the ButtonHoldManager corresponding to the button parameter
uint8_t Get_ButtonHoldManager(uint8_t buttonID) {
	return ButtonHoldManager & (1 << buttonID);
}

// FOR THE TRAIN BUTTON
// Checks if the left train button is pressed
uint8_t Check_ButtonTrainLeft() {
	// It has a negation because the button is normaly closed
	return !(*Pins[Get_Port_Train(DATA_Train_Button_Left_Port)] & (1 << Get_Pin_Train(DATA_Train_Button_Left_Port + TRAIN_Pin_Offset)));
}

// Checks if the right train button is pressed
uint8_t Check_ButtonTrainRight() {
	// It has a negation because the button is normaly closed
	return !(*Pins[Get_Port_Train(DATA_Train_Button_Right_Port)] & (1 << Get_Pin_Train(DATA_Train_Button_Right_Port + TRAIN_Pin_Offset)));
}

// FOR THE SERVICE MODE BUTTON
// Checks if the service mode on button is pressed
uint8_t Check_ButtonServiceOn() {
	return (*Pins[Get_Data_MasterTable(DATA_Service_Button_On, DATA_Service_Button_Port)] & (1 << Get_Data_MasterTable(DATA_Service_Button_On, DATA_Service_Button_Pin)));
}

// Checks if the service mode off button is pressed
uint8_t Check_ButtonServiceOff() {
	return (*Pins[Get_Data_MasterTable(DATA_Service_Button_Off, DATA_Service_Button_Port)] & (1 << Get_Data_MasterTable(DATA_Service_Button_Off, DATA_Service_Button_Pin)));
}