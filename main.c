/*
 * File:   main.c
 * Author: Roger Berry
 *
 * Created on 27 December 2019, 16:12
 */


#include <xc.h>
#include "Init.h"   //this file includes all of the port definitions
#include "Comms.h"  //this file includes comms functions
#include "ADC.h"    //this file includes ADC functions
#include "DAC.h"    //this file includes DAC functions
#include "PWM.h"    //this file includes PWM functions
#include "SPI.h"    //this file includes SPI and DREV8711 functions
#include "Timer.h"  //this file includes timer functions

/*
 * The default state for the CONFIG registers is as follows:
 * CONFI1 register:
 *  FCMEN = ON
 *  IESO = ON
 *  CLKOUTEN = ON (this is inverted logic and is therefore disabled
 *  BOREN = ON (both bits set and is enabled
 *  CP = OFF 
 *  MCLR = ON 
 *  PWRTE = OFF
 *  WDTE = ON
 *  FOSC = ECH
 * 
 * CONFIG2 register
 *  LVP = ON
 *  DEBUG = OFF
 *  LPBOR = OFF
 *  BORV = LO
 *  STVREN = ON
 *  PLLEN = ON
 *  ZCD = OFF
 *  PPS1WAY = ON
 *  WRT = OFF
 * 
 * 
 * On the basis that these are the default settings then we need to consider how we are using the device.
 * The following pragma statements configure the device as required
 *  
 * 
*/

//CONFIG1
#pragma config FCMEN 	= OFF 		// Fail safe clock disabled
#pragma config IESO 	= OFF  		// Internal/external switch over disable
#pragma config MCLRE    = OFF       // MCLR pin is a digital input
#pragma config WDTE     = OFF       // watch dog off
#pragma config FOSC	 	= HS    	// crystal oscillator

//CONFIG2
#pragma config LVP      = OFF       // low voltage programming disabled


//define constants

#define     STEPPER_MOTOR       1
#define     DC_MOTOR            2

//define strings
const unsigned char OptionMessage[] = "\r\n\r\n **** SYSTEM OPION LIST ****\r\n";
const unsigned char OptionMessage1[] = " 1: Test relay 1\r\n";
const unsigned char OptionMessage2[] = " 2: Test relay 2\r\n";
const unsigned char OptionMessage3[] = " 3: Test relay 3\r\n";
const unsigned char OptionMessage4[] = " 4: Display speed control input value\r\n";
const unsigned char OptionMessage5[] = " 5: Display analogue channel 1 input value\r\n";
const unsigned char OptionMessage6[] = " 6: Display analogue channel 2 input value\r\n";
const unsigned char OptionMessage7[] = " 7: Test DAC 1 output\r\n";
const unsigned char OptionMessage8[] = " 8: Test DAC 2 output\r\n";
const unsigned char OptionMessage9[] = " 9: Test PWM 1\r\n";
const unsigned char OptionMessage10[] = "10: Test PWM 2\r\n";
const unsigned char OptionMessage11[] = "11: Test PWM 3\r\n";
const unsigned char OptionMessage12[] = "12: Test PWM 4\r\n";
const unsigned char OptionMessage13[] = "13: Test stepper motor\r\n";
const unsigned char OptionMessage14[] = "14: Test DC motor 1\r\n";
const unsigned char OptionMessage15[] = "15: Test DC motor 2\r\n";
const unsigned char OptionMessage16[] = "16: Read DRV8711 STATUS register register\r\n";
const unsigned char OptionMessage17[] = "17: Clear DRV8711 STATUS register\r\n";
const unsigned char OptionMessage18[] = "18: Test GPIO inputs\r\n";
const unsigned char OptionMessage19[] = "19: Test Absolute Encoder\r\n";
const unsigned char OptionSelectMessage[] = "\r\nEnter option number: ";
const unsigned char CRLF[] = "\r\n";

//stepper/DC motor test options list
const unsigned char StepperOptionMessage[] = "\r\n\r\n **** STEPPER MOTOR TEST OPION LIST ****\r\n";
const unsigned char DC_MotorOptionMessage[] = "\r\n\r\n **** DC MOTOR TEST OPION LIST ****\r\n";
const unsigned char MotorOptionMessage1[] = " 1: Toggle direction\r\n";
const unsigned char MotorOptionMessage2[] = " 2: Set the motor step interval\r\n";
const unsigned char MotorOptionMessage2a[] = " 2: Set the PWM duty cycle\r\n";
const unsigned char MotorOptionMessage3[] = " 3: Start motor\r\n";
const unsigned char MotorOptionMessage4[] = " 4: Return to main menu\r\n";
const unsigned char MotorRunningMessage[] = "\r\n Motor running. Enter any character to stop: ";

//stepper/DC Motor drive status messages
const unsigned char StepperMotorStatusMessage[] = "\r\n\r\n*** STEPPER MOTOR DRIVE STATUS ***\r\n";
const unsigned char StepperMotorStatusMessage1[] = "\r\n      Direction: ";
const unsigned char StepperMotorStatusMessage2[] = "\r\n  Step interval: ";
const unsigned char DC_MotorStatusMessage[] = "\r\n\r\n*** DC MOTOR DRIVE STATUS ***\r\n";
const unsigned char DC_MotorStatusMessage1[] = "\r\n                  Direction: ";
const unsigned char DC_MotorStatusMessage2[] = "\r\n  PWM duty cycle percentage: ";
const unsigned char Clockwise[] = "CLOCKWISE";
const unsigned char AntiClockwise[] = "ANTICLOCKWISE";


//string error messages
const unsigned char MessageTooLong[] = "\r\n String entered is too long";
const unsigned char MessageNoValue[] = "\r\n No Value Entered";
const unsigned char InvalidNumber[] = "\r\n Value out of range";
const unsigned char TooManyDecimalPoints[] = "\r\n Too many decimal points";
const unsigned char TooLarge[] = "\r\n Value too large";
const unsigned char TooSmall[] = "\r\n Value too small";

//relay option message
const unsigned char RelayOptionMessage[] = "\r\n1 = ON, 0 = OFF, X/x = EXIT. Enter value: ";

//DAC test message
const unsigned char DAC_TestMessage[] = "\r\n DAC test running. Press X/x to exit: ";

//PWM test message
const unsigned char PWM_16Bit_TestMessage[] = "\r\n Enter a value between 1000 and 2000. Enter 0 to exit: ";
const unsigned char PWM_10Bit_TestMessage[] = "\r\n Enter a percentage value between 1 and 99. Enter 0 to exit: ";

//Stepper motor speed message
const unsigned char StepperMotorSpeedMessage[] = "\r\n Enter a step interval in microseconds. Value between 500 and 9999: ";

//DC motor speed message
const unsigned char DC_MotorSpeedMessage[] = "\r\n Enter a percentage speed value. Value between 1 and 99: ";

//GPIO messages
const unsigned char TestGPIO_Message[] = "\r\n GPIO test running. Press any key to exit";
const unsigned char GPIO_1_Message[] = "\r\n GPIO 1 = ";
const unsigned char GPIO_2_Message[] = "\r\n GPIO 2 = ";
const unsigned char GPIO_3_Message[] = "\r\n GPIO 3 = ";
const unsigned char GPIO_4_Message[] = "\r\n GPIO 4 = ";
const unsigned char GPIO_5_Message[] = "\r\n GPIO 5 = ";
const unsigned char GPIO_6_Message[] = "\r\n GPIO 6 = ";
const unsigned char GPIO_7_Message[] = "\r\n GPIO 7 = ";
const unsigned char GPIO_8_Message[] = "\r\n GPIO 8 = ";
const unsigned char GPIO_ON_Message[] = "ON";
const unsigned char GPIO_OFF_Message[] = "OFF";

//absolute encoder messages
const unsigned char TestAbsoluteEncoderMessage[] = "\r\n Absolute Encoder test running. Press any key to exit";
const unsigned char Position0[] = "0 ";
const unsigned char Position1[] = "22.5 ";
const unsigned char Position2[] = "45 ";
const unsigned char Position3[] = "67.5 ";
const unsigned char Position4[] = "90 ";
const unsigned char Position5[] = "112.5 ";
const unsigned char Position6[] = "135 ";
const unsigned char Position7[] = "157.5 ";
const unsigned char Position8[] = "180 ";
const unsigned char Position9[] = "202.5 ";
const unsigned char Position10[] = "225 ";
const unsigned char Position11[] = "247.5 ";
const unsigned char Position12[] = "270 ";
const unsigned char Position13[] = "292.5 ";
const unsigned char Position14[] = "315 ";
const unsigned char Position15[] = "337.5 ";
const unsigned char Degrees[] = "Degrees";



//global variables
volatile unsigned int GLOBAL_TimerEventCounter = 0;
volatile unsigned int GLOBAL_TimerEventFlag = 0;
volatile unsigned int GLOBAL_MasterTimeOutCounter = 0;
volatile unsigned int GLOBAL_MasterTimeOutFlag = 0;
volatile unsigned char GLOBAL_ResultString[RESULT_STRING_LENGTH];
volatile unsigned char GLOBAL_RxString[RX_STRING_LENGTH];
volatile unsigned int GLOBAL_PWM1_PulseTime;
volatile unsigned int GLOBAL_PWM2_PulseTime;
volatile unsigned int GLOBAL_PWM3_PulseTime;
volatile unsigned int GLOBAL_PWM4_PulseTime;
volatile unsigned int GLOBAL_StepperMotorSpeed;
volatile unsigned int GLOBAL_DirectionStatus;



//list functions
void    DisplaySystemOptionsList(void);
void    DisplayStringError(unsigned int);
void    TestRelay(unsigned int);
void    DisplaySpeedControl(void);
void    DisplayAnalogueInput_1(void);
void    DisplayAnalogueInput_2(void);
void    TestDAC(unsigned int);
void    TestPWM_10Bit(unsigned int);
void    TestPWM_16Bit(unsigned int);
void    TestStepperMotor(void);
void    TestDC_Motor(unsigned int);
void    DisplayStepperMotorOptionsList(unsigned int);
void    DisplayDC_MotorOptionsList(unsigned int);
void    SetStepperMotorSpeed(void);
void    SetDC_MotorSpeed(unsigned int);
void    DisplayStepperMotorStatus(void);
void    DisplayDC_MotorStatus(unsigned int);
void    TestGPIO(void);
unsigned char   GetGPIO_Status(void);
void    DisplayGPIO_Status(unsigned char GPIO_Status);
void    TestAbsoluteEncoder(void);
void    DisplayAbsoluteEncoderPosition(unsigned char);

//main function

void main(void) {
    
    unsigned int StringStatus;
    unsigned int Value;
    unsigned int SPIAddress;
    unsigned int SPIValue;

    //wait for PLL to stabilise
    while(OSCSTATbits.PLLR == 0);
    
    InitialisePorts(); 
    InitialiseComms();
    InitialiseADC();
    InitialiseDAC();
    InitialisePWM_10Bit();
    InitialisePWM_16Bit();
    InitialiseTimers();
    InitialiseSPI();
    InitialiseDRV8711();
    
    //enable interrupts
    INTCONbits.PEIE = 1;        //enable peripheral interrupts
    INTCONbits.GIE = 1;         //enable interrupts
    
    
    //main loop
    while(1)
    {
        //display options list
        DisplaySystemOptionsList();
        
        //test for any string entry
        StringStatus = GetString(2,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //string ok
            //convert string to binary value
            Value = StringToInteger(GLOBAL_RxString);
            //test for required action
            switch(Value)
            {
                case 1:     //test relay 1
                    TestRelay(1);
                    break;
                    
                case 2:     //test relay 2
                    TestRelay(2);
                    break;
                    
                case 3:     //test relay 3
                    TestRelay(3);
                    break;
                
                case 4:     //display speed control input
                    DisplaySpeedControl();
                    break;
                    
                case 5:     //display analogue input 1
                    DisplayAnalogueInput_1();
                    break;
                    
                case 6:     //display analogue input 1
                    DisplayAnalogueInput_2();
                    break;
                    
                case 7:     //test DAC 1
                    TestDAC(1);
                    break;
                    
                case 8:     //test DAC2
                    TestDAC(2);
                    break;
                    
                case 9:     //test PWM 1
                    TestPWM_10Bit(1);
                    break;
                    
                case 10:    //test PWM 2
                    TestPWM_10Bit(2);
                    break;
                    
                case 11:    //test PWM 3
                    TestPWM_16Bit(3);
                    break;
                    
                case 12:    //test PWM 4
                    TestPWM_16Bit(4);
                    break;

                case 13:    //test stepper motor
                    TestStepperMotor();
                    break;

                case 14:     //test DC motor 1
                    TestDC_Motor(1);
                    break;
                    
                case 15:    //test DC motor 2
                    TestDC_Motor(2);
                    break;
                    
                case 16:    //Get DRV8711 status and display
                    SPIAddress = DRV_STATUS_REG;
                    SPIValue = ReadSPI(SPIAddress);
                    //display binary value
                    BinaryToResultString(2, GLOBAL_ResultString, SPIValue);
                    //display the result
                    SendMessage(CRLF);
                    SendString(GLOBAL_ResultString);
                    break;

                case 17:    //Get DRV8711 status
                    SPIAddress = DRV_STATUS_REG;
                    SPIValue = 0;
                    WriteSPI(SPIAddress,SPIValue);
                    break;

                case 18:    //test GPIO
                    TestGPIO();
                    break;
                    
                case 19:    //test absolute encoder
                    TestAbsoluteEncoder();
                    break;
                    
                default:    //invalid entry
                    SendMessage(InvalidNumber);
                    
            }
        }
    }
    //end of main loop. Should never get to this point
    return;
}

 

//*********************************************
//test relays
void    TestRelay(unsigned int RelayNumber)
{
    unsigned int Character;
    unsigned int Status = 0;
    
    //send the command string
    SendMessage(RelayOptionMessage);
    //adjust oscillator until any other key pressed
    while(Status == 0)
    {
        //test for character entry
        Character = GetChar();
        if(Character != NO_DATA)
        {
            //echo character
            SendChar(Character);
            //test for action
            switch(Character)
            {
                case '0':   //relay is off
                    //which relay?
                    switch(RelayNumber)
                    {
                        case 1: //switch relay 1 off
                            RELAY_1_WRITE = OFF;
                            break;
                            
                        case 2: //switch relay2 off
                            RELAY_2_WRITE = OFF;
                            break;
                            
                        case 3: //switch relay 3 off
                            RELAY_3_WRITE = OFF;
                            break;
                    }
                    break;  

                case '1':   //relay is on
                    //which relay?
                    switch(RelayNumber)
                    {
                        case 1: //switch relay 1 on
                            RELAY_1_WRITE = ON;
                            break;
                            
                        case 2: //switch relay2 on
                            RELAY_2_WRITE = ON;
                            break;
                            
                        case 3: //switch relay 3 on
                            RELAY_3_WRITE = ON;
                            break;
                    }
                    break;  

                case 'x':    //exit
                    Status = 1;
                    break;

                case 'X':    //exit
                    Status = 1;
                    break;
                 
                //no default condition    
                    
            }
        }
    }
    //ensure the all relays are off
    RELAY_1_WRITE = OFF;
    RELAY_2_WRITE = OFF;
    RELAY_3_WRITE = OFF;
}


//*********************************************
//Display speed control analogue input value

void    DisplaySpeedControl(void)
{
    unsigned int Value;
    
    SendMessage(CRLF);
    //get ADC value
    Value = GetSpeedControlValue();
    //Convert value into string. Maximum length 4 digits
    DecimalToResultString(Value,GLOBAL_ResultString,4);
    //transmit string
    SendString(GLOBAL_ResultString);
}



//*********************************************
//Display analogue channel 1 input value

void    DisplayAnalogueInput_1(void)
{
    unsigned int Value;
    
    SendMessage(CRLF);
    //get ADC value
    Value = GetAnalogueChannel_1_Value();
    //Convert value into string. Maximum length 4 digits
    DecimalToResultString(Value,GLOBAL_ResultString,4);
    //transmit string
    SendString(GLOBAL_ResultString);
}



//*********************************************
//Display analogue channel 2 input value

void    DisplayAnalogueInput_2(void)
{
    unsigned int Value;
    
    SendMessage(CRLF);
    //get ADC value
    Value = GetAnalogueChannel_2_Value();
    //Convert value into string. Maximum length 4 digits
    DecimalToResultString(Value,GLOBAL_ResultString,4);
    //transmit string
    SendString(GLOBAL_ResultString);
}



//*********************************************
//test selected DACX (values passed to function) and create triangle wave form
//'x' is detected

void    TestDAC(unsigned int DAC_Number)
{
    unsigned int Status = 0;
    unsigned int Character;
    unsigned int DAC_Value = 0;
    unsigned int SlopeDirection = 0;
    
    //send the command string
    SendMessage(DAC_TestMessage);
    //loop until exit character received
    while(Status == 0)
    {
        //test for exit
        Character = GetChar();
        if(Character == NO_DATA)
        {
            //no exit requested
            //create triangle waveform
            switch(DAC_Number)
            {
                case 1: //test DAC 1
                    LoadDAC_1(DAC_Value);
                    break;
                    
                case 2: //load DAC 2
                    LoadDAC_2(DAC_Value);
                    break;
                    
                //no default
            } 
            
            //increment/decrement waveform
            if(SlopeDirection == 0)
            {
                //voltage output rising
                //increment by 1
                DAC_Value++;
                //test for 1023
                if(DAC_Value == 1023)
                {
                    //limit reached for 10 bits
                    //change slope direction
                    SlopeDirection = 1;
                }
            }
            else
            {
                //voltage output falling
                //decrement by 1
                DAC_Value--;
                //test for 0
                if(DAC_Value == 0)
                {
                    //limit reached for 10 bits
                    //change slope direction
                    SlopeDirection = 0;
                }
            }
        }
        else
        {
            //character received
            //test for exit key entered    
            switch(Character)
            {
                case 'x':    //exit
                    Status = 1;
                    break;

                case 'X':    //exit
                    Status = 1;
                    break;
            
                    //no default condition  
            }
        }
    }
    //reset all DACs to zero
    DAC_Value = 0;
    LoadDAC_1(DAC_Value);
    LoadDAC_2(DAC_Value);
}



//*********************************************
//test PWM 10 bit. Pass PWM number
//asked to enter duty cycle as a percentage
//0% to exit

void    TestPWM_10Bit(unsigned int PWM_Number)
{
    unsigned int Status = 0;
    unsigned int StringStatus;
    unsigned int Value;
    
    //enable selected PWM
    switch(PWM_Number)
    {
        case 1: //PWM 1 selected
            EnablePWM_1();
            Enable_10BitPWM_Timer();
            break;
            
        case 2: //PWM 2 selected
            EnablePWM_2();
            Enable_10BitPWM_Timer();
            break;
            
        //no default    
    }
    
    //loop until exit character received
    while(Status == 0)
    {
        //send the command string
        SendMessage(PWM_10Bit_TestMessage);
        StringStatus = GetString(2,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //convert string to binary
            Value = StringToInteger(GLOBAL_RxString);
            //test for string value
            if(Value == 0)
            {
                //exit PWM test
                Status = 1;
            }
            else
            {
                //test for which PWM to change
                switch(PWM_Number)
                {
                    case 1: //load PWM 1 time value
                        GLOBAL_PWM1_PulseTime = Value * 10;
                        break;

                    case 2: //load PWM 2 time value
                        GLOBAL_PWM2_PulseTime = Value * 10;
                        break;

                    //No default
                }

            }
        }
    }
    //turn 10 bit PWM timer off
    Disable_10BitPWM_Timer();
    //disable 10 bit PWMs
    DisablePWM_1();
    DisablePWM_2();
    //reset PWM values to 10%
    GLOBAL_PWM1_PulseTime = 100;
    GLOBAL_PWM2_PulseTime = 100;
}    



//*********************************************
//test PWM 16 bit. Enter a number between 1000 and 2000 to change the duty cycle
//Time in microseconds. Enter 0 to exit

void    TestPWM_16Bit(unsigned int PWM_Number)
{
    unsigned int Status = 0;
    unsigned int StringStatus;
    unsigned int Value;
    
    //loop until exit character received
    while(Status == 0)
    {
        //send the command string
        SendMessage(PWM_16Bit_TestMessage);
        StringStatus = GetString(4,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //convert string to binary
            Value = StringToInteger(GLOBAL_RxString);
            //test for string value
            if(Value == 0)
            {
                //exit PWM test
                Status = 1;
            }
            else if(Value < 1000)   
            {
                //string value is too small
                //string error
                DisplayStringError(VALUE_TOO_SMALL);
            }
            else if(Value > 2000)
            {
                //string value is too big
                //string error
                DisplayStringError(VALUE_TOO_LARGE);
            }
            else
            {
                //test for which PWM to change
                switch(PWM_Number)
                {
                    case 3: //load PWM 5 time value
                        GLOBAL_PWM3_PulseTime = Value;
                        break;

                    case 4: //load PWM 6 time value
                        GLOBAL_PWM4_PulseTime = Value;
                        break;

                    //No default
                }

            }
        }
    }
    //reset PWM values to 1500
    GLOBAL_PWM3_PulseTime = 1500;
    GLOBAL_PWM4_PulseTime = 1500;
}



//*********************************************
//test stepper motor.
//this has a secondary HMI for selecting direction, speed and motor on/off

void    TestStepperMotor(void)
{
    unsigned int Status = 0;
    unsigned int StringStatus;
    unsigned int Value;
    unsigned int MotorStopStatus;
    
    //set for stepper mode
    SetDRV8711_Mode(STEPPER_MODE);
    
    //set direction to clockwise
    GLOBAL_DirectionStatus = 0;

    //run motor control options menu
    while(Status == 0)
    {
        //display status
        DisplayStepperMotorStatus();
        //display options list
        DisplayStepperMotorOptionsList(STEPPER_MOTOR);

        //test for any string entry
        StringStatus = GetString(1,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //string ok
            //convert string to binary value
            Value = StringToInteger(GLOBAL_RxString);
            //test for required action
            switch(Value)
            {
                case 1:     //toggle motor direction
                    if(GLOBAL_DirectionStatus == 0)
                    {
                        //change from clockwise to anticlockwise
                        DRV8711_DIR_WRITE = 0b1;
                        GLOBAL_DirectionStatus = 1;
                    }
                    else
                    {
                        //change from anticlockwise to clockwise
                        DRV8711_DIR_WRITE = 0b0;
                        GLOBAL_DirectionStatus = 0;
                    }
                    break;

                case 2:     //set motor speed
                    SetStepperMotorSpeed();
                    break;

                case 3:     //switch motor on until character received
                    SendMessage(MotorRunningMessage);
                    //stepper is off therefore turn it on
                    MotorOn();
                    //enable the stepper interrupt timer
                    StepperTimerOn();
                    //wait for character to exit
                    MotorStopStatus = 0;
                    while(MotorStopStatus == 0)
                    {
                        Value = GetChar();
                        if(Value != 0xFFFF)
                        {
                            MotorStopStatus = 1;
                        }
                    }
                    //disable the stepper interrupt timer
                    StepperTimerOff();
                    //set the step output to 0
                    DRV8711_STEP_WRITE = 0b0;
                    //switch motor drive off
                    MotorOff();
                    break;

                case 4:     //return to main screen
                    Status = 1;
                    break;

                default:    //invalid entry
                    SendMessage(InvalidNumber);
            }
        }
    }
    //ensure the stepper interrupt timer is off
    StepperTimerOff();
    //set the step output to 0
    DRV8711_STEP_WRITE = 0b0;
    //ensure that motor drive is off
    MotorOff();
    //set stepper direction output to 0 and clockwise
    DRV8711_DIR_WRITE = 0; 
}



//***********************************************
//Set stepper motor speed. load a value between 500 and 9999

void    SetStepperMotorSpeed(void)
{
    unsigned int    Value;
    unsigned int    StringStatus; 
    
    //send the command string
    SendMessage(StepperMotorSpeedMessage);
    //get the string, maximum 4 characters
    StringStatus = GetString(4,GLOBAL_RxString);
    if(StringStatus != STRING_OK)
    {
        //string error
        DisplayStringError(StringStatus);
    }
    else
    {
        //convert string to binary
        Value = StringToInteger(GLOBAL_RxString);
        //test for value too small
        if(Value < 500)
        {
            DisplayStringError(VALUE_TOO_SMALL);
        }
        else
        {
            //load the speed value into the speed control register
            GLOBAL_StepperMotorSpeed = Value;
        }
    }
}    
    


//*********************************************
//test DC motor. This has it own HMI to get direction, speed and motor control
//it is passed the number of the DC motor to control

void    TestDC_Motor(unsigned int PWM_Number)
{
    unsigned int Status = 0;
    unsigned int StringStatus;
    unsigned int Value;
    unsigned int MotorStopStatus;
    
    //set direction to clockwise
    GLOBAL_DirectionStatus = 0;
    //set for PWM mode
    SetDRV8711_Mode(PWM_MODE);
    //if PWM number = 1 then A1IN and A2IN are used on the DRV8711
    //these pins are normally connected to STEP and DIR respectively
    //if PWM number = 2 then B1IN and B2IN are used on the DRV8711
    //these pins are normally connected to PWM1 and PWM2
    //
    //Therefore to get the correct PWM control we need to apply the PWM to either
    //A1IN to PWM 1 and the A2IN is to 0V (forwards) DEFAULT
    //or
    //A2IN to PWM 1 and the A1IN is to 0V (backwards)
    //similarly:
    //B1IN to PWM 2 and the B2IN is to 0V (forwards) DEFAULT
    //or
    //B2IN to PWM 2 and the B1IN is to 0V (backwards)
    
    //set selected PWM to associated motor drive and default conditions
    switch(PWM_Number)
    {
        case 1: //PWM 1 selected therefore
                //B1IN to PWM 1 and the B2IN is to 0V (forwards) DEFAULT
            RA2PPS = PWM3_OUTPUT;       //connect PWM1 to B1IN
            DRV8711_B2IN_WRITE = 0;  
            break;
            
        case 2: //PWM 2 selected therefore
                //A1IN to PWM 2 and the A2IN is to 0V (forwards) DEFAULT
            RD2PPS = PWM4_OUTPUT;       //connect PWM2 to A1IN (STEP pin))
            DRV8711_A2IN_WRITE = 0;  
    }
    
    //run motor control options menu
    while(Status == 0)
    {
        //display status
        DisplayDC_MotorStatus(PWM_Number);
        //display options list
        DisplayDC_MotorOptionsList(STEPPER_MOTOR);

        //test for any string entry
        StringStatus = GetString(1,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //string ok
            //convert string to binary value
            Value = StringToInteger(GLOBAL_RxString);
            //test for required action
            switch(Value)
            {
                case 1:     //toggle motor direction
                    //test for which motor
                    switch(PWM_Number)
                    {
                        case 1: //PWM1 connected
                            //test present direction
                            if(GLOBAL_DirectionStatus == 0)
                            {
                                //change direction to anticlockwise
                                //B2IN to PWM 1 and the B1IN is to 0V (forwards) DEFAULT
                                RA2PPS = 0;                 //release PPS setting
                                RA3PPS = PWM3_OUTPUT;       //connect PWM1 to B2IN
                                DRV8711_B1IN_WRITE = 0;     
                                GLOBAL_DirectionStatus = 1;
                            }
                            else
                            {
                                //change direction to clockwise
                                //B2IN to PWM 1 and the B1IN is to 0V (forwards) DEFAULT
                                RA3PPS = 0;                 //release PPS setting
                                RA2PPS = PWM3_OUTPUT;       //connect PWM1 to B1IN
                                DRV8711_B2IN_WRITE = 0;     
                                GLOBAL_DirectionStatus = 0;
                            }
                            break;
                            
                        case 2: //PWM2 connected
                            //test present direction
                            if(GLOBAL_DirectionStatus == 0)
                            {
                                //change direction to anticlockwise
                                //A2IN to PWM 2 and the A1IN is to 0V 
                                RD2PPS = 0;                 //release PPS setting
                                RD1PPS = PWM4_OUTPUT;       //connect PWM2 to A2IN
                                DRV8711_A1IN_WRITE = 0;     
                                GLOBAL_DirectionStatus = 1;
                            }
                            else
                            {
                                //change direction to clockwise
                                //B2IN to PWM 1 and the B1IN is to 0V (forwards) DEFAULT
                                RD1PPS = 0;                 //release PPS setting
                                RD2PPS = PWM4_OUTPUT;       //connect PWM2 to A1IN
                                DRV8711_A2IN_WRITE = 0;     
                                GLOBAL_DirectionStatus = 0;
                            }
                            break;
                            
                        //no default   
                    }
                    break;

                case 2:     //set motor speed
                    SetDC_MotorSpeed(PWM_Number);
                    break;

                case 3:     //switch motor on until character received
                    SendMessage(MotorRunningMessage);
                    //switch motor drive on
                    MotorOn();
                    //enable selected PWM
                    switch(PWM_Number)
                    {
                        case 1: //pWM1 selected
                            EnablePWM_1();
                            break;
                            
                        case 2: //PWM2 selected
                            EnablePWM_2();
                            break;
                            
                        //no default
                            
                    }
                    //enable PWM timer
                    Enable_10BitPWM_Timer();                    
                    //wait for character to stop motor
                    MotorStopStatus = 0;
                    while(MotorStopStatus == 0)
                    {
                        Value = GetChar();
                        if(Value != 0xFFFF)
                        {
                            MotorStopStatus = 1;
                        }
                    }
                    //disable PWM timer
                    Disable_10BitPWM_Timer();  
                    //disable motor power
                    MotorOff();
                    DisablePWM_1();
                    DisablePWM_2();
                    break;

                case 4:     //return to main screen
                    Status = 1;
                    break;

                default:    //invalid entry
                    SendMessage(InvalidNumber);
            }
        }
    }
    //ensure that motor drive is off
    MotorOff();
    
    //disable 10 bit PWMs
    DisablePWM_1();
    DisablePWM_2();
    Disable_10BitPWM_Timer();
    
    //reset PWM values to 10%
    GLOBAL_PWM1_PulseTime = 100;
    GLOBAL_PWM2_PulseTime = 100;
    
    //restore PWM port allocations
    RA2PPS = 0;                 //release PPS setting
    RA3PPS = 0;                 //release PPS setting
    RD1PPS = 0;                 //release PPS setting
    RD2PPS = 0;                 //release PPS setting
    RA2PPS = PWM3_OUTPUT;       //restore PPS to original setting
    RA3PPS = PWM4_OUTPUT;       //restore PPS to original setting
    DRV8711_B1IN_WRITE = 0;     //set direction to clockwise
    DRV8711_B2IN_WRITE = 0;     //set direction to clockwise
    DRV8711_B1IN_WRITE = 0;     //set direction to clockwise
    DRV8711_B2IN_WRITE = 0;     //set direction to clockwise
}

    
//***********************************************
//Set DC motor speed. load a percentage value 1 and 99

void    SetDC_MotorSpeed(unsigned int PWM_Number)
{
    unsigned int    Value;
    unsigned int    StringStatus; 
    
    //send the command string
    SendMessage(DC_MotorSpeedMessage);
    //get the string, maximum 4 characters
    StringStatus = GetString(2,GLOBAL_RxString);
    if(StringStatus != STRING_OK)
    {
        //string error
        DisplayStringError(StringStatus);
    }
    else
    {
        //convert string to binary
        Value = StringToInteger(GLOBAL_RxString);
        //test for value too small
        if(Value < 1)
        {
            DisplayStringError(VALUE_TOO_SMALL);
        }
        else 
        {
            //test for which PWM to change
            switch(PWM_Number)
            {
                case 1: //load PWM 1 time value
                    GLOBAL_PWM1_PulseTime = Value * 10;
                    break;

                case 2: //load PWM 2 time value
                    GLOBAL_PWM2_PulseTime = Value * 10;
                    break;

                //No default
            }
        }
    }
}    
    



//*********************************************
//test GPIO states 

void    TestGPIO(void)
{
    unsigned char GPIO_Status;
    unsigned char OldGPIO_Status;
    unsigned int Status = 0;
    unsigned int Value;
    
    //test message for GPIO
    SendMessage(TestGPIO_Message);
    
    //get the GPIO status
    GPIO_Status = GetGPIO_Status();
    //display the GPIO status
    DisplayGPIO_Status(GPIO_Status);
    //save the latest value of the GPIO status
    OldGPIO_Status = GPIO_Status;
    
    //enter main loop
    while(Status == 0)
    {
        //get any characters
        Value = GetChar();
        //test for any characters received
        if(Value == NO_DATA)
        {
            //get the GPIO status
            GPIO_Status = GetGPIO_Status();
            //test for difference
            if(GPIO_Status != OldGPIO_Status)
            {
                //display the GPIO status
                DisplayGPIO_Status(GPIO_Status);
                //save the latest value of the GPIO status
                OldGPIO_Status = GPIO_Status;
            }
        }
        else
        {
            //exit request
            Status = 1;
        }
    }
}



//*********************************************
//test absolute encoder and display angular information 

void    TestAbsoluteEncoder(void)
{
    unsigned char GPIO_Status;
    unsigned char OldGPIO_Status;
    unsigned int Status = 0;
    unsigned int Value;
    
    //test message for absolute encoder
    SendMessage(TestAbsoluteEncoderMessage);
    
    //get the GPIO status
    GPIO_Status = GetGPIO_Status();
    //display the angular position
    DisplayAbsoluteEncoderPosition(GPIO_Status);
    //save the latest value of the GPIO status
    OldGPIO_Status = GPIO_Status;
    
    //enter main loop
    while(Status == 0)
    {
        //get any characters
        Value = GetChar();
        //test for any characters received
        if(Value == NO_DATA)
        {
            //get the GPIO status
            GPIO_Status = GetGPIO_Status();
            //test for difference
            if(GPIO_Status != OldGPIO_Status)
            {
                //display the absolute angular position
                DisplayAbsoluteEncoderPosition(GPIO_Status);
                //save the latest value of the GPIO status
                OldGPIO_Status = GPIO_Status;
            }
        }
        else
        {
            //exit request
            Status = 1;
        }
    }
}




//*********************************************
//get the status of all 8 bits of the GPIO and return a byte value 

unsigned char   GetGPIO_Status(void)
{
    unsigned char GPIO_Status = 0;
    
    //test each GPIO bit
    if(GPIO_1_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00000001;
    }
    if(GPIO_2_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00000010;
    }
    if(GPIO_3_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00000100;
    }
    if(GPIO_4_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00001000;
    }
    if(GPIO_5_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00010000;
    }
    if(GPIO_6_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b00100000;
    }
    if(GPIO_7_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b01000000;
    }
    if(GPIO_8_READ_PORT == 1)
    {
        GPIO_Status = GPIO_Status | 0b10000000;
    }
    
    //return value
    return GPIO_Status;
}
              


//*********************************************
//display GPIO status

void    DisplayGPIO_Status(unsigned char GPIO_Status)
{
    SendMessage(CRLF);
    //display and check the status of the GPIO bit
    SendMessage(GPIO_1_Message);
    if(GPIO_Status & 0b00000001)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_2_Message);
    if(GPIO_Status & 0b00000010)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_3_Message);
    if(GPIO_Status & 0b00000100)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_4_Message);
    if(GPIO_Status & 0b00001000)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_5_Message);
    if(GPIO_Status & 0b00010000)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_6_Message);
    if(GPIO_Status & 0b00100000)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_7_Message);
    if(GPIO_Status & 0b01000000)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
    SendMessage(GPIO_8_Message);
    if(GPIO_Status & 0b10000000)
    {
        SendMessage(GPIO_ON_Message);
    }
    else
    {
        SendMessage(GPIO_OFF_Message);
    }
}



//*********************************************
//display the absolute angular position from the gray code value passed

void    DisplayAbsoluteEncoderPosition(unsigned char GPIO_Status)
{
    GPIO_Status = GPIO_Status & 0b00001111;
    SendMessage(CRLF);
    switch(GPIO_Status)
    {
        case 0: //0 degrees
            SendMessage(Position0);
            break;
            
        case 1: //22.5 degrees
            SendMessage(Position1);
            break;
            
        case 2: //67.5 degrees
            SendMessage(Position3);
            break;
            
        case 3: //45 degrees
            SendMessage(Position2);
            break;
            
        case 4: //157.5 degrees
            SendMessage(Position7);
            break;
            
        case 5: //135 degrees
            SendMessage(Position6);
            break;

        case 6: //90 degrees
            SendMessage(Position4);
            break;
            
        case 7: //112.5 degrees
            SendMessage(Position5);
            break;
            
        case 8: //337.5 degrees
            SendMessage(Position15);
            break;
            
        case 9: //315 degrees
            SendMessage(Position14);
            break;
            
        case 10: //270 degrees
            SendMessage(Position12);
            break;
            
        case 11: //292.5 degrees
            SendMessage(Position13);
            break;
            
        case 12: //180 degrees
            SendMessage(Position8);
            break;
            
        case 13: //202.5 degrees
            SendMessage(Position9);
            break;
            
        case 14: //247.5 degrees
            SendMessage(Position11);
            break;
            
        case 15: //225 degrees
            SendMessage(Position10);
            break;
    }
    
    //add degrees string
    SendMessage(Degrees);
}


//*********************************************
//option list
void    DisplaySystemOptionsList(void)
{
    //list the user options
    SendMessage(OptionMessage);
    SendMessage(OptionMessage1);
    SendMessage(OptionMessage2);
    SendMessage(OptionMessage3);
    SendMessage(OptionMessage4);
    SendMessage(OptionMessage5);
    SendMessage(OptionMessage6);
    SendMessage(OptionMessage7);
    SendMessage(OptionMessage8);
    SendMessage(OptionMessage9);
    SendMessage(OptionMessage10);
    SendMessage(OptionMessage11);
    SendMessage(OptionMessage12);
    SendMessage(OptionMessage13);
    SendMessage(OptionMessage14);
    SendMessage(OptionMessage15);
    SendMessage(OptionMessage16);
    SendMessage(OptionMessage17);
    SendMessage(OptionMessage18);
    SendMessage(OptionMessage19);
    SendMessage(OptionSelectMessage);
}



//*********************************************
//display Stepper motor drive options list

void    DisplayStepperMotorOptionsList(unsigned int MotorType)
{
    //list the user options
    switch(MotorType)
    {
        case STEPPER_MOTOR: //stepper motor test selected
            SendMessage(StepperOptionMessage);
            break;
            
        case DC_MOTOR:  //DC motor test selected
            SendMessage(DC_MotorOptionMessage);
            break;
            
        //no default
    }
    SendMessage(MotorOptionMessage1);
    SendMessage(MotorOptionMessage2);
    SendMessage(MotorOptionMessage3);
    SendMessage(MotorOptionMessage4);
    SendMessage(OptionSelectMessage);
}



//*********************************************
//display DC motor drive options list

void    DisplayDC_MotorOptionsList(unsigned int MotorType)
{
    //list the user options
    switch(MotorType)
    {
        case STEPPER_MOTOR: //stepper motor test selected
            SendMessage(StepperOptionMessage);
            break;
            
        case DC_MOTOR:  //DC motor test selected
            SendMessage(DC_MotorOptionMessage);
            break;
            
        //no default
    }
    SendMessage(MotorOptionMessage1);
    SendMessage(MotorOptionMessage2a);
    SendMessage(MotorOptionMessage3);
    SendMessage(MotorOptionMessage4);
    SendMessage(OptionSelectMessage);
}


//*********************************************
//display stepper motor status

void    DisplayStepperMotorStatus(void)
{
    //send status header message
    SendMessage(StepperMotorStatusMessage);
    
    //send motor direction information
    SendMessage(StepperMotorStatusMessage1);
    if(GLOBAL_DirectionStatus == 0)
    {
        SendMessage(Clockwise);
    }
    else
    {
        SendMessage(AntiClockwise);
    }

    //send motor speed information
    SendMessage(StepperMotorStatusMessage2);
    //convert the integer value into a string
    DecimalToResultString(GLOBAL_StepperMotorSpeed, GLOBAL_ResultString, 4);
    //display the result
    SendString(GLOBAL_ResultString);
}



//*********************************************
//display stepper motor status

void    DisplayDC_MotorStatus(unsigned int PWM_Number)
{
    unsigned int Value;
    
    //send status header message
    SendMessage(DC_MotorStatusMessage);
    
    //send motor direction information
    SendMessage(DC_MotorStatusMessage1);
    if(GLOBAL_DirectionStatus == 0)
    {
        SendMessage(Clockwise);
    }
    else
    {
        SendMessage(AntiClockwise);
    }

    //send motor speed information
    SendMessage(DC_MotorStatusMessage2);
    //convert the selected integer value into a string
    switch(PWM_Number)
    {
        case 1:
            Value = GLOBAL_PWM1_PulseTime / 10;
            DecimalToResultString(Value, GLOBAL_ResultString, 2);
            break;
            
        case 2:
            Value = GLOBAL_PWM2_PulseTime / 10;
            DecimalToResultString(Value, GLOBAL_ResultString, 2);
            break;
            
        //no default    
    }
    SendString(GLOBAL_ResultString);
}




//*********************************************
//display string error

void    DisplayStringError(unsigned int ErrorValue)
{
    switch(ErrorValue)
    {
        case TOO_LONG:  //string is too long
            SendMessage(MessageTooLong);
            break;
            
        case NO_DATA:  //string has no data
            SendMessage(MessageNoValue);
            break;
            
        case INVALID_STRING:  //string has too many decimal points
            SendMessage(TooManyDecimalPoints);
            break;
            
        case VALUE_TOO_LARGE:  //string exceeds maximum value
            SendMessage(TooLarge);
            break;
            
        case VALUE_TOO_SMALL:  //string exceeds minimum value
            SendMessage(TooSmall);
            break;
            
        //No default
    }
}

