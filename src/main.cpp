/*
 * Created on Sat Oct 19, 2024
 *
 * Copy Right (C) | 2024 Embedded Knowledge
 *
 * Author: Matthew Buchkowski
---------------------------------------------------- */


#include <avr/io.h>
#include "main.h"
#include <util/delay.h>
#include "Arduino.h"
#include <avr/interrupt.h>
#include <math.h>


//Data line is connected to PORTD
//RS, R/W and EN are connected on PORTB

/*
  TODO:

  RS pin needs to be used to toggle between sending Commands or Data to the LCD screen. We will need to configure this in Data mode
  R/W pin toggles between the LCD either Reading data coming to it, or Writing data away from it. We need to configure this in Read mode for displaying
  Enable Pin is used to enable to R/W operation, which ever one it is. This one must always be held high. 


  RS ---> Pin 10, or PB2
  R/W ---> Pin 9, or PB1
  EN ---> Pin 8, or PB0

  PD0 - PD7  => D0 - D7
*/

#define displayMode 0b00001000 | displayOn | cursorOn | blinkOn
#define functionSet 0b00100000 | dataLength8bit | displayLinesTwo | font10Dots


#define RS PORTB2
#define EN PORTB0
#define RW PORTB1



int reg1 = 0x79;
int reg2 = 0x78;

void setup(){
  DDRC &= ~(1 << DDC5);

  DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB0);
  DDRD = 0b11111111;
  PORTB = 0x00;  
}



int main(void){
  setup(); //setups ATMega328P registers for use
  setupADC(); //setups ADC

  initLCD(); //initalizes LCD screen
  
  int counter = 1;
  while(1){ //this loop will poll thermistor, read back voltage, and pass into the printing function the calculated temperature of the thermistor.
    float ans = grabAns();
    Serial.println(calcTemperature(ans));
    printTemperature(calcTemperature(ans));

  }
}
void printTemperature(float temperature){
  int asciiMAP[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};//This is a ASCII map of 0-9 for printing the values to LCD screen.
  int tempCh = temperature * 100; // changing number from float to 4 digit int.
  char tempString[3] = "00"; //truncating to 2 digit temp reading.
  
  int stringLength = sizeof(tempString);

  while(tempCh != 0){ //This while loop loops backwards through the temperature integer R->L and replaces the corresponding char in tempString.
        int tmp = (tempCh % 10);
        if (tempString[stringLength] == '.'){
          tempString[stringLength] = '.';
        }
        tempString[stringLength] = (char)asciiMAP[tmp];
        tempCh = (tempCh - tmp) / 10;
        stringLength -= 1;
        
    }
    
    for(int i=0; i<sizeof(tempString)-1; i++){ //Taking tempString, which is the stringified temperature, and passing ascii value to lcd screen.
      int j = tempString[i] - '0';
      send_character(asciiMAP[j]);
    }
    send_character(0x43); //sending ascii  of "C", for celcius reading.
  _delay_ms(100);
  send_command(clearLCD);
  send_command(returnHome);
  

}

void send_command(unsigned int command){ //configured the rightSelect and Enable registers for sending over command to configure LCD screen. Order matters here.
  
  PORTB &= ~(1 << RS);
  PORTB &= ~(1 << EN);
  
  PORTD = 0b11111111;
  _delay_ms(10);
  PORTD = command;
  PORTB |= (1 << EN);
  _delay_us(1);
  PORTB &= ~(1 << EN);
}

void send_character(unsigned int character){//configured the rightSelect and Enable registers for sending over character to display LCD screen. Order matters here.
  PORTB |= (1 << RS);
  PORTB &= ~(1 << EN);
  
  PORTD = 0b11111111;
  _delay_ms(10);
  PORTD = character;
  PORTB |= (1 << EN);
  _delay_us(1);
  PORTB &= ~(1 << EN);
}

void initLCD(){
  /*
  This function configured the LCD into a specific mode for writing messages for displayment. That being, the functional set and bit mode. The macros are described
  above before main flow of program in the #defined section
  */
  send_command(lcd_FunctionReset);
  _delay_ms(10);
  send_command(lcd_FunctionReset);
  _delay_us(200);
  send_command(lcd_FunctionReset);
  _delay_us(200);


  send_command(clearLCD);
  send_command(returnHome);
  send_command(functionSet); 
  send_command(displayMode);
  send_command(entryModeLR);

}

void startTempConversion(){//Starts the conversion of voltage values into the ADCSRA register
  ADCSRA |= _BV(ADSC);
}

void setupADC(){ //Configures the ADC for specific capturing mode, prescaler for which intervals to capture vales, and which ADC to use. 
  ADMUX = 0b00000101;
  ADCSRA = _BV(ADEN) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2) | _BV(ADATE);
  ADCSRB = 0x00;
  DIDR0 |= (1 << ADC5D);

  //The actual filling of the ADC register starts during this 'startTempConversion' function
  startTempConversion();

}

float grabAns(){
  /*
  Grabs the answer from the ADCL and ADCH register, 16-bit resolution, converts into a voltage value, and scales between 0-5V.
  Since the polling is really fast, an average between 5 measured values is performed before returning the avg.
  */
  float avg = 0.0;
  for(int i=0; i < 5; i++)
  { 
  
    uint8_t low  = ADCL;
    uint8_t high = ADCH;


    uint16_t adc = (high << 8) | low;		// 0<= result <=1023
    float adcVoltage = ((5.0 * adc) / 1023.0);
    avg += adcVoltage;
  }
 
  return (avg) / 5.0;

}

float currResistance(float currVoltage, float refVoltage, float thermistorResistance){ //Calculates the real time resistance of the 10k thermistor for calculation later
  float currRes = currVoltage / ( (refVoltage - currVoltage) / thermistorResistance);
  return currRes;
}

float calcTemperature(float sensedVoltage){ //Calculates the current temperature of a 10k thermistor using appropriate Stein-Hart coefficients. 
  float A = 0.002108508;
  float B = 0.000079792;
  float C = 0.000000653507;

  float currRes = currResistance(sensedVoltage, 5.0, 10000.0);
  
  float tempVal = A + B*log(currRes) + C * pow(log(currRes), 3);
  return (1/tempVal - 273.15);
}

