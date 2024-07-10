// ========================================================================================
// PLL Arduino Code
// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// This code is only for the demonstration of PLL working principle, 
//    since the Atmega328p has no enough performance to run sophisticated PLL algorithms, 
//    like we see in the grid-connected inverter control.
//  * run on Arduino Lab System - Module 1 (Arduino UNO - Atmega328p) 
//  * Sample rate = 4kHz ==> 66.67 samples per period (fundamental at 60Hz)
// PLL calculations are processed by the ADC ISR at 4kHz:
//            -> ISR is invoked when a new sample is ready
//            -> samples are converted to per unit value => pu=Volt/Vbase : pu=[(digital word)*gain + offset]/Vbase
//            -> PI calculations are performed
// * The Vd signal is sent to PWM -> OC1A -> PB1 
//     Vd is the voltage controlled oscillator (VCO) signal synchronized with input signal
// * This simplified PLL algorithm leads to a frequency oscillation around the equilibrium point, 
//     then a 1st order low-pass filter is used to show the mean frequency into LCD:
//                                 0.00157  
//  1st order digital LPF G(z)=  ------------  cut-off frequency=0.12Hz, sample rate=4000Hz
//                                z - 0.99843
//  Note: The use of a LPF into PLL control loop is not recommended, 
//        since it has implies a poor dynamic performance or instability.
// ========================================================================================
//
#include <math.h>
#include <Wire.h>
#include <Keypad_I2C.h>        // Keypad libs
#include <Keypad.h>
#define I2CADDR 0x21           // Keypad i2c expansor address (PCF8574)
#include <LiquidCrystal_I2C.h> //LCD I2C Library
#define LCD_col 20 // defines the number of columns of the used display
#define LCD_lin  4 // defines the number of rows of the used display
#define LDCi2c_Address  0x27 // set the i2c address of the display

// ============ Config kepad ====================================
const byte ROWS = 4; //number of columns of keypad
const byte COLS = 4; //number of rows of keypad
char keys[ROWS][COLS] = { //key specification 
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// keypad rows and columns config, bit numbers of PCF8574 i/o port
byte rowPins[ROWS] = {4, 5, 6, 7}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {0, 1, 2, 3}; //connect to the column pinouts of the keypad

TwoWire *jwire = &Wire;   // passing pointer to keypad lib
Keypad_I2C kpd( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR, PCF8574, jwire );

// ============ Config LCD ====================================
LiquidCrystal_I2C lcd(LDCi2c_Address,LCD_col,LCD_lin);

//-> Sine Table - Sometimes we see strategies that consider the 1/4 cycle symmetry in order to save memory,
//   but here we try to avoid the time to process which quadrant is being considered.
//-> The Sine Table has 200 points (800 bytes, 39% of Data RAM), which is more than necessary, considering 
//   the sampling rate of 4 kHz and a fundamental frequency around 60Hz (66.7 points per period).
float Sin_Table[200]={ // Float Sine Table -> 200 points: 0 to 199/200*2*pi
0.000000, 0.031411, 0.062791, 0.094108, 0.125333, 0.156434, 0.187381, 0.218143, 0.248690, 0.278991, 
0.309017, 0.338738, 0.368125, 0.397148, 0.425779, 0.453990, 0.481754, 0.509041, 0.535827, 0.562083, 
0.587785, 0.612907, 0.637424, 0.661312, 0.684547, 0.707107, 0.728969, 0.750111, 0.770513, 0.790155, 
0.809017, 0.827081, 0.844328, 0.860742, 0.876307, 0.891007, 0.904827, 0.917755, 0.929776, 0.940881, 
0.951057, 0.960294, 0.968583, 0.975917, 0.982287, 0.987688, 0.992115, 0.995562, 0.998027, 0.999507, 
1.000000, 0.999507, 0.998027, 0.995562, 0.992115, 0.987688, 0.982287, 0.975917, 0.968583, 0.960294, 
0.951057, 0.940881, 0.929776, 0.917755, 0.904827, 0.891007, 0.876307, 0.860742, 0.844328, 0.827081, 
0.809017, 0.790155, 0.770513, 0.750111, 0.728969, 0.707107, 0.684547, 0.661312, 0.637424, 0.612907, 
0.587785, 0.562083, 0.535827, 0.509041, 0.481754, 0.453990, 0.425779, 0.397148, 0.368125, 0.338738, 
0.309017, 0.278991, 0.248690, 0.218143, 0.187381, 0.156434, 0.125333, 0.094108, 0.062791, 0.031411, 
0.000000, -0.031411, -0.062791, -0.094108, -0.125333, -0.156434, -0.187381, -0.218143, -0.248690, -0.278991, 
-0.309017, -0.338738, -0.368125, -0.397148, -0.425779, -0.453990, -0.481754, -0.509041, -0.535827, -0.562083, 
-0.587785, -0.612907, -0.637424, -0.661312, -0.684547, -0.707107, -0.728969, -0.750111, -0.770513, -0.790155, 
-0.809017, -0.827081, -0.844328, -0.860742, -0.876307, -0.891007, -0.904827, -0.917755, -0.929776, -0.940881, 
-0.951057, -0.960294, -0.968583, -0.975917, -0.982287, -0.987688, -0.992115, -0.995562, -0.998027, -0.999507, 
-1.000000, -0.999507, -0.998027, -0.995562, -0.992115, -0.987688, -0.982287, -0.975917, -0.968583, -0.960294, 
-0.951057, -0.940881, -0.929776, -0.917755, -0.904827, -0.891007, -0.876307, -0.860742, -0.844328, -0.827081, 
-0.809017, -0.790155, -0.770513, -0.750111, -0.728969, -0.707107, -0.684547, -0.661312, -0.637424, -0.612907, 
-0.587785, -0.562083, -0.535827, -0.509041, -0.481754, -0.453990, -0.425779, -0.397148, -0.368125, -0.338738, 
-0.309017, -0.278991, -0.248690, -0.218143, -0.187381, -0.156434, -0.125333, -0.094108, -0.062791, -0.031411}; 

int interrupt_counter; // ADC interrupt counter 
volatile int print_LCD;      // trigger signal to update LCD at each second
volatile int control;        // 1-turn on/0-turn off PLL control loop
int index;      // Sine table index
float A_1;      // 1st order LPF coefficients (denominator) 
float B_1;      // 1st order LPF coefficients (numerator) 
int sample;     // digital word - acquisition result
float fsample;  // sample in float
float sample_pu;// voltage sample in per unit value
float gain, offset; //coefficients to convert voltage sample in Volts
float b0, b1; // 'b' coefficients of the PI discrete function of PLL loop -> considering unitary amplitude => V/220*sqrt(2)
float Ts;     // sampling period
float wo;     // fundamental frequency
float wpll, fpll; // PLL frequency in rad/s and Hz
float wavg, wavgk_1;  //average frequency (LCD), current and one sampling step back 
float dw_k, dw_k_1;     // PI output: PLL frequency deviation - current and one sampling step back
float Epll_k, Epll_k_1; // PI input: PLL phase detector Error - current and one sampling step back
float Vd, Vq;       //Direct and quadrature PLL voltage
float phase, angle; //PLL phase in rad - normalized phase 0 to 199 (200 points)
char quantity1[10], quantity2[15]; //variables used in formatting the frequency value for writing on the LCD


void setup() {
  
  PORTB = 0x00; // reset PORTB output data
  DDRB |= 0x23; // 0010 0011 -> set PB5 (Onboad LED), PB1 (OC1A) and PB0 to output 
  //------- inicializations ---------
  // -> ADC Calibration to convert the grid voltage sample to per unit value (see https://ernane-aac.github.io/RMS-Voltage-Measurement/)
  gain   = 0.84436/311.0; // gain of the voltage signal conditioning system/nominal voltage amplitude (to get unitary amplitude)
  offset = -432.76/311.0; // Offset of the voltage signal conditioning system/nominal voltage amplitude
  // -> PI compensator  Uk = Uk-1 + b0*Ek + b1*Ek-1 ('zoh' discretization)
  b0 = 92.0;     // discrete PI coefficients - sample rate of 4kHz 
  b1 = -90.94;   // this values implies a PLL unitary amplitude input -> Vinput=Vgrid/nominal amplitude=Vgrid/(220*sqrt(2))
  Ts= 1/4000.0f; // sampling period
  wo = TWO_PI * 60.0; // fundamental frequency
  wpll=wo;            // starting frequency of PLL
  fpll= 60.0;         // initial frequency printed on LCD
  dw_k_1 = 0;         // initial PLL frequency deviation - one sampling step back
  Epll_k_1 = 0;       // initial PLL phase detector Error - one sampling step back  
  wavg = wo;          // inital value for average frequency for current average (in case of the control loop is off)
  wavgk_1 = wo;       // inital value for average frequency - one sampling step back
  A_1 = -0.99843f;    // 1st order IIR LPF coefficients 
  B_1 =  0.00157f,    // sample rate 400Hz - cut-ott frequency 0.12Hz
  phase = 0;     // initial phase of VCO (Voltage control oscillator)
  interrupt_counter = 0;
  control = 0; // PLL control loop starts turned off
  print_LCD = 0;

  jwire->begin( );
  kpd.begin( );     // keypad starting
  lcd.begin();      // LCD starting
  lcd.backlight();
  Config_ADC();    // ADC configuration
  Timer1_Init();   // Timer 1 initialization
  

  lcd.setCursor(0,0);  //set cursor to line 1, column 0          
  //         01234567890123456789 
  lcd.print("PLL A-Run B-Stop");
  lcd.setCursor(0,1);  //set cursor to line 2, column 0  
  lcd.print("Sample rate 4kHz"); 
  lcd.setCursor(20,0); //set cursor to line 3, column 0 
  lcd.print("PLL Frequency:  ");
  lcd.setCursor(20,1); //set cursor to line 4, column 14  
           //01234567890123456789
  lcd.print("             PLL OFF"); 
}

void loop() {
  char key=0;
  
  if(print_LCD==1)
     {
      print_LCD=0;
      fpll = wavg/TWO_PI; //calculates the average frequency in Hz
      dtostrf(fpll, 6, 2,quantity1);
      sprintf(quantity2,"%6s Hz", quantity1); 
      lcd.setCursor(20,1); // set cursor to line 4, column 1 
      lcd.print(quantity2); //print frequency on LCD
  }

  key = kpd.getKey(); 
  if (key != 0)
  {
    if (key=='A')      //turn on PLL
          {
            control=1; //PLL control loop is turned on
            lcd.setCursor(33,1); // set cursor to line 4, column 14  
        //01234567890123456789
            lcd.print("PLL ON ");
          }   
    if (key=='B')      //turn off PLL
          {
            control=0; //PLL control loop is turned off
            lcd.setCursor(33,1); // set cursor to line 4, column 14  
        //01234567890123456789
            lcd.print("PLL OFF");            
          } 
  }
}

//====================================================================================
// ========== ADC interrupt service  ===============
ISR(ADC_vect)
{
  PINB |=0x01;    //toggle PB0 - used to measure the duration of ISR (real time check) 
  //--------------------------------------------------
  //TIFR1 – Timer/Counter1 Interrupt Flag Register
  //Bit         7  6    5   4  3    2     1     0
  //0x16 (0x36) –  –  ICF1  –  –  OCF1B OCF1A TOV1 
  //Read/Write  R  R   R/W  R  R   R/W   R/W   R/W
  //Init. Val.  0  0    0   0  0    0     0     0
  //--------------------------------------------------     
  TIFR1 |= 0x01; // Write "1" -> Reset Bit-0 TOV1 (Timer1 Overflow Flag) for triggering the first acquisition  
                 // Timer overflow generates SOC trigger for ADC on rising edge
                 // As there is no timer interrupt, there is no automatic reset of the flag
                 // Therefore, it is necessary to reset via software to trigger a new acquisition.

  sample=ADC;                             // reads new digital sample
  sample_pu=gain*(float)sample + offset;  // convert sample to per unit value
  wpll = wavg;    // keeps the PLL over the last average frequency in case of the control loop is off
  if (control==1)
     { 
      Epll_k= sample_pu*Vq; // Error from PLL detector - current sample
      dw_k = dw_k_1 + b0*Epll_k + b1*Epll_k_1; //calculates new frequency deviation (PI compensator)
      Epll_k_1 = Epll_k;   // update PLL detector error - one time step back
      dw_k_1 = dw_k;       // update PLL frequency deviation - one time step back
      wpll=wo+dw_k;        // calculates the new PLL frequency 
     }

  phase = phase + wpll*Ts; // frequency integration
  if(phase >= TWO_PI) phase = phase-TWO_PI; //anti-windup
  angle = phase*200/TWO_PI;
  index = (int) angle;
  Vd = Sin_Table[index];   // Vd=sin(phase); Vd amplitude = 1
  index = index + 50;      // phase advance in 90 degrees (fifty points in sine table) => cos(x)=sin(x+90)
  if (index >= 200) index= index - 200; //circular buffer adjust
  Vq = Sin_Table[index];   // Vd=cos(phase);
  fsample=(Vd*1990) + 2000;// convert Vd amplitude to digital compare match PWM1
  sample=(int)fsample;     // convert sample to integer value
  OCR1A=sample;            // load sample into Output Compare Register (PWM1)

  wavg = B_1*wpll - A_1*wavgk_1 ; // run 1st order filter do calculate PLL average frequency (phase advance - input sample)
  wavgk_1=wavg;  

  interrupt_counter++;
  if(interrupt_counter>=4000)
     {
       interrupt_counter=0;
       PINB |=0x20;            // blink onboad LED -> 0.5 Hz
       print_LCD=1;            // trigger to update the LCD
     }  
                 
  PINB |=0x01;   //toggle PB0 - used to measure the duration of ISR on oscilloscope connected at pin PB0(real time check)            
}


//====================================================================================
void Config_ADC(void)
{
 //----------------------------------------------------
 //ADMUX – ADC Multiplexer Selection Register
 //Bit         7     6     5    4    3    2    1    0
 //(0x7C)    REFS1 REFS0 ADLAR  –  MUX3 MUX2 MUX1 MUX0 
 //Read/Write R/W   R/W   R/W   R   R/W  R/W  R/W  R/W
 //Init. Val.  0     0     0    0    0    0    0    0
 //This code   0     1     0    0    0    0    0    0   -> Vref=AVCC*; result right adjusted; Mux-> Channel A0   
 //-----------------------------------------------------   * AVCC on Arduino-UNO board is connected to 5V
 //                                                     -> for reference stability, the use of an external power supply (7 to 12V)
                                                         // with a 5V level defined by the internal regulator is better than the power via USB                                             
 ADMUX=0x40;                                             // consider this in calibration process

 //----------------------------------------------------
 //ADCSRA – ADC Control and Status Register A
 //Bit         7    6    5     4    3    2     1     0
 //(0x7A)    ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0 
 //Read/Write R/W  R/W  R/W   R/W  R/W  R/W   R/W   R/W
 //Init. Val.  0    0    0     0    0    0     0     0
 //This code   1    0    1     0    1    1     1     1  -> enable ADC; Auto trigger; enable interrupt; ADC_Clk=125kHz
 //---------------------------------------------------- -> ADC_Clk affects the precision. Atmega328P datasheet recommends ADC_Clk<200kHz
 //                                                     -> In auto-trigger -> sample time: 2 cycles, total conversion time 13 cycles (13.5/125kHz=104us)
 //----------------------------------------------------    
 ADCSRA=0xAF;                                             

 //----------------------------------------------------
 //ADCSRB – ADC Control and Status Register B
 //Bit        7    6   5  4  3    2     1     0
 //(0x7B)     –  ACME  –  –  –  ADTS2 ADTS1 ADTS0      -> Analog Comparator Multiplexer Enable=0; not used in this application
 //Read/Write R   R/W  R  R  R   R/W   R/W   R/W
 //Init. Val. 0    0   0  0  0    0     0     0
 //This code  0    0   0  0  0    1     1     0        -> Trigger-> Timer/Counter1 overflow
 //----------------------------------------------------
  ADCSRB=0x06;

 //----------------------------------------------------
 //DIDR0 – Digital Input Disable Register 0
 //Bit        7  6    5     4     3     2     1     0
 //(0x7E)     –  –  ADC5D ADC4D ADC3D ADC2D ADC1D ADC0D 
 //Read/Write R  R   R/W   R/W   R/W   R/W   R/W   R/W
 //Init. Val. 0  0    0     0     0     0     0     0
 //This Code  0  0    0     0     0     0     0     1 -> disable digital input PC0 -> channel A0 
//----------------------------------------------------
 DIDR0=0x01;
}

//====================================================================================
void Timer1_Init(void)
{              
 //--------------------------------------------------------------- 
 //TCCR1A - Timer/Counter1 Control Register A
 //Bit        7      6      5      4     3     2     1      0
 //(0x80) COM1A1 COM1A0 COM1B1 COM1B0    –     –  WGM11  WGM10 
 //Init. Val. 0      0      0      0     0     0     0      0
 //This code  1      0      0      0     0     0     1      0   -> OC1A->clear on compare match -set on bottom /OC1B disconnected.
 //------------------------------------------------------------ -> Waveform generation Mode=14 Fast PWM  -> WGM=14 ->  WGM11:WGM10=10 
 TCCR1A=0x82;     
 //---------------------------------------------------------------           
 //TCCR1B – Timer/Counter1 Control Register B
 //Bit       7     6     5     4     3     2     1     0
 //(0x81) ICNC1 ICES1    –  WGM13 WGM12  CS12  CS11  CS10 
 //Init. Val. 0      0   0    0     0     0     0      0
 //This code  0      0   0    1     1     0     0      1   -> Waveform generation Mode=14 WGM=14 ->  WGM13:WGM12=11; clk_timer=clk_cpu/1=16MHz 
 //---------------------------------------------------------------  
 TCCR1B=0x19;
                           
 //->Timer cycle= Timer_CLK/(4000 steps)=16MHz/4000=4000 Hz 
 //-> 4000 steps in counting UP -> TOP=3999 
 ICR1=3999; //config TOP
 OCR1A=2000; //compare in 50%
         
 //---------------------------------------------------------------
 //TIMSK1 – Timer/Counter1 Interrupt Mask Register
 //Bit        7    6    5    4    3      2      1     0
 //(0x6F)     –    – ICIE1   –    –  OCIE1B OCIE1A  TOIE1 
 //Read/Write R    R   R/W   R    R     R/W    R/W   R/W
 //Init. Val. 0    0    0    0    0      0      0     0      -> timer 1 interrupt control
 //--------------------------------------------------------------- 
 TIMSK1=0x00;  //no timer 1 interrupt is used -> only the trigger for the ADC
 TCNT1=0;  //reset timer1 counter
 TIFR1 |= 0x01; // Write "1" -> Reset Bit-0 TOV1 (Timer1 Overflow Flag) for triggering the first acquisition  
                 // Timer overflow generates SOC trigger for ADC on rising edge
                 // As there is no timer interrupt, there is no automatic reset of the flag
                 // Therefore, it is necessary to reset via software to trigger a new acquisition.
} 

