/* @Title: Transmission line control system 

   @Author: Jake Belmont
   @Author: Cooper Dulesky

   @Brief: Program will utilize the ADS131MO8 ADC to measure 4 voltages and 3 currents from a transmission line system
   The program will us this data to display bus voltages, bus currents, system powerfactor, and real power to an external GUI.
   the program will control disconnects that will be used to apply and remove power from busses within the system. The program 
   also allows the user to induce 3 fault types, enable powerfactor correction, enable and disable a motor, and reset the system.
*/

#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>
#include <SPI.h>
#include <Arduino.h>
#include "ADS131M08.h"
#include <Wire.h>

//ADC object
ADS131M08 adc;

//GPIO expander object
Adafruit_MCP23X17 mcp;

//GPIO expander adderess
#define GPIO_ExpanderAdderess 0x20

//Custom functions

/* @def: RunAnalysis()

   @Brief: Takes n samples of differential voltages and calculates VRMS and IRMS for repsective channels 
           function also checks for over and under voltage and current events will enable and disable buss disconnects
           function also formats serial output that is parsed by the GUI (IMPORTANT: output must be line by line in order to ensure GUI sends data to the correct positions)
void RunAnalysis();

/* @def: GENTEST()

    @Brief: will measure the voltage at the generator bus over 20 measurments if 10 consecutive measruments fall in selected range the generator will be considered stable and return true
            after 20 cycles with no consecutive stability the generator will be considered unstable and return false 
    
    @Param: Gain[]: Array containing all gains from voltage front end L-Blocks 
    @param: Disconnect_IO[]: An array containing the IO for bus disconnects
    @param: samples: The number of samples being taken (150 is close to minimum for accurate measurments)
    @Param: upperBoundGen: Upper allowable RMS bound for Gen bus voltage
    @Param: lowerBoundGen: Lower allowable RMS bound for Gen bus voltage
*/
bool GENTEST(double Gain[], double NominalGeneratorVoltage, int samples, double upperBoundGen, double lowerBoundGen);

/* @def lim_Check()

   @Brief: Will check the state of the system limitswiches and disable 120v to generator when they detect an open state
   Will hold the user in a loop waiting for them to close it 
*/
void lim_Check();

void systemDiagnostic();

void SafeMode(int faultedChannel);

void FaultEnableCheck();

void PowerFactorEnable();

void MotorStartup();


//IMPORTANT KEEP ALL ARRAYS IN GEN,HV,INDUS,RES order if not including one bus remove it from array and keep order the same

//DUE to board complications currents are now read INDUS,RES,GEN

// 120 Vac Gen enabe I0
int GEN_IO = D7;
//High voltage bus IO
int HV_IO = D4;
//Industrial bus IO
int INDUS_IO = D5;
//residential bus IO
int RES_IO = D6;
//Motor select IO
int Motor_Sel = A1;
//Limit Switch IO
int LimitSW = A0;
// Power factor enable IO
int PF_ENABLE = A2;
// Line Ground fault select switch IO
int LG_FAULT_SEL = A3;
// Line Neautral fault select switch IO
int LN_FAULT_SEL = A4;
// Power factor select switch IO
int PF_SEL = A5;
// Line to line fault select switch IO
int LL_FAULT_SEL = D2;
// Motor enable IO
int Motor_ENABLE = D3;


bool fault_GEN = false;

bool fault_HV = false;

bool fault_INDUS = false;

bool fault_RES = false;

bool systemFault = false;

// IO for GPIO Expander

int LG_FAULT_EN = 0;
int LL_FAULT_EN = 1;
int LN_FAULT_EN = 2;

int GPIO6 = 3;
int GPIO5 = 4;
int GPIO4 = 5;
int GPIO3 = 6;
int GPIO2 = 7;

int RES_DIS_INDICATOR = 15;
int LL_FAULT_INDICATOR = 14;
int LG_FAULT_INDICATOR = 13;
int LN_FAULT_INDICATOR = 12;
int PF_INDICATOR = 11;
int HV_DIS_INDICATOR = 10;
int INDUS_DIS_INDICATOR = 9;

int GPIO1 = 8;

int FaultCNT = 0;

int activeFault = 0;


// Disconnect I/O for each bus
//int Disconnect_IO[4] = { GEN_IO, HV_IO, INDUS_IO, RES_IO };

//Using LG Fault Because of damaged IO on board
int Disconnect_IO[4] = { LG_FAULT_EN, HV_IO, INDUS_IO, RES_IO };

// Number of voltage readings being taken
int VoltageReadings = 4;
//number of current readings being taken
int CurrentReadings = 3;
//number of frames processed
int samples = 300;

//Resistor Dividers for voltage input (TS will need changed)
double RH = 930000.0;
//double RL_GEN = 9200;
double RL_GEN = 8907;
//double RL_HV = 4750;
double RL_HV = 4675;
double RL_INDUS = 6900;
//double RL_RES = 17800;
double RL_RES = 16800;

/*NOMINAL GAIN values for 7 GUYS 1 Goal Project

GEN:102.09
HV:196.79
INDUS:135.79
RES:53.34
*/
double Gain[4] = {
  (RH + RL_GEN) / RL_GEN,
  (RH + RL_HV) / RL_HV,
  ((RH + RL_INDUS) / RL_INDUS),
  (RH + RL_RES) / RL_RES
};

//Turns ratio of CT
double TurnsRatio = 1000;

//Burden resistors for current measurments
//double RB_GEN = 499;
// double RB_GEN = 261;
// //double RB_INDUS = 1500;
// double RB_INDUS = 390;
// //double RB_RES = 470;
// double RB_RES = 248;

//SUPER BOOF FOR GETTING IT TO WORK IN A RUSH
double RB_GEN = 400;
//double RB_INDUS = 1500;
double RB_INDUS = 390;
//double RB_RES = 470;
double RB_RES = 248;

double Burden[3] = { RB_GEN, RB_INDUS, RB_RES };

//Nominal Current readings
// double NOM_GEN_I = 1;
// double NOM_INDUS_I = .6;
// double NOM_RES_I = .33;

double NOM_GEN_I = .6;
double NOM_INDUS_I = .37;
double NOM_RES_I = 1;


double NominalCurrents[3] = { NOM_GEN_I, NOM_INDUS_I, NOM_RES_I };

//Nominal Bus voltages
double NOM_GEN_Unloaded = 72;
double NOM_GEN = 62;
double NOM_HV = 107;
double NOM_INDUS = 63;
double NOM_RES = 37;

//TEST: comment out for real use
// double NOM_GEN = .15;
// double NOM_HV = .15;
// double NOM_INDUS = .15;
// double NOM_RES = .15;

double NominalVoltage[4] = { NOM_GEN, NOM_HV, NOM_INDUS, NOM_RES };

//IN theory lower allowable error will cause furthest bus's from gen to disconnect first
double percentError_GEN = .15;
double percentError_HV = .1;
double percentError_INDUS = .1;
double percentError_RES = .25;

//TEST: comment out for real use
// double percentError_GEN = .25;
// double percentError_HV = .25;
// double percentError_INDUS = .25;
// double percentError_RES = .25;

// Error allowed at each bus
double percentError[4] = { percentError_GEN, percentError_HV, percentError_INDUS, percentError_RES };


//UPPER and LOWER Voltage bounds in VRMS
/*
GEN = +/- 9.3 V
HV = +/- 10.7 V
INDUS +/- 6.3 V
RES: +/- 1.65 V
*/
double upperBound[4] = { (percentError_GEN + 1) * (NOM_GEN), (percentError_HV + 1) * (NOM_HV), (percentError_INDUS + 1) * (NOM_INDUS), (percentError_RES + 1) * (NOM_RES) };
double lowerBound[4] = { (1 - percentError_GEN) * (NOM_GEN), (1 - percentError_HV) * (NOM_HV), (1 - percentError_INDUS) * (NOM_INDUS), (1 - percentError_RES) * (NOM_RES) };

//UPPER and LOWER Voltage bounds in IRMS
/*
GEN = +/- 
HV = +/- 
INDUS +/- 
RES: +/- 
*/
double upperBoundI[3] = { (percentError_GEN + 1) * (NOM_GEN_I), (percentError_INDUS + 1) * (NOM_INDUS_I), (percentError_RES + 1) * (NOM_RES_I) };
double lowerBoundI[3] = { (1 - percentError_GEN) * (NOM_GEN_I), (1 - percentError_INDUS) * (NOM_INDUS_I), (1 - percentError_RES) * (NOM_RES_I) };
//Generator bounds using RMS voltage
double upperBoundGen = (percentError_GEN + 1) * NOM_GEN_Unloaded;
double lowerBoundGen = (1 - percentError_GEN) * NOM_GEN_Unloaded;

//ADD current bounds


void setup() {


  Serial.begin(9600);
  Wire.begin();

  if (!mcp.begin_I2C()) {
    //Warning 1: I2C Failure Reset system
    Serial.println("Warning1");
    while (1)
      ;
  }

  // Digital I/O

  //Motor Enable(BOOF)
  pinMode(Motor_Sel, INPUT);

  //Limit Switch
  pinMode(LimitSW, INPUT);

  //Power Factor Enable
  pinMode(PF_ENABLE, OUTPUT);

  //Line to ground fault select (CURRENTLY USED AS GEN_IO DUE TO DAMAGED BOARD)
  pinMode(LG_FAULT_SEL, INPUT);

  //Line to neutral fault select
  pinMode(LN_FAULT_SEL, INPUT);

  //Power factor select
  pinMode(PF_SEL, INPUT);

  //Line to Line fault select
  pinMode(LL_FAULT_SEL, INPUT);

  //Motor Enable
  pinMode(Motor_ENABLE, OUTPUT);

  //High Voltage Disconnect
  pinMode(HV_IO, OUTPUT);

  //Industrial Disconnect
  pinMode(INDUS_IO, OUTPUT);

  //Residential Disconnect
  pinMode(RES_IO, OUTPUT);

  //120 Disconnect
  pinMode(GEN_IO, OUTPUT);


  //BOOF Clock Pin
  pinMode(D9, OUTPUT);

  // GPIO Expander definitions

  mcp.pinMode(LG_FAULT_EN, OUTPUT);
  mcp.pinMode(LL_FAULT_EN, OUTPUT);
  mcp.pinMode(LN_FAULT_EN, OUTPUT);
  mcp.pinMode(GPIO6, OUTPUT);
  mcp.pinMode(GPIO5, OUTPUT);
  mcp.pinMode(GPIO4, OUTPUT);
  mcp.pinMode(GPIO3, OUTPUT);
  mcp.pinMode(GPIO2, OUTPUT);
  mcp.pinMode(RES_DIS_INDICATOR, OUTPUT);
  mcp.pinMode(LL_FAULT_INDICATOR, OUTPUT);
  mcp.pinMode(LG_FAULT_INDICATOR, OUTPUT);
  mcp.pinMode(LN_FAULT_INDICATOR, OUTPUT);
  mcp.pinMode(PF_INDICATOR, OUTPUT);
  mcp.pinMode(HV_DIS_INDICATOR, OUTPUT);
  mcp.pinMode(INDUS_DIS_INDICATOR, OUTPUT);
  mcp.pinMode(GPIO1, OUTPUT);


  //BOOF Clock directive (UNO Q only)
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
  GPIOB->MODER &= ~(3 << (8 * 2));
  GPIOB->MODER |= (2 << (8 * 2));
  GPIOB->OSPEEDR |= (3 << (8 * 2));
  GPIOB->AFR[1] &= ~(0xF << 0);
  GPIOB->AFR[1] |= (2 << 0);
  TIM4->CR1 = 0;
  TIM4->PSC = 0;
  TIM4->ARR = 19;
  TIM4->CCR3 = 10;
  TIM4->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
  TIM4->CCER |= TIM_CCER_CC3E;
  TIM4->CR1 |= TIM_CR1_CEN;

  /* Define SPI Parameters

      SCKL: 13 1 Mhz
      MISO: 12
      MOSI: 11
        CS: 10 - Unused
    DREADY: 8
  
  */

  adc.begin(13, 12, 11, 10, 8);

  //ADC Configuration Sequence

  adc.SendCommand(CMD_RESET);

  //Enable channels (Channel, 1 / 0)
  delay(50);
  adc.setChannelEnable(0, 1);
  delay(50);
  adc.setChannelEnable(1, 1);
  delay(50);
  adc.setChannelEnable(2, 1);
  delay(50);
  adc.setChannelEnable(3, 1);
  delay(50);
  adc.setChannelEnable(4, 1);
  delay(50);
  adc.setChannelEnable(5, 1);
  delay(50);
  adc.setChannelEnable(6, 1);
  delay(50);
  adc.setChannelEnable(7, 0);
  delay(50);

  //Channel Select: 0 for differential inputs, 2 for DC test signal (Channel, WREG)
  adc.setInputChannelSelection(0, 0x00);
  delay(50);
  adc.setInputChannelSelection(1, 0x00);
  delay(50);
  adc.setInputChannelSelection(2, 0x00);
  delay(50);
  adc.setInputChannelSelection(3, 0x00);
  delay(50);
  adc.setInputChannelSelection(4, 0x00);
  delay(50);
  adc.setInputChannelSelection(5, 0x00);
  delay(50);
  adc.setInputChannelSelection(6, 0x00);
  delay(50);
  //adc.setInputChannelSelection(7, 0x00);

  systemDiagnostic();
  delay(1000);
}

void loop() {


  lim_Check();
  //Check the fault enable switches
  FaultEnableCheck();
  delay(500);
  // Run analysis imiediatly after inducing a fault
  RunAnalysis();
  delay(100);
  //powerFactorAnalysis();
  // delay(500);
  lim_Check();

  //Power factor enable
  // PowerFactorEnable();
  // delay(10);

  //Check if motor needs enabled
  if (digitalRead(Motor_Sel) == HIGH) {
    MotorStartup();
  }
  delay(100);
  lim_Check();
}

void RunAnalysis() {

  // Total Channels being read by ADC
  int total_CHAN = (VoltageReadings + CurrentReadings);

  // Array of squared instantaneous voltages at each channel
  double Squares[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with gain accounted for
  double Vrms[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with no gain applied
  double Vrms_NOGAIN[total_CHAN] = { 0 };
  // The instantaneous voltage measured (one of 150 voltages measured)
  double Instant_V[total_CHAN];
  // The resulting IRMS calculated from channels measuring current
  double irms[CurrentReadings] = { 0 };


  // Reset the sample count to 0 ensuring the loop always fully runs
  int sampleCount = 0;

  // I am sorry for this do not dare mess with this set of loops it will break the chip will start doing weird shit and it will not stop
  // you will need to turn the chip off and on and push the old code to hope to fix it
  // also dont dare try and add or subtract from the arrays without full knoledge of this loop it will break everything
  while (sampleCount < samples) {

    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      for (int i = 0; i < (total_CHAN); i++) {
        Instant_V[i] = res.ch[i].f;

        //Emergency shut off to protect chip not not fast but plus TVS diode is on board but nonetheless good precaution
        if (Instant_V[i] > 1.2) {
          digitalWrite(Disconnect_IO[0], false);
          //   // Error3: Dangerous Voltage Detected: Check Chip Health and reset!
          Serial.println("Error3");
          while (1) {
          }
        }

        Squares[i] += Instant_V[i] * Instant_V[i];
      }

      sampleCount++;

      if (sampleCount >= samples) {

        for (int i = 0; i < (total_CHAN); i++) {
          //index used to get currents

          Vrms[i] = sqrt(Squares[i] / samples) * Gain[i];
          Vrms_NOGAIN[i] = sqrt(Squares[i] / samples);

          // Very Primitive and sensitive system one value out of error range will fault out and requiers a system restart on every fault
          // percent errors should be chosen so that the generator may fluctuate and not cause faults while shorts and major overcurrents fault the system
          // Posible solution is to count number of invalid measurments and fault after x number of out of bounds values are found (Hard and stinky)

          //   if ((Vrms[i] > upperBound[i]) || (Vrms[i] < lowerBound[i])) {

          //     digitalWrite(Disconnect_IO[i], LOW);
          //     systemFault = true;
          //     // Error4: Fault detected on Gen bus
          //     // Error5: Fault detected on HV bus
          //     // Error6: Fault detected on Industrial
          //     // Error7: Fault detected on Residential bus
          //     switch (i) {

          //       case 0:
          //         Serial.println("Error4");
          //         fault_GEN = true;
          //         break;
          //       case 1:
          //         Serial.println("Error5");
          //         fault_HV = true;
          //         break;
          //       case 2:
          //         Serial.println("Error6");
          //         fault_INDUS = true;
          //         break;
          //       case 3:
          //         Serial.println("Error7");
          //         fault_RES = true;
          //         break;
          //     }
          //     if (digitalRead(LN_FAULT_SEL) == true || digitalRead(LL_FAULT_SEL) == true) {

          //       Serial.println("Warning6");
          //       //Warning6: System entering safe mode clear faults to return to normal operation
          //       while (1) {
          //         SafeMode(i);
          //         delay(100);
          //         //Run in safe mode wait for user fault to clear,test,and return to normal operation
          //         if (digitalRead(LN_FAULT_SEL) == true && digitalRead(LL_FAULT_SEL) == true) {

          //           return;
          //         }
          //       }

          //     } else {
          //       //Error 8: Unknown or generator fault occured inspect and restart system!
          //       digitalWrite(LG_FAULT_EN, LOW);
          //       Serial.println("Error8");
          //       while (1) {
          //         //do nothing forever
          //       }
          //     }
          //   }
          // }
        }

        Serial.println("BEGIN");

        for (int i = 0; i < (VoltageReadings); i++) {

          Serial.println(Vrms[i], 4);
          //Serial.println(Vrms_NOGAIN[i+3], 4);
        }


        for (int i = 0; i < CurrentReadings; i++) {

          int adc_index = i + VoltageReadings;

          //The variables in this calculation are soo boofed and were selected to make it work
          //good idea to properly calculate this shit but our burden resistors dont work

          irms[i] = (Vrms_NOGAIN[adc_index] * TurnsRatio) / (2 * (Burden[i]));
          // irms[i] = (Vrms_NOGAIN[adc_index] * TurnsRatio) / ((Burden[i]))

          if (digitalRead(Motor_Sel) == false) {

            if ((irms[i] > upperBoundI[i]) || (irms[i] < lowerBoundI[i])) {

              if (digitalRead(Motor_Sel) == 0 && i == 0) {
                continue;
              }
              if (digitalRead(Motor_Sel) == 1 && i == 1) {
                continue;
              }
              if (digitalRead(Motor_Sel) == 1 && i == 0) {
                continue;
              }


              systemFault = true;
              // Error4: Fault detected on Gen bus
              // Error5: Fault detected on HV bus
              // Error6: Fault detected on Industrial
              // Error7: Fault detected on Residential bus
              switch (i) {

                case 2:
                  Serial.println("Error4");
                  digitalWrite(LG_FAULT_EN, LOW);
                  fault_GEN = true;
                  break;
                case 0:
                  Serial.println("Error6");
                  digitalWrite(INDUS_IO, LOW);
                  fault_INDUS = true;
                  break;
                case 1:
                  Serial.println("Error7");
                  Serial.println(irms[i]);
                  digitalWrite(RES_IO, LOW);
                  fault_RES = true;
                  break;
              }
              if (digitalRead(LN_FAULT_SEL) == true || digitalRead(LL_FAULT_SEL) == true) {
                //Warning6: System entering safe mode clear faults to return to normal operation
                Serial.println("Warning6");

                while (1) {
                  //Serial.println("Fault enabled");
                  // SafeMode(i);
                  delay(100);
                  // powerFactorAnalysis();
                  // delay(10);
                  //Run in safe mode wait for user fault to clear,test,and return to normal operation
                  if (digitalRead(LN_FAULT_SEL) == false && digitalRead(LL_FAULT_SEL) == false) {

                    Serial.println("Fault clear");
                    FaultEnableCheck();

                    if (fault_RES == true) {
                      digitalWrite(RES_IO, HIGH);
                    } else if (fault_GEN == true) {
                      mcp.digitalWrite(LG_FAULT_EN, HIGH);
                    } else {
                      digitalWrite(INDUS_IO, HIGH);
                    }
                    return;
                  }
                }

              } else {
                //Error 8: Unknown or generator fault occured inspect and restart system!
                mcp.digitalWrite(LG_FAULT_EN, LOW);
                Serial.println("Error8");
                while (1) {
                  //do nothing forever
                }
              }
            }
          }
        }

        for (int i = 0; i < (CurrentReadings); i++) {
          Serial.println(irms[i], 4);
          //Serial.println(Vrms_NOGAIN[i + 4], 4);
        }

        //GenPowerAnalysis();
        for (int i = 0; i < (3); i++) {
          Serial.println("N/A");
        }
        return;
      }
    }
  }
}

void RunAnalysisNoFault() {

  // Total Channels being read by ADC
  int total_CHAN = (VoltageReadings + CurrentReadings);

  // Array of squared instantaneous voltages at each channel
  double Squares[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with gain accounted for
  double Vrms[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with no gain applied
  double Vrms_NOGAIN[total_CHAN] = { 0 };
  // The instantaneous voltage measured (one of 150 voltages measured)
  double Instant_V[total_CHAN];
  // The resulting IRMS calculated from channels measuring current
  double irms[CurrentReadings] = { 0 };


  // Reset the sample count to 0 ensuring the loop always fully runs
  int sampleCount = 0;

  // I am sorry for this do not dare mess with this set of loops it will break the chip will start doing weird shit and it will not stop
  // you will need to turn the chip off and on and push the old code to hope to fix it
  // also dont dare try and add or subtract from the arrays without full knoledge of this loop it will break everything
  while (sampleCount < samples) {

    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      for (int i = 0; i < (total_CHAN); i++) {
        Instant_V[i] = res.ch[i].f;

        //Emergency shut off to protect chip not not fast but plus TVS diode is on board but nonetheless good precaution
        if (Instant_V[i] > 1.2) {
          digitalWrite(Disconnect_IO[0], false);
          //   // Error3: Dangerous Voltage Detected: Check Chip Health and reset!
          Serial.println("Error3");
          while (1) {
          }
        }

        Squares[i] += Instant_V[i] * Instant_V[i];
      }

      sampleCount++;

      if (sampleCount >= samples) {

        for (int i = 0; i < (total_CHAN); i++) {
          //index used to get currents

          Vrms[i] = sqrt(Squares[i] / samples) * Gain[i];
          Vrms_NOGAIN[i] = sqrt(Squares[i] / samples);
        }

        Serial.println("BEGIN");

        for (int i = 0; i < (VoltageReadings); i++) {

          Serial.println(Vrms[i], 4);
          //Serial.println(Vrms_NOGAIN[i+3], 4);
        }


        for (int i = 0; i < CurrentReadings; i++) {

          int adc_index = i + VoltageReadings;

          //The variables in this calculation are soo boofed and were selected to make it work
          //good idea to properly calculate this shit but our burden resistors dont work

          irms[i] = (Vrms_NOGAIN[adc_index] * TurnsRatio) / (2 * (Burden[i]));
          // irms[i] = (Vrms_NOGAIN[adc_index] * TurnsRatio) / ((Burden[i]))
        }

        for (int i = 0; i < (CurrentReadings); i++) {
          Serial.println(irms[i], 4);
          //Serial.println(Vrms_NOGAIN[i + 4], 4);
        }

        //GenPowerAnalysis();
        for (int i = 0; i < (3); i++) {
          Serial.println("N/A");
        }
        return;
      }
    }
  }
}

void powerFactorAnalysis() {

  int total_CHAN = (VoltageReadings + CurrentReadings);

  // Array of squared instantaneous voltages at each channel
  double Squares;

  // Array of Vrms values at each channel with gain accounted for
  double Vrms;

  // The instantaneous voltage measured (one of 150 voltages measured)
  double Instant_V = 0;

  double Instant_I = 0;

  double Power = 0;

  double PF = 0;

  double irms = 0;

  double Instant_V_NG = 0;

  double Vrms_NG = 0;

  // Reset the sample count to 0 ensuring the loop always fully runs
  int sampleCount = 0;

  while (sampleCount < samples) {

    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      Instant_V = res.ch[2].f * Gain[2];

      Instant_I = (res.ch[4].f * TurnsRatio) / (2 * (Burden[0]));

      Instant_V_NG = res.ch[2].f;

      double Instant_P = Instant_V * Instant_I;

      double Power = Power + Instant_P;

      Squares += Instant_V_NG * Instant_V_NG;

      sampleCount++;

      if (sampleCount >= samples) {

        Vrms = sqrt(Squares / samples) * Gain[2];

        Vrms_NG = sqrt(Squares / samples);

        irms = (Vrms_NG * TurnsRatio) / (2 * (Burden[0]));

        double PowerAVG = fabs(Power / samples);

        Serial.println(PowerAVG, 4);

        Serial.println(PowerAVG / (Vrms * irms), 3);

        return;
      }
    }
  }
}

void GenPowerAnalysis() {

  int total_CHAN = (VoltageReadings + CurrentReadings);

  // Array of squared instantaneous voltages at each channel
  double Squares;

  // Array of Vrms values at each channel with gain accounted for
  double Vrms;

  // The instantaneous voltage measured (one of 150 voltages measured)
  double Instant_V;

  double Instant_I;

  double Power = 0;

  double PF = 0;

  double irms;

  double Instant_V_NG;

  double Vrms_NG;

  // Reset the sample count to 0 ensuring the loop always fully runs
  int sampleCount = 0;

  while (sampleCount < samples) {

    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      Instant_V = res.ch[0].f * Gain[0];

      Instant_I = (res.ch[6].f * TurnsRatio) / (2 * (Burden[2]));


      double Instant_P = Instant_V * Instant_I;

      double Power = Power + Instant_P;

      sampleCount++;

      if (sampleCount >= samples) {

        double PowerAVG = Power / samples;

        Serial.println(PowerAVG);



        return;
      }
    }
  }
}

//Will likely need tested for open circuit voltages and have the nominal gen parameter changed to that
bool GENTEST(double Gain[], double NominalGeneratorVoltage, int samples, double upperBoundGen, double lowerBoundGen) {

  double Instant_V = 0;
  double Squares = 0;
  double Vrms = 0;
  int GENTIMEOUT = 0;
  int stableCount = 0;
  int unstableCount = 0;
  int sampleCount = 0;

  while (1) {



    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      Instant_V = res.ch[0].f;

      Squares += Instant_V * Instant_V;
      sampleCount++;

      if (sampleCount >= samples) {

        Vrms = sqrt(Squares / samples) * Gain[0];

        Squares = 0;
        sampleCount = 0;

        if (Vrms <= upperBoundGen && Vrms >= lowerBoundGen) {
          stableCount++;
          unstableCount = 0;
        } else {
          unstableCount++;
          stableCount = 0;
        }

        if (stableCount >= 5) {
          return true;
        }

        if (unstableCount >= 3) {
          return false;
        }

        GENTIMEOUT++;

        if (GENTIMEOUT >= 25) {
          return false;
        }
      }
    }
  }
}
//Need to add a way to add gen test before returning to program position
void lim_Check() {

  bool ENABLED = false;

  // If limit switch trips
  if (digitalRead(LimitSW) == false) {

    // Warning0 "Limit Switch broken"
    Serial.println("Warning0");
    delay(50);

    // Shut off busses immediately
    mcp.digitalWrite(LG_FAULT_EN, LOW);
    delay(100);
    digitalWrite(HV_IO, LOW);
    delay(100);
    digitalWrite(INDUS_IO, LOW);
    delay(100);
    digitalWrite(RES_IO, LOW);



    unsigned long t0 = millis();

    while (millis() - t0 < 2000) {

      if (digitalRead(LimitSW) == false && digitalRead(Motor_Sel) == false) {
        // switch opened again → restart wait
        t0 = millis();
      }
    }

    // Turn generator back on once
    mcp.digitalWrite(LG_FAULT_EN, HIGH);
    delay(50);
    ENABLED = true;
    delay(10000);


    // Test generator stability
    if (GENTEST(Gain, NOM_GEN, samples, upperBoundGen, lowerBoundGen)) {

      digitalWrite(HV_IO, HIGH);
      delay(1000);
      digitalWrite(INDUS_IO, HIGH);
      delay(1000);
      digitalWrite(RES_IO, HIGH);
      delay(500);
      // reconnect HV bus
    } else {
      mcp.digitalWrite(LG_FAULT_EN, LOW);  // keep generator off

      while (1) {
        // Error2 "Gen not stable reset system"
        Serial.println("Error2");
        
      }
    }
  }
}

void systemDiagnostic() {

  //BEGIN startup and system validation

  //Expected Mode and CLK register status for this application
  int ExpectedMode = 0x510;

  int ExpectedClock = 0x7F0E;

  int ModeREG = adc.getModeReg();

  int ClockReg = adc.getClockReg();

  delay(50);

  //Clear serial buffer of MISC transmisisons/Noise
  for (int i = 0; i < (5); i++) {

    Serial.println(i);
    delay(10);
  }
  Serial.println("READY");
  delay(50);

  //ensure registers are correctly initialized
  if (ModeREG != ExpectedMode || ClockReg != ExpectedClock) {
    //Error1: Register configuration failed Reset System!
    Serial.println("Error1");
    adc.SendCommand(CMD_RESET);

    while (1) {
      // halt system
    }
  }

  // // //ADD I/O check ensure all I/O is in the home position (ENSURE LOGIC IS SOUND FOR NO/NC CONNECTIONS)

  if (digitalRead(Motor_Sel) == true) {
    //warning2: "Home motor enable switch"
    Serial.println("warning2");
    while (1) {

      if (digitalRead(Motor_Sel) == false) {
        break;
      }
    }
  }

  if (digitalRead(LimitSW) == false) {
    //warning3: "Lid or Doors open, Close to continue"
    Serial.println("warning3");
    while (1) {

      if (digitalRead(LimitSW) == true) {
        break;
      }
    }
  }

  //ONLY USE IF BOARD HAS BEEN REPAIRED AND GEN_IO has been restored in code
  // if (LG_FAULT_SEL == true || LN_FAULT_SEL == true || LL_FAULT_SEL == true) {
  //   //warning4: "Home all fault selector switches"
  //   Serial.println("warning4");
  //   while (1) {

  //     if (LG_FAULT_SEL == false && LN_FAULT_SEL == false && LL_FAULT_SEL == false) {
  //       break;
  //     }
  //   }
  // }


  if (digitalRead(LN_FAULT_SEL) == true || digitalRead(LL_FAULT_SEL) == true) {
    //warning4: "Home all fault selector switches"
    Serial.println("warning4");
    while (1) {

      if (digitalRead(LN_FAULT_SEL) == false && digitalRead(LL_FAULT_SEL) == false) {
        break;
      }
    }
  }

  if (digitalRead(PF_SEL) == true) {
    //warning5: "Home power factor select switch"
    Serial.println("warning5");
    while (1) {

      if (digitalRead(PF_SEL) == false) {
        break;
      }
    }
  }


  // Enable Power to Generator

  //USING LG_FAULT_EN due to damaged I/O on board
  mcp.digitalWrite(LG_FAULT_EN, 1);

  delay(10000);

  //Check for generator stability before connecting all busses
  if (GENTEST(Gain, NOM_GEN, samples, upperBoundGen, lowerBoundGen) == false) {

    mcp.digitalWrite(LG_FAULT_EN, 0);
    //Error2: Generator Not Stable Reset System!
    Serial.println("Error2");
    while (1) {
      // halt system
    }
  } else {
    Serial.println("GEN stable");
    digitalWrite(HV_IO, 1);
    delay(500);
    digitalWrite(INDUS_IO, 1);
    delay(500);
    digitalWrite(RES_IO, 1);
    delay(500);
    return;
  }

  // Limit switch function call
  //attachInterrupt(digitalPinToInterrupt(A0), lim_Check, LOW);
}
// //Needs to keep ckecking the system while ignoring busses that are currently faulted if another fault occurs the system needs disabled and shut down
void SafeMode(int faultedChannel) {


  // Total Channels being read by ADC
  int total_CHAN = VoltageReadings + CurrentReadings;

  // Array of squared instantaneous voltages at each channel
  double Squares[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with gain accounted for
  double Vrms[total_CHAN] = { 0 };
  // Array of Vrms values at each channel with no gain applied
  double Vrms_NOGAIN[total_CHAN] = { 0 };
  // The instantaneous voltage measured (one of 150 voltages measured)
  double Instant_V[total_CHAN];
  // The resulting IRMS calculated from channels measuring current
  double irms[CurrentReadings] = { 0 };

  // Reset the sample count to 0 ensuring the loop always fully runs
  int sampleCount = 0;

  // I am sorry for this do not dare mess with this set of loops it will break the chip will start doing weird shit and it will not stop
  // you will need to turn the chip off and on and push the old code to hope to fix it
  // also dont dare try and add or subtract from the arrays without full knoledge of this loop it will break everything
  while (sampleCount < samples) {

    if (adc.isDataReady()) {

      AdcOutput res = adc.readAdcFloat();

      for (int i = 0; i < (total_CHAN); i++) {
        Instant_V[i] = res.ch[i].f;

        //Emergency shut off to protect chip not not fast but plus TVS diode is on board but nonetheless good precaution
        if (Instant_V[i] > 1.2) {
          digitalWrite(Disconnect_IO[0], false);
          // Error3: Dangerous Voltage Detected: Check Chip Health and reset!
          Serial.println("Error3");
          while (1) {
          }
        }

        Squares[i] += Instant_V[i] * Instant_V[i];
      }

      sampleCount++;

      if (sampleCount >= samples) {

        for (int i = 0; i < total_CHAN; i++) {
          Vrms[i] = sqrt(Squares[i] / samples) * Gain[i];
          Vrms_NOGAIN[i] = sqrt(Squares[i] / samples);
        }
        // Fault check (skip the already-faulted channel)
        // for (int i = 0; i < VoltageReadings; i++) {

        //   if (i == faultedChannel) {
        //     continue;  // ignore this bus
        //   }

        //   if (Vrms[i] > upperBound[i] || Vrms[i] < lowerBound[i]) {

        //     //Error9: Second fault detected
        //     Serial.println("Error9");

        //     digitalWrite(LG_FAULT_EN, LOW);
        //     delay(50);
        //     digitalWrite(HV_IO, LOW);
        //     delay(50);
        //     digitalWrite(INDUS_IO, LOW);
        //     delay(50);
        //     digitalWrite(RES_IO, LOW);

        //     while (1) {
        //       // force restart
        //     }
        //   }
        //}

        for (int i = 0; i < (VoltageReadings); i++) {
          Serial.println(Vrms[i], 4);
        }

        for (int i = 0; i < CurrentReadings; i++) {

          int adc_index = i + VoltageReadings;

          if (i == faultedChannel) {
            continue;  // ignore this bus
          }

          irms[i] = (Vrms_NOGAIN[adc_index] * TurnsRatio) / (2 * (Burden[i]));

          if ((irms[i] > upperBound[i]) || (irms[i] < lowerBound[i])) {

            if (digitalRead(Motor_Sel) == 0 && i == 0) {
              continue;
            }
            if (digitalRead(Motor_Sel) == 1 && i == 1) {
              continue;
            }

            //Error9: Second fault detected
            Serial.println("Error9");

            digitalWrite(LG_FAULT_EN, LOW);
            delay(50);
            digitalWrite(HV_IO, LOW);
            delay(50);
            digitalWrite(INDUS_IO, LOW);
            delay(50);
            digitalWrite(RES_IO, LOW);
          }

          for (int i = 0; i < (CurrentReadings); i++) {
            Serial.println(Vrms[i], 4);
          }

          GenPowerAnalysis();

          return;
        }
      }
    }
  }
}

void MotorStartup() {

  digitalWrite(RES_IO, 0);
  mcp.digitalWrite(RES_DIS_INDICATOR, 1);

  delay(100);
  // Turn that motor on
  digitalWrite(Motor_ENABLE, 1);

  while (digitalRead(Motor_Sel) == HIGH) {

    lim_Check();
    RunAnalysisNoFault();
    delay(500);
    //powerFactorAnalysis();
    // delay(500);
    PowerFactorEnable();
    delay(100);
    lim_Check();
  }

  delay(100);
  digitalWrite(Motor_ENABLE, 0);
  delay(100);
  digitalWrite(RES_IO, 1);

  mcp.digitalWrite(RES_DIS_INDICATOR, 0);
}

void FaultEnableCheck() {

  //Fault enables
  // --- RESET CONDITION ---
  // Only reset when ALL switches are OFF
  if (digitalRead(LG_FAULT_SEL) == LOW && digitalRead(LL_FAULT_SEL) == LOW && digitalRead(LN_FAULT_SEL) == LOW) {

    activeFault = 0;
  }

  // --- SELECTION LOGIC (only if nothing active) ---
  if (activeFault == 0) {

    if (digitalRead(LG_FAULT_SEL) == HIGH) {
      activeFault = 1;
    }

    else if (digitalRead(LL_FAULT_SEL) == HIGH) {
      activeFault = 2;
    }

    else if (digitalRead(LN_FAULT_SEL) == HIGH) {
      activeFault = 3;
    }
  }

  // --- OUTPUT CONTROL ---
  if (activeFault == 1) {

    //mcp.digitalWrite(LG_FAULT_EN, 1);
    // mcp.digitalWrite(LG_FAULT_INDICATOR, 1);

    mcp.digitalWrite(LL_FAULT_EN, 0);
    mcp.digitalWrite(LL_FAULT_INDICATOR, 0);

    mcp.digitalWrite(LN_FAULT_EN, 0);
    mcp.digitalWrite(LN_FAULT_INDICATOR, 0);
  }

  else if (activeFault == 2) {

    mcp.digitalWrite(LL_FAULT_EN, 1);
    mcp.digitalWrite(LL_FAULT_INDICATOR, 1);

    //mcp.digitalWrite(LG_FAULT_EN, 0);
    mcp.digitalWrite(LG_FAULT_INDICATOR, 0);

    mcp.digitalWrite(LN_FAULT_EN, 0);
    mcp.digitalWrite(LN_FAULT_INDICATOR, 0);
  }

  else if (activeFault == 3) {

    mcp.digitalWrite(LN_FAULT_EN, 1);
    mcp.digitalWrite(LN_FAULT_INDICATOR, 1);

    //mcp.digitalWrite(LG_FAULT_EN, 0);
    mcp.digitalWrite(LG_FAULT_INDICATOR, 0);

    mcp.digitalWrite(LL_FAULT_EN, 0);
    mcp.digitalWrite(LL_FAULT_INDICATOR, 0);
  }

  // Nothing active
  else {

    //mcp.digitalWrite(LG_FAULT_EN, 0);
    mcp.digitalWrite(LG_FAULT_INDICATOR, 0);

    mcp.digitalWrite(LL_FAULT_EN, 0);
    mcp.digitalWrite(LL_FAULT_INDICATOR, 0);

    mcp.digitalWrite(LN_FAULT_EN, 0);
    mcp.digitalWrite(LN_FAULT_INDICATOR, 0);
  }
}

void PowerFactorEnable() {

  if (digitalRead(PF_SEL) == HIGH) {

    digitalWrite(PF_ENABLE, 1);
    mcp.digitalWrite(PF_INDICATOR, 1);
  }
  delay(100);
  // Power factor disable
  if (digitalRead(PF_SEL) == LOW) {

    digitalWrite(PF_ENABLE, 0);
    mcp.digitalWrite(PF_INDICATOR, 0);
  }
}
