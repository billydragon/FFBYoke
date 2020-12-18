#include "Due_QDEC.h"




Due_QDEC::Due_QDEC() {
  
  // activate peripheral functions for quad pins Encoder-X
  REG_PIOB_PDR = mask_quadX_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadX_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quadX_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadX_B;   // choose peripheral option B


  // activate peripheral functions for quad pins Encoder-Y
  REG_PIOB_PDR = mask_quadY_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadY_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quadY_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadY_B;   // choose peripheral option B

  // activate peripheral functions for quad pins second decoder
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC25B_TIOA6, PIO_DEFAULT);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC26B_TIOB6, PIO_DEFAULT);


  // activate clock for TC0
  activateCNT_TC0();
  
  // activate clock for TC2
  activateCNT_TC2();

}

void Due_QDEC::activateCNT_TC0() // X Axis
{
  // activate clock for TC0
  REG_PMC_PCER0 = (1<<27);
  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12)|(1<<13);
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_IER1=0b10000000; // enable overflow interrupt TC0
  REG_TC0_IDR1=0b01111111; // disable other interrupts TC0
  NVIC_EnableIRQ(TC0_IRQn); // enable TC0 interrupts
}

void Due_QDEC::activateCNT_TC2() // Y Axis
{
  REG_PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
  REG_PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;
  // select XC0 as clock source and set capture mode
  REG_TC2_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC2_BMR = (1<<9)|(1<<8)|(1<<12)|(1<<13);
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC2_CCR0 = 5;
  REG_TC2_IER1=0b10000000; // enable overflow interrupt TC2
  REG_TC2_IDR1=0b01111111; // disable other interrupts TC2
  NVIC_EnableIRQ(TC2_IRQn); // enable TC2 interrupts
}


void Due_QDEC::setConfig(YokeConfig YokeConfig) {
  axis[0].cPR = YokeConfig.configCPR_X ;
  axis[0].maxAngle = YokeConfig.configMaxAngle_X;
  axis[0].inverted = YokeConfig.configInverted_X;
  axis[0].maxValue = (float)axis[0].maxAngle / 2 / 360 * axis[0].cPR ;
  axis[0].minValue =  - axis[0].maxValue;
 
  axis[1].cPR = YokeConfig.configCPR_Y ;
  axis[1].maxAngle = YokeConfig.configMaxAngle_Y;
  axis[1].inverted = YokeConfig.configInverted_Y;
  axis[1].maxValue = (float)axis[1].maxAngle / 2 / 360 * axis[1].cPR ;
  axis[1].minValue =  -axis[1].maxValue;
  
  initVariables();
}

void Due_QDEC::initVariables() {
  for (int i=0;i<2;i++)
  {
      axis[i].currentPosition = 0;
      axis[i].lastPosition = 0;
      axis[i].correctPosition = 0;
      axis[i].maxAcceleration = 0;
      axis[i].maxVelocity = 0;
      axis[i].lastEncoderTime = (uint32_t) millis();
      axis[i].lastVelocity = 0;
     
  }
}

void  Due_QDEC::updatePosition_X() {

        axis[0].currentPosition = REG_TC0_CV0;  // Value of Encoder X
        axis[0].positionChange = axis[0].currentPosition - axis[0].lastPosition;
        uint32_t currentEncoderTime = (int32_t) millis();
        int16_t diffTime = (int16_t)(currentEncoderTime - axis[0].lastEncoderTime) ;
        if (diffTime > 0) {
          axis[0].currentVelocity = axis[0].positionChange / diffTime;
          axis[0].currentAcceleration = (abs(axis[0].currentVelocity) - abs(axis[0].lastVelocity)) / diffTime;
          axis[0].lastEncoderTime = currentEncoderTime;
          axis[0].lastVelocity = axis[0].currentVelocity;
        }
        axis[0].lastPosition = axis[0].currentPosition;
      
}

void  Due_QDEC::updatePosition_Y() {
        axis[1].currentPosition = REG_TC2_CV0;
        axis[1].positionChange = axis[1].currentPosition - axis[1].lastPosition;
        uint32_t currentEncoderTime = (int32_t) millis();
        int16_t diffTime = (int16_t)(currentEncoderTime - axis[1].lastEncoderTime) ;
        if (diffTime > 0) {
          axis[1].currentVelocity = axis[1].positionChange / diffTime;
          axis[1].currentAcceleration = (abs(axis[1].currentVelocity) - abs(axis[1].lastVelocity)) / diffTime;
          axis[1].lastEncoderTime = currentEncoderTime;
          axis[1].lastVelocity = axis[1].currentVelocity;
        }
        axis[1].lastPosition = axis[1].currentPosition;
      
}

void Due_QDEC::Reset_Encoder_X()
{
    REG_TC0_CCR0 = TC_CCR_CLKDIS;  //disable clock
      REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;  //enable clock and set Trigger
                                                   //for Counter reset at the next clock
      activateCNT_TC0();  // Reset Counter TC0
}

void Due_QDEC::Reset_Encoder_Y()
{
    REG_TC2_CCR0 = TC_CCR_CLKDIS;  //disable clock
      REG_TC2_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;  //enable clock and set Trigger 
                                                   //for Counter reset at the next clock

       activateCNT_TC2();  // Reset Counter TC2
}

Due_QDEC::~Due_QDEC() {

}
