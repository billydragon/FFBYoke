#include "Due_QDEC.h"


#ifdef _VARIANT_ARDUINO_DUE_X_

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


void Due_QDEC::setConfig() {
  
  axis[X_AXIS].maxValue = MAX_X_VALUE;
  axis[X_AXIS].minValue =  - axis[X_AXIS].maxValue;

  axis[Y_AXIS].maxValue = MAX_Y_VALUE ;
  axis[Y_AXIS].minValue =  -axis[Y_AXIS].maxValue;
  
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

void  Due_QDEC::updatePosition(int idx) {

      axis[X_AXIS].currentPosition = REG_TC0_CV0; 
      axis[Y_AXIS].currentPosition = REG_TC2_CV0;

        axis[idx].positionChange = axis[idx].currentPosition - axis[idx].lastPosition;
        uint32_t currentEncoderTime = (int32_t) millis();
        int16_t diffTime = (int16_t)(currentEncoderTime - axis[idx].lastEncoderTime) ;
        if (diffTime > 0) {
          axis[idx].currentVelocity = axis[idx].positionChange / diffTime;
          axis[idx].currentAcceleration = (abs(axis[idx].currentVelocity) - abs(axis[idx].lastVelocity)) / diffTime;
          axis[idx].lastEncoderTime = currentEncoderTime;
          axis[idx].lastVelocity = axis[idx].currentVelocity;
        }
        axis[idx].lastPosition = axis[idx].currentPosition;
      
}


void Due_QDEC::Reset_Encoder(int idx)
{
  if (idx==0)
  {
      REG_TC0_CCR0 = TC_CCR_CLKDIS;  //disable clock
      REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;  //enable clock and set Trigger
                                                   //for Counter reset at the next clock
      activateCNT_TC0();  // Reset Counter TC0
      delay(500);
  }
  else
  {
      REG_TC2_CCR0 = TC_CCR_CLKDIS;  //disable clock
      REG_TC2_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;  //enable clock and set Trigger 
                                                   //for Counter reset at the next clock

       activateCNT_TC2();  // Reset Counter TC2
       delay(500);
  }
  
   
}


Due_QDEC::~Due_QDEC() {

}
#endif