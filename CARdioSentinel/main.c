#include "ch.h"
#include "hal.h"
#include "chprintf.h"



#define VOLT_MAX                 3.3f //Tensione massima del microcontrollore
#define MSG_ADC_OK               0x1337
#define MSG_ADC_KO               0x7331
static thread_reference_t trp = NULL;

#define TENSIONE_SOGLIA 2.0f
uint16_t numero_battiti = 0;
static uint32_t infarto = 0;



BaseSequentialStream * chp = (BaseSequentialStream *) &SD2;


#define TIMER_FREQ  1000   // PWM Timer Frequency  10KHz
#define PWM_PERIOD  100     // PWM Period (ticks)   20ms

PWMConfig pwmcfg_motoredi = {
  TIMER_FREQ,
  PWM_PERIOD,
  NULL,                              // Period callback
  {                                 // Channels mode and callback
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  // CH1 output enabled: active is HIGH
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},     // CH2 output disabled
   {PWM_OUTPUT_DISABLED, NULL},     // CH3 output disabled
   {PWM_OUTPUT_DISABLED, NULL}      // CH4 output disabled
  },
  0,
  0,
  0
};
PWMConfig pwmcfg_motoreav = {
  TIMER_FREQ,
  PWM_PERIOD,
  NULL,                              // Period callback
  {                                 // Channels mode and callback
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},  // CH1 output enabled: active is HIGH
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},     // CH2 output disabled
   {PWM_OUTPUT_DISABLED, NULL},     // CH3 output disabled
   {PWM_OUTPUT_DISABLED, NULL}      // CH4 output disabled
  },
  0,
  0,
  0
};

/*
 * GPT4 configuration. This timer is used as trigger for the ADC.
 */

const GPTConfig gpt4cfg = {
  .frequency    =  1000000U, //1000000
  .callback     =  NULL,
  .cr2          =  TIM_CR2_MMS_1,   /* MMS = 010 = TRGO on Update Event.    */
  .dier         =  0U
};


#define ADC_GRP_NUM_CHANNELS        1
#define ADC_GRP_BUF_DEPTH           1



static adcsample_t samples[ADC_GRP_BUF_DEPTH  ]; //Questa per il campionamento
static float converted= { 0.0f }; //Questo serve per la callback corretta in cui avviene la conversione


static void adccallback(ADCDriver *adcp) {

  palToggleLine( LINE_LED_GREEN ); //Per avere un feedback che funge

  if (adcIsBufferComplete(adcp)) {


      converted = (float) samples[0]  / 4096 * VOLT_MAX;

  }



    chSysLockFromISR();
    chThdResumeI(&trp, (msg_t) MSG_ADC_OK ); //Invia un messaggio sulla coda trp.
    chSysUnlockFromISR();
  }





static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  chSysLockFromISR();
  chThdResumeI(&trp, (msg_t) MSG_ADC_KO );
  chSysUnlockFromISR();
}

//funzione che DOVREBBE convertire la tensione in BPM


const ADCConversionGroup adcgrpcfg = {
  .circular     = true,
  .num_channels = ADC_GRP_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = adcerrorcallback,
  .cfgr         = ADC_CFGR_EXTEN_RISING |
                  ADC_CFGR_EXTSEL_SRC(12),  /* TIM4_TRGO */
  .cfgr2        = 0U,
  .tr1          = ADC_TR_DISABLED,
  .tr2          = ADC_TR_DISABLED,
  .tr3          = ADC_TR_DISABLED,
  .awd2cr       = 0U,
  .awd3cr       = 0U,
  .smpr         = {
    0U,
    ADC_SMPR2_SMP_AN15(ADC_SMPR_SMP_247P5)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN15),
    0U,
    0U,
    0U
  }
};



static THD_WORKING_AREA( waBlink, 256 );
static THD_FUNCTION( thdBlink, arg ) {
  (void) arg;

  while(true){
    if(infarto == 1){
        palTogglePad(GPIOC, 1);
        chThdSleepMilliseconds(400);
      }
      chThdSleepMilliseconds(200);
  }
}

static THD_WORKING_AREA( waBpm, 256 );
static THD_FUNCTION( thdBpm, arg ) {
  (void) arg;

  chThdSleepMilliseconds(10000);

  numero_battiti = 0;
  while(true){
    chThdSleepMilliseconds(5000);
    float bpm = numero_battiti * 60/5;

    if(bpm<=30 || bpm >= 120){
      chSysLock();
      infarto=1;
      chSysUnlock();
    }

    chprintf( chp, "bpm %f\r\n",bpm);
    numero_battiti = 0;

  }
}


static THD_WORKING_AREA( waPulse, 256 );
static THD_FUNCTION( thdPulse, arg ) {
  systime_t start, stop;
  sysinterval_t delta;
  chThdSleepMilliseconds(5000);
  (void) arg;
  /*
   * Setting as analog input:
   *    PORTA PIN A0 -> ADC1_CH1
   */

  /* ADC inputs. A3*/
  palSetPadMode(GPIOB, 0U, PAL_MODE_INPUT_ANALOG);

  /*
   * Starting GPT4 driver, it is used for triggering the ADC.
   * Starting the ADC1 driver.
   */
  gptStart(&GPTD4, &gpt4cfg);
  adcStart(&ADCD1, NULL);

  adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);

  /*
   * Start the GPT4 driver with a period of 10000 cycles and a
   * frequency of 1000000 Hz
   */



  gptStartContinuous(&GPTD4, 10000);
  start = chVTGetSystemTime();
  /*Quando chiami chVTGetSystemTime,
  restituir� il tempo di sistema corrente in ticks.
  Questo valore rappresenta il numero di ticks trascorsi dall'avvio del sistema operativo.*/


      uint8_t rising = 0;


  while( true ) {
    msg_t msg;

    chSysLock();
    msg = chThdSuspendS(&trp);

    stop = chVTGetSystemTime();

    chSysUnlock();


    delta = chTimeDiffX( start, stop ); //intervallo di campionamento
    /*
     * Check the message and inform the user
     */

    if( (uint32_t) msg == MSG_ADC_OK ) {

      if(converted > TENSIONE_SOGLIA && !rising) {

        //chprintf( chp, "battito n. %d\r\n",numero_battiti++);
        numero_battiti++;

        rising = 1;

      }
      if(converted < TENSIONE_SOGLIA && rising) {
        rising = 0;
      }

    } else {
      chprintf( chp, "Elapsed = %d ms\r\n" , TIME_I2MS( delta ) );
      chprintf( chp, "Error!\r\n" );
    }

    start = stop;

  }
  adcStop(&ADCD1);
}

static THD_WORKING_AREA( waAuto, 256 );
static THD_FUNCTION( thdAuto, arg ){
  (void) arg;

  pwmStart(&PWMD5, &pwmcfg_motoreav);  // Inizia la generazione PWM sul TIM1
  pwmStart(&PWMD3, &pwmcfg_motoredi);

  while (true) {
    if( infarto == 0 ){
      // Imposta il duty cycle per controllare la velocità del motore
      pwmEnableChannel(&PWMD5, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 6000));  // Ad esempio, impostato al 60%
      pwmEnableChannel(&PWMD5, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 6000));
      pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 6000));  // Ad esempio, impostato al 60%
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 6000));
      while( infarto == 0 ){
        chThdSleepMilliseconds(500);
      }
    }
    else{
      pwmEnableChannel(&PWMD5, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 3000));  // Ad esempio, impostato al 60%
      pwmEnableChannel(&PWMD5, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 3000));
      pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 3000));  // Ad esempio, impostato al 60%
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 3000));
      chThdSleepSeconds(5);

      pwmDisableChannel(&PWMD5,0);  // disabilito il canale pwm per interrompere le ruote
      pwmDisableChannel(&PWMD5,1);
      pwmDisableChannel(&PWMD3,0);  // disabilito il canale pwm per interrompere le ruote
      pwmDisableChannel(&PWMD3,1);

      while( infarto == 1 ){
        chThdSleepMilliseconds(500);
      }
    }
    chThdSleepMilliseconds(5000);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();
  chSysInit();
   /*
   * Activates the serial driver 2 using the A2 and A3 pins.
   */
  palSetPadMode( GPIOC, 1, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPadMode( GPIOA, 2, PAL_MODE_ALTERNATE(7) );
  palSetPadMode( GPIOA, 3, PAL_MODE_ALTERNATE(7) );
  palSetPadMode(GPIOA, 0U, PAL_MODE_ALTERNATE(2)); //Ruota avanti
  palSetPadMode(GPIOA, 1U, PAL_MODE_ALTERNATE(2)); //Ruota avanti
  palSetPadMode(GPIOA, 4U, PAL_MODE_ALTERNATE(2));  //Ruota dietro
  palSetPadMode(GPIOA, 6U, PAL_MODE_ALTERNATE(2));  //Ruota dietro

  sdStart( &SD2, NULL );

  chThdCreateStatic( waAuto, sizeof( waAuto), NORMALPRIO + 5 , thdAuto, (void*) NULL );
  chThdCreateStatic( waPulse, sizeof( waPulse), NORMALPRIO + 5, thdPulse, (void*) NULL );
  chThdCreateStatic( waBpm, sizeof( waBpm), NORMALPRIO + 6, thdBpm, (void*) NULL );
  chThdCreateStatic( waBlink, sizeof( waBlink), NORMALPRIO + 5, thdBlink, (void*) NULL );


}

