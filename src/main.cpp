// PROJECT: Use a relatively complex motion for a gyroscope in order to
// unlock some code in the project. Look out for email called Embedded Sentry

#include "mbed.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// Starter Code Provided by the March 27 Recitation.
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

// For OUT_X_L and others, page 36 on I3G4250D Datasheet
#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts                 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define SCALING_FACTOR 17.5f*0.017453292519943295769236907684886f / 1000.0f

// Page 22 of STM32F429 User Manual
InterruptIn int2(PA_2,PullDown);
// Page 18 of the User Manual
DigitalOut l1(LED1), l2(LED2);

uint8_t write_buf[32];
uint8_t read_buf[32];

//Initialize the UserButton, TouchScreen and LCD
LCD_DISCO_F429ZI lcd;
InterruptIn btn(USER_BUTTON, PullDown);
TS_StateTypeDef ts;

/*
For this problem, we mainly use the UserButton (PA_0) as the trigger to our mode change,
  Here are the possible mode for this problem:
    0 - Default mode, nothing happens
    1 - After the first press of the UserButton. It will start the Lock recording mode.
    2 - After the second press of the UserButton. It will shut down the Lock recording 
        and save it.
    3 - After the third press of the UserButton. It will start the Unlock recording mode.
    4 - After the fourth press of the UserButton. It will shut down the Unlock recording
        and save it. Besides the recording, this part will also do the comparison of 
        two saved data.

    5 - After the fifth press of the UserButton. It will return the result of the 
        comparison. Then there are two choices for the next step:
          First choice: If we press the UserButton again, we will reset the whole process
          and start from the Stage-0, which is the default mode. (Reset)

          Second choice: If we press the touch screen, it will turn Stage-2, which allow
          us to record the Unlock again and try one more time. (Retry)
*/
volatile int RecordMode = 0; //Set up the int variable for recording mode

// We will define a structure or a class here for the gyroscope data.
class GyroscopeReading {
  private:
    float x;
    float y;
    float z;
  public:
    // Functions and definitions go here
};

EventFlags flags;
//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

// TODO: An “enter key” and ”record” functionality must be developed.
void data_cb(){
  flags.set(DATA_READY_FLAG);
};

// TODO: Change this to a switch statement
//This function is designed to switch the mode and visual LED signal (for UserButton):
void Switch(){
  if(RecordMode == 0 || RecordMode == 1){
    l1 = !l1; //Switch the LED signal for Lock Recording
    RecordMode += 1; //Switch the Recording mode
  }else if(RecordMode == 2 || RecordMode == 3){
    l2 = !l2; //Switch the LED signal for Unlock Recording
    RecordMode += 1; //Switch the Recording mode
  }else if(RecordMode == 4){
    //Set all LEDs up as a signal of result
    l1 = 1;
    l2 = 1;

    RecordMode += 1;  //Switch the Recording mode
  }else{
    //Reset all LEDs
    l1 = 0;
    l2 = 0;

    //Reset record mode
    RecordMode = 0;
  }
}

//This function will be used to avoid mis-read of Screen Touch
bool TouchEnding(){
  TS_StateTypeDef Now;          //Define a new TouchScreen State for further check
  thread_sleep_for(100);        //Leaving a period time to allow finger move away
  BSP_TS_GetState(&Now);        //Update the TouchScreen State
  return !Now.TouchDetected;    //If now the touch ends, it will return true(bool).
}
 
int main() {

  btn.rise(&Switch);
  // Everything below will need to be in a new function

  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,3);
  spi.frequency(1'000'000);

  write_buf[0]=CTRL_REG1;
  write_buf[1]=CTRL_REG1_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG4;
  write_buf[1]=CTRL_REG4_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  //configure the interrupt to call our function
  //when the pin becomes high
  int2.rise(&data_cb);

  // Testing LCD Output
  lcd.Clear(LCD_COLOR_RED);
  lcd.DisplayStringAtLine(0, (uint8_t*)"Hello World!");

  write_buf[0]=CTRL_REG3;
  write_buf[1]=CTRL_REG3_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  // The gyroscope sensor keeps its configuration between power cycles.
  // This means that the gyroscope will already have it's data-ready interrupt
  // configured when we turn the board on the second time. This can lead to
  // the pin level rising before we have configured our interrupt handler.
  // To account for this, we manually check the signal and set the flag
  // for the first sample.
  if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
    flags.set(DATA_READY_FLAG);
  }
  while (1) {
    int16_t raw_gx,raw_gy,raw_gz;
    float gx, gy, gz;

    //wait until new sample is ready
    flags.wait_all(DATA_READY_FLAG);
    //prepare the write buffer to trigger a sequential read
    write_buf[0]=OUT_X_L|0x80|0x40;

    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
    //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
    raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
    raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
    raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

    //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);

    gx=((float)raw_gx)*(SCALING_FACTOR);
    gy=((float)raw_gy)*(SCALING_FACTOR);
    gz=((float)raw_gz)*(SCALING_FACTOR);
    
    printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);

  }

  // This will serve as our new main function, above will need to be put
  // into a new function

  /*
  while(1) {
    BSP_TS_GetState(&ts); //Get the TouchScreen status first

    if (RecordMode == 5 && ts.TouchDetected && TouchEnding()) {//Retry setting
      RecordMode = 2;
      //Clear all saved data in Unlock recording part (leave Lock alone!)
    }

    if(RecordMode == 1){  //Start recording of Lock 
      //Save gyro data
    }else if(RecordMode == 3){  //Start recording of Unlock
      //Save gyro data
    }else if(RecordMode == 4){  //Start recording comparison
      //Calculation code
    }else if(RecordMode == 5){  //Result return
      
    }
  }
  */
}