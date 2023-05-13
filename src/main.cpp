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
using namespace std;

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
Here are the possible mode for this problem, numbers correspond to presses of the UserButton:
    0 - Initial state, nothing happens
    1 - Starts lock recording mode.
    2 - Should save the lock we just recorded. Nothing else happens
    3 - We will attempt to unlock the resource.
    4 - Compare the lock and unlock motions to determine if its fine
     to unlock the resource.
    5 - Determines if resource unlocks or stays locked.
        User gets two choices:
        - First choice: Press the UserButton again, reset everything
          and start from stage 0. (Reset)

        - Second choice: Touch on the screen, returns to stage 2.
          We also wipe the unlock vector (Retry)
*/
volatile int buttonPresses = 0; //Set up the int variable for counting button presses

// We will define a structure or a class here for the gyroscope data.
class GyroReading {
  public:
    float axis_x;
    float axis_y;
    float axis_z;
    // Functions and definitions go here
    GyroReading(float x, float y, float z) {
      axis_x = x;
      axis_y = y;
      axis_z = z;
    }

    void printCoords() {
      cout << "x_axis: " << axis_x << ", y_axis: " << axis_y << ", z_axis: " << axis_z << endl;
    }
};

GyroReading readMovement() {
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

  //printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);
  GyroReading current_reading(gy, gy, gz);
  return current_reading;
}

class GyroRecording {
public:
  vector<GyroReading> gyroSample;

  void recordMovement(const GyroReading& reading) {
    gyroSample.push_back(reading);
  }

  int sampleSize() const {
    return gyroSample.size();
  }

  GyroReading getGyroReadSample(int index) const {
    return gyroSample[index];
  }
};

EventFlags flags;
//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

void data_cb(){
  flags.set(DATA_READY_FLAG);
};

// TODO: Change this to a switch statement
//This function is designed to switch the mode and visual LED signal (for UserButton):
void Switch() {
  if(buttonPresses == 0 || buttonPresses == 1){
    l1 = !l1; //Switch the LED signal for Lock Recording
    buttonPresses += 1; //Switch the Recording mode
  }else if(buttonPresses == 2 || buttonPresses == 3){
    l2 = !l2; //Switch the LED signal for Unlock Recording
    buttonPresses += 1; //Switch the Recording mode
  }else if(buttonPresses == 4){
    //Set all LEDs up as a signal of result
    l1 = 1;
    l2 = 1;

    buttonPresses += 1;  //Switch the Recording mode
  }else{
    //Reset all LEDs
    l1 = 0;
    l2 = 0;

    //Reset record mode
    buttonPresses = 0;
  }
}

//This function will be used to avoid mis-read of Screen Touch
bool screenTouched(){
  TS_StateTypeDef currentTouch;           //Define a new TouchScreen State for further check
  thread_sleep_for(100);                  //Leaving a period time to allow finger move away
  BSP_TS_GetState(&currentTouch);         //Update the TouchScreen State
  return !currentTouch.TouchDetected;     //If screen is touched, it will return true
}

/*
To calculate distance between 2 points in 3D, we used this resource:
https://byjus.com/maths/distance-between-two-points-3d/

 sqrt[(x2 – x1)^2 + (y2 – y1)^2 + (z2 – z1)^2]
*/
float distanceCalc(const GyroReading& currentSample, const GyroReading& previousSample) {
  float comp_xaxis = pow(currentSample.axis_x - previousSample.axis_x, 2);
  float comp_yaxis = pow(currentSample.axis_y - previousSample.axis_y, 2);
  float comp_zaxis = pow(currentSample.axis_z - previousSample.axis_z, 2);

  float dist_difference = sqrt(comp_xaxis + comp_yaxis + comp_zaxis);

  return dist_difference;
}

/*
We must use two vectors that we just finished sampling, and compare the two.
As there will invariably be differences when it comes to size and samples, we must:

- Take into account the limitations of the smaller set.
- Compare the values of the axes to determine whether the resource can be unlocked.
*/
float similarityComp(const GyroRecording& largerSet, const GyroRecording& smallerSet) {
  int smallerSetAmount = smallerSet.sampleSize();

  float similarity = 0.00;
  /*
  We'll be starting from index 1, as the function (distanceCalc) above will calculate the distance
  between one point and the previous one. Since Vectors such as the GyroRecording class start at
  an index of 0, this should be fine. We should then compare the larger and smaller sets to see
  how similar they are via division.
  We should ideally get values close to 1 when dividing. Then we add our value we used division with
  into the similarityVal above.
  */
  for (int i = 1; i <= smallerSetAmount; i++) {
    GyroReading smlSetSmpCrnt = smallerSet.getGyroReadSample(i);
    GyroReading smlSetSmpPrv = smallerSet.getGyroReadSample(i-1);
    float smlSetSmpDist = distanceCalc(smlSetSmpCrnt, smlSetSmpPrv);

    GyroReading lrgSetSmpCrnt = largerSet.getGyroReadSample(i);
    GyroReading lrgSetSmpPrv = largerSet.getGyroReadSample(i-1);
    float lrgSetSmpDist = distanceCalc(lrgSetSmpCrnt, lrgSetSmpPrv);

    float currSimilarity = lrgSetSmpDist/smlSetSmpDist;
    similarity += currSimilarity;
  }

  float finalSimilarity = similarity/smallerSetAmount;

  return finalSimilarity;
}
 
int main() {
  // Enable the button interrupt
  btn.rise(&Switch);

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

  // Testing LCD Output
  lcd.Clear(LCD_COLOR_RED);
  lcd.DisplayStringAtLine(0, (uint8_t*)"Hello World!");

  write_buf[0]=CTRL_REG3;
  write_buf[1]=CTRL_REG3_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  // This will serve as our new main function, above will need to be put
  // into a new function

  while(1) {
    BSP_TS_GetState(&ts); //Get the TouchScreen status

    if (buttonPresses == 5 && ts.TouchDetected && screenTouched()) {//Retry setting
      buttonPresses = 2;
      //Clear all saved data in Unlock recording part (leave Lock alone!)
    }

    if(buttonPresses == 1){  //Start recording of Lock 
      //Save gyro data
    }else if(buttonPresses == 3){  //Start recording of Unlock
      //Save gyro data
    }else if(buttonPresses == 4){  //Start recording comparison
      //Calculation code
    }else if(buttonPresses == 5){  //Result return
      
    }
  }
}