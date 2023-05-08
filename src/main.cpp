// PROJECT: Use a relatively complex motion for a gyroscope in order to
// unlock some code in the project. Look out for email called Embedded Sentry

// In the User Manual, pay attention to I3G4250D and find its datasheet.
#include "mbed.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "drivers/LCD_DISCO_F429ZI.h"

// Starter Code Provided by the March 27 Recitation.
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

// Page 22 of STM32F429 User Manual
InterruptIn int2(PA_2,PullDown);
InterruptIn int1(PA_1,PullDown);

// Page 18 of the User Manual
DigitalIn userbutton(PA_0);
DigitalOut l1(LED1), l2(LED2);

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


uint8_t write_buf[32];
uint8_t read_buf[32];

LCD_DISCO_F429ZI lcd;



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

// TODO: An “enter key” and ”record” functionality must be developed.
void enterKey() {

}

void recordKey() {

}
 
int main() {
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
  int1.rise(&recordKey);

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

    gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
    
    printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);

  }
}