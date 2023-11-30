// Group 33 RTES Challenge
// Group Member:
//  Name:                   | NetID:
//    Tommy Martinez        |   tm3272
//    Vamsi Krishna Bunga   |   vb2279
//    Tianle Liu            |   tl3796
//    Vidya Bhagnani        |   vb2356

// PROJECT: Use a relatively complex motion for a gyroscope in order to
// unlock some code in the project. Look out for email called Embedded Sentry

#include "mbed.h"
#include "rtos/Thread.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <deque>
using namespace std;

/*
For this problem, we mainly use the TouchScreen(ts defined below) as the trigger 
    to our mode change,
Here are the possible mode for this problem, numbers correspond to presses of the TouchScreen:
    0 - Initial state, nothing happens
    1 - Starts lock recording mode.
    2 - Should save the lock we just recorded. Nothing else happens
    3 - We will attempt to unlock the resource.
    4 - Compare the lock and unlock motions to determine if its fine
     to unlock the resource. The result will be shown to user.
*/

/* Firstly, we will do the configuration & initialization: */
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

#define SPI_FLAG 1
#define SCALING_FACTOR 17.5f*0.017453292519943295769236907684886f / 1000.0f

uint8_t write_buf[32];
uint8_t read_buf[32];

//Initialize the UserButton, TouchScreen and LCD
LCD_DISCO_F429ZI lcd;
DigitalOut l1(LED1);
DigitalIn btn(USER_BUTTON);
InterruptIn btnINT(USER_BUTTON);
TS_StateTypeDef ts;

//Initialize usable variables
volatile int pressCount = 0;                //Set up the int variable for counting screen presses
volatile bool RecordStatus = false;         //Set up the bool variable for tracking recording status
string currStatus = "...";                 //Set up string variable for saving the status of unlock
int gyroThreshold = 0;                                //Set up int variable for saving threashold
float Similarity = 0.00;                    //Set up float variable for saving the comparison similarity
bool resetFlag = false;                         // Reset Status, which will reset all data and start the procee all over again.
bool retryFlag = false;                         // Retry Status, which will retry the recording process when the match if unsuccesful.

//Function used to avoid mis-read of Screen Touch
bool screenTouched(){
  TS_StateTypeDef currentTouch;             //Define a new TouchScreen State for further check
  thread_sleep_for(100);                    //Leaving a period time to allow finger move away
  BSP_TS_GetState(&currentTouch);           //Update the TouchScreen State
  return !currentTouch.TouchDetected;       //If screen is touched, it will return true
}

// The UserButton pressed.
void btnPressed(){
    resetFlag = true;
}

/* Starting here, we will modify and create the Gyro data storage and calculation related class
    and functions. */

// We will define a structure or a class here for the gyroscope data.
class GyroReading {
    public:
        float x;
        float y;
        float z;

        GyroReading(float x, float y, float z) {
            x = x;
            y = y;
            z = z;
        }
};

using GyroDataSet = vector<GyroReading>;   //Using vector to store GyroReading class variable

// Initialize variables for GyroReading storage
GyroDataSet LockGyro;                           //Set up the vector to store the Lock gyro recording
GyroDataSet UnlockGyro;                         //Set up the vector to store the Unlock gyro recording

// The next two functions will be used to calculate the comparison similarity.
// Function to calculate the Euclidean distance between two GyroReadings
float calculateGyroDistance(const GyroReading& reading1, const GyroReading& reading2) {
    // Subtract the x component of the second reading from the first, square the result
    float xDifferenceSquared = pow(reading1.x - reading2.x, 2);

    // Subtract the y component of the second reading from the first, square the result
    float yDifferenceSquared = pow(reading1.y - reading2.y, 2);

    // Subtract the z component of the second reading from the first, square the result
    float zDifferenceSquared = pow(reading1.z - reading2.z, 2);

    // Sum the squared differences
    float sumOfSquaredDifferences = xDifferenceSquared + yDifferenceSquared + zDifferenceSquared;

    // Return the square root of the sum, which is the Euclidean distance
    return sqrt(sumOfSquaredDifferences);
}

/*
    Here we introduce and use the concept of Dynamic Time Warping (DTW, a way of comparing two, 
    temporal sequences that don't perfectly sync up) to help us compare. 
    
    Article on Distance Time Warp: https://towardsdatascience.com/dynamic-time-warping-3933f25fcdd
*/
// Function to calculate the Dynamic Time Warping (DTW) distance between two GyroDataSets
float DTW_Calculation(const GyroDataSet& reading1, const GyroDataSet& reading2) {
    // Get the size of each data set
    int dataSet1Size = reading1.size();
    int dataSet2Size = reading2.size();

    // Initialize a 2D vector (matrix) with infinity values
    // The size of the matrix is (dataSet1Size+1) x (dataSet2Size+1)
    vector<vector<float>> dtwMatrix(dataSet1Size + 1, vector<float>(dataSet2Size + 1, numeric_limits<float>::infinity()));

    // The distance between the start points of both sequences is set to 0
    dtwMatrix[0][0] = 0;

    // Iterate over each item in reading1
    for (int i = 1; i <= dataSet1Size; ++i) {
        // For each item in reading1, iterate over each item in reading2
        for (int j = 1; j <= dataSet2Size; ++j) {
            // Calculate the Euclidean distance between the current pair of gyro readings
            float eucDist = calculateGyroDistance(reading1[i - 1], reading2[j - 1]);

            // The value for the current pair in the matrix is the cost plus the minimum value from the previous pair in the matrix
            dtwMatrix[i][j] = eucDist + min({dtwMatrix[i - 1][j], dtwMatrix[i][j - 1], dtwMatrix[i - 1][j - 1]});
        }
    }

    // Return the value from the last cell of the matrix, which is the DTW distance
    return dtwMatrix[dataSet1Size][dataSet2Size];
}

EventFlags flags;
//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
};

/*
    The following variable definition and function will be used as filtering for the raw gyro
    data and transfer it to a more formal and usable format.
*/
deque<GyroReading> filter;  //Define a double ended queue for filter usage
const int FILTER_SIZE = 5;   //Define the filter size

// Function to get filtered gyro data
GyroReading filterGyro() {

    // Declare variables to store raw gyro data
    volatile int16_t raw_gx, raw_gy, raw_gz;

    // Declare variables to store actual gyro data
    volatile float gx, gy, gz;

    float totalGx = 0;
    float totalGy = 0;
    float totalGz = 0;

    // Read the status register. Bit 4 of the status register
    // is 1 when a new set of samples is ready
    write_buf[0] = 0x27 | 0x80;
    while ((read_buf[1] & 0b00001000) == 0){
        spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE );
        flags.wait_all(SPI_FLAG);
    };

    // Prepare the write buffer to trigger a sequential read
    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    // Start sequential sample reading
    spi.transfer(write_buf, 7, read_buf, 8, spi_cb, SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    // Reorder the high and low bytes: lowB, HighB -> HighB, LowB
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    // Convert the raw data to actual data in dps (degrees per second) and then to rad/s
    gx = ((float)raw_gx) * (SCALING_FACTOR);
    gy = ((float)raw_gy) * (SCALING_FACTOR);
    gz = ((float)raw_gz) * (SCALING_FACTOR);

    
    // Create a GyroReading instance with the actual data
    GyroReading gyroOutput(gx, gy, gz);

    // Add raw sample to the filter window
    filter.push_back(gyroOutput);

    // If the window size exceeds FILTER_SIZE, remove the oldest sample
    if (filter.size() > FILTER_SIZE) {
        filter.pop_front();
    }

    // Calculate the sum of all samples in the window for each axis
    for (const auto &sample : filter) {
        totalGx += sample.x;
        totalGy += sample.y;
        totalGz += sample.z;
    }

    // Compute the average for each axis
    float averageGx = totalGx / filter.size();
    float averageGy = totalGy / filter.size();
    float averageGz = totalGz / filter.size();

    // Return the filtered gyroscope data
    return GyroReading(averageGx, averageGy, averageGz);
}

/* The following functions are used as the helping function 
    that will be used in the Main function. */
void ResetModify(){
    lcd.Clear(LCD_COLOR_WHITE);

    LockGyro.clear();
    UnlockGyro.clear();

    RecordStatus = false;
    pressCount = 0;

    Similarity = 0.00;
    currStatus = "...";
    resetFlag = false;
    gyroThreshold = 0;
}

void RetryModify(){
    lcd.Clear(LCD_COLOR_WHITE);

    // Only clear UnlockGyro.
    UnlockGyro.clear();
            
    RecordStatus = false;
    pressCount = 2;

    Similarity = 0.00;
    currStatus = "...";
    resetFlag = false;
    retryFlag = false;
}

// Displays the LCD on the board
void LCD_Display(uint16_t x, char* buffer1, char* buffer2){
    lcd.DisplayStringAt(x, 110, (uint8_t *)buffer1, LEFT_MODE);
    lcd.DisplayStringAt(x, 135, (uint8_t *)buffer2, LEFT_MODE);
}


// Main function.
int main() {
    // Set up the SPI for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,3);
    spi.frequency(1'000'000);

    // Write to Control Register 1 to configure the gyro
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    // Write to Control Register 4 to configure the gyro
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    // Set the rising edge interrupt callback function.
    btnINT.rise(&btnPressed);

    // Initialize LCD display with white background and black text.
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font20);

    // Buffers to store string conversion of different variables.
    char pressCountBuffer[32];
    char unlockStatusBuffer[32];

    uint16_t displayStartX = 20; // X-coordinate for LCD display start position.

    // Initialize Touch Screen.
    BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

    // Indicator Variables
    int lockGyroLength = 0; // Length of LockGyro.
    float lengthRatio = 0.00; // The length ratio of UnlockGyro when compared to LockGyro.

    while (1) {
        if (resetFlag == true && retryFlag == false) {          // When we need to reset and don't need to retry
            ResetModify();
            lockGyroLength = 0;
            lengthRatio = 0.00;
        } else if (resetFlag == true && retryFlag == true){     // When we need to reset and retry
            RetryModify();
            lengthRatio = 0.00;
        }

        // Convert variables to strings for displaying on LCD.
        sprintf(pressCountBuffer, "pressCount: %d", pressCount);
        sprintf(unlockStatusBuffer, "Status: %s", currStatus.c_str());
        BSP_TS_GetState(&ts);

        // Prevent pressCount from increasing after reaching 4.
        if (pressCount < 4 && ts.TouchDetected && screenTouched()) {
            RecordStatus = !RecordStatus;
            pressCount = pressCount + 1;
        }

        // Handle the LED and LockGyro or UnlockGyro storage.
        if (RecordStatus) {
            l1 = 1;
            if (pressCount == 1) {
                LockGyro.push_back(filterGyro());
                currStatus = "REC_LOCK";
                lcd.Clear(LCD_COLOR_YELLOW);
                lcd.SetBackColor(LCD_COLOR_YELLOW);
                lcd.SetTextColor(LCD_COLOR_BLACK);
            } else if (pressCount == 3) {
                UnlockGyro.push_back(filterGyro());
                currStatus = "REC_UNLK";
                lcd.Clear(LCD_COLOR_BLUE);
                lcd.SetBackColor(LCD_COLOR_BLUE);
                lcd.SetTextColor(LCD_COLOR_BLACK);
            }
            thread_sleep_for(25);
        } else {
            l1 = 0;
            // Clear filter_windows after each completed recording.
            filter.clear();
        }

        // While not recording, we allow time for user to get ready to unlock mechanism
        if (pressCount == 2) {
            currStatus = "STDBY";
            lcd.Clear(LCD_COLOR_WHITE);
            lcd.SetBackColor(LCD_COLOR_WHITE);
            lcd.SetTextColor(LCD_COLOR_BLACK);
        }

        // Make decision based on recorded data.
        if (pressCount == 4) {
            // Calculate the Dynamic Time Warping (DTW) distance between LockGyro and UnlockGyro
            Similarity = DTW_Calculation(LockGyro, UnlockGyro);

            // Calculate the sizes of LockGyro and UnlockGyro
            size_t lockGyroSize = LockGyro.size();
            size_t unlockGyroSize = UnlockGyro.size();

            // Cast sizes to appropriate types for calculations
            lockGyroLength = static_cast<int>(lockGyroSize);
            lengthRatio = static_cast<float>(unlockGyroSize) / static_cast<float>(lockGyroSize);

            // Define the threshold value
            gyroThreshold = static_cast<int>((static_cast<float>(lockGyroLength) / 100.0f) * 60.0f);

            // Check for length difference between LockGyro and UnlockGyro
            // If the difference is too large, set an error message and enable retryFlag
            if (lengthRatio < 0.75 || lengthRatio > 1.25) {
                currStatus = "REDO";
                lcd.Clear(LCD_COLOR_RED);
                lcd.SetBackColor(LCD_COLOR_RED);
                lcd.SetTextColor(LCD_COLOR_WHITE);
                retryFlag = true;
            } else {
                // If the similarity is within the threshold, report YES
                // If it exceeds the threshold, report NO and enable retryFlag
                if (Similarity < gyroThreshold) {
                    currStatus = "UNLOCK";
                    lcd.Clear(LCD_COLOR_GREEN);
                    lcd.SetBackColor(LCD_COLOR_GREEN);
                    lcd.SetTextColor(LCD_COLOR_BLACK);
                } else {
                    currStatus = "REDO";
                    lcd.Clear(LCD_COLOR_RED);
                    lcd.SetBackColor(LCD_COLOR_RED);
                    lcd.SetTextColor(LCD_COLOR_WHITE);
                    retryFlag = true;
                }
            }
        }

        // Display the information on LCD.
        LCD_Display(displayStartX, pressCountBuffer, unlockStatusBuffer);
    }
}