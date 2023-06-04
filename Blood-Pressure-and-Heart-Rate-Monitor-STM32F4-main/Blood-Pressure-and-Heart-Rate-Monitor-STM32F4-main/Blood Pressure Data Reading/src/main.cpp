/*
Blood Pressure Data Reading
By Kevin Sukher
*/

#include <mbed.h>
#include <USBSerial.h>
#include <I2C.h>
#define ADDR 0x18       // I2C address
#define pMin 0          // Pressure min
#define pMax 300        // Pressure max
#define outMin 419430   // Output min
#define outMax 3774874  // Output max
#define bufData 3000    // Number of buffer samples

// Defining the serial for output
USBSerial serial;

// Defining the pins for the pullup resistors
DigitalIn sdaPU(PB_9, PullUp);
DigitalIn sclPU(PB_8, PullUp);

// Defining the connection pins for the I2C pressure sensor
I2C i2c(PB_9 , PB_8); // (I2C_SDA, I2C_SCL)

// Constants for read and write addresses
const int addrWrite = 0x30;
const int addrRead = 0x31;

int main() {

  // Setting up separate arrays for writing to and reading from the pressure sensor
  const char dataWrite[] = {0xAA, 0x00, 0x00};
  char dataRead[] = {0, 0, 0, 0};

  // Setting up the buffer for the pressure data
  float buf[bufData];
  
  // Setting the frequency
  i2c.frequency(300000);

  // Setting the pullup resistors
  sdaPU.mode(OpenDrainPullUp);
  sclPU.mode(OpenDrainPullUp);

  while (1) {

    int i = 0;
    int time = 60;

    while(i < bufData) {

      // Setting up the configuration register
      i2c.write(addrWrite, dataWrite, 3);

      // Waiting for write
      wait_ms(10);

      // Reading pressure register
      i2c.read(addrRead, dataRead, 4);

      // Converting read data for printing by shifting the data and using the datasheet formula to calculate pressure in mmHg
      float output = float((dataRead[1] << 16 | dataRead[2] << 8 | dataRead[3]));
      float pressure = ((output - outMin) * (pMax - pMin) / (outMax - outMin)) + pMin;

      // Adding the pressure data to the buffer for printing
      buf[i] = pressure;

      // Printing pressure data every 50 samples with remaining time interval before the whole buffer is printed
      if (i % 50 == 0) {
        serial.printf("Pressure (Displayed every x50) = %3.2f (%d intervals)\n\r", buf[i], time);
        time--;
      }

      i++;
    }

    // Printed data in array format for Python processing
    // To be copy and pasted into ipynb file for results
    serial.printf("------------------- Pressure Data --------------------\n");
    for(int x = 0; x < bufData - 1; x++) {
      serial.printf("%3.2f, ", buf[x]);
    }
    
    serial.printf("%3.2f\n", buf[bufData - 2]);
  }
}
