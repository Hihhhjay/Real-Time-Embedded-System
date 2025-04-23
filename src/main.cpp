#include "mbed.h"
#include "./drivers/LCD_DISCO_F429ZI.h"
#include "math.h"
#include "config.h"
#include <cmath> 
//#include "arm_math.h"  // Include CMSIS DSP library
//#include "gyro.h"


SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

//address of first register with gyro data
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

uint8_t write_buf[32]; 
uint8_t read_buf[32];

EventFlags flags;
//The spi.transfer() function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
}


float samples[40][3]; // sampled at the rate of TIME_INTERVALs for 20s; Values in terms of angular velocity(dps)
float linearVelStorage[40][3];

int8_t dataCollected = 0;
int8_t dataSent = 0;

bool SWITCH_STATUS = false;

double globDist; // distance covered in 20 seconds

DigitalIn buttonStatus(SWITCH);
LCD_DISCO_F429ZI lcd;
DigitalOut led1(LED1,0); 
DigitalOut led2(LED2,1); 
static BufferedSerial serial_port(USBTX, USBRX);

// Gyro related functions
int Gyro_Init();
void Gyro_Get_XYZ(float xyz[]);

int main()
{


  serial_port.set_baud(115200);
  //GyroCs = 1;
  // spi.format(8, 3);
  // spi.frequency(1000000);

  int Gyro_ID;

  float GyroXYZ[3]; // buffer for storing 3 bytes X, Y, Z
  float linearVelocity[3];

  int sampleIndex = 0;

  //Gyro_ID = Gyro_Init();
    //printf("Gyro_ID: %d\n", Gyro_ID);

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


  // LCD related initialization
  //////////////////////////////////////////////////////////////////////////
  // lcd.SetBackColor(LCD_COLOR_RED);
  lcd.Clear(LCD_COLOR_RED);
  lcd.SetBackColor(LCD_COLOR_RED);
  BSP_LCD_SetFont(&Font20);
  lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Press The", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Blue Button", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"To Start", CENTER_MODE);

  wait_us(1000000);

  char x_msg[25] = {0};
  char y_msg[25] = {0};
  char z_msg[25] = {0};

  char x_vel[25] = {0};
  char y_vel[25] = {0};
  char z_vel[25] = {0};

  char Tremor_display[25] = {0};
  char time_display[10] = {0};
  /////////////////////////////////////////////////////////////////////////

  while (!SWITCH_STATUS)
  {
    if (buttonStatus.read() == 1)
    {
      SWITCH_STATUS = true;
      led2 = 0;
      led1 = 1;
      lcd.ClearStringLine(5);
      lcd.ClearStringLine(12);
      lcd.ClearStringLine(7);
      lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Angular Velocity", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Linear Velocity", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Ellapsed Time", CENTER_MODE);
      BSP_LCD_SetFont(&Font16);
      lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"0 s", CENTER_MODE);
    }
  }
  while (1)
  {
   // Gyro_Get_XYZ(GyroXYZ);
    /*---------------------------------------------------------------------*/
    int16_t raw_gx,raw_gy,raw_gz;
    float gx, gy, gz;
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[12] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t",raw_gx,raw_gy,raw_gz);

      GyroXYZ[0]=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      GyroXYZ[1]=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      GyroXYZ[2]=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);

      printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",GyroXYZ[0],GyroXYZ[1],GyroXYZ[2]);


    if (sampleIndex == 0)
    {
      linearVelocity[0] = 0;
      linearVelocity[1] = 0;
      linearVelocity[2] = 0;
    }
    else
    {
      // linear velociy = angular velocity * radius
      linearVelocity[0] = (samples[sampleIndex - 1][0] - GyroXYZ[0]) * (RADIUS_X * 0.01);
      linearVelocity[1] = (samples[sampleIndex - 1][1] - GyroXYZ[1]) * (RADIUS_Y * 0.01);
      linearVelocity[2] = (samples[sampleIndex - 1][2] - GyroXYZ[2]) * (RADIUS_Z * 0.01);
    }

    if (sampleIndex < 12)
    {
      // Accumulate data till 6s
      samples[sampleIndex][0] = GyroXYZ[0];
      samples[sampleIndex][1] = GyroXYZ[1];
      samples[sampleIndex][2] = GyroXYZ[2];

      linearVelStorage[sampleIndex][0] = linearVelocity[0];
      linearVelStorage[sampleIndex][1] = linearVelocity[1];
      linearVelStorage[sampleIndex][2] = linearVelocity[2];
    }

    if (sampleIndex == 12)
    {
      led1 = 0;
      led2 = 1;
      for (int i = 1; i < 12; i++)
      {
        float x_dist, y_dist, z_dist;
        // distance = linear velocity * time taken
        x_dist = linearVelStorage[i][0] * TIME_INTERVAL;
        y_dist = linearVelStorage[i][1] * TIME_INTERVAL;
        z_dist = linearVelStorage[i][2] * TIME_INTERVAL;

        // calculate distance travelled in 0.5s interval
        double dist = sqrt((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist));

        globDist += dist;

        // LCD Display
        // lcd.Clear(LCD_COLOR_WHITE);
        lcd.Clear(LCD_COLOR_RED);
        BSP_LCD_SetFont(&Font16);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Tremor", CENTER_MODE);
        
        double frequency;
        if (globDist > 0.6) {          
            frequency = 10 * (globDist - 0.1) + 1;
        } else if (globDist >= 0.1) {
            frequency = 10 * (globDist - 0.1) + 1;
        } else {
            frequency = 1;
        }
        int displayFrequency = static_cast<int>(ceil(frequency));

        sprintf(Tremor_display, "%d C/s", displayFrequency);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)Tremor_display, CENTER_MODE);

        //Judging tremor
        if (frequency >= 3.0) {
            BSP_LCD_SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Tremor frequency", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"is too high", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Dangerous", CENTER_MODE);
        } else {
            BSP_LCD_SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Tremor frequency", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"is safe", CENTER_MODE);
        }

        BSP_LCD_SetFont(&Font16);
        lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"Press black", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"button to restart", CENTER_MODE);

      }

      // calibration section
      globDist -= 0.035; // offset
      if (globDist < 0)
      {
        globDist = 0;
      }
      globDist /= 0.165; // scaling
      printf("globDist = %f\n", globDist);

      dataCollected = 1;
      sampleIndex++;
    }

    if (dataCollected == 1 && dataSent == 0)
    {
      // Send the data accumulated for 20s through UART
      char message[30] = {0};
      for (int i = 0; i < 40; i++)
      {
        sprintf(message, "%2d sample:\nx value: %4.5f\ny value: %4.5f\nz value: %4.5f\n\n\0", i, samples[i][0], samples[i][1], samples[i][2]);
        int j = 0;
        while (message[j] != '\0')
        {
          serial_port.write(&message[j], 1);
          j++;
        }
        dataSent = 1;
      }
    }

    // LCD Display
    /////////////////////////////////////////////////////////////////////////
    if (sampleIndex < 12)
    {
      sprintf(x_msg,"X: %5.2f", GyroXYZ[0]);
      sprintf(y_msg, "Y: %5.2f", GyroXYZ[1]);
      sprintf(z_msg, "Z: %5.2f", GyroXYZ[2]);

      sprintf(x_vel, "X: %5.2f", linearVelocity[0]);
      sprintf(y_vel, "Y: %5.2f", linearVelocity[1]);
      sprintf(z_vel, "Z: %5.2f", linearVelocity[2]);

      sprintf(time_display, "%5.2f s", sampleIndex * TIME_INTERVAL);

      lcd.ClearStringLine(3);
      lcd.ClearStringLine(4);
      lcd.ClearStringLine(5);
      // char ang_out = ;
      lcd.DisplayStringAt(0, LINE(3), (uint8_t *)x_msg, CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(4), (uint8_t *)y_msg, CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)z_msg, CENTER_MODE);

      lcd.ClearStringLine(9);
      lcd.ClearStringLine(10);
      lcd.ClearStringLine(11);

      lcd.DisplayStringAt(0, LINE(9), (uint8_t *)x_vel, CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(10), (uint8_t *)y_vel, CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(11), (uint8_t *)z_vel, CENTER_MODE);

      lcd.ClearStringLine(16);
      lcd.DisplayStringAt(0, LINE(16), (uint8_t *)time_display, CENTER_MODE);
    }
    /////////////////////////////////////////////////////////////////////////

    sampleIndex++;

    wait_us(500000);
  }
}
