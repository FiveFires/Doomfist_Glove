// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
 
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
uint8_t button_state;
long timer_start;
char start_char;

#define SAMPLE_RATE 1000                 // sample frequency in Hz
#define SAMPLE_DELAY (1000/SAMPLE_RATE)  // delay that is needed to send in required frequency

void setup() {

    // CONFIGURE PIN 4 AS INPUT AND ENABLE INTERNAL PULL-UP RESISTOR
    pinMode(4, INPUT_PULLUP);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // NASTAVENI +-16 G ACCELEROMETR
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

    // NASTAVENI +-1000degrees/sec
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    // TOHLE ZAKOMENTUJ POKUD NECHCES FILTRACI
    //accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);

    while(1)
    {
      start_char = receive_char();
      if(start_char == 'R')
      {
        for(uint8_t i = 0; i < 3; i++)
        {
        accelgyro.getAcceleration(&ax, &ay, &az);
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        //Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        //Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        //Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));

        //fake button byte
        Serial.write(0x00);
        delay(SAMPLE_DELAY);
        }
        start_char = 0;
        break;    
      }
    }
}
 
void loop()
{   
    // if timer is reset, start it
    if(timer_start == 0)
    {
      timer_start = millis();
    }
    
    // reading the button state (state of pin 4)
    button_state = digitalRead(4);

    start_char = receive_char();
   
    if(((millis()-timer_start) >= SAMPLE_DELAY) && (start_char == 'R'))
    {
      start_char = 0;
      timer_start = 0;
      
      accelgyro.getAcceleration(&ax, &ay, &az);
      Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
      Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
      Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
      //Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
      //Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
      //Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
      
      if(button_state == 0)
      {
        Serial.write(0x01); // send Click = 1
      }
      else
      {
        Serial.write(0x00); // send Click = 0
      }
    }
} 

char receive_char() {
    if (Serial.available() > 0) {
        return Serial.read();
    }
}

/*
        // display tab-separated accel/gyro x/y/z values
       // Serial.print("a/g:\t");
        int start=millis();Serial.print(start);Serial.print("\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif
    
 */
