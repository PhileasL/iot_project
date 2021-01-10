#include <TinyLoRa.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_SleepyDog.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

uint8_t NwkSkey[16] = { "secret" };
uint8_t AppSkey[16] = { "secret" };
uint8_t DevAddr[4] = { "secret" };

TinyLoRa lora = TinyLoRa(3, 8, 4); 
RH_RF95 rf95(RFM95_CS, RFM95_INT); // using RH_RF95 allows us to make the LoRa module sleep

const unsigned int sendInterval = 4; // x16.5s (see l.158)

const byte TRIGGER_PIN = 9; // pin TRIGGER
const byte ECHO_PIN = 6;    // pin ECHO

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const float SOUND_SPEED = 340.0 / 1000;

bool boolStored = false;

unsigned char datatest[9]; // data to transfer to TTN


/*
 * start_LoRa function wakes up the LoRa module
 */
void start_LoRa(){
    
    lora.setChannel(MULTI); // define multi-channel sending
    lora.setDatarate(SF7BW125); //spreading factor 7 and bandwidth of 125 kHz
    while(!lora.begin())
    {
        Serial.println("Failed");
        Serial.println("Check your radio");
        delay(5000);
    }
    Serial.println("OK radio success");
}

void setup()
{
    // Initialize pin LED_BUILTIN as an output
    pinMode(LED_BUILTIN, OUTPUT);

    /*
     * Initialize HC-SR04 pins states
     */
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    // Initialize security pin as an input
    pinMode(12, INPUT);

    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    /* 
     * This loop is a security:
     * When the pin 12 is connected to 3v3, it enters this loop
     * giving us the opportunity to change the program before
     * it falls again in deep sleep mode.
     */
    while (digitalRead(12)){ 
        digitalWrite(13, HIGH);
        delay(300);
        digitalWrite(13, LOW);
        delay(300);
    }

    /*
     * Measuring a distance by HC-SR04
    */
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
    float distance = measure / 2.0 * SOUND_SPEED;
    distance = distance / 10.0, 2;

    /*
     * computing if there is a car or not
    */
    bool car = false;
    if (distance < 20) car = true;

    Serial.println((String)car );

    float flat = 47.582356;
    float flng = -122.328654;

    /*
     * If there is a state switch, 
     * meaning if during the previous awakening there was a car
     * and not in this awakening (and vice-versa), 
     * the µC will enter this statement
     */
    if (car != boolStored){
      
        boolStored = car; // storing the new state

        start_LoRa();

        /*
         * Converts the float into a long keeping 6 figures after the comma
         */
        long lat = (long)(flat*1e6);
        long lng = (long)(flng*1e6);

        /*
         * The following sections fills up the array to send to send to TTN
         * 0 -> the bytes of whether there is a car or not
         * 1 -> 4 : the 4 bytes of the long type latitude
         * 5 -> 8 : the 4 bytes of the long type longitude
         */
        if (car) datatest[0] = 0xff;
        else datatest[0] = 0x00;

        datatest[1] = ( ( lat >> 24) & 0xFF );
        datatest[2] = ( ( lat >> 16 ) & 0xFF );
        datatest[3] = ( ( lat >> 8 ) & 0xFF );
        datatest[4] = ( lat & 0xFF );

        datatest[5] = ( ( lng >> 24 ) & 0xFF );
        datatest[6] = ( ( lng >> 16 ) & 0xFF );
        datatest[7] = ( ( lng >> 8 ) & 0xFF );
        datatest[8] = ( lng & 0xFF );

        Serial.print(" Payload ");
        for (int i = 0; i < sizeof(datatest); i = i + 1) {
            Serial.print((String)datatest[i] + " ");  
        }
        Serial.println();

        /*
         * Sending datatest to TTN.
         * While sending the bytes, the built-in LED turns on.
         * Considering the transfer just takes a fration of a second,
         * the LED just blink one time.
         */
        Serial.println("Sending LoRa Data...");
        digitalWrite(LED_BUILTIN, HIGH);
        lora.sendData(datatest, sizeof(datatest), lora.frameCounter);
        digitalWrite(LED_BUILTIN, LOW);

        Serial.print("Frame Counter: ");
        Serial.println(lora.frameCounter);
        lora.frameCounter++;

        rf95.sleep(); // puts the LoRa module in sleep mode
    }
    
    /*
     * Puts the µC in deep sleep for sendInterval x 16.5 seconds
     */
    int i = 0;
    while (i<sendInterval){
        Watchdog.sleep(60000);
        i++;
    }
}
