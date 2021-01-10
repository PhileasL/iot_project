#include <TinyLoRa.h>
#include <SPI.h>

uint8_t NwkSkey[16] = { "secret" };
uint8_t AppSkey[16] = { "secret" };
uint8_t DevAddr[4] = { "secret" };

TinyLoRa lora = TinyLoRa(3, 8, 4);

const unsigned int sendInterval = 60;

const byte TRIGGER_PIN = 9; // Broche TRIGGER
const byte ECHO_PIN = 6;    // Broche ECHO
long measure = 0.0;
float distance = 0;
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

const float SOUND_SPEED = 340.0 / 1000;

bool car = false;

float flat = 47.582356;
float flng = -122.328654;
long lat = (long)(flat*1e6);
long lng = (long)(flng*1e6);

unsigned char datatest[9];

void setup()
{
  delay(1000);
  Serial.begin(9600);

  // else doesn't start without serial monitor
  int waitcnt = 0;
  while(!Serial && (waitcnt++ < 10))  // wait (only so long) for serial port to connect.
  {
    delay(100);
    digitalWrite(LED_BUILTIN, waitcnt & 1);
  }

  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize LoRa
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  while(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    //while(true);
    delay(5000);
  }
  Serial.println("OK radio success");


  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
}

void loop()
{
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
  distance = measure / 2.0 * SOUND_SPEED;
  distance = distance / 10.0, 2;

  if (distance < 20)
    car = true;
  else 
    car = false;
  Serial.println((String)car );
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

  Serial.println("Sending LoRa Data...");
  lora.sendData(datatest, sizeof(datatest), lora.frameCounter);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("delaying...");
  delay(sendInterval * 1000);

}
