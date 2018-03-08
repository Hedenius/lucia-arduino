#include <Arduino.h>
#include <CurieBLE.h>

#define UUID4_SERVICE "d68b43f7-dbdf-4496-badb-0c59f8e7a5ac"
#define UUID4_DISTANCE "250416ca-a580-4a39-959d-32bdab46403b"
#define UUID4_LED "5377a75b-0b55-41f2-a415-bcf8e7510921"
#define UUID4_VIBRATE "c88f5ba0-dee3-4d6c-8e33-38ad5261cc85"
#define UUID4_SENSOR "b4ded119-98a3-4a4c-a001-015ce5142cc1"
#define UUID4_LIGHT "77198360-91bb-421b-9626-564a5c3704f8"

#define BUILTIN_LED_PIN 13 // Built in LED-pin, used to show that BLE central is connected.
#define TRIG_PIN 11 // Arduino pin connected to sensor trigger pin
#define ECHO_PIN 12 // Arduino pin connected to sensor echo pin
#define LED_PIN 10 // Arduino pin connected to led
#define PHOTOCELL_ANALOG_PIN 0 // Arduino analog connected to photocell 

#define HCSR04_MAX_DISTANCE_CM 400 // Max distance according to datasheet
#define MAX_DISTANCE_CM 200 // Max distance we want to ping for

#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5.0
#define REF_RESISTANCE 1000

#define MICROS_ROUNDTRIP_CM 58.2 // Time it takes for sound to travel 2cm in micros

#define NO_ECHO 0 // Default value for no sensor reading
#define OFF 0
#define ON 1
#define LIGHT 0
#define DARK 1

#define LIGHT_RAW_THRESHOLD 200

#define DISTANCE_POLL_TIME_MS 250 // Poll sensor 4 times a second
#define LIGHT_POLL_TIME_MS 1000 // Poll photocell every 5 seconds

// Time in micros to wait after a digital write to make sure pin is low or high. 
#define WAIT_FOR_LOW 4
#define WAIT_FOR_HIGH 10

unsigned int pingDistanceCm(); // Prototype needed if using .cpp instead of .ino
void turnOnLeds();
void turnOffLeds();
void turnOnVibration();
void turnOffVibration();

unsigned char isSensorOn = ON;

// Variables for distance sensor
unsigned long max_echo_time = MAX_DISTANCE_CM * MICROS_ROUNDTRIP_CM; // micros
unsigned long previousDistancePollTime = 0; // millis
unsigned long previousLightPollTime = 0;

unsigned int oldDistance = 0;
unsigned int oldPhotocellValue = 0;

// Variables for BLE
BLEPeripheral blePeripheral;
BLEService luciaService(UUID4_SERVICE);
BLEUnsignedIntCharacteristic distanceCharacteristic(UUID4_DISTANCE, BLERead | BLENotify);
BLEUnsignedCharCharacteristic ledCharacteristic(UUID4_LED, BLERead | BLEWrite | BLENotify);
BLEUnsignedCharCharacteristic vibrateCharacteristic(UUID4_VIBRATE, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic sensorCharacteristic(UUID4_SENSOR, BLERead | BLEWrite | BLENotify);
BLEUnsignedCharCharacteristic lightCharacteristic(UUID4_LIGHT, BLERead | BLEWrite | BLENotify);

void setup() {
    Serial.begin(9600);

    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(PHOTOCELL_ANALOG_PIN, INPUT);

    blePeripheral.setLocalName("Lucia");
    blePeripheral.setAdvertisedServiceUuid(luciaService.uuid());

    blePeripheral.addAttribute(luciaService);
    blePeripheral.addAttribute(distanceCharacteristic);
    blePeripheral.addAttribute(ledCharacteristic);
    blePeripheral.addAttribute(vibrateCharacteristic);
    blePeripheral.addAttribute(sensorCharacteristic);
    blePeripheral.addAttribute(lightCharacteristic);

    distanceCharacteristic.setValue(NO_ECHO);
    ledCharacteristic.setValue(OFF);
    vibrateCharacteristic.setValue(OFF);
    sensorCharacteristic.setValue(ON);
    lightCharacteristic.setValue(LIGHT);

    blePeripheral.begin();
    Serial.println("Bluetooth active, waiting for connections.");
}

void loop() {
    blePeripheral.poll(); // not sure if this is needed if we're not using an event based approach...
    
    BLECentral central = blePeripheral.central();
    
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(BUILTIN_LED_PIN, HIGH);
        
        while (central.connected()) {
            long currentTime = millis();
            
            if (isSensorOn) {
                if (currentTime - previousDistancePollTime >= DISTANCE_POLL_TIME_MS) {
                    previousDistancePollTime = currentTime;

                    unsigned int distance = pingDistanceCm();
                    
                    Serial.print("Distance: ");
                    Serial.print(distance);
                    Serial.println("cm");

                    distanceCharacteristic.setValue(distance);

                    //if (distance != oldDistance) { 
                        //distanceCharacteristic.setValue(distance);
                        //oldDistance = distance;
                    //}
                }
            }
            
            if (currentTime - previousLightPollTime >= LIGHT_POLL_TIME_MS) {
                previousLightPollTime = currentTime;

                unsigned int photocellValue = analogRead(PHOTOCELL_ANALOG_PIN);

                float resistorVoltage = (float) photocellValue / MAX_ADC_READING * ADC_REF_VOLTAGE;
                float ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;
                float ldrResistance = ldrVoltage/resistorVoltage * REF_RESISTANCE;
                
                if (oldPhotocellValue != photocellValue) {
                    if (photocellValue < LIGHT_RAW_THRESHOLD) {
                        lightCharacteristic.setValue(DARK);
                    }
                    else {
                        lightCharacteristic.setValue(LIGHT);
                    }
                }
            }
            
            if (sensorCharacteristic.written()) {
                isSensorOn = sensorCharacteristic.value();
                if (isSensorOn) {
                    Serial.println("SENSOR ON");
                }
                else {
                    Serial.println("SENSOR OFF");
                }
            }

            if (ledCharacteristic.written()) {
                if (ledCharacteristic.value()) {
                    turnOnLeds();
                }
                else {
                    turnOffLeds();
                }
            }

            if (vibrateCharacteristic.written()) {
                if (vibrateCharacteristic.value()) {
                    turnOnVibration();
                }
                else {
                    turnOffVibration();
                }
            }
        }

        digitalWrite(BUILTIN_LED_PIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}

unsigned int pingDistanceCm() {
    // Trigger sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(WAIT_FOR_LOW);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(WAIT_FOR_HIGH);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for pulse and get time in micros
    unsigned long duration = pulseIn(ECHO_PIN, HIGH);

    return duration > max_echo_time ? NO_ECHO : duration / MICROS_ROUNDTRIP_CM;
}

void turnOnLeds() {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LEDS ON");
}

void turnOffLeds() {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LEDS OFF");
}

void turnOnVibration() {
    Serial.println("VIBRATION ON");
}

void turnOffVibration() {
    Serial.println("VIBRATION OFF");
}

