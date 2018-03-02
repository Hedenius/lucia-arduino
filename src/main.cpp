#include <Arduino.h>
#include <CurieBLE.h>

#define UUID4_SERVICE "d68b43f7-dbdf-4496-badb-0c59f8e7a5ac"
#define UUID4_DISTANCE "250416ca-a580-4a39-959d-32bdab46403b"
#define UUID4_LED "5377a75b-0b55-41f2-a415-bcf8e7510921"

#define LED_PIN  13 // Built in LED-pin, used to show that BLE central is connected.
#define TRIG_PIN 12 // Arduino pin connected to sensor trigger pin
#define ECHO_PIN 11 // Arduino pin connected to sensor echo pin

#define HCSR04_MAX_DISTANCE_CM 400 // Max distance according to datasheet
#define MAX_DISTANCE_CM 200 // Max distance we want to ping for

#define MICROS_ROUNDTRIP_CM 58.2 // Time it takes for sound to travel 2cm in micros

#define NO_ECHO 0 // Default value for no sensor reading
#define LED_OFF 0

#define DISTANCE_POLL_TIME_MS 250 // Poll sensor 4 times a second

// Time in micros to wait after a digital write to make sure pin is low or high. 
#define WAIT_FOR_LOW 4
#define WAIT_FOR_HIGH 10

unsigned int pingDistanceCm(); // Prototype needed if using .cpp instead of .ino
void turnOnLeds();
void turnOffLeds();

// Variables for distance sensor
unsigned long max_echo_time = MAX_DISTANCE_CM * MICROS_ROUNDTRIP_CM; // micros
unsigned long previousPollTime = 0; // millis
unsigned int oldDistance = 0;

// Variables for BLE
BLEPeripheral blePeripheral;
BLEService luciaService(UUID4_SERVICE);
BLEUnsignedIntCharacteristic distanceCharacteristic(UUID4_DISTANCE, BLERead | BLENotify);
BLEUnsignedCharCharacteristic ledCharacteristic(UUID4_LED, BLERead | BLEWrite);

void setup() {
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    blePeripheral.setLocalName("Lucia");
    blePeripheral.setAdvertisedServiceUuid(luciaService.uuid());

    blePeripheral.addAttribute(luciaService);
    blePeripheral.addAttribute(distanceCharacteristic);
    blePeripheral.addAttribute(ledCharacteristic);

    distanceCharacteristic.setValue(NO_ECHO);
    ledCharacteristic.setValue(LED_OFF);

    blePeripheral.begin();
    Serial.println("Bluetooth active, waiting for connections.");
}

void loop() {
    blePeripheral.poll(); // not sure if this is needed if we're not using an event based approach...
    
    BLECentral central = blePeripheral.central();
    
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_PIN, HIGH);
        
        while (central.connected()) {
            long currentTime = millis();
            
            if (currentTime - previousPollTime >= DISTANCE_POLL_TIME_MS) {
                previousPollTime = currentTime;

                unsigned int distance = pingDistanceCm();
                
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.println("cm");

                if (distance != oldDistance) { 
                    distanceCharacteristic.setValue(distance);
                    oldDistance = distance;
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
        }

        digitalWrite(LED_PIN, LOW);
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
    Serial.println("LEDS ON");
}

void turnOffLeds() {
    Serial.println("LEDS OFF");
}

