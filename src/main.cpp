/**************************************************************************
 *  Includes and Definitions
 **************************************************************************/
#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"
#include <TaskScheduler.h>

#include "swc_isa_shunt.h"

// Define GVRET_PORT and MONITOR_PORT.
// GVRET communication uses the primary Serial port.
// Monitoring/debug output uses USBSerial1.
#define GVRET_PORT Serial // USB Enhanced Serial CH343
#define MONITOR_PORT USBSerial1

// USB Serial Setup: Use a clear name for the USB CDC object.
USBCDC USBSerial1(0); // First virtual serial port

#include "gvret.h"
#include "canmanager.h"

// RGB LED Config (ESP32-S3 Built-in)
// #define RGB_BUILTIN    48   // Built-in LED pin on ESP32-S3
// #define RGB_BRIGHTNESS 0    // 0 for OFF

Scheduler runner;
Shunt_IVTS shunt(/*little_endian=*/false);

/**************************************************************************
 *  Function Prototypes
 **************************************************************************/
void passthroughCAN();
void blinkLED();
void printStatus();

/**************************************************************************
 *  Task Definitions
 **************************************************************************/
Task taskBlinkLED(500, TASK_FOREVER, &blinkLED, &runner, true);
Task taskPrintStatus(500, TASK_FOREVER, &printStatus, &runner, true);

/**************************************************************************
 *  Passthrough CAN
 **************************************************************************/
void passthroughCAN()
{
    CANMessage frame;

    // -------------------------------------------------------------
    // Handle messages from CAN1 - Motor CAN bus
    // -------------------------------------------------------------
    if (can.available())
    {
        can.receive(frame);
        can2.tryToSend(frame);
        sendFrameToUSB(frame, 0);

        shunt.DecodeCAN(frame);
    }

    // -------------------------------------------------------------
    // Handle messages from CAN2 - Vehicle CAN bus
    // -------------------------------------------------------------
    if (can2.available())
    {
        can2.receive(frame);
        can.tryToSend(frame);
        sendFrameToUSB(frame, 1);
    }
}

// Task Function: Toggle the built-in LED.
void blinkLED()
{
    digitalWrite(RGB_BUILTIN, !digitalRead(RGB_BUILTIN));
}

/**************************************************************************
 *  Prints out the status
 **************************************************************************/
void printStatus()
{
    MONITOR_PORT.printf("I=%.2f A  U1=%.2f V  T=%.1f C  W=%.1f\n",
                        shunt.current_A(),
                        shunt.u1_V(),
                        shunt.temp_C(),
                        shunt.power_W());
}

/**************************************************************************
 *  Setup and Loop
 **************************************************************************/
void setup()
{
    // Initialize the primary Serial port for GVRET communication.
    Serial.begin(1000000);

    // Initialize USB CDC for monitoring/debug output.
    USBSerial1.begin();
    USB.begin();

    // Configure the built-in RGB LED.
    pinMode(RGB_BUILTIN, OUTPUT);
    digitalWrite(RGB_BUILTIN, LOW);

    // Initialize the CAN buses using the CAN manager.
    canManager_setup();

    // Optionally, print a startup message.
    MONITOR_PORT.println("System Initialized. Starting tasks...");
}

void loop()
{
    runner.execute();
    passthroughCAN();
    gvret_loop();
}