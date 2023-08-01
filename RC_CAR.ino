#include <IRremote.h>
#include <Servo.h>

// Define the IR receiver pin
const int receiverPin = 2;

// Create an instance of the IRremote library
IRrecv irReceiver(receiverPin);

// Create a variable to store the received IR code
decode_results irResults;

// Create a servo object for ESC control
Servo esc;

int speed = 1000;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  Serial.print("hello world");
  irReceiver.enableIRIn();  // Enable the IR receiver
  esc.attach(9);  // Attach the ESC control to pin 9
  esc.writeMicroseconds(1000);
  delay(2000);
}

void loop() {
  if (irReceiver.decode(&irResults)) {  // Check if a code is received
    unsigned long irCode = irResults.value;  // Get the received code

    // Print the received code to the serial monitor
    Serial.print("Received IR code: ");
    Serial.println(irCode, HEX);

    // Control the ESC based on the received code
    if (irCode == 0xFA05FF00 || irCode == 0xF664D7BF) {
      if (speed < 2000) {
        speed += 20;
      }  // Example code for a specific button
      // Set the ESC to a specific speed or action
    } else if(irCode == 0x9CB77FBB) {
      if (speed > 1000) {
        speed -= 20;
      }
    }
    Serial.print(speed);
    esc.writeMicroseconds(speed);    
    // Add more conditions to control the ESC based on different IR codes

    irReceiver.resume();
    delay(100);  // Enable receiving the next IR code
  }
}
