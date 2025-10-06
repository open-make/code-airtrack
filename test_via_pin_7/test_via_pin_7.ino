// Pin where the LED is connected
int ledPin = 7; 

void setup() {
  // Initialize the digital pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn the LED on
  digitalWrite(ledPin, HIGH);
  // Wait for 3 seconds (3000 milliseconds)
  delay(3000);  
  
  // Turn the LED off
  digitalWrite(ledPin, LOW);
  // Wait for another 3 seconds (3000 milliseconds)
  delay(3000); 
}
