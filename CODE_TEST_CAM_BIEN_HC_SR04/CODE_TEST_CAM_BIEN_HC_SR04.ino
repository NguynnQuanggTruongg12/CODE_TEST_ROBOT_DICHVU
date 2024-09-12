
//==================RSR_01========================//
// const int echo[7] = {40, 38, 37, 35, 43, 44, 46}; // echo 
// const int trig[7] = {41, 39, 36, 34, 42, 45, 47}; // trig 


//==================RSR_02========================//
const int echo[7] = {7, 32, 38, 44, 42, 51, 50}; // echo 
const int trig[7] = {6, 33, 39, 30, 40, 52, 53}; // trig 




void setup() {
  // Initialize the Serial Monitor
  Serial.begin(9600);
  for (int i = 0; i < 7; i++) {
    pinMode(trig[i], OUTPUT); 
    pinMode(echo[i], INPUT);
  }
}

void readSensor(int echoPin, int trigPin) {
  long duration, distance;
  
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2;

  // Print the distance on the Serial Monitor
  Serial.print("Khoảng cách: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    Serial.print("Cám biến:");
    Serial.println(input);
    if (input >= '1' && input <= '7') {
      int sensorIndex = input - '1'; // Convert char to int
      while (true) {
        readSensor(echo[sensorIndex], trig[sensorIndex]);
        delay(300);  // Adjust the delay as needed
        if (Serial.available() > 0) {
          char stopInput = Serial.read();
          if (stopInput == '0') {
            Serial.println("====STOP====");
            delay(100); // Small delay to ensure Serial buffer is cleared
            while (Serial.available() > 0) {
              Serial.read(); // Clear the Serial buffer
            }
            break; // Exit the continuous reading loop
          }
        }
      }
    }
  }
}


