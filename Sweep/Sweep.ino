/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo1;  // create Servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;

// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo1.attach(2);  // attaches the servo on pin 9 to the Servo object
  myservo2.attach(3);
  myservo3.attach(5);
  myservo4.attach(6);

  myservo1.write(90);
  delay(2000);
  myservo2.write(90);
  delay(2000);
  myservo3.write(-173);
  delay(2000);
  myservo4.write(173);
}

void loop() {
  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo2.write(pos);
  //   myservo1.write(pos);   
  //   myservo3.write(90);        // tell servo to go to position in variable 'pos'
  //   delay(30);       
  //              // waits 15 ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo2.write(pos);
  //   myservo1.write(pos); 
  //   myservo3.write(90);              // tell servo to go to position in variable 'pos'
  //   delay(30);                       // waits 15 ms for the servo to reach the position
  // }
    if(Serial.available()){
      String input = Serial.readStringUntil('\n');
      input.trim();

      int angles[5];
      int index = 0;

      while (input.length() > 0 && index < 5) {
        int commaIndex = input.indexOf(',');
        String part;
        if (commaIndex == -1) {
          part = input;
          input = "";
        } else {
          part = input.substring(0, commaIndex);
          input = input.substring(commaIndex + 1);
        }

        angles[index] = part.toInt();
        index++;
      }
      Serial.println(angles[0]);
      Serial.println(angles[1]);
      Serial.println(angles[2]);
      Serial.println(angles[3]);
      Serial.println(angles[4]);
    }
  }
