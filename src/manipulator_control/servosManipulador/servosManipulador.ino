#include <Servo.h>  // librería para poder controlar el servo

Servo servo1;   // Crea un objeto servo llamado servo1 (garra)
Servo servo2;   // Crea un objeto servo llamado servo2 (brazo)
Servo servo3;   // Crea un objeto servo llamado servo3 (antebrazo)
Servo servo4;   // Crea un objeto servo llamado servo4 (base)

int pos1 = 0; // Posicion del servo del joint #1
int pos2 = 180; // Posicion del servo del joint #2
int pos3 = 270; // Posicion del servo del joint #3
int pos4 = 0; // Posicion del servo del joint #4

int pinservo1 = 5;
int pinservo2 = 6;
int pinservo3 = 7;
int pinservo4 = 8;

int incomingByte = 0;  // for incoming serial data
int value1, value2, value3;
char recibido[20];

void setup(){ 
  Serial.begin(9600);

  servo1.attach(pinservo1, 660, 2600); // Servo del joint #1 asociado al pin PWM5
  servo2.attach(pinservo2, 660, 2600); // Servo del joint #2 asociado al pin PWM6
  servo3.attach(pinservo3, 660, 2600); // Servo del joint #3 asociado al pin PWM7
  servo4.attach(pinservo4, 660, 2600); // Servo del joint #4 asociado al pin PWM8

  pinMode(pinservo1, OUTPUT);
  pinMode(pinservo2, OUTPUT);
  pinMode(pinservo3, OUTPUT);
  pinMode(pinservo4, OUTPUT);
}
 
void loop(){
  if (Serial.available()) {
    delay(10);
    // Read input from serial
    String receivedString = Serial.readStringUntil('\n');
    
    // Tokenize the received string
    char* token = strtok(const_cast<char*>(receivedString.c_str()), ",");
    
    // Loop through all tokens
    for (int i = 0; i < 3; i++) {
      // Convert the token to an integer value
      int value = atoi(token);
      
      // Assign the value to the appropriate variable
      switch (i) {
        case 0:
          value1 = value;
          break;
        case 1:
          value2 = value;
          break;
        case 2:
          value3 = value;
          break;
      }
      
      // Get the next token
      token = strtok(NULL, ",");
    }

    if (value1 = 1.0 && value2 == 1.0 && value3 == 1.0) 
    {
      pinza(0, 180);
    }
    else if (value1 > 0.0 && value2 == 0.0 && value3 == 0.0) 
    {
      antebrazo_antihorario(value1);
    }
    else if (value1 < 0.0 && value2 == 0.0 && value3 == 0.0) 
    {
      antebrazo_horario(abs(value1));
    }
    else if (value1 == 0.0 && value2 > 0.0 && value3 == 0.0) 
    {
      brazo_antihorario(value2);
    }
    else if (value1 == 0.0 && value2 < 0.0 && value3 == 0.0) 
    {
      brazo_horario(abs(value2));
    }
    else if (value1 == 0.0 && value2 == 0.0 && value3 > 0.0) 
    {
      base_antihorario(value3);
    }
    else if (value1 == 0.0 && value2 == 0.0 && value3 < 0.0) 
    {
      base_horario(abs(value3));
    }
  }
}

// Procedimiento que abre y cierra la pinza para agarrar un objeto o soltarlo
void pinza(int inicio, int fin){
  for (int i = inicio; i <= fin; i++){
    pos1 = i;
    servo1.write(pos1);
    delay(10);
  }
  delay(2000);
  for (int i = fin; i >= inicio; i--){
    pos1 = i;
    servo1.write(pos1);
    delay(10);
  }
}

// Procedimiento que mueve el antebrazo en sentido antihorario
void antebrazo_antihorario(int velocidad){
  while (pos2 >= 0){
    servo2.write(pos2);
    pos2 --;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}

// Procedimiento que mueve el antebrazo en sentido horario
void antebrazo_horario(int velocidad){
  while (pos2 <= 180){
    servo2.write(pos2);
    pos2 ++;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}

// Procedimiento que mueve el brazo en sentido antihorario
void brazo_antihorario(int velocidad){
  while (pos3 >= 0){
    servo3.write(pos3);
    pos3 --;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}

// Procedimiento que mueve el brazo en sentido horario
void brazo_horario(int velocidad){
  while (pos3 <= 180){
    servo3.write(pos3);
    pos3 ++;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}

// Procedimiento que mueve la base en sentido antihorario
void base_antihorario(int velocidad){
  while (pos4 >= 0){
    servo4.write(pos4);
    pos4 --;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}

// Procedimiento que mueve la base en sentido horario
void base_horario(int velocidad){
  while (pos4 <= 180){
    servo4.write(pos4);
    pos4 ++;
    // La velocidad máxima es de 600deg/s -> delay = 60/0.06v - 0.10/0.06
    float d = (60/(0.06*velocidad)) - 5/3;
    delay(d);
  }
}
