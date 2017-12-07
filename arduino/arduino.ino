#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper MotorDEC(1, 9, 8);
AccelStepper MotorRA(1, 6, 7);


float ra_sirius = 101.28;
float dec_sirius = 343.277;
float ra_rigel = 78.63;
float dec_rigel = 351.789;
float ra = 100;
float dec = 200;

bool star = true;
int steps_ra;
int steps_dec;
int negative;
int counter = 0;

String ra_str = "nada";
String dec_str = "nada2";
String data;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  MotorRA.setMaxSpeed(200);
  MotorRA.setAcceleration(30); 
  MotorDEC.setMaxSpeed(200);
  MotorDEC.setAcceleration(30);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String command = Serial.readString();
    if (command == "coords"){
      send_coords();
    }
    else if (command == "position"){
      send_position();
    }
    else if (command == "move"){
      move_motor();
    }
    else if (command == "new"){
      ra = Serial.read();
      dec = Serial.read();
    }
  }
}

void send_coords() {

  Serial.println("coords");
  if (star==true){
    Serial.println(ra_sirius);
    Serial.println(dec_sirius);
    star = false;
  }
  else {
    Serial.println(ra_rigel);
    Serial.println(dec_rigel);
    star = true;
  }
}

void send_position() {
  Serial.println("position");
  Serial.println(ra);
  Serial.println(dec);
}

void move_motor(){
  Serial.println("move_ready");
  while (!(Serial.available()));
  data = Serial.readString();
  parse(data);
  Serial.println("debug");
  Serial.println(ra_str);
  Serial.println("debug");
  Serial.println(dec_str);
  MotorRA.move(ra);
  MotorDEC.move(dec);
  MotorRA.run();
  MotorDEC.run();  
}


// void parse(String data){
//   char *ptr = strtok(data," ");
//   ra_str = *ptr;
//   ptr++;
//   dec_str = *ptr;
// }

  void parse(String data){
    int index = data.indexOf(' ');
    ra_str = data.substring(0,index);
    dec_str = data.substring(index);
  }



