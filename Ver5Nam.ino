#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 12); // RX, TX

#define IN1	4
#define IN2	5
#define IN3	6
#define IN4	7
#define MAX_SPEED 255 //từ 0-255
#define MIN_SPEED 0
#define right 8
#define mid 9
#define left 10

int leftValue = 0;
int midValue = 0;
int rightValue = 0;

int error, previous_error;

float Kp = 0;
float Ki = 0;
float Kd = 0;


void setup()
{
  mySerial.begin(9600);
  // Serial.begin(9600);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  digitalWrite(A5, HIGH);
}
 
void motor_Trai_Dung() {
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
}
 
void motor_Phai_Dung() {
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}
 
void motor_Phai_Tien(int speed) { //speed: từ 0 - MAX_SPEED
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	digitalWrite(IN1, HIGH);// chân này không có PWM
	analogWrite(IN2, 255 - speed);
}
 
void motor_Phai_Lui(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	digitalWrite(IN1, LOW);// chân này không có PWM
	analogWrite(IN2, speed);
}
 
void motor_Trai_Tien(int speed) { //speed: từ 0 - MAX_SPEED
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	analogWrite(IN3, speed); //dau nguoc
	digitalWrite(IN4, LOW);// chân này không có PWM
}
 
void motor_Trai_Lui(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	analogWrite(IN3, 255 - speed);
	digitalWrite(IN4, HIGH);// chân này không có PWM
}

void re_Trai(int speed, int PID_value){
  motor_Phai_Tien(speed + PID_value); // motor 1 tiến
	motor_Trai_Lui(speed + PID_value); // motor 1 tiến
}

void re_Phai(int speed, int PID_value){
  motor_Trai_Tien(speed + PID_value); // motor 1 tiến
	motor_Phai_Lui(speed + PID_value);
}

void di_Thang(int speed){
  motor_Trai_Tien(speed); // motor 1 tiến
	motor_Phai_Tien(speed);
}

void di_Lui(int speed){
  motor_Trai_Lui(speed); // motor 1 tiến
	motor_Phai_Lui(speed);
}

void dung_Xe(){
  motor_Trai_Dung();
	motor_Phai_Dung();
}
 


int read_Sensor_Values(){
  //Tốc độ rẽ
  int speed = 100;

  leftValue = digitalRead(left); //Luu lich su trai
  midValue = digitalRead(mid); //Luu lich su giua
  rightValue = digitalRead(right); //Luu lich su phap

  /* 
  * error < 0 : Lech Trai
  * error = 0 : Khong lech
  * error > 0 : Lech Phai
  */

  if((leftValue == 0)&&(midValue == 0)&&(rightValue == 0)){// Truong hop lech khoi Vach Den
    if(error > 0){
      error = 3;
    } else if(error < 0){
      error = -3;
    }else if(error == 0){
      error = 0;
    }
  }else if((leftValue == 0)&&(rightValue == 1)){
    error = 2;  
  }else if((leftValue == 0)&&(rightValue == 0)){
    error = 0;
  }else if((leftValue == 0)&&(rightValue == 1)){
    error = 1;
  }else if((leftValue == 1)&&(midValue == 0)&&(rightValue == 0)){
    error = -2;   
  }else if((leftValue == 1)&&(rightValue == 1)){
    // di_Lui(speed);
  }else if((leftValue == 1)&&(midValue == 1)&&(rightValue == 0)){
    error = 0;



    
  }else if((leftValue == 1)&&(midValue == 1)&&(rightValue == 1)){
    // if(error > 0){
    //   error = 3;
    // } else if(error < 0){
    //   error = -3;
    // }else if(error == 0){
    //   error = 0;
    // }
    error = 0;
  }

  return error;
}

int calculate_pid(int error, int previous_error)
{
  int P;
  float I, D;
   
  P = error;
  I = I + error;
  D = error - previous_error;
  
  int PID_value = (Kp*P) + (Ki*I) + (Kd*D);
  
  return PID_value;
}

void moto_Control(int speed, int PID_value){
  if(error < 0){
    re_Phai(speed, PID_value);
  } else if(error > 0){
    re_Trai(speed, PID_value);
  }else if(error == 0) {
    di_Thang(speed);
  }
}

void do_Duong(){
  previous_error = error;
  error = read_Sensor_Values();
  int PID_value = calculate_pid(error, previous_error);
  moto_Control(180, PID_value);
}

void BlueTooh_run(char signal){
  switch (signal) {
    case 'w':
      di_Thang(200);
      delay(500);
      dung_Xe();
      break;
    case 'l':
      re_Trai(200, 0);
      delay(500);
      dung_Xe();
      break;
    case 'r':
      re_Phai(200, 0);
      delay(500);
      dung_Xe();
      break;
    case 'b':
      di_Lui(200);
      delay(500);
      dung_Xe();
      break;
    default:
      dung_Xe();
  }
}

bool doDuong = false;

// void loop(){
//   re_Trai(255, 0);
// }
void loop()
{
  char c;
  if(mySerial.available()){
    c = mySerial.read();
  }

  if(c == 'n'){
    doDuong = true;
  }

  if(c == 'f'){
    doDuong = false;
  }

  if(doDuong){
    do_Duong();
  }else{
    BlueTooh_run(c);
  }
}