#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
#define st 0
#define go 1

#define dead_speed 20
String comdata = "";
int state  = st;
int invFlag = 0;
int numdata[5] = {0};
float yaw;
int x0, y0, targX, targY;
int last_targX = 0, last_targY = 0;
float err_yaw, err_pos;
float last_err_yaw, last_err_pos;
float yaw_output, pos_output;
float pid_yaw_P = 2.2, pid_pos_P = 3.0;
float pid_yaw_I = 0.0, pid_pos_I = 1.0;
float pid_yaw_D = 4.0, pid_pos_D = 0.0;
float err_pos_sum = 0, err_yaw_sum = 0;
float diff_err_pos = 0, diff_err_yaw = 0;
float targYaw = 0;
int index  = 0;
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT); //PWM1
  pinMode(11, OUTPUT); //PWM2
  lcd.init();                      // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  //lcd.print("Hello, world!");
}
char line[100];
void control(int left_speed, int right_speed)
{

  if (left_speed >= 0)
  {
    left_speed += dead_speed;
    if (left_speed > 255) left_speed = 255;
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    analogWrite(10, left_speed);
  }
  else if (left_speed < 0)
  {
    left_speed -= dead_speed;
    if (left_speed < -255) left_speed = -255;
    left_speed = -left_speed;

    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    analogWrite(10, left_speed);
  }

  if (right_speed >= 0)
  {
    right_speed += dead_speed;
    if (right_speed > 255) right_speed = 255;
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    analogWrite(11, right_speed);
  }
  else if (right_speed < 0)
  {
    right_speed -= dead_speed;
    if (right_speed < -255) right_speed = -255;
    right_speed  = -right_speed;

    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    analogWrite(11, right_speed);
  }
}

int lim(int in, int min_num, int max_num)
{
  if (in < min_num) in = min_num;
  else if (in > max_num) in = max_num;
  return in;
}
void loop() {
  //"##数据1,数据2，数据3,数据4,数据5\r"
  //"ST\r"
  // control(90, 90);
  //while(1);
  if (Serial.available() > 0) {
    int numIndex = 0;

    for (int i = 0; i < 5; i++)
      numdata[i] = 0;
    Serial.readBytesUntil('\n', line, 100);
    comdata = line;
    if (comdata[0] == 'S') // 停止
    {
      state = st; //停止
    }
    else  if (comdata[0] == 'w')
      control(255, 255);
    else  if (comdata[0] == 'a')
      control(-100, 100);
    else  if (comdata[0] == 's')
      control(-255, -255);
    else  if (comdata[0] == 'd')
      control(100, -100);
    else //有数据
    {

      for (int i = 0; i < comdata.length() ; i++)
      {

        if (comdata[i] == ',')
        {
          if (invFlag == 1) //前一个数是负数
          {
            numdata[numIndex] *= -1;
            invFlag = 0;
          }
          numIndex++;
        }
        else
        {
          if (comdata[i] == '-')
            invFlag = 1;
          else
            numdata[numIndex] = numdata[numIndex] * 10 + (comdata[i] - '0');
        }
      }
      yaw = numdata[0] / 100;
      x0 = numdata[1];
      y0 = numdata[2];
      targX = numdata[3];
      targY = numdata[4];


      targYaw = -atan2(x0 - targX, y0 - targY) * 57.3;


      // Serial.println(targYaw);
      err_yaw = yaw - targYaw;
      if (err_yaw < -180) err_yaw = err_yaw + 360;
      if (err_yaw > 180) err_yaw = err_yaw - 360;
      //      if ( err_yaw > -120 && err_yaw < 120)
      //        err_yaw_sum += err_yaw;
      if (err_yaw_sum > 3000) err_yaw_sum = 3000;
      else if (err_yaw_sum < -3000) err_yaw_sum = -3000;
      diff_err_yaw = err_yaw - last_err_yaw;
      yaw_output = pid_yaw_P * err_yaw + pid_yaw_I * err_yaw_sum + pid_yaw_D * diff_err_yaw;
      yaw_output = lim(yaw_output, -255, 255);
      last_err_yaw = err_yaw;



      err_pos = sqrt((float)(x0 - targX) * (float)(x0 - targX) + (float)(y0 - targY) * (float)(y0 - targY));
//      if ( err_pos > -0 && err_pos < 10)
//        err_pos_sum += err_pos;
      if (err_pos_sum > 5000) err_pos_sum = 5000;
      else if (err_pos_sum < -5000) err_pos_sum = -5000;
      diff_err_pos = err_pos - last_err_pos;
      pos_output = pid_pos_P * err_pos + pid_pos_I * err_pos_sum + pid_pos_D * diff_err_pos;
      pos_output = lim(pos_output, -255, 255);
      last_err_pos = err_pos;


    }//有目标点


  }//接收到命令

  if (state == st)
  {
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    err_pos_sum = 0;
    err_yaw_sum = 0;
    digitalWrite(13, LOW);
    if (last_targX != targX || last_targY != targY) //目标点发生改变
    {
      err_yaw_sum = 0;
      err_pos_sum = 0;
      state = go;
      last_targX = targX;
      last_targY = targY;
    }
  }
  else if (state == go)
  {
    digitalWrite(13, HIGH);
   
    if (x0 > targX - 20 && x0 < targX + 20 && y0 > targY - 20 && y0 < targY + 20)
      state = st;
    else
      control(0.5 * pos_output - 0.5 * yaw_output, 0.5 * pos_output + 0.5 * yaw_output);
  }

  if (index == 50)
  {
    index  = 0;
    lcd.setCursor(0, 0);
    lcd.print(err_pos);
    //lcd.setCursor(0, 1);
    //lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(x0);
    lcd.setCursor(4, 1);
    lcd.print(y0);
    lcd.setCursor(8, 1);
    lcd.print(targX);
    lcd.setCursor(12, 1);
    lcd.print(targY);
  }
  index ++;
}