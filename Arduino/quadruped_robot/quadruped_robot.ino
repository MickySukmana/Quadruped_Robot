#include <Servo.h>

Servo serva1;
Servo serva2;
Servo serva3;

Servo servb1;
Servo servb2;
Servo servb3;

Servo servc1;
Servo servc2;
Servo servc3;

Servo servd1;
Servo servd2;
Servo servd3;

double x = -31.75,y = 0,z = 15;
double l1 = 31.75, l2 = 80, l3 = 80;

double x2da = 0, y2da = 15;
double x2db = 30, y2db = 15;
double x2dc = 0, y2dc = 15;
double x2dd = 30, y2dd = 15;

double theta1Deg = 0;
double theta2Deg = 0;
double theta3Deg = 0;

double theta1Deg2d = 0;
double theta2Deg2d = 0;

int td = 1;
int resolution = 50;
int state = 0;

char command;
/** 3D Inverse Kinematics **/
void IK_Calc(double x, double y, double z)
{  
  /****Inverse Kinematics****/
  double ag = sqrt(sq(x)+sq(y)-sq(l1));
  
  double ac = sqrt(sq(ag)+sq(z));

  double theta1 = PI-(atan2(y,x)+atan2(ag,l1));
  
  double a = acos(-(sq(ac)-sq(l2)-sq(l3))/(2*l2*l3));

  double theta3 = PI - a;

  double theta2 = atan2(z,ag)-atan2(l3*sin(theta3),l2+l3*cos(theta3));
  /***************************/
  
  /******radian 2 degree******/
  theta1Deg=theta1*180/PI;
  theta2Deg=theta2*180/PI;
  theta3Deg=theta3*180/PI;
  /***************************/
}

/** 2d inverse Kinematics **/
void IK_Calc2D(double x, double y)
{
  double r = sq(x)+sq(y);
  
  double c2 = (r-sq(l2)-sq(l3))/(2*l2*l3);
  
  double a = acos(-(r-sq(l2)-sq(l3))/(2*l2*l3));
  
  double s2 = sqrt(1-sq(c2));
  
  double theta2=atan2(s2,c2);

  double b = atan2(l3*s2,l2+(l3*c2));

  double c = atan2(y,x);

  double theta1 = c-b;

  theta1Deg2d=theta1*180/PI;
  theta2Deg2d=theta2*180/PI;
}

void setup() {
  Serial.begin(115200);
  serva1.attach(2); //180 invert angle
  serva2.attach(3); //0   normal angle
  serva3.attach(4); //180 invert angle
  
  servb1.attach(5); //0   normal angle
  servb2.attach(6); //0   normal angle
  servb3.attach(7); //180 invert angle

  servc1.attach(8); //0   normal  angle
  servc2.attach(9); //180 invert angle
  servc3.attach(10);  //0 normal angle
  
  servd1.attach(11);  //180 invert angle
  servd2.attach(12);  //180 invert angle
  servd3.attach(13);  //0   normal angle

  IK_Calc(x, y, z);
  IK_Calc2D(x2da, y2da);
  serva1.write(map(theta1Deg,0,180,180,0));
  serva2.write(theta1Deg2d);
  serva3.write(map(theta2Deg2d,0,180,180,0));
  IK_Calc2D(x2db, y2db);
  servb1.write(theta1Deg);
  servb2.write(theta1Deg2d);
  servb3.write(map(theta2Deg2d,0,180,180,0));
  IK_Calc2D(x2dc, y2dc);
  servc1.write(theta1Deg);
  servc2.write(map(theta1Deg2d,0,180,180,0));
  servc3.write(theta2Deg2d);
  IK_Calc2D(x2dd, y2dd);
  servd1.write(map(theta1Deg,0,180,180,0));
  servd2.write(map(theta1Deg2d,0,180,180,0));
  servd3.write(theta2Deg2d);//*/
  delay(10000);
  
  for(int i = 0;i<resolution;i++){
    y2da=y2da+1.7;
    y2db=y2db+2.1;
    y2dc=y2dc+1.7;
    y2dd=y2dd+1.9;
    if (y2da>100)y2da=100;
    if (y2db>120)y2db=120;
    if (y2dc>100)y2dc=100;
    if (y2dd>110)y2dd=110;
    
    IK_Calc2D(x2da, y2da);
    serva1.write(map(theta1Deg,0,180,180,0));
    serva2.write(theta1Deg2d);
    serva3.write(map(theta2Deg2d,0,180,180,0));
    IK_Calc2D(x2db, y2db);
    servb1.write(theta1Deg);
    servb2.write(theta1Deg2d);
    servb3.write(map(theta2Deg2d,0,180,180,0));
    IK_Calc2D(x2dc, y2dc);
    servc1.write(theta1Deg);
    servc2.write(map(theta1Deg2d,0,180,180,0));
    servc3.write(theta2Deg2d);
    IK_Calc2D(x2dd, y2dd);
    servd1.write(map(theta1Deg,0,180,180,0));
    servd2.write(map(theta1Deg2d,0,180,180,0));
    servd3.write(theta2Deg2d);
    delay(5);
  }
  delay(1000);
}

void loop() {
  if (state == 0)
  {
    for(int Step = 0;Step<20;Step++)
    {
      //step1
      for(int i = 0;i<=resolution;i++)
      {
        y2da=y2da-0.4;
        x2da=x2da-0.4;
        if(y2da<80)y2da=80;
        if(x2da<-20)x2da=-20;
        IK_Calc2D(x2da, y2da);
        serva2.write(theta1Deg2d);
        serva3.write(map(theta2Deg2d,0,180,180,0));
        delay(td);  
      }
      for(int i = 0;i<=resolution;i++)
      {
        y2da=y2da+0.4;
        if(y2da>100)y2da=100;
        IK_Calc2D(x2da, y2da);
        serva2.write(theta1Deg2d);
        serva3.write(map(theta2Deg2d,0,180,180,0));
        delay(td);  
      }
      delay(200);
      //step2
      for(int i = 0;i<=resolution;i++)
      {
        y2dd=y2dd-0.4;
        x2dd=x2dd-0.4;
        if(y2dd<80)y2dd=80;
        if(x2dd<10)x2dd=10;
        IK_Calc2D(x2dd, y2dd);
        servd2.write(map(theta1Deg2d,0,180,180,0));
        servd3.write(theta2Deg2d);
        delay(td);  
      }
      for(int i = 0;i<=resolution;i++)
      {
        y2dd=y2dd+0.6;
        if(y2dd>110)y2dd=110;
        IK_Calc2D(x2dd, y2dd);
        servd2.write(map(theta1Deg2d,0,180,180,0));
        servd3.write(theta2Deg2d);
        delay(td);  
      }
      delay(200);
      //step3
      for(int i = 0;i<=resolution;i++)
      {
        y2dc=y2dc-0.4;
        x2dc=x2dc-0.4;
        if(y2dc<80)y2dc=80;
        if(x2dc<-20)x2dc=-20;
        IK_Calc2D(x2dc, y2dc);
        servc2.write(map(theta1Deg2d,0,180,180,0));
        servc3.write(theta2Deg2d);
        delay(td);  
      }
      for(int i = 0;i<=resolution;i++)
      {
        y2dc=y2dc+0.4;
        if(y2dc>100)y2dc=100;
        IK_Calc2D(x2dc, y2dc);
        servc2.write(map(theta1Deg2d,0,180,180,0));
        servc3.write(theta2Deg2d);
        delay(td);  
      }
      delay(200);
      
      //step4
      for(int i = 0;i<=resolution;i++)
      {
        y2db=y2db-0.4;
        x2db=x2db-0.4;
        if(y2db<80)y2db=80;
        if(x2db<10)x2db=10;
        IK_Calc2D(x2db, y2db);
        servb2.write(theta1Deg2d);
        servb3.write(map(theta2Deg2d,0,180,180,0));
        delay(td);  
      }
      for(int i = 0;i<=resolution;i++)
      {
        y2db=y2db+0.8;
        if(y2db>120)y2db=120;
        IK_Calc2D(x2db, y2db);
        servb2.write(theta1Deg2d);
        servb3.write(map(theta2Deg2d,0,180,180,0));
        delay(td);  
      }
      delay(200);
      //lean forward
      for(int i = 0;i<=resolution;i++)
      {
        x2da=x2da+0.4;
        x2db=x2db+0.4;
        x2dc=x2dc+0.4;
        x2dd=x2dd+0.4;
        if(x2da>0)x2da=0;
        if(x2db>30)x2db=30;
        if(x2dc>0)x2dc=0;
        if(x2dd>30)x2dd=30;
        IK_Calc2D(x2da, y2da);
        serva2.write(theta1Deg2d);
        serva3.write(map(theta2Deg2d,0,180,180,0));
        IK_Calc2D(x2db, y2db);
        servb2.write(theta1Deg2d);
        servb3.write(map(theta2Deg2d,0,180,180,0));
        IK_Calc2D(x2dc, y2dc);
        servc2.write(map(theta1Deg2d,0,180,180,0));
        servc3.write(theta2Deg2d);
        IK_Calc2D(x2dd, y2dd);
        servd2.write(map(theta1Deg2d,0,180,180,0));
        servd3.write(theta2Deg2d);
        delay(td);  
      }
      delay(200);
    }
    //*/
    for(int i = 0;i<resolution;i++){
    y2da=y2da-1.7;
    y2db=y2db-2.1;
    y2dc=y2dc-1.7;
    y2dd=y2dd-1.9;
    if (y2da<15)y2da=15;
    if (y2db<15)y2db=15;
    if (y2dc<15)y2dc=15;
    if (y2dd<15)y2dd=15;
    
    IK_Calc2D(x2da, y2da);
    serva1.write(map(theta1Deg,0,180,180,0));
    serva2.write(theta1Deg2d);
    serva3.write(map(theta2Deg2d,0,180,180,0));
    IK_Calc2D(x2db, y2db);
    servb1.write(theta1Deg);
    servb2.write(theta1Deg2d);
    servb3.write(map(theta2Deg2d,0,180,180,0));
    IK_Calc2D(x2dc, y2dc);
    servc1.write(theta1Deg);
    servc2.write(map(theta1Deg2d,0,180,180,0));
    servc3.write(theta2Deg2d);
    IK_Calc2D(x2dd, y2dd);
    servd1.write(map(theta1Deg,0,180,180,0));
    servd2.write(map(theta1Deg2d,0,180,180,0));
    servd3.write(theta2Deg2d);
    delay(5);
    }
    state = 1;
  }
}
