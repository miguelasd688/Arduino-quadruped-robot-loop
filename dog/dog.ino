#include <Servo.h>
#include <math.h>


#define L1 40
#define L2 100
#define L3 100
#define MAX_GAMMA 50
///VARIABLES SERVOMOTORES
#define MAX_SERVOS 12
#define MAX_PULSE 2500
#define MIN_PULSE 560
Servo Servos[MAX_SERVOS];


//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long loopTime;
unsigned long previousLooptime;
double t;

struct angles {
  double tetta;
  double alpha;
  double gamma;
};////vector
struct coordinates {
  double x4;
  double y4;
  double z4;
};
struct coordinates coord0;
struct coordinates step_coord;
struct coordinates coordFR;
struct coordinates coordFL;
struct coordinates coordBR;
struct coordinates coordBL;
int pulse0, pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulse9, pulse10, pulse11;

char a;
int pulse;
float angle = 0;
float lateralGain;
double incremento;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  connectServos();
  coord0.x4 = -170;
  coord0.y4 = 30;
  coord0.z4 = -60;

  coordFR = coord0;
  coordFL = coord0;
  coordBR = coord0;
  coordBL = coord0;
}

void loop() {
  // put your main code here, to run repeatedly

  lateralGain = 20;//lateral inclination
  incremento = 0.0001;//speed up the loop.
  for (double i = 0 ; i <= 0.99 ; i = i + incremento) {//BR leg up
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial.print("Time: ");
      Serial.print(t);
      Serial.print("s <");
      Serial.print(loopTime);
      Serial.print("#");
      Serial.print("\t");
      previousLooptime = currentMillis;
      Serial.print(i);Serial.println("\t");
      
      step_coord = _step(i , 'u');//BR
      coordBR.x4 =  coord0.x4 + step_coord.x4;
      coordBR.y4 =  coord0.y4 + lateralGain*i;
      coordBR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 2 , 'd'); //FR
      coordFR.x4 =  coord0.x4 + step_coord.x4;
      coordFR.y4 =  coord0.y4 + lateralGain*i;
      coordFR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 1, 'd');//BL
      coordBL.x4 =  coord0.x4 + step_coord.x4;
      coordBL.y4 =  coord0.y4 - lateralGain*i;
      coordBL.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'd');//FL
      coordFL.x4 =  coord0.x4 + step_coord.x4;
      coordFL.y4 =  coord0.y4 - lateralGain*i;
      coordFL.z4 =  coord0.z4 + step_coord.z4;


      moveServos(coordFR, coordFL, coordBR, coordBL);
    }
  }
  for (double i = 0 ; i <= 0.99 ; i = i + incremento) {//FR leg up
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial.print("Time: ");
      Serial.print(t);
      Serial.print("s <");
      Serial.print(loopTime);
      Serial.print("#");
      Serial.print("\t");
      previousLooptime = currentMillis;
      Serial.print(i);Serial.println("\t");
      
      step_coord = _step(i , 'd');//BR
      coordBR.x4 =  coord0.x4 + step_coord.x4;
      coordBR.y4 =  coord0.y4 + lateralGain*(1-i);
      coordBR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'u');//FR
      coordFR.x4 =  coord0.x4 + step_coord.x4;
      coordFR.y4 =  coord0.y4 + lateralGain*(1-i);
      coordFR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 2 , 'd'); //BL
      coordBL.x4 =  coord0.x4 + step_coord.x4;
      coordBL.y4 =  coord0.y4 - lateralGain*(1-i);
      coordBL.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 1 , 'd');//FL
      coordFL.x4 =  coord0.x4 + step_coord.x4;
      coordFL.y4 =  coord0.y4 - lateralGain*(1-i);
      coordFL.z4 =  coord0.z4 + step_coord.z4;

      moveServos(coordFR, coordFL, coordBR, coordBL);
    }
  }
  for (double i = 0 ; i <= 0.99 ; i = i + incremento) {//BL leg up
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial.print("Time: ");
      Serial.print(t);
      Serial.print("s <");
      Serial.print(loopTime);
      Serial.print("#");
      Serial.print("\t");
      previousLooptime = currentMillis;
      Serial.print(i);Serial.println("\t");
      
      step_coord = _step(i + 1 , 'd');//BR
      coordBR.x4 =  coord0.x4 + step_coord.x4;
      coordBR.y4 =  coord0.y4 - lateralGain*i;
      coordBR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'd');//FR
      coordFR.x4 =  coord0.x4 + step_coord.x4;
      coordFR.y4 =  coord0.y4 - lateralGain*i;
      coordFR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'u');//BL
      coordBL.x4 =  coord0.x4 + step_coord.x4;
      coordBL.y4 =  coord0.y4 + lateralGain*i;
      coordBL.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 2 , 'd'); //FL
      coordFL.x4 =  coord0.x4 + step_coord.x4;
      coordFL.y4 =  coord0.y4 + lateralGain*i;
      coordFL.z4 =  coord0.z4 + step_coord.z4;

      moveServos(coordFR, coordFL, coordBR, coordBL);
    }
  }
  for (double i = 0 ; i <= 0.99 ; i = i + incremento) {//FL leg up
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial.print("Time: ");
      Serial.print(t);
      Serial.print("s <");
      Serial.print(loopTime);
      Serial.print("#");
      Serial.print("\t");
      previousLooptime = currentMillis;
      Serial.print(i);Serial.println("\t");
      
      step_coord = _step(i + 2 , 'd'); //BR
      coordBR.x4 =  coord0.x4 + step_coord.x4;
      coordBR.y4 =  coord0.y4 - lateralGain*(1-i);
      coordBR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i + 1, 'd');//FR
      coordFR.x4 =  coord0.x4 + step_coord.x4;
      coordFR.y4 =  coord0.y4 - lateralGain*(1-i);
      coordFR.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'd');//BL
      coordBL.x4 =  coord0.x4 + step_coord.x4;
      coordBL.y4 =  coord0.y4 + lateralGain*(1-i);
      coordBL.z4 =  coord0.z4 + step_coord.z4;

      step_coord = _step(i , 'u');//FL
      coordFL.x4 =  coord0.x4 + step_coord.x4;
      coordFL.y4 =  coord0.y4 + lateralGain*(1-i);
      coordFL.z4 =  coord0.z4 + step_coord.z4;

      moveServos(coordFR, coordFL, coordBR, coordBL);
    }
  }
}







void connectServos() {
  Servos[0].attach(7);//FR
  Servos[1].attach(6);//0=2500
  Servos[2].attach(5);

  Servos[3].attach(8);//FL
  Servos[4].attach(9);//0=550
  Servos[5].attach(10);

  Servos[6].attach(2);//BR
  Servos[7].attach(3);//0=2500
  Servos[8].attach(4);

  Servos[9].attach(13);//BL
  Servos[10].attach(12);//0=550
  Servos[11].attach(11);
}



struct coordinates _step( double t , char sig) {
  double x, y;
  float x0, x1, x2, x3, y0, y1, y2, y3;
  float L;
  struct coordinates coord;

  L = 80;
  if (sig == 'u') {
    //BEZIER CURVE WITH 4 POINTS
    x0 = 0;
    y0 = 0;

    x1 = -40;
    y1 = 20;

    x2 = 120;
    y2 = 20;

    x3 = 80;
    y3 = 0;
    
    coord.z4 = (1 - t) * ((1 - t) * ((1 - t) * x0 + t * x1) + t * ((1 - t) * x1 + t * x2)) +
               t * ((1 - t) * ((1 - t) * x1 + t * x2) + t * ((1 - t) * x2 + t * x3));
    coord.y4 = 0;
    coord.x4 = (1 - t) * ((1 - t) * ((1 - t) * y0 + t * y1) + t * ((1 - t) * y1 + t * y2)) +
               t * ((1 - t) * ((1 - t) * y1 + t * y2) + t * ((1 - t) * y2 + t * y3));
    return coord;

  }
  else if (sig == 'd') {

    coord.z4 = L - L * t / 3;
    coord.y4 = 0;
    coord.x4 = 0;
    return coord;
  }

}




int angleToPulse(double angle, int i) {
  double pulse;
  if (i == 0) {///////FR
    pulse = int(-10.822 * angle) + 925; //pos inicial
  }
  else if (i == 1) {
    pulse = int(-10.822 * angle) + 2350;
  }
  else if (i == 2) {
    pulse = int(-10.822 * angle) + 1000;
  }

  else if (i == 3) {//FL
    pulse = int(10.822 * angle) + 985;
  }
  else if (i == 4) {
    pulse = int(10.822 * angle) + 600;
  }
  else if (i == 5) {
    pulse = int(10.822 * angle) + 1150;
  }

  else if (i == 6) {//BR
    pulse = int(10.822 * angle) + 1025;
  }
  else if (i == 7) {
    pulse = int(-10.822 * angle) + 2325;
  }
  else if (i == 8) {
    pulse = int(-10.822 * angle) + 1100;
  }

  else if (i == 9) {//BL
    pulse = int(-10.822 * angle) + 900;
  }
  else if (i == 10) {
    pulse = int(10.822 * angle) + 710;
  }
  else if (i == 11) {
    pulse = int(10.822 * angle) + 1050;
  }
  return pulse;
}

//it was just for debuging, moving this can destabilize the robot
void keyboardInput() {
  if (Serial.available() > 0) {
    a = Serial.read();
    if (a == 'p') {
      coordFR.x4 = coordFR.x4 + 3;
      coordFL.x4 = coordFL.x4 + 3;
      coordBR.x4 = coordBR.x4 + 3;
      coordBL.x4 = coordBL.x4 + 3;
    }
    else if (a == 'o') {
      coordFR.x4 = coordFR.x4 - 3;
      coordFL.x4 = coordFL.x4 - 3;
      coordBR.x4 = coordBR.x4 - 3;
      coordBL.x4 = coordBL.x4 - 3;
    }

    if (a == 'a') {
      coordFR.z4 = coordFR.z4 + 3;
      coordFL.z4 = coordFL.z4 + 3;
      coordBR.z4 = coordBR.z4 + 3;
      coordBL.z4 = coordBL.z4 + 3;
    }
    else if (a == 's') {
      coordFR.z4 = coordFR.z4 - 3;
      coordFL.z4 = coordFL.z4 - 3;
      coordBR.z4 = coordBR.z4 - 3;
      coordBL.z4 = coordBL.z4 - 3;
    }
  }
}


int moveServos(struct coordinates coordFR , struct coordinates coordFL, struct coordinates coordBR, struct coordinates coordBL) { //  struct angles anglesFR;
  struct angles anglesFR;
  struct angles anglesFL;
  struct angles anglesBR;
  struct angles anglesBL;


  anglesFR = legFR(coordFR.x4 , coordFR.y4 , coordFR.z4);
  anglesFL = legFL(coordFL.x4 , coordFL.y4 , coordFL.z4);
  anglesBR = legBR(coordBR.x4 , coordBR.y4 , coordBR.z4);
  anglesBL = legBL(coordBL.x4 , coordBL.y4 , coordBL.z4);


  pulse0 = angleToPulse(anglesFR.tetta, 0);
  pulse1 = angleToPulse(anglesFR.alpha, 1);
  pulse2 = angleToPulse(anglesFR.gamma, 2);

  pulse3 = angleToPulse(anglesFL.tetta, 3);
  pulse4 = angleToPulse(anglesFL.alpha, 4);
  pulse5 = angleToPulse(anglesFL.gamma, 5);

  pulse6 = angleToPulse(anglesBR.tetta, 6);
  pulse7 = angleToPulse(anglesBR.alpha, 7);
  pulse8 = angleToPulse(anglesBR.gamma, 8);

  pulse9 = angleToPulse(anglesBL.tetta, 9);
  pulse10 = angleToPulse(anglesBL.alpha, 10);
  pulse11 = angleToPulse(anglesBL.gamma, 11);


  Servos[0].writeMicroseconds(pulse0);//initial pos
  Servos[1].writeMicroseconds(pulse1);
  Servos[2].writeMicroseconds(pulse2);

  Servos[3].writeMicroseconds(pulse3);
  Servos[4].writeMicroseconds(pulse4);
  Servos[5].writeMicroseconds(pulse5);

  Servos[6].writeMicroseconds(pulse6);
  Servos[7].writeMicroseconds(pulse7);
  Servos[8].writeMicroseconds(pulse8);

  Servos[9].writeMicroseconds(pulse9);
  Servos[10].writeMicroseconds(pulse10);
  Servos[11].writeMicroseconds(pulse11);
}


//INVERSE KINEMATICS
struct angles legFR(double x4, double y4, double z4) {

  struct angles ang;
  double D;

  D = (square(x4) + square(-y4) - square(L1) + square(z4) - square(L2) - square(L3)) / (2 * L2 * L3);
  //  if (D >= 1){D=1;}
  //  else if (D <= 0){D=0;}
  /////////////////////////////////////////////DOMINIO
  ang.tetta = -atan2(y4, x4) - atan2(sqrt(square(x4) + square(-y4) - square(L1)), -L1);
  ang.gamma = atan2(sqrt(1 - square(D)), D);
  ang.alpha = atan2(z4, sqrt(square(x4) + square(-y4) - square(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
  ang.alpha = -ang.alpha * 360 / (2 * PI);
  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;

  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
  if (ang.gamma >= MAX_GAMMA) {
    ang.gamma = MAX_GAMMA;
  }
  return ang;
}

struct angles legFL(double x4, double y4, double z4) {

  struct angles ang;
  double D;

  D = (square(x4) + square(-y4) - square(L1) + square(z4) - square(L2) - square(L3)) / (2 * L2 * L3);
  //  if (D >= 1){D=1;}
  //  else if (D <= 0){D=0;}
  /////////////////////////////////////////////DOMINIO
  ang.tetta = -atan2(y4, x4) - atan2(sqrt(square(x4) + square(-y4) - square(L1)), -L1);
  ang.gamma = atan2(sqrt(1 - square(D)), D);
  ang.alpha = atan2(z4, sqrt(square(x4) + square(-y4) - square(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
  ang.alpha = -ang.alpha * 360 / (2 * PI);
  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;

  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
  if (ang.gamma >= MAX_GAMMA) {
    ang.gamma = MAX_GAMMA;
  }
  return ang;
}

struct angles legBR(double x4, double y4, double z4) {

  struct angles ang;
  double D;

  D = (square(x4) + square(-y4) - square(L1) + square(z4) - square(L2) - square(L3)) / (2 * L2 * L3);
  //  if (D >= 1){D=1;}
  //  else if (D <= 0){D=0;}
  /////////////////////////////////////////////DOMINIO
  ang.tetta = -atan2(y4, x4) - atan2(sqrt(square(x4) + square(-y4) - square(L1)), -L1);
  ang.gamma = atan2(sqrt(1 - square(D)), D);
  ang.alpha = atan2(z4, sqrt(square(x4) + square(-y4) - square(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
  ang.alpha = -ang.alpha * 360 / (2 * PI);
  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;

  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
  if (ang.gamma >= MAX_GAMMA) {
    ang.gamma = MAX_GAMMA;
  }

  return ang;
}

struct angles legBL(double x4, double y4, double z4) {

  struct angles ang;
  double D;

  D = (square(x4) + square(-y4) - square(L1) + square(z4) - square(L2) - square(L3)) / (2 * L2 * L3);
  //  if (D >= 1){D=1;}
  //  else if (D <= 0){D=0;}
  /////////////////////////////////////////////DOMINIO
  ang.tetta = -atan2(y4, x4) - atan2(sqrt(square(x4) + square(-y4) - square(L1)), -L1);
  ang.gamma = atan2(sqrt(1 - square(D)), D);
  ang.alpha = atan2(z4, sqrt(square(x4) + square(-y4) - square(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
  ang.alpha = -ang.alpha * 360 / (2 * PI);
  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;

  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
  if (ang.gamma >= MAX_GAMMA) {
    ang.gamma = MAX_GAMMA;
  }
  return ang;
}
