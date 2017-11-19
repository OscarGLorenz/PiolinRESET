#include <PS4USB.h>
#include <Servo.h>

//PARAMETROS SERVOS
const double limits[][2] = {
  {0,180},
  {0,180},
  {0,180},
  {0,180},
  {0,180},
  {0,180},
  {0,180},
  {0,180}
};
const int servoPins[] = {1,1,1,1,1,1,1,1};

//PARAMETROS GEOMÉTRICOS
const double l1 = 1;
const double l2 = 1;

//CONTROL
const double incrPad = 0.1;
const double incrJoy = 0.1;





USB Usb;
PS4USB PS4(&Usb);
double x,y,z;
double angle[8];
Servo servos[8];


void setup() {
  Serial.begin(115200);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS4 USB Library Started"));


  for (int i = 0; i < 8; i++) 
    servos[i].attach(servoPins[i]);

  angle[0] = 90;
  angle[1] = 90;
  angle[2] = 0;
  angle[3] = 90;
  x = 0;
  y = l2;
  z = l1;
  for (int i = 0; i < 4; i++)
      servos[i].write(angle[i]);
    
}

void loop() {
  Usb.Task();
  
  //LECTURA DEL MANDO
  int incrX = 0, incrY = 0, incrZ = 0;
  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117) {
      incrX = map(PS4.getAnalogHat(LeftHatX),0, 255, -incrJoy, +incrJoy);
    }
    if (PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {
      incrY = map(PS4.getAnalogHat(LeftHatY),0, 255, -incrJoy, +incrJoy);
    }
    if (PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117) {
      incrZ = map(PS4.getAnalogHat(RightHatY),0, 255, -incrJoy, +incrJoy);
    }
    
    if (PS4.getAnalogButton(R2)) //ABRIR CERRAR GARRA
      angle[7] = map(PS4.getAnalogButton(R2), 0, 255, limits[7][1], limits[7][2]); 

    if (PS4.getButtonClick(UP))
      angle[5] = constrain(angle[5]+=incrPad, limits[5][1], limits[5][2]);
    if (PS4.getButtonClick(RIGHT))
      angle[4] = constrain(angle[5]+=incrPad, limits[4][1], limits[4][2]);
    if (PS4.getButtonClick(DOWN))
      angle[5] = constrain(angle[5]-=incrPad, limits[5][1], limits[5][2]);
    if (PS4.getButtonClick(LEFT))
      angle[4] = constrain(angle[5]-=incrPad, limits[4][1], limits[4][2]);

    if (PS4.getButtonClick(L1))
     angle[6] = constrain(angle[6]-=incrPad, limits[6][1], limits[6][2]);
    if (PS4.getButtonClick(R1))
     angle[6] = constrain(angle[6]+=incrPad, limits[6][1], limits[6][2]);
  }


  //CINEMATICA INVERSA
  /*SI    CORDENADA RADIAL < (l1+l2)*cos(phi)=(l1+l2)*rho/(sqrt(rho^2+z^2))
   *SI    Z < (l1+l2)*sin(phi)=(l1+l2)=(l1+l2)*z/(sqrt(rho^2+z^2))
      - CALCULAR POSICIONES
        SI    NINGUNO ESTA FUERA DE RANGO
          - MOVER SERVOS
  */
  double iX = incrX + x;
  double iY = incrX + x;
  double iZ = incrX + x;
  double iRho = sqrt(iX*iX+iY*iY);

  if (sqrt(iX*iX+iY*iY) < (l1+l2)*sqrt(iRho)/sqrt(iRho*iRho+iZ*iZ) && iZ < (l1+l2)*iZ/sqrt(iRho*iRho+iZ*iZ)) {
        double l0 = sqrt(iRho*iRho+iZ*iZ);
        angle[1] = atan2(iY, iX);
        double cosG = (l1*l1+l2*l2-l0*l0)/(2*l1*l2);
        angle[2] = asin(l2/l0*sqrt(1-cosG*cosG))+atan2(iZ,iRho);
        angle[3] = angle[2] - asin((iZ-l1)*sin(angle[2])/l2);

        angle[1] = angle[1]*180.0/M_PI;
        angle[2] = angle[2]*180.0/M_PI;
        angle[3] = (90+angle[3])*180.0/M_PI;
        angle[0] = 180 - angle[1];

        if (angle[0] > limits[0][2] && angle[0] < limits[0][2] &&
            angle[1] > limits[1][2] && angle[1] < limits[1][2] &&
            angle[2] > limits[2][2] && angle[2] < limits[2][2] &&
            angle[3] > limits[3][2] && angle[3] < limits[3][2]) {

            for (int i = 0; i < 4; i++)
                servos[i].write(angle[i]);

            x = iX;
            y = iY;
            z = iZ;
        }
  }

  //MOVIMIENTO DEL RESTO DE MOTORES
  for (int i = 4; i < 8; i++)
    servos[i].write(angle[i]);

  delay(10);

}
