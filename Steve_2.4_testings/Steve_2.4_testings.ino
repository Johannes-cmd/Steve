#define WIFI_NAME "Cristo salva"
#define WIFI_PASSWORD ""
#define DEVICE_ID 4
#define DEVICE_NAME "Robo Salvavidas"
#define TOKEN "~991902_jNysKCl7ke3LDlgi2NXNxND7"

#include <RemoteMe.h>
#include <RemoteMeSocketConnector.h>
#include <RemoteMeDirectWebSocketConnector.h> // !important library needs WebSockets by Markus Sattler Please install it from Library manager

#include <ESP8266WiFi.h>


#define ENA D3
#define IN1 D1
#define IN2 D2

#define IN3 D5
#define IN4 D6
#define ENB D7

//pra frente
int a = 0;

int motor = 0;

int aceleracao = 0;
int acelerado = 0;

int antacelerado = 0;

int parada = 0;

unsigned long lastExecutedMillis1 = 0;
unsigned long lastExecutedMillis2 = 0;
unsigned long lastExecutedMillis3 = 0;

RemoteMe & remoteMe = RemoteMe::getInstance(TOKEN, DEVICE_ID);

int vertical;
int horizontal;
int X;
int Y;

int motorA;
int motorB;

//***** CODE FOR COMFORTABLE VARIABLE SET *******

inline void setJoystick(int16_t i1, int16_t i2) {
  remoteMe.getVariables() -> setSmallInteger2("Joystick", i1, i2);
}

inline void setBackward(boolean b) {
  remoteMe.getVariables() -> setBoolean("Backward", b);
}
inline void setForward(boolean b) {
  remoteMe.getVariables() -> setBoolean("Forward", b);
}
inline void setLeft(boolean b) {
  remoteMe.getVariables() -> setBoolean("Left", b);
}
inline void setRight(boolean b) {
  remoteMe.getVariables() -> setBoolean("Right", b);
}
inline void setSpeed(int32_t i) {
  remoteMe.getVariables() -> setInteger("Speed", i);
}
inline void setWiFi_strength(int32_t i) {
  remoteMe.getVariables() -> setInteger("WiFi strength", i);
}

//***** IMPLEMENT FUNCTIONS BELOW *******

void onWiFi_strengthChange(int32_t i) {
}

void onJoystickChange(int16_t i1, int16_t i2) {
  //Serial.printf("onJoystickChange: i1: %d, i2: %d\n",i1, i2);

  vertical = i2;
  horizontal = i1;
  X = i1;
  Y = i2;


  //as duas rodas para frente:

  if (i2 >= 13) {   

    if (i1 >= 13) {    //se para direita
      if (i1 * 2 <= i2) {
        
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      analogWrite(ENB, i2);
      analogWrite(ENA, i2);

      Serial.print("frente");
      Serial.println(i2);
      }
      
      if (i2 <= i1 / 3) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        analogWrite(ENB, i1);
        analogWrite(ENA, i1);

        Serial.println("direita");
      }
      
      if ((i1 * 2 > i2) and (i2 >= i1 / 3)) {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        motorB = i2 + i1;             // change later to if one is bigger, chose that one...
        motorA = motorB * 3 / 4;
        if (motorB >= 1023) {
          motorB = 1023;
        }
        Serial.println("frente direita");
        analogWrite(ENB, motorA);
        analogWrite(ENA, motorB);
      }
    }

    if (i1 <= 13) {    //se para esquerda:

      if (-i1 * 2 <= i2) {
        
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      analogWrite(ENB, i2);
      analogWrite(ENA, i2);

      Serial.print("frente");
      Serial.println(i2);
      }
            
      if (i2 < -i1 / 3) {    //uma roda para frente e uma para traz:

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorB = -horizontal;

        analogWrite(ENB, motorB);
        analogWrite(ENA, motorB);
        
        Serial.println("esquerda");
      }
      
      if ((-i1 * 2 > i2) and (i2 >= -i1 / 3)) {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        motorA = vertical - horizontal;
        motorB = motorA * 3 / 4;
        if (motorA >= 1023) {
          motorA = 1023;
        }
        Serial.println("frente esquerda");
        Serial.println(motorA);
        Serial.println(motorB);
    
        analogWrite(ENB, motorA);
        analogWrite(ENA, motorB);
      }
    }
  }

  if (i2 <= 13) {           //as duas para traz:
      
    if (i1 <= 13) {     //se para esquerda
 
      if (-i1 * 2 <= -i2) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorA = -i2;

        analogWrite(ENB, motorA);
        analogWrite(ENA, motorA);
        
        Serial.println("traz");
      }
      
      if (-i2 < -i1 / 3) {    //uma roda para frente e uma para traz:

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorB = -horizontal;

        analogWrite(ENB, motorB);
        analogWrite(ENA, motorB);
        
        Serial.println("esquerda");
      }
      
      if ((-i1 * 2 > -i2) and (-i2 >= -i1 / 3)) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorB = -i2 - i1;
        motorA = motorB * 3 / 4;
        if (motorB >= 1023) {
          motorB = 1023;
        }
        analogWrite(ENA, motorA);
        analogWrite(ENB, motorB);
        Serial.println("traz esquerda");
      }
    }
    
     if (i1 >= 13) {    //se para direita

        if (i1 * 2 <= -i2) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorA = -i2;

        analogWrite(ENB, motorA);
        analogWrite(ENA, motorA);
        
        Serial.println("traz");
      }

      if (-i2 <= i1 / 3) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        analogWrite(ENB, i1);
        analogWrite(ENA, i1);

        Serial.println("direita");
      }
      
      if ((i1 * 2 > -i2) and (-i2 > i1 / 3)) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorB = -i2 + i1;
        motorA = motorB * 3 / 4;
        if (motorB >= 1023) {
          motorB = 1023;
        }
        analogWrite(ENA, motorB);
        analogWrite(ENB, motorA);
        
      Serial.println("traz direita");
      Serial.println(motorB);
      }
    }
  }
}


void onBackwardChange(boolean b) {
  Serial.printf("onBackwardChange: b: %d\n", b);
  digitalWrite(IN2, b ? HIGH : LOW);
  digitalWrite(IN3, b ? HIGH : LOW);

  a = b;

  acelerado = 0;

 /*
  if (!b) // test for falsity
  {
    parada = 0; // do whatever...
  }
  parada = 1;
*/
}
void onForwardChange(boolean b) {

  Serial.printf("onForwardChange: b: %d\n", b);
  digitalWrite(IN1, b ? HIGH : LOW);
  digitalWrite(IN4, b ? HIGH : LOW);

  a = b;

  acelerado = 0;

/*
  if (!b) // test for falsity
  {
    parada = 0; // do whatever...
  }
  parada = 1;
*/

}
void onLeftChange(boolean b) {
  Serial.printf("onLeftChange: b: %d\n", b);
  digitalWrite(IN1, b ? HIGH : LOW);
  digitalWrite(IN3, b ? HIGH : LOW);
  a = b;

  acelerado = 0;
/*
  if (!b) // test for falsity
  {
    parada = 0; // do whatever...
  }
  parada = 1;
*/
}
void onRightChange(boolean b) {
  Serial.printf("onRightChange: b: %d\n", b);
  digitalWrite(IN2, b ? HIGH : LOW);
  digitalWrite(IN4, b ? HIGH : LOW);
  a = b;

  acelerado = 0;
/*
  if (!b) // test for falsity
  {
    parada = 0; // do whatever...
  }
  else {
    parada = 1;
  }
  */
}
void onSpeedChange(int32_t i) {
  Serial.printf("onSpeedChange: i: %d\n", i);
  //analogWrite(ENA, i); 
  //analogWrite(ENB, i);
  aceleracao = i;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);


  }

  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(100);
  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(50);
  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(100);
  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(50);
  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(100);
  analogWrite(ENA, 500);
  analogWrite(ENB, 500);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(50);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  remoteMe.getVariables() -> observeInteger("WiFi strength", onWiFi_strengthChange);

  remoteMe.getVariables() -> observeSmallInteger2("Joystick", onJoystickChange);

  remoteMe.getVariables() -> observeBoolean("Backward", onBackwardChange);
  remoteMe.getVariables() -> observeBoolean("Forward", onForwardChange);
  remoteMe.getVariables() -> observeBoolean("Left", onLeftChange);
  remoteMe.getVariables() -> observeBoolean("Right", onRightChange);
  remoteMe.getVariables() -> observeInteger("Speed", onSpeedChange);

  remoteMe.setConnector(new RemoteMeSocketConnector());
  remoteMe.setDirectConnector(new RemoteMeDirectWebSocketConnector());
  remoteMe.sendRegisterDeviceMessage(DEVICE_NAME);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}

void loop() {
  remoteMe.loop();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  if (millis() > lastExecutedMillis1 + 1000) {
    lastExecutedMillis1 = millis();
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    setWiFi_strength(WiFi.RSSI());
  }

  /////////////////////////////////////////////////////////////////////////////////////////////

  // app de botoes:

  //menos que 500 (rapido)

  if (acelerado < 500) {

    if ((a == 1) & (acelerado < aceleracao)) {

      acelerado = acelerado + 12;

      Serial.print("menos que 500 'rapido' ");
      Serial.println(acelerado);

      analogWrite(ENA, acelerado);
      analogWrite(ENB, acelerado);
      delay(1);
    }

    if ((a == 1) & (acelerado > aceleracao)) {

      acelerado = aceleracao;

      Serial.print("menos que 500 'rapido' ");
      Serial.println(acelerado);

      analogWrite(ENA, acelerado);
      analogWrite(ENB, acelerado);
      delay(1);
    }
  }
  // mais que 500 (devagar)

  if (acelerado >= 500) {

    if ((a == 1) & (acelerado < aceleracao)) {

      acelerado = acelerado + 5;

      Serial.print("mais0 que 500 'devagar' ");
      Serial.println(acelerado);

      analogWrite(ENA, acelerado);
      analogWrite(ENB, acelerado);
      delay(1);
    }

    if ((a == 1) & (acelerado > aceleracao)) {

      acelerado = aceleracao;

      Serial.print("mais0 que 500 'devagar' ");
      Serial.println(acelerado);

      analogWrite(ENA, acelerado);
      analogWrite(ENB, acelerado);
      delay(1);
    }
  }

  if ((a == 0) and (-17 < Y < 17) and (-17 < X < 17) and (Y=X)) {
    acelerado = 10;
    delay(1);

    if (millis() > lastExecutedMillis2 + 500) {
      lastExecutedMillis2 = millis();
      Serial.print("Stop");
      Serial.println();
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////

}
