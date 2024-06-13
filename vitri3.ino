#include <WiFi.h>
#include <WebServer.h>


const char* ssid = "destroy";
const char* password = "012345678";

// PID parameters
float kp = 0.02;
float ki = 0.00015;
float kd = 0.03;

// Time variables
unsigned long t;
unsigned long t_prev = 0;

// PWM configuration
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

int interruptPinA = 2;
int interruptPinB = 4;
volatile long EncoderCount = 0;
int PWMPin = 26;
int DirPin1 = 25;
int DirPin2 = 27;
unsigned long count = 0;
unsigned long count_prev = 0;
float Vmax = 6;
float Vmin = 0;
unsigned long dt;
float V;
float e, e_prev = 0, inte, inte_prev = 0;
float target = 0;
int PWMval1 = 0;
bool motorReachedDestination = false;
bool targetSet = false;

// Encoder pulses per revolution
// const int PPR = 125;

// Timer configuration
hw_timer_t *timer = NULL;
WebServer server(80);

void IRAM_ATTR dem_xung_motor() {
  if (digitalRead(interruptPinA) == HIGH) {
    EncoderCount++;
  } else {
    EncoderCount--;
  }
}

void IRAM_ATTR onTimer() {
  count++;
}

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWMPin, pwmChannel);
  
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(interruptPinA), dem_xung_motor, RISING);
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 150000, true);
  timerAlarmEnable(timer);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(1000);
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // HTTP server handlers
  server.on("/set_distance", HTTP_GET, []() {
    if (server.hasArg("distance")) {
      String distance = server.arg("distance");
      target = (distance.toFloat() * 100.0 / 62.8) * 130;
      targetSet = true;
      motorReachedDestination = false;
      EncoderCount = 0; // Reset EncoderCount
      t_prev = micros();
      inte_prev = 0;
      server.send(200, "text/plain", "Motor started");
    } else {
      server.send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/stop_motor", HTTP_GET, []() {
    stopMotor();
    motorReachedDestination = true;
    targetSet = false;
    server.send(200, "text/plain", "Motor stopped");
  });

  server.on("/get_data", HTTP_GET, []() {
    String data = "Pos_current:" + String(EncoderCount) + ":Pos_target:" + String(target);
    server.send(200, "text/plain", data);
  });

  server.begin();
}

void loop() {
  server.handleClient();

  if (targetSet && !motorReachedDestination) {
    if (count > count_prev) {
      t = micros();
      dt = t - t_prev;
      PID();
      t_prev = t;
      count_prev = count;

      // Debugging information
      Serial.print("EncoderCount: ");
      Serial.println(EncoderCount);
      Serial.print("target: ");
      Serial.println(target);
      Serial.print("Error: ");
      Serial.println(e);
      Serial.print("Integral: ");
      Serial.println(inte);
      Serial.print("Voltage: ");
      Serial.println(V);

      if (abs(e) < 3 ) { // Threshold to stop motor
        stopMotor();
        motorReachedDestination = true;
        targetSet = false;
        Serial.println("Motor has reached the target. You can enter the next distance.");
      }
    }
  }
}

void PID() {
  e = target - EncoderCount;
  inte = inte_prev + (dt * (e + e_prev) / 2);
  V = kp * e + ki * inte + kd * ((e - e_prev) / dt);

  if (V > Vmax) {
    V = Vmax;
    inte = inte_prev;
  }
  if (V < Vmin) {
    V = Vmin;
    inte = inte_prev;
  }

  PWMval1 = int(100 * abs(V) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 100;
  }

  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }

  ledcWrite(pwmChannel, PWMval1);

  e_prev = e;
  inte_prev = inte;
}

void stopMotor() {
  digitalWrite(DirPin1, LOW);
  digitalWrite(DirPin2, LOW);
  ledcWrite(pwmChannel, 0);
}
