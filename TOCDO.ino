#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "destroy";
const char* password = "012345678";

float kp = 0.02;      // Hệ số tỉ lệ P
float ki = 0.00015;   // Hệ số tích phân I
float kd = 0.003;      // Hệ số đạo hàm D

float Vmax = 6;       // Giá trị tối đa của điều khiển
float Vmin = 0;       // Giá trị tối thiểu của điều khiển

int motor1Pin1 = 25;   // Chân kết nối động cơ
int motor1Pin2 = 27;   // Chân kết nối động cơ
int enable1Pin = 26;   // Chân PWM kết nối động cơ

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

int encoA_motor = 2; // Chân encoder
int encoB_motor = 4; // Chân encoder

volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta_now1;
float Theta_prev1 = 0;
float RPM_output1, RPM_input1;
float dt1;
unsigned long t_now1;
unsigned long t_prev1 = 0;
volatile long dem_motor = 0;
int PWMval1 = 0;
float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;
float globalV = 1;

hw_timer_t *timer = NULL;
WebServer server(80); 

void IRAM_ATTR dem_xung_motor() {
  if (digitalRead(encoB_motor) == HIGH) {
    dem_motor++;
  } else {
    dem_motor--;
  }
}

void IRAM_ATTR onTimer() {
  count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  pinMode(encoA_motor, INPUT);
  pinMode(encoB_motor, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoA_motor), dem_xung_motor, RISING);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 150000, true);
  timerAlarmEnable(timer);

  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  server.on("/set_direction", HTTP_GET, handleSetDirection);
  server.on("/get_data", HTTP_GET, handleGetData);

  server.begin();
}

void loop() {
  if (count > count_prev) {
    t_now1 = millis();
    dt1 = (t_now1 - t_prev1);
    PID1(); 
    t_prev1 = t_now1;
    count_prev = count;
    Serial.println(RPM_output1);
  }

  server.handleClient();
}

void PID1() {
  Theta_now1 = dem_motor / 134;
  RPM_input1 = (globalV / (0.184 * 3.14)) * 60;
  RPM_output1 = (Theta_now1 - Theta_prev1) / (dt1 / 1000.0) * 60;

  float V1;
  error_now1 = RPM_input1 - RPM_output1;
  integ_now1 = integ_prev1 + (dt1 * (error_now1 + error_prev1) / 2);
  integ_prev1 = integ_now1;

  V1 = kp * error_now1 + ki * integ_now1 + kd * ((error_now1 - error_prev1) / dt1);

  if (V1 > Vmax) {
    V1 = Vmax;
    error_prev1 = error_now1;
  } else if (V1 < Vmin) {
    V1 = Vmin;
    error_prev1 = error_now1;
  }

  PWMval1 = int(255 * abs(V1) / Vmax);
  if (PWMval1 > 255) {
    PWMval1 = 255;
  }

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  ledcWrite(pwmChannel, PWMval1);

  Theta_prev1 = Theta_now1;
  
}

void handleSetDirection() {
  if (server.hasArg("direction")) {
    String direction = server.arg("direction");
    if (direction == "forward") {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
    } else if (direction == "reverse") {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
    } else if (direction == "stop") {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
    }
    server.send(200, "text/plain", "Direction set");
  } else {
    server.send(400, "text/plain", "Invalid direction");
  }
}

void handleGetData() {
  String data = "RPM_output1:" + String(RPM_output1) + ":RPM_input1:" + String(RPM_input1) + ":error:" + String(error_now1);
  server.send(200, "text/plain", data);
}