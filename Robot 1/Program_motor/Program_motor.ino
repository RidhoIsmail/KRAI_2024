#include <PS4Controller.h>
#include <math.h>
#include <Wire.h>

#define PWMA_Motor1_WB 5
#define PWMB_Motor1_WB 4
#define PWMA_Motor2_WB 27
#define PWMB_Motor2_WB 26
#define PWMA_Motor3_WB 5
#define PWMB_Motor3_WB 4

#define PWMA_Motor1_Pelempar 13
#define PWMB_Motor1_Pelempar 14
#define PWMA_Motor2_Pelempar 5
#define PWMB_Motor2_Pelempar 4

#define pendorong 23

bool pelemparAktif = false;
int initial_speed = 25;
int min_speed = 25;
int max_speed = 255;

float theta1 = 0;
float theta2 = 2 * PI / 3;
float theta3 = 4 * PI / 3;

unsigned long lastSpeedAdjust = 0;
unsigned long pneumaticStartTime = 0;
bool pneumaticActive = false;
const int pneumaticDuration = 150;

void kontrolMotor(int kecepatan, int PWMA, int PWMB) {
  kecepatan = constrain(kecepatan, -255, 255);
  if (kecepatan > 0) {
    analogWrite(PWMA, kecepatan);
    analogWrite(PWMB, 0);
  } else if (kecepatan < 0) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, abs(kecepatan));
  } else {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
}

void kontrolPelempar() {
  if (pelemparAktif) {
    analogWrite(PWMA_Motor1_Pelempar, initial_speed);
    analogWrite(PWMB_Motor1_Pelempar, 0);
    analogWrite(PWMA_Motor2_Pelempar, 0);
    analogWrite(PWMB_Motor2_Pelempar, initial_speed);
  } else {
    analogWrite(PWMA_Motor1_Pelempar, 0);
    analogWrite(PWMB_Motor1_Pelempar, 0);
    analogWrite(PWMA_Motor2_Pelempar, 0);
    analogWrite(PWMB_Motor2_Pelempar, 0);
  }
}

void handleSpeedAdjustment() {
  unsigned long now = millis();
  if (now - lastSpeedAdjust > 150) {
    if (PS4.Up()) {
      initial_speed = min(initial_speed + 5, max_speed);
      lastSpeedAdjust = now;
    }
    if (PS4.Down()) {
      initial_speed = max(initial_speed - 5, min_speed);
      lastSpeedAdjust = now;
    }
  }
}

void handlePneumatic() {
  unsigned long now = millis();
  static bool circlePrev = false;
  bool circleNow = PS4.Circle();

  if (pelemparAktif && circleNow && !circlePrev && !pneumaticActive) {
    pneumaticActive = true;
    pneumaticStartTime = now;
    digitalWrite(pendorong, HIGH);
    Serial.println("Pneumatik aktif");
  }

  if (pneumaticActive && (now - pneumaticStartTime >= pneumaticDuration)) {
    digitalWrite(pendorong, LOW);
    pneumaticActive = false;
    Serial.println("Pneumatik mati");
  }

  circlePrev = circleNow;
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA_Motor1_WB, OUTPUT);
  pinMode(PWMB_Motor1_WB, OUTPUT);
  pinMode(PWMA_Motor2_WB, OUTPUT);
  pinMode(PWMB_Motor2_WB, OUTPUT);
  pinMode(PWMA_Motor3_WB, OUTPUT);
  pinMode(PWMB_Motor3_WB, OUTPUT);

  pinMode(PWMA_Motor1_Pelempar, OUTPUT);
  pinMode(PWMB_Motor1_Pelempar, OUTPUT);
  pinMode(PWMA_Motor2_Pelempar, OUTPUT);
  pinMode(PWMB_Motor2_Pelempar, OUTPUT);

  pinMode(pendorong, OUTPUT);
  digitalWrite(pendorong, LOW);

  PS4.begin();
}

void loop() {
  if (PS4.isConnected()) {
    static bool crossPrev = false;
    bool crossNow = PS4.Cross();
    if (crossNow && !crossPrev) {
      pelemparAktif = !pelemparAktif;
      Serial.print("Pelempar: ");
      Serial.println(pelemparAktif ? "AKTIF" : "NONAKTIF");
      if (!pelemparAktif) {
        digitalWrite(pendorong, LOW); // Pastikan pneumatic OFF
      }
    }
    crossPrev = crossNow;

    handleSpeedAdjustment();
    handlePneumatic();
    kontrolPelempar();

    float raw_lx = PS4.LStickX() / 128.0;
    float raw_ly = PS4.LStickY() / 128.0;
    float lx = raw_ly;
    float ly = -raw_lx;
    float omega_input = PS4.RStickX() / 128.0;

    float v1 = -sin(theta1) * lx + cos(theta1) * ly + omega_input;
    float v2 = -sin(theta2) * lx + cos(theta2) * ly + omega_input;
    float v3 = -sin(theta3) * lx + cos(theta3) * ly + omega_input;

    float maxV = max(max(abs(v1), abs(v2)), abs(v3));
    if (maxV > 1.0) {
      v1 /= maxV;
      v2 /= maxV;
      v3 /= maxV;
    }

    int pwm1 = int(v1 * 255);
    int pwm2 = int(v2 * 255);
    int pwm3 = int(v3 * 255);

    kontrolMotor(pwm1, PWMA_Motor1_WB, PWMB_Motor1_WB);
    kontrolMotor(pwm2, PWMA_Motor2_WB, PWMB_Motor2_WB);
    kontrolMotor(pwm3, PWMA_Motor3_WB, PWMB_Motor3_WB);
  }
}
