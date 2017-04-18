#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define BLUE 0x4
#define WHITE 0x7
#define SENSITIVITY 11
#define SIDESENSE 4
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define SENSOR_OUT 8
#define LEFT_ENCODER_PIN 10
#define RIGHT_ENCODER_PIN 11
#define KP 2
#define SHORT_FRONT_SENSOR A0
#define SHORT_LEFT_SENSOR A1
#define SHORT_RIGHT_SENSOR A2

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

Servo RServo, LServo;

bool SETUP_DONE = false;
int SETUP_OPTION = 0;

int right_encoder_count = 0,
    left_encoder_count = 0,
    left_encoder_last = 0,
    right_encoder_last = 0,
    enc_count_wanted = 0;

// DIRECTIONS, START AT NONE
enum DIRECTION_NAMES { NORTH, EAST, SOUTH, WEST, NONE };
enum MAZE_VALUES { BLANK, CURRENT, UNVISITED, VISITED, WALL };

int current_direction = NONE;
int maze[9][9] = {
  { WALL,      WALL,      WALL,      WALL,      WALL,      WALL,      WALL,      WALL, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, UNVISITED, WALL },
  { WALL,      WALL,      WALL,      WALL,      WALL,      WALL,      WALL,      WALL, WALL },
};
int start_location = 0, end_location = 0, start_direction = 0;


// WELP, I THINK THIS IS GONNA HOLD POSITIONS
bool board[4][4] = { 0 };
int row_position = 0, col_position = 0;

// SENSOR VALUES
int avg_sensor_front, avg_sensor_right, avg_sensor_left;

String color = "none";
bool looking = false;

void printDirection(int inputDirection);
void setupStartLocation();
void setupEndLocation();
void setupStartDirection();
void setupProgram();
void readAvgSensorValues();

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  pinMode(LEFT_ENCODER_PIN,INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN,INPUT_PULLUP);

  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  // SETUP SERVOS
  LServo.attach(2);
  RServo.attach(3);
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);
  //SET RGB SHIELD
  Serial.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(WHITE);
}

void loop() {
  Serial.println("SETTING UP");
  while (!SETUP_DONE) {
    Serial.println("SETTING UP");
    setupProgram();
  }
}

void printDirection(int input_direction) {
  switch(input_direction) {
    case 0:
      lcd.print("EAST");
      break;
    case 1:
      lcd.print("NORTH");
      break;
    case 2:
      lcd.print("WEST");
      break;
    case 3:
      lcd.print("SOUTH");
      break;
    default:
      lcd.print("ERROR");
  }
}

void setupStartLocation() {
  uint8_t buttons;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Start Location?");
  lcd.setCursor(0, 1);
  lcd.print(start_location);
  while (!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_UP) {
      start_location = (start_location + 16 + 1) % 16;
    }
    if (buttons & BUTTON_DOWN) {
      start_location = (start_location + 16 - 1) % 16;
    }
    if (buttons & BUTTON_LEFT) {
      SETUP_OPTION = (SETUP_OPTION + 3 - 1) % 3;
    }
    if (buttons & BUTTON_RIGHT) {
      SETUP_OPTION = (SETUP_OPTION + 3 + 1) % 3;
    }
    if (buttons & BUTTON_SELECT) {
      SETUP_DONE = true;
      lcd.clear();
    }
  }

}

void setupEndLocation() {
  uint8_t buttons;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("End Location?");
  lcd.setCursor(0, 1);
  lcd.print(end_location);
  while (!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_UP) {
      end_location = (end_location + 16 + 1) % 16;
    }
    if (buttons & BUTTON_DOWN) {
      end_location = (end_location + 16 - 1) % 16;
    }
    if (buttons & BUTTON_LEFT) {
      SETUP_OPTION = (SETUP_OPTION + 3 - 1) % 3;
    }
    if (buttons & BUTTON_RIGHT) {
      SETUP_OPTION = (SETUP_OPTION + 3 + 1) % 3;
    }
    if (buttons & BUTTON_SELECT) {
      SETUP_DONE = true;
      lcd.clear();
    }
  }
}

void setupStartDirection() {
  uint8_t buttons;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Start Direction?");
  lcd.setCursor(0, 1);
  printDirection(start_direction);
  while (!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_UP) {
      start_direction = (start_direction + 4 + 1) % 4;
    }
    if (buttons & BUTTON_DOWN) {
      start_direction = (start_direction + 4 - 1) % 4;
    }
    if (buttons & BUTTON_LEFT) {
      SETUP_OPTION = (SETUP_OPTION + 3 - 1) % 3;
    }
    if (buttons & BUTTON_RIGHT) {
      SETUP_OPTION = (SETUP_OPTION + 3 + 1) % 3;
    }
    if (buttons & BUTTON_SELECT) {
      SETUP_DONE = true;
      lcd.clear();
    }
  }

}

void setupProgram() {
  Serial.println(SETUP_OPTION);
  switch(SETUP_OPTION) {
    case 0:
      setupStartLocation();
      break;
    case 1:
      setupEndLocation();
      break;
    case 2:
      setupStartDirection();
      break;
    default:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("\\/(^.^)\\/");
  }
  delay(200);
}

void readAvgSensorValues(int d = 100) {
  float front[15], right[15], left[15];
  for(int x = 0; x < 15; x++) {
    front[x] = analogRead(SHORT_FRONT_SENSOR);
    right[x] = analogRead(SHORT_RIGHT_SENSOR);
    left[x] = analogRead(SHORT_LEFT_SENSOR);
    delay(d);
  }
  int tmp;
  for(int i = 0; i < 15; i++) {
    for(int j = 0; j < 14; j++) {
      if(front[j] < front[j+1]) {
        tmp = front[j];
        front[j] = front[j+1];
        front[j+1] = tmp;
      }
      if(left[j] < left[j+1]) {
        tmp       = left[j];
        left[j]    = left[j+1];
        left[j+1]  = tmp;
      }
      if(right[j] < right[j+1]) {
        tmp       = right[j];
        right[j]    = right[j+1];
        right[j+1]  = tmp;
      }
    }
  }
  int mdn_front = front[7], mdn_left = left[7], mdn_right = right[7];
  avg_sensor_front = 500*pow(mdn_front, -0.85);
  avg_sensor_right = 500*pow(mdn_right, -0.85);
  avg_sensor_left = 500*pow(mdn_left, -0.85);
}
