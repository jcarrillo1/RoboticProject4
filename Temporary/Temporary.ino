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

int maze_row_pos = 1, maze_col_pos = 1;

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

void printMaze(){
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      Serial.print(maze[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
void printGrid() {
  lcd.clear();
  lcd.setCursor(0, 0);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (board[i][j]) {
        lcd.print("X");
      } else {
        lcd.print("O");
      }
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("G");
  int gridPos = (row_position + 1) * (col_position + 1);
  lcd.print(gridPos);
  lcd.print(" ");
  bool frontWall = checkFrontWall();
  bool leftWall = checkLeftWall();
  bool rightWall = checkRightWall();
  switch(current_direction) {
    case NORTH: {
      lcd.print("W");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("N");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("E");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("SU");
      break;
    }
    case EAST: {
      lcd.print("WU ");
      lcd.print("N");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("E");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("S");
      if (rightWall) lcd.print("X");
      else lcd.print("O");
      break;
    }
    case SOUTH: {
      lcd.print("W");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("NU ");
      lcd.print("E");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("S");
      if (frontWall) lcd.print("X");
      else lcd.print("O");
      break;
    }
    case WEST: {
      lcd.print("W");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("N");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("EU ");
      lcd.print("S");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      break;
    }
    default: {
      lcd.print("\\/(o.o)\\/");
    }
  }
}

bool checkFrontWall() {
  int checked_row = row_position;
  int checked_col = col_position;
  switch(current_direction) {
    case NORTH: {
      checked_row = row_position - 1;
      if (checked_row < 0) return true;
      break;
    }
    case EAST: {
      checked_col = col_position + 1;
      if (checked_col > 3) return true;
      break;
    }
    case SOUTH: {
      checked_row = row_position + 1;
      if (checked_row > 3) return true;
      break;
    }
    case WEST: {
      checked_col = col_position - 1;
      if (checked_col < 0) return true;
      break;
    }
    default: {
      return true;
    }
  }
  bool isWall = avg_sensor_front < SENSITIVITY && avg_sensor_front > 0;
  if (isWall) {
    maze[checked_row][checked_col] = WALL;
  }
  return isWall;
}

bool checkRightWall() {
  int checked_row = row_position;
  int checked_col = col_position;
  switch(current_direction) {
    case WEST: {
      checked_row = row_position - 1;
      if (checked_row < 1) return true;
      break;
    }
    case NORTH: {
      checked_col = col_position + 1;
      if (checked_col > 7) return true;
      break;
    }
    case EAST: {
      checked_row = row_position + 1;
      if (checked_row > 7) return true;
      break;
    }
    case SOUTH: {
      checked_col = col_position - 1;
      if (checked_col < 1) return true;
      break;
    }
    default: {
      return true;
    }
  }
  bool isWall = avg_sensor_front < SENSITIVITY && avg_sensor_front > 0;
  if (isWall) {
    maze[checked_row][checked_col] = WALL;
  }
  return isWall;
}

bool checkLeftWall() {
  int checked_row = row_position;
  int checked_col = col_position;
  switch(current_direction) {
    case EAST: {
      checked_row = row_position - 1;
      if (checked_row < 1) return true;
      break;
    }
    case SOUTH: {
      checked_col = col_position + 1;
      if (checked_col > 7) return true;
      break;
    }
    case WEST: {
      checked_row = row_position + 1;
      if (checked_row > 7) return true;
      break;
    }
    case NORTH: {
      checked_col = col_position - 1;
      if (checked_col < 1) return true;
      break;
    }
    default: {
      return true;
    }
  }
  bool isWall = avg_sensor_front < SENSITIVITY && avg_sensor_front > 0;
  if (isWall) {
    maze[checked_row][checked_col] = WALL;
  }
  return isWall;
}

bool checkFrontVisited() {
  switch(current_direction) {
    case NORTH: {
      int row_new = row_position - 1;
      if (row_new < 0) return true;
      return board[row_new][col_position];
    }
    case EAST: {
      int col_new = col_position + 1;
      if (col_new > 3) return true;
      return board[row_position][col_new];
    }
    case SOUTH: {
      int row_new = row_position + 1;
      if (row_new > 3) return true;
      return board[row_new][col_position];
    }
    case WEST: {
      int col_new = col_position - 1;
      if (col_new < 0) return true;
      return board[row_position][col_new];
    }
    default: {
      return true;
    }
  }
}

bool checkRightVisited() {
  switch(current_direction) {
    case WEST: {
      int row_new = row_position - 1;
      if (row_new < 0) return true;
      return board[row_new][col_position];
    }
    case NORTH: {
      int col_new = col_position + 1;
      if (col_new > 3) return true;
      return board[row_position][col_new];
    }
    case EAST: {
      int row_new = row_position + 1;
      if (row_new > 3) return true;
      return board[row_new][col_position];
    }
    case SOUTH: {
      int col_new = col_position - 1;
      if (col_new < 0) return true;
      return board[row_position][col_new];
    }
    default: {
      return true;
    }
  }
}

bool checkLeftVisited() {
  switch(current_direction) {
    case EAST: {
      int row_new = row_position - 1;
      if (row_new < 0) return true;
      return board[row_new][col_position];
    }
    case SOUTH: {
      int col_new = col_position + 1;
      if (col_new > 3) return true;
      return board[row_position][col_new];
    }
    case WEST: {
      int row_new = row_position + 1;
      if (row_new > 3) return true;
      return board[row_new][col_position];
    }
    case NORTH: {
      int col_new = col_position - 1;
      if (col_new < 0) return true;
      return board[row_position][col_new];
    }
    default: {
      return true;
    }
  }
}


void getSensorValues() {
  float front = analogRead(SHORT_FRONT_SENSOR);
  float right = analogRead(SHORT_RIGHT_SENSOR);
  float left  = analogRead(SHORT_LEFT_SENSOR);
  avg_sensor_front = 500*pow(front, -0.85);
  avg_sensor_right = 500*pow(right, -0.85);
  avg_sensor_left = 500*pow(left, -0.85);
}


void updateDirection(int x) {
  current_direction = (current_direction + 4 + x) % 4;
}

void encoder() {
  int left_encoder_value = digitalRead(LEFT_ENCODER_PIN);
  int right_encoder_value = digitalRead(RIGHT_ENCODER_PIN);

  if (left_encoder_value != left_encoder_last) left_encoder_count++;
  if (right_encoder_value != right_encoder_last) right_encoder_count++;

  left_encoder_last = left_encoder_value;
  right_encoder_last = right_encoder_value;
}

void markBoard() {
  switch(current_direction) {
    case NORTH: {
      board[row_position - 1][col_position] = VISITED;
      row_position -= 2;
      lcd.setBacklight(BLUE);
      break;
    }
    case EAST: {
      board[row_position][col_position + 1] = VISITED;
      col_position += 2;
      lcd.setBacklight(RED);
      break;

    }
    case SOUTH: {
      board[row_position + 1][col_position] = VISITED;
      row_position += 2;
      lcd.setBacklight(YELLOW);
      break;
    }
    case WEST: {
      board[row_position][col_position - 1] = VISITED;
      col_position -= 2;
      lcd.setBacklight(GREEN);
      break;
    }
    default: {
      return;
    }
  }
  board[row_position][col_position] = VISITED;
  looking = false;
}

void readColors() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);

  int fRed = pulseIn(SENSOR_OUT, LOW);

  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);

  int fGreen = pulseIn(SENSOR_OUT, LOW);


  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);

  int fBlue = pulseIn(SENSOR_OUT, LOW);

  if (looking && (fRed < 200 || fBlue < 200) && color == "none") {
    color = "wooh";

    markBoard();
  } else if (!(fRed < 200 || fBlue < 200) && color != "none") {

    color = "none";
    lcd.setBacklight(WHITE);
  }
}

void setEncoder(int initial = 101, int w = 100) {
  right_encoder_count = initial;
  left_encoder_count = initial;
  enc_count_wanted = w;
}

void stop(int x = 0) {
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);
  delay(x);
}

void turnLeft() {
  updateDirection(-1);
  LServo.writeMicroseconds(1450);
  RServo.writeMicroseconds(1450);
  setEncoder(0, 26);
  while (!(enc_count_wanted < right_encoder_count && enc_count_wanted < left_encoder_count)) {
    encoder();
  }
  setEncoder();
  stop();
}

void turnRight() {
  updateDirection(1);
  LServo.writeMicroseconds(1560);
  RServo.writeMicroseconds(1560);
  setEncoder(0, 26);
  while (!(enc_count_wanted < right_encoder_count && enc_count_wanted < left_encoder_count)) {
    encoder();
  }
  setEncoder();
  stop();
}

int saturate(int x) {
  if (x > 5) return 5;
  if (x < -5) return -5;
  return x;
}

void correctMotion() {
  getSensorValues();

  if (avg_sensor_left < 6 || (avg_sensor_right >= 8 && avg_sensor_right < SENSITIVITY)) {
    int error = saturate(round((avg_sensor_right - avg_sensor_left) * KP));
    LServo.writeMicroseconds(1565);
    RServo.writeMicroseconds(1455);
  } else if (avg_sensor_right < 6 || (avg_sensor_left >= 8 && avg_sensor_left < SENSITIVITY)) {
    // add more to right wheel
    int error = saturate(round((avg_sensor_left - avg_sensor_right) * KP));
    LServo.writeMicroseconds(1550);
    RServo.writeMicroseconds(1440);
  } else {
     LServo.writeMicroseconds(1550);
     RServo.writeMicroseconds(1455);
  }
}

void moveForward() {
  setEncoder(0, 144);
  looking = true;
  LServo.writeMicroseconds(1550);
  RServo.writeMicroseconds(1455);
  while(enc_count_wanted > right_encoder_count && enc_count_wanted > left_encoder_count) {
    encoder();
//    correctMotion();
    readColors();
  }
  setEncoder();
  stop();
}

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
  while (!SETUP_DONE) {
    Serial.println("SETTING UP");
    setupProgram();
  }
  readAvgSensorValues();
  printGrid();
  delay(100);
  if (!checkFrontWall() && !checkFrontVisited()) {
    moveForward();
  } else if(!checkRightWall() && !checkRightVisited()) {
    turnRight();
    moveForward();
  } else if (!checkLeftWall() && !checkLeftVisited()) {
    turnLeft();
    moveForward();
  } else if (!checkFrontWall()) {
    moveForward();
  } else if (!checkRightWall()) {
    turnRight();
    moveForward();
  } else if (!checkLeftWall()) {
    turnLeft();
    moveForward();
  } else {
    turnLeft();
    turnLeft();
  }
  printMaze();
}
