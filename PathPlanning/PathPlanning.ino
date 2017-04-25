#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#include <QueueArray.h>

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define WHITE 0x7
#define SENSOR_OUT 8
#define FRONT_SENSOR A0
#define LEFT_SENSOR A1
#define RIGHT_SENSOR A2
#define LEFT_ENCODER_PIN 10
#define RIGHT_ENCODER_PIN 11
#define SENSOR_SENSITIVITY 11


Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Servo LServo, RServo;

enum DIRECTION_NAMES { NORTH, EAST, SOUTH, WEST, NONE };
enum FLOW_STATE { SETUP_ONE, PATH_PLANNING, SETUP_TWO, SHORTEST_PATH };
enum CELL_STATE { U, V };
enum WALL_STATE { NO_WALL, WALL };

// Setup variables
int setup_option = 0,
    setup_direction = NORTH,
    setup_start_location = 0,
    setup_end_location = 0;

// Control variables
int robot_state = SETUP_ONE;
int current_row = 0,
    current_col = 0,
    current_direction = EAST;
int target_row = 0,
    target_col = 0;
int target_encoder_count,
    left_encoder_count,
    right_encoder_count,
    left_encoder_last = 0,
    right_encoder_last = 0;
int front_distance,
    left_distance,
    right_distance;

bool looking_for_color = false;
String color = "none";

struct cell {
  int state;
  int north;
  int south;
  int west;
  int east;
};

cell maze[4][4] = {
  {
    { U, WALL, NO_WALL, WALL, NO_WALL },
    { U, WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, WALL, NO_WALL, NO_WALL, WALL }
  }, {
    { U, NO_WALL, NO_WALL, WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, WALL }
  }, {
    { U, NO_WALL, NO_WALL, WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, NO_WALL, NO_WALL, WALL }
  }, {
    { U, NO_WALL, WALL, WALL, NO_WALL },
    { U, NO_WALL, WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, WALL, NO_WALL, NO_WALL },
    { U, NO_WALL, WALL, NO_WALL, WALL }
  }
};

int brushfire_maze[4][4] = { 0 };
QueueArray<int> queue;

void printErrorMessage();
void printHelper(String msg, int info);
void printHelper(String msg, String info);
void updateDirection(int x);
String getDirection(int direction_to_check);
void buttonsSetup(int &value, int range);
void setupStartLocation();
void setupEndLocation();
void setupStartDirection();
void setupInitialValues();
void runInitialSetup();
void runSecondSetup();
void readAvgSensorValues(int d);
void updateLcdColor();
void readColors();
void markBoard();
void getVisitedCount();
bool checkWall(int value);
bool checkFrontWall();
bool checkLeftWall();
bool checkRightWall();
bool checkFrontVisited();
bool checkLeftVisited();
bool checkRightVisited();
void setEncoderCounts(int target, int initial);
void updateEncoderCounts();
void stop(int x);
void moveForward();
void turnLeft();
void turnRight();
void runPathPlanning();
void checkSides(int position);
void brushfire();

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  LServo.attach(2);
  RServo.attach(3);
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(WHITE);
}


void loop() {
  switch(robot_state) {
    case SETUP_ONE:
      runInitialSetup();
      break;
    case PATH_PLANNING:
      runPathPlanning();
      break;
    case SETUP_TWO:
      runSecondSetup();
      break;
    case SHORTEST_PATH:
    default:
      printErrorMessage();
  }
}

void printErrorMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR");
  lcd.setCursor(0, 1);
  lcd.print("\\/(^.^)\\/");
  delay(5000);
}

void printHelper(String msg, int info) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print(info);
}

void printHelper(String msg, String info) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print(info);
}

void updateDirection(int x) {
  current_direction = (current_direction + 4 + x) % 4;
}

String getDirection(int direction_to_check) {
  switch(direction_to_check) {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case WEST: return "WEST";
    case SOUTH: return "SOUTH";
    default: return "NONE";
  }
}

void buttonsSetup(int &value, int range) {
  uint8_t buttons;
  while(!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_UP) {
      value = (value + range + 1) % range;
    }
    if (buttons & BUTTON_DOWN) {
      value = (value + range - 1) % range;
    }
    if (buttons & BUTTON_LEFT) {
      setup_option = (setup_option + 3 - 1) % 3;
    }
    if (buttons & BUTTON_RIGHT) {
      setup_option = (setup_option + 3 + 1) % 3;
    }
    if (buttons & BUTTON_SELECT) {
      setup_option = 3;
    }
  }
}

void setupStartLocation() {
  printHelper("Start Location?", setup_start_location);
  buttonsSetup(setup_start_location, 16);
}

void setupEndLocation() {
  printHelper("End Location?", setup_end_location);
  buttonsSetup(setup_end_location, 16);
}

void setupStartDirection() {
  printHelper("Start Direction?", getDirection(setup_direction));
  buttonsSetup(setup_direction, 4);
}

void setupInitialValues() {
  current_direction = setup_direction;
  current_row = (setup_start_location / 4);
  current_col = (setup_start_location % 4);
  target_row = (setup_end_location / 4);
  target_col = (setup_end_location % 4);
  robot_state = PATH_PLANNING;
  maze[current_row][current_col].state = V;
}

void runInitialSetup() {
  switch(setup_option) {
    case 0:
      setupStartLocation();
      break;
    case 1:
      setupEndLocation();
      break;
    case 2:
      setupStartDirection();
      break;
    case 3:
      setupInitialValues();
      break;
    default:
      printErrorMessage();
  }
  delay(200);
}

void runSecondSetup() {
  printHelper("Go to Start", "Hit Select");
  uint8_t buttons;
  while(!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_SELECT) {
      robot_state = SHORTEST_PATH;
    }
  }
}

void readAvgSensorValues(int d = 100) {
  float front[15], right[15], left[15];
  for(int x = 0; x < 15; x++) {
    front[x] = analogRead(FRONT_SENSOR);
    right[x] = analogRead(RIGHT_SENSOR);
    left[x] = analogRead(LEFT_SENSOR);
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
  front_distance = 500*pow(mdn_front, -0.85);
  right_distance = 500*pow(mdn_right, -0.85);
  left_distance = 500*pow(mdn_left, -0.85);
}

void updateLcdColor() {
  switch(current_direction) {
    case NORTH:
      lcd.setBacklight(BLUE);
      break;
    case EAST:
      lcd.setBacklight(RED);
      break;
    case SOUTH:
      lcd.setBacklight(YELLOW);
      break;
    case WEST:
      lcd.setBacklight(GREEN);
      break;
  }
  looking_for_color = false;
}

void readColors() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  int fRed = pulseIn(SENSOR_OUT, LOW);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  int fGreen = pulseIn(SENSOR_OUT, LOW);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  int fBlue = pulseIn(SENSOR_OUT, LOW);

  if (looking_for_color && (fRed < 200 || fBlue < 200) && color == "none") {
    color = "lit";
    updateLcdColor();
  } else if (!(fRed < 200 || fBlue < 200) && color != "none") {
    color = "none";
    lcd.setBacklight(WHITE);
  }
}
void markBoard() {
  if (current_direction == NORTH) current_row -= 1;
  if (current_direction == EAST) current_col += 1;
  if (current_direction == SOUTH) current_row += 1;
  if (current_direction == WEST) current_col -= 1;
  maze[current_row][current_col].state = V;
}

void getVisitedCount() {
  int count = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (maze[i][j].state) count += 1;
    }
  }
  if (count == 16) {
    readAvgSensorValues();
    checkFrontWall();
    checkLeftWall();
    checkRightWall();
    current_direction = setup_direction;
    current_row = (setup_start_location / 4);
    current_col = (setup_start_location % 4);
    robot_state = SETUP_TWO;
    brushfire();
  }
}

bool checkWall(int value) {
  return value < SENSOR_SENSITIVITY && value > 0;
}

bool checkFrontWall() {
  bool isWall = checkWall(front_distance);
  switch(current_direction) {
    case NORTH:
      if (current_row == 0) return true;
      if (isWall) {
        maze[current_row][current_col].north = WALL;
        maze[current_row - 1][current_col].south = WALL;
      }
      break;
    case EAST:
      if (current_col == 3) return true;
      if (isWall) {
        maze[current_row][current_col].east = WALL;
        maze[current_row][current_col + 1].west = WALL;
      }
      break;
    case SOUTH:
      if (current_row == 3) return true;
      if (isWall) {
        maze[current_row][current_col].south = WALL;
        maze[current_row + 1][current_col].north = WALL;
      }
      break;
    case WEST:
      if (current_col == 0) return true;
      if (isWall) {
        maze[current_row][current_col].west = WALL;
        maze[current_row][current_col - 1].east = WALL;
      }
      break;
    default:
      return true;
  }
  return isWall;
}

bool checkLeftWall() {
  bool isWall = checkWall(left_distance);
  switch(current_direction) {
    case EAST:
      if (current_row == 0) return true;
      if (isWall) {
        maze[current_row][current_col].north = WALL;
        maze[current_row - 1][current_col].south = WALL;
      }
      break;
    case SOUTH:
      if (current_col == 3) return true;
      if (isWall) {
        maze[current_row][current_col].east = WALL;
        maze[current_row][current_col + 1].west = WALL;
      }
      break;
    case WEST:
      if (current_row == 3) return true;
      if (isWall) {
        maze[current_row][current_col].south = WALL;
        maze[current_row + 1][current_col].north = WALL;
      }
      break;
    case NORTH:
      if (current_col == 0) return true;
      if (isWall) {
        maze[current_row][current_col].west = WALL;
        maze[current_row][current_col - 1].east = WALL;
      }
      break;
    default:
      return true;
  }
  return isWall;
}

bool checkRightWall() {
  bool isWall = checkWall(right_distance);
  switch(current_direction) {
    case WEST:
      if (current_row == 0) return true;
      if (isWall) {
        maze[current_row][current_col].north = WALL;
        maze[current_row - 1][current_col].south = WALL;
      }
      break;
    case NORTH:
      if (current_col == 3) return true;
      if (isWall) {
        maze[current_row][current_col].east = WALL;
        maze[current_row][current_col + 1].west = WALL;
      }
      break;
    case EAST:
      if (current_row == 3) return true;
      if (isWall) {
        maze[current_row][current_col].south = WALL;
        maze[current_row + 1][current_col].north = WALL;
      }
      break;
    case SOUTH:
      if (current_col == 0) return true;
      if (isWall) {
        maze[current_row][current_col].west = WALL;
        maze[current_row][current_col - 1].east = WALL;
      }
      break;
    default:
      return true;
  }
  return isWall;
}

bool checkFrontVisited() {
  switch(current_direction) {
    case NORTH:
      return current_row == 0 || maze[current_row - 1][current_col].state;
    case EAST:
      return current_col == 3 || maze[current_row][current_col + 1].state;
    case SOUTH:
      return current_row == 3 || maze[current_row + 1][current_col].state;
    case WEST:
      return current_col == 0 || maze[current_row][current_col - 1].state;
    default:
      return true;
  }
}

bool checkLeftVisited() {
  switch(current_direction) {
    case EAST:
      return current_row == 0 || maze[current_row - 1][current_col].state;
    case SOUTH:
      return current_col == 3 || maze[current_row][current_col + 1].state;
    case WEST:
      return current_row == 3 || maze[current_row + 1][current_col].state;
    case NORTH:
      return current_col == 0 || maze[current_row][current_col - 1].state;
    default:
      return true;
  }
}

bool checkRightVisited() {
  switch(current_direction) {
    case WEST:
      return current_row == 0 || maze[current_row - 1][current_col].state;
    case NORTH:
      return current_col == 3 || maze[current_row][current_col + 1].state;
    case EAST:
      return current_row == 3 || maze[current_row + 1][current_col].state;
    case SOUTH:
      return current_col == 0 || maze[current_row][current_col - 1].state;
    default:
      return true;
  }
}

void setEncoderCounts(int target = 100, int initial = 101) {
  target_encoder_count = target;
  left_encoder_count = initial;
  right_encoder_count = initial;
}

void updateEncoderCounts() {
  int left_encoder_value = digitalRead(LEFT_ENCODER_PIN);
  int right_encoder_value = digitalRead(RIGHT_ENCODER_PIN);
  if (left_encoder_value != left_encoder_last) left_encoder_count++;
  if (right_encoder_value != right_encoder_last) right_encoder_count++;
  left_encoder_last = left_encoder_value;
  right_encoder_last = right_encoder_value;
}

void stop(int x = 100) {
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);
  delay(x);
}

void moveForward() {
  markBoard();
  setEncoderCounts(145, 0);
  LServo.writeMicroseconds(1525);
  RServo.writeMicroseconds(1476);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    readColors();
    updateEncoderCounts();
  }
  stop();
  setEncoderCounts();

}

void turnLeft() {
  updateDirection(-1);
  setEncoderCounts(30, 0);
  LServo.writeMicroseconds(1480);
  RServo.writeMicroseconds(1480);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    updateEncoderCounts();
  }
  stop();
  setEncoderCounts();

}

void turnRight() {
  updateDirection(1);
  setEncoderCounts(31, 0);
  LServo.writeMicroseconds(1520);
  RServo.writeMicroseconds(1520);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    updateEncoderCounts();
  }
  stop();
  setEncoderCounts();

}


void runPathPlanning() {
  readAvgSensorValues();
  bool wallLeft = checkLeftWall();
  bool wallFront = checkFrontWall();
  bool wallRight = checkRightWall();
  if (!wallFront && !checkFrontVisited()) {
     moveForward();
  } else if (!wallLeft && !checkLeftVisited()) {
     turnLeft();
  } else if (!wallRight && !checkRightVisited()) {
     turnRight();
  } else if (!wallFront) {
     moveForward();
  } else if (!wallLeft) {
     turnLeft();
  } else if (!wallRight) {
     turnRight();
  } else {
     turnLeft();
     turnLeft();
  }
  getVisitedCount();
}

void checkSides(int position) {
  int row = position / 4;
  int col = position % 4;
  int cost = brushfire_maze[row][col] + 1;
  // if there is no wall west
  if (!maze[row][col].west && !brushfire_maze[row][col - 1]) {
    brushfire_maze[row][col - 1] = cost;
    queue.push(row*4 + (col - 1));
  }
  if (!maze[row][col].north && !brushfire_maze[row - 1][col]) {
    brushfire_maze[row - 1][col] = cost;
    queue.push((row - 1) * 4 + col);
  }
  if (!maze[row][col].east && !brushfire_maze[row][col + 1]) {
    brushfire_maze[row][col + 1] = cost;
    queue.push(row * 4 + (col + 1));
  }
  if (!maze[row][col].south && !brushfire_maze[row + 1][col]) {
    brushfire_maze[row + 1][col] = cost;
    queue.push((row + 1) * 4 + col);
  }
}

void brushfire() {
  brushfire_maze[target_row][target_col] = 1;
  queue.push(target_row * 4 + target_col);
  while(!queue.isEmpty()) {
    int position = queue.pop();
    checkSides(position);
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(brushfire_maze[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
