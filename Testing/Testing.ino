#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>

#define WHITE 0x7

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Servo LServo, RServo;

enum DIRECTION_NAMES { NORTH, EAST, SOUTH, WEST, NONE };
enum FLOW_STATE { SETUP_ONE, PATH_PLANNING, SETUP_TWO, SHORTEST_PATH };
enum cell_state { V, U };
enum wall_state { NO_WALL, WALL };

// Setup variables
int setup_option = 0,
    setup_direction = NORTH,
    setup_start_location = 0,
    setup_end_location = 0;

// Control variables
int robot_state = SETUP_ONE;
int current_row = 1,
    current_col = 1,
    current_direction = EAST;
int target_row = 1,
    target_col = 1;

struct cell {
  int cell_state;
  int north_cell;
  int south_cell;
  int west_cell;
  int east_cell;
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

void printTestMessage();
void printErrorMessage();
void printHelper(String msg, int info);
void printHelper(String msg, String info);
String getDirection(int direction_to_check);
void runButtonsSetup(int &value, int range);
void setupStartLocation();
void setupEndLocation();
void setupStartDirection();
void setupInitialValues();
void intialize();

void setup() {
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
      initialize();
      break;
    case PATH_PLANNING:
      printTestMessage();
      break;
    default:
      printErrorMessage();
  }
}

void printTestMessage() {
  Serial.print("\nENTERED START LOCATION: ");
  Serial.print(setup_start_location);
  Serial.print("\nENTERED END LOCATION: ");
  Serial.print(setup_end_location);
  Serial.print("\nCURRENT ROBOT STATE: ");
  Serial.print(robot_state);
  Serial.print("\nCURRENT ROW: ");
  Serial.print(current_row);
  Serial.print("\nCURRENT COL: ");
  Serial.print(current_col);
  Serial.print("\nCURRENT DIRECTION: ");
  Serial.print(getDirection(current_direction));
  Serial.print("\nTARGET ROW: ");
  Serial.print(target_row);
  Serial.print("\nTARGET COL: ");
  Serial.print(target_col);
  delay(50000);
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

String getDirection(int direction_to_check) {
  switch(direction_to_check) {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case WEST: return "WEST";
    case SOUTH: return "SOUTH";
    default: return "NONE";
  }
}

void runButtonsSetup(int &value, int range) {
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
  runButtonsSetup(setup_start_location, 16);
}

void setupEndLocation() {
  printHelper("End Location?", setup_end_location);
  runButtonsSetup(setup_end_location, 16);
}

void setupStartDirection() {
  printHelper("Start Direction?", getDirection(setup_direction));
  runButtonsSetup(setup_direction, 4);
}

void setupInitialValues() {
  current_direction = setup_direction;
  current_row = (setup_start_location / 4) * 2 + 1;
  current_col = (setup_start_location % 4) * 2 + 1;
  target_row = (setup_end_location / 4) * 2 + 1;
  target_col = (setup_end_location % 4) * 2 + 1;
  robot_state = PATH_PLANNING;
}

void initialize() {
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
