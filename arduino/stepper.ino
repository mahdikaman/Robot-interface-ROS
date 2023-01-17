
/**
 *  Data message for moving joint
 *
 *
   7   6   5   4   3   2   1   0
 ---------------------------------
 | C | C | D | J | J | D | J | J |
 |   |   |   | 2 | 1 |   | 2 | 1 |
 ---------------------------------
 * C = Control bits are always set to one
 * J = Joint, specifies which joint to use (0-4)
 * D = Direction, specifies which direction to move
       the joint. (0-1)
*/

// LEFT ARM
#define LA 2
#define LB 3
#define LC 4
#define LD 5

// RIGHT ARM
#define RA 6
#define RB 7
#define RC 8
#define RD 9

// HEAD
#define HA 10
#define HB 11
#define HC 12
#define HD 13

// JOINTS
#define HEAD 0
#define LEFT_ARM 1
#define RIGHT_ARM 2

// DIRECTION
#define CW 0
#define CCW 1

// Struct
struct joint_pin {
  char a;
  char b;
  char c;
  char d;
};

// Global const
const int tempo = 1;

// Global vars
char data;
char direction;
char control;
char joint;
char mirrorData;
char moveData;
joint_pin leftArm = { LA, LB ,LC ,LD };
joint_pin rightArm = { RA, RB ,RC ,RD };
joint_pin head = { HA, HB ,HC, HD };

void write(char a, char b, char c, char d, char j) {
  joint_pin driver;
  switch (j) {
    case LEFT_ARM:
      driver = leftArm;
      break;
    case RIGHT_ARM:
      driver = rightArm;
      break;
    case HEAD:
      driver = head;
      break;
  }

  digitalWrite(driver.a, a);
  digitalWrite(driver.b, b);
  digitalWrite(driver.c, c);
  digitalWrite(driver.d, d);
}

void move_CCW(char j) {
  write(1, 0, 0, 0, j);
  delay(tempo);
  write(1, 1, 0, 0, j);
  delay(tempo);
  write(0, 1, 0, 0, j);
  delay(tempo);
  write(0, 1, 1, 0, j);
  delay(tempo);
  write(0, 0, 1, 0, j);
  delay(tempo);
  write(0, 0, 1, 1, j);
  delay(tempo);
  write(0, 0, 0, 1, j);
  delay(tempo);
  write(1, 0, 0, 1, j);
  delay(tempo);
  write(0, 0, 0, 0, j);
}

void move_CW(char j) {
  write(1, 0, 0, 1, j);
  delay(tempo);
  write(0, 0, 0, 1, j);
  delay(tempo);
  write(0, 0, 1, 1, j);
  delay(tempo);
  write(0, 0, 1, 0, j);
  delay(tempo);
  write(0, 1, 1, 0, j);
  delay(tempo);
  write(0, 1, 0, 0, j);
  delay(tempo);
  write(1, 1, 0, 0, j);
  delay(tempo);
  write(1, 0, 0, 0, j);
  delay(tempo);
  write(0, 0, 0, 0, j);
}

void move(char j, char direction) {
  switch (direction) {
    case CW:
      move_CW(j);
      break;
    case CCW:
      move_CCW(j);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(LA, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(LC, OUTPUT);
  pinMode(LD, OUTPUT);
  pinMode(RA, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(RC, OUTPUT);
  pinMode(RD, OUTPUT);
  pinMode(HA, OUTPUT);
  pinMode(HB, OUTPUT);
  pinMode(HC, OUTPUT);
  pinMode(HD, OUTPUT);
}

int counter = 0;
void loop() {
  
  if (Serial.available() > 0) {
    // Read one byte
    data = Serial.read();

    // Bitshift to get to controlbits
    control = (data & 0b11000000) >> 6;
    // Control bits are set
    if (control == 0b11) {
      // Extract data
      moveData = data & 0b00000111;
      mirrorData = (data & 0b00111000) >> 3;
      // If the data is mirrored (control n2)
      if (moveData == mirrorData) {
        // Extract joint and direction
        joint = moveData & 0b11;
        direction = (moveData & 0b100) >> 2;
        // Move the joint in the direction
        move(joint, direction);
        delay(1);
        move(joint, direction);
        delay(1);
        move(joint, direction);
        delay(1);
        counter++;
        //Serial.println(counter);
      }
    }
  }
  //delay(1);
}
