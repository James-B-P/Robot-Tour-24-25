// ROBOT INSTRUCTIONS: Should be the only modified part during competitions. Path is programmed
// as a matrix of points it passes. Currently, adjacent points must share either an x value or
// a y value.
// INSTRUCTIONS_LEN must be equal to the number of points in the instructions matrix.
const int INSTRUCTIONS_LEN = 1; 
// Example matrix of instructions to trace a figure eight:
const int instructions[INSTRUCTIONS_LEN][2] = 
{
  {100, 0},
};
// Robot's starting position and orientation
volatile int robot_direction = 0;
volatile int robot_x = 0;
volatile int robot_y = 0;

// Pin Constants
const int MLE = 7;
const int MLA = 6;
const int MLB = 5;
const int MRA = 4;
const int MRB = 3;
const int MRE = 2;
const int INL = 18;
const int INR = 19;

const int BUTTON = 21;

// Motor control constants
const float K_P =  10;
const float K_I =  0.005;
const float K_D =  1;
const float K_Pt = 50;
const float K_It = 0;
const float K_Dt = 1.5;
const int DELAY = 20;
const float L2R_RATIO = 1.05;
const float L2R_SIZE = 1.01;
const float TICKS_PER_UNIT = 1;

// Pulse constants
const int PULSE_POWER = 255;
const int PULSE_DELAY = 0;

// Motor class: Used to simplify interactions between program and physical motors
class Motor
{
  public:
    Motor(int InA, int InB, int En):InA(InA), InB(InB), En(En){};
    volatile float power = 0;
    volatile int ticks = 0;

    // Updates signal to L298N
    void run()
    {
      if (power > 0)
      {
      analogWrite(En, (int)(power));
      digitalWrite(InA, LOW);
      digitalWrite(InB, HIGH);
      }
      if (power < 0)
      {
        analogWrite(En, (int)(-power));
        digitalWrite(InA, HIGH);
        digitalWrite(InB, LOW);
      }
      if (power == 0)
      {
        stop();
      }
    }

    void stop()
    {
      analogWrite(En, 255);
      digitalWrite(InA, LOW);
      digitalWrite(InB, LOW);
    }

    void set_power(float newPower)
    {
      power = min(255, max(-255, newPower));
      run();
    }

    void inc_power(float delta)
    {
      set_power(power+delta);
    }

    void dec_power(float delta)
    {
      set_power(power-delta);
    }

  private:
    const int InA;
    const int InB;
    const int En;
};

// Motor setup
Motor left_motor(MLA, MLB, MLE);
Motor right_motor(MRA, MRB, MRE);

// Tracks wheel encoder ticks
void incL()
{
  if (left_motor.power != 0)
  {
    left_motor.ticks += abs(left_motor.power)/left_motor.power;
  }
}

void incR()
{
  if (right_motor.power != 0)
  {
    right_motor.ticks += abs(right_motor.power)/right_motor.power;
  }
}

int left_ticks()
{
  return left_motor.ticks;
}

int right_ticks()
{
  return right_motor.ticks;
}

// Gives a short pulse to the motors to either quickly slow down or speed up
//   • left_direction and right_direction are ints either -1 or 1
void retro_pulse(int left_direction, int right_direction)
{
  left_motor.set_power(left_direction*PULSE_POWER);
  right_motor.set_power(right_direction*PULSE_POWER);
  delay(PULSE_DELAY);
  left_motor.set_power(0);
  right_motor.set_power(0);
}

// Using PID motor control, rotates motors in a given direction for a given length
//   • left_direction and right_direction are ints either -1 or 1
//   • base_power is an int between 0 and 255
//   • total_ticks is a positive int corresponding to the total number of wheel encoder 
//     ticks counted in the movement
void move_PID(int left_direction, int right_direction, int slow_power, float acc_millis, float base_power, int total_ticks, bool is_turning)
{
  int avg_ticks;
  int tick_dif;
  int integrated_tick_dif;
  float PID;
  int prev_tick_dif;
  unsigned long prevMillis;
  unsigned long startMillis;
  unsigned long nowMillis;
  unsigned long deltaTime;
  int ticksL;
  int ticksR;
  float left_base;
  float right_base;

  left_motor.ticks = 0;
  right_motor.ticks = 0;
  avg_ticks = 0;
  integrated_tick_dif = 0;
  left_base = slow_power*L2R_RATIO;
  right_base = slow_power;

  int time = 0;

  // retro_pulse(left_direction, right_direction);

  left_motor.set_power(left_direction*left_base);
  right_motor.set_power(right_direction*right_base);

  prevMillis = millis();
  startMillis = millis();

  while (avg_ticks < total_ticks)
  {
    // Timing
    time+=1;
    delay(DELAY);
    nowMillis = millis();
    deltaTime = nowMillis-prevMillis;
    prevMillis = nowMillis;

    // Access encoder ticks
    ticksL = left_ticks()*L2R_SIZE;
    ticksR = right_ticks();

    Serial.print(left_ticks());
    Serial.print(",");
    Serial.println(right_ticks());

    // Encoder ticks update
    avg_ticks = (ticksL*left_direction+ticksR*right_direction)/2;
    
    // PID control
    tick_dif = left_direction*ticksL-right_direction*ticksR;
    integrated_tick_dif += tick_dif*deltaTime;
    if (is_turning)
    {
      PID = K_Pt*tick_dif + K_It*integrated_tick_dif + K_Dt*(tick_dif-prev_tick_dif)/deltaTime;
    } else
    {
      PID = K_P*tick_dif + K_I*integrated_tick_dif + K_D*(tick_dif-prev_tick_dif)/deltaTime;
    }

    // Serial.print(tick_dif);
    // Serial.print(",");
    // Serial.print(avg_ticks);
    // Serial.print(",");
    // Serial.print(K_P*tick_dif);
    // Serial.print(",");
    // Serial.print(K_I*integrated_tick_dif);
    // Serial.print(",");
    // Serial.println(K_D*(tick_dif-prev_tick_dif)/deltaTime);'
    prev_tick_dif = tick_dif;
    left_motor.set_power(left_direction*(left_base-PID));
    right_motor.set_power(right_direction*(right_base+PID));

    if (right_base < base_power)
    {
      right_base += (base_power-slow_power)/acc_millis*deltaTime;
      left_base = right_base*L2R_RATIO;
    }
    Serial.print(-1000);
    Serial.print(",");
    Serial.print(1000);
    Serial.print(",");
    // Serial.println(PID);
  }

  left_motor.set_power(0);
  right_motor.set_power(0);
}

void forward(int units)
{
  move_PID(1, 1, 100, 0, 100, (int)(TICKS_PER_UNIT*units), false);
  delay(200);
}

void turn(int degrees_clockwise)
{
  if (degrees_clockwise > 0)
  {
    move_PID(1, -1, 90, 0, 90, degrees_clockwise*18/180, true);
  } else
  {
    move_PID(-1, 1, 90, 0, 90, -degrees_clockwise*18/180, true);
  } 
  delay(200);
}

// Lines robot up for next movement
//   • new_direction is an int between 0 and 3 with each value corresponding
//     to a different direction by increments of 90°
void rotate_to(int new_direction)
{
  int dir_change = (new_direction-robot_direction+2)%4-2;
  if (dir_change != 0)
  {
    turn(90*dir_change);
  }
  robot_direction += dir_change;
}

volatile bool paused = true;

void startProgram()
{
  paused = false;
}

void setup()
{
  // General setup and wait for button press to start program
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(INL), incL, RISING);
  attachInterrupt(digitalPinToInterrupt(INR), incR, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON), startProgram, FALLING);
  while (paused){}

  delay(1000);
  int x_move;
  int y_move;

  // Iterate through instructions matrix and move accordingly
  for (int i = 0; i < INSTRUCTIONS_LEN; i++)
  {
    x_move = instructions[i][0]-robot_x;
    y_move = instructions[i][1]-robot_y;

    if (x_move > 0)
    {
      rotate_to(1);
      forward(x_move);
    } else if (x_move < 0)
    {
      rotate_to(3);
      forward(-x_move);
    } else if (y_move > 0)
    {
      rotate_to(0);
      forward(y_move);
    } else if (y_move < 0)
    {
      rotate_to(2);
      forward(-y_move);
    }
    robot_x += x_move;
    robot_y += y_move;
  }
}

void loop()
{

}
