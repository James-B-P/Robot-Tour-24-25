const int MLE = 2;
const int MLA = 4;
const int MLB = 3;
const int MRA = 6;
const int MRB = 5;
const int MRE = 7;
const int INL = 19;
const int INR = 18;
const int BUTTON = 21;

const float K_P = 0.125;
const float K_I = 0.000005;
const float K_D = 250;
const int DELAY = 50;

const float l2r_power_ratio = 1.2;

const int PULSE_POWER = 200;
const int PULSE_DELAY = 50;

class Motor
{
  public:
    Motor(int InA, int InB, int En, int Interrupt):InA(InA), InB(InB), En(En), Interrupt(Interrupt){};
    float power = 0;
    volatile int ticks = 0;
    float prev_power = 1;

    void run()
    {
      analogWrite(En, (int)(power));
      digitalWrite(InA, LOW);
      digitalWrite(InB, HIGH);
      if (power < 0)
      {
        analogWrite(En, (int)(-power));
        digitalWrite(InA, HIGH);
        digitalWrite(InB, LOW);
      }
    }

    void set_power(float newPower)
    {
      if (newPower == 0 && power != 0)
        prev_power = power;
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
    const int Interrupt;
};

Motor left_motor(MLA, MLB, MLE, INL);
Motor right_motor(MRA, MRB, MRE, INR);

volatile bool paused = true;

void incL()
{
  if (left_motor.power == 0)
  {
    left_motor.ticks += abs(left_motor.prev_power)/left_motor.prev_power;
  } else
  {
    left_motor.ticks += abs(left_motor.power)/left_motor.power;
  }
}

void incR()
{
  if (right_motor.power == 0)
  {
    right_motor.ticks += abs(right_motor.prev_power)/right_motor.prev_power;
  } else
  {
    right_motor.ticks += abs(right_motor.power)/right_motor.power;
  }
}


void startProgram()
{
  paused = false;
}

void p(float v)
{
  Serial.print(v);
  Serial.print(",");
}

int left_ticks()
{
  return left_motor.ticks;
}

int right_ticks()
{
  return right_motor.ticks;
}

void retro_pulse(int left_direction, int right_direction)
{
  left_motor.set_power(-left_direction*PULSE_POWER);
  right_motor.set_power(-right_direction*PULSE_POWER);
  delay(PULSE_DELAY);
  left_motor.set_power(0);
  right_motor.set_power(0);
}

void move_PID(int left_direction, int right_direction, int avg_power, int total_ticks)
{
  int avg_ticks;
  int tick_dif;
  int accum_tick_dif;
  float PID;
  int prev_tick_dif;

  left_motor.ticks = 0;
  right_motor.ticks = 0;
  avg_ticks = 0;
  accum_tick_dif = 0;

  left_motor.set_power(left_direction*avg_power);
  right_motor.set_power(right_direction*avg_power);

  while (avg_ticks < total_ticks)
  {
    // Tick update
    avg_ticks = (left_ticks()*left_direction+right_ticks()*right_direction)/2;
    
    // PID
    tick_dif = left_direction*left_ticks()-right_direction*right_ticks();
    accum_tick_dif += tick_dif;
    PID = K_P*tick_dif + K_I*accum_tick_dif*DELAY + K_D*(tick_dif-prev_tick_dif)/DELAY;
    prev_tick_dif = tick_dif;
    left_motor.dec_power(left_direction*PID);
    right_motor.inc_power(right_direction*PID);

    // Delay
    delay(DELAY);
  }

  retro_pulse(left_direction, right_direction);  
}

void forward(int ticks)
{
  move_PID(1, 1, 100, ticks);
  delay(200);
}

void turn(int degrees_clockwise)
{
  if (degrees_clockwise > 0)
  {
    move_PID(-1, 1, 100, degrees_clockwise/11.25);
  } else
  {
    move_PID(1, -1, 100, -degrees_clockwise/11.25);
  } 
  delay(200);
}


void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(INL), incL, RISING);
  attachInterrupt(digitalPinToInterrupt(INR), incR, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON), startProgram, FALLING);
  while (paused){}
  delay(500);
}

void loop()
{
  delay(500);
  
  forward(16);
  turn(90);
  forward(32);
  turn(90);
  forward(32);
  turn(90);
  forward(32);
  turn(90);
  forward(32);
  turn(-90);
  forward(32);
  turn(-90);
  forward(32);
  turn(-90);
  forward(32);
  turn(-90);
  forward(16);

  
  while (true){}
}
