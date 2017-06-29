/*
 * AUTHOR: Loredan E. Bucur
 * LAST_EDIT: 2017 Mar
 *
 */
 
#define DEBUG 1
#define MAX_DISTANCE 200 //cm
#define MMAX 255 //pwm_max
#define MMIN 0 //pwm_min
#define TASKS 3 //implemented task count
#define TASK0 0 //check obstacle
#define TASK1 1 //buzz distance
#define TASK2 2 //debug info
#define BIT(x) (0x1 << x)

#include <SoftwareSerial.h> //Bluetooth RX/TX
#include <Debug.h>          //DBG_printsln()
#include <NewPing.h>        //SONAR
#include <NewTone.h>        //tone generator for buzzer
#include <HC05.h>           //bluetooth master/slave

struct {
  byte const sonarEcho = 17; //A3, digital in
  byte const sonarTrig = 16; //A2, digital out
  byte const buzzer = 8; //D8, digital out
  byte const sw1 = A0; //LSB
  byte const sw2 = A1; //active tasks switch
  byte const sw3 = 4; //MSB
  byte const ML1 = 9; //pwm
  byte const ML2 = 5; //pwm
  byte const MR1 = 6; //pwm
  byte const MR2 = 7; //pwm
  byte const btrx = 3; //D2
  byte const bttx = 2; //D3
  byte const led = 13;
} Pins;

NewPing Sonar(Pins.sonarTrig, Pins.sonarEcho, MAX_DISTANCE);
SoftwareSerial Bluetooth = SoftwareSerial(Pins.bttx, Pins.btrx);

struct {
  byte active; //bit mask
  unsigned long interval[TASKS]; //repeat interval, ms
  unsigned long time[TASKS]; //last execution time, ms
} Tasks;

bool stopped = false;

void setup()
{
  pinMode(Pins.sonarTrig, OUTPUT);
  pinMode(Pins.buzzer, OUTPUT);
  pinMode(Pins.ML1, OUTPUT); //PWM
  pinMode(Pins.ML2, OUTPUT); //PWM
  pinMode(Pins.MR1, OUTPUT); //PWM
  pinMode(Pins.MR2, OUTPUT); //PWM
  pinMode(Pins.sw1, INPUT_PULLUP);
  pinMode(Pins.sw2, INPUT_PULLUP);
  pinMode(Pins.sw3, INPUT_PULLUP);
  
  stop();

  Bluetooth.begin(9600);
  Serial.begin(9600);
  DBG_println("SETUP");
  print_commands();
  
  Tasks.interval[TASK0] = 50;   //motion
  Tasks.interval[TASK1] = 1000; //buzzer
  Tasks.interval[TASK2] = 2000; //debug
}

//4 bits available, only 3 used
byte getNibble()
{
  byte nibble = 0;
  nibble |= digitalRead(Pins.sw1) << 0;
  nibble |= digitalRead(Pins.sw2) << 1;
  nibble |= digitalRead(Pins.sw3) << 2;
  return nibble;
}

//should not block while 0 readings
//want the average of nr nonzero readings
byte getDistance(byte nr)
{
  unsigned sum = 0, posnr = 0;
  while (!sum && nr) {
    unsigned cm = Sonar.ping_cm();
    if (cm) {
      sum += cm;
      ++posnr;
    }
    nr--;
  }
  return (posnr) ? (sum / posnr) : 200;
}

void loop()
{
    Tasks.active = getNibble();
  
    //Check obstacle & Adjust trajectory
    if (Tasks.active & BIT(0))
    {
        if (millis() - Tasks.time[TASK0] > Tasks.interval[TASK0])
        {
            //DBG_printsln("\nSTART_TASK_0");
            unsigned obstacle = getDistance(2); //#tries as parameter
            DBG_prints("SONAR: "); DBG_print(obstacle); DBG_printsln("cm");
            if (obstacle) {
                if (obstacle < 30) { //less than 20cm ahead
                    stop();
                    delay(1000);
                    setBackward(MMAX, MMAX);
                    delay(3000);
                    stop();
                    delay(500);
                }
                else {
                    setForward(MMAX, MMAX);
                }
            }
            //DBG_printsln("END_TASK_0");
            Tasks.time[TASK0] = millis();
        }
    }
    else
    {
        stop();
    }
    
    //Buzz distance
    if ((Tasks.active & BIT(1)) && (millis() - Tasks.time[TASK1] > Tasks.interval[TASK1])) 
    {
        //DBG_printsln("\nSTART_TASK_1");
        unsigned distance = getDistance(2); //#tries
        DBG_prints("SONAR: "); DBG_print(distance); DBG_printsln("cm");
        if (distance <= 100) {
            distance *= 100; //map to frequency
            NewTone(Pins.buzzer, 10000 - distance, 100);
        }
        //DBG_printsln("END_TASK_1");
        Tasks.time[TASK1] = millis();
    }

    //Debug task
    if ((Tasks.active & BIT(2)) && (millis() - Tasks.time[TASK2] > Tasks.interval[TASK2])) 
    {
        DBG_printsln("\nSTART_TASK_2");        
        DBG_printsln("END_TASK_2");
        Tasks.time[TASK2] = millis();
    }
    
    while(readBluetooth()); //bluetooth connected buffer
    readBuffer(); //get command from input serial
}

bool readBluetooth()
{
    if (Bluetooth.available())
    {
        char c = Bluetooth.read();
        switch (c)
        {
            case '|':
                DBG_printsln("FW");
                setForward(MMAX, MMAX);
            break;
            
            case '*':
                DBG_printsln("STOP");
                stop();
            break;
            
            case '%':
                DBG_printsln("BW");
                setBackward(MMAX, MMAX);
            break;
            
            case '@':
                DBG_printsln("HORN");
                horn(600); //freq
            break;
            
            case '<':
                DBG_printsln("LEFT");
                setForward(MMIN, MMAX);
            break;
            
            case '>':
                DBG_printsln("RIGHT");
                setForward(MMAX, MMIN);
            break;
            
            default:
            break;
        }
        return true;
    }
    else {
        return false; //no more commands
    }
}

void readBuffer()
{
    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
            case 'b': //HC-05 AT line
                debugBluetooth();
            break;
            
            case 'm': //check memory
                DBG_prints("Free RAM: ");
                DBG_print(get_free_ram());
                DBG_printsln(" bytes");
            break;

            case 'w': //fw
              setForward(MMAX, MMAX);
            break;

            case 'c': //stop
              stop();
            break;

            case 's': //reverse
              setBackward(MMAX, MMAX);
            break;

            case 'a': //left
              setForward(MMIN, MMAX);
            break;

            case 'd': //right
              setForward(MMAX, MMIN);
            break;
            
            default:
                print_commands();
            break;
        }
    }
    else
    {
      while(-1 != Serial.read());
    }
}

void print_commands()
{
    DBG_printsln("Commands:");
    DBG_printsln("  b: debug Bluetooth");
    DBG_printsln("  w: move forward");
    DBG_printsln("  s: move backward");
    DBG_printsln("  a: rotate left");
    DBG_printsln("  d: rotate right");
    DBG_printsln("  c: stop");
    DBG_printsln("  m: show free RAM");
}

void debugBluetooth()
{
#if DEBUG
    DBG_printsln("\n** Bluetooth terminal **");
    char c = 100; //backquote ASCII
    do {
        while (Bluetooth.available()) {
            Serial.write(Bluetooth.read());
        }
        while (Serial.available()) {
            c = Serial.read();
            Bluetooth.write(c);
            Serial.write(c);
        }
    } while (c != '`');
    DBG_printsln("\n** Quit terminal **");
#endif
}

void horn(unsigned freq)
{
    NewTone(Pins.buzzer, freq, 300);
    delay(600);
    NewTone(Pins.buzzer, freq, 400);
}

void setForward(byte left, byte right)
{
  analogWrite(Pins.ML1, left);
  analogWrite(Pins.ML2, MMIN);
  analogWrite(Pins.MR1, right);
  analogWrite(Pins.MR2, MMIN);
  stopped = false;
}

void setBackward(byte left, byte right)
{
  analogWrite(Pins.ML1, MMIN);
  analogWrite(Pins.ML2, left);
  analogWrite(Pins.MR1, MMIN);
  analogWrite(Pins.MR2, right);
  stopped = false;
}

void stop()
{
    setBackward(0, 0);
    stopped = true;
}
