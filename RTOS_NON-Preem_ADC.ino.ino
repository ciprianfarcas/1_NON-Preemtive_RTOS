/*
 Board: A0- Potentiometru
        7- Switch
        6- LED_3
        5- LED_2
        3- LED_1
        13- LED_13(on Arduino board)
        
Program care foloseste un scheduler pentru a executa 6 taskuri:

Task_1: citeste valoarea unui potentiometru si o afiseaza in portul serial

Task_2: Aprinde LED 3 sau LED2 in functie de val potentiometrului (Daca val este mai mare de jumatate LED_3 ON daca este mai mica de jumatate LED_2 ON)

Task_3: Foloseste un swich statement cu 3 cazuri si afiseaza pe portul un sir inversat cu ajutorul pointerilor in functie de caz

Task4: Seteaza dutyciclelul cu care va lumina un led Duty intr 0- 99%, Duty se citeste prin serial port

Task5: Citeste statusul unui Boten si daca este apasat led 13 clipoceste 

Task6: Verifica daca fiecare task a fos executat cel putin o singura data
*/


#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdlib.h>
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) /// macro to set the bit(the second argument) of the address(the first argument) to 1

#define FREQ 100
#define LF 0x0A //enter
#define VAL_OHM(a) (a * POT/ 1023)
#define VAL_KOHM(b) (((b * POT/ 1023)/1000))


int val;
float POT =10000;//ohm
volatile uint8_t duty;
const float T = 1/FREQ;// perioada la care se executa intreruperea


char sir[3];
char buf [100];
char T1[6] ="TASK_1";
char T2[6] ="TASK_2";
char T3[6] ="TASK_3";
char T4[6] ="TASK_4";
char T5[6] ="TASK_5";
//static variables
static long int T1_count=0;
static long int T2_count=0;
static long int T3_count=0;
static long int T4_count=0;
static long int T5_count=0;
static long int T6_count=0;

const unsigned ntasks = 6;
enum task_state { WAIT, READY, RUN };

struct task
{
  unsigned id;
  unsigned vtime;  // cpu time
  enum task_state state {WAIT};
  void (*f)(void*);
  void *param ;
  unsigned T;  // period
  unsigned D;  // delay
  bool periodic {true};  // periodic or one-shot task?
};

struct task tasks[ntasks];

unsigned add_task(void (*f)(void*), void *param, unsigned t, unsigned d, bool periodic)
{
  for (unsigned i = 0u; i < ntasks; i++)
    if (tasks[i].f == nullptr) {
      tasks[i].f = f;
      tasks[i].param = param;
      tasks[i].T = t;
      tasks[i].D = d;
      tasks[i].periodic = periodic;
      tasks[i].vtime = 0;
      return i;
    }
  return ntasks;
}

void delete_task(unsigned i){
  if (i < ntasks) {
    tasks[i].f = nullptr;
    tasks[i].T = 0u;
    tasks[i].D = 0u;
    tasks[i].state = WAIT;
    tasks[i].id = 0;
    tasks[i].vtime = 0;
  }
}
//====bitfield===============================
struct {
    uint8_t Task1 :1 ;
    uint8_t Task2 :2 ;
    uint8_t Task3 :2 ;
    uint8_t Task4 :3 ;
    uint8_t Task5 :3 ;
    uint8_t Task6 :3 ;
   
} t_bit_field;

//========functions for Tasks====================================================
void Task_1(void *led){
T1_count++;
  
 //Serial.println(T1);
unsigned int val_ohm;
float val_Kohm;
val_ohm = (unsigned int)VAL_OHM(analogRead(A0));// call MACRO
val_Kohm = (float)VAL_KOHM(analogRead(A0));// Call MACRO
 
    sprintf (buf, "POT values is:  %dΩ \n", val_ohm);
    Serial.print (buf);
    printFloat(val_Kohm, 3);// functie pentru print float
    sprintf (buf, "DUTY:  %d \n", duty);
    Serial.print (buf);
    t_bit_field.Task1= 1; // folosit oarecum ca si un flag
}
void Task_2 (void *led)
{
T2_count++;
// Serial.println(T2);
  if(analogRead(A0) > 511)
  {
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH); 
  }
  else
  {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW); 
      
  }
  t_bit_field.Task2= 2; // folosit oarecum ca si un flag
}

 char mesaj_1[15]= "\nEMIERT AMIRP";// PRIMA TREIME
 char mesaj_2[15]= "\nEMIERT AUOD A";// A DOUA TREIME
 char mesaj_3[15]= "\nEMIERT AMITLU";// ULTIMA TREIME

void Task_3 (void *led){
 T3_count++;
 uint16_t value;
 value= analogRead(A0);

  switch(value)
  {
    case 341:
   
      revers_mesaj(mesaj_1);
      Serial.print (mesaj_1);
      revers_mesaj(mesaj_1);
      break;
    case 682:
     
      revers_mesaj(mesaj_2);
      Serial.print (mesaj_2);
      revers_mesaj(mesaj_2);
      break;
    case 1023:
   
      revers_mesaj(mesaj_3);
      Serial.print (mesaj_3);
      revers_mesaj(mesaj_3);
      break;
    default:
      break;
    

  }
 

    t_bit_field.Task3= 3; // folosit oarecum ca si un flag
   
}

void LED(void *led)//Task_4
{
  T4_count++;
 //Serial.println(T4);
    if (Serial.available() > 0)  
    {
    for (int i= 0; i<3;i++)
    {
       sir[i] = Serial.read();
       if (sir[i] == LF)
       {
          duty = (int)(255*(float)atoi(sir)/99);//atoi converts the string sir to an integer
          i=0;
       }
    }
      
 
   }

   PWM(3, duty);// PWM generate on pin 3
  
  t_bit_field.Task4= 4; // folosit oarecum ca si un flag
}

void button(void)//Task 5
{
  T5_count++;
  if( digitalRead(7) == LOW)
  delay(30);// some time for debounce
  if( digitalRead(7) == LOW){
   
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(100);
  }
  t_bit_field.Task5= 5; // folosit oarecum ca si un flag
}

void chack_flags(void)// task 6
{
  T6_count++;
  
  t_bit_field.Task6=6;
  sprintf (buf, "\nTask_%d: %ld, Task_%d: %ld, Task_%d: %ld, Task_%d: %ld, Task_%d: %ld, Task_%d: %ld\n",   t_bit_field.Task1, T1_count,
                                                                                                            t_bit_field.Task2, T2_count,
                                                                                                            t_bit_field.Task3, T3_count,
                                                                                                            t_bit_field.Task4, T4_count,
                                                                                                            t_bit_field.Task5, T5_count,
                                                                                                            t_bit_field.Task6, T6_count);
  Serial.print (buf);
}
//=============================================================================================

//=====INTRRUPT==============================================================================
ISR(TIMER1_COMPA_vect)// la fiecare T= 1/ FREQ
{

  for (unsigned i = 0u; i < ntasks; i++) {
    if (tasks[i].f != nullptr) {
      if (tasks[i].D == 0) {
        tasks[i].state = READY;
        if (tasks[i].periodic)
          tasks[i].D = tasks[i].T;
      } else {
        tasks[i].D--;
      }
    }
    }
}
//===========================================================================================
//=======REVERSE MESAJ=======================================================================
void revers_mesaj(char* msg){
    int l;
    char *inceput, *sfarsit, ch;
    l = strlen(msg);
//setam poiinterii la inceput si sifarsitul mesajului
    inceput = msg;
    sfarsit = msg;
  
    // mut pointerul de sfarsit la sfarsitul mesajului
    for (int i = 0; i < l - 1; i++)
        sfarsit++;
    for (int i = 0; i < l / 2; i++) {
  
        // inversez caracterele
        ch = *sfarsit;
        *sfarsit = *inceput;
        *inceput = ch;
  
        // update pozitie pointeri
        inceput++;
        sfarsit--;
    }
}
 //===========================================================================================

 //======PWM==================================================================================
void PWM(uint8_t pin, int duty)//PWM just for pins wich work with timer 2
{
   //sprintf (buf, "DUTY:  %d\n", duty);
   //Serial.print (buf);
  // TIMER2 is used to generat an PWM on PIN 3 
  pinMode(pin, OUTPUT);
  if (duty == 0)
  {
    digitalWrite(pin, LOW);
  }
  else if (duty == 255)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {   
    // connect pwm to pin on timer 2, channel A
    TCCR2A|= 0b00100000;// COM2B1 is set to 1
    //  TCCR2A|= _BV(COM2B1);
    //sbi(TCCR2A, COM2B1);//Clear OC2B on compare match, set OC2B at BOTTOM,(non-inverting mode).
       OCR2B = duty; // set pwm duty
    }
}
//===========================================================================================

//===========printFloat==================================================================
void printFloat(float number, uint8_t digits){// functie pentru a printa numere float
 float zecimale;
 int int_part= (int) number;

  zecimale= number - int_part;
  zecimale= zecimale* pow(10, digits);
 // zecimale= round(zecimale);// ex: number= 5.20 fara raound va fi afisat 5.19
  
   sprintf (buf, "POT values is:  %d.%d kΩ \n", int_part, (int)zecimale);
   Serial.print (buf);
}
//=============================================================================================

//===========setups============================================================================

void setup_timer()
{
  cli();
  TCCR1A = 0;  // control registers for timer 1
  TCCR1B = 0;
  TCNT1 = 0;  // counter value (increased on every timer tick)

  OCR1A = (16* pow(10, 6) / (256 *FREQ)) -1 ; //16 * 10 ^ 6 / (prescaler * frecv_dortia)
  
  TCCR1B = TCCR1B | (1 << WGM12);// Compare Match (CTC) Mode

 
  TCCR1B |= (1 << CS12); // set the prescaler to 256:
  TIMSK1 |= (1 << OCIE1A);  // output compare mode interrupt
  sei();
}

void setup_board()
{
  pinMode(6, OUTPUT);  
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(7,INPUT);
  digitalWrite(13,LOW);
}

void setup_serial()
{
  Serial.begin(9600);
}
void init_scheduler()
{
  //delay 1000 =10s   1/FREQ * period => perioada in secunde la care se executa un task 
  
 //unsigned add_task(void (*f)(void*), void *param, unsigned t, unsigned d, bool periodic)
 add_task(button, nullptr,1,0,true);//task5-> P1
 add_task(LED,nullptr,5,5,true);//task4 -> P2

 add_task(Task_2,nullptr, 10, 10, true);//task 2-> P3
 add_task(Task_3,nullptr, 6000,15, true);//task 3 -> P4
 add_task(Task_1, nullptr,500,20,true);//task1-> P5
 add_task(chack_flags, nullptr,2000,25,true);//task6-> P6
}
//======================================================================

//==========MAIN_setup==================================================
void setup()
{

  
  init_scheduler();
  setup_timer();  // mandatory

 
  setup_board();
  setup_serial();
  sprintf (buf, "\nTask: %d, Task: %d, Task: %d, Task: %d, Task: %d, Task: %d\n", t_bit_field.Task1,
                                                                                  t_bit_field.Task2,
                                                                                  t_bit_field.Task3,
                                                                                  t_bit_field.Task4,
                                                                                  t_bit_field.Task5,
                                                                                  t_bit_field.Task6);
   Serial.print (buf);
   
}
//=====================================================================

//============loop=====================================================
void loop()
{
  for (unsigned i = 0u; i < ntasks; i++) {
    if(tasks[i].f != nullptr && tasks[i].state == READY) {
      tasks[i].state = RUN;
      tasks[i].f(tasks[i].param);
      tasks[i].state = WAIT;
      if (!tasks[i].periodic)
        delete_task(i);
    }
  }
}

//=====================================================================
