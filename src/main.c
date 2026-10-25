/*
   Embedded Systems Final Project
   Team 4
*/
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#define F_CPU   16000000UL
#include <util/delay.h>

//=========================
// State definitions
//=========================
typedef enum {
   PROX_IDLE = 0,
   PROX_NEAR
} ProxState_t;

typedef enum {
   MOVE_IDLE = 0,
   MOVE_N,
   MOVE_M,
   MOVE_CRIT
} MoveState_t;

typedef enum {
   MED_IDLE = 0,
   MED_DUE,
   MED_DONE,
   MED_EARLY,
   MED_OVERDOSE
} MedState_t;

//=========================
// Flags and events
//=========================
typedef enum {
   FLAG_FSM_UPDATE = (1u << 0),
   FLAG_PROX_NEAR  = (1u << 1)
} SysFlags;

typedef uint8_t SysFlagMask;

typedef enum {
   EV_MED_SWITCH = (1u << 0)
} SysEvents;

typedef uint8_t SysEventMask;

//=========================
// Character LCD facial-expression states
//=========================
typedef enum {
   FACE_IDLE = 0,    // Default expression: a slight smile
   FACE_HAPPY,       // Happy: proximity interaction or medication confirmation
   FACE_TEARFUL,     // Tearful: the user has moved away

   FACE_BLANK,       // Blank: first inactivity threshold reached
   FACE_SOFT_SAD,    // Slightly sad: second inactivity threshold reached
   FACE_SAD,         // Sad: critical inactivity threshold reached

   FACE_BIGEYE,      // Wide eyes: medication reminder is due
   FACE_SURPRISE,    // Surprised: early or repeated medication confirmation
} Face_t;

typedef enum {
   PRIO_IDLE         = 0,   // Higher values indicate higher display priority
   PRIO_PROX         = 1,
   PRIO_MOVE         = 3,
   PRIO_MED          = 2,
   PRIO_OVERLAY_HIGH = 6,
   PRIO_OVERLAY_MID  = 5,
   PRIO_OVERLAY_LOW  = 4
} Prio_t;

typedef struct {
   Face_t    face;
   uint16_t  hold_ms;   // Expression hold duration, counted by display updates
   Prio_t    prio;      // Display priority
} FaceState_t;

typedef enum {
   STATE_UP_DISP_0,
   STATE_UP_DISP_1,
   STATE_UP_DISP_2,
   STATE_UP_DISP_3,
   STATE_UP_DISP_4,
   STATE_UP_DISP_5,
   STATE_UP_DISP_6,
   STATE_UP_DISP_7,
   } UpDispState_t;

#define HOLD_KEEP 0x0000   // A hold_ms value of HOLD_KEEP disables automatic expiration

//=========================
// System time in 24-hour format
//=========================
typedef struct {
   uint8_t hour;
   uint8_t min;
   uint8_t sec;
} Time_t;

#define INIT_TIME_HOUR  10
#define INIT_TIME_MIN   30
#define INIT_TIME_SEC   00

//=========================
// Piezo buzzer commands
//=========================
typedef enum {
   BUZ_MED = 0,
   BUZ_STOP
} Buz_t;

//=========================
// Hardware macros and global variables
//=========================
#define BUZZER 3 // Buzzer signal (output)

// Medication confirmation button
#define MED 4 // Medication confirmation button (input)
volatile uint16_t med_btn_debounce_cnt = 0;

// Repeated medication confirmation counter
uint8_t cnt_med_overdose = 0;   // Number of additional confirmations

// Ultrasonic sensor
#define SOUND_VELOCITY 340UL
#define TRIG 6 // HC-SR04 trigger signal (output)
#define ECHO 7 // HC-SR04 echo signal (input)

// LCD initialization counter
volatile unsigned int CLCD_cnt = 0;

// Servo parameters for tail control
#define SERVO_MIN   225   // 900 us pulse at 4 us per timer tick
#define SERVO_MID   375   // 1500 us pulse at 4 us per timer tick
#define SERVO_MAX   525   // 2100 us pulse at 4 us per timer tick

#define D_L_90      SERVO_MIN
#define D_0         SERVO_MID
#define D_R_90      SERVO_MAX

#define TAIL_STEP       1   // Pulse-width count increment per position update
#define TAIL_MIN_DUTY   D_L_90
#define TAIL_MAX_DUTY   D_R_90 

//=========================
// Global state variables
//=========================
ProxState_t      prox_next_state = PROX_IDLE;
ProxState_t      prox_curr_state = PROX_IDLE;
ProxState_t      prox_prev_state = PROX_IDLE;

MoveState_t      move_next_state = MOVE_IDLE;
MoveState_t      move_curr_state = MOVE_IDLE;
MoveState_t      move_prev_state = MOVE_IDLE;

MedState_t       med_next_state  = MED_IDLE;
MedState_t       med_curr_state  = MED_IDLE;
MedState_t       med_prev_state  = MED_IDLE;

volatile SysFlagMask  flags;
volatile SysEventMask events;

FaceState_t      curr_face_state = { FACE_IDLE, HOLD_KEEP, PRIO_IDLE };

UpDispState_t   up_disp_state = STATE_UP_DISP_0;

volatile Time_t  curr_time;
volatile Time_t  cnt_inactive;

Time_t           med_time;


// ----- LCD variables -----
//LCD Instructions
#define BIT4_LINE2_DOT58      0x28   //4 bit interface, 2 lines, 5*8 dot

#define DISPON_CUROFF_BLCKOFF   0x0C   //Display ON, Cursor OFF, Blink OFF
#define DISOFF_CUROFF_BLCKOFF   0x08   //Display OFF, Cursor OFF, Blink OFF
#define INC_NOSHIFT            0x06   //Entry Mode : Cursor Increment, Display No Shift

#define DISPCLEAR            0x01   //Clear Display : Cursor position = 0

#define CUR1LINE            0x80   //Cursor Position Line 1 First
#define CUR2LINE            0xC0   //Cursor Position Line 2 First
#define CURHOME               0x02   //Return Home

#define CGRAM_ADSET            0x40   // CGRAM address command; the lower six bits specify the address
#define DRAM_ADSET            0x80   // DDRAM address command

//LCD Initial Condition
#define E_BIT             4   //Enable Bit #
#define RW_BIT               3   //Read Write Bit #
#define RS_BIT               2   //Register Select Bit #

// Glyph patterns stored in CGRAM
// Character LCD facial expressions
// Patterns are listed starting at the upper left
uint8_t emo_IDLE[7][8] = {
   {0x00,0x01,0x02,0x04,0x08,0x10,0x00,0x00},      // Left eye, slash pattern; glyph 0
   {0x00,0x10,0x08,0x04,0x02,0x01,0x00,0x00},      // Right eye, backslash pattern; glyph 1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},         // Whiskers; glyph 2
   {0x00,0x00,0x00,0x00,0x00,0x01,0x12,0x0C},      // Left end of mouth; glyph 3
   {0x01,0x02,0x04,0x08,0x10,0x00,0x00,0x00},      // Left middle of mouth; glyph 4
   {0x10,0x08,0x04,0x02,0x01,0x00,0x00,0x00},      // Right middle of mouth; glyph 5
   {0x00,0x00,0x00,0x00,0x00,0x10,0x09,0x06}      // Right end of mouth; glyph 6
}; // Default expression

uint8_t emo_HAPPY[5][8] = {
   {0x00,0x00,0xc,0x6,0x3,0x6,0xc,0x00},      //">"                  //0
   {0x00,0x00,0x6,0xc,0x18,0xc,0x6,0x00},      //"<"                  //1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // Whiskers; glyph 2
   {0x00,0x0,0x0,0x1f,0xf,0x7,0x3,0x1},      // Left side of mouth; glyph 3
   {0x00,0x0,0x0,0x1f,0x1e,0x1c,0x18,0x10}      // Right side of mouth; glyph 4
};   // Happy expression

uint8_t emo_TEARFUL[4][8] = {
   {0x00,0x00,0x00,0x01,0x02,0x04,0x18,0x00},   // Left eyebrow; glyph 0
   {0x00,0x00,0x00,0x10,0x08,0x04,0x03,0x00},   // Right eyebrow; glyph 1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // Whiskers; glyph 2
   {0x00,0x1F,0x1F,0x0E,0x0E,0x0E,0x0E,0x00}   // Eyes; glyph 3
};   // Expression shown after the user moves away

uint8_t emo_BLANK[3][8] = {
   {0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00},   // Eyebrows; glyph 0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // Whiskers; glyph 1
   {0x0,0x0,0x1f,0x0A,0x0A,0x0E,0x0,0x00}      // Eyes; glyph 2
};   // Blank expression: first inactivity threshold reached


uint8_t emo_SOFT_SAD[6][8] = {
   {0x00,0x0,0x1,0x2,0x4,0x8,0x10,0x0},      // Left eyebrow; glyph 0
   {0x00,0x0,0x10,0x8,0x4,0x2,0x1,0x0},      // Right eyebrow; glyph 1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // Whiskers; glyph 2
   {0x0,0xe,0xa,0xa,0xa,0xe,0x0,0x00},         // Surprised eyes; glyph 3
   {0x00,0x0,0xf,0x8,0x8,0x8,0xf,0x0},         // Left side of mouth; glyph 4
   {0x00,0x0,0x1e,0x2,0x2,0x2,0x1e,0x0}      // Right side of mouth; glyph 5
};   // Slightly sad expression: second inactivity threshold reached

uint8_t emo_SAD[6][8] = {
   {0x00,0x1F,0x0A,0x0A,0x0A,0x0A,0x0A,0x00},   // Crying-eye pattern; glyph 0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // Whiskers; glyph 1
   {0x4,0x0,0x4,0x0,0x4,0x0,0x00,0x0},         // Tears; glyph 2
   {0x1f,0x10,0x10,0x10,0x10,0x10,0x10,0x1f},   // Left side of mouth; glyph 3
   {0x1f,0x0,0x0,0x0,0x0,0x0,0x00,0x1f},      // Center of mouth; glyph 4
   {0x1f,0x1,0x1,0x1,0x1,0x1,0x1,0x1f}         // Right side of mouth; glyph 5
};   // Sad expression: critical inactivity threshold reached

uint8_t emo_BIGEYE[4][8] = {
   {0x4,0xe,0x1f,0x1f,0x1f,0xe,0x4,0x00},      // Eyes; glyph 0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // Whiskers; glyph 1
   {0x00,0x01,0x02,0x04,0x04,0x02,0x01,0x00},   // Left side of mouth; glyph 2
   {0x00,0x10,0x08,0x04,0x04,0x08,0x10,0x00}   // Right side of mouth; glyph 3
};      // Medication reminder expression

uint8_t emo_SURPRISE[5][8] = {
   {0x00,0x0,0xe,0x11,0x15,0x11,0xe,0x0},         // Eyes; glyph 0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},         // Whiskers; glyph 1
   {0x1f,0x10,0x10,0x10,0x10,0x10,0x10,0x1f},      // Left side of mouth; glyph 2
   {0x1f,0x0,0x0,0x0,0x0,0x0,0x00,0x1f},         // Center of mouth; glyph 3
   {0x1f,0x1,0x1,0x1,0x1,0x1,0x1,0x1f}            // Right side of mouth; glyph 4
};      // Surprised expression

// ----- Facial-expression display data -----
// ----- Glyph codes written to DDRAM -----
uint8_t display_IDLE[10] = {0, 1, 0, 1, 2, 3, 4, 5, 6, 2};               // Default expression
uint8_t display_HAPPY[6] = {0, 1, 2, 3, 4, 2};                        // Happy expression
uint8_t display_TEARFUL[6] = {0, 1, 2, 3, 3, 2};                     // User has moved away
uint8_t display_BLANK[6] = {0, 0, 1, 2, 2, 1};                        // Blank expression
uint8_t display_SOFT_SAD[8] = {0, 1, 2, 3, 4, 5, 3, 2};                  // Slightly sad expression
uint8_t display_SAD[10] = {0, 0, 1, 2, 3, 4, 4, 5, 2, 1};               // Sad expression
uint8_t display_BIGEYE[6] = {0, 0, 1, 2, 3, 1};                        // Medication reminder expression
uint8_t display_SURPRISE[8] = {0, 0, 1, 2, 3, 3, 4, 1};                  // Surprised expression

// ----- Facial-feature positions -----
uint8_t emo_IDLE_pos[10] = {0x04, 0x05, 0x0A, 0x0B, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4C};         // Default expression
uint8_t emo_HAPPY_pos[6] = {0x05, 0x0A, 0x43, 0x47, 0x48, 0x4C};                           // Happy expression
uint8_t emo_TEARFUL_pos[6] = {0x06, 0x09, 0x43, 0x46, 0x49, 0x4C};                           // User has moved away
uint8_t emo_BLANK_pos[6] = {0x05, 0x0A, 0x43, 0x45, 0x4A, 0x4C};                           // Blank expression
uint8_t emo_SOFT_SAD_pos[8] = {0x05, 0x0A, 0x43, 0x45, 0x47, 0x48, 0x4A, 0x4C};                  // Slightly sad expression
uint8_t emo_SAD_pos[10] = {0x05, 0x0A, 0x43, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4C};            // Sad expression
uint8_t emo_BIGEYE_pos[6] = {0x05, 0x0A, 0x43, 0x47, 0x48, 0x4C};                           // Medication reminder expression
uint8_t emo_SURPRISE_pos[8] = {0x05, 0x0A, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4C};                  // Surprised expression

// Tail servo control
#define TAIL_MOVE 0
#define TAIL_STOP 1

volatile uint16_t tail_curr_duty = D_0;
volatile uint8_t  tail_direction = 1;  // 1: move right; 0: move left

volatile uint16_t slow_div = 0;   // Divider used to pace servo position updates
//=========================
// Function Prototypes
//=========================

// System initialization
void init(void);

// Button debounce countdown
void med_debounce_timer(void);

// LCD expression, display, tail, and timer functions
void CLCD_data(char data);
void CLCD_cmd(char cmd);
void request_face(Face_t req_face, uint16_t req_hold_ms, Prio_t req_prio);
void clear_face_area(void);
void update_display(void);
void ctrl_tail(int cmd);


// UART output functions
void uart_puts_without_newline(const char *msg);
void uart_puts(const char *msg);
void uart_systime(void);

// Piezo buzzer control
void ctrl_buzzer(Buz_t cmd);

// Ultrasonic sensor FSM
void ultrasonic(void);

// Helper functions
void uchar_to_str(uint8_t num, char *ptr);
void zero_extend_to_2digit(char *ptr);

// Proximity FSM state handlers
void Prox_on_entry(ProxState_t curr_state);
void Prox_on_during(ProxState_t curr_state);
void Prox_on_exit(ProxState_t prev_state);

// Inactivity FSM state handlers
void Move_on_entry(MoveState_t curr_state);
void Move_on_during(MoveState_t curr_state);
void Move_on_exit(MoveState_t prev_state);

// Medication FSM state handlers
void Med_on_entry(MedState_t curr_state);
void Med_on_during(MedState_t curr_state);
void Med_on_exit(MedState_t prev_state);

// FSM next-state decision functions
ProxState_t Prox_decide_next(ProxState_t curr_state);
MoveState_t Move_decide_next(MoveState_t curr_state);
MedState_t Med_decide_next(MedState_t curr_state);

// FSM step functions
void fsm_prox_step(void);
void fsm_move_step(void);
void fsm_med_step(void);

// main
int main(void);

//=========================
// Initialization
//=========================
void init(void) {
   // Initialize the system time
   curr_time.hour = INIT_TIME_HOUR;
   curr_time.min  = INIT_TIME_MIN;
   curr_time.sec  = INIT_TIME_SEC;

   // Initialize the inactivity counter
   cnt_inactive.sec  = 0;
   cnt_inactive.min  = 0;
   cnt_inactive.hour = 0;

   // Set the scheduled medication time
   med_time.hour = 10;
   med_time.min  = 30;
   med_time.sec  = 3;

   // Timer0: system timebase, CTC mode, prescaler 64, 1 kHz compare interrupt
   TCCR0 = (1 << WGM01) | (1 << CS02);
   OCR0  = 249;
   TIMSK = (1 << OCIE0);

   // Initialize UART
   UCSR0B = 0x08; // TX Enable
   UCSR0C = 0x06; // 8N1
   UBRR0H = 0;
   UBRR0L = 103;    // UART baud-rate divisor; not a 1 Mbaud configuration

   // Initialize the buzzer
   DDRE |= (1 << BUZZER);
   TCCR3A = 0;
   TCCR3B = 0;
   TCCR3C = 0;

   // Initialize the medication confirmation button
   DDRE  &= ~(1 << MED); // Input
   EICRB |= 0x02;        // INT4 falling edge
   EIMSK |= 0x10;        // INT4 enable

   // Initialize the ultrasonic sensor
   DDRE = ((DDRE | (1 << TRIG)) & ~(1 << ECHO)); // TRIG is an output; ECHO is an input
   PORTE &= ~(1 << TRIG);

   // Configure the LCD ports
   DDRC = 0xff;         // PORTC: LCD command/data output
   DDRD = 0xff;         // PORTD: LCD control output
   
   // Initialize the LCD
   _delay_ms(50);
      
   CLCD_cmd(BIT4_LINE2_DOT58);            // Configure the LCD interface
   _delay_us(50);
   CLCD_cmd(DISPON_CUROFF_BLCKOFF);      //display ON
   _delay_us(50);
   CLCD_cmd(INC_NOSHIFT);               // Configure cursor movement
   _delay_us(50);
   
   // Send DISPCLEAR directly, then allow for command execution time
   PORTD = 0x00;            

   PORTD |= (1<<E_BIT);              
   PORTC = DISPCLEAR & 0xf0;          
   PORTD &= ~(1<<E_BIT);              
   PORTD |= (1<<E_BIT);              
   PORTC = (DISPCLEAR << 4) & 0xf0;      
   PORTD &= ~(1<<E_BIT);               
   _delay_ms(3);            // Execution delay(Clear Display)
   // End of the clear-display sequence and execution delay
   
   CLCD_cmd(CUR1LINE);                  // Move the cursor to the start of the first line
   
   // Initialize Timer1 for servo PWM
   DDRB |=  (1 << PB5);      // Servo control signal on PB5

   TCCR1A = 0x82;            // Fast   PWM OC1A, 16bit TCNT1, Set at bottom, Clear on compare Match
   TCCR1B = 0x1b;         // Fast PWM, prescaler 64
     
   ICR1 = 4999;   // Generate 50 Hz PWM on OC1A (PB5)
   OCR1A = D_0;   // Initial servo position
   
   // Initialize Timer2 for tail position updates
   // Timer2 is configured below in normal mode, not CTC mode
   // Compare interrupt timing depends on the timer configuration and OCR2 value
   TCCR2 = (1<<CS21) | (1<<CS20);  // Normal mode, prescaler 64; no explicit OCR2 assignment is made here
   
   // Keep the tail timer compare interrupt disabled initially
   TIMSK &= ~(1<<OCIE2);
   
   ctrl_tail(TAIL_STOP);
         
   sei();

}

//=========================
// Helper functions
//=========================
void uchar_to_str(uint8_t num, char *ptr) {   // ptr must point to a buffer of at least four bytes
   if (num / 100) {
      ptr[0] = (num / 100) % 10 + '0';
      ptr[1] = (num / 10)  % 10 + '0';
      ptr[2] =  num        % 10 + '0';
      ptr[3] =  0x00;
   }
   else if (num / 10) {
      ptr[0] = (num / 10)  % 10 + '0';
      ptr[1] = num         % 10 + '0';
      ptr[2] = 0x00;
   }
   else {
      ptr[0] = num         % 10 + '0';
      ptr[1] = 0x00;
   }
}

void zero_extend_to_2digit(char *ptr) {   // Pad a one-digit string with a leading zero; requires a three-byte buffer
   if(ptr[1] == 0x00) {            // Leave strings with two or more digits unchanged
      ptr[2] = ptr[1];
      ptr[1] = ptr[0];
      ptr[0] = '0';
   }
}

//=========================
// Interrupt service routines
//=========================
ISR(TIMER0_COMP_vect) {
   flags |= FLAG_FSM_UPDATE;

   med_debounce_timer(); // Decrement the medication-button debounce counter

   // Maintain a 24-hour clock
   static uint16_t cnt_1000ms = 0;
   cnt_1000ms += 1;

   if (cnt_1000ms >= 1000) {
      cnt_1000ms = 0;

      // System time
      curr_time.sec++;
      if (curr_time.sec >= 60) {
         curr_time.sec = 0;
         curr_time.min++;
         if (curr_time.min >= 60) {
            curr_time.min = 0;
            curr_time.hour++;
            if (curr_time.hour >= 24) {
               curr_time.hour = 0;
            }
         }
      }

      // Inactivity duration counter
      cnt_inactive.sec++;
      if (cnt_inactive.sec >= 60) {
         cnt_inactive.sec = 0;
         cnt_inactive.min++;
         if (cnt_inactive.min >= 60) {
            cnt_inactive.min = 0;
            cnt_inactive.hour++;
         }
      }
   }
}

ISR(TIMER2_COMP_vect)   // Servo tail position update
{   
   slow_div++;
   if(slow_div > 50)   // Begin position updates once slow_div exceeds 50
   {
     // Sweep back and forth
     if (tail_direction) {
        // Increase the position count toward the right endpoint
        if (tail_curr_duty + TAIL_STEP < TAIL_MAX_DUTY) {
           tail_curr_duty += TAIL_STEP;
          } else {
           tail_curr_duty = TAIL_MAX_DUTY;
          tail_direction = 0;   // Reverse direction at the endpoint and reset the divider
         slow_div = 0;
      }
      } else {
        // Decrease the position count toward the left endpoint
       if (tail_curr_duty > TAIL_MIN_DUTY + TAIL_STEP) {
         tail_curr_duty -= TAIL_STEP;
         } else {
          tail_curr_duty = TAIL_MIN_DUTY;
         tail_direction = 1;   // Reverse direction at the endpoint and reset the divider
         slow_div = 0;
      }
     }

   // Apply the updated servo pulse-width count
   OCR1A = tail_curr_duty;
   }
}

// Medication confirmation button ISR
ISR(INT4_vect) {
   if (med_btn_debounce_cnt == 0) {
      events |= EV_MED_SWITCH;
      med_btn_debounce_cnt = 1000; // Ignore further button interrupts for 1000 system ticks (nominally 1 s)
   }
}

//=========================
// Button debounce countdown
//=========================
void med_debounce_timer(void) {
   if (med_btn_debounce_cnt > 0) {
      med_btn_debounce_cnt--;
   }
}

//=========================
// LCD facial-expression functions
//=========================
void request_face(Face_t req_face, uint16_t req_hold_ms, Prio_t req_prio) {   
   if(curr_face_state.prio <= req_prio)            // Accept requests with priority greater than or equal to the current priority
   {
      curr_face_state.face = req_face;            // Apply the requested expression, hold duration, and priority
      curr_face_state.hold_ms = req_hold_ms;
      curr_face_state.prio = req_prio;
   }
}

void update_display(void) {
   //=========================
   // 1. Update the expression hold countdown
   //=========================
   if (curr_face_state.hold_ms != HOLD_KEEP) {
      if (curr_face_state.hold_ms > 0) {
         if (--curr_face_state.hold_ms == 0) {
            curr_face_state.face    = FACE_IDLE;
            curr_face_state.hold_ms = HOLD_KEEP;
            curr_face_state.prio    = PRIO_IDLE;
         }
      }
   }

   //=========================   
   // 2. LCD display FSM
   // At most one CLCD_cmd or CLCD_data call per display update
   //=========================
   typedef enum {
      DISP_PHASE_IDLE = 0,
      DISP_PHASE_CLEAR,
      DISP_PHASE_CGRAM,
      DISP_PHASE_DDRAM
   } DispPhase_t;

   static DispPhase_t phase = DISP_PHASE_IDLE;

   // Positions to clear in the expression area
   static const uint8_t clear_positions[] = {
      0x04, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
      0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C
   };

   static uint8_t clear_idx  = 0;
   static uint8_t clear_sub  = 0;  // 0: set DDRAM address; 1: write a space

   static uint8_t cgram_glyph = 0; // Current glyph index
   static uint8_t cgram_row   = 0; // Current row within a glyph, from 0 through 7
   static uint8_t cgram_sub   = 0; // 0: CGRAM_ADSET, 1: data write

   static uint8_t ddram_idx = 0;   // Index into the display_* and emo_*_pos arrays
   static uint8_t ddram_sub = 0;   // 0: set DDRAM address; 1: write glyph data

   static Face_t drawing_face = FACE_IDLE;
   static Face_t shown_face = FACE_BLANK;
   
   static uint8_t need_clear = 0;   // Request another clearing pass over the expression area

   //=========================
   // 2-1. Start a new expression only when the display FSM is idle
   //=========================
   if (phase == DISP_PHASE_IDLE) {
      if (need_clear || (curr_face_state.face != shown_face)) {
       need_clear     = 0; // Consume the pending clear request
         // Start a new expression by clearing the previous expression area
         drawing_face = curr_face_state.face;
         clear_idx    = 0;
         clear_sub    = 0;
         cgram_glyph  = 0;
         cgram_row    = 0;
         cgram_sub    = 0;
         ddram_idx    = 0;
         ddram_sub    = 0;
         phase        = DISP_PHASE_CLEAR;
         }
      else {
         // No work is pending; skip LCD access for this update
         return;
      }
   }

   //=========================
   // 2-2. Process one display step per update
   //=========================
   switch (phase) {
      //-----------------------------
      // (1) Clear the expression area
      //-----------------------------
      case DISP_PHASE_CLEAR:
      if (clear_idx < ((uint8_t)( sizeof(clear_positions) / sizeof(clear_positions[0]) )) ) {
         if (clear_sub == 0) {
            // Set DDRAM address
            CLCD_cmd(DRAM_ADSET + clear_positions[clear_idx]);
            clear_sub = 1;
            } 
      else {
            // Write a space
            CLCD_data(' ');
            clear_sub = 0;
            clear_idx++;
      }
      } 
     else {
         // Clearing is complete; proceed to CGRAM pattern loading
         cgram_glyph = 0;
         cgram_row   = 0;
         cgram_sub   = 0;
         phase       = DISP_PHASE_CGRAM;
      }
      break;

      //-----------------------------
      // (2) Write glyph patterns to CGRAM
      //-----------------------------
      case DISP_PHASE_CGRAM:
     
     if(drawing_face != curr_face_state.face) {   // Restart drawing if the requested expression changes during rendering
        phase = DISP_PHASE_IDLE;
        need_clear = 1;
        break;
     }
     
      switch (drawing_face) {
         //================= FACE_IDLE =================
         case FACE_IDLE:
         if (cgram_glyph >= 7) {
            // All seven glyphs are loaded; proceed to DDRAM output
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            // Set the start address of this glyph
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            // Write one glyph row at a time
            CLCD_data(emo_IDLE[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_HAPPY =================
         case FACE_HAPPY:
         if (cgram_glyph >= 5) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_HAPPY[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_TEARFUL =================
         case FACE_TEARFUL:
         if (cgram_glyph >= 4) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_TEARFUL[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= 
       //=================
         case FACE_BLANK:
         if (cgram_glyph >= 3) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_BLANK[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_SOFT_SAD =================
         case FACE_SOFT_SAD:
         if (cgram_glyph >= 6) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_SOFT_SAD[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_SAD =================
         case FACE_SAD:
         if (cgram_glyph >= 6) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_SAD[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_BIGEYE =================
         case FACE_BIGEYE:
         if (cgram_glyph >= 4) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_BIGEYE[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         //================= FACE_SURPRISE =================
         case FACE_SURPRISE:
         if (cgram_glyph >= 5) {
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            CLCD_data(emo_SURPRISE[cgram_glyph][cgram_row]);
            cgram_row++;
            if (cgram_row >= 8) {
               cgram_row = 0;
               cgram_glyph++;
               cgram_sub = 0;
            }
         }
         break;

         default:
         // Return to idle for an unexpected expression value
         phase = DISP_PHASE_IDLE;
         break;
      }
      break;

      //-----------------------------
      // (3) Write facial-feature glyphs to DDRAM
      //-----------------------------
      case DISP_PHASE_DDRAM:
     if(drawing_face != curr_face_state.face) {   // Restart drawing if the requested expression changes during rendering
        phase = DISP_PHASE_IDLE;
        need_clear = 1;
        break;
     }
      switch (drawing_face) {
         //============= FACE_IDLE =============
         case FACE_IDLE:
         if (ddram_idx >= 10) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_IDLE_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_IDLE[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_HAPPY =============
         case FACE_HAPPY:
         if (ddram_idx >= 6) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_HAPPY_pos[ddram_idx]);
            ddram_sub = 1;
            } 
      else {
            CLCD_data(display_HAPPY[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_TEARFUL =============
         case FACE_TEARFUL:
         if (ddram_idx >= 6) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_TEARFUL_pos[ddram_idx]);
            ddram_sub = 1;
            } 
      else {
            CLCD_data(display_TEARFUL[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_BLANK =============
         case FACE_BLANK:
         if (ddram_idx >= 6) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_BLANK_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_BLANK[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_SOFT_SAD =============
         case FACE_SOFT_SAD:
         if (ddram_idx >= 8) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_SOFT_SAD_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_SOFT_SAD[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_SAD =============
         case FACE_SAD:
         if (ddram_idx >= 10) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_SAD_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_SAD[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_BIGEYE =============
         case FACE_BIGEYE:
         if (ddram_idx >= 6) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_BIGEYE_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_BIGEYE[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         //============= FACE_SURPRISE =============
         case FACE_SURPRISE:
         if (ddram_idx >= 8) {
            phase = DISP_PHASE_IDLE;
            shown_face = drawing_face;
            break;
         }
         if (ddram_sub == 0) {
            CLCD_cmd(DRAM_ADSET + emo_SURPRISE_pos[ddram_idx]);
            ddram_sub = 1;
            } else {
            CLCD_data(display_SURPRISE[ddram_idx]);
            ddram_sub = 0;
            ddram_idx++;
         }
         break;

         default:
         phase = DISP_PHASE_IDLE;
         break;
      }
      break;

      //-----------------------------
      // Fallback for an unexpected display phase
      //-----------------------------
      default:
      phase = DISP_PHASE_IDLE;
      break;
   }
}


void CLCD_data(char data)   // Write one data byte to the LCD
{
   PORTD = (1<<RS_BIT);               // E = 0, R/W = 0, RS = 1: data-write mode
   // At 16 MHz, one CPU clock period is 62.5 ns; no explicit delay is inserted here
   PORTD |= (1<<E_BIT);            //0001_0100, E(bit 4) = 1, R/W(bit 3) = 0, RS(bit 2) = 1 // Enable
   PORTC = data & 0xf0;            // Write the upper nibble
   /*
   The original implementation relies on instruction execution time for LCD setup timing.
   Actual timing depends on generated instructions and has not been verified here.
   */
   PORTD &= ~(1<<E_BIT);            // E = 0, R/W = 0, RS = 1: finish this nibble transfer
   // No explicit hold-time delay is inserted here
   
   PORTD |= (1<<E_BIT);            //0001_0100, E(bit4) = 1, R/W(bit 3) = 0, RS(bit 2) = 1 // Enable
   PORTC = (data << 4) & 0xf0;   // Write the lower nibble
   PORTD &= ~(1<<E_BIT);          // E = 0, R/W = 0, RS = 1: end the transfer
}

void CLCD_cmd(char cmd)         // Write a command byte to the LCD
{
   PORTD = 0x00;            // E = 0, R/W = 0, RS = 0: command-write mode
   

   PORTD |= (1<<E_BIT);               //0001_0000, E(bit 4) = 1, R/W(bit 3) = 0, RS(bit 2) = 0 // Enable
   PORTC = cmd & 0xf0;            // Write the upper nibble
   PORTD &= ~(1<<E_BIT);               // E = 0, R/W = 0, RS = 0: finish this nibble transfer
 
   PORTD |= (1<<E_BIT);               //0001_0000, E(bit4) = 1, R/W(bit 3) = 0, RS(bit 2) = 0 // Enable
   
   PORTC = (cmd << 4) & 0xf0;      // Write the lower nibble
   PORTD &= ~(1<<E_BIT);               // E = 0, R/W = 0, RS = 0: end the transfer
}
//=========================
// Tail servo control
//=========================
void ctrl_tail(int cmd)
{
   switch (cmd) {
      case TAIL_MOVE:
     TCCR1B |= ( (1<<CS10) | (1<<CS11) );
      // Start sweeping from the retained position
      //tail_curr_duty = D_0;
      tail_direction = 1;    // Move toward the right endpoint first

      OCR1A = tail_curr_duty;

      // Timer2 Compare Match interrupt ON
      TCNT2 = 0;             // Reset the timer counter
      TIMSK |= (1<<OCIE2);
      break;

      case TAIL_STOP:
      default:
      TCCR1B &= ~( (1<<CS10) | (1<<CS11) );
      // Stop the servo timer and disable tail position updates
      TIMSK &= ~(1<<OCIE2);  // Timer2 Compare Match interrupt OFF
      //tail_curr_duty = D_0;
      OCR1A = tail_curr_duty;
      break;
   }
}

//=========================
// UART functions
//=========================
void uart_puts_without_newline(const char *msg) {
   while (*msg) { // Continue until the terminating null character
      while(!(UCSR0A & (1<<UDRE0))); // Wait for UDRE0, the USART0 data-register-empty flag (bit 5)
      UDR0 = *msg++; // Transmit one character
   }
}

void uart_systime(void) {
   char buf[5];
   uart_puts_without_newline("[");
   uchar_to_str(curr_time.hour, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   
   uart_puts_without_newline(":");
   
   uchar_to_str(curr_time.min, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   
   uart_puts_without_newline(":");
   
   uchar_to_str(curr_time.sec, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   uart_puts_without_newline("] ");
}

void uart_puts(const char *msg) {
   char buf[5];

   uart_puts_without_newline("[");
   uchar_to_str(curr_time.hour, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   uart_puts_without_newline(":");
   uchar_to_str(curr_time.min, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   uart_puts_without_newline(":");
   uchar_to_str(curr_time.sec, buf);
   zero_extend_to_2digit(buf);
   uart_puts_without_newline(buf);
   uart_puts_without_newline("] ");

   uart_puts_without_newline(msg);
   uart_puts_without_newline("\r\n");
}

//=========================
// Piezo buzzer control
//=========================
void ctrl_buzzer(Buz_t cmd) {
   if (cmd == BUZ_MED) {
      OCR3A = 61;             // 2016Hz
      TCCR3A = (1 << COM3A0); // toggle
      TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30); // CTC mode, prescaler 64
   }
   else {
      TCCR3A = 0;
      TCCR3B = 0;
      PORTE &= ~(1 << BUZZER);
   }
}

//=========================
// Ultrasonic sensor state machine
//=========================
void ultrasonic(void) {
   static uint8_t  ultra_state = 0;
   static uint16_t echo_cnt    = 0;
   static uint16_t timeout     = 0;

   switch (ultra_state) {
      case 0:
         echo_cnt = 0;
         timeout  = 0;
         PORTE |= (1 << TRIG);
         ultra_state = 1;
         break;

      case 1:
         PORTE &= ~(1 << TRIG);
         if (PINE & (1 << ECHO)) {
            ultra_state = 2;
         }
         else {
            if (++timeout > 30) ultra_state = 0;
         }
         break;

      case 2:
         if (PINE & (1 << ECHO)) {
            echo_cnt++;
            if (echo_cnt > 30) {
               flags &= ~FLAG_PROX_NEAR;
               ultra_state = 0;
            }
         }
         else {
            ultra_state = 3;
         }
         break;

      case 3:
         if (echo_cnt < 6) flags |= FLAG_PROX_NEAR;
         else              flags &= ~FLAG_PROX_NEAR;
         ultra_state = 0;
         break;
   }
}

//=========================
// FSM state handlers
//=========================
// 1. Proximity
void Prox_on_entry(ProxState_t curr_state) {
   switch (curr_state) {
      case PROX_IDLE:
         request_face(FACE_TEARFUL, 3000, PRIO_PROX);
         ctrl_tail(TAIL_STOP);
         break;

      case PROX_NEAR:
         request_face(FACE_HAPPY, HOLD_KEEP, PRIO_PROX);
         uart_puts("Movement has been detected.");
         ctrl_tail(TAIL_MOVE);
         break;
   }
}

void Prox_on_during(ProxState_t curr_state) {
   switch (curr_state) {
      case PROX_IDLE:
         request_face(FACE_IDLE, HOLD_KEEP, PRIO_IDLE);
         break;

      case PROX_NEAR:
         cnt_inactive.hour = 0;
         cnt_inactive.min  = 0;
         cnt_inactive.sec  = 0;
         request_face(FACE_HAPPY, HOLD_KEEP, PRIO_PROX);
         break;
   }
}

void Prox_on_exit(ProxState_t prev_state) {
   (void)prev_state;
}

// 2. Movement
void Move_on_entry(MoveState_t curr_state) {
   switch (curr_state) {
      case MOVE_IDLE:
         break;
      case MOVE_N:
         uart_puts("Duration without detected movement: 3 hours");
         break;
      case MOVE_M:
         uart_puts("Duration without detected movement: 6 hours");
         break;
      case MOVE_CRIT:
         uart_puts("Duration without detected movement: 9 hours");
         uart_puts("[ALERT] Notifying the nearest emergency services.");
         break;
   }
}

void Move_on_during(MoveState_t curr_state) {
   switch (curr_state) {
      case MOVE_IDLE:
         break;
      case MOVE_N:
         request_face(FACE_BLANK, HOLD_KEEP, PRIO_MOVE);
         break;
      case MOVE_M:
         request_face(FACE_SOFT_SAD, HOLD_KEEP, PRIO_MOVE);
         break;
      case MOVE_CRIT:
         request_face(FACE_SAD, HOLD_KEEP, PRIO_MOVE);
         break;
   }
}

void Move_on_exit(MoveState_t prev_state) {
   request_face(FACE_HAPPY, 1000, PRIO_MOVE);
}

// 3. Medicine
void Med_on_entry(MedState_t curr_state) {
   switch (curr_state) {
      case MED_IDLE:
       cnt_med_overdose = 0;   // Reset the additional-confirmation counter on entry to MED_IDLE
         break;

      case MED_DUE:
         uart_puts("Time for medication.");
         break;

      case MED_DONE:
         ctrl_buzzer(BUZ_STOP);
         request_face(FACE_HAPPY, 3000, PRIO_OVERLAY_MID);
         break;

      case MED_EARLY: {
         ctrl_buzzer(BUZ_STOP);
         request_face(FACE_SURPRISE, 3000, PRIO_OVERLAY_HIGH);

         uint8_t delta = med_time.hour - curr_time.hour;
         if ((med_time.hour - curr_time.hour > 0)
          && (med_time.min - curr_time.min > 0)
          &&   (med_time.sec - curr_time.sec > 0) ) {
         char msg_med_early[4];
         uchar_to_str(delta, msg_med_early);
         uart_systime();
         uart_puts_without_newline("[ALERT] Medication taken about ");
         uart_puts_without_newline(msg_med_early);
         uart_puts_without_newline(" hours earlier than scheduled.\r\n");
         }
       else {
            uart_puts("[ALERT] Medication taken little bit earlier than scheduled.");
       }
         break;
     }
   
    case MED_OVERDOSE:   {
       request_face(FACE_SURPRISE, 3000, PRIO_OVERLAY_HIGH);   // Request the surprised expression for 3000 display updates
         
       cnt_med_overdose++;   // Increment the additional-confirmation counter

       char msg_med_overdose[4];
       uchar_to_str(cnt_med_overdose, msg_med_overdose);   // Convert the additional-confirmation count to an ASCII string
       
       uart_systime();
       uart_puts_without_newline("[ALERT] Overdose detected: ");
       uart_puts_without_newline(msg_med_overdose);
       uart_puts_without_newline(" times excess intake.\r\n");
       
       break;
      }
   }
}

void Med_on_during(MedState_t curr_state) {
   switch (curr_state) {
      case MED_IDLE:
         break;

      case MED_DUE:
         request_face(FACE_BIGEYE, HOLD_KEEP, PRIO_MED);
         ctrl_buzzer(BUZ_MED);
         break;

      case MED_DONE:
         break;

      case MED_EARLY:
         break;
   
     case MED_OVERDOSE:
       break;
   }
}

void Med_on_exit(MedState_t prev_state) {
   switch (prev_state) {
      case MED_DUE: 
         if (med_curr_state == MED_DONE)
            uart_puts("Medication completed.");
         break;
      
      case MED_EARLY:
         uart_puts("Medication completed.");
         break;
         
      default:
         break;
   }
}

//=========================
// FSM next-state logic
//=========================
ProxState_t Prox_decide_next(ProxState_t curr_state) {
   if (flags & FLAG_PROX_NEAR) return PROX_NEAR;
   return PROX_IDLE;
}

MoveState_t Move_decide_next(MoveState_t curr_state) {
   switch (curr_state) {
      case MOVE_IDLE:
         if (flags & FLAG_PROX_NEAR) return MOVE_IDLE;
         if (cnt_inactive.sec >= 15) return MOVE_N;
         return curr_state;

      case MOVE_N:
         if (flags & FLAG_PROX_NEAR) return MOVE_IDLE;
         if (cnt_inactive.sec >= 30) return MOVE_M;
         return curr_state;

      case MOVE_M:
         if (flags & FLAG_PROX_NEAR) return MOVE_IDLE;
         if (cnt_inactive.sec >= 45) return MOVE_CRIT;
         return curr_state;

      case MOVE_CRIT:
         if (flags & FLAG_PROX_NEAR) return MOVE_IDLE;
         return curr_state;
   }
   // Fallback return after the handled enum cases
   return curr_state;
}

MedState_t Med_decide_next(MedState_t curr_state) {
   switch (curr_state) {
      case MED_IDLE:
         if ((curr_time.hour == med_time.hour) &&
             (curr_time.min  == med_time.min)  &&
             (curr_time.sec  == med_time.sec))
            return MED_DUE;
         if (events & EV_MED_SWITCH) {
            events &= ~EV_MED_SWITCH;
            return MED_EARLY;
         }
         return curr_state;

      case MED_DUE:
         if (events & EV_MED_SWITCH) {
            events &= ~EV_MED_SWITCH;
            return MED_DONE;
         }
       else if (  (curr_time.hour   == 0)
              &&(curr_time.min   == 0)
              &&(curr_time.sec   == 0)) {
          return MED_IDLE;
       }
         return curr_state;

      case MED_DONE:
        if (events & EV_MED_SWITCH) {
          events &= ~EV_MED_SWITCH;
          return MED_OVERDOSE;
        }
       else if (  (curr_time.hour   == 0)
              &&(curr_time.min   == 0)
              &&(curr_time.sec   == 0)) {
          return MED_IDLE;
       }
         return curr_state;

      case MED_EARLY:
         return MED_DONE;
   
     case MED_OVERDOSE:
       return MED_DONE;
   }
   // Fallback return after the handled enum cases
   return curr_state;
}

//=========================
// FSM step functions
//=========================
void fsm_prox_step(void) {
   if (prox_prev_state != prox_curr_state) {
      Prox_on_exit(prox_prev_state);
      Prox_on_entry(prox_curr_state);
   }
   Prox_on_during(prox_curr_state);
   prox_next_state = Prox_decide_next(prox_curr_state);
   prox_prev_state = prox_curr_state;
   prox_curr_state = prox_next_state;
}

void fsm_move_step(void) {
   if (move_prev_state != move_curr_state) {
      Move_on_exit(move_prev_state);
      Move_on_entry(move_curr_state);
   }
   Move_on_during(move_curr_state);
   move_next_state = Move_decide_next(move_curr_state);
   move_prev_state = move_curr_state;
   move_curr_state = move_next_state;
}

void fsm_med_step(void) {
   if (med_prev_state != med_curr_state) {
      Med_on_exit(med_prev_state);
      Med_on_entry(med_curr_state);
   }
   Med_on_during(med_curr_state);
   med_next_state = Med_decide_next(med_curr_state);
   med_prev_state = med_curr_state;
   med_curr_state = med_next_state;
}

//=========================
// Main function
//=========================
int main(void) {
   init();

   while (true) {
      if (flags & FLAG_FSM_UPDATE) {
         flags &= ~FLAG_FSM_UPDATE;
       
         fsm_prox_step();
         fsm_move_step();
         fsm_med_step();

         ultrasonic();
       
         update_display();
      }
   }
}
