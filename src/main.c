/*
   임베디드 시스템: 기말 프로젝트
   4조
*/
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#define F_CPU   16000000UL
#include <util/delay.h>

//=========================
// 상태 정의(Definition)
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
// Flag 및 Event 정의
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
// CLCD 얼굴 상태 정의
//=========================
typedef enum {
   FACE_IDLE = 0,    // 약하게 미소 짓는 얼굴; 평상시 얼굴
   FACE_HAPPY,       // 행복한 얼굴(가까워졌을 때 & 약 먹었을 때)
   FACE_TEARFUL,     // 울먹이는 얼굴; 가까웠던 상태에서 멀어짐

   FACE_BLANK,       // 무표정; 움직임 감시 카운터 N시간 초과
   FACE_SOFT_SAD,    // 약간 슬픈 얼굴; 움직임 감시 카운터 M시간 초과
   FACE_SAD,         // 슬픈 얼굴; 움직임 감시 카운터 최종 초과

   FACE_BIGEYE,      // 눈이 동그래진 얼굴; 약 먹을 시간 됨
   FACE_SURPRISE,    // 놀란 얼굴; 약을 일찍 먹음
} Face_t;

typedef enum {
   PRIO_IDLE         = 0,   // 값이 클 수록 더 높은 우선순위
   PRIO_PROX         = 1,
   PRIO_MOVE         = 3,
   PRIO_MED          = 2,
   PRIO_OVERLAY_HIGH = 6,
   PRIO_OVERLAY_MID  = 5,
   PRIO_OVERLAY_LOW  = 4
} Prio_t;

typedef struct {
   Face_t    face;
   uint16_t  hold_ms;   // 얼굴 유지할 시간
   Prio_t    prio;      // 우선순위
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

#define HOLD_KEEP 0x0000   // hold_ms == HOLD_KEEP이면 제한 시간 없음

//=========================
// 시스템 기준 시간(24시간 기준)
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
// 피에조 부저 타입
//=========================
typedef enum {
   BUZ_MED = 0,
   BUZ_STOP
} Buz_t;

//=========================
// 하드웨어 관련 매크로/전역
//=========================
#define BUZZER 3 // 버저 신호 (출력)

// 복약 버튼
#define MED 4 // 복약 버튼 (입력)
volatile uint16_t med_btn_debounce_cnt = 0;

// 약물 남용 카운터
uint8_t cnt_med_overdose = 0;   // 약물 남용 횟수

// 초음파 센서
#define SOUND_VELOCITY 340UL
#define TRIG 6 // HC-SR04 Trigger 신호 (출력)
#define ECHO 7 // HC-SR04 Echo 신호 (입력)

//CLCD 초기화 카운터
volatile unsigned int CLCD_cnt = 0;

// 서보모터용 파라미터(꼬리 함수)
#define SERVO_MIN   225   // ? 900 us
#define SERVO_MID   375   // ? 1500 us
#define SERVO_MAX   525   // ? 2100 us

#define D_L_90      SERVO_MIN
#define D_0         SERVO_MID
#define D_R_90      SERVO_MAX

#define TAIL_STEP       1   // 한 번 업데이트할 때 duty 변화량
#define TAIL_MIN_DUTY   D_L_90
#define TAIL_MAX_DUTY   D_R_90 

//=========================
// 전역 상태 변수
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


//-----CLCD 변수-----//
//LCD Instructions
#define BIT4_LINE2_DOT58      0x28   //4 bit interface, 2 lines, 5*8 dot

#define DISPON_CUROFF_BLCKOFF   0x0C   //Display ON, Cursor OFF, Blink OFF
#define DISOFF_CUROFF_BLCKOFF   0x08   //Display OFF, Cursor OFF, Blink OFF
#define INC_NOSHIFT            0x06   //Entry Mode : Cursor Increment, Display No Shift

#define DISPCLEAR            0x01   //Clear Display : Cursor position = 0

#define CUR1LINE            0x80   //Cursor Position Line 1 First
#define CUR2LINE            0xC0   //Cursor Position Line 2 First
#define CURHOME               0x02   //Return Home

#define CGRAM_ADSET            0x40   //CGRAM address 설정  //하위 6개 bit에 address 저장
#define DRAM_ADSET            0x80   //DRAM address 설정

//LCD Initial Condition
#define E_BIT             4   //Enable Bit #
#define RW_BIT               3   //Read Write Bit #
#define RS_BIT               2   //Register Select Bit #

//CGRAM에 저장할 data set
//CLCD 얼굴 디스플레이
//윗줄 왼쪽부터 배열 작성
uint8_t emo_IDLE[7][8] = {
   {0x00,0x01,0x02,0x04,0x08,0x10,0x00,0x00},      // "/" 눈 왼쪽            //0
   {0x00,0x10,0x08,0x04,0x02,0x01,0x00,0x00},      // "\" 눈 오른쪽            //1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},         // 수염                  //2
   {0x00,0x00,0x00,0x00,0x00,0x01,0x12,0x0C},      // 입 왼쪽 끝            //3
   {0x01,0x02,0x04,0x08,0x10,0x00,0x00,0x00},      // 입 왼쪽 중간            //4
   {0x10,0x08,0x04,0x02,0x01,0x00,0x00,0x00},      // 입 오른쪽 중간            //5
   {0x00,0x00,0x00,0x00,0x00,0x10,0x09,0x06}      // 입 오른쪽 끝            //6
}; //평상시

uint8_t emo_HAPPY[5][8] = {
   {0x00,0x00,0xc,0x6,0x3,0x6,0xc,0x00},      //">"                  //0
   {0x00,0x00,0x6,0xc,0x18,0xc,0x6,0x00},      //"<"                  //1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // 수염                  //2
   {0x00,0x0,0x0,0x1f,0xf,0x7,0x3,0x1},      // 입 왼쪽               //3
   {0x00,0x0,0x0,0x1f,0x1e,0x1c,0x18,0x10}      // 입 오른쪽               //4
};   //행복

uint8_t emo_TEARFUL[4][8] = {
   {0x00,0x00,0x00,0x01,0x02,0x04,0x18,0x00},   // 눈썹 왼쪽               //0
   {0x00,0x00,0x00,0x10,0x08,0x04,0x03,0x00},   // 눈썹 오른쪽            //1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // 수염                  //2
   {0x00,0x1F,0x1F,0x0E,0x0E,0x0E,0x0E,0x00}   // 눈                  //3
};   //사용자가 멀어졌을 때

uint8_t emo_BLANK[3][8] = {
   {0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00},   // 눈썹                  //0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // 수염                  //1
   {0x0,0x0,0x1f,0x0A,0x0A,0x0E,0x0,0x00}      //눈                     //2
};   //무표정; 부동 N시간 초과


uint8_t emo_SOFT_SAD[6][8] = {
   {0x00,0x0,0x1,0x2,0x4,0x8,0x10,0x0},      // 눈썹 왼쪽               //0
   {0x00,0x0,0x10,0x8,0x4,0x2,0x1,0x0},      // 눈썹 오른쪽            //1
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0, 0x00},      // 수염                  //2
   {0x0,0xe,0xa,0xa,0xa,0xe,0x0,0x00},         // 놀란 눈               //3
   {0x00,0x0,0xf,0x8,0x8,0x8,0xf,0x0},         // 입 왼쪽               //4
   {0x00,0x0,0x1e,0x2,0x2,0x2,0x1e,0x0}      // 입 오른쪽               //5
};   //약간 슬픈 표정; 부동 M시간 초과

uint8_t emo_SAD[6][8] = {
   {0x00,0x1F,0x0A,0x0A,0x0A,0x0A,0x0A,0x00},   // "ㅠ"                  //0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // 수염                  //1
   {0x4,0x0,0x4,0x0,0x4,0x0,0x00,0x0},         // 눈물                  //2
   {0x1f,0x10,0x10,0x10,0x10,0x10,0x10,0x1f},   // 입 왼쪽               //3
   {0x1f,0x0,0x0,0x0,0x0,0x0,0x00,0x1f},      // 입 가운데               //4
   {0x1f,0x1,0x1,0x1,0x1,0x1,0x1,0x1f}         // 입 오른쪽               //5
};   //슬픈 표정; 위험 상황 감지

uint8_t emo_BIGEYE[4][8] = {
   {0x4,0xe,0x1f,0x1f,0x1f,0xe,0x4,0x00},      // 눈                  //0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},      // 수염                  //1
   {0x00,0x01,0x02,0x04,0x04,0x02,0x01,0x00},   // 입 왼쪽               //2
   {0x00,0x10,0x08,0x04,0x04,0x08,0x10,0x00}   // 입 오른쪽               //3
};      // 약 먹을 시간

uint8_t emo_SURPRISE[5][8] = {
   {0x00,0x0,0xe,0x11,0x15,0x11,0xe,0x0},         // 눈                  //0
   {0x0,0x1f,0x0,0x1f,0x0,0x1f,0x0,0x00},         // 수염                  //1
   {0x1f,0x10,0x10,0x10,0x10,0x10,0x10,0x1f},      // 입 왼쪽               //2
   {0x1f,0x0,0x0,0x0,0x0,0x0,0x00,0x1f},         // 입 가운데               //3
   {0x1f,0x1,0x1,0x1,0x1,0x1,0x1,0x1f}            // 입 오른쪽               //4
};      // 놀란 얼굴

//-----표정 display-----//
//---DDRAM에 write하는 용도의 배열---//
uint8_t display_IDLE[10] = {0, 1, 0, 1, 2, 3, 4, 5, 6, 2};               //평상시
uint8_t display_HAPPY[6] = {0, 1, 2, 3, 4, 2};                        //행복
uint8_t display_TEARFUL[6] = {0, 1, 2, 3, 3, 2};                     //사용자가 멀어졌을 떄
uint8_t display_BLANK[6] = {0, 0, 1, 2, 2, 1};                        //무표정
uint8_t display_SOFT_SAD[8] = {0, 1, 2, 3, 4, 5, 3, 2};                  //약간 슬픈 표정
uint8_t display_SAD[10] = {0, 0, 1, 2, 3, 4, 4, 5, 2, 1};               //슬픈 표정
uint8_t display_BIGEYE[6] = {0, 0, 1, 2, 3, 1};                        //약 먹을 시간
uint8_t display_SURPRISE[8] = {0, 0, 1, 2, 3, 3, 4, 1};                  //놀란 얼굴

//-----이목구비 위치-----//
uint8_t emo_IDLE_pos[10] = {0x04, 0x05, 0x0A, 0x0B, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4C};         //평상시
uint8_t emo_HAPPY_pos[6] = {0x05, 0x0A, 0x43, 0x47, 0x48, 0x4C};                           //행복
uint8_t emo_TEARFUL_pos[6] = {0x06, 0x09, 0x43, 0x46, 0x49, 0x4C};                           //사용자가 멀어졌을 때
uint8_t emo_BLANK_pos[6] = {0x05, 0x0A, 0x43, 0x45, 0x4A, 0x4C};                           //무표정
uint8_t emo_SOFT_SAD_pos[8] = {0x05, 0x0A, 0x43, 0x45, 0x47, 0x48, 0x4A, 0x4C};                  //약간 슬픈 표정
uint8_t emo_SAD_pos[10] = {0x05, 0x0A, 0x43, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4C};            //슬픈 표정
uint8_t emo_BIGEYE_pos[6] = {0x05, 0x0A, 0x43, 0x47, 0x48, 0x4C};                           //약 먹을 시간
uint8_t emo_SURPRISE_pos[8] = {0x05, 0x0A, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4C};                  //놀란 얼굴

// 꼬리(서보) 제어용
#define TAIL_MOVE 0
#define TAIL_STOP 1

volatile uint16_t tail_curr_duty = D_0;
volatile uint8_t  tail_direction = 1;  // 1: 오른쪽으로, 0: 왼쪽으로

volatile uint16_t slow_div = 0;   // 모터 속도 제어 (각도가 1ms * slow_div 간격으로 증가)
//=========================
// Function Prototypes
//=========================

// 시스템 초기화
void init(void);

// 디바운싱 타이머
void med_debounce_timer(void);

// CLCD 얼굴/디스플레이/꼬리/타이머
void CLCD_data(char data);
void CLCD_cmd(char cmd);
void request_face(Face_t req_face, uint16_t req_hold_ms, Prio_t req_prio);
void clear_face_area(void);
void update_display(void);
void ctrl_tail(int cmd);


// UART 출력 함수
void uart_puts_without_newline(const char *msg);
void uart_puts(const char *msg);
void uart_systime(void);

// 피에조 부저 제어
void ctrl_buzzer(Buz_t cmd);

// 초음파 센서 FSM
void ultrasonic(void);

// Helper 함수
void uchar_to_str(uint8_t num, char *ptr);
void zero_extend_to_2digit(char *ptr);

// Proximity FSM 상태 핸들러
void Prox_on_entry(ProxState_t curr_state);
void Prox_on_during(ProxState_t curr_state);
void Prox_on_exit(ProxState_t prev_state);

// Movement FSM 상태 핸들러
void Move_on_entry(MoveState_t curr_state);
void Move_on_during(MoveState_t curr_state);
void Move_on_exit(MoveState_t prev_state);

// Medicine FSM 상태 핸들러
void Med_on_entry(MedState_t curr_state);
void Med_on_during(MedState_t curr_state);
void Med_on_exit(MedState_t prev_state);

// FSM Next-state 결정 함수
ProxState_t Prox_decide_next(ProxState_t curr_state);
MoveState_t Move_decide_next(MoveState_t curr_state);
MedState_t Med_decide_next(MedState_t curr_state);

// FSM Step 함수들
void fsm_prox_step(void);
void fsm_move_step(void);
void fsm_med_step(void);

// main
int main(void);

//=========================
// Initialization(초기화)
//=========================
void init(void) {
   // 시스템 기준 시간 초기화
   curr_time.hour = INIT_TIME_HOUR;
   curr_time.min  = INIT_TIME_MIN;
   curr_time.sec  = INIT_TIME_SEC;

   // 비거동 시간 카운터 초기화
   cnt_inactive.sec  = 0;
   cnt_inactive.min  = 0;
   cnt_inactive.hour = 0;

   // 약 먹어야 하는 시간 설정
   med_time.hour = 10;
   med_time.min  = 30;
   med_time.sec  = 3;

   // Timer0_시스템 기준시간 생성; CTC mode, prescaler 64 -> 1kHz
   TCCR0 = (1 << WGM01) | (1 << CS02);
   OCR0  = 249;
   TIMSK = (1 << OCIE0);

   // UART 초기화
   UCSR0B = 0x08; // TX Enable
   UCSR0C = 0x06; // 8N1
   UBRR0H = 0;
   UBRR0L = 103;    // 16MHz, 1M baud (현재 값 기준)   

   // 부저 초기화
   DDRE |= (1 << BUZZER);
   TCCR3A = 0;
   TCCR3B = 0;
   TCCR3C = 0;

   // 복약 버튼 초기화
   DDRE  &= ~(1 << MED); // 입력
   EICRB |= 0x02;        // INT4 falling edge
   EIMSK |= 0x10;        // INT4 enable

   // 초음파 센서 초기화
   DDRE = ((DDRE | (1 << TRIG)) & ~(1 << ECHO)); // TRIG 출력, ECHO 입력
   PORTE &= ~(1 << TRIG);

   //CLCD 포트 설정
   DDRC = 0xff;         //PORTC : LCD 명령/data port -> 출력 포트로 설정
   DDRD = 0xff;         //PORTD : LCD control port -> 출력 포트로 설정
   
   //CLCD 초기화
   _delay_ms(50);
      
   CLCD_cmd(BIT4_LINE2_DOT58);            //LCD 환경 설정
   _delay_us(50);
   CLCD_cmd(DISPON_CUROFF_BLCKOFF);      //display ON
   _delay_us(50);
   CLCD_cmd(INC_NOSHIFT);               //커서 설정
   _delay_us(50);
   
   // CLCD_cmd(DISPCLEAR); 직접 구현. execuation delay때문에 이렇게 함
   PORTD = 0x00;            

   PORTD |= (1<<E_BIT);              
   PORTC = DISPCLEAR & 0xf0;          
   PORTD &= ~(1<<E_BIT);              
   PORTD |= (1<<E_BIT);              
   PORTC = (DISPCLEAR << 4) & 0xf0;      
   PORTD &= ~(1<<E_BIT);               
   _delay_ms(3);            // Execution delay(Clear Display)
   // 여기까지 DISPCLEAR + Execution time
   
   CLCD_cmd(CUR1LINE);                  //Cursor Position을 첫째 줄에 두기
   
   // 서보모터용 타이머 초기화 Timer1
   DDRB |=  (1 << PB5);      //서보모터 PB5 연결

   TCCR1A = 0x82;            // Fast   PWM OC1A, 16bit TCNT1, Set at bottom, Clear on compare Match
   TCCR1B = 0x1b;         //Fast PWM, 64분주
     
   ICR1 = 4999;   // Generate 50Hz at OCR1A(PB5)
   OCR1A = D_0;   // IDLE at zero
   
   // 서보 모터용 타이머 초기화 TIMER2
   // Timer2: CTC mode, prescale 1024
   // 16MHz / 1024 = 15625 Hz → OCR2 = 155 → 약 10ms
   TCCR2 = (1<<CS21) | (1<<CS20);  // Normal 모드, 64분주, 1ms 당 꼬리 타이머 인터럽트 발생
   
   // 처음에는 꼬리 타이머 인터럽트 OFF
   TIMSK &= ~(1<<OCIE2);
   
   ctrl_tail(TAIL_STOP);
         
   sei();

}

//=========================
// Helper 함수
//=========================
void uchar_to_str(uint8_t num, char *ptr) {   // ptr는 최소 4바이트 이상
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

void zero_extend_to_2digit(char *ptr) {   // Zero extend; NULL포함 2바이트 str -> 3바이트 str
   if(ptr[1] == 0x00) {            // 위 조건식 비 만족 시 아무 것도 안함
      ptr[2] = ptr[1];
      ptr[1] = ptr[0];
      ptr[0] = '0';
   }
}

//=========================
// ISR(인터럽트 서비스 루틴)
//=========================
ISR(TIMER0_COMP_vect) {
   flags |= FLAG_FSM_UPDATE;

   med_debounce_timer(); // 복약 버튼 디바운싱 카운터 감소

   // 24시간 구현
   static uint16_t cnt_1000ms = 0;
   cnt_1000ms += 1;

   if (cnt_1000ms >= 1000) {
      cnt_1000ms = 0;

      // 시스템 시간
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

      // 움직임 비거동 감시 카운터
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

ISR(TIMER2_COMP_vect)   // 꼬리 구현에서 사용 서보모터
{   
   slow_div++;
   if(slow_div > 50)   //1ms * slow_div 간격으로 각도 변화
   {
     // 왕복 운동
     if (tail_direction) {
        // 오른쪽으로 증가
        if (tail_curr_duty + TAIL_STEP < TAIL_MAX_DUTY) {
           tail_curr_duty += TAIL_STEP;
          } else {
           tail_curr_duty = TAIL_MAX_DUTY;
          tail_direction = 0;   // 끝까지 갔으니 반대로
         slow_div = 0;
      }
      } else {
        // 왼쪽으로 감소
       if (tail_curr_duty > TAIL_MIN_DUTY + TAIL_STEP) {
         tail_curr_duty -= TAIL_STEP;
         } else {
          tail_curr_duty = TAIL_MIN_DUTY;
         tail_direction = 1;   // 끝까지 갔으니 다시 반대방향
         slow_div = 0;
      }
     }

   // 실제 서보 PWM duty 반영
   OCR1A = tail_curr_duty;
   }
}

// 복약 스위치 ISR
ISR(INT4_vect) {
   if (med_btn_debounce_cnt == 0) {
      events |= EV_MED_SWITCH;
      med_btn_debounce_cnt = 1000; // 50ms 잠금
   }
}

//=========================
// 스위치 디바운스 타이머
//=========================
void med_debounce_timer(void) {
   if (med_btn_debounce_cnt > 0) {
      med_btn_debounce_cnt--;
   }
}

//=========================
// CLCD 얼굴 함수 (Stub)
//=========================
void request_face(Face_t req_face, uint16_t req_hold_ms, Prio_t req_prio) {   
   if(curr_face_state.prio <= req_prio)            //현재 display 중인 face_state보다 요청된 표정의 우선순위가 높을 경우
   {
      curr_face_state.face = req_face;            //요청된 바를 실행
      curr_face_state.hold_ms = req_hold_ms;
      curr_face_state.prio = req_prio;
   }
}

void update_display(void) {
   //=========================
   // 1. hold_ms 처리
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
   // 2. CLCD 표시 FSM
   //    - 1틱(1ms)에 CLCD_cmd / CLCD_data 최대 1번
   //=========================
   typedef enum {
      DISP_PHASE_IDLE = 0,
      DISP_PHASE_CLEAR,
      DISP_PHASE_CGRAM,
      DISP_PHASE_DDRAM
   } DispPhase_t;

   static DispPhase_t phase = DISP_PHASE_IDLE;

   // clear 영역 주소 (원래 clear_face_area()에 있던 것)
   static const uint8_t clear_positions[] = {
      0x04, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
      0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C
   };

   static uint8_t clear_idx  = 0;
   static uint8_t clear_sub  = 0;  // 0: DDRAM 주소 설정, 1: 공백 쓰기

   static uint8_t cgram_glyph = 0; // 몇 번째 글리프(캐릭터)
   static uint8_t cgram_row   = 0; // 글리프 내부 0~7 row
   static uint8_t cgram_sub   = 0; // 0: CGRAM_ADSET, 1: data write

   static uint8_t ddram_idx = 0;   // display_* / emo_*_pos 인덱스
   static uint8_t ddram_sub = 0;   // 0: DDRAM 주소 설정, 1: data write

   static Face_t drawing_face = FACE_IDLE;
   static Face_t shown_face = FACE_BLANK;
   
   static uint8_t need_clear = 0;   // FLAG로 활용할 것임. 이게 켜져 있으면 무조건 CLCD전체를 한번 클리어해야됨.

   //=========================
   // 2-1. 새 얼굴 요청 → IDLE일 때만 시작
   //=========================
   if (phase == DISP_PHASE_IDLE) {
      if (need_clear || (curr_face_state.face != shown_face)) {
       need_clear     = 0; // CLCD 전체 클리어 플래그 소모(해제)
         // 새 얼굴 그리기 시작: 먼저 기존 영역 clear
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
         // 할 일 없으면 바로 리턴 (이번 틱에는 CLCD 접근 없음)
         return;
      }
   }

   //=========================
   // 2-2. 단계별 처리 (틱당 최대 1명령)
   //=========================
   switch (phase) {
      //-----------------------------
      // (1) 얼굴 영역 Clear
      //-----------------------------
      case DISP_PHASE_CLEAR:
      if (clear_idx < ((uint8_t)( sizeof(clear_positions) / sizeof(clear_positions[0]) )) ) {
         if (clear_sub == 0) {
            // DDRAM 주소 설정
            CLCD_cmd(DRAM_ADSET + clear_positions[clear_idx]);
            clear_sub = 1;
            } 
      else {
            // 공백 쓰기
            CLCD_data(' ');
            clear_sub = 0;
            clear_idx++;
      }
      } 
     else {
         // clear 완료 → CGRAM 패턴 쓰기로 진입
         cgram_glyph = 0;
         cgram_row   = 0;
         cgram_sub   = 0;
         phase       = DISP_PHASE_CGRAM;
      }
      break;

      //-----------------------------
      // (2) CGRAM에 패턴 쓰기
      //-----------------------------
      case DISP_PHASE_CGRAM:
     
     if(drawing_face != curr_face_state.face) {   // CLCD에 얼굴을 그리는 도중 curr_face_state.face가 변한다면 현재 그리는 작업 과정 reset
        phase = DISP_PHASE_IDLE;
        need_clear = 1;
        break;
     }
     
      switch (drawing_face) {
         //================= FACE_IDLE =================
         case FACE_IDLE:
         if (cgram_glyph >= 7) {
            // 7개 글리프 다 채웠으면 DDRAM 단계로
            ddram_idx = 0;
            ddram_sub = 0;
            phase     = DISP_PHASE_DDRAM;
            break;
         }
         if (cgram_sub == 0) {
            // 해당 글리프의 시작 주소 설정
            CLCD_cmd(CGRAM_ADSET | (cgram_glyph << 3));
            cgram_sub = 1;
            } else {
            // 한 줄씩 데이터 쓰기
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
         // 방어 코드: 이상한 값이면 그냥 종료
         phase = DISP_PHASE_IDLE;
         break;
      }
      break;

      //-----------------------------
      // (3) DDRAM에 이목구비 찍기
      //-----------------------------
      case DISP_PHASE_DDRAM:
     if(drawing_face != curr_face_state.face) {   // CLCD에 얼굴을 그리는 도중 curr_face_state.face가 변한다면 현재 그리는 작업 과정 reset
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
      // 방어 코드
      //-----------------------------
      default:
      phase = DISP_PHASE_IDLE;
      break;
   }
}


void CLCD_data(char data)   //받은 문자 1개를 LCD에 write
{
   PORTD = (1<<RS_BIT);               //0000_0100, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 1 // Write 사이클, 데이터 모드
   // 16Mhz의 1클럭 period는 62.5ns, 즉 62.5ns 이하의 시간은 굳이 기다려줄 필요가 없다.
   PORTD |= (1<<E_BIT);            //0001_0100, E(bit 4) = 1, R/W(bit 3) = 0, RS(bit 2) = 1 // Enable
   PORTC = data & 0xf0;            //upper 4bit 쓰고
   /*
   80ns의 setup time을 보장해줘야 하는데, 다음 명령어가 1. shift 연산, 2. AND연산 후 PORTD에 값을 반영해주는 것이기 때문에
   2cc 이상, 125ns 이상의 delay가 보장된다.
   */
   PORTD &= ~(1<<E_BIT);            //0000_0100, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 1 // Write 사이클, 데이터 모드
   // hold time 신경 쓸 필요 X
   
   PORTD |= (1<<E_BIT);            //0001_0100, E(bit4) = 1, R/W(bit 3) = 0, RS(bit 2) = 1 // Enable
   PORTC = (data << 4) & 0xf0;   //lower 4bit 쓰고
   PORTD &= ~(1<<E_BIT);          //0000_0100, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 1 // 사이클 종료
}

void CLCD_cmd(char cmd)         //CLD에 명령어 입력 함수
{
   PORTD = 0x00;            //0000_0000, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 0 // Write 사이클, 명령어 모드
   

   PORTD |= (1<<E_BIT);               //0001_0000, E(bit 4) = 1, R/W(bit 3) = 0, RS(bit 2) = 0 // Enable
   PORTC = cmd & 0xf0;            //upper 4bit 쓰고
   PORTD &= ~(1<<E_BIT);               //0000_0000, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 0 // Write 사이클, 명령어 모드
 
   PORTD |= (1<<E_BIT);               //0001_0000, E(bit4) = 1, R/W(bit 3) = 0, RS(bit 2) = 0 // Enable
   
   PORTC = (cmd << 4) & 0xf0;      //lower 4bit 쓰고
   PORTD &= ~(1<<E_BIT);               //0000_0000, E(bit 4) = 0, R/W(bit 3) = 0, RS(bit 2) = 0 // 사이클 종료
}
//=========================
// 꼬리 함수(서보모터)
//=========================
void ctrl_tail(int cmd)
{
   switch (cmd) {
      case TAIL_MOVE:
     TCCR1B |= ( (1<<CS10) | (1<<CS11) );
      // 왕복 시작: 중앙에서 출발
      //tail_curr_duty = D_0;
      tail_direction = 1;    // 먼저 오른쪽으로 가게

      OCR1A = tail_curr_duty;

      // Timer2 Compare Match interrupt ON
      TCNT2 = 0;             // 카운터 초기화
      TIMSK |= (1<<OCIE2);
      break;

      case TAIL_STOP:
      default:
      TCCR1B &= ~( (1<<CS10) | (1<<CS11) );
      // 꼬리 정지: 중앙에 두고, 인터럽트 OFF
      TIMSK &= ~(1<<OCIE2);  // Timer2 Compare Match interrupt OFF
      //tail_curr_duty = D_0;
      OCR1A = tail_curr_duty;
      break;
   }
}

//=========================
// UART 함수
//=========================
void uart_puts_without_newline(const char *msg) {
   while (*msg) { // NULL을 만날 때까지 반복
      while(!(UCSR0A & (1<<UDRE0))); // UCSR0A 5번 비트 = UDRE(UDR Empty). UDRE0는 define된 값이 5이므로 5번 비트 검사
      UDR0 = *msg++; // 1 문자 전송
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
// 피에조 부저 함수
//=========================
void ctrl_buzzer(Buz_t cmd) {
   if (cmd == BUZ_MED) {
      OCR3A = 61;             // 2016Hz
      TCCR3A = (1 << COM3A0); // toggle
      TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30); // CTC, 64분주
   }
   else {
      TCCR3A = 0;
      TCCR3B = 0;
      PORTE &= ~(1 << BUZZER);
   }
}

//=========================
// 초음파 센서 함수
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
// FSM 상태별 구현
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
       cnt_med_overdose = 0;   // 하루가 지났으므로 약물 남용 횟수 카운터 초기화
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
       request_face(FACE_SURPRISE, 3000, PRIO_OVERLAY_HIGH);   // 놀란 얼굴 3초 동안 유지
         
       cnt_med_overdose++;   // 약물 남용 횟수 증가

       char msg_med_overdose[4];
       uchar_to_str(cnt_med_overdose, msg_med_overdose);   // 약물 남용 횟수 ASCII string 변환
       
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
// FSM Next-state 로직
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
   // 모든 case를 이미 다뤘지만, 컴파일러 안심용
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
   // 컴파일러 안심용
   return curr_state;
}

//=========================
// FSM Step 함수
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
// main 함수
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
