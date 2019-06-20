#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 38 "/opt/msp430/msp430/include/sys/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void *source);




static __inline uint8_t __nesc_hton_uint8(void *target, uint8_t value);





static __inline uint8_t __nesc_ntoh_leuint8(const void *source);




static __inline uint8_t __nesc_hton_leuint8(void *target, uint8_t value);





static __inline int8_t __nesc_hton_int8(void *target, int8_t value);
#line 269
static __inline uint16_t __nesc_hton_uint16(void *target, uint16_t value);






static __inline uint16_t __nesc_ntoh_leuint16(const void *source);




static __inline uint16_t __nesc_hton_leuint16(void *target, uint16_t value);
#line 385
typedef struct { char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 151 "/opt/msp430/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 41 "/opt/msp430/msp430/include/sys/types.h"
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/opt/msp430/msp430/include/string.h"
extern void *memcpy(void *, const void *, size_t );

extern void *memset(void *, int , size_t );
#line 63
extern void *memset(void *, int , size_t );
# 59 "/opt/msp430/msp430/include/stdlib.h"
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/msp430/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/msp430/msp430/include/sys/_types.h"
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/msp430/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int );




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/msp430/msp430/include/math.h"
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 20 "/opt/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 21
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;





struct __nesc_attr_atmostonce {
};
#line 31
struct __nesc_attr_atleastonce {
};
#line 32
struct __nesc_attr_exactlyonce {
};
# 34 "/opt/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2);
# 39 "/opt/msp430/msp430/include/msp430/iostructures.h"
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
# 106 "/opt/msp430/msp430/include/msp430/iostructures.h" 3
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};




struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 103 "/opt/msp430/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");






volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");






volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");






volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");






volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");






volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 85 "/opt/msp430/msp430/include/msp430/usart.h"
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 256
volatile unsigned char U1TCTL __asm ("0x0079");
# 24 "/opt/msp430/msp430/include/msp430/timera.h"
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");

volatile unsigned int TA0CCTL2 __asm ("0x0166");

volatile unsigned int TA0R __asm ("0x0170");
# 114 "/opt/msp430/msp430/include/msp430/timera.h" 3
#line 105
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 130
#line 116
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/opt/msp430/msp430/include/msp430/timerb.h"
volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBR __asm ("0x0190");

volatile unsigned int TBCCR0 __asm ("0x0192");
#line 75
#line 63
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 90
#line 77
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 18 "/opt/msp430/msp430/include/msp430/basic_clock.h"
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/msp430/msp430/include/msp430/adc12.h"
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");





volatile unsigned int ADC12IV __asm ("0x01A8");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 71 "/opt/msp430/msp430/include/msp430x16x.h"
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 142 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
 static volatile uint8_t U0CTLnr __asm ("0x0070");
 static volatile uint8_t I2CTCTLnr __asm ("0x0071");
 static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 177
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2);


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void );





static inline void __nesc_enable_interrupt(void );




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )  ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)  ;
# 33 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_SET_SIMO0_PIN(void);
#line 33
static inline void TOSH_CLR_SIMO0_PIN(void);
#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT(void);
static inline void TOSH_SET_UCLK0_PIN(void);
#line 34
static inline void TOSH_CLR_UCLK0_PIN(void);
#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT(void);
#line 69
static inline void TOSH_MAKE_GIO57_OUTPUT(void);
#line 93
enum __nesc_unnamed4258 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN(void);
#line 102
static inline void TOSH_CLR_FLASH_CS_PIN(void);
#line 102
static inline void TOSH_MAKE_FLASH_CS_OUTPUT(void);
static inline void TOSH_SET_FLASH_HOLD_PIN(void);
#line 103
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT(void);








static inline void TOSH_SET_DS28DG02_SI_PIN(void);
#line 112
static inline void TOSH_CLR_DS28DG02_SI_PIN(void);
#line 112
static inline void TOSH_MAKE_DS28DG02_SI_OUTPUT(void);
static inline uint8_t TOSH_READ_DS28DG02_SO_PIN(void);
#line 113
static inline void TOSH_MAKE_DS28DG02_SO_INPUT(void);
static inline void TOSH_SET_DS28DG02_SCK_PIN(void);
#line 114
static inline void TOSH_CLR_DS28DG02_SCK_PIN(void);
#line 114
static inline void TOSH_MAKE_DS28DG02_SCK_OUTPUT(void);
static inline void TOSH_SET_DS28DG02_CS_PIN(void);
#line 115
static inline void TOSH_CLR_DS28DG02_CS_PIN(void);
#line 115
static inline void TOSH_MAKE_DS28DG02_CS_OUTPUT(void);
static inline void TOSH_MAKE_DS28DG02_ALM_INPUT(void);
static inline void TOSH_MAKE_DS28DG02_RST_INPUT(void);


static inline void TOSH_MAKE_SW1_INPUT(void);
# 43 "/opt/msp430/lib/gcc-lib/msp430/3.2.3/include/stdarg.h"
typedef __builtin_va_list __gnuc_va_list;
# 110 "/opt/msp430/lib/gcc-lib/msp430/3.2.3/include/stdarg.h" 3
typedef __gnuc_va_list va_list;
# 21 "UBee430_AP_MSG.h"
#line 4
typedef nx_struct msg {

  nx_uint8_t start;


  nx_uint8_t msg_type;
  nx_uint8_t device_type;
  nx_uint16_t node_id;
  nx_uint8_t ID[6];
  nx_uint8_t RTC[6];
  nx_uint8_t internalvolt;
  nx_uint8_t internaltemp;
  nx_uint8_t temp;
  nx_uint8_t humid;
  nx_uint16_t photo;
  nx_uint16_t adc0;
  nx_uint8_t etx;
} __attribute__((packed)) msg_t;
#line 36
#line 24
typedef nx_struct _recv_msg {
  nx_uint8_t start;
  nx_uint8_t msg_type;
  nx_uint8_t device_type;
  nx_uint16_t node_id;
  nx_uint8_t d0;
  nx_uint8_t d1;
  nx_uint8_t d2;
  nx_uint8_t d3;
  nx_uint8_t d4;
  nx_uint8_t d5;
  nx_uint8_t etx;
} __attribute__((packed)) recv_msg;
#line 52
#line 38
typedef nx_struct rt_msg {

  nx_uint8_t start;


  nx_uint8_t msg_type;
  nx_uint8_t device_type;
  nx_uint16_t node_id;
  nx_uint8_t d0;
  nx_uint8_t d1;
  nx_uint8_t d2;
  nx_uint8_t d3;
  nx_uint8_t etx;
} __attribute__((packed)) 
return_msg;

enum __nesc_unnamed4259 {
  AM_MSG = 9
};
# 30 "/opt/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4260 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 59
#line 45
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;






  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 64
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 86
#line 71
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t tx_power;
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_bool crc;
  nx_bool ack;
  nx_uint16_t time;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 






cc2420_metadata_t;





#line 89
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 123
enum __nesc_unnamed4261 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};
# 6 "/opt/tinyos-2.x/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4262 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4263 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/opt/tinyos-2.x/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4264 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4265 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4266 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;
# 48 "/opt/tinyos-2.x/tos/platforms/telosa/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;



#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
} message_metadata_t;
# 19 "/opt/tinyos-2.x/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 29 "/opt/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4267 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4268 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4269 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4270 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4271 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4272 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4273 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 37 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.h"
enum __nesc_unnamed4274 {
  SHT11_TEMPERATURE_BITS = 14, 
  SHT11_HUMIDITY_BITS = 12
};

enum __nesc_unnamed4275 {
  SHT11_STATUS_LOW_RES_BIT = 1 << 0, 
  SHT11_STATUS_NO_RELOAD_BIT = 1 << 1, 
  SHT11_STATUS_HEATER_ON_BIT = 1 << 2, 
  SHT11_STATUS_LOW_BATTERY_BIT = 1 << 6
};
# 33 "/opt/tinyos-2.x/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 59 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 48
typedef struct __nesc_unnamed4276 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
  unsigned int  : 0;
} msp430adc12_channel_config_t;








#line 61
typedef struct __nesc_unnamed4277 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};
# 38 "/opt/tinyos-2.x/tos/chips/cc2420/IEEE802154.h"
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 56 "/opt/tinyos-2.x/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4278 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4279 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4280 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4281 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4282 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4283 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4284 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4285 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4286 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4287 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4288 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4289 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4290 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4291 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4292 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
typedef uint16_t UBee430_APC$AdcZero$val_t;
typedef TMilli UBee430_APC$Timer0$precision_tag;
typedef uint16_t UBee430_APC$InternalVolt$val_t;
typedef uint16_t UBee430_APC$Photo$val_t;
typedef uint16_t UBee430_APC$Humidity$val_t;
typedef uint16_t UBee430_APC$InternalTemp$val_t;
typedef uint16_t UBee430_APC$Temperature$val_t;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC$0$__nesc_unnamed4293 {
  Msp430Timer32khzC$0$ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
typedef uint16_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$val_t;
typedef uint16_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$val_t;
typedef TMilli /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$precision_tag;
typedef TMilli HplSensirionSht11P$Timer$precision_tag;
enum /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$__nesc_unnamed4294 {
  SensirionSht11C$0$TEMP_KEY = 0U
};
enum /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$__nesc_unnamed4295 {
  SensirionSht11C$0$HUM_KEY = 1U
};
typedef const msp430adc12_channel_config_t *AdcP$ConfigReadStream$adc_config_t;
typedef uint16_t AdcP$Read$val_t;
typedef uint16_t AdcP$ReadNow$val_t;
typedef const msp430adc12_channel_config_t *AdcP$Config$adc_config_t;
typedef uint16_t AdcP$ReadStream$val_t;
typedef TMilli Msp430RefVoltGeneratorP$SwitchOffTimer$precision_tag;
typedef TMilli Msp430RefVoltGeneratorP$SwitchOnTimer$precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP$Config$adc_config_t;
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$__nesc_unnamed4296 {
  Msp430Adc12ClientAutoRVGC$0$ID = 0U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t;
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$__nesc_unnamed4297 {
  AdcReadClientC$0$CLIENT = 0U
};
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$__nesc_unnamed4298 {
  Msp430Adc12ClientAutoRVGC$1$ID = 1U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t;
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$__nesc_unnamed4299 {
  AdcReadStreamClientC$0$RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *Msp430InternalVoltageP$AdcConfigure$adc_config_t;
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$__nesc_unnamed4300 {
  Msp430Adc12ClientAutoRVGC$2$ID = 2U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$adc_config_t;
enum /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$__nesc_unnamed4301 {
  AdcReadNowClientC$0$CLIENT = 1U
};
enum /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$__nesc_unnamed4302 {
  Msp430Adc12ClientAutoRVGC$3$ID = 3U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$adc_config_t;
enum /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$__nesc_unnamed4303 {
  AdcReadClientC$1$CLIENT = 2U
};
enum /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$__nesc_unnamed4304 {
  Msp430Adc12ClientAutoRVGC$4$ID = 4U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$adc_config_t;
enum /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$__nesc_unnamed4305 {
  AdcReadStreamClientC$1$RSCLIENT = 1U
};
typedef const msp430adc12_channel_config_t *Msp430InternalTemperatureP$AdcConfigure$adc_config_t;
enum /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$__nesc_unnamed4306 {
  Msp430Adc12ClientAutoRVGC$5$ID = 5U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$adc_config_t;
enum /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$__nesc_unnamed4307 {
  AdcReadClientC$2$CLIENT = 3U
};
typedef const msp430adc12_channel_config_t *LightToVoltageP$AdcConfigure$adc_config_t;
typedef TMilli Ds28dg02P$Timer0$precision_tag;
enum /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$__nesc_unnamed4308 {
  Msp430Adc12ClientAutoRVGC$6$ID = 6U
};
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$adc_config_t;
enum /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$__nesc_unnamed4309 {
  AdcReadClientC$3$CLIENT = 4U
};
typedef const msp430adc12_channel_config_t *AdcZeroP$AdcConfigure$adc_config_t;
typedef T32khz CC2420ControlP$StartupTimer$precision_tag;
typedef uint32_t CC2420ControlP$StartupTimer$size_type;
typedef uint16_t CC2420ControlP$ReadRssi$val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC$1$__nesc_unnamed4310 {
  Msp430Timer32khzC$1$ALARM_ID = 1U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC$1$to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC$1$from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC$1$from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC$1$upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC$1$from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC$1$from_size_type /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC$1$to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC$0$__nesc_unnamed4311 {
  CC2420SpiC$0$CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$__nesc_unnamed4312 {
  Msp430Spi0C$0$CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$__nesc_unnamed4313 {
  Msp430Usart0C$0$CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$__nesc_unnamed4314 {
  CC2420SpiC$1$CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC$2$__nesc_unnamed4315 {
  CC2420SpiC$2$CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP$BackoffTimer$precision_tag;
typedef uint32_t CC2420TransmitP$BackoffTimer$size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC$3$__nesc_unnamed4316 {
  CC2420SpiC$3$CLIENT_ID = 3U
};
enum /*CC2420ReceiveC.Spi*/CC2420SpiC$4$__nesc_unnamed4317 {
  CC2420SpiC$4$CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgP$SeedInit$parameter;
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void UBee430_APC$SplitControl$startDone(error_t arg_0x4062baf0);
#line 117
static  void UBee430_APC$SplitControl$stopDone(error_t arg_0x4062a6e8);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void UBee430_APC$AdcZero$readDone(error_t arg_0x40624580, UBee430_APC$AdcZero$val_t arg_0x40624708);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void UBee430_APC$Timer0$fired(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void UBee430_APC$rt_Send$runTask(void);
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static  void UBee430_APC$Boot$booted(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void UBee430_APC$InternalVolt$readDone(error_t arg_0x40624580, UBee430_APC$InternalVolt$val_t arg_0x40624708);
#line 63
static  void UBee430_APC$Photo$readDone(error_t arg_0x40624580, UBee430_APC$Photo$val_t arg_0x40624708);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static  void UBee430_APC$AMSend$sendDone(message_t *arg_0x406665f8, error_t arg_0x40666780);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void UBee430_APC$Send$runTask(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void UBee430_APC$Humidity$readDone(error_t arg_0x40624580, UBee430_APC$Humidity$val_t arg_0x40624708);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *UBee430_APC$Receive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void UBee430_APC$InternalTemp$readDone(error_t arg_0x40624580, UBee430_APC$InternalTemp$val_t arg_0x40624708);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void UBee430_APC$next$runTask(void);
# 12 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
static  void UBee430_APC$DS28$EEPROM_WriteDone(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void UBee430_APC$Temperature$readDone(error_t arg_0x40624580, UBee430_APC$Temperature$val_t arg_0x40624708);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t LedsP$Init$init(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static   void LedsP$Leds$led0Toggle(void);
#line 72
static   void LedsP$Leds$led1Toggle(void);
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$get(void);
#line 52
static   uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$getRaw(void);






static   bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$get(void);
#line 52
static   uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$getRaw(void);
#line 64
static   void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$makeInput(void);
#line 59
static   bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$get(void);
#line 52
static   uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$getRaw(void);
#line 64
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeInput(void);






static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeOutput(void);
#line 59
static   bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$get(void);
#line 52
static   uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$getRaw(void);
#line 34
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$set(void);




static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$clr(void);
#line 64
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeInput(void);






static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$set(void);




static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$clr(void);
#line 71
static   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$set(void);




static   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$clr(void);
#line 85
static   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectModuleFunc(void);






static   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectModuleFunc(void);






static   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectModuleFunc(void);






static   void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP$20$IO$selectIOFunc(void);
#line 85
static   void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP$21$IO$selectIOFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$makeInput(void);
#line 59
static   bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$get(void);
#line 85
static   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectIOFunc(void);
#line 52
static   uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$getRaw(void);
#line 78
static   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectModuleFunc(void);
#line 71
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$set(void);




static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$clr(void);
#line 71
static   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$set(void);




static   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$clr(void);
#line 71
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$set(void);




static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$clr(void);




static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void);
#line 71
static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void);









static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void);
#line 71
static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void);
#line 71
static   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void);
#line 64
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void);
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void);



static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void);

static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void);



static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void);





static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t PlatformP$Init$init(void);
#line 51
static  error_t MotePlatformC$Init$init(void);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void);
#line 32
static  void Msp430ClockP$Msp430ClockInit$default$initTimerB(void);



static  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void);
#line 31
static  void Msp430ClockP$Msp430ClockInit$default$initTimerA(void);





static  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void);
#line 34
static  void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void);
#line 29
static  void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void);
static  void Msp430ClockP$Msp430ClockInit$default$initClocks(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t Msp430ClockP$Init$init(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40875be0);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void);


static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t arg_0x40850ca0);
#line 43
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void);
#line 39
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int arg_0x40851b08);





static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t arg_0x4084f178);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40875be0);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void);
static   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void);



static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(msp430_compare_control_t arg_0x40865180);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t arg_0x4085fed0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void);



static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(msp430_compare_control_t arg_0x40865180);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t arg_0x4085fed0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void);







static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void);
#line 36
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void);



static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void);
#line 33
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t arg_0x4085fed0);

static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t arg_0x4085e868);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void);
#line 57
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$clearOverflow(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$setControlAsCapture(bool arg_0x40865958);
#line 31
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void);







static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$enableEvents(void);
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$disableEvents(void);
#line 33
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$clearPendingInterrupt(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void);







static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void);
#line 36
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$setControlAsCompare(void);



static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void);
#line 33
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t arg_0x4085fed0);

static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t arg_0x4085e868);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t arg_0x40872358);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405d4868);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405d4868);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static  void SchedulerBasicP$Scheduler$init(void);
#line 61
static  void SchedulerBasicP$Scheduler$taskLoop(void);
#line 54
static  bool SchedulerBasicP$Scheduler$runNextTask(void);
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static   mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static   void McuSleepC$McuSleep$sleep(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void);
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type arg_0x4092b728);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void);






static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void);










static   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void);
#line 53
static   /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void);
#line 92
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type arg_0x4092b728);
#line 105
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void);




static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void);
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void);
#line 118
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t arg_0x40684258, uint32_t arg_0x406843e8);
#line 67
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void);
#line 72
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409e58a8);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409e58a8, 
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t arg_0x40686030);








static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409e58a8, 
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t arg_0x40686600);




static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409e58a8);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void);
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidityDone(error_t arg_0x40a3c088, uint16_t arg_0x40a3c218);
#line 116
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$writeStatusRegDone(error_t arg_0x40a3b7d8);
#line 100
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$readStatusRegDone(error_t arg_0x40a3cb70, uint8_t arg_0x40a3ccf8);
#line 54
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$resetDone(error_t arg_0x40a3fd10);
#line 69
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureTemperatureDone(error_t arg_0x40a3e5e0, uint16_t arg_0x40a3e770);
# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
static  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumidityMetadata$getSignificantBits(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$read(void);
# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
static  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TemperatureMetadata$getSignificantBits(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$granted(void);
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureHumidityDone(error_t arg_0x40a3c088, uint16_t arg_0x40a3c218);
#line 116
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$writeStatusRegDone(error_t arg_0x40a3b7d8);
#line 100
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$readStatusRegDone(error_t arg_0x40a3cb70, uint8_t arg_0x40a3ccf8);
#line 54
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$resetDone(error_t arg_0x40a3fd10);
#line 69
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperatureDone(error_t arg_0x40a3e5e0, uint16_t arg_0x40a3e770);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$granted(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$read(void);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$fired(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$runTask(void);
#line 64
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$runTask(void);
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureHumidityDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3c088, uint16_t arg_0x40a3c218);
#line 76
static  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidity(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58);
# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperature(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58);
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$writeStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3b7d8);
#line 100
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$readStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3cb70, uint8_t arg_0x40a3ccf8);
#line 54
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$resetDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3fd10);
#line 69
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureTemperatureDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3e5e0, uint16_t arg_0x40a3e770);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$fired(void);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeInput(void);
#line 32
static   bool /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$get(void);


static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeOutput(void);
#line 29
static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$set(void);
static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$clr(void);


static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeInput(void);

static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeOutput(void);
#line 29
static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$set(void);
static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$clr(void);




static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$makeOutput(void);
#line 29
static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$set(void);
static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$clr(void);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t HplSensirionSht11P$SplitControl$start(void);
#line 109
static  error_t HplSensirionSht11P$SplitControl$stop(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void HplSensirionSht11P$stopTask$runTask(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void HplSensirionSht11P$Timer$fired(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void HplMsp430InterruptP$Port14$clear(void);
#line 36
static   void HplMsp430InterruptP$Port14$disable(void);
#line 56
static   void HplMsp430InterruptP$Port14$edge(bool arg_0x40aceef8);
#line 31
static   void HplMsp430InterruptP$Port14$enable(void);









static   void HplMsp430InterruptP$Port26$clear(void);
#line 61
static   void HplMsp430InterruptP$Port26$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port17$clear(void);
#line 61
static   void HplMsp430InterruptP$Port17$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port21$clear(void);
#line 61
static   void HplMsp430InterruptP$Port21$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port12$clear(void);
#line 61
static   void HplMsp430InterruptP$Port12$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port24$clear(void);
#line 61
static   void HplMsp430InterruptP$Port24$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port15$clear(void);
#line 36
static   void HplMsp430InterruptP$Port15$disable(void);
#line 56
static   void HplMsp430InterruptP$Port15$edge(bool arg_0x40aceef8);
#line 31
static   void HplMsp430InterruptP$Port15$enable(void);









static   void HplMsp430InterruptP$Port27$clear(void);
#line 61
static   void HplMsp430InterruptP$Port27$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port10$clear(void);
#line 36
static   void HplMsp430InterruptP$Port10$disable(void);
#line 56
static   void HplMsp430InterruptP$Port10$edge(bool arg_0x40aceef8);
#line 31
static   void HplMsp430InterruptP$Port10$enable(void);









static   void HplMsp430InterruptP$Port22$clear(void);
#line 61
static   void HplMsp430InterruptP$Port22$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port13$clear(void);
#line 61
static   void HplMsp430InterruptP$Port13$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port25$clear(void);
#line 61
static   void HplMsp430InterruptP$Port25$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port16$clear(void);
#line 61
static   void HplMsp430InterruptP$Port16$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port20$clear(void);
#line 61
static   void HplMsp430InterruptP$Port20$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port11$clear(void);
#line 61
static   void HplMsp430InterruptP$Port11$default$fired(void);
#line 41
static   void HplMsp430InterruptP$Port23$clear(void);
#line 61
static   void HplMsp430InterruptP$Port23$default$fired(void);
#line 61
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$fired(void);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$disable(void);
#line 43
static   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$enableFallingEdge(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void);








static   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t arg_0x40b83e20);







static   resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$default$requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$release(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$request(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$default$granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$runTask(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$startDone(error_t arg_0x4062baf0);
#line 117
static  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stopDone(error_t arg_0x4062a6e8);
# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$default$cleanup(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$runTask(void);
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$requested(void);
#line 46
static   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$granted(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$runTask(void);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$start(void);









static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$stop(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$default$getConfiguration(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31430);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$default$getData(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00);
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27a00[], uint16_t arg_0x40c27b98);
#line 138
static   error_t AdcP$SingleChannelReadStream$default$configureMultiple(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2b848, uint16_t arg_0x40c2b9f8[], uint16_t arg_0x40c2bb90, uint16_t arg_0x40c2bd20);
#line 206
static   error_t AdcP$SingleChannelReadStream$singleDataReady(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27250);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$SubResourceReadNow$granted(
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c33168);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceReadStream$default$release(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c25a58);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceReadStream$granted(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c25a58);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t AdcP$Read$read(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0fb48);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void AdcP$Read$default$readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0fb48, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t arg_0x40624580, AdcP$Read$val_t arg_0x40624708);
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static   void AdcP$ReadNow$default$readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0e9a0, 
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t arg_0x40bfdd98, AdcP$ReadNow$val_t arg_0x40bfdf20);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceReadNow$default$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0c7b0);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$Config$adc_config_t AdcP$Config$default$getConfiguration(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c33bb8);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void AdcP$finishStreamRequest$runTask(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$default$getData(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$default$configureSingle(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2c2c0);
#line 227
static   uint16_t *AdcP$SingleChannel$multipleDataReady(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27a00[], uint16_t arg_0x40c27b98);
#line 206
static   error_t AdcP$SingleChannel$singleDataReady(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27250);
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static  void AdcP$ReadStream$default$bufferDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0b1d8, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x40c04638, 
AdcP$ReadStream$val_t *arg_0x40c047f0, uint16_t arg_0x40c04980);
#line 102
static  void AdcP$ReadStream$default$readDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0b1d8, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x40c03010, uint32_t arg_0x40c031a8);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$default$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$default$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceRead$granted(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool AdcP$ResourceRead$default$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void AdcP$signalBufferDone$runTask(void);
#line 64
static  void AdcP$readDone$runTask(void);
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static   void Msp430Adc12ImplP$MultiChannel$default$dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96068, 
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *arg_0x40c82e90, uint16_t arg_0x40c80068);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t arg_0x40cb7120);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA1$fired(void);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$default$memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96818);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96818);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t Msp430Adc12ImplP$Init$init(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void Msp430Adc12ImplP$TimerA$overflow(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$getData(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2c2c0);
#line 227
static   uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27a00[], uint16_t arg_0x40c27b98);
#line 138
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2b848, uint16_t arg_0x40c2b9f8[], uint16_t arg_0x40c2bb90, uint16_t arg_0x40c2bd20);
#line 206
static   error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27250);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA0$fired(void);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void);
#line 82
static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t arg_0x40c8d928);
#line 106
static   void HplAdc12P$HplAdc12$resetIFGs(void);
#line 118
static   bool HplAdc12P$HplAdc12$isBusy(void);
#line 75
static   void HplAdc12P$HplAdc12$setMCtl(uint8_t arg_0x40c8d1d8, adc12memctl_t arg_0x40c8d370);
#line 128
static   void HplAdc12P$HplAdc12$startConversion(void);
#line 51
static   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t arg_0x40c8e010);
#line 89
static   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t arg_0x40c8ded8);





static   void HplAdc12P$HplAdc12$setIEFlags(uint16_t arg_0x40c8c488);
#line 123
static   void HplAdc12P$HplAdc12$stopConversion(void);









static   void HplAdc12P$HplAdc12$enableConversion(void);
#line 57
static   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t arg_0x40c8e550);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void);








static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t arg_0x40b83e20);







static   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d399c8);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d374d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d374d8);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d39088);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d39088);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d39088);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d39088);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t arg_0x40cb7120);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void);
#line 83
static  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void);
#line 109
static  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$default$getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d75d80);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t arg_0x4062baf0);
#line 117
static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t arg_0x4062a6e8);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d77d50);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$request(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d77d50);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$ClientResource$default$granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d77d50);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$ClientResource$isOwner(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d77d50);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void Msp430RefVoltArbiterImplP$switchOff$runTask(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t arg_0x4062baf0);
#line 117
static  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t arg_0x4062a6e8);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void);
#line 58
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void);
#line 58
static   Msp430InternalVoltageP$AdcConfigure$adc_config_t Msp430InternalVoltageP$AdcConfigure$getConfiguration(void);
#line 58
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$getConfiguration(void);
#line 58
static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$getConfiguration(void);
#line 58
static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$getConfiguration(void);
#line 58
static   Msp430InternalTemperatureP$AdcConfigure$adc_config_t Msp430InternalTemperatureP$AdcConfigure$getConfiguration(void);
#line 58
static   /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$adc_config_t /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$getConfiguration(void);
#line 58
static   LightToVoltageP$AdcConfigure$adc_config_t LightToVoltageP$AdcConfigure$getConfiguration(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Ds28dg02P$Timer0$fired(void);
# 21 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number4(void);



static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_seconds(void);









static  void Ds28dg02P$Ds28dg02s$WriteRTC_dayofweek(uint8_t arg_0x4067b100);
#line 33
static  void Ds28dg02P$Ds28dg02s$WriteRTC_minutes(uint8_t arg_0x4067c7c8);
static  void Ds28dg02P$Ds28dg02s$WriteRTC_hours(uint8_t arg_0x4067cc50);
#line 19
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number2(void);
#line 31
static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_years(void);
#line 5
static  void Ds28dg02P$Ds28dg02s$init(void);
#line 22
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number5(void);
#line 37
static  void Ds28dg02P$Ds28dg02s$WriteRTC_months(uint8_t arg_0x4067bb10);
#line 17
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number0(void);









static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_hours(void);


static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_months(void);

static  void Ds28dg02P$Ds28dg02s$WriteRTC_seconds(uint8_t arg_0x4067c338);
#line 29
static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_date(void);
#line 20
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number3(void);





static  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_minutes(void);
#line 38
static  void Ds28dg02P$Ds28dg02s$WriteRTC_years(uint8_t arg_0x4067a010);
#line 36
static  void Ds28dg02P$Ds28dg02s$WriteRTC_date(uint8_t arg_0x4067b688);
#line 18
static  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number1(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$adc_config_t /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$getConfiguration(void);
#line 58
static   AdcZeroP$AdcConfigure$adc_config_t AdcZeroP$AdcConfigure$getConfiguration(void);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  void CC2420ActiveMessageP$SubSend$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ActiveMessageP$SubReceive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static  void CC2420ActiveMessageP$CC2420Config$syncDone(error_t arg_0x40ea9e98);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static  error_t CC2420ActiveMessageP$AMSend$send(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebd188, 
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t arg_0x406682c0, message_t *arg_0x40668470, uint8_t arg_0x406685f8);
#line 125
static  void *CC2420ActiveMessageP$AMSend$getPayload(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebd188, 
# 125 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t *arg_0x40665248);
#line 99
static  void CC2420ActiveMessageP$AMSend$default$sendDone(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebd188, 
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t *arg_0x406665f8, error_t arg_0x40666780);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ActiveMessageP$Snoop$default$receive(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebc320, 
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 67 "/opt/tinyos-2.x/tos/interfaces/Packet.nc"
static  uint8_t CC2420ActiveMessageP$Packet$payloadLength(message_t *arg_0x40661900);
#line 108
static  void *CC2420ActiveMessageP$Packet$getPayload(message_t *arg_0x40674d60, uint8_t *arg_0x40674f08);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ActiveMessageP$Receive$default$receive(
# 40 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebdab8, 
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 57 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static  am_addr_t CC2420ActiveMessageP$AMPacket$address(void);









static  am_addr_t CC2420ActiveMessageP$AMPacket$destination(message_t *arg_0x40e89d70);
#line 136
static  am_id_t CC2420ActiveMessageP$AMPacket$type(message_t *arg_0x40e85258);
#line 125
static  bool CC2420ActiveMessageP$AMPacket$isForMe(message_t *arg_0x40e86b10);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t CC2420CsmaP$SplitControl$start(void);
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static   void CC2420CsmaP$RadioBackoff$default$requestCca(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9ccf8);
#line 81
static   void CC2420CsmaP$RadioBackoff$default$requestInitialBackoff(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9c190);






static   void CC2420CsmaP$RadioBackoff$default$requestCongestionBackoff(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9c748);
#line 81
static   void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t *arg_0x40e9c190);






static   void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t *arg_0x40e9c748);
# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static   void CC2420CsmaP$CC2420Transmit$sendDone(message_t *arg_0x40efc720, error_t arg_0x40efc8a8);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  error_t CC2420CsmaP$Send$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790);
# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static   void CC2420CsmaP$CC2420Power$startOscillatorDone(void);
#line 56
static   void CC2420CsmaP$CC2420Power$startVRegDone(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420CsmaP$Resource$granted(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void CC2420CsmaP$sendDone_task$runTask(void);
#line 64
static  void CC2420CsmaP$stopDone_task$runTask(void);
#line 64
static  void CC2420CsmaP$startDone_task$runTask(void);
# 101 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static   bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void);
#line 96
static   bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void);
#line 64
static   uint16_t CC2420ControlP$CC2420Config$getShortAddr(void);
#line 52
static  error_t CC2420ControlP$CC2420Config$sync(void);
#line 70
static   uint16_t CC2420ControlP$CC2420Config$getPanAddr(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void CC2420ControlP$StartupTimer$fired(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void CC2420ControlP$ReadRssi$default$readDone(error_t arg_0x40624580, CC2420ControlP$ReadRssi$val_t arg_0x40624708);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void CC2420ControlP$syncDone$runTask(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t CC2420ControlP$Init$init(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420ControlP$SpiResource$granted(void);
#line 92
static  void CC2420ControlP$SyncResource$granted(void);
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static   error_t CC2420ControlP$CC2420Power$startOscillator(void);
#line 90
static   error_t CC2420ControlP$CC2420Power$rxOn(void);
#line 51
static   error_t CC2420ControlP$CC2420Power$startVReg(void);
#line 63
static   error_t CC2420ControlP$CC2420Power$stopVReg(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void CC2420ControlP$sync$runTask(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420ControlP$Resource$release(void);
#line 78
static   error_t CC2420ControlP$Resource$request(void);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void CC2420ControlP$InterruptCCA$fired(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420ControlP$RssiResource$granted(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void);
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type arg_0x4092b598, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type arg_0x4092b728);
#line 62
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$stop(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Init$init(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$overflow(void);
#line 53
static   /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$size_type /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$get(void);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$getNow(void);
#line 92
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$size_type arg_0x4092b598, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$size_type arg_0x4092b728);
#line 55
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$size_type arg_0x4092d460);






static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$stop(void);




static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$fired(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$overflow(void);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$makeInput(void);
#line 32
static   bool /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$get(void);


static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$makeOutput(void);
#line 29
static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set(void);
static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr(void);

static   bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$GeneralIO$get(void);
#line 32
static   bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$GeneralIO$get(void);


static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$makeOutput(void);
#line 29
static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$set(void);
static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$clr(void);


static   void /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$makeInput(void);
#line 32
static   bool /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$get(void);


static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$makeOutput(void);
#line 29
static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$set(void);
static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$clr(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$captured(uint16_t arg_0x40872358);
# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureFallingEdge(void);
#line 55
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$disable(void);
#line 42
static   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureRisingEdge(void);
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$fired(void);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$disable(void);
#line 42
static   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$enableRisingEdge(void);
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$fired(void);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$disable(void);
#line 43
static   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$enableFallingEdge(void);
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static   void CC2420SpiP$SpiPacket$sendDone(uint8_t *arg_0x41040b98, uint8_t *arg_0x41040d40, uint16_t arg_0x41040ed0, 
error_t arg_0x4103f088);
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   error_t CC2420SpiP$Fifo$continueRead(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x41021010, uint8_t arg_0x41021198);
#line 91
static   void CC2420SpiP$Fifo$default$writeDone(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x4101f838, uint8_t arg_0x4101f9c0, error_t arg_0x4101fb48);
#line 82
static   cc2420_status_t CC2420SpiP$Fifo$write(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x4101f0b0, uint8_t arg_0x4101f238);
#line 51
static   cc2420_status_t CC2420SpiP$Fifo$beginRead(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x41022848, uint8_t arg_0x410229d0);
#line 71
static   void CC2420SpiP$Fifo$default$readDone(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x410217c8, uint8_t arg_0x41021950, error_t arg_0x41021ad8);
# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static   void CC2420SpiP$ChipSpiResource$abortRelease(void);







static   error_t CC2420SpiP$ChipSpiResource$attemptRelease(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420SpiP$SpiResource$granted(void);
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static   cc2420_status_t CC2420SpiP$Ram$write(
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x4104cd70, 
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t arg_0x40f535f0, uint8_t *arg_0x40f53798, uint8_t arg_0x40f53920);
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420SpiP$Reg$read(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104b448, 
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *arg_0x40f4fa50);







static   cc2420_status_t CC2420SpiP$Reg$write(
# 46 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104b448, 
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t arg_0x40f4e010);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420SpiP$Resource$release(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420SpiP$Resource$immediateRequest(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420SpiP$Resource$request(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420SpiP$Resource$default$granted(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool CC2420SpiP$Resource$isOwner(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void CC2420SpiP$grant$runTask(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420SpiP$Strobe$strobe(
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104bb08);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t StateImplP$Init$init(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   void StateImplP$State$toIdle(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x4107af20);
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   bool StateImplP$State$isState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x4107af20, 
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t arg_0x40f26368);
#line 61
static   bool StateImplP$State$isIdle(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x4107af20);
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   error_t StateImplP$State$requestState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x4107af20, 
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t arg_0x40f27230);





static   void StateImplP$State$forceState(
# 67 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x4107af20, 
# 51 "/opt/tinyos-2.x/tos/interfaces/State.nc"
uint8_t arg_0x40f277b8);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$unconfigure(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e2d08);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$configure(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e2d08);
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$send(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1740, 
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t *arg_0x41040118, uint8_t *arg_0x410402c0, uint16_t arg_0x41040450);
#line 71
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$default$sendDone(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1740, 
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t *arg_0x41040b98, uint8_t *arg_0x41040d40, uint16_t arg_0x41040ed0, 
error_t arg_0x4103f088);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static   msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$default$getConfig(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e07a0);
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
static   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiByte$write(uint8_t arg_0x41045678);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$immediateRequest(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$granted(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$immediateRequest(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$default$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$isOwner(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$rxDone(uint8_t arg_0x410a88e8);
#line 49
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$txDone(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$runTask(void);
# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static   void HplMsp430Usart0P$Usart$enableRxIntr(void);
#line 197
static   void HplMsp430Usart0P$Usart$clrRxIntr(void);
#line 97
static   void HplMsp430Usart0P$Usart$resetUsart(bool arg_0x410d7c08);
#line 179
static   void HplMsp430Usart0P$Usart$disableIntr(void);
#line 90
static   void HplMsp430Usart0P$Usart$setUmctl(uint8_t arg_0x410d73e0);
#line 177
static   void HplMsp430Usart0P$Usart$disableRxIntr(void);









static   bool HplMsp430Usart0P$Usart$isTxIntrPending(void);
#line 207
static   void HplMsp430Usart0P$Usart$clrIntr(void);
#line 80
static   void HplMsp430Usart0P$Usart$setUbr(uint16_t arg_0x410d8b88);
#line 224
static   void HplMsp430Usart0P$Usart$tx(uint8_t arg_0x410ce010);
#line 128
static   void HplMsp430Usart0P$Usart$disableUart(void);
#line 153
static   void HplMsp430Usart0P$Usart$enableSpi(void);
#line 168
static   void HplMsp430Usart0P$Usart$setModeSpi(msp430_spi_union_config_t *arg_0x410d3d50);
#line 231
static   uint8_t HplMsp430Usart0P$Usart$rx(void);
#line 192
static   bool HplMsp430Usart0P$Usart$isRxIntrPending(void);









static   void HplMsp430Usart0P$Usart$clrTxIntr(void);
#line 158
static   void HplMsp430Usart0P$Usart$disableSpi(void);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41197908, 
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t arg_0x410a88e8);
#line 49
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41197908);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawI2CInterrupts$fired(void);
#line 39
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$default$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41196010);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t arg_0x410a88e8);
#line 49
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void);








static   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t arg_0x40b83e20);







static   resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$immediateRequested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$release(void);
#line 73
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$requested(void);
#line 46
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$granted(void);
#line 81
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$immediateRequested(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$release(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$immediateRequest(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$request(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$default$granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$isOwner(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$inUse(void);







static   uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$userId(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$runTask(void);
# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static   void HplMsp430I2C0P$HplI2C$clearModeI2C(void);
#line 6
static   bool HplMsp430I2C0P$HplI2C$isI2C(void);
# 44 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static   am_addr_t ActiveMessageAddressC$amAddress(void);
# 48 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static   am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void);




static   am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void);
# 66 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static   void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t arg_0x40e9e688);
#line 60
static   void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t arg_0x40e9e108);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static   void CC2420TransmitP$CaptureSFD$captured(uint16_t arg_0x41011ea0);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void CC2420TransmitP$BackoffTimer$fired(void);
# 61 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static   void CC2420TransmitP$CC2420Receive$receive(uint8_t arg_0x41254b18, message_t *arg_0x41254cc8);
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static   error_t CC2420TransmitP$Send$send(message_t *arg_0x40efd620, bool arg_0x40efd7a8);
# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static   void CC2420TransmitP$ChipSpiResource$releasing(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t CC2420TransmitP$Init$init(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420TransmitP$SpiResource$granted(void);
# 39 "/opt/tinyos-2.x/tos/interfaces/RadioTimeStamping.nc"
static   void CC2420TransmitP$TimeStamp$default$transmittedSFD(uint16_t arg_0x4122b388, message_t *arg_0x4122b538);










static   void CC2420TransmitP$TimeStamp$default$receivedSFD(uint16_t arg_0x4122ba68);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static  error_t CC2420TransmitP$StdControl$start(void);









static  error_t CC2420TransmitP$StdControl$stop(void);
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   void CC2420TransmitP$TXFIFO$writeDone(uint8_t *arg_0x4101f838, uint8_t arg_0x4101f9c0, error_t arg_0x4101fb48);
#line 71
static   void CC2420TransmitP$TXFIFO$readDone(uint8_t *arg_0x410217c8, uint8_t arg_0x41021950, error_t arg_0x41021ad8);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static  void CC2420ReceiveP$CC2420Config$syncDone(error_t arg_0x40ea9e98);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void CC2420ReceiveP$receiveDone_task$runTask(void);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static   void CC2420ReceiveP$CC2420Receive$sfd_dropped(void);
#line 47
static   void CC2420ReceiveP$CC2420Receive$sfd(uint16_t arg_0x41254220);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t CC2420ReceiveP$Init$init(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420ReceiveP$SpiResource$granted(void);
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   void CC2420ReceiveP$RXFIFO$writeDone(uint8_t *arg_0x4101f838, uint8_t arg_0x4101f9c0, error_t arg_0x4101fb48);
#line 71
static   void CC2420ReceiveP$RXFIFO$readDone(uint8_t *arg_0x410217c8, uint8_t arg_0x41021950, error_t arg_0x41021ad8);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void CC2420ReceiveP$InterruptFIFOP$fired(void);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static  error_t CC2420ReceiveP$StdControl$start(void);









static  error_t CC2420ReceiveP$StdControl$stop(void);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *CC2420PacketC$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);




static   cc2420_metadata_t *CC2420PacketC$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0);
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static   uint16_t RandomMlcgP$Random$rand16(void);
#line 35
static   uint32_t RandomMlcgP$Random$rand32(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t RandomMlcgP$Init$init(void);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  void UniqueSendP$SubSend$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868);
#line 64
static  error_t UniqueSendP$Send$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t UniqueSendP$Init$init(void);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *UniqueReceiveP$SubReceive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t UniqueReceiveP$Init$init(void);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *UniqueReceiveP$DuplicateReceive$default$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t UBee430_APC$SplitControl$start(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t UBee430_APC$AdcZero$read(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void UBee430_APC$Timer0$startPeriodic(uint32_t arg_0x40686030);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t UBee430_APC$rt_Send$postTask(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t UBee430_APC$InternalVolt$read(void);
#line 55
static  error_t UBee430_APC$Photo$read(void);
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static  error_t UBee430_APC$AMSend$send(am_addr_t arg_0x406682c0, message_t *arg_0x40668470, uint8_t arg_0x406685f8);
#line 125
static  void *UBee430_APC$AMSend$getPayload(message_t *arg_0x40665248);
# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
static  uint8_t UBee430_APC$HumidityMetadata$getSignificantBits(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t UBee430_APC$Send$postTask(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t UBee430_APC$Humidity$read(void);
# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
static  uint8_t UBee430_APC$TemperatureMetadata$getSignificantBits(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t UBee430_APC$InternalTemp$read(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static   void UBee430_APC$Leds$led0Toggle(void);
#line 72
static   void UBee430_APC$Leds$led1Toggle(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t UBee430_APC$next$postTask(void);
# 21 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
static  uint8_t UBee430_APC$DS28$ReadID_number4(void);



static  uint8_t UBee430_APC$DS28$ReadRTC_seconds(void);









static  void UBee430_APC$DS28$WriteRTC_dayofweek(uint8_t arg_0x4067b100);
#line 33
static  void UBee430_APC$DS28$WriteRTC_minutes(uint8_t arg_0x4067c7c8);
static  void UBee430_APC$DS28$WriteRTC_hours(uint8_t arg_0x4067cc50);
#line 19
static  uint8_t UBee430_APC$DS28$ReadID_number2(void);
#line 31
static  uint8_t UBee430_APC$DS28$ReadRTC_years(void);
#line 5
static  void UBee430_APC$DS28$init(void);
#line 22
static  uint8_t UBee430_APC$DS28$ReadID_number5(void);
#line 37
static  void UBee430_APC$DS28$WriteRTC_months(uint8_t arg_0x4067bb10);
#line 17
static  uint8_t UBee430_APC$DS28$ReadID_number0(void);









static  uint8_t UBee430_APC$DS28$ReadRTC_hours(void);


static  uint8_t UBee430_APC$DS28$ReadRTC_months(void);

static  void UBee430_APC$DS28$WriteRTC_seconds(uint8_t arg_0x4067c338);
#line 29
static  uint8_t UBee430_APC$DS28$ReadRTC_date(void);
#line 20
static  uint8_t UBee430_APC$DS28$ReadID_number3(void);





static  uint8_t UBee430_APC$DS28$ReadRTC_minutes(void);
#line 38
static  void UBee430_APC$DS28$WriteRTC_years(uint8_t arg_0x4067a010);
#line 36
static  void UBee430_APC$DS28$WriteRTC_date(uint8_t arg_0x4067b688);
#line 18
static  uint8_t UBee430_APC$DS28$ReadID_number1(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t UBee430_APC$Temperature$read(void);
# 45 "UBee430_APC.nc"
enum UBee430_APC$__nesc_unnamed4318 {
#line 45
  UBee430_APC$Send = 0U
};
#line 45
typedef int UBee430_APC$__nesc_sillytask_Send[UBee430_APC$Send];


enum UBee430_APC$__nesc_unnamed4319 {
#line 48
  UBee430_APC$rt_Send = 1U
};
#line 48
typedef int UBee430_APC$__nesc_sillytask_rt_Send[UBee430_APC$rt_Send];
#line 69
enum UBee430_APC$__nesc_unnamed4320 {
#line 69
  UBee430_APC$next = 2U
};
#line 69
typedef int UBee430_APC$__nesc_sillytask_next[UBee430_APC$next];
#line 40
message_t UBee430_APC$packet;
bool UBee430_APC$locked = FALSE;
uint8_t UBee430_APC$count = 0;
msg_t UBee430_APC$message;



return_msg UBee430_APC$rtm;

bool UBee430_APC$boot_tx_flag = 0;

static inline void UBee430_APC$getRTC(void);








static inline void UBee430_APC$getID(void);








static inline  void UBee430_APC$next$runTask(void);










static inline  void UBee430_APC$Send$runTask(void);
#line 100
static inline  void UBee430_APC$Boot$booted(void);
#line 117
static inline void UBee430_APC$rtc_init(void);










static inline  void UBee430_APC$Photo$readDone(error_t result, uint16_t data);







static inline  void UBee430_APC$InternalTemp$readDone(error_t result, uint16_t data);








static inline  void UBee430_APC$InternalVolt$readDone(error_t result, uint16_t data);







static  void UBee430_APC$Humidity$readDone(error_t result, uint16_t data);
#line 167
static  void UBee430_APC$Temperature$readDone(error_t result, uint16_t data);
#line 180
static inline  void UBee430_APC$AdcZero$readDone(error_t result, uint16_t data);








static inline  void UBee430_APC$DS28$EEPROM_WriteDone(void);

static inline  void UBee430_APC$rt_Send$runTask(void);




static inline  message_t *UBee430_APC$Receive$receive(message_t *buf, void *payload, uint8_t len);



static inline  void UBee430_APC$AMSend$sendDone(message_t *buf, error_t error);




static inline  void UBee430_APC$SplitControl$startDone(error_t err);







static inline  void UBee430_APC$SplitControl$stopDone(error_t err);

static inline  void UBee430_APC$Timer0$fired(void);
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void LedsP$Led0$toggle(void);



static   void LedsP$Led0$makeOutput(void);
#line 29
static   void LedsP$Led0$set(void);

static   void LedsP$Led1$toggle(void);



static   void LedsP$Led1$makeOutput(void);
#line 29
static   void LedsP$Led1$set(void);





static   void LedsP$Led2$makeOutput(void);
#line 29
static   void LedsP$Led2$set(void);
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline  error_t LedsP$Init$init(void);
#line 73
static inline   void LedsP$Leds$led0Toggle(void);
#line 88
static inline   void LedsP$Leds$led1Toggle(void);
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$getRaw(void);
static inline   bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$get(void);
#line 48
static inline   uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$getRaw(void);
static inline   bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$get(void);
#line 48
static inline   uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$getRaw(void);
static inline   bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$get(void);
static inline   void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$makeInput(void);
#line 45
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$set(void);
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$clr(void);

static inline   uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$getRaw(void);
static inline   bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$get(void);
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeInput(void);

static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeOutput(void);
#line 45
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$set(void);
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$clr(void);



static inline   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeInput(void);

static inline   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$set(void);
static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$makeOutput(void);

static inline   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectIOFunc(void);
#line 54
static inline   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectIOFunc(void);
#line 54
static inline   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectIOFunc(void);
#line 56
static inline   void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP$20$IO$selectIOFunc(void);
#line 56
static inline   void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP$21$IO$selectIOFunc(void);
#line 48
static inline   uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$getRaw(void);
static inline   bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$get(void);
static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectIOFunc(void);
#line 45
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$set(void);
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$set(void);
static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$makeOutput(void);
#line 45
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$set(void);
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void);

static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void);




static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void);

static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void);




static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void);






static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void);
#line 71
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void);

static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void);



static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle(void);
#line 71
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void);

static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void);



static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void);





static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t PlatformP$MoteInit$init(void);
#line 51
static  error_t PlatformP$MoteClockInit$init(void);
#line 51
static  error_t PlatformP$LedsInit$init(void);
# 10 "/opt/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline  error_t PlatformP$Init$init(void);
# 6 "/opt/tinyos-2.x/tos/platforms/UBee430/MotePlatformC.nc"
static __inline void MotePlatformC$uwait(uint16_t u);




static __inline void MotePlatformC$TOSH_wait(void);




static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void);
#line 56
static inline  error_t MotePlatformC$Init$init(void);
# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static  void Msp430ClockP$Msp430ClockInit$initTimerB(void);
#line 31
static  void Msp430ClockP$Msp430ClockInit$initTimerA(void);
#line 29
static  void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void);
static  void Msp430ClockP$Msp430ClockInit$initClocks(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
 static volatile uint8_t Msp430ClockP$IE1 __asm ("0x0000");
 static volatile uint16_t Msp430ClockP$TA0CTL __asm ("0x0160");
 static volatile uint16_t Msp430ClockP$TA0IV __asm ("0x012E");
 static volatile uint16_t Msp430ClockP$TBCTL __asm ("0x0180");
 static volatile uint16_t Msp430ClockP$TBIV __asm ("0x011E");

enum Msp430ClockP$__nesc_unnamed4321 {

  Msp430ClockP$ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP$TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP$ACLK_CALIB_PERIOD
};


static inline  void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void);
#line 64
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void);
#line 85
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void);
#line 100
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void);
#line 115
static inline   void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void);




static inline   void Msp430ClockP$Msp430ClockInit$default$initClocks(void);




static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerA(void);




static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerB(void);





static inline void Msp430ClockP$startTimerA(void);
#line 148
static inline void Msp430ClockP$startTimerB(void);
#line 160
static void Msp430ClockP$set_dco_calib(int calib);





static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib);
#line 189
static inline void Msp430ClockP$busyCalibrateDco(void);
#line 214
static inline  error_t Msp430ClockP$Init$init(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40875be0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void);
# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode);









static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void);









static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void);





static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void);








static inline    void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40875be0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void);
#line 70
static inline   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void);
#line 115
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void);




static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void);





static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void);








static    void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x);
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void);
#line 89
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x);
#line 169
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x);
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void);
#line 89
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x);
#line 169
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void);
#line 169
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x);
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x);

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void);
#line 119
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t x);
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x);
#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$captureControl(uint8_t l_cm);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void);









static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$clearPendingInterrupt(void);
#line 99
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$setControlAsCapture(uint8_t cm);
#line 119
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$enableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$disableEvents(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void);
#line 164
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$clearOverflow(void);




static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void);
#line 181
static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t x);
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x);

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$compareControl(void);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void);









static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void);









static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$setControlAsCompare(void);
#line 119
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t x);









static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t x);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t arg_0x40872358);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void Msp430TimerCommonP$VectorTimerB1$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerA0$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerA1$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerB0$fired(void);
# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(12))) ;
void sig_TIMERA1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(10))) ;
void sig_TIMERB0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(26))) ;
void sig_TIMERB1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(24))) ;
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t RealMainP$SoftwareInit$init(void);
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static  void RealMainP$Boot$booted(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t RealMainP$PlatformInit$init(void);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static  void RealMainP$Scheduler$init(void);
#line 61
static  void RealMainP$Scheduler$taskLoop(void);
#line 54
static  bool RealMainP$Scheduler$runNextTask(void);
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
int main(void)   ;
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405d4868);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static   void SchedulerBasicP$McuSleep$sleep(void);
# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4322 {

  SchedulerBasicP$NUM_TASKS = 25U, 
  SchedulerBasicP$NO_TASK = 255
};

volatile uint8_t SchedulerBasicP$m_head;
volatile uint8_t SchedulerBasicP$m_tail;
volatile uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void);
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline  void SchedulerBasicP$Scheduler$init(void);









static  bool SchedulerBasicP$Scheduler$runNextTask(void);
#line 138
static inline  void SchedulerBasicP$Scheduler$taskLoop(void);
#line 159
static   error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static   void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static   mcu_power_t McuSleepC$McuPowerOverride$lowestState(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC$dirty = TRUE;
mcu_power_t McuSleepC$powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC$msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC$getPowerState(void);
#line 104
static inline void McuSleepC$computePowerState(void);




static inline   void McuSleepC$McuSleep$sleep(void);
#line 124
static inline    mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t arg_0x4085fed0);

static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t arg_0x4085e868);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void);
#line 36
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void);



static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void);
#line 33
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void);
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void);
#line 54
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void);




static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void);










static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void);
static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void);
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void);




static inline   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void);









static inline   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void);






static   bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void);










static   void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void);
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC$0$__nesc_unnamed4323 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static   /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void);
#line 122
static inline   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void);
#line 92
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x4092b728);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void);
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$__nesc_unnamed4324 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void);




static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void);










static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void);




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void);
#line 136
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt);
#line 151
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void);
#line 166
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void);
#line 92
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x4092b728);
#line 105
static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void);
#line 62
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void);
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4325 {
#line 63
  AlarmToTimerC$0$fired = 3U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void);


static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void);






static inline   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void);
#line 82
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void);
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void);
#line 118
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t arg_0x40684258, uint32_t arg_0x406843e8);
#line 67
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void);




static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409e58a8);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4326 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 4U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4327 {

  VirtualizeTimerC$0$NUM_TIMERS = 6U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4328 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 88
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void);
#line 127
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);




static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);




static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
#line 192
static inline   void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline   void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void);
# 76 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidity(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$readDone(error_t arg_0x40624580, /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$val_t arg_0x40624708);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$release(void);
#line 78
static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$request(void);
# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperature(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$release(void);
#line 78
static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$request(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$readDone(error_t arg_0x40624580, /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$val_t arg_0x40624708);
# 58 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TemperatureMetadata$getSignificantBits(void);

static inline  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$read(void);




static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$granted(void);







static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperatureDone(error_t result, uint16_t val);




static inline  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumidityMetadata$getSignificantBits(void);

static inline  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$read(void);




static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$granted(void);







static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidityDone(error_t result, uint16_t val);




static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$resetDone(error_t result);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureHumidityDone(error_t result, uint16_t val);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$readStatusRegDone(error_t result, uint8_t val);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$writeStatusRegDone(error_t result);

static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$resetDone(error_t result);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureTemperatureDone(error_t result, uint16_t val);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$readStatusRegDone(error_t result, uint8_t val);
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$writeStatusRegDone(error_t result);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$disable(void);
#line 43
static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$enableFallingEdge(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$postTask(void);
#line 56
static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$postTask(void);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$makeOutput(void);
#line 29
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set(void);
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr(void);
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidityDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3c088, uint16_t arg_0x40a3c218);
#line 116
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$writeStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3b7d8);
#line 100
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$readStatusRegDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3cb70, uint8_t arg_0x40a3ccf8);
#line 54
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$resetDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3fd10);
#line 69
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperatureDone(
# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x40a5fc58, 
# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t arg_0x40a3e5e0, uint16_t arg_0x40a3e770);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput(void);
#line 32
static   bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$get(void);


static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput(void);
#line 29
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set(void);
static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$clr(void);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(uint32_t arg_0x40686600);




static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$stop(void);
# 102 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_unnamed4329 {
#line 102
  SensirionSht11LogicP$0$readSensor = 5U
};
#line 102
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_sillytask_readSensor[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor];
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_unnamed4330 {
#line 103
  SensirionSht11LogicP$0$signalStatusDone = 6U
};
#line 103
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_sillytask_signalStatusDone[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone];
#line 72
#line 66
typedef enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_unnamed4331 {
  SensirionSht11LogicP$0$CMD_MEASURE_TEMPERATURE = 0x3, 
  SensirionSht11LogicP$0$CMD_MEASURE_HUMIDITY = 0x5, 
  SensirionSht11LogicP$0$CMD_READ_STATUS = 0x7, 
  SensirionSht11LogicP$0$CMD_WRITE_STATUS = 0x6, 
  SensirionSht11LogicP$0$CMD_SOFT_RESET = 0x1E
} /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sht_cmd_t;

enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$__nesc_unnamed4332 {
  SensirionSht11LogicP$0$TIMEOUT_RESET = 11, 
  SensirionSht11LogicP$0$TIMEOUT_14BIT = 250, 
  SensirionSht11LogicP$0$TIMEOUT_12BIT = 250, 
  SensirionSht11LogicP$0$TIMEOUT_8BIT = 250
};

bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$on = TRUE;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status = 0;
/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sht_cmd_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$newStatus;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeFail = FALSE;

uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient;

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$performCommand(void);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$initPins(void);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$resetDevice(void);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$transmissionStart(void);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sendCommand(uint8_t _cmd);
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeByte(uint8_t byte);
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$waitForResponse(void);
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$enableInterrupt(void);
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte(void);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$ack(void);
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$endTransmission(void);
#line 113
static inline  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperature(uint8_t client);







static inline  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidity(uint8_t client);
#line 149
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$performCommand(void);
#line 220
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$initPins(void);







static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$resetDevice(void);










static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$transmissionStart(void);
#line 251
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sendCommand(uint8_t _cmd);



static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeByte(uint8_t byte);
#line 268
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$waitForResponse(void);
#line 281
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$enableInterrupt(void);





static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$fired(void);
#line 315
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$fired(void);




static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$runTask(void);
#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte(void);
#line 372
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$ack(void);








static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$endTransmission(void);






static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$runTask(void);
#line 406
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$resetDone(uint8_t client, error_t result);
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureTemperatureDone(uint8_t client, error_t result, uint16_t val);
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureHumidityDone(uint8_t client, error_t result, uint16_t val);
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$readStatusRegDone(uint8_t client, error_t result, uint8_t val);
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$writeStatusRegDone(uint8_t client, error_t result);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeInput(void);






static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeOutput(void);
#line 59
static   bool /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$get(void);
#line 34
static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$set(void);




static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$set(void);
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$clr(void);

static inline   bool /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$get(void);
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeInput(void);

static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeOutput(void);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeInput(void);






static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeOutput(void);
#line 34
static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$set(void);




static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$set(void);
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$clr(void);


static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeInput(void);

static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeOutput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$makeOutput(void);
#line 34
static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$set(void);




static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$set(void);
static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$clr(void);




static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$makeOutput(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void HplSensirionSht11P$SplitControl$startDone(error_t arg_0x4062baf0);
#line 117
static  void HplSensirionSht11P$SplitControl$stopDone(error_t arg_0x4062a6e8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t HplSensirionSht11P$stopTask$postTask(void);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void HplSensirionSht11P$SCK$makeInput(void);
#line 30
static   void HplSensirionSht11P$SCK$clr(void);




static   void HplSensirionSht11P$PWR$makeOutput(void);
#line 29
static   void HplSensirionSht11P$PWR$set(void);
static   void HplSensirionSht11P$PWR$clr(void);


static   void HplSensirionSht11P$DATA$makeInput(void);
#line 30
static   void HplSensirionSht11P$DATA$clr(void);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void HplSensirionSht11P$Timer$startOneShot(uint32_t arg_0x40686600);
# 50 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
enum HplSensirionSht11P$__nesc_unnamed4333 {
#line 50
  HplSensirionSht11P$stopTask = 7U
};
#line 50
typedef int HplSensirionSht11P$__nesc_sillytask_stopTask[HplSensirionSht11P$stopTask];

static  error_t HplSensirionSht11P$SplitControl$start(void);






static inline  void HplSensirionSht11P$Timer$fired(void);



static inline  error_t HplSensirionSht11P$SplitControl$stop(void);









static inline  void HplSensirionSht11P$stopTask$runTask(void);
# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void HplMsp430InterruptP$Port14$fired(void);
#line 61
static   void HplMsp430InterruptP$Port26$fired(void);
#line 61
static   void HplMsp430InterruptP$Port17$fired(void);
#line 61
static   void HplMsp430InterruptP$Port21$fired(void);
#line 61
static   void HplMsp430InterruptP$Port12$fired(void);
#line 61
static   void HplMsp430InterruptP$Port24$fired(void);
#line 61
static   void HplMsp430InterruptP$Port15$fired(void);
#line 61
static   void HplMsp430InterruptP$Port27$fired(void);
#line 61
static   void HplMsp430InterruptP$Port10$fired(void);
#line 61
static   void HplMsp430InterruptP$Port22$fired(void);
#line 61
static   void HplMsp430InterruptP$Port13$fired(void);
#line 61
static   void HplMsp430InterruptP$Port25$fired(void);
#line 61
static   void HplMsp430InterruptP$Port16$fired(void);
#line 61
static   void HplMsp430InterruptP$Port20$fired(void);
#line 61
static   void HplMsp430InterruptP$Port11$fired(void);
#line 61
static   void HplMsp430InterruptP$Port23$fired(void);
# 53 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(8))) ;
#line 68
static inline    void HplMsp430InterruptP$Port11$default$fired(void);
static inline    void HplMsp430InterruptP$Port12$default$fired(void);
static inline    void HplMsp430InterruptP$Port13$default$fired(void);


static inline    void HplMsp430InterruptP$Port16$default$fired(void);
static inline    void HplMsp430InterruptP$Port17$default$fired(void);
static inline   void HplMsp430InterruptP$Port10$enable(void);



static inline   void HplMsp430InterruptP$Port14$enable(void);
static inline   void HplMsp430InterruptP$Port15$enable(void);


static inline   void HplMsp430InterruptP$Port10$disable(void);



static inline   void HplMsp430InterruptP$Port14$disable(void);
static inline   void HplMsp430InterruptP$Port15$disable(void);


static inline   void HplMsp430InterruptP$Port10$clear(void);
static inline   void HplMsp430InterruptP$Port11$clear(void);
static inline   void HplMsp430InterruptP$Port12$clear(void);
static inline   void HplMsp430InterruptP$Port13$clear(void);
static inline   void HplMsp430InterruptP$Port14$clear(void);
static inline   void HplMsp430InterruptP$Port15$clear(void);
static inline   void HplMsp430InterruptP$Port16$clear(void);
static inline   void HplMsp430InterruptP$Port17$clear(void);








static inline   void HplMsp430InterruptP$Port10$edge(bool l2h);
#line 131
static inline   void HplMsp430InterruptP$Port14$edge(bool l2h);





static inline   void HplMsp430InterruptP$Port15$edge(bool l2h);
#line 158
void sig_PORT2_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(2))) ;
#line 171
static inline    void HplMsp430InterruptP$Port20$default$fired(void);
static inline    void HplMsp430InterruptP$Port21$default$fired(void);
static inline    void HplMsp430InterruptP$Port22$default$fired(void);
static inline    void HplMsp430InterruptP$Port23$default$fired(void);
static inline    void HplMsp430InterruptP$Port24$default$fired(void);
static inline    void HplMsp430InterruptP$Port25$default$fired(void);
static inline    void HplMsp430InterruptP$Port26$default$fired(void);
static inline    void HplMsp430InterruptP$Port27$default$fired(void);
#line 195
static inline   void HplMsp430InterruptP$Port20$clear(void);
static inline   void HplMsp430InterruptP$Port21$clear(void);
static inline   void HplMsp430InterruptP$Port22$clear(void);
static inline   void HplMsp430InterruptP$Port23$clear(void);
static inline   void HplMsp430InterruptP$Port24$clear(void);
static inline   void HplMsp430InterruptP$Port25$clear(void);
static inline   void HplMsp430InterruptP$Port26$clear(void);
static inline   void HplMsp430InterruptP$Port27$clear(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$clear(void);
#line 36
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$disable(void);
#line 56
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$edge(bool arg_0x40aceef8);
#line 31
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$enable(void);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$fired(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$enable(bool rising);
#line 54
static inline   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$enableFallingEdge(void);



static   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$disable(void);







static inline   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$fired(void);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$__nesc_unnamed4334 {
#line 39
  FcfsResourceQueueC$0$NO_ENTRY = 0xFF
};
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[2U];
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

static inline  error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void);




static inline   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void);



static inline   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id);



static inline   resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void);
#line 72
static inline   error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$isEmpty(void);
#line 60
static   resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$dequeue(void);
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$requested(void);
#line 46
static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$postTask(void);
# 74 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$__nesc_unnamed4335 {
#line 74
  ArbiterP$0$grantedTask = 8U
};
#line 74
typedef int /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$__nesc_sillytask_grantedTask[/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask];
#line 67
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$__nesc_unnamed4336 {
#line 67
  ArbiterP$0$RES_CONTROLLED, ArbiterP$0$RES_GRANTING, ArbiterP$0$RES_IMM_GRANTING, ArbiterP$0$RES_BUSY
};
#line 68
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$__nesc_unnamed4337 {
#line 68
  ArbiterP$0$default_owner_id = 2U
};
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_CONTROLLED;
 uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$default_owner_id;
 uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$reqResId;



static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$request(uint8_t id);
#line 107
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$release(uint8_t id);
#line 126
static inline   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void);
#line 173
static inline  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$runTask(void);
#line 185
static inline   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$default$granted(uint8_t id);

static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$default$requested(uint8_t id);
#line 199
static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$start(void);
#line 109
static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stop(void);
# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$cleanup(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$postTask(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$release(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$postTask(void);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$start(void);









static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$stop(void);
# 63 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$__nesc_unnamed4338 {
#line 63
  PowerManagerP$0$startTask = 9U
};
#line 63
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$__nesc_sillytask_startTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask];




enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$__nesc_unnamed4339 {
#line 68
  PowerManagerP$0$stopTask = 10U
};
#line 68
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$__nesc_sillytask_stopTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask];
#line 60
 bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopping = FALSE;
 bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$requested = FALSE;

static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$runTask(void);




static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$runTask(void);





static inline   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$requested(void);









static inline   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$start(void);







static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$startDone(error_t error);



static inline   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$granted(void);




static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stopDone(error_t error);










static inline   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$stop(void);







static inline    void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$default$cleanup(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$getConfiguration(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31430);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$getData(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00);
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$configureMultiple(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c31f00, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2b848, uint16_t arg_0x40c2b9f8[], uint16_t arg_0x40c2bb90, uint16_t arg_0x40c2bd20);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceReadStream$release(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c25a58);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void AdcP$Read$readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0fb48, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t arg_0x40624580, AdcP$Read$val_t arg_0x40624708);
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static   void AdcP$ReadNow$readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0e9a0, 
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t arg_0x40bfdd98, AdcP$ReadNow$val_t arg_0x40bfdf20);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceReadNow$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0c7b0);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$Config$adc_config_t AdcP$Config$getConfiguration(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c33bb8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t AdcP$finishStreamRequest$postTask(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$getData(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$configureSingle(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c26748, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x40c2c2c0);
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static  void AdcP$ReadStream$bufferDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0b1d8, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x40c04638, 
AdcP$ReadStream$val_t *arg_0x40c047f0, uint16_t arg_0x40c04980);
#line 102
static  void AdcP$ReadStream$readDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0b1d8, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x40c03010, uint32_t arg_0x40c031a8);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool AdcP$ResourceRead$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40c0a7c8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t AdcP$signalBufferDone$postTask(void);
#line 56
static   error_t AdcP$readDone$postTask(void);
# 83 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP$__nesc_unnamed4340 {
#line 83
  AdcP$finishStreamRequest = 11U
};
#line 83
typedef int AdcP$__nesc_sillytask_finishStreamRequest[AdcP$finishStreamRequest];
enum AdcP$__nesc_unnamed4341 {
#line 84
  AdcP$signalBufferDone = 12U
};
#line 84
typedef int AdcP$__nesc_sillytask_signalBufferDone[AdcP$signalBufferDone];
#line 161
enum AdcP$__nesc_unnamed4342 {
#line 161
  AdcP$readDone = 13U
};
#line 161
typedef int AdcP$__nesc_sillytask_readDone[AdcP$readDone];
#line 60
enum AdcP$__nesc_unnamed4343 {
  AdcP$STATE_READ, 
  AdcP$STATE_READNOW, 
  AdcP$STATE_READNOW_INVALID_CONFIG, 
  AdcP$STATE_READSTREAM
};

struct AdcP$stream_entry_t {
  uint16_t count;
  struct AdcP$stream_entry_t *next;
};


 uint8_t AdcP$state;
 uint8_t AdcP$owner;
 uint16_t AdcP$value;
 uint16_t *AdcP$resultBuf;


 struct AdcP$stream_entry_t *AdcP$streamBuf[2U];
 uint32_t AdcP$usPeriod[2U];
msp430adc12_channel_config_t AdcP$streamConfig;





static error_t AdcP$configure(uint8_t client);









static  error_t AdcP$Read$read(uint8_t client);






static  void AdcP$ResourceRead$granted(uint8_t client);
#line 123
static inline  void AdcP$SubResourceReadNow$granted(uint8_t nowClient);
#line 161
static inline void  AdcP$readDone$runTask(void);





static   error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data);
#line 186
static inline   uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t length);
#line 222
static inline void  AdcP$finishStreamRequest$runTask(void);
#line 241
static  void AdcP$ResourceReadStream$granted(uint8_t streamClient);
#line 278
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 312
static inline void  AdcP$signalBufferDone$runTask(void);





static inline   error_t AdcP$SingleChannelReadStream$singleDataReady(uint8_t streamClient, uint16_t data);





static inline    error_t AdcP$ResourceRead$default$request(uint8_t client);

static inline    error_t AdcP$ResourceRead$default$release(uint8_t client);
static inline    bool AdcP$ResourceRead$default$isOwner(uint8_t client);
static   void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val);




static inline   void AdcP$ResourceReadNow$default$granted(uint8_t nowClient);
static inline    void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val);






static inline    error_t AdcP$ResourceReadStream$default$release(uint8_t streamClient);

static inline   void AdcP$ReadStream$default$bufferDone(uint8_t streamClient, error_t result, 
uint16_t *buf, uint16_t count);
static inline   void AdcP$ReadStream$default$readDone(uint8_t streamClient, error_t result, uint32_t actualPeriod);

static inline    error_t AdcP$SingleChannel$default$getData(uint8_t client);





const msp430adc12_channel_config_t AdcP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline    const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client);




static inline    const msp430adc12_channel_config_t *
AdcP$ConfigReadStream$default$getConfiguration(uint8_t client);




static inline    error_t AdcP$SingleChannelReadStream$default$configureMultiple(uint8_t client, 
const msp430adc12_channel_config_t *config, uint16_t buffer[], 
uint16_t numSamples, uint16_t jiffies);




static inline    error_t AdcP$SingleChannelReadStream$default$getData(uint8_t client);




static inline    error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static   void Msp430Adc12ImplP$MultiChannel$dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96068, 
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *arg_0x40c82e90, uint16_t arg_0x40c80068);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void);
#line 82
static   adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t arg_0x40c8d928);
#line 106
static   void Msp430Adc12ImplP$HplAdc12$resetIFGs(void);
#line 75
static   void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t arg_0x40c8d1d8, adc12memctl_t arg_0x40c8d370);
#line 128
static   void Msp430Adc12ImplP$HplAdc12$startConversion(void);
#line 51
static   void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t arg_0x40c8e010);
#line 89
static   uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t arg_0x40c8ded8);





static   void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t arg_0x40c8c488);
#line 123
static   void Msp430Adc12ImplP$HplAdc12$stopConversion(void);









static   void Msp430Adc12ImplP$HplAdc12$enableConversion(void);
#line 57
static   void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t arg_0x40c8e550);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port64$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port64$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port64$selectModuleFunc(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t arg_0x4085fed0);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t arg_0x40865180);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port62$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port62$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port62$selectModuleFunc(void);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96818);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c96818);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port67$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port67$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port67$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port60$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port60$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port60$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port65$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port65$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port65$selectModuleFunc(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void Msp430Adc12ImplP$TimerA$clear(void);


static   void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t arg_0x40850ca0);
#line 43
static   void Msp430Adc12ImplP$TimerA$disableEvents(void);
#line 39
static   void Msp430Adc12ImplP$TimerA$setMode(int arg_0x40851b08);





static   void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t arg_0x4084f178);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t arg_0x40865180);
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   uint16_t *Msp430Adc12ImplP$SingleChannel$multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27a00[], uint16_t arg_0x40c27b98);
#line 206
static   error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40c97518, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x40c27250);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port63$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port63$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port63$selectModuleFunc(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t arg_0x4085fed0);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port61$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port61$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port61$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port66$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port66$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port66$selectModuleFunc(void);
# 66 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
enum Msp430Adc12ImplP$__nesc_unnamed4344 {
  Msp430Adc12ImplP$SINGLE_DATA = 1, 
  Msp430Adc12ImplP$SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP$MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP$MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP$CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP$ADC_BUSY = 32, 
  Msp430Adc12ImplP$USE_TIMERA = 64, 
  Msp430Adc12ImplP$ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP$state;

uint16_t *Msp430Adc12ImplP$resultBuffer;
uint16_t Msp430Adc12ImplP$resultBufferLength;
uint16_t Msp430Adc12ImplP$resultBufferIndex;
uint8_t Msp430Adc12ImplP$numChannels;
uint8_t Msp430Adc12ImplP$clientID;

static inline  error_t Msp430Adc12ImplP$Init$init(void);





static inline void Msp430Adc12ImplP$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 111
static inline void Msp430Adc12ImplP$startTimerA(void);
#line 132
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch);
#line 149
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch);
#line 166
static   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 257
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 370
static   error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id);
#line 475
static void Msp430Adc12ImplP$stopConversion(void);
#line 516
static inline   void Msp430Adc12ImplP$TimerA$overflow(void);
static inline   void Msp430Adc12ImplP$CompareA0$fired(void);
static inline   void Msp430Adc12ImplP$CompareA1$fired(void);

static inline   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv);
#line 603
static    error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data);




static    uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t length);




static inline    void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline    void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id);
static inline    void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void HplAdc12P$HplAdc12$conversionDone(uint16_t arg_0x40cb7120);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
 static volatile uint16_t HplAdc12P$ADC12CTL0 __asm ("0x01A0");
 static volatile uint16_t HplAdc12P$ADC12CTL1 __asm ("0x01A2");
 static volatile uint16_t HplAdc12P$ADC12IFG __asm ("0x01A4");
 static volatile uint16_t HplAdc12P$ADC12IE __asm ("0x01A6");
 static volatile uint16_t HplAdc12P$ADC12IV __asm ("0x01A8");

static inline   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0);



static inline   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1);



static inline   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void);







static inline   void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl);





static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i);







static inline   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i);



static inline   void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask);


static inline   void HplAdc12P$HplAdc12$resetIFGs(void);
#line 106
static inline   void HplAdc12P$HplAdc12$startConversion(void);




static inline   void HplAdc12P$HplAdc12$stopConversion(void);




static inline   void HplAdc12P$HplAdc12$enableConversion(void);



static inline   bool HplAdc12P$HplAdc12$isBusy(void);

void sig_ADC_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(14))) ;
# 39 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$__nesc_unnamed4345 {
  RoundRobinResourceQueueC$0$NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC$0$SIZE = 7U ? (7U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id);



static inline  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void);




static inline   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void);








static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id);



static inline   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void);
#line 87
static inline   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d399c8);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d374d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d374d8);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void);
#line 60
static   resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40d39088);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void);
# 69 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4346 {
#line 69
  SimpleArbiterP$0$grantedTask = 14U
};
#line 69
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask];
#line 62
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4347 {
#line 62
  SimpleArbiterP$0$RES_IDLE = 0, SimpleArbiterP$0$RES_GRANTING = 1, SimpleArbiterP$0$RES_BUSY = 2
};
#line 63
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4348 {
#line 63
  SimpleArbiterP$0$NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
 uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
 uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;



static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id);
#line 97
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id);
#line 136
static inline   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void);






static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id);






static inline  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void);









static inline   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id);

static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id);



static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void);
#line 118
static   bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void);
#line 51
static   void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t arg_0x40c8e010);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t arg_0x40686600);




static  void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t arg_0x4062baf0);
#line 117
static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t arg_0x4062a6e8);
#line 92
static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t arg_0x4062baf0);
#line 117
static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t arg_0x4062a6e8);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t arg_0x40686600);




static  void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
enum Msp430RefVoltGeneratorP$__nesc_unnamed4349 {

  Msp430RefVoltGeneratorP$GENERATOR_OFF, 
  Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING, 
  Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING, 
  Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE, 
  Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE
};

uint8_t Msp430RefVoltGeneratorP$state;

static error_t Msp430RefVoltGeneratorP$switchOn(uint8_t level);
#line 78
static error_t Msp430RefVoltGeneratorP$switchOff(void);
#line 94
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void);
#line 127
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void);
#line 157
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void);
#line 220
static inline  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void);
#line 244
static inline  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void);
#line 274
static inline   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d75d80);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$AdcResource$isOwner(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d76720);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$ClientResource$granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40d77d50);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void);
#line 109
static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4350 {
#line 51
  Msp430RefVoltArbiterImplP$switchOff = 15U
};
#line 51
typedef int Msp430RefVoltArbiterImplP$__nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP$switchOff];
#line 46
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4351 {
  Msp430RefVoltArbiterImplP$NO_OWNER = 0xFF
};
 uint8_t Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;



static inline   error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client);
#line 70
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client);
#line 98
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error);








static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error);








static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client);
#line 136
static inline  void Msp430RefVoltArbiterImplP$switchOff$runTask(void);










static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error);



static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error);



static inline   uint8_t Msp430RefVoltArbiterImplP$ClientResource$isOwner(uint8_t client);




static   void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client);
static    error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client);







static    bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(uint8_t client);
static    error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline    const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc"
const msp430adc12_channel_config_t Msp430InternalVoltageP$config = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline   const msp430adc12_channel_config_t *Msp430InternalVoltageP$AdcConfigure$getConfiguration(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$getConfiguration(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$getConfiguration(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$getConfiguration(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalTemperatureP.nc"
const msp430adc12_channel_config_t Msp430InternalTemperatureP$config = { 
.inch = TEMPERATURE_DIODE_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline   const msp430adc12_channel_config_t *Msp430InternalTemperatureP$AdcConfigure$getConfiguration(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$adc_config_t /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$getConfiguration(void);
# 8 "/opt/tinyos-2.x/tos/platforms/UBee430/LightToVoltageP.nc"
const msp430adc12_channel_config_t LightToVoltageP$config = { 
.inch = INPUT_CHANNEL_A5, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_2_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline   const msp430adc12_channel_config_t *LightToVoltageP$AdcConfigure$getConfiguration(void);
# 12 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
static  void Ds28dg02P$Ds28dg02s$EEPROM_WriteDone(void);
# 22 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static void Ds28dg02P$SPI_WRITE(uint8_t w_instruction, uint8_t addr, uint8_t data);
static void Ds28dg02P$SPI_INSTRUCTION_SET(uint8_t instruction);
static inline void Ds28dg02P$DS28DG02_SEL(void);
static inline void Ds28dg02P$DS28DG02_DSEL(void);
static inline void Ds28dg02P$delay(uint8_t data);
static void Ds28dg02P$spi_clk(void);

static inline  void Ds28dg02P$Ds28dg02s$init(void);
#line 60
static inline void Ds28dg02P$DS28DG02_SEL(void);


static inline void Ds28dg02P$DS28DG02_DSEL(void);


static inline void Ds28dg02P$delay(uint8_t data);



static void Ds28dg02P$spi_clk(void);








static void Ds28dg02P$SPI_INSTRUCTION_SET(uint8_t instruction);
#line 98
static void Ds28dg02P$SPI_WRITE(uint8_t w_instruction, uint8_t addr, uint8_t data);
#line 135
static uint8_t Ds28dg02P$SPI_READ(uint8_t r_instruction, uint8_t addr);
#line 215
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number5(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number4(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number3(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number2(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number1(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number0(void);
#line 252
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_seconds(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_minutes(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_hours(void);









static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_date(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_months(void);




static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_years(void);




static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_seconds(uint8_t seconds);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_minutes(uint8_t minutes);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_hours(uint8_t hours);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_dayofweek(uint8_t dayofweek);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_date(uint8_t date);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_months(uint8_t months);



static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_years(uint8_t years);



static inline  void Ds28dg02P$Timer0$fired(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$adc_config_t /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$getConfiguration(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$getConfiguration(void);
# 8 "/opt/tinyos-2.x/tos/platforms/UBee430/AdcZeroP.nc"
const msp430adc12_channel_config_t AdcZeroP$config = { 
.inch = INPUT_CHANNEL_A0, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline   const msp430adc12_channel_config_t *AdcZeroP$AdcConfigure$getConfiguration(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  error_t CC2420ActiveMessageP$SubSend$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790);
# 70 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static   uint16_t CC2420ActiveMessageP$CC2420Config$getPanAddr(void);
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
static  void CC2420ActiveMessageP$AMSend$sendDone(
# 39 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebd188, 
# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t *arg_0x406665f8, error_t arg_0x40666780);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ActiveMessageP$Snoop$receive(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebc320, 
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *CC2420ActiveMessageP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ActiveMessageP$Receive$receive(
# 40 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x40ebdab8, 
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 48 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static   am_addr_t CC2420ActiveMessageP$ActiveMessageAddress$amAddress(void);
# 56 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
enum CC2420ActiveMessageP$__nesc_unnamed4352 {
  CC2420ActiveMessageP$CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};


static  error_t CC2420ActiveMessageP$AMSend$send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 80
static inline  void *CC2420ActiveMessageP$AMSend$getPayload(am_id_t id, message_t *m);
#line 103
static inline  am_addr_t CC2420ActiveMessageP$AMPacket$address(void);



static inline  am_addr_t CC2420ActiveMessageP$AMPacket$destination(message_t *amsg);
#line 127
static inline  bool CC2420ActiveMessageP$AMPacket$isForMe(message_t *amsg);




static  am_id_t CC2420ActiveMessageP$AMPacket$type(message_t *amsg);
#line 160
static inline  uint8_t CC2420ActiveMessageP$Packet$payloadLength(message_t *msg);
#line 172
static  void *CC2420ActiveMessageP$Packet$getPayload(message_t *msg, uint8_t *len);








static inline  void CC2420ActiveMessageP$SubSend$sendDone(message_t *msg, error_t result);





static inline  message_t *CC2420ActiveMessageP$SubReceive$receive(message_t *msg, void *payload, uint8_t len);
#line 202
static inline  void CC2420ActiveMessageP$CC2420Config$syncDone(error_t error);



static inline   message_t *CC2420ActiveMessageP$Receive$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline   message_t *CC2420ActiveMessageP$Snoop$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline   void CC2420ActiveMessageP$AMSend$default$sendDone(uint8_t id, message_t *msg, error_t err);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void CC2420CsmaP$SplitControl$startDone(error_t arg_0x4062baf0);
#line 117
static  void CC2420CsmaP$SplitControl$stopDone(error_t arg_0x4062a6e8);
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static   void CC2420CsmaP$RadioBackoff$requestCca(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9ccf8);
#line 81
static   void CC2420CsmaP$RadioBackoff$requestInitialBackoff(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9c190);






static   void CC2420CsmaP$RadioBackoff$requestCongestionBackoff(
# 41 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
am_id_t arg_0x40f0a320, 
# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t *arg_0x40e9c748);
#line 66
static   void CC2420CsmaP$SubBackoff$setCongestionBackoff(uint16_t arg_0x40e9e688);
#line 60
static   void CC2420CsmaP$SubBackoff$setInitialBackoff(uint16_t arg_0x40e9e108);
# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static   error_t CC2420CsmaP$CC2420Transmit$send(message_t *arg_0x40efd620, bool arg_0x40efd7a8);
# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  void CC2420CsmaP$Send$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868);
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static   uint16_t CC2420CsmaP$Random$rand16(void);
# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static  error_t CC2420CsmaP$SubControl$start(void);









static  error_t CC2420CsmaP$SubControl$stop(void);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *CC2420CsmaP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);




static   cc2420_metadata_t *CC2420CsmaP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0);
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static   error_t CC2420CsmaP$CC2420Power$startOscillator(void);
#line 90
static   error_t CC2420CsmaP$CC2420Power$rxOn(void);
#line 51
static   error_t CC2420CsmaP$CC2420Power$startVReg(void);
#line 63
static   error_t CC2420CsmaP$CC2420Power$stopVReg(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420CsmaP$Resource$release(void);
#line 78
static   error_t CC2420CsmaP$Resource$request(void);
# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   bool CC2420CsmaP$SplitControlState$isState(uint8_t arg_0x40f26368);
#line 45
static   error_t CC2420CsmaP$SplitControlState$requestState(uint8_t arg_0x40f27230);





static   void CC2420CsmaP$SplitControlState$forceState(uint8_t arg_0x40f277b8);
# 57 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
static  am_addr_t CC2420CsmaP$AMPacket$address(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t CC2420CsmaP$sendDone_task$postTask(void);
#line 56
static   error_t CC2420CsmaP$stopDone_task$postTask(void);
#line 56
static   error_t CC2420CsmaP$startDone_task$postTask(void);
# 75 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP$__nesc_unnamed4353 {
#line 75
  CC2420CsmaP$startDone_task = 16U
};
#line 75
typedef int CC2420CsmaP$__nesc_sillytask_startDone_task[CC2420CsmaP$startDone_task];

enum CC2420CsmaP$__nesc_unnamed4354 {
#line 77
  CC2420CsmaP$stopDone_task = 17U
};
#line 77
typedef int CC2420CsmaP$__nesc_sillytask_stopDone_task[CC2420CsmaP$stopDone_task];
enum CC2420CsmaP$__nesc_unnamed4355 {
#line 78
  CC2420CsmaP$sendDone_task = 18U
};
#line 78
typedef int CC2420CsmaP$__nesc_sillytask_sendDone_task[CC2420CsmaP$sendDone_task];
#line 59
enum CC2420CsmaP$__nesc_unnamed4356 {
  CC2420CsmaP$S_STOPPED, 
  CC2420CsmaP$S_STARTING, 
  CC2420CsmaP$S_STARTED, 
  CC2420CsmaP$S_STOPPING, 
  CC2420CsmaP$S_TRANSMITTING
};

message_t *CC2420CsmaP$m_msg;

error_t CC2420CsmaP$sendErr = SUCCESS;


 bool CC2420CsmaP$ccaOn;







static inline void CC2420CsmaP$shutdown(void);


static  error_t CC2420CsmaP$SplitControl$start(void);
#line 124
static inline  error_t CC2420CsmaP$Send$send(message_t *p_msg, uint8_t len);
#line 194
static inline   void CC2420CsmaP$CC2420Transmit$sendDone(message_t *p_msg, error_t err);




static inline   void CC2420CsmaP$CC2420Power$startVRegDone(void);



static inline  void CC2420CsmaP$Resource$granted(void);



static inline   void CC2420CsmaP$CC2420Power$startOscillatorDone(void);




static inline   void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t *msg);







static inline   void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t *msg);
#line 234
static inline  void CC2420CsmaP$sendDone_task$runTask(void);
#line 247
static inline  void CC2420CsmaP$startDone_task$runTask(void);







static inline  void CC2420CsmaP$stopDone_task$runTask(void);









static inline void CC2420CsmaP$shutdown(void);
#line 279
static inline    void CC2420CsmaP$RadioBackoff$default$requestInitialBackoff(am_id_t amId, 
message_t *msg);


static inline    void CC2420CsmaP$RadioBackoff$default$requestCongestionBackoff(am_id_t amId, 
message_t *msg);


static inline    void CC2420CsmaP$RadioBackoff$default$requestCca(am_id_t amId, 
message_t *msg);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static  void CC2420ControlP$CC2420Config$syncDone(error_t arg_0x40ea9e98);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420ControlP$RXCTRL1$write(uint16_t arg_0x40f4e010);
# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void CC2420ControlP$StartupTimer$start(CC2420ControlP$StartupTimer$size_type arg_0x4092d460);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420ControlP$MDMCTRL0$write(uint16_t arg_0x40f4e010);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420ControlP$RSTN$makeOutput(void);
#line 29
static   void CC2420ControlP$RSTN$set(void);
static   void CC2420ControlP$RSTN$clr(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void CC2420ControlP$ReadRssi$readDone(error_t arg_0x40624580, CC2420ControlP$ReadRssi$val_t arg_0x40624708);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t CC2420ControlP$syncDone$postTask(void);
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420ControlP$RSSI$read(uint16_t *arg_0x40f4fa50);







static   cc2420_status_t CC2420ControlP$IOCFG0$write(uint16_t arg_0x40f4e010);
# 48 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static   am_addr_t CC2420ControlP$ActiveMessageAddress$amAddress(void);




static   am_group_t CC2420ControlP$ActiveMessageAddress$amGroup(void);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420ControlP$CSN$makeOutput(void);
#line 29
static   void CC2420ControlP$CSN$set(void);
static   void CC2420ControlP$CSN$clr(void);




static   void CC2420ControlP$VREN$makeOutput(void);
#line 29
static   void CC2420ControlP$VREN$set(void);
static   void CC2420ControlP$VREN$clr(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420ControlP$SXOSCON$strobe(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420ControlP$SpiResource$release(void);
#line 78
static   error_t CC2420ControlP$SpiResource$request(void);
#line 110
static   error_t CC2420ControlP$SyncResource$release(void);
#line 78
static   error_t CC2420ControlP$SyncResource$request(void);
# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static   void CC2420ControlP$CC2420Power$startOscillatorDone(void);
#line 56
static   void CC2420ControlP$CC2420Power$startVRegDone(void);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420ControlP$IOCFG1$write(uint16_t arg_0x40f4e010);
#line 55
static   cc2420_status_t CC2420ControlP$FSCTRL$write(uint16_t arg_0x40f4e010);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420ControlP$SRXON$strobe(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void CC2420ControlP$Resource$granted(void);
# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static   cc2420_status_t CC2420ControlP$PANID$write(uint8_t arg_0x40f535f0, uint8_t *arg_0x40f53798, uint8_t arg_0x40f53920);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t CC2420ControlP$InterruptCCA$disable(void);
#line 42
static   error_t CC2420ControlP$InterruptCCA$enableRisingEdge(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420ControlP$RssiResource$release(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420ControlP$SRFOFF$strobe(void);
# 112 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP$__nesc_unnamed4357 {
#line 112
  CC2420ControlP$sync = 19U
};
#line 112
typedef int CC2420ControlP$__nesc_sillytask_sync[CC2420ControlP$sync];
enum CC2420ControlP$__nesc_unnamed4358 {
#line 113
  CC2420ControlP$syncDone = 20U
};
#line 113
typedef int CC2420ControlP$__nesc_sillytask_syncDone[CC2420ControlP$syncDone];
#line 86
#line 80
typedef enum CC2420ControlP$__nesc_unnamed4359 {
  CC2420ControlP$S_VREG_STOPPED, 
  CC2420ControlP$S_VREG_STARTING, 
  CC2420ControlP$S_VREG_STARTED, 
  CC2420ControlP$S_XOSC_STARTING, 
  CC2420ControlP$S_XOSC_STARTED
} CC2420ControlP$cc2420_control_state_t;

uint8_t CC2420ControlP$m_channel;

uint8_t CC2420ControlP$m_tx_power;

uint16_t CC2420ControlP$m_pan;

uint16_t CC2420ControlP$m_short_addr;

bool CC2420ControlP$m_sync_busy;

bool CC2420ControlP$autoAckEnabled;

bool CC2420ControlP$hwAutoAckDefault;

bool CC2420ControlP$addressRecognition;

 CC2420ControlP$cc2420_control_state_t CC2420ControlP$m_state = CC2420ControlP$S_VREG_STOPPED;



static void CC2420ControlP$writeFsctrl(void);
static void CC2420ControlP$writeMdmctrl0(void);
static void CC2420ControlP$writeId(void);





static inline  error_t CC2420ControlP$Init$init(void);
#line 156
static inline   error_t CC2420ControlP$Resource$request(void);







static inline   error_t CC2420ControlP$Resource$release(void);







static inline   error_t CC2420ControlP$CC2420Power$startVReg(void);
#line 184
static inline   error_t CC2420ControlP$CC2420Power$stopVReg(void);







static inline   error_t CC2420ControlP$CC2420Power$startOscillator(void);
#line 234
static inline   error_t CC2420ControlP$CC2420Power$rxOn(void);
#line 264
static inline   uint16_t CC2420ControlP$CC2420Config$getShortAddr(void);







static inline   uint16_t CC2420ControlP$CC2420Config$getPanAddr(void);
#line 285
static inline  error_t CC2420ControlP$CC2420Config$sync(void);
#line 331
static inline   bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void);






static inline   bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void);









static inline  void CC2420ControlP$SyncResource$granted(void);
#line 362
static inline  void CC2420ControlP$SpiResource$granted(void);




static inline  void CC2420ControlP$RssiResource$granted(void);
#line 380
static inline   void CC2420ControlP$StartupTimer$fired(void);









static inline   void CC2420ControlP$InterruptCCA$fired(void);
#line 414
static inline  void CC2420ControlP$sync$runTask(void);



static inline  void CC2420ControlP$syncDone$runTask(void);









static void CC2420ControlP$writeFsctrl(void);
#line 442
static void CC2420ControlP$writeMdmctrl0(void);
#line 461
static void CC2420ControlP$writeId(void);
#line 478
static inline   void CC2420ControlP$ReadRssi$default$readDone(error_t error, uint16_t data);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(uint16_t arg_0x4085fed0);

static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(uint16_t arg_0x4085e868);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents(void);
#line 36
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$setControlAsCompare(void);



static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents(void);
#line 33
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt(void);
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Init$init(void);
#line 54
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$stop(void);




static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void);










static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$size_type /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$get(void);






static   bool /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending(void);










static   void /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$overflow(void);
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC$1$upper_count_type /*Counter32khz32C.Transform*/TransformCounterC$1$m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC$1$__nesc_unnamed4360 {

  TransformCounterC$1$LOW_SHIFT_RIGHT = 0, 
  TransformCounterC$1$HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC$1$from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC$1$LOW_SHIFT_RIGHT, 
  TransformCounterC$1$NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC$1$from_size_type ) + 0, 



  TransformCounterC$1$OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC$1$NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC$1$upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC$1$NUM_UPPER_BITS - 1)) - 1 : 0
};

static   /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$get(void);
#line 122
static inline   void /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$overflow(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$fired(void);
#line 92
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$size_type arg_0x4092b598, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$size_type arg_0x4092b728);
#line 62
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$stop(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$get(void);
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$__nesc_unnamed4361 {

  TransformAlarmC$1$MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_size_type ) - 1 - 0, 
  TransformAlarmC$1$MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$MAX_DELAY_LOG2
};

static inline   /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$getNow(void);
#line 91
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$stop(void);




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$set_alarm(void);
#line 136
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type dt);









static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type dt);




static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$fired(void);
#line 166
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$overflow(void);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$makeInput(void);
#line 59
static   bool /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$get(void);
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$get(void);
static inline   void /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$makeInput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$makeOutput(void);
#line 34
static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$set(void);




static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set(void);
static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr(void);




static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$makeOutput(void);
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$HplGeneralIO$get(void);
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$GeneralIO$get(void);
# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$HplGeneralIO$get(void);
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$GeneralIO$get(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$makeOutput(void);
#line 34
static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$set(void);




static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$set(void);
static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$clr(void);




static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$makeOutput(void);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$makeInput(void);
#line 59
static   bool /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$get(void);
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$get(void);
static inline   void /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$makeInput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$makeOutput(void);
#line 34
static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$set(void);




static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$set(void);
static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$clr(void);




static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$makeOutput(void);
# 57 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$clearOverflow(void);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captured(uint16_t arg_0x41011ea0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$setControlAsCapture(bool arg_0x40865958);

static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$enableEvents(void);
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$disableEvents(void);
#line 33
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$clearPendingInterrupt(void);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectIOFunc(void);
#line 78
static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectModuleFunc(void);
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$enableCapture(uint8_t mode);
#line 50
static inline   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureRisingEdge(void);



static inline   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureFallingEdge(void);



static inline   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$disable(void);






static inline   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$captured(uint16_t time);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$clear(void);
#line 36
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$disable(void);
#line 56
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$edge(bool arg_0x40aceef8);
#line 31
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$enable(void);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$fired(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$enable(bool rising);








static inline   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$enableRisingEdge(void);







static inline   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$disable(void);







static inline   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$fired(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$clear(void);
#line 36
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$disable(void);
#line 56
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$edge(bool arg_0x40aceef8);
#line 31
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$enable(void);
# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$fired(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$enable(bool rising);
#line 54
static inline   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$enableFallingEdge(void);



static inline   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$disable(void);







static inline   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$fired(void);
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static   error_t CC2420SpiP$SpiPacket$send(uint8_t *arg_0x41040118, uint8_t *arg_0x410402c0, uint16_t arg_0x41040450);
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   void CC2420SpiP$Fifo$writeDone(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x4101f838, uint8_t arg_0x4101f9c0, error_t arg_0x4101fb48);
#line 71
static   void CC2420SpiP$Fifo$readDone(
# 44 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104c408, 
# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t *arg_0x410217c8, uint8_t arg_0x41021950, error_t arg_0x41021ad8);
# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static   void CC2420SpiP$ChipSpiResource$releasing(void);
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
static   uint8_t CC2420SpiP$SpiByte$write(uint8_t arg_0x41045678);
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   void CC2420SpiP$WorkingState$toIdle(void);




static   bool CC2420SpiP$WorkingState$isIdle(void);
#line 45
static   error_t CC2420SpiP$WorkingState$requestState(uint8_t arg_0x40f27230);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420SpiP$SpiResource$release(void);
#line 87
static   error_t CC2420SpiP$SpiResource$immediateRequest(void);
#line 78
static   error_t CC2420SpiP$SpiResource$request(void);
#line 118
static   bool CC2420SpiP$SpiResource$isOwner(void);
#line 92
static  void CC2420SpiP$Resource$granted(
# 43 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4104da80);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t CC2420SpiP$grant$postTask(void);
# 86 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP$__nesc_unnamed4362 {
#line 86
  CC2420SpiP$grant = 21U
};
#line 86
typedef int CC2420SpiP$__nesc_sillytask_grant[CC2420SpiP$grant];
#line 61
enum CC2420SpiP$__nesc_unnamed4363 {
  CC2420SpiP$RESOURCE_COUNT = 5U, 
  CC2420SpiP$NO_HOLDER = 0xFF
};


enum CC2420SpiP$__nesc_unnamed4364 {
  CC2420SpiP$S_IDLE, 
  CC2420SpiP$S_BUSY
};


 uint16_t CC2420SpiP$m_addr;


uint8_t CC2420SpiP$m_requests = 0;


uint8_t CC2420SpiP$m_holder = CC2420SpiP$NO_HOLDER;


bool CC2420SpiP$release;


static error_t CC2420SpiP$attemptRelease(void);







static inline   void CC2420SpiP$ChipSpiResource$abortRelease(void);






static inline   error_t CC2420SpiP$ChipSpiResource$attemptRelease(void);




static   error_t CC2420SpiP$Resource$request(uint8_t id);
#line 124
static   error_t CC2420SpiP$Resource$immediateRequest(uint8_t id);
#line 147
static   error_t CC2420SpiP$Resource$release(uint8_t id);
#line 176
static inline   uint8_t CC2420SpiP$Resource$isOwner(uint8_t id);





static inline  void CC2420SpiP$SpiResource$granted(void);




static   cc2420_status_t CC2420SpiP$Fifo$beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 207
static inline   error_t CC2420SpiP$Fifo$continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline   cc2420_status_t CC2420SpiP$Fifo$write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 258
static inline   cc2420_status_t CC2420SpiP$Ram$write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 283
static inline   cc2420_status_t CC2420SpiP$Reg$read(uint8_t addr, uint16_t *data);
#line 301
static   cc2420_status_t CC2420SpiP$Reg$write(uint8_t addr, uint16_t data);
#line 314
static   cc2420_status_t CC2420SpiP$Strobe$strobe(uint8_t addr);










static   void CC2420SpiP$SpiPacket$sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP$attemptRelease(void);
#line 354
static inline  void CC2420SpiP$grant$runTask(void);








static inline   void CC2420SpiP$Resource$default$granted(uint8_t id);


static inline    void CC2420SpiP$Fifo$default$readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline    void CC2420SpiP$Fifo$default$writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t StateImplP$state[4U];

enum StateImplP$__nesc_unnamed4365 {
  StateImplP$S_IDLE = 0
};


static inline  error_t StateImplP$Init$init(void);
#line 96
static   error_t StateImplP$State$requestState(uint8_t id, uint8_t reqState);
#line 111
static inline   void StateImplP$State$forceState(uint8_t id, uint8_t reqState);






static inline   void StateImplP$State$toIdle(uint8_t id);







static inline   bool StateImplP$State$isIdle(uint8_t id);






static   bool StateImplP$State$isState(uint8_t id, uint8_t myState);
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$sendDone(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1740, 
# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t *arg_0x41040b98, uint8_t *arg_0x41040d40, uint16_t arg_0x41040ed0, 
error_t arg_0x4103f088);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static   msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$getConfig(
# 46 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e07a0);
# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$enableRxIntr(void);
#line 197
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrRxIntr(void);
#line 97
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$resetUsart(bool arg_0x410d7c08);
#line 177
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableRxIntr(void);









static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isTxIntrPending(void);
#line 224
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$tx(uint8_t arg_0x410ce010);
#line 168
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$setModeSpi(msp430_spi_union_config_t *arg_0x410d3d50);
#line 231
static   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$rx(void);
#line 192
static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isRxIntrPending(void);









static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrTxIntr(void);
#line 158
static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableSpi(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$immediateRequest(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e1e28);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x410e23a8);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$postTask(void);
# 66 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$__nesc_unnamed4366 {
#line 66
  Msp430SpiNoDmaP$0$signalDone_task = 22U
};
#line 66
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$__nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task];
#line 55
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$__nesc_unnamed4367 {
  Msp430SpiNoDmaP$0$SPI_ATOMIC_SIZE = 2
};

 uint8_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf;
 uint8_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf;
 uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len;
 uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos;
 uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone(void);


static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$immediateRequest(uint8_t id);



static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$request(uint8_t id);



static inline   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$isOwner(uint8_t id);



static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$release(uint8_t id);



static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$configure(uint8_t id);



static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$unconfigure(uint8_t id);





static inline  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$granted(uint8_t id);



static   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiByte$write(uint8_t tx);
#line 110
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$isOwner(uint8_t id);
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$request(uint8_t id);
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$immediateRequest(uint8_t id);
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$release(uint8_t id);
static inline    msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$default$getConfig(uint8_t id);



static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$default$granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$continueOp(void);
#line 146
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 168
static inline  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$runTask(void);



static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$rxDone(uint8_t data);
#line 185
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone(void);




static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$txDone(void);

static inline    void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$default$sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void HplMsp430Usart0P$UCLK$selectIOFunc(void);
#line 78
static   void HplMsp430Usart0P$UCLK$selectModuleFunc(void);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static   void HplMsp430Usart0P$Interrupts$rxDone(uint8_t arg_0x410a88e8);
#line 49
static   void HplMsp430Usart0P$Interrupts$txDone(void);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void HplMsp430Usart0P$URXD$selectIOFunc(void);
#line 85
static   void HplMsp430Usart0P$UTXD$selectIOFunc(void);
# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static   void HplMsp430Usart0P$HplI2C$clearModeI2C(void);
#line 6
static   bool HplMsp430Usart0P$HplI2C$isI2C(void);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void HplMsp430Usart0P$SOMI$selectIOFunc(void);
#line 78
static   void HplMsp430Usart0P$SOMI$selectModuleFunc(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static   void HplMsp430Usart0P$I2CInterrupts$fired(void);
# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void HplMsp430Usart0P$SIMO$selectIOFunc(void);
#line 78
static   void HplMsp430Usart0P$SIMO$selectModuleFunc(void);
# 89 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
 static volatile uint8_t HplMsp430Usart0P$IE1 __asm ("0x0000");
 static volatile uint8_t HplMsp430Usart0P$ME1 __asm ("0x0004");
 static volatile uint8_t HplMsp430Usart0P$IFG1 __asm ("0x0002");
 static volatile uint8_t HplMsp430Usart0P$U0TCTL __asm ("0x0071");

 static volatile uint8_t HplMsp430Usart0P$U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(18))) ;




void sig_UART0TX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(16))) ;
#line 132
static inline   void HplMsp430Usart0P$Usart$setUbr(uint16_t control);










static inline   void HplMsp430Usart0P$Usart$setUmctl(uint8_t control);







static inline   void HplMsp430Usart0P$Usart$resetUsart(bool reset);
#line 207
static inline   void HplMsp430Usart0P$Usart$disableUart(void);
#line 238
static inline   void HplMsp430Usart0P$Usart$enableSpi(void);








static inline   void HplMsp430Usart0P$Usart$disableSpi(void);








static inline void HplMsp430Usart0P$configSpi(msp430_spi_union_config_t *config);








static   void HplMsp430Usart0P$Usart$setModeSpi(msp430_spi_union_config_t *config);
#line 316
static inline   bool HplMsp430Usart0P$Usart$isTxIntrPending(void);
#line 330
static inline   bool HplMsp430Usart0P$Usart$isRxIntrPending(void);






static inline   void HplMsp430Usart0P$Usart$clrTxIntr(void);



static inline   void HplMsp430Usart0P$Usart$clrRxIntr(void);



static inline   void HplMsp430Usart0P$Usart$clrIntr(void);



static inline   void HplMsp430Usart0P$Usart$disableRxIntr(void);







static inline   void HplMsp430Usart0P$Usart$disableIntr(void);



static inline   void HplMsp430Usart0P$Usart$enableRxIntr(void);
#line 382
static inline   void HplMsp430Usart0P$Usart$tx(uint8_t data);



static   uint8_t HplMsp430Usart0P$Usart$rx(void);
# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse(void);







static   uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(void);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41197908, 
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t arg_0x410a88e8);
#line 49
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41197908);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x41196010);








static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void);




static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t data);




static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawI2CInterrupts$fired(void);




static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(uint8_t id);
static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(uint8_t id, uint8_t data);
static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$default$fired(uint8_t id);
# 39 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$__nesc_unnamed4368 {
#line 39
  FcfsResourceQueueC$1$NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;

static inline  error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void);




static inline   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void);



static inline   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t id);



static inline   resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void);
#line 72
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$requested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$immediateRequested(
# 55 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9c308);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$unconfigure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$configure(
# 60 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9b4d8);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$enqueue(resource_client_id_t arg_0x40b828b0);
#line 43
static   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$isEmpty(void);
#line 60
static   resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$dequeue(void);
# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$requested(void);
#line 46
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$granted(void);
#line 81
static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$immediateRequested(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$granted(
# 54 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40b9d968);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$postTask(void);
# 74 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$__nesc_unnamed4369 {
#line 74
  ArbiterP$1$grantedTask = 23U
};
#line 74
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$__nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$__nesc_unnamed4370 {
#line 67
  ArbiterP$1$RES_CONTROLLED, ArbiterP$1$RES_GRANTING, ArbiterP$1$RES_IMM_GRANTING, ArbiterP$1$RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$__nesc_unnamed4371 {
#line 68
  ArbiterP$1$default_owner_id = 1U
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_CONTROLLED;
 uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$default_owner_id;
 uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId;



static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$request(uint8_t id);
#line 89
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$immediateRequest(uint8_t id);
#line 107
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$release(uint8_t id);
#line 126
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$release(void);
#line 146
static inline   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$inUse(void);








static inline   uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$userId(void);






static   uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$isOwner(uint8_t id);










static inline  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$runTask(void);
#line 185
static inline   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$default$granted(uint8_t id);

static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$requested(uint8_t id);

static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$immediateRequested(uint8_t id);

static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$granted(void);

static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$requested(void);


static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$immediateRequested(void);


static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$configure(uint8_t id);

static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$unconfigure(uint8_t id);
# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static   void HplMsp430I2C0P$HplUsart$resetUsart(bool arg_0x410d7c08);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
 static volatile uint8_t HplMsp430I2C0P$U0CTL __asm ("0x0070");





static inline   bool HplMsp430I2C0P$HplI2C$isI2C(void);



static inline   void HplMsp430I2C0P$HplI2C$clearModeI2C(void);
# 51 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC$addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC$group = TOS_AM_GROUP;






static inline   am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void);
#line 82
static inline   am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void);
#line 95
static   am_addr_t ActiveMessageAddressC$amAddress(void);
# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static   void CC2420TransmitP$RadioBackoff$requestInitialBackoff(message_t *arg_0x40e9c190);






static   void CC2420TransmitP$RadioBackoff$requestCongestionBackoff(message_t *arg_0x40e9c748);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420TransmitP$STXONCCA$strobe(void);
# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static   error_t CC2420TransmitP$CaptureSFD$captureFallingEdge(void);
#line 55
static   void CC2420TransmitP$CaptureSFD$disable(void);
#line 42
static   error_t CC2420TransmitP$CaptureSFD$captureRisingEdge(void);
# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$BackoffTimer$size_type arg_0x4092d460);






static   void CC2420TransmitP$BackoffTimer$stop(void);
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static   cc2420_status_t CC2420TransmitP$TXCTRL$write(uint16_t arg_0x40f4e010);
# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static   void CC2420TransmitP$CC2420Receive$sfd_dropped(void);
#line 47
static   void CC2420TransmitP$CC2420Receive$sfd(uint16_t arg_0x41254220);
# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static   void CC2420TransmitP$Send$sendDone(message_t *arg_0x40efc720, error_t arg_0x40efc8a8);
# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static   void CC2420TransmitP$ChipSpiResource$abortRelease(void);







static   error_t CC2420TransmitP$ChipSpiResource$attemptRelease(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420TransmitP$SFLUSHTX$strobe(void);
# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420TransmitP$CSN$makeOutput(void);
#line 29
static   void CC2420TransmitP$CSN$set(void);
static   void CC2420TransmitP$CSN$clr(void);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *CC2420TransmitP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);




static   cc2420_metadata_t *CC2420TransmitP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420TransmitP$SpiResource$release(void);
#line 87
static   error_t CC2420TransmitP$SpiResource$immediateRequest(void);
#line 78
static   error_t CC2420TransmitP$SpiResource$request(void);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420TransmitP$CCA$makeInput(void);
#line 32
static   bool CC2420TransmitP$CCA$get(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420TransmitP$SNOP$strobe(void);
# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420TransmitP$SFD$makeInput(void);
#line 32
static   bool CC2420TransmitP$SFD$get(void);
# 39 "/opt/tinyos-2.x/tos/interfaces/RadioTimeStamping.nc"
static   void CC2420TransmitP$TimeStamp$transmittedSFD(uint16_t arg_0x4122b388, message_t *arg_0x4122b538);










static   void CC2420TransmitP$TimeStamp$receivedSFD(uint16_t arg_0x4122ba68);
# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   cc2420_status_t CC2420TransmitP$TXFIFO$write(uint8_t *arg_0x4101f0b0, uint8_t arg_0x4101f238);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420TransmitP$STXON$strobe(void);
# 90 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 78
typedef enum CC2420TransmitP$__nesc_unnamed4372 {
  CC2420TransmitP$S_STOPPED, 
  CC2420TransmitP$S_STARTED, 
  CC2420TransmitP$S_LOAD, 
  CC2420TransmitP$S_SAMPLE_CCA, 
  CC2420TransmitP$S_BEGIN_TRANSMIT, 
  CC2420TransmitP$S_SFD, 
  CC2420TransmitP$S_EFD, 
  CC2420TransmitP$S_ACK_WAIT, 
  CC2420TransmitP$S_LOAD_CANCEL, 
  CC2420TransmitP$S_TX_CANCEL, 
  CC2420TransmitP$S_CCA_CANCEL
} CC2420TransmitP$cc2420_transmit_state_t;





enum CC2420TransmitP$__nesc_unnamed4373 {
  CC2420TransmitP$CC2420_ABORT_PERIOD = 320
};

 message_t *CC2420TransmitP$m_msg;

 bool CC2420TransmitP$m_cca;

 uint8_t CC2420TransmitP$m_tx_power;

CC2420TransmitP$cc2420_transmit_state_t CC2420TransmitP$m_state = CC2420TransmitP$S_STOPPED;

bool CC2420TransmitP$m_receiving = FALSE;

uint16_t CC2420TransmitP$m_prev_time;


bool CC2420TransmitP$sfdHigh;


bool CC2420TransmitP$abortSpiRelease;


 int8_t CC2420TransmitP$totalCcaChecks;


 uint16_t CC2420TransmitP$myInitialBackoff;


 uint16_t CC2420TransmitP$myCongestionBackoff;



static inline error_t CC2420TransmitP$send(message_t *p_msg, bool cca);

static void CC2420TransmitP$loadTXFIFO(void);
static void CC2420TransmitP$attemptSend(void);
static void CC2420TransmitP$congestionBackoff(void);
static error_t CC2420TransmitP$acquireSpiResource(void);
static inline error_t CC2420TransmitP$releaseSpiResource(void);
static void CC2420TransmitP$signalDone(error_t err);



static inline  error_t CC2420TransmitP$Init$init(void);







static inline  error_t CC2420TransmitP$StdControl$start(void);










static inline  error_t CC2420TransmitP$StdControl$stop(void);
#line 172
static inline   error_t CC2420TransmitP$Send$send(message_t *p_msg, bool useCca);
#line 229
static inline   void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t backoffTime);







static inline   void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t backoffTime);
#line 259
static inline   void CC2420TransmitP$CaptureSFD$captured(uint16_t time);
#line 328
static inline   void CC2420TransmitP$ChipSpiResource$releasing(void);
#line 340
static inline   void CC2420TransmitP$CC2420Receive$receive(uint8_t type, message_t *ack_msg);
#line 368
static inline  void CC2420TransmitP$SpiResource$granted(void);
#line 407
static inline   void CC2420TransmitP$TXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 446
static inline   void CC2420TransmitP$TXFIFO$readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline   void CC2420TransmitP$BackoffTimer$fired(void);
#line 511
static inline error_t CC2420TransmitP$send(message_t *p_msg, bool cca);
#line 583
static void CC2420TransmitP$attemptSend(void);
#line 624
static void CC2420TransmitP$congestionBackoff(void);






static error_t CC2420TransmitP$acquireSpiResource(void);







static inline error_t CC2420TransmitP$releaseSpiResource(void);
#line 661
static void CC2420TransmitP$loadTXFIFO(void);
#line 683
static void CC2420TransmitP$signalDone(error_t err);
#line 695
static inline    void CC2420TransmitP$TimeStamp$default$transmittedSFD(uint16_t time, message_t *p_msg);


static inline    void CC2420TransmitP$TimeStamp$default$receivedSFD(uint16_t time);
# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   bool CC2420ReceiveP$FIFO$get(void);
# 101 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static   bool CC2420ReceiveP$CC2420Config$isAutoAckEnabled(void);
#line 96
static   bool CC2420ReceiveP$CC2420Config$isHwAutoAckDefault(void);
#line 64
static   uint16_t CC2420ReceiveP$CC2420Config$getShortAddr(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t CC2420ReceiveP$receiveDone_task$postTask(void);
# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   bool CC2420ReceiveP$FIFOP$get(void);
# 61 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static   void CC2420ReceiveP$CC2420Receive$receive(uint8_t arg_0x41254b18, message_t *arg_0x41254cc8);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420ReceiveP$SACK$strobe(void);
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void CC2420ReceiveP$CSN$set(void);
static   void CC2420ReceiveP$CSN$clr(void);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *CC2420ReceiveP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);




static   cc2420_metadata_t *CC2420ReceiveP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *CC2420ReceiveP$Receive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t CC2420ReceiveP$SpiResource$release(void);
#line 87
static   error_t CC2420ReceiveP$SpiResource$immediateRequest(void);
#line 78
static   error_t CC2420ReceiveP$SpiResource$request(void);
#line 118
static   bool CC2420ReceiveP$SpiResource$isOwner(void);
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static   error_t CC2420ReceiveP$RXFIFO$continueRead(uint8_t *arg_0x41021010, uint8_t arg_0x41021198);
#line 51
static   cc2420_status_t CC2420ReceiveP$RXFIFO$beginRead(uint8_t *arg_0x41022848, uint8_t arg_0x410229d0);
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static   error_t CC2420ReceiveP$InterruptFIFOP$disable(void);
#line 43
static   error_t CC2420ReceiveP$InterruptFIFOP$enableFallingEdge(void);
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static   cc2420_status_t CC2420ReceiveP$SFLUSHRX$strobe(void);
# 109 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP$__nesc_unnamed4374 {
#line 109
  CC2420ReceiveP$receiveDone_task = 24U
};
#line 109
typedef int CC2420ReceiveP$__nesc_sillytask_receiveDone_task[CC2420ReceiveP$receiveDone_task];
#line 71
#line 65
typedef enum CC2420ReceiveP$__nesc_unnamed4375 {
  CC2420ReceiveP$S_STOPPED, 
  CC2420ReceiveP$S_STARTED, 
  CC2420ReceiveP$S_RX_LENGTH, 
  CC2420ReceiveP$S_RX_FCF, 
  CC2420ReceiveP$S_RX_PAYLOAD
} CC2420ReceiveP$cc2420_receive_state_t;

enum CC2420ReceiveP$__nesc_unnamed4376 {
  CC2420ReceiveP$RXFIFO_SIZE = 128, 
  CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP$SACK_HEADER_LENGTH = 7
};

uint16_t CC2420ReceiveP$m_timestamp_queue[CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP$m_timestamp_head;

uint8_t CC2420ReceiveP$m_timestamp_size;


uint8_t CC2420ReceiveP$m_missed_packets;


bool CC2420ReceiveP$receivingPacket;


 uint8_t CC2420ReceiveP$rxFrameLength;

 uint8_t CC2420ReceiveP$m_bytes_left;

 message_t *CC2420ReceiveP$m_p_rx_buf;

message_t CC2420ReceiveP$m_rx_buf;

CC2420ReceiveP$cc2420_receive_state_t CC2420ReceiveP$m_state;


static void CC2420ReceiveP$reset_state(void);
static void CC2420ReceiveP$beginReceive(void);
static void CC2420ReceiveP$receive(void);
static void CC2420ReceiveP$waitForNextPacket(void);
static void CC2420ReceiveP$flush(void);




static inline  error_t CC2420ReceiveP$Init$init(void);





static inline  error_t CC2420ReceiveP$StdControl$start(void);









static inline  error_t CC2420ReceiveP$StdControl$stop(void);
#line 157
static inline   void CC2420ReceiveP$CC2420Receive$sfd(uint16_t time);








static inline   void CC2420ReceiveP$CC2420Receive$sfd_dropped(void);
#line 183
static inline   void CC2420ReceiveP$InterruptFIFOP$fired(void);










static inline  void CC2420ReceiveP$SpiResource$granted(void);








static inline   void CC2420ReceiveP$RXFIFO$readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 325
static inline   void CC2420ReceiveP$RXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline  void CC2420ReceiveP$receiveDone_task$runTask(void);
#line 348
static inline  void CC2420ReceiveP$CC2420Config$syncDone(error_t error);






static void CC2420ReceiveP$beginReceive(void);
#line 373
static void CC2420ReceiveP$flush(void);
#line 390
static void CC2420ReceiveP$receive(void);









static void CC2420ReceiveP$waitForNextPacket(void);
#line 428
static void CC2420ReceiveP$reset_state(void);
# 89 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketC.nc"
static inline   cc2420_header_t *CC2420PacketC$CC2420PacketBody$getHeader(message_t *msg);



static inline   cc2420_metadata_t *CC2420PacketC$CC2420PacketBody$getMetadata(message_t *msg);
# 41 "/opt/tinyos-2.x/tos/system/RandomMlcgP.nc"
uint32_t RandomMlcgP$seed;


static inline  error_t RandomMlcgP$Init$init(void);
#line 58
static   uint32_t RandomMlcgP$Random$rand32(void);
#line 78
static inline   uint16_t RandomMlcgP$Random$rand16(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
static  error_t UniqueSendP$SubSend$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790);
#line 89
static  void UniqueSendP$Send$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868);
# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
static   uint16_t UniqueSendP$Random$rand16(void);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *UniqueSendP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);
# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   void UniqueSendP$State$toIdle(void);
#line 45
static   error_t UniqueSendP$State$requestState(uint8_t arg_0x40f27230);
# 54 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP$localSendId;

enum UniqueSendP$__nesc_unnamed4377 {
  UniqueSendP$S_IDLE, 
  UniqueSendP$S_SENDING
};


static inline  error_t UniqueSendP$Init$init(void);
#line 75
static inline  error_t UniqueSendP$Send$send(message_t *msg, uint8_t len);
#line 104
static inline  void UniqueSendP$SubSend$sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *UniqueReceiveP$Receive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static   cc2420_header_t *UniqueReceiveP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68);
# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
static  message_t *UniqueReceiveP$DuplicateReceive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8);
# 59 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP$__nesc_unnamed4378 {
  am_addr_t source;
  uint8_t dsn;
} UniqueReceiveP$receivedMessages[4];

uint8_t UniqueReceiveP$writeIndex = 0;


uint8_t UniqueReceiveP$recycleSourceElement;

enum UniqueReceiveP$__nesc_unnamed4379 {
  UniqueReceiveP$INVALID_ELEMENT = 0xFF
};


static inline  error_t UniqueReceiveP$Init$init(void);









static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn);
#line 104
static inline  message_t *UniqueReceiveP$SubReceive$receive(message_t *msg, void *payload, 
uint8_t len);
#line 130
static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 156
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn);
#line 177
static inline   message_t *UniqueReceiveP$DuplicateReceive$default$receive(message_t *msg, void *payload, uint8_t len);
# 196 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void)
{
}

# 516 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$TimerA$overflow(void)
#line 516
{
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void){
#line 37
  Msp430Adc12ImplP$TimerA$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void)
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow();
}





static inline    void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(uint8_t arg_0x40875be0){
#line 28
  switch (arg_0x40875be0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(arg_0x40875be0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void)
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerA0$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4380 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(* (volatile uint16_t *)354U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void)
{
  return * (volatile uint16_t *)370U;
}

# 517 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$CompareA0$fired(void)
#line 517
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void){
#line 34
  Msp430Adc12ImplP$CompareA0$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4381 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(* (volatile uint16_t *)356U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void)
{
  return * (volatile uint16_t *)372U;
}

# 518 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$CompareA1$fired(void)
#line 518
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void){
#line 34
  Msp430Adc12ImplP$CompareA1$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$__nesc_unnamed4382 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(* (volatile uint16_t *)358U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void)
{
  return * (volatile uint16_t *)374U;
}

#line 181
static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void)
{
  uint8_t n = * (volatile uint16_t *)302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerA1$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void)
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerB0$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void)
{
}

# 103 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void)
{
}

#line 103
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void)
{
}

# 47 "/opt/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline   void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void)
{
}

# 166 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void)
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void)
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC$0$m_upper & /*CounterMilli32C.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 166 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$overflow(void)
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   void /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$overflow(void){
#line 71
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline   void /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$overflow(void)
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC$1$m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC$1$m_upper & /*Counter32khz32C.Transform*/TransformCounterC$1$OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$overflow();
      }
  }
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void){
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$overflow();
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void)
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow();
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow();
#line 37
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void)
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void)
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void)
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void)
{
  * (volatile uint16_t *)386U &= ~0x0010;
}

# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void){
#line 40
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents();
#line 40
}
#line 40
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void)
{
  return * (volatile uint16_t *)402U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4383 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(* (volatile uint16_t *)386U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired();
    }
}

# 86 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void)
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get();
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void)
{
  return * (volatile uint16_t *)384U & 1U;
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void)
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending();
}

# 60 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void)
{
  * (volatile uint16_t *)386U |= 0x0010;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void){
#line 39
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents();
#line 39
}
#line 39
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void)
{
  * (volatile uint16_t *)386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)402U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t arg_0x4085fed0){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(arg_0x4085fed0);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t arg_0x4085e868){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(arg_0x4085e868);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x4092b728){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(arg_0x4092b598, arg_0x4092b728);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void)
{
  return * (volatile uint16_t *)404U;
}

# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline uint16_t __nesc_ntoh_leuint16(const void *source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 269
static __inline uint16_t __nesc_hton_uint16(void *target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 166 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline   void CC2420ReceiveP$CC2420Receive$sfd_dropped(void)
#line 166
{
  if (CC2420ReceiveP$m_timestamp_size) {
      CC2420ReceiveP$m_timestamp_size--;
    }
}

# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static   void CC2420TransmitP$CC2420Receive$sfd_dropped(void){
#line 53
  CC2420ReceiveP$CC2420Receive$sfd_dropped();
#line 53
}
#line 53
# 50 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureRisingEdge(void)
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static   error_t CC2420TransmitP$CaptureSFD$captureRisingEdge(void){
#line 42
  unsigned char result;
#line 42

#line 42
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureRisingEdge();
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$getRaw(void)
#line 48
{
#line 48
  return * (volatile uint8_t *)28U & (0x01 << 1);
}

#line 49
static inline   bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$get(void)
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   bool /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$get(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$get(void)
#line 40
{
#line 40
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   bool CC2420TransmitP$SFD$get(void){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 157 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline   void CC2420ReceiveP$CC2420Receive$sfd(uint16_t time)
#line 157
{
  if (CC2420ReceiveP$m_timestamp_size < CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP$m_timestamp_head + CC2420ReceiveP$m_timestamp_size) % 
      CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE;

#line 161
      CC2420ReceiveP$m_timestamp_queue[tail] = time;
      CC2420ReceiveP$m_timestamp_size++;
    }
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static   void CC2420TransmitP$CC2420Receive$sfd(uint16_t arg_0x41254220){
#line 47
  CC2420ReceiveP$CC2420Receive$sfd(arg_0x41254220);
#line 47
}
#line 47
# 698 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline    void CC2420TransmitP$TimeStamp$default$receivedSFD(uint16_t time)
#line 698
{
}

# 50 "/opt/tinyos-2.x/tos/interfaces/RadioTimeStamping.nc"
inline static   void CC2420TransmitP$TimeStamp$receivedSFD(uint16_t arg_0x4122ba68){
#line 50
  CC2420TransmitP$TimeStamp$default$receivedSFD(arg_0x4122ba68);
#line 50
}
#line 50
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline   error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureFallingEdge(void)
#line 54
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$enableCapture(MSP430TIMER_CM_FALLING);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static   error_t CC2420TransmitP$CaptureSFD$captureFallingEdge(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captureFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$get(void){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$getNow(void)
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$get();
}

#line 146
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$getNow(), dt);
}

# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$BackoffTimer$size_type arg_0x4092d460){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$start(arg_0x4092d460);
#line 55
}
#line 55
# 89 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketC.nc"
static inline   cc2420_header_t *CC2420PacketC$CC2420PacketBody$getHeader(message_t *msg)
#line 89
{
  return (cc2420_header_t *)(msg->data - sizeof(cc2420_header_t ));
}

# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_header_t *CC2420TransmitP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 93 "/opt/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketC.nc"
static inline   cc2420_metadata_t *CC2420PacketC$CC2420PacketBody$getMetadata(message_t *msg)
#line 93
{
  return (cc2420_metadata_t *)msg->metadata;
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_metadata_t *CC2420TransmitP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketC$CC2420PacketBody$getMetadata(arg_0x40ead0d0);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void)
{
  * (volatile uint16_t *)390U &= ~0x0010;
}

# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents(void){
#line 40
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents();
#line 40
}
#line 40
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$stop(void)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$stop(void){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$stop(void)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void CC2420TransmitP$BackoffTimer$stop(void){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$stop();
#line 62
}
#line 62
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420TransmitP$SpiResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP$Resource$release(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 639 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$releaseSpiResource(void)
#line 639
{
  CC2420TransmitP$SpiResource$release();
  return SUCCESS;
}

#line 695
static inline    void CC2420TransmitP$TimeStamp$default$transmittedSFD(uint16_t time, message_t *p_msg)
#line 695
{
}

# 39 "/opt/tinyos-2.x/tos/interfaces/RadioTimeStamping.nc"
inline static   void CC2420TransmitP$TimeStamp$transmittedSFD(uint16_t arg_0x4122b388, message_t *arg_0x4122b538){
#line 39
  CC2420TransmitP$TimeStamp$default$transmittedSFD(arg_0x4122b388, arg_0x4122b538);
#line 39
}
#line 39
# 259 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$CaptureSFD$captured(uint16_t time)
#line 259
{
  /* atomic removed: atomic calls only */
#line 260
  {
    switch (CC2420TransmitP$m_state) {

        case CC2420TransmitP$S_SFD: 
          CC2420TransmitP$m_state = CC2420TransmitP$S_EFD;
        CC2420TransmitP$sfdHigh = TRUE;
        CC2420TransmitP$CaptureSFD$captureFallingEdge();
        CC2420TransmitP$TimeStamp$transmittedSFD(time, CC2420TransmitP$m_msg);
        if (__nesc_ntoh_leuint16((unsigned char *)&CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg)->fcf) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP$abortSpiRelease = TRUE;
          }
        CC2420TransmitP$releaseSpiResource();
        CC2420TransmitP$BackoffTimer$stop();


        if (((__nesc_ntoh_leuint16((unsigned char *)&CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg)->fcf) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {
            __nesc_hton_uint16((unsigned char *)&CC2420TransmitP$CC2420PacketBody$getMetadata(CC2420TransmitP$m_msg)->time, time);
          }

        if (CC2420TransmitP$SFD$get()) {
            break;
          }


        case CC2420TransmitP$S_EFD: 
          CC2420TransmitP$sfdHigh = FALSE;
        CC2420TransmitP$CaptureSFD$captureRisingEdge();

        if (__nesc_ntoh_leuint16((unsigned char *)&CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg)->fcf) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP$m_state = CC2420TransmitP$S_ACK_WAIT;
            CC2420TransmitP$BackoffTimer$start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 292
          {
            CC2420TransmitP$signalDone(SUCCESS);
          }

        if (!CC2420TransmitP$SFD$get()) {
            break;
          }


        default: 
          if (!CC2420TransmitP$m_receiving) {
              CC2420TransmitP$sfdHigh = TRUE;
              CC2420TransmitP$CaptureSFD$captureFallingEdge();
              CC2420TransmitP$TimeStamp$receivedSFD(time);
              CC2420TransmitP$CC2420Receive$sfd(time);
              CC2420TransmitP$m_receiving = TRUE;
              CC2420TransmitP$m_prev_time = time;
              if (CC2420TransmitP$SFD$get()) {

                  return;
                }
            }

        CC2420TransmitP$sfdHigh = FALSE;
        CC2420TransmitP$CaptureSFD$captureRisingEdge();
        CC2420TransmitP$m_receiving = FALSE;
        if (time - CC2420TransmitP$m_prev_time < 10) {
            CC2420TransmitP$CC2420Receive$sfd_dropped();
          }
        break;
      }
  }
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captured(uint16_t arg_0x41011ea0){
#line 50
  CC2420TransmitP$CaptureSFD$captured(arg_0x41011ea0);
#line 50
}
#line 50
# 164 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$clearOverflow(void)
{
  * (volatile uint16_t *)388U &= ~0x0002;
}

# 57 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$clearOverflow(void){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$clearOverflow();
#line 57
}
#line 57
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$clearPendingInterrupt(void)
{
  * (volatile uint16_t *)388U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$clearPendingInterrupt(void){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$captured(time);
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$__nesc_unnamed4384 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(* (volatile uint16_t *)388U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired();
    }
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)31U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectModuleFunc();
#line 78
}
#line 78
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$__nesc_unnamed4385 {
#line 46
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$CC2int(x);
}

#line 99
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t *)388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$captureControl(cm);
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$setControlAsCapture(bool arg_0x40865958){
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$setControlAsCapture(arg_0x40865958);
#line 37
}
#line 37
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$enableEvents(void)
{
  * (volatile uint16_t *)388U |= 0x0010;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$enableEvents(void){
#line 39
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$enableEvents();
#line 39
}
#line 39
# 118 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline   void StateImplP$State$toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP$state[id] = StateImplP$S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   void CC2420SpiP$WorkingState$toIdle(void){
#line 56
  StateImplP$State$toIdle(0U);
#line 56
}
#line 56
# 93 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   void CC2420SpiP$ChipSpiResource$abortRelease(void)
#line 93
{
  /* atomic removed: atomic calls only */
#line 94
  CC2420SpiP$release = FALSE;
}

# 31 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static   void CC2420TransmitP$ChipSpiResource$abortRelease(void){
#line 31
  CC2420SpiP$ChipSpiResource$abortRelease();
#line 31
}
#line 31
# 328 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$ChipSpiResource$releasing(void)
#line 328
{
  if (CC2420TransmitP$abortSpiRelease) {
      CC2420TransmitP$ChipSpiResource$abortRelease();
    }
}

# 24 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static   void CC2420SpiP$ChipSpiResource$releasing(void){
#line 24
  CC2420TransmitP$ChipSpiResource$releasing();
#line 24
}
#line 24
# 151 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$resetUsart(bool arg_0x410d7c08){
#line 97
  HplMsp430Usart0P$Usart$resetUsart(arg_0x410d7c08);
#line 97
}
#line 97
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$UCLK$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$SOMI$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$SIMO$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectIOFunc();
#line 85
}
#line 85
# 247 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$disableSpi(void)
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P$ME1 &= ~(1 << 6);
    HplMsp430Usart0P$SIMO$selectIOFunc();
    HplMsp430Usart0P$SOMI$selectIOFunc();
    HplMsp430Usart0P$UCLK$selectIOFunc();
  }
}

# 158 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableSpi(void){
#line 158
  HplMsp430Usart0P$Usart$disableSpi();
#line 158
}
#line 158
# 88 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$unconfigure(uint8_t id)
#line 88
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$resetUsart(FALSE);
}

# 201 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$unconfigure(uint8_t id)
#line 201
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$unconfigure(uint8_t arg_0x40b9b4d8){
#line 55
  switch (arg_0x40b9b4d8) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID:
#line 55
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$unconfigure(arg_0x40b9b4d8);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 191 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$granted(void)
#line 191
{
}

# 46 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$granted(void){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$granted();
#line 46
}
#line 46
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void)
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead;

#line 62
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$dequeue(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void)
#line 50
{
  return /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$isEmpty(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 107 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$release(uint8_t id)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$postTask();
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$granted();
          }
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$unconfigure(id);
      }
  }
  return FAIL;
}

# 113 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$release(uint8_t id)
#line 113
{
#line 113
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$release(uint8_t arg_0x410e1e28){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x410e1e28) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 110
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$release(arg_0x410e1e28);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 80 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$release(uint8_t id)
#line 80
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$release(id);
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420SpiP$SpiResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$size_type /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$get(void){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53







inline static   bool /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void)
{
  * (volatile uint16_t *)390U |= 0x0010;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents(void){
#line 39
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents();
#line 39
}
#line 39
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void)
{
  * (volatile uint16_t *)390U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt(void){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)406U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(uint16_t arg_0x4085fed0){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(arg_0x4085fed0);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(uint16_t arg_0x4085e868){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(arg_0x4085e868);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$size_type arg_0x4092b598, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$size_type arg_0x4092b728){
#line 92
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(arg_0x4092b598, arg_0x4092b728);
#line 92
}
#line 92
# 100 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   error_t CC2420SpiP$ChipSpiResource$attemptRelease(void)
#line 100
{
  return CC2420SpiP$attemptRelease();
}

# 39 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static   error_t CC2420TransmitP$ChipSpiResource$attemptRelease(void){
#line 39
  unsigned char result;
#line 39

#line 39
  result = CC2420SpiP$ChipSpiResource$attemptRelease();
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420CsmaP$sendDone_task$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$sendDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 194 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline   void CC2420CsmaP$CC2420Transmit$sendDone(message_t *p_msg, error_t err)
#line 194
{
  /* atomic removed: atomic calls only */
#line 195
  CC2420CsmaP$sendErr = err;
  CC2420CsmaP$sendDone_task$postTask();
}

# 73 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static   void CC2420TransmitP$Send$sendDone(message_t *arg_0x40efc720, error_t arg_0x40efc8a8){
#line 73
  CC2420CsmaP$CC2420Transmit$sendDone(arg_0x40efc720, arg_0x40efc8a8);
#line 73
}
#line 73
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ControlP$SpiResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP$Resource$request(/*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 156 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$Resource$request(void)
#line 156
{
  return CC2420ControlP$SpiResource$request();
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420CsmaP$Resource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420ControlP$Resource$request();
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 199 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline   void CC2420CsmaP$CC2420Power$startVRegDone(void)
#line 199
{
  CC2420CsmaP$Resource$request();
}

# 56 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   void CC2420ControlP$CC2420Power$startVRegDone(void){
#line 56
  CC2420CsmaP$CC2420Power$startVRegDone();
#line 56
}
#line 56
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$RSTN$set(void){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$set();
#line 29
}
#line 29
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$RSTN$clr(void){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$clr();
#line 30
}
#line 30
# 380 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   void CC2420ControlP$StartupTimer$fired(void)
#line 380
{
  if (CC2420ControlP$m_state == CC2420ControlP$S_VREG_STARTING) {
      CC2420ControlP$m_state = CC2420ControlP$S_VREG_STARTED;
      CC2420ControlP$RSTN$clr();
      CC2420ControlP$RSTN$set();
      CC2420ControlP$CC2420Power$startVRegDone();
    }
}

# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420TransmitP$SFLUSHTX$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SFLUSHTX);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$getRaw(void)
#line 48
{
#line 48
  return * (volatile uint8_t *)32U & (0x01 << 4);
}

#line 49
static inline   bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$get(void)
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   bool /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$get(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$get(void)
#line 40
{
#line 40
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   bool CC2420TransmitP$CCA$get(void){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 458 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$BackoffTimer$fired(void)
#line 458
{
  /* atomic removed: atomic calls only */
#line 459
  {
    switch (CC2420TransmitP$m_state) {

        case CC2420TransmitP$S_SAMPLE_CCA: 


          if (CC2420TransmitP$CCA$get()) {
              CC2420TransmitP$m_state = CC2420TransmitP$S_BEGIN_TRANSMIT;
              CC2420TransmitP$BackoffTimer$start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP$congestionBackoff();
            }
        break;

        case CC2420TransmitP$S_CCA_CANCEL: 
          CC2420TransmitP$m_state = CC2420TransmitP$S_TX_CANCEL;


        case CC2420TransmitP$S_BEGIN_TRANSMIT: 
          case CC2420TransmitP$S_TX_CANCEL: 
            if (CC2420TransmitP$acquireSpiResource() == SUCCESS) {
                CC2420TransmitP$attemptSend();
              }
        break;

        case CC2420TransmitP$S_ACK_WAIT: 
          CC2420TransmitP$signalDone(SUCCESS);
        break;

        case CC2420TransmitP$S_SFD: 


          CC2420TransmitP$SFLUSHTX$strobe();
        CC2420TransmitP$CaptureSFD$captureRisingEdge();
        CC2420TransmitP$releaseSpiResource();
        CC2420TransmitP$signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$fired(void){
#line 67
  CC2420TransmitP$BackoffTimer$fired();
#line 67
  CC2420ControlP$StartupTimer$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$fired(void)
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired(void){
#line 67
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$fired();
#line 67
}
#line 67
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void){
#line 34
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void)
{
  return * (volatile uint16_t *)406U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$__nesc_unnamed4386 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(* (volatile uint16_t *)390U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired();
    }
}

# 246 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_leuint8(const void *source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 283 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline    void CC2420CsmaP$RadioBackoff$default$requestCongestionBackoff(am_id_t amId, 
message_t *msg)
#line 284
{
}

# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420CsmaP$RadioBackoff$requestCongestionBackoff(am_id_t arg_0x40f0a320, message_t *arg_0x40e9c748){
#line 88
    CC2420CsmaP$RadioBackoff$default$requestCongestionBackoff(arg_0x40f0a320, arg_0x40e9c748);
#line 88
}
#line 88
# 78 "/opt/tinyos-2.x/tos/system/RandomMlcgP.nc"
static inline   uint16_t RandomMlcgP$Random$rand16(void)
#line 78
{
  return (uint16_t )RandomMlcgP$Random$rand32();
}

# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static   uint16_t CC2420CsmaP$Random$rand16(void){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgP$Random$rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 237 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t backoffTime)
#line 237
{
  CC2420TransmitP$myCongestionBackoff = backoffTime + 1;
}

# 66 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420CsmaP$SubBackoff$setCongestionBackoff(uint16_t arg_0x40e9e688){
#line 66
  CC2420TransmitP$RadioBackoff$setCongestionBackoff(arg_0x40e9e688);
#line 66
}
#line 66
# 220 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline   void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t *msg)
#line 220
{
  CC2420CsmaP$SubBackoff$setCongestionBackoff(CC2420CsmaP$Random$rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP$RadioBackoff$requestCongestionBackoff(__nesc_ntoh_leuint8((unsigned char *)&((cc2420_header_t *)(msg->data - 
  sizeof(cc2420_header_t )))->type), msg);
}

# 88 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420TransmitP$RadioBackoff$requestCongestionBackoff(message_t *arg_0x40e9c748){
#line 88
  CC2420CsmaP$SubBackoff$requestCongestionBackoff(arg_0x40e9c748);
#line 88
}
#line 88
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420TransmitP$SpiResource$immediateRequest(void){
#line 87
  unsigned char result;
#line 87

#line 87
  result = CC2420SpiP$Resource$immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   error_t CC2420SpiP$WorkingState$requestState(uint8_t arg_0x40f27230){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP$State$requestState(0U, arg_0x40f27230);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 110 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$isOwner(uint8_t id)
#line 110
{
#line 110
  return FAIL;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$isOwner(uint8_t arg_0x410e1e28){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x410e1e28) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 118
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$isOwner(arg_0x410e1e28);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 76 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$isOwner(uint8_t id)
#line 76
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$isOwner(id);
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool CC2420SpiP$SpiResource$isOwner(void){
#line 118
  unsigned char result;
#line 118

#line 118
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 114 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$default$getConfig(uint8_t id)
#line 114
{
  return &msp430_spi_default_config;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static   msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$getConfig(uint8_t arg_0x410e07a0){
#line 39
  union __nesc_unnamed4284 *result;
#line 39

#line 39
    result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$default$getConfig(arg_0x410e07a0);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 168 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$setModeSpi(msp430_spi_union_config_t *arg_0x410d3d50){
#line 168
  HplMsp430Usart0P$Usart$setModeSpi(arg_0x410d3d50);
#line 168
}
#line 168
# 84 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$configure(uint8_t id)
#line 84
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Msp430SpiConfigure$getConfig(id));
}

# 199 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$configure(uint8_t id)
#line 199
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$configure(uint8_t arg_0x40b9b4d8){
#line 49
  switch (arg_0x40b9b4d8) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$ResourceConfigure$configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$default$configure(arg_0x40b9b4d8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 196 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$immediateRequested(void)
#line 196
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$release();
}

# 81 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$immediateRequested(void){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$immediateRequested();
#line 81
}
#line 81
# 189 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$immediateRequested(uint8_t id)
#line 189
{
}

# 51 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$immediateRequested(uint8_t arg_0x40b9c308){
#line 51
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$immediateRequested(arg_0x40b9c308);
#line 51
}
#line 51
# 89 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$immediateRequest(uint8_t id)
#line 89
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId);
  /* atomic removed: atomic calls only */
#line 91
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 96
        FAIL;

#line 96
        return __nesc_temp;
      }
  }
#line 98
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 103
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_CONTROLLED;
  return FAIL;
}

# 112 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$immediateRequest(uint8_t id)
#line 112
{
#line 112
  return FAIL;
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$immediateRequest(uint8_t arg_0x410e1e28){
#line 87
  unsigned char result;
#line 87

#line 87
  switch (arg_0x410e1e28) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 87
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$immediateRequest(arg_0x410e1e28);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 68 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$immediateRequest(uint8_t id)
#line 68
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$immediateRequest(id);
}

# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420SpiP$SpiResource$immediateRequest(void){
#line 87
  unsigned char result;
#line 87

#line 87
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 97 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void HplMsp430I2C0P$HplUsart$resetUsart(bool arg_0x410d7c08){
#line 97
  HplMsp430Usart0P$Usart$resetUsart(arg_0x410d7c08);
#line 97
}
#line 97
# 59 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline   void HplMsp430I2C0P$HplI2C$clearModeI2C(void)
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P$U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P$HplUsart$resetUsart(TRUE);
  }
}

# 7 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static   void HplMsp430Usart0P$HplI2C$clearModeI2C(void){
#line 7
  HplMsp430I2C0P$HplI2C$clearModeI2C();
#line 7
}
#line 7
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP$21$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$URXD$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP$21$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP$20$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$UTXD$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP$20$IO$selectIOFunc();
#line 85
}
#line 85
# 207 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$disableUart(void)
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P$ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P$UTXD$selectIOFunc();
    HplMsp430Usart0P$URXD$selectIOFunc();
  }
}

#line 143
static inline   void HplMsp430Usart0P$Usart$setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline   void HplMsp430Usart0P$Usart$setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P$configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P$U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P$Usart$setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P$Usart$setUmctl(0x00);
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)27U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$UCLK$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP$19$IO$selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)27U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$SOMI$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP$18$IO$selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)27U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void HplMsp430Usart0P$SIMO$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP$17$IO$selectModuleFunc();
#line 78
}
#line 78
# 238 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$enableSpi(void)
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P$SIMO$selectModuleFunc();
    HplMsp430Usart0P$SOMI$selectModuleFunc();
    HplMsp430Usart0P$UCLK$selectModuleFunc();
  }
  HplMsp430Usart0P$ME1 |= 1 << 6;
}

#line 345
static inline   void HplMsp430Usart0P$Usart$clrIntr(void)
#line 345
{
  HplMsp430Usart0P$IFG1 &= ~((1 << 7) | (1 << 6));
}









static inline   void HplMsp430Usart0P$Usart$disableIntr(void)
#line 357
{
  HplMsp430Usart0P$IE1 &= ~((1 << 7) | (1 << 6));
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420TransmitP$SpiResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP$Resource$request(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 193 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$requested(void)
#line 193
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$release();
}

# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$requested(void){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$default$requested();
#line 73
}
#line 73
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t id)
#line 54
{
  return /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qTail == id;
}

#line 72
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qHead = id;
          }
        else {
#line 78
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qTail] = id;
          }
#line 79
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$enqueue(resource_client_id_t arg_0x40b828b0){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(arg_0x40b828b0);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 187 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$requested(uint8_t id)
#line 187
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$requested(uint8_t arg_0x40b9c308){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$default$requested(arg_0x40b9c308);
#line 43
}
#line 43
# 76 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$request(uint8_t id)
#line 76
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceRequested$requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId);
  /* atomic removed: atomic calls only */
#line 78
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 83
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Queue$enqueue(id);

#line 83
        return __nesc_temp;
      }
  }
#line 85
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$requested();
  return SUCCESS;
}

# 111 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$request(uint8_t id)
#line 111
{
#line 111
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$request(uint8_t arg_0x410e1e28){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x410e1e28) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 78
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$default$request(arg_0x410e1e28);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 72 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$request(uint8_t id)
#line 72
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$request(id);
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420SpiP$SpiResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 382 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P$U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$tx(uint8_t arg_0x410ce010){
#line 224
  HplMsp430Usart0P$Usart$tx(arg_0x410ce010);
#line 224
}
#line 224
# 330 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   bool HplMsp430Usart0P$Usart$isRxIntrPending(void)
#line 330
{
  if (HplMsp430Usart0P$IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isRxIntrPending(void){
#line 192
  unsigned char result;
#line 192

#line 192
  result = HplMsp430Usart0P$Usart$isRxIntrPending();
#line 192

#line 192
  return result;
#line 192
}
#line 192
# 341 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$clrRxIntr(void)
#line 341
{
  HplMsp430Usart0P$IFG1 &= ~(1 << 6);
}

# 197 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrRxIntr(void){
#line 197
  HplMsp430Usart0P$Usart$clrRxIntr();
#line 197
}
#line 197
#line 231
inline static   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$rx(void){
#line 231
  unsigned char result;
#line 231

#line 231
  result = HplMsp430Usart0P$Usart$rx();
#line 231

#line 231
  return result;
#line 231
}
#line 231
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420TransmitP$STXONCCA$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_STXONCCA);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static   cc2420_status_t CC2420TransmitP$STXON$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_STXON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static   cc2420_status_t CC2420TransmitP$SNOP$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SNOP);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 181 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void)
{
  return * (volatile uint16_t *)408U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$__nesc_unnamed4387 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(* (volatile uint16_t *)392U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void)
{
  return * (volatile uint16_t *)410U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$__nesc_unnamed4388 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(* (volatile uint16_t *)394U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void)
{
  return * (volatile uint16_t *)412U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$__nesc_unnamed4389 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(* (volatile uint16_t *)396U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void)
{
  return * (volatile uint16_t *)414U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t arg_0x40872358){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(arg_0x40872358);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$__nesc_unnamed4390 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(* (volatile uint16_t *)398U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired();
    }
}

# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void)
{
  uint8_t n = * (volatile uint16_t *)286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerB1$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline  void SchedulerBasicP$Scheduler$init(void)
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  void RealMainP$Scheduler$init(void){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 6;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led2$set(void){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 5;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$set(void){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 4;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$set(void){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led2$makeOutput(void){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$makeOutput(void){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$makeOutput(void){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline  error_t LedsP$Init$init(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$LedsInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 33 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_SET_SIMO0_PIN(void)
#line 33
{
#line 33
   static volatile uint8_t r __asm ("0x0019");

#line 33
  r |= 1 << 1;
}

#line 34
static inline void TOSH_SET_UCLK0_PIN(void)
#line 34
{
#line 34
   static volatile uint8_t r __asm ("0x0019");

#line 34
  r |= 1 << 3;
}

#line 102
static inline void TOSH_SET_FLASH_CS_PIN(void)
#line 102
{
#line 102
   static volatile uint8_t r __asm ("0x001D");

#line 102
  r |= 1 << 4;
}

#line 34
static inline void TOSH_CLR_UCLK0_PIN(void)
#line 34
{
#line 34
   static volatile uint8_t r __asm ("0x0019");

#line 34
  r &= ~(1 << 3);
}

#line 102
static inline void TOSH_CLR_FLASH_CS_PIN(void)
#line 102
{
#line 102
   static volatile uint8_t r __asm ("0x001D");

#line 102
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.x/tos/platforms/UBee430/MotePlatformC.nc"
static __inline void MotePlatformC$TOSH_wait(void)
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 103 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_SET_FLASH_HOLD_PIN(void)
#line 103
{
#line 103
   static volatile uint8_t r __asm ("0x001D");

#line 103
  r |= 1 << 7;
}

#line 102
static inline void TOSH_MAKE_FLASH_CS_OUTPUT(void)
#line 102
{
#line 102
   static volatile uint8_t r __asm ("0x001E");

#line 102
  r |= 1 << 4;
}

#line 103
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT(void)
#line 103
{
#line 103
   static volatile uint8_t r __asm ("0x001E");

#line 103
  r |= 1 << 7;
}

#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT(void)
#line 34
{
#line 34
   static volatile uint8_t r __asm ("0x001A");

#line 34
  r |= 1 << 3;
}

#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT(void)
#line 33
{
#line 33
   static volatile uint8_t r __asm ("0x001A");

#line 33
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.x/tos/platforms/UBee430/MotePlatformC.nc"
static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void)
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC$TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC$uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline  error_t MotePlatformC$Init$init(void)
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1DIR = 0xe0;
    P1OUT = 0x00;

    P2DIR = 0x7b;
    P2OUT = 0x30;

    P3DIR = 0xf1;
    P3OUT = 0x00;

    P4DIR = 0xfd;
    P4OUT = 0xdd;

    P5DIR = 0xff;
    P5OUT = 0xff;

    P6DIR = 0xff;
    P6OUT = 0x00;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC$uwait(1024 * 10);

    MotePlatformC$TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$MoteInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MotePlatformC$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 148 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$startTimerB(void)
{

  Msp430ClockP$TBCTL = 0x0020 | (Msp430ClockP$TBCTL & ~(0x0020 | 0x0010));
}

#line 136
static inline void Msp430ClockP$startTimerA(void)
{

  Msp430ClockP$TA0CTL = 0x0020 | (Msp430ClockP$TA0CTL & ~(0x0020 | 0x0010));
}

#line 100
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void)
{
  TBR = 0;









  Msp430ClockP$TBCTL = 0x0100 | 0x0002;
}

#line 130
static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerB(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerB();
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initTimerB(void){
#line 32
  Msp430ClockP$Msp430ClockInit$default$initTimerB();
#line 32
}
#line 32
# 85 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void)
{
  TA0R = 0;









  Msp430ClockP$TA0CTL = 0x0200 | 0x0002;
}

#line 125
static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerA(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerA();
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initTimerA(void){
#line 31
  Msp430ClockP$Msp430ClockInit$default$initTimerA();
#line 31
}
#line 31
# 64 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void)
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP$IE1 &= ~(1 << 1);
}

#line 120
static inline   void Msp430ClockP$Msp430ClockInit$default$initClocks(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitClocks();
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initClocks(void){
#line 30
  Msp430ClockP$Msp430ClockInit$default$initClocks();
#line 30
}
#line 30
# 166 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP$busyCalibrateDco(void)
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP$test_calib_busywait_delta(calib | step) <= Msp430ClockP$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP$set_dco_calib(calib);
}

#line 52
static inline  void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void)
{



  Msp430ClockP$TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 115
static inline   void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void)
{
  Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate();
}

# 29 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void){
#line 29
  Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate();
#line 29
}
#line 29
# 214 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline  error_t Msp430ClockP$Init$init(void)
{

  Msp430ClockP$TA0CTL = 0x0004;
  Msp430ClockP$TA0IV = 0;
  Msp430ClockP$TBCTL = 0x0004;
  Msp430ClockP$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP$Msp430ClockInit$setupDcoCalibrate();
    Msp430ClockP$busyCalibrateDco();
    Msp430ClockP$Msp430ClockInit$initClocks();
    Msp430ClockP$Msp430ClockInit$initTimerA();
    Msp430ClockP$Msp430ClockInit$initTimerB();
    Msp430ClockP$startTimerA();
    Msp430ClockP$startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$MoteClockInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 10 "/opt/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline  error_t PlatformP$Init$init(void)
#line 10
{
  PlatformP$MoteClockInit$init();
  PlatformP$MoteInit$init();
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t RealMainP$PlatformInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 33 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_CLR_SIMO0_PIN(void)
#line 33
{
#line 33
   static volatile uint8_t r __asm ("0x0019");

#line 33
  r &= ~(1 << 1);
}

# 54 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  bool RealMainP$Scheduler$runNextTask(void){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_hton_uint8(void *target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

#line 257
static __inline int8_t __nesc_hton_int8(void *target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

# 210 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline   message_t *CC2420ActiveMessageP$Snoop$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 210
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static  message_t *CC2420ActiveMessageP$Snoop$receive(am_id_t arg_0x40ebc320, message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
    result = CC2420ActiveMessageP$Snoop$default$receive(arg_0x40ebc320, arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 196 "UBee430_APC.nc"
static inline  message_t *UBee430_APC$Receive$receive(message_t *buf, void *payload, uint8_t len)
#line 196
{
  return buf;
}

# 206 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline   message_t *CC2420ActiveMessageP$Receive$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 206
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static  message_t *CC2420ActiveMessageP$Receive$receive(am_id_t arg_0x40ebdab8, message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  switch (arg_0x40ebdab8) {
#line 67
    case AM_MSG:
#line 67
      result = UBee430_APC$Receive$receive(arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67
      break;
#line 67
    default:
#line 67
      result = CC2420ActiveMessageP$Receive$default$receive(arg_0x40ebdab8, arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_header_t *CC2420ActiveMessageP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 107 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  am_addr_t CC2420ActiveMessageP$AMPacket$destination(message_t *amsg)
#line 107
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 109
  return __nesc_ntoh_leuint16((unsigned char *)&header->dest);
}

# 61 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline   am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void)
#line 61
{
  return ActiveMessageAddressC$amAddress();
}

# 48 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static   am_addr_t CC2420ActiveMessageP$ActiveMessageAddress$amAddress(void){
#line 48
  unsigned int result;
#line 48

#line 48
  result = ActiveMessageAddressC$ActiveMessageAddress$amAddress();
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 103 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  am_addr_t CC2420ActiveMessageP$AMPacket$address(void)
#line 103
{
  return CC2420ActiveMessageP$ActiveMessageAddress$amAddress();
}

#line 127
static inline  bool CC2420ActiveMessageP$AMPacket$isForMe(message_t *amsg)
#line 127
{
  return CC2420ActiveMessageP$AMPacket$destination(amsg) == CC2420ActiveMessageP$AMPacket$address() || 
  CC2420ActiveMessageP$AMPacket$destination(amsg) == AM_BROADCAST_ADDR;
}

#line 187
static inline  message_t *CC2420ActiveMessageP$SubReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 187
{
  if (CC2420ActiveMessageP$AMPacket$isForMe(msg)) {
      return CC2420ActiveMessageP$Receive$receive(CC2420ActiveMessageP$AMPacket$type(msg), msg, payload, len - CC2420ActiveMessageP$CC2420_SIZE);
    }
  else {
      return CC2420ActiveMessageP$Snoop$receive(CC2420ActiveMessageP$AMPacket$type(msg), msg, payload, len - CC2420ActiveMessageP$CC2420_SIZE);
    }
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static  message_t *UniqueReceiveP$Receive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = CC2420ActiveMessageP$SubReceive$receive(arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 156 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn)
#line 156
{
  uint8_t element = UniqueReceiveP$recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 160
    {
      if (element == UniqueReceiveP$INVALID_ELEMENT || UniqueReceiveP$writeIndex == element) {

          element = UniqueReceiveP$writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP$receivedMessages[element].source = msgSource;
      UniqueReceiveP$receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP$writeIndex++;
          UniqueReceiveP$writeIndex %= 4;
        }
    }
#line 173
    __nesc_atomic_end(__nesc_atomic); }
}


static inline   message_t *UniqueReceiveP$DuplicateReceive$default$receive(message_t *msg, void *payload, uint8_t len)
#line 177
{
  return msg;
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static  message_t *UniqueReceiveP$DuplicateReceive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = UniqueReceiveP$DuplicateReceive$default$receive(arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 130 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 130
{
  int i;

#line 132
  UniqueReceiveP$recycleSourceElement = UniqueReceiveP$INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP$receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP$receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 139
                    TRUE;

                    {
#line 139
                      __nesc_atomic_end(__nesc_atomic); 
#line 139
                      return __nesc_temp;
                    }
                  }
                }
#line 142
              UniqueReceiveP$recycleSourceElement = i;
            }
        }
    }
#line 145
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_header_t *UniqueReceiveP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 104 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline  message_t *UniqueReceiveP$SubReceive$receive(message_t *msg, void *payload, 
uint8_t len)
#line 105
{
  uint16_t msgSource = __nesc_ntoh_leuint16((unsigned char *)&UniqueReceiveP$CC2420PacketBody$getHeader(msg)->src);
  uint8_t msgDsn = __nesc_ntoh_leuint8((unsigned char *)&UniqueReceiveP$CC2420PacketBody$getHeader(msg)->dsn);

  if (UniqueReceiveP$hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP$DuplicateReceive$receive(msg, payload, len);
    }
  else {
      UniqueReceiveP$insert(msgSource, msgDsn);
      return UniqueReceiveP$Receive$receive(msg, payload, len);
    }
}

# 67 "/opt/tinyos-2.x/tos/interfaces/Receive.nc"
inline static  message_t *CC2420ReceiveP$Receive$receive(message_t *arg_0x4065a780, void *arg_0x4065a920, uint8_t arg_0x4065aaa8){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = UniqueReceiveP$SubReceive$receive(arg_0x4065a780, arg_0x4065a920, arg_0x4065aaa8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_header_t *CC2420ReceiveP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42





inline static   cc2420_metadata_t *CC2420ReceiveP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketC$CC2420PacketBody$getMetadata(arg_0x40ead0d0);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 333 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  void CC2420ReceiveP$receiveDone_task$runTask(void)
#line 333
{
  cc2420_metadata_t *metadata = CC2420ReceiveP$CC2420PacketBody$getMetadata(CC2420ReceiveP$m_p_rx_buf);
  uint8_t *buf = (uint8_t *)CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf);

#line 335
  ;

  __nesc_hton_int8((unsigned char *)&metadata->crc, buf[CC2420ReceiveP$rxFrameLength] >> 7);
  __nesc_hton_uint8((unsigned char *)&metadata->rssi, buf[CC2420ReceiveP$rxFrameLength - 1]);
  __nesc_hton_uint8((unsigned char *)&metadata->lqi, buf[CC2420ReceiveP$rxFrameLength] & 0x7f);
  CC2420ReceiveP$m_p_rx_buf = CC2420ReceiveP$Receive$receive(CC2420ReceiveP$m_p_rx_buf, CC2420ReceiveP$m_p_rx_buf->data, 
  CC2420ReceiveP$rxFrameLength);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 343
    CC2420ReceiveP$receivingPacket = FALSE;
#line 343
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP$waitForNextPacket();
}

# 176 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   uint8_t CC2420SpiP$Resource$isOwner(uint8_t id)
#line 176
{
  /* atomic removed: atomic calls only */
#line 177
  {
    unsigned char __nesc_temp = 
#line 177
    CC2420SpiP$m_holder == id;

#line 177
    return __nesc_temp;
  }
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool CC2420ReceiveP$SpiResource$isOwner(void){
#line 118
  unsigned char result;
#line 118

#line 118
  result = CC2420SpiP$Resource$isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 361 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$enableRxIntr(void)
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P$IFG1 &= ~(1 << 6);
      HplMsp430Usart0P$IE1 |= 1 << 6;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$enableRxIntr(void){
#line 180
  HplMsp430Usart0P$Usart$enableRxIntr();
#line 180
}
#line 180
# 316 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   bool HplMsp430Usart0P$Usart$isTxIntrPending(void)
#line 316
{
  if (HplMsp430Usart0P$IFG1 & (1 << 7)) {
      return TRUE;
    }
  return FALSE;
}

# 187 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isTxIntrPending(void){
#line 187
  unsigned char result;
#line 187

#line 187
  result = HplMsp430Usart0P$Usart$isTxIntrPending();
#line 187

#line 187
  return result;
#line 187
}
#line 187
# 337 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$clrTxIntr(void)
#line 337
{
  HplMsp430Usart0P$IFG1 &= ~(1 << 7);
}

# 202 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrTxIntr(void){
#line 202
  HplMsp430Usart0P$Usart$clrTxIntr();
#line 202
}
#line 202
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 87 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ReceiveP$SpiResource$immediateRequest(void){
#line 87
  unsigned char result;
#line 87

#line 87
  result = CC2420SpiP$Resource$immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
#line 78
inline static   error_t CC2420ReceiveP$SpiResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP$Resource$request(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420SpiP$grant$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420SpiP$grant);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 182 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline  void CC2420SpiP$SpiResource$granted(void)
#line 182
{
  CC2420SpiP$grant$postTask();
}

# 118 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$default$granted(uint8_t id)
#line 118
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$granted(uint8_t arg_0x410e23a8){
#line 92
  switch (arg_0x410e23a8) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 92
      CC2420SpiP$SpiResource$granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$default$granted(arg_0x410e23a8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 94 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$granted(uint8_t id)
#line 94
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Resource$granted(id);
}

# 185 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$default$granted(uint8_t id)
#line 185
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$granted(uint8_t arg_0x40b9d968){
#line 92
  switch (arg_0x40b9d968) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartResource$granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$default$granted(arg_0x40b9d968);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 173 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline  void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$runTask(void)
#line 173
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 174
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_BUSY;
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceConfigure$configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId);
}

# 192 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline    void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$default$sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 192
{
}

# 71 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$sendDone(uint8_t arg_0x410e1740, uint8_t *arg_0x41040b98, uint8_t *arg_0x41040d40, uint16_t arg_0x41040ed0, error_t arg_0x4103f088){
#line 71
  switch (arg_0x410e1740) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID:
#line 71
      CC2420SpiP$SpiPacket$sendDone(arg_0x41040b98, arg_0x41040d40, arg_0x41040ed0, arg_0x4103f088);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$default$sendDone(arg_0x410e1740, arg_0x41040b98, arg_0x41040d40, arg_0x41040ed0, arg_0x4103f088);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 185 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone(void)
#line 185
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len, 
  SUCCESS);
}

#line 168
static inline  void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$runTask(void)
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone();
#line 169
    __nesc_atomic_end(__nesc_atomic); }
}

# 446 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$TXFIFO$readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 447
{
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ReceiveP$SpiResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP$Resource$release(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ReceiveP$CSN$set(void){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set();
#line 29
}
#line 29
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420ReceiveP$receiveDone_task$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420ReceiveP$receiveDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 340 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$CC2420Receive$receive(uint8_t type, message_t *ack_msg)
#line 340
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK) {
      ack_header = CC2420TransmitP$CC2420PacketBody$getHeader(ack_msg);
      msg_header = CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg);


      if (CC2420TransmitP$m_state == CC2420TransmitP$S_ACK_WAIT && __nesc_ntoh_leuint8((unsigned char *)&msg_header->dsn) == __nesc_ntoh_leuint8((unsigned char *)&ack_header->dsn)) {
          CC2420TransmitP$BackoffTimer$stop();

          msg_metadata = CC2420TransmitP$CC2420PacketBody$getMetadata(CC2420TransmitP$m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8((unsigned char *)&ack_header->length);

          __nesc_hton_int8((unsigned char *)&msg_metadata->ack, TRUE);
          __nesc_hton_uint8((unsigned char *)&msg_metadata->rssi, ack_buf[length - 1]);
          __nesc_hton_uint8((unsigned char *)&msg_metadata->lqi, ack_buf[length] & 0x7f);
          CC2420TransmitP$signalDone(SUCCESS);
        }
    }
}

# 61 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static   void CC2420ReceiveP$CC2420Receive$receive(uint8_t arg_0x41254b18, message_t *arg_0x41254cc8){
#line 61
  CC2420TransmitP$CC2420Receive$receive(arg_0x41254b18, arg_0x41254cc8);
#line 61
}
#line 61
# 59 "/opt/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static   error_t CC2420SpiP$SpiPacket$send(uint8_t *arg_0x41040118, uint8_t *arg_0x410402c0, uint16_t arg_0x41040450){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C$0$CLIENT_ID, arg_0x41040118, arg_0x410402c0, arg_0x41040450);
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 207 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   error_t CC2420SpiP$Fifo$continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 208
{
  return CC2420SpiP$SpiPacket$send((void *)0, data, len);
}

# 62 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static   error_t CC2420ReceiveP$RXFIFO$continueRead(uint8_t *arg_0x41021010, uint8_t arg_0x41021198){
#line 62
  unsigned char result;
#line 62

#line 62
  result = CC2420SpiP$Fifo$continueRead(CC2420_RXFIFO, arg_0x41021010, arg_0x41021198);
#line 62

#line 62
  return result;
#line 62
}
#line 62
#line 51
inline static   cc2420_status_t CC2420ReceiveP$RXFIFO$beginRead(uint8_t *arg_0x41022848, uint8_t arg_0x410229d0){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420SpiP$Fifo$beginRead(CC2420_RXFIFO, arg_0x41022848, arg_0x410229d0);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ReceiveP$CSN$clr(void){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr();
#line 30
}
#line 30
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420ReceiveP$SACK$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SACK);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 264 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   uint16_t CC2420ControlP$CC2420Config$getShortAddr(void)
#line 264
{
  /* atomic removed: atomic calls only */
#line 265
  {
    unsigned int __nesc_temp = 
#line 265
    CC2420ControlP$m_short_addr;

#line 265
    return __nesc_temp;
  }
}

# 64 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static   uint16_t CC2420ReceiveP$CC2420Config$getShortAddr(void){
#line 64
  unsigned int result;
#line 64

#line 64
  result = CC2420ControlP$CC2420Config$getShortAddr();
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 331 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void)
#line 331
{
  /* atomic removed: atomic calls only */
#line 332
  {
    unsigned char __nesc_temp = 
#line 332
    CC2420ControlP$hwAutoAckDefault;

#line 332
    return __nesc_temp;
  }
}

# 96 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static   bool CC2420ReceiveP$CC2420Config$isHwAutoAckDefault(void){
#line 96
  unsigned char result;
#line 96

#line 96
  result = CC2420ControlP$CC2420Config$isHwAutoAckDefault();
#line 96

#line 96
  return result;
#line 96
}
#line 96
# 338 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void)
#line 338
{
  /* atomic removed: atomic calls only */
#line 339
  {
    unsigned char __nesc_temp = 
#line 339
    CC2420ControlP$autoAckEnabled;

#line 339
    return __nesc_temp;
  }
}

# 101 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static   bool CC2420ReceiveP$CC2420Config$isAutoAckEnabled(void){
#line 101
  unsigned char result;
#line 101

#line 101
  result = CC2420ControlP$CC2420Config$isAutoAckEnabled();
#line 101

#line 101
  return result;
#line 101
}
#line 101
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$getRaw(void)
#line 48
{
#line 48
  return * (volatile uint8_t *)32U & (0x01 << 0);
}

#line 49
static inline   bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$get(void)
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$HplGeneralIO$get(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP$0$IO$get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$GeneralIO$get(void)
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$HplGeneralIO$get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   bool CC2420ReceiveP$FIFOP$get(void){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC$9$GeneralIO$get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$getRaw(void)
#line 48
{
#line 48
  return * (volatile uint8_t *)32U & (0x01 << 3);
}

#line 49
static inline   bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$get(void)
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$HplGeneralIO$get(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP$3$IO$get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$GeneralIO$get(void)
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$HplGeneralIO$get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   bool CC2420ReceiveP$FIFO$get(void){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC$8$GeneralIO$get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 203 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline   void CC2420ReceiveP$RXFIFO$readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 204
{
  cc2420_header_t *header = CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf);
  cc2420_metadata_t *metadata = CC2420ReceiveP$CC2420PacketBody$getMetadata(CC2420ReceiveP$m_p_rx_buf);
  uint8_t *buf = (uint8_t *)header;

#line 208
  CC2420ReceiveP$rxFrameLength = buf[0];

  switch (CC2420ReceiveP$m_state) {

      case CC2420ReceiveP$S_RX_LENGTH: 
        CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_FCF;
      if (CC2420ReceiveP$rxFrameLength + 1 > CC2420ReceiveP$m_bytes_left) {

          CC2420ReceiveP$flush();
        }
      else {
          if (!CC2420ReceiveP$FIFO$get() && !CC2420ReceiveP$FIFOP$get()) {
              CC2420ReceiveP$m_bytes_left -= CC2420ReceiveP$rxFrameLength + 1;
            }

          if (CC2420ReceiveP$rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP$rxFrameLength > 0) {
                  if (CC2420ReceiveP$rxFrameLength > CC2420ReceiveP$SACK_HEADER_LENGTH) {

                      CC2420ReceiveP$RXFIFO$continueRead(buf + 1, CC2420ReceiveP$SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_PAYLOAD;
                      CC2420ReceiveP$RXFIFO$continueRead(buf + 1, CC2420ReceiveP$rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP$receivingPacket = FALSE;
                  CC2420ReceiveP$CSN$set();
                  CC2420ReceiveP$SpiResource$release();
                  CC2420ReceiveP$waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP$flush();
            }
        }
      break;

      case CC2420ReceiveP$S_RX_FCF: 
        CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_PAYLOAD;










      if (CC2420ReceiveP$CC2420Config$isAutoAckEnabled() && !CC2420ReceiveP$CC2420Config$isHwAutoAckDefault()) {


          if (((__nesc_ntoh_leuint16((unsigned char *)&
#line 263
          header->fcf) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && __nesc_ntoh_leuint16((unsigned char *)&header->dest) == CC2420ReceiveP$CC2420Config$getShortAddr()
           && ((__nesc_ntoh_leuint16((unsigned char *)&header->fcf) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP$CSN$set();
              CC2420ReceiveP$CSN$clr();
              CC2420ReceiveP$SACK$strobe();
              CC2420ReceiveP$CSN$set();
              CC2420ReceiveP$CSN$clr();
              CC2420ReceiveP$RXFIFO$beginRead(buf + 1 + CC2420ReceiveP$SACK_HEADER_LENGTH, 
              CC2420ReceiveP$rxFrameLength - CC2420ReceiveP$SACK_HEADER_LENGTH);
              return;
            }
        }


      CC2420ReceiveP$RXFIFO$continueRead(buf + 1 + CC2420ReceiveP$SACK_HEADER_LENGTH, 
      CC2420ReceiveP$rxFrameLength - CC2420ReceiveP$SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP$S_RX_PAYLOAD: 
        CC2420ReceiveP$CSN$set();

      if (!CC2420ReceiveP$m_missed_packets) {

          CC2420ReceiveP$SpiResource$release();
        }

      if (CC2420ReceiveP$m_timestamp_size) {
          if (CC2420ReceiveP$rxFrameLength > 10) {
              __nesc_hton_uint16((unsigned char *)&metadata->time, CC2420ReceiveP$m_timestamp_queue[CC2420ReceiveP$m_timestamp_head]);
              CC2420ReceiveP$m_timestamp_head = (CC2420ReceiveP$m_timestamp_head + 1) % CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE;
              CC2420ReceiveP$m_timestamp_size--;
            }
        }
      else 
#line 297
        {
          __nesc_hton_uint16((unsigned char *)&metadata->time, 0xffff);
        }



      if (buf[CC2420ReceiveP$rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16((unsigned char *)&header->fcf) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 305
          CC2420ReceiveP$CC2420Receive$receive(type, CC2420ReceiveP$m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP$receiveDone_task$postTask();
              return;
            }
        }

      CC2420ReceiveP$waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP$receivingPacket = FALSE;
      CC2420ReceiveP$CSN$set();
      CC2420ReceiveP$SpiResource$release();
      break;
    }
}

# 366 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline    void CC2420SpiP$Fifo$default$readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 366
{
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static   void CC2420SpiP$Fifo$readDone(uint8_t arg_0x4104c408, uint8_t *arg_0x410217c8, uint8_t arg_0x41021950, error_t arg_0x41021ad8){
#line 71
  switch (arg_0x4104c408) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP$TXFIFO$readDone(arg_0x410217c8, arg_0x41021950, arg_0x41021ad8);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP$RXFIFO$readDone(arg_0x410217c8, arg_0x41021950, arg_0x41021ad8);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP$Fifo$default$readDone(arg_0x4104c408, arg_0x410217c8, arg_0x41021950, arg_0x41021ad8);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420ReceiveP$SFLUSHRX$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SFLUSHRX);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 279 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline    void CC2420CsmaP$RadioBackoff$default$requestInitialBackoff(am_id_t amId, 
message_t *msg)
#line 280
{
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420CsmaP$RadioBackoff$requestInitialBackoff(am_id_t arg_0x40f0a320, message_t *arg_0x40e9c190){
#line 81
    CC2420CsmaP$RadioBackoff$default$requestInitialBackoff(arg_0x40f0a320, arg_0x40e9c190);
#line 81
}
#line 81
# 229 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t backoffTime)
#line 229
{
  CC2420TransmitP$myInitialBackoff = backoffTime + 1;
}

# 60 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420CsmaP$SubBackoff$setInitialBackoff(uint16_t arg_0x40e9e108){
#line 60
  CC2420TransmitP$RadioBackoff$setInitialBackoff(arg_0x40e9e108);
#line 60
}
#line 60
# 212 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline   void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t *msg)
#line 212
{
  CC2420CsmaP$SubBackoff$setInitialBackoff(CC2420CsmaP$Random$rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP$RadioBackoff$requestInitialBackoff(__nesc_ntoh_leuint8((unsigned char *)&((cc2420_header_t *)(msg->data - 
  sizeof(cc2420_header_t )))->type), msg);
}

# 81 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420TransmitP$RadioBackoff$requestInitialBackoff(message_t *arg_0x40e9c190){
#line 81
  CC2420CsmaP$SubBackoff$requestInitialBackoff(arg_0x40e9c190);
#line 81
}
#line 81
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420TransmitP$CSN$set(void){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set();
#line 29
}
#line 29

inline static   void CC2420TransmitP$CSN$clr(void){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr();
#line 30
}
#line 30
# 407 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline   void CC2420TransmitP$TXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 408
{

  CC2420TransmitP$CSN$set();
  if (CC2420TransmitP$m_state == CC2420TransmitP$S_LOAD_CANCEL) {
      /* atomic removed: atomic calls only */
#line 412
      {
        CC2420TransmitP$CSN$clr();
        CC2420TransmitP$SFLUSHTX$strobe();
        CC2420TransmitP$CSN$set();
      }
      CC2420TransmitP$releaseSpiResource();
      CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
    }
  else {
#line 420
    if (!CC2420TransmitP$m_cca) {
        /* atomic removed: atomic calls only */
#line 421
        {
          if (CC2420TransmitP$m_state == CC2420TransmitP$S_LOAD_CANCEL) {
              CC2420TransmitP$m_state = CC2420TransmitP$S_TX_CANCEL;
            }
          else 
#line 424
            {
              CC2420TransmitP$m_state = CC2420TransmitP$S_BEGIN_TRANSMIT;
            }
        }
        CC2420TransmitP$attemptSend();
      }
    else {
        CC2420TransmitP$releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 432
        {
          if (CC2420TransmitP$m_state == CC2420TransmitP$S_LOAD_CANCEL) {
              CC2420TransmitP$m_state = CC2420TransmitP$S_CCA_CANCEL;
            }
          else 
#line 435
            {
              CC2420TransmitP$m_state = CC2420TransmitP$S_SAMPLE_CCA;
            }
        }

        CC2420TransmitP$RadioBackoff$requestInitialBackoff(CC2420TransmitP$m_msg);
        CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$myInitialBackoff);
      }
    }
}

# 325 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline   void CC2420ReceiveP$RXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 325
{
}

# 369 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline    void CC2420SpiP$Fifo$default$writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 369
{
}

# 91 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static   void CC2420SpiP$Fifo$writeDone(uint8_t arg_0x4104c408, uint8_t *arg_0x4101f838, uint8_t arg_0x4101f9c0, error_t arg_0x4101fb48){
#line 91
  switch (arg_0x4104c408) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP$TXFIFO$writeDone(arg_0x4101f838, arg_0x4101f9c0, arg_0x4101fb48);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP$RXFIFO$writeDone(arg_0x4101f838, arg_0x4101f9c0, arg_0x4101fb48);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP$Fifo$default$writeDone(arg_0x4104c408, arg_0x4101f838, arg_0x4101f9c0, arg_0x4101fb48);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static   cc2420_status_t CC2420ControlP$RXCTRL1$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_RXCTRL1, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static   cc2420_status_t CC2420ControlP$IOCFG0$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_IOCFG0, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420ControlP$SXOSCON$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SXOSCON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 79 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port14$enable(void)
#line 79
{
#line 79
  P1IE |= 1 << 4;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$enable(void){
#line 31
  HplMsp430InterruptP$Port14$enable();
#line 31
}
#line 31
# 131 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port14$edge(bool l2h)
#line 131
{
  /* atomic removed: atomic calls only */
#line 132
  {
    if (l2h) {
#line 133
      P1IES &= ~(1 << 4);
      }
    else {
#line 134
      P1IES |= 1 << 4;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$edge(bool arg_0x40aceef8){
#line 56
  HplMsp430InterruptP$Port14$edge(arg_0x40aceef8);
#line 56
}
#line 56
# 95 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port14$clear(void)
#line 95
{
#line 95
  P1IFG &= ~(1 << 4);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$clear(void){
#line 41
  HplMsp430InterruptP$Port14$clear();
#line 41
}
#line 41
# 87 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port14$disable(void)
#line 87
{
#line 87
  P1IE &= ~(1 << 4);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$disable(void){
#line 36
  HplMsp430InterruptP$Port14$disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$disable(void)
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$enable();
  }
  return SUCCESS;
}

static inline   error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$enableRisingEdge(void)
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$enable(TRUE);
}

# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t CC2420ControlP$InterruptCCA$enableRisingEdge(void){
#line 42
  unsigned char result;
#line 42

#line 42
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$enableRisingEdge();
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static   cc2420_status_t CC2420ControlP$IOCFG1$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_IOCFG1, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 192 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$CC2420Power$startOscillator(void)
#line 192
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 193
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 195
            FAIL;

            {
#line 195
              __nesc_atomic_end(__nesc_atomic); 
#line 195
              return __nesc_temp;
            }
          }
        }
#line 198
      CC2420ControlP$m_state = CC2420ControlP$S_XOSC_STARTING;
      CC2420ControlP$IOCFG1$write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP$InterruptCCA$enableRisingEdge();
      CC2420ControlP$SXOSCON$strobe();

      CC2420ControlP$IOCFG0$write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP$writeFsctrl();
      CC2420ControlP$writeMdmctrl0();

      CC2420ControlP$RXCTRL1$write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));
    }
#line 218
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   error_t CC2420CsmaP$CC2420Power$startOscillator(void){
#line 71
  unsigned char result;
#line 71

#line 71
  result = CC2420ControlP$CC2420Power$startOscillator();
#line 71

#line 71
  return result;
#line 71
}
#line 71
# 203 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline  void CC2420CsmaP$Resource$granted(void)
#line 203
{
  CC2420CsmaP$CC2420Power$startOscillator();
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void CC2420ControlP$Resource$granted(void){
#line 92
  CC2420CsmaP$Resource$granted();
#line 92
}
#line 92
# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$CSN$clr(void){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$clr();
#line 30
}
#line 30
# 362 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  void CC2420ControlP$SpiResource$granted(void)
#line 362
{
  CC2420ControlP$CSN$clr();
  CC2420ControlP$Resource$granted();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420ControlP$syncDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420ControlP$syncDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ControlP$SyncResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP$Resource$release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$CSN$set(void){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static   cc2420_status_t CC2420ControlP$SRXON$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SRXON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static   cc2420_status_t CC2420ControlP$SRFOFF$strobe(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP$Strobe$strobe(CC2420_SRFOFF);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 348 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  void CC2420ControlP$SyncResource$granted(void)
#line 348
{
  CC2420ControlP$CSN$clr();
  CC2420ControlP$SRFOFF$strobe();
  CC2420ControlP$writeFsctrl();
  CC2420ControlP$writeMdmctrl0();
  CC2420ControlP$writeId();
  CC2420ControlP$CSN$set();
  CC2420ControlP$CSN$clr();
  CC2420ControlP$SRXON$strobe();
  CC2420ControlP$CSN$set();
  CC2420ControlP$SyncResource$release();
  CC2420ControlP$syncDone$postTask();
}

#line 478
static inline   void CC2420ControlP$ReadRssi$default$readDone(error_t error, uint16_t data)
#line 478
{
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  void CC2420ControlP$ReadRssi$readDone(error_t arg_0x40624580, CC2420ControlP$ReadRssi$val_t arg_0x40624708){
#line 63
  CC2420ControlP$ReadRssi$default$readDone(arg_0x40624580, arg_0x40624708);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ControlP$RssiResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP$Resource$release(/*CC2420ControlC.RssiResource*/CC2420SpiC$2$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 34 "/opt/tinyos-2.x/tos/interfaces/SpiByte.nc"
inline static   uint8_t CC2420SpiP$SpiByte$write(uint8_t arg_0x41045678){
#line 34
  unsigned char result;
#line 34

#line 34
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiByte$write(arg_0x41045678);
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 126 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline   bool StateImplP$State$isIdle(uint8_t id)
#line 126
{
  return StateImplP$State$isState(id, StateImplP$S_IDLE);
}

# 61 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   bool CC2420SpiP$WorkingState$isIdle(void){
#line 61
  unsigned char result;
#line 61

#line 61
  result = StateImplP$State$isIdle(0U);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 283 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   cc2420_status_t CC2420SpiP$Reg$read(uint8_t addr, uint16_t *data)
#line 283
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 287
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 289
            status;

            {
#line 289
              __nesc_atomic_end(__nesc_atomic); 
#line 289
              return __nesc_temp;
            }
          }
        }
    }
#line 293
    __nesc_atomic_end(__nesc_atomic); }
#line 293
  status = CC2420SpiP$SpiByte$write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP$SpiByte$write(0) << 8;
  *data |= CC2420SpiP$SpiByte$write(0);

  return status;
}

# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static   cc2420_status_t CC2420ControlP$RSSI$read(uint16_t *arg_0x40f4fa50){
#line 47
  unsigned char result;
#line 47

#line 47
  result = CC2420SpiP$Reg$read(CC2420_RSSI, arg_0x40f4fa50);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 367 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  void CC2420ControlP$RssiResource$granted(void)
#line 367
{
  uint16_t data;

#line 369
  CC2420ControlP$CSN$clr();
  CC2420ControlP$RSSI$read(&data);
  CC2420ControlP$CSN$set();

  CC2420ControlP$RssiResource$release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP$ReadRssi$readDone(SUCCESS, data);
}

# 368 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline  void CC2420TransmitP$SpiResource$granted(void)
#line 368
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 371
    {
      cur_state = CC2420TransmitP$m_state;
    }
#line 373
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP$S_LOAD: 
        CC2420TransmitP$loadTXFIFO();
      break;

      case CC2420TransmitP$S_BEGIN_TRANSMIT: 
        CC2420TransmitP$attemptSend();
      break;

      case CC2420TransmitP$S_LOAD_CANCEL: 
        case CC2420TransmitP$S_CCA_CANCEL: 
          case CC2420TransmitP$S_TX_CANCEL: 
            CC2420TransmitP$CSN$clr();
      CC2420TransmitP$SFLUSHTX$strobe();
      CC2420TransmitP$CSN$set();
      CC2420TransmitP$releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 391
        {
          CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
        }
#line 393
        __nesc_atomic_end(__nesc_atomic); }
      break;

      default: 
        CC2420TransmitP$releaseSpiResource();
      break;
    }
}

# 194 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  void CC2420ReceiveP$SpiResource$granted(void)
#line 194
{
  CC2420ReceiveP$receive();
}

# 363 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   void CC2420SpiP$Resource$default$granted(uint8_t id)
#line 363
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void CC2420SpiP$Resource$granted(uint8_t arg_0x4104da80){
#line 92
  switch (arg_0x4104da80) {
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID:
#line 92
      CC2420ControlP$SpiResource$granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID:
#line 92
      CC2420ControlP$SyncResource$granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RssiResource*/CC2420SpiC$2$CLIENT_ID:
#line 92
      CC2420ControlP$RssiResource$granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID:
#line 92
      CC2420TransmitP$SpiResource$granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID:
#line 92
      CC2420ReceiveP$SpiResource$granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP$Resource$default$granted(arg_0x4104da80);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 354 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline  void CC2420SpiP$grant$runTask(void)
#line 354
{
  uint8_t holder;

#line 356
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 356
    {
      holder = CC2420SpiP$m_holder;
    }
#line 358
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP$Resource$granted(holder);
}

# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static   cc2420_status_t CC2420TransmitP$TXCTRL$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_TXCTRL, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 212 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   cc2420_status_t CC2420SpiP$Fifo$write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 213
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 217
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 219
            status;

            {
#line 219
              __nesc_atomic_end(__nesc_atomic); 
#line 219
              return __nesc_temp;
            }
          }
        }
    }
#line 223
    __nesc_atomic_end(__nesc_atomic); }
#line 223
  CC2420SpiP$m_addr = addr;

  status = CC2420SpiP$SpiByte$write(CC2420SpiP$m_addr);
  CC2420SpiP$SpiPacket$send(data, (void *)0, len);

  return status;
}

# 82 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static   cc2420_status_t CC2420TransmitP$TXFIFO$write(uint8_t *arg_0x4101f0b0, uint8_t arg_0x4101f238){
#line 82
  unsigned char result;
#line 82

#line 82
  result = CC2420SpiP$Fifo$write(CC2420_TXFIFO, arg_0x4101f0b0, arg_0x4101f238);
#line 82

#line 82
  return result;
#line 82
}
#line 82
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void *source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 55 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static   cc2420_status_t CC2420ControlP$FSCTRL$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_FSCTRL, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static   cc2420_status_t CC2420ControlP$MDMCTRL0$write(uint16_t arg_0x40f4e010){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP$Reg$write(CC2420_MDMCTRL0, arg_0x40f4e010);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 258 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline   cc2420_status_t CC2420SpiP$Ram$write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 260
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 264
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 266
            status;

            {
#line 266
              __nesc_atomic_end(__nesc_atomic); 
#line 266
              return __nesc_temp;
            }
          }
        }
    }
#line 270
    __nesc_atomic_end(__nesc_atomic); }
#line 270
  addr += offset;

  CC2420SpiP$SpiByte$write(addr | 0x80);
  CC2420SpiP$SpiByte$write((addr >> 1) & 0xc0);
  for (; len; len--) {
      status = CC2420SpiP$SpiByte$write(* data++);
    }

  return status;
}

# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static   cc2420_status_t CC2420ControlP$PANID$write(uint8_t arg_0x40f535f0, uint8_t *arg_0x40f53798, uint8_t arg_0x40f53920){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420SpiP$Ram$write(CC2420_RAM_PANID, arg_0x40f535f0, arg_0x40f53798, arg_0x40f53920);
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 202 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  void CC2420ActiveMessageP$CC2420Config$syncDone(error_t error)
#line 202
{
}

# 348 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  void CC2420ReceiveP$CC2420Config$syncDone(error_t error)
#line 348
{
}

# 53 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static  void CC2420ControlP$CC2420Config$syncDone(error_t arg_0x40ea9e98){
#line 53
  CC2420ReceiveP$CC2420Config$syncDone(arg_0x40ea9e98);
#line 53
  CC2420ActiveMessageP$CC2420Config$syncDone(arg_0x40ea9e98);
#line 53
}
#line 53
# 418 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  void CC2420ControlP$syncDone$runTask(void)
#line 418
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 419
    CC2420ControlP$m_sync_busy = FALSE;
#line 419
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP$CC2420Config$syncDone(SUCCESS);
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ControlP$SyncResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP$Resource$request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 285 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  error_t CC2420ControlP$CC2420Config$sync(void)
#line 285
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 286
    {
      if (CC2420ControlP$m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 288
            FAIL;

            {
#line 288
              __nesc_atomic_end(__nesc_atomic); 
#line 288
              return __nesc_temp;
            }
          }
        }
#line 291
      CC2420ControlP$m_sync_busy = TRUE;
      if (CC2420ControlP$m_state == CC2420ControlP$S_XOSC_STARTED) {
          CC2420ControlP$SyncResource$request();
        }
      else 
#line 294
        {
          CC2420ControlP$syncDone$postTask();
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 414
static inline  void CC2420ControlP$sync$runTask(void)
#line 414
{
  CC2420ControlP$CC2420Config$sync();
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void)
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t *)49U ^= 0x01 << 5;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle(void){
#line 44
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void)
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$toggle(void){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle();
#line 31
}
#line 31
# 88 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led1Toggle(void)
#line 88
{
  LedsP$Led1$toggle();
  ;
#line 90
  ;
}

# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static   void UBee430_APC$Leds$led1Toggle(void){
#line 72
  LedsP$Leds$led1Toggle();
#line 72
}
#line 72
# 200 "UBee430_APC.nc"
static inline  void UBee430_APC$AMSend$sendDone(message_t *buf, error_t error)
#line 200
{
  UBee430_APC$Leds$led1Toggle();
}

# 214 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline   void CC2420ActiveMessageP$AMSend$default$sendDone(uint8_t id, message_t *msg, error_t err)
#line 214
{
  return;
}

# 99 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static  void CC2420ActiveMessageP$AMSend$sendDone(am_id_t arg_0x40ebd188, message_t *arg_0x406665f8, error_t arg_0x40666780){
#line 99
  switch (arg_0x40ebd188) {
#line 99
    case AM_MSG:
#line 99
      UBee430_APC$AMSend$sendDone(arg_0x406665f8, arg_0x40666780);
#line 99
      break;
#line 99
    default:
#line 99
      CC2420ActiveMessageP$AMSend$default$sendDone(arg_0x40ebd188, arg_0x406665f8, arg_0x40666780);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 181 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  void CC2420ActiveMessageP$SubSend$sendDone(message_t *msg, error_t result)
#line 181
{
  CC2420ActiveMessageP$AMSend$sendDone(CC2420ActiveMessageP$AMPacket$type(msg), msg, result);
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static  void UniqueSendP$Send$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868){
#line 89
  CC2420ActiveMessageP$SubSend$sendDone(arg_0x40eb36e0, arg_0x40eb3868);
#line 89
}
#line 89
# 104 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline  void UniqueSendP$SubSend$sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP$State$toIdle();
  UniqueSendP$Send$sendDone(msg, error);
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static  void CC2420CsmaP$Send$sendDone(message_t *arg_0x40eb36e0, error_t arg_0x40eb3868){
#line 89
  UniqueSendP$SubSend$sendDone(arg_0x40eb36e0, arg_0x40eb3868);
#line 89
}
#line 89
# 111 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline   void StateImplP$State$forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP$state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   void CC2420CsmaP$SplitControlState$forceState(uint8_t arg_0x40f277b8){
#line 51
  StateImplP$State$forceState(1U, arg_0x40f277b8);
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420CsmaP$stopDone_task$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$stopDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)29U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$VREN$clr(void){
#line 30
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$clr();
#line 30
}
#line 30
# 184 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$CC2420Power$stopVReg(void)
#line 184
{
  CC2420ControlP$m_state = CC2420ControlP$S_VREG_STOPPED;
  CC2420ControlP$RSTN$clr();
  CC2420ControlP$VREN$clr();
  CC2420ControlP$RSTN$set();
  return SUCCESS;
}

# 63 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   error_t CC2420CsmaP$CC2420Power$stopVReg(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420ControlP$CC2420Power$stopVReg();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 49 "/opt/tinyos-2.x/tos/types/TinyError.h"
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 91 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port10$clear(void)
#line 91
{
#line 91
  P1IFG &= ~(1 << 0);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$clear(void){
#line 41
  HplMsp430InterruptP$Port10$clear();
#line 41
}
#line 41
# 83 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port10$disable(void)
#line 83
{
#line 83
  P1IE &= ~(1 << 0);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$disable(void){
#line 36
  HplMsp430InterruptP$Port10$disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$disable(void)
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$clear();
  }
  return SUCCESS;
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t CC2420ReceiveP$InterruptFIFOP$disable(void){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 128 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  error_t CC2420ReceiveP$StdControl$stop(void)
#line 128
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      CC2420ReceiveP$m_state = CC2420ReceiveP$S_STOPPED;
      CC2420ReceiveP$reset_state();
      CC2420ReceiveP$CSN$set();
      CC2420ReceiveP$InterruptFIFOP$disable();
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)31U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$selectIOFunc();
#line 85
}
#line 85
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$disableEvents(void)
{
  * (volatile uint16_t *)388U &= ~0x0010;
}

# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$disableEvents(void){
#line 40
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$disableEvents();
#line 40
}
#line 40
# 58 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline   void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$disable(void)
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectIOFunc();
  }
}

# 55 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static   void CC2420TransmitP$CaptureSFD$disable(void){
#line 55
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Capture$disable();
#line 55
}
#line 55
# 159 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline  error_t CC2420TransmitP$StdControl$stop(void)
#line 159
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 160
    {
      CC2420TransmitP$m_state = CC2420TransmitP$S_STOPPED;
      CC2420TransmitP$BackoffTimer$stop();
      CC2420TransmitP$CaptureSFD$disable();
      CC2420TransmitP$SpiResource$release();
      CC2420TransmitP$CSN$set();
    }
#line 166
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static  error_t CC2420CsmaP$SubControl$stop(void){
#line 84
  unsigned char result;
#line 84

#line 84
  result = CC2420TransmitP$StdControl$stop();
#line 84
  result = ecombine(result, CC2420ReceiveP$StdControl$stop());
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 265 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$shutdown(void)
#line 265
{
  CC2420CsmaP$SubControl$stop();
  CC2420CsmaP$CC2420Power$stopVReg();
  CC2420CsmaP$stopDone_task$postTask();
}

# 66 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   bool CC2420CsmaP$SplitControlState$isState(uint8_t arg_0x40f26368){
#line 66
  unsigned char result;
#line 66

#line 66
  result = StateImplP$State$isState(1U, arg_0x40f26368);
#line 66

#line 66
  return result;
#line 66
}
#line 66
# 234 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline  void CC2420CsmaP$sendDone_task$runTask(void)
#line 234
{
  error_t packetErr;

#line 236
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 236
    packetErr = CC2420CsmaP$sendErr;
#line 236
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STOPPING)) {
      CC2420CsmaP$shutdown();
    }
  else {
      CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STARTED);
    }

  CC2420CsmaP$Send$sendDone(CC2420CsmaP$m_msg, packetErr);
}

# 213 "UBee430_APC.nc"
static inline  void UBee430_APC$SplitControl$stopDone(error_t err)
#line 213
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void CC2420CsmaP$SplitControl$stopDone(error_t arg_0x4062a6e8){
#line 117
  UBee430_APC$SplitControl$stopDone(arg_0x4062a6e8);
#line 117
}
#line 117
# 255 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline  void CC2420CsmaP$stopDone_task$runTask(void)
#line 255
{
  CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STOPPED);
  CC2420CsmaP$SplitControl$stopDone(SUCCESS);
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t UBee430_APC$SplitControl$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = CC2420CsmaP$SplitControl$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t UBee430_APC$next$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(UBee430_APC$next);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 311 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_years(uint8_t years)
#line 311
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2f, years);
}

# 38 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_years(uint8_t arg_0x4067a010){
#line 38
  Ds28dg02P$Ds28dg02s$WriteRTC_years(arg_0x4067a010);
#line 38
}
#line 38
# 307 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_months(uint8_t months)
#line 307
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2e, months);
}

# 37 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_months(uint8_t arg_0x4067bb10){
#line 37
  Ds28dg02P$Ds28dg02s$WriteRTC_months(arg_0x4067bb10);
#line 37
}
#line 37
# 303 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_date(uint8_t date)
#line 303
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2d, date);
}

# 36 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_date(uint8_t arg_0x4067b688){
#line 36
  Ds28dg02P$Ds28dg02s$WriteRTC_date(arg_0x4067b688);
#line 36
}
#line 36
# 299 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_dayofweek(uint8_t dayofweek)
#line 299
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2c, dayofweek);
}

# 35 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_dayofweek(uint8_t arg_0x4067b100){
#line 35
  Ds28dg02P$Ds28dg02s$WriteRTC_dayofweek(arg_0x4067b100);
#line 35
}
#line 35
# 295 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_hours(uint8_t hours)
#line 295
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2b, hours);
}

# 34 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_hours(uint8_t arg_0x4067cc50){
#line 34
  Ds28dg02P$Ds28dg02s$WriteRTC_hours(arg_0x4067cc50);
#line 34
}
#line 34
# 291 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_minutes(uint8_t minutes)
#line 291
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x2a, minutes);
}

# 33 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_minutes(uint8_t arg_0x4067c7c8){
#line 33
  Ds28dg02P$Ds28dg02s$WriteRTC_minutes(arg_0x4067c7c8);
#line 33
}
#line 33
# 287 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$WriteRTC_seconds(uint8_t seconds)
#line 287
{
  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x29, seconds);
}

# 32 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$WriteRTC_seconds(uint8_t arg_0x4067c338){
#line 32
  Ds28dg02P$Ds28dg02s$WriteRTC_seconds(arg_0x4067c338);
#line 32
}
#line 32
# 115 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_SET_DS28DG02_CS_PIN(void)
#line 115
{
#line 115
   static volatile uint8_t r __asm ("0x001D");

#line 115
  r |= 1 << 3;
}

#line 114
static inline void TOSH_MAKE_DS28DG02_SCK_OUTPUT(void)
#line 114
{
#line 114
   static volatile uint8_t r __asm ("0x001A");

#line 114
  r |= 1 << 0;
}

#line 115
static inline void TOSH_MAKE_DS28DG02_CS_OUTPUT(void)
#line 115
{
#line 115
   static volatile uint8_t r __asm ("0x001E");

#line 115
  r |= 1 << 3;
}

#line 113
static inline void TOSH_MAKE_DS28DG02_SO_INPUT(void)
#line 113
{
#line 113
   static volatile uint8_t r __asm ("0x002A");

#line 113
  r &= ~(1 << 1);
}

#line 112
static inline void TOSH_MAKE_DS28DG02_SI_OUTPUT(void)
#line 112
{
#line 112
   static volatile uint8_t r __asm ("0x002A");

#line 112
  r |= 1 << 0;
}



static inline void TOSH_MAKE_DS28DG02_RST_INPUT(void)
#line 117
{
#line 117
   static volatile uint8_t r __asm ("0x002A");

#line 117
  r &= ~(1 << 4);
}

#line 116
static inline void TOSH_MAKE_DS28DG02_ALM_INPUT(void)
#line 116
{
#line 116
   static volatile uint8_t r __asm ("0x002A");

#line 116
  r &= ~(1 << 3);
}


static inline void TOSH_MAKE_SW1_INPUT(void)
#line 120
{
#line 120
   static volatile uint8_t r __asm ("0x002A");

#line 120
  r &= ~(1 << 7);
}

# 29 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Ds28dg02s$init(void)
{







  TOSH_MAKE_SW1_INPUT();
  TOSH_MAKE_DS28DG02_ALM_INPUT();
  TOSH_MAKE_DS28DG02_RST_INPUT();
  TOSH_MAKE_DS28DG02_SI_OUTPUT();
  TOSH_MAKE_DS28DG02_SO_INPUT();
  TOSH_MAKE_DS28DG02_CS_OUTPUT();
  TOSH_MAKE_DS28DG02_SCK_OUTPUT();

  TOSH_SET_DS28DG02_CS_PIN();


  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x22, 0x00);

  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x23, 0xff);

  Ds28dg02P$SPI_INSTRUCTION_SET(0x06);
  Ds28dg02P$SPI_WRITE(0x0a, 0x34, 0x02);
}

# 5 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void UBee430_APC$DS28$init(void){
#line 5
  Ds28dg02P$Ds28dg02s$init();
#line 5
}
#line 5
# 117 "UBee430_APC.nc"
static inline void UBee430_APC$rtc_init(void)
#line 117
{
  UBee430_APC$DS28$init();
  UBee430_APC$DS28$WriteRTC_seconds(0x00);
  UBee430_APC$DS28$WriteRTC_minutes(0x00);
  UBee430_APC$DS28$WriteRTC_hours(0x23);
  UBee430_APC$DS28$WriteRTC_dayofweek(0x02);
  UBee430_APC$DS28$WriteRTC_date(0x1c);
  UBee430_APC$DS28$WriteRTC_months(0x0C);
  UBee430_APC$DS28$WriteRTC_years(0x0A);
}

#line 205
static inline  void UBee430_APC$SplitControl$startDone(error_t err)
#line 205
{
  if (err == SUCCESS) {
      UBee430_APC$rtc_init();
      UBee430_APC$next$postTask();
    }
  else {
#line 210
    UBee430_APC$SplitControl$start();
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void CC2420CsmaP$SplitControl$startDone(error_t arg_0x4062baf0){
#line 92
  UBee430_APC$SplitControl$startDone(arg_0x4062baf0);
#line 92
}
#line 92
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420ControlP$SpiResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP$Resource$release(/*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 164 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$Resource$release(void)
#line 164
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 165
    {
      CC2420ControlP$CSN$set();
      {
        unsigned char __nesc_temp = 
#line 167
        CC2420ControlP$SpiResource$release();

        {
#line 167
          __nesc_atomic_end(__nesc_atomic); 
#line 167
          return __nesc_temp;
        }
      }
    }
#line 170
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t CC2420CsmaP$Resource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420ControlP$Resource$release();
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 234 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$CC2420Power$rxOn(void)
#line 234
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 235
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 237
            FAIL;

            {
#line 237
              __nesc_atomic_end(__nesc_atomic); 
#line 237
              return __nesc_temp;
            }
          }
        }
#line 239
      CC2420ControlP$SRXON$strobe();
    }
#line 240
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   error_t CC2420CsmaP$CC2420Power$rxOn(void){
#line 90
  unsigned char result;
#line 90

#line 90
  result = CC2420ControlP$CC2420Power$rxOn();
#line 90

#line 90
  return result;
#line 90
}
#line 90
# 75 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port10$enable(void)
#line 75
{
#line 75
  P1IE |= 1 << 0;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$enable(void){
#line 31
  HplMsp430InterruptP$Port10$enable();
#line 31
}
#line 31
# 107 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port10$edge(bool l2h)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    if (l2h) {
#line 109
      P1IES &= ~(1 << 0);
      }
    else {
#line 110
      P1IES |= 1 << 0;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$edge(bool arg_0x40aceef8){
#line 56
  HplMsp430InterruptP$Port10$edge(arg_0x40aceef8);
#line 56
}
#line 56
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$enable();
  }
  return SUCCESS;
}





static inline   error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$enableFallingEdge(void)
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$enable(FALSE);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t CC2420ReceiveP$InterruptFIFOP$enableFallingEdge(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$enableFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 118 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  error_t CC2420ReceiveP$StdControl$start(void)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    {
      CC2420ReceiveP$reset_state();
      CC2420ReceiveP$m_state = CC2420ReceiveP$S_STARTED;
      CC2420ReceiveP$receivingPacket = FALSE;
      CC2420ReceiveP$InterruptFIFOP$enableFallingEdge();
    }
#line 124
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 148 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline  error_t CC2420TransmitP$StdControl$start(void)
#line 148
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
    {
      CC2420TransmitP$CaptureSFD$captureRisingEdge();
      CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
      CC2420TransmitP$m_receiving = FALSE;
      CC2420TransmitP$abortSpiRelease = FALSE;
      CC2420TransmitP$m_tx_power = 0;
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static  error_t CC2420CsmaP$SubControl$start(void){
#line 74
  unsigned char result;
#line 74

#line 74
  result = CC2420TransmitP$StdControl$start();
#line 74
  result = ecombine(result, CC2420ReceiveP$StdControl$start());
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 247 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline  void CC2420CsmaP$startDone_task$runTask(void)
#line 247
{
  CC2420CsmaP$SubControl$start();
  CC2420CsmaP$CC2420Power$rxOn();
  CC2420CsmaP$Resource$release();
  CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STARTED);
  CC2420CsmaP$SplitControl$startDone(SUCCESS);
}

# 115 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_CLR_DS28DG02_CS_PIN(void)
#line 115
{
#line 115
   static volatile uint8_t r __asm ("0x001D");

#line 115
  r &= ~(1 << 3);
}

# 60 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline void Ds28dg02P$DS28DG02_SEL(void)
#line 60
{
  TOSH_CLR_DS28DG02_CS_PIN();
}



static inline void Ds28dg02P$delay(uint8_t data)
#line 66
{
  uint8_t i;

#line 68
  for (i = 0; i <= data; i++) ;
}

# 112 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_SET_DS28DG02_SI_PIN(void)
#line 112
{
#line 112
   static volatile uint8_t r __asm ("0x0029");

#line 112
  r |= 1 << 0;
}

#line 112
static inline void TOSH_CLR_DS28DG02_SI_PIN(void)
#line 112
{
#line 112
   static volatile uint8_t r __asm ("0x0029");

#line 112
  r &= ~(1 << 0);
}

#line 114
static inline void TOSH_CLR_DS28DG02_SCK_PIN(void)
#line 114
{
#line 114
   static volatile uint8_t r __asm ("0x0019");

#line 114
  r &= ~(1 << 0);
}

#line 114
static inline void TOSH_SET_DS28DG02_SCK_PIN(void)
#line 114
{
#line 114
   static volatile uint8_t r __asm ("0x0019");

#line 114
  r |= 1 << 0;
}

# 63 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline void Ds28dg02P$DS28DG02_DSEL(void)
#line 63
{
  TOSH_SET_DS28DG02_CS_PIN();
}

# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   error_t CC2420CsmaP$SplitControlState$requestState(uint8_t arg_0x40f27230){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP$State$requestState(1U, arg_0x40f27230);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 55 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void CC2420ControlP$StartupTimer$start(CC2420ControlP$StartupTimer$size_type arg_0x4092d460){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$start(arg_0x4092d460);
#line 55
}
#line 55
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)29U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$VREN$set(void){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$set();
#line 29
}
#line 29
# 172 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   error_t CC2420ControlP$CC2420Power$startVReg(void)
#line 172
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 173
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 175
            FAIL;

            {
#line 175
              __nesc_atomic_end(__nesc_atomic); 
#line 175
              return __nesc_temp;
            }
          }
        }
#line 177
      CC2420ControlP$m_state = CC2420ControlP$S_VREG_STARTING;
    }
#line 178
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP$VREN$set();
  CC2420ControlP$StartupTimer$start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   error_t CC2420CsmaP$CC2420Power$startVReg(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420ControlP$CC2420Power$startVReg();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(Msp430RefVoltArbiterImplP$switchOff);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void)
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void)
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 147 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t arg_0x40686600){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(4U, arg_0x40686600);
#line 62
}
#line 62
# 151 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t arg_0x4062a6e8){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(arg_0x4062a6e8);
#line 117
}
#line 117
# 147 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t arg_0x4062a6e8){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(arg_0x4062a6e8);
#line 117
}
#line 117
# 152 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(3U);
#line 67
}
#line 67
# 127 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 
          if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
              Msp430RefVoltGeneratorP$SwitchOnTimer$stop();
              Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
              if (Msp430RefVoltGeneratorP$state == Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) {
                Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(SUCCESS);
                }
              else {
#line 140
                Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(SUCCESS);
                }
#line 141
              return SUCCESS;
            }
          else {
#line 143
            return FAIL;
            }
#line 144
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

        default: 

          return FAIL;
    }
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void){
#line 109
  unsigned char result;
#line 109

#line 109
  result = Msp430RefVoltGeneratorP$RefVolt_1_5V$stop();
#line 109

#line 109
  return result;
#line 109
}
#line 109
# 136 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$switchOff$runTask(void)
{

  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {
      if (Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop() == SUCCESS) {
          Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;
        }
      else {
#line 143
        Msp430RefVoltArbiterImplP$switchOff$postTask();
        }
    }
}

# 120 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   bool HplAdc12P$HplAdc12$isBusy(void)
#line 120
{
#line 120
  return HplAdc12P$ADC12CTL1 & 0x0001;
}

# 118 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void){
#line 118
  unsigned char result;
#line 118

#line 118
  result = HplAdc12P$HplAdc12$isBusy();
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 65 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void)
#line 65
{
  return * (adc12ctl0_t *)&HplAdc12P$ADC12CTL0;
}

# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void){
#line 63
  struct __nesc_unnamed4254 result;
#line 63

#line 63
  result = HplAdc12P$HplAdc12$getCtl0();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0)
#line 57
{
  HplAdc12P$ADC12CTL0 = * (uint16_t *)&control0;
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t arg_0x40c8e010){
#line 51
  HplAdc12P$HplAdc12$setCtl0(arg_0x40c8e010);
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 160 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id)
#line 160
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(uint8_t arg_0x40d39088){
#line 92
  switch (arg_0x40d39088) {
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(arg_0x40d39088);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 166 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 166
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x40d374d8){
#line 49
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(arg_0x40d374d8);
#line 49
}
#line 49
# 150 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void)
#line 150
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY;
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc"
static inline   const msp430adc12_channel_config_t *Msp430InternalVoltageP$AdcConfigure$getConfiguration(void)
{
  return &Msp430InternalVoltageP$config;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration();
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration();
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$adc_config_t /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfUp$getConfiguration();
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalTemperatureP.nc"
static inline   const msp430adc12_channel_config_t *Msp430InternalTemperatureP$AdcConfigure$getConfiguration(void)
{
  return &Msp430InternalTemperatureP$config;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = Msp430InternalTemperatureP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfUp$getConfiguration();
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$adc_config_t /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = Msp430InternalTemperatureP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfUp$getConfiguration();
}

# 19 "/opt/tinyos-2.x/tos/platforms/UBee430/LightToVoltageP.nc"
static inline   const msp430adc12_channel_config_t *LightToVoltageP$AdcConfigure$getConfiguration(void)
{
  return &LightToVoltageP$config;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$adc_config_t /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = LightToVoltageP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfUp$getConfiguration();
}

# 19 "/opt/tinyos-2.x/tos/platforms/UBee430/AdcZeroP.nc"
static inline   const msp430adc12_channel_config_t *AdcZeroP$AdcConfigure$getConfiguration(void)
{
  return &AdcZeroP$config;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$adc_config_t /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$getConfiguration(void){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  result = AdcZeroP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline   const msp430adc12_channel_config_t */*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$getConfiguration(void)
{
  return /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfUp$getConfiguration();
}

# 172 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline    const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(uint8_t arg_0x40d75d80){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  switch (arg_0x40d75d80) {
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 58
      result = /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 58
      result = /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 58
      result = /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$2$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 58
      result = /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$3$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 58
      result = /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$4$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 58
      result = /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$5$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 58
      result = /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$6$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = Msp430RefVoltArbiterImplP$Config$default$getConfiguration(arg_0x40d75d80);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 56 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void)
#line 56
{
  int i;

  /* atomic removed: atomic calls only */
#line 58
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[i] > 0) {
          unsigned char __nesc_temp = 
#line 60
          FALSE;

#line 60
          return __nesc_temp;
        }
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      TRUE;

#line 61
      return __nesc_temp;
    }
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 47 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id)
#line 47
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] &= ~(1 << id % 8);
}

#line 69
static inline   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void)
#line 69
{
  int i;

  /* atomic removed: atomic calls only */
#line 71
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last + 1; ; i++) {
        if (i == 7U) {
          i = 0;
          }
#line 75
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last = i;
            {
              unsigned char __nesc_temp = 
#line 78
              i;

#line 78
              return __nesc_temp;
            }
          }
#line 80
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last) {
          break;
          }
      }
#line 83
    {
      unsigned char __nesc_temp = 
#line 83
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$NO_ENTRY;

#line 83
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 168 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 168
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x40d374d8){
#line 55
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x40d374d8);
#line 55
}
#line 55
# 162 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 162
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(uint8_t arg_0x40d399c8){
#line 43
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(arg_0x40d399c8);
#line 43
}
#line 43
# 87 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id)
#line 87
{
  /* atomic removed: atomic calls only */
#line 88
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      EBUSY;

#line 93
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x40b828b0){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(arg_0x40b828b0);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 98 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t arg_0x4062baf0){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(arg_0x4062baf0);
#line 92
}
#line 92
# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(4U);
#line 67
}
#line 67
# 94 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
      Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(17);
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING;
            return SUCCESS;
          }
        else {
#line 108
          return FAIL;
          }
#line 109
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE;
            Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
            return SUCCESS;
          }
        else {
#line 116
          return FAIL;
          }
#line 117
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

          default: 

            return FAIL;
    }
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = Msp430RefVoltGeneratorP$RefVolt_1_5V$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 354 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client)
{
  return &AdcP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   AdcP$Config$adc_config_t AdcP$Config$getConfiguration(uint8_t arg_0x40c33bb8){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  switch (arg_0x40c33bb8) {
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 58
      result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT:
#line 58
      result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 58
      result = Msp430InternalTemperatureP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 58
      result = LightToVoltageP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 58
      result = AdcZeroP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcP$Config$default$getConfiguration(arg_0x40c33bb8);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 378 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 379
{
#line 379
  return FAIL;
}

# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannel$configureSingle(uint8_t arg_0x40c26748, const msp430adc12_channel_config_t *arg_0x40c2c2c0){
#line 84
  unsigned char result;
#line 84

#line 84
  switch (arg_0x40c26748) {
#line 84
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    default:
#line 84
      result = AdcP$SingleChannel$default$configureSingle(arg_0x40c26748, arg_0x40c2c2c0);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 136 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void)
#line 136
{
  /* atomic removed: atomic calls only */
#line 137
  {
    unsigned char __nesc_temp = 
#line 137
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId;

#line 137
    return __nesc_temp;
  }
}

# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static   uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void){
#line 63
  struct __nesc_unnamed4254 result;
#line 63

#line 63
  result = HplAdc12P$HplAdc12$getCtl0();
#line 63

#line 63
  return result;
#line 63
}
#line 63
#line 51
inline static   void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t arg_0x40c8e010){
#line 51
  HplAdc12P$HplAdc12$setCtl0(arg_0x40c8e010);
#line 51
}
#line 51
# 61 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1)
#line 61
{
  HplAdc12P$ADC12CTL1 = * (uint16_t *)&control1;
}

# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t arg_0x40c8e550){
#line 57
  HplAdc12P$HplAdc12$setCtl1(arg_0x40c8e550);
#line 57
}
#line 57
# 91 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask)
#line 91
{
#line 91
  HplAdc12P$ADC12IE = mask;
}

# 95 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t arg_0x40c8c488){
#line 95
  HplAdc12P$HplAdc12$setIEFlags(arg_0x40c8c488);
#line 95
}
#line 95
# 347 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannel$default$getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannel$getData(uint8_t arg_0x40c26748){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x40c26748) {
#line 189
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 189
      break;
#line 189
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID);
#line 189
      break;
#line 189
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 189
      break;
#line 189
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 189
      break;
#line 189
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcP$SingleChannel$default$getData(arg_0x40c26748);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return result;
#line 189
}
#line 189
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 7);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 7;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 6);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 6;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 5);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 3);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 2);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 0);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 0;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc();
#line 78
}
#line 78
# 132 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectModuleFunc();
#line 137
      Msp430Adc12ImplP$Port60$makeInput();
#line 137
      break;
      case 1: Msp430Adc12ImplP$Port61$selectModuleFunc();
#line 138
      Msp430Adc12ImplP$Port61$makeInput();
#line 138
      break;
      case 2: Msp430Adc12ImplP$Port62$selectModuleFunc();
#line 139
      Msp430Adc12ImplP$Port62$makeInput();
#line 139
      break;
      case 3: Msp430Adc12ImplP$Port63$selectModuleFunc();
#line 140
      Msp430Adc12ImplP$Port63$makeInput();
#line 140
      break;
      case 4: Msp430Adc12ImplP$Port64$selectModuleFunc();
#line 141
      Msp430Adc12ImplP$Port64$makeInput();
#line 141
      break;
      case 5: Msp430Adc12ImplP$Port65$selectModuleFunc();
#line 142
      Msp430Adc12ImplP$Port65$makeInput();
#line 142
      break;
      case 6: Msp430Adc12ImplP$Port66$selectModuleFunc();
#line 143
      Msp430Adc12ImplP$Port66$makeInput();
#line 143
      break;
      case 7: Msp430Adc12ImplP$Port67$selectModuleFunc();
#line 144
      Msp430Adc12ImplP$Port67$makeInput();
#line 144
      break;
    }
}

# 106 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$startConversion(void)
#line 106
{
  HplAdc12P$ADC12CTL0 |= 0x0010;
  HplAdc12P$ADC12CTL0 |= 0x0001 + 0x0002;
}

# 128 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$startConversion(void){
#line 128
  HplAdc12P$HplAdc12$startConversion();
#line 128
}
#line 128
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$setMode(int arg_0x40851b08){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(arg_0x40851b08);
#line 39
}
#line 39
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4391 {
#line 46
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
{
  * (volatile uint16_t *)356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t arg_0x40865180){
#line 35
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(arg_0x40865180);
#line 35
}
#line 35
# 111 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$startTimerA(void)
{

  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP$ControlA1$setControl(ccResetSHI);
  Msp430Adc12ImplP$ControlA1$setControl(ccSetSHI);

  Msp430Adc12ImplP$ControlA1$setControl(ccRSOutmod);
  Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_UP_MODE);
}

# 180 "UBee430_APC.nc"
static inline  void UBee430_APC$AdcZero$readDone(error_t result, uint16_t data)
#line 180
{

  if (data < 300) {
    data == 0;
    }
  else {
#line 185
    __nesc_hton_uint16((unsigned char *)&UBee430_APC$message.adc0, ((float )data - 300) / 1365 * 10);
    }
#line 186
  UBee430_APC$count++;
}

#line 128
static inline  void UBee430_APC$Photo$readDone(error_t result, uint16_t data)
#line 128
{
  if (result == SUCCESS) {
    __nesc_hton_uint16((unsigned char *)&UBee430_APC$message.photo, (uint16_t )data);
    }
  else {
#line 132
    __nesc_hton_uint16((unsigned char *)&UBee430_APC$message.photo, 0xFFFF);
    }
#line 133
  UBee430_APC$count++;
}

static inline  void UBee430_APC$InternalTemp$readDone(error_t result, uint16_t data)
{
  if (result == SUCCESS) {
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.internaltemp, (uint8_t )((data / 4096.0 * 1.5 - 0.986) / 0.00355));
    }
  else {
#line 141
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.internaltemp, 0xFF);
    }
#line 142
  UBee430_APC$count++;
}

static inline  void UBee430_APC$InternalVolt$readDone(error_t result, uint16_t data)
#line 145
{
  if (result == SUCCESS) {
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.internalvolt, (uint8_t )(data / 4096.0 * 3.0 * 10.0));
    }
  else {
#line 149
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.internalvolt, 0xFF);
    }
#line 150
  UBee430_APC$count++;
}

# 360 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    const msp430adc12_channel_config_t *
AdcP$ConfigReadStream$default$getConfiguration(uint8_t client)
{
  return &AdcP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$getConfiguration(uint8_t arg_0x40c31430){
#line 58
  struct __nesc_unnamed4276 const *result;
#line 58

#line 58
  switch (arg_0x40c31430) {
#line 58
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 58
      result = Msp430InternalVoltageP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT:
#line 58
      result = Msp430InternalTemperatureP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcP$ConfigReadStream$default$getConfiguration(arg_0x40c31430);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 366 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannelReadStream$default$configureMultiple(uint8_t client, 
const msp430adc12_channel_config_t *config, uint16_t buffer[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannelReadStream$configureMultiple(uint8_t arg_0x40c31f00, const msp430adc12_channel_config_t *arg_0x40c2b848, uint16_t arg_0x40c2b9f8[], uint16_t arg_0x40c2bb90, uint16_t arg_0x40c2bd20){
#line 138
  unsigned char result;
#line 138

#line 138
  switch (arg_0x40c31f00) {
#line 138
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 138
      result = Msp430Adc12ImplP$SingleChannel$configureMultiple(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID, arg_0x40c2b848, arg_0x40c2b9f8, arg_0x40c2bb90, arg_0x40c2bd20);
#line 138
      break;
#line 138
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT:
#line 138
      result = Msp430Adc12ImplP$SingleChannel$configureMultiple(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID, arg_0x40c2b848, arg_0x40c2b9f8, arg_0x40c2bb90, arg_0x40c2bd20);
#line 138
      break;
#line 138
    default:
#line 138
      result = AdcP$SingleChannelReadStream$default$configureMultiple(arg_0x40c31f00, arg_0x40c2b848, arg_0x40c2b9f8, arg_0x40c2bb90, arg_0x40c2bd20);
#line 138
      break;
#line 138
    }
#line 138

#line 138
  return result;
#line 138
}
#line 138
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)372U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t arg_0x4085fed0){
#line 30
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(arg_0x4085fed0);
#line 30
}
#line 30
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)370U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t arg_0x4085fed0){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(arg_0x4085fed0);
#line 30
}
#line 30
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4392 {
#line 46
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
{
  * (volatile uint16_t *)354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t arg_0x40865180){
#line 35
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(arg_0x40865180);
#line 35
}
#line 35
# 110 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t arg_0x4084f178){
#line 45
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(arg_0x4084f178);
#line 45
}
#line 45
# 105 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t arg_0x40850ca0){
#line 44
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(arg_0x40850ca0);
#line 44
}
#line 44
# 100 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void)
{
  * (volatile uint16_t *)352U &= ~2U;
}

# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$disableEvents(void){
#line 43
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents();
#line 43
}
#line 43
# 90 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void)
{
  * (volatile uint16_t *)352U |= 4U;
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$clear(void){
#line 41
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear();
#line 41
}
#line 41
# 93 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP$TimerA$clear();
  Msp430Adc12ImplP$TimerA$disableEvents();
  Msp430Adc12ImplP$TimerA$setClockSource(csSAMPCON);
  Msp430Adc12ImplP$TimerA$setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP$ControlA0$setControl(ccResetSHI);
  Msp430Adc12ImplP$CompareA0$setEvent(interval - 1);
  Msp430Adc12ImplP$CompareA1$setEvent((interval - 1) / 2);
}

# 373 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannelReadStream$default$getData(uint8_t client)
{
  return FAIL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannelReadStream$getData(uint8_t arg_0x40c31f00){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x40c31f00) {
#line 189
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 189
      break;
#line 189
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcP$SingleChannelReadStream$default$getData(arg_0x40c31f00);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return result;
#line 189
}
#line 189
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$finishStreamRequest$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$finishStreamRequest);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 333 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline   void AdcP$ResourceReadNow$default$granted(uint8_t nowClient)
#line 333
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void AdcP$ResourceReadNow$granted(uint8_t arg_0x40c0c7b0){
#line 92
    AdcP$ResourceReadNow$default$granted(arg_0x40c0c7b0);
#line 92
}
#line 92
# 123 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline  void AdcP$SubResourceReadNow$granted(uint8_t nowClient)
{
  if (AdcP$configure(nowClient) == SUCCESS) {
    AdcP$state = AdcP$STATE_READNOW;
    }
  else {
#line 128
    AdcP$state = AdcP$STATE_READNOW_INVALID_CONFIG;
    }
#line 129
  AdcP$ResourceReadNow$granted(nowClient);
}

# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t arg_0x4062baf0){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(arg_0x4062baf0);
#line 92
}
#line 92
# 157 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
      Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(17);
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING;
            return SUCCESS;
          }
        else {
#line 171
          return FAIL;
          }
#line 172
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE;
            Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
            return SUCCESS;
          }
        else {
#line 179
          return FAIL;
          }
#line 180
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

          default: 

            return FAIL;
    }
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = Msp430RefVoltGeneratorP$RefVolt_2_5V$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 326 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceRead$default$release(uint8_t client)
#line 326
{
#line 326
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceRead$release(uint8_t arg_0x40c0a7c8){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40c0a7c8) {
#line 110
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = AdcP$ResourceRead$default$release(arg_0x40c0a7c8);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$readDone$runTask(void)
{
  AdcP$ResourceRead$release(AdcP$owner);
  AdcP$Read$readDone(AdcP$owner, SUCCESS, AdcP$value);
}

#line 343
static inline   void AdcP$ReadStream$default$bufferDone(uint8_t streamClient, error_t result, 
uint16_t *buf, uint16_t count)
#line 344
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static  void AdcP$ReadStream$bufferDone(uint8_t arg_0x40c0b1d8, error_t arg_0x40c04638, AdcP$ReadStream$val_t *arg_0x40c047f0, uint16_t arg_0x40c04980){
#line 89
    AdcP$ReadStream$default$bufferDone(arg_0x40c0b1d8, arg_0x40c04638, arg_0x40c047f0, arg_0x40c04980);
#line 89
}
#line 89
# 312 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$signalBufferDone$runTask(void)
{
  AdcP$ReadStream$bufferDone(AdcP$owner, SUCCESS, AdcP$resultBuf, AdcP$value);
  AdcP$resultBuf = 0;
}

#line 345
static inline   void AdcP$ReadStream$default$readDone(uint8_t streamClient, error_t result, uint32_t actualPeriod)
#line 345
{
}

# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static  void AdcP$ReadStream$readDone(uint8_t arg_0x40c0b1d8, error_t arg_0x40c03010, uint32_t arg_0x40c031a8){
#line 102
    AdcP$ReadStream$default$readDone(arg_0x40c0b1d8, arg_0x40c03010, arg_0x40c031a8);
#line 102
}
#line 102
# 341 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceReadStream$default$release(uint8_t streamClient)
#line 341
{
#line 341
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceReadStream$release(uint8_t arg_0x40c25a58){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40c25a58) {
#line 110
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = AdcP$ResourceReadStream$default$release(arg_0x40c25a58);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 222 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$finishStreamRequest$runTask(void)
{
  AdcP$ResourceReadStream$release(AdcP$owner);
  if (!AdcP$streamBuf[AdcP$owner]) {

    AdcP$ReadStream$readDone(AdcP$owner, SUCCESS, AdcP$usPeriod[AdcP$owner]);
    }
  else 
#line 228
    {








      AdcP$ReadStream$readDone(AdcP$owner, FAIL, 0);
    }
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = HplSensirionSht11P$SplitControl$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 84 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$start(void)
#line 84
{
  return SUCCESS;
}

# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$start(void){
#line 74
  unsigned char result;
#line 74

#line 74
  result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$start();
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 63 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$runTask(void)
#line 63
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$start();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$start();
}

# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$makeOutput(void)
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t *)34U |= 0x01 << 7;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$PWR$makeOutput(void){
#line 35
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)33U |= 0x01 << 7;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$PWR$set(void){
#line 29
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$set();
#line 29
}
#line 29
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void HplSensirionSht11P$Timer$startOneShot(uint32_t arg_0x40686600){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(1U, arg_0x40686600);
#line 62
}
#line 62
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t HplSensirionSht11P$stopTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(HplSensirionSht11P$stopTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)33U &= ~(0x01 << 7);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP$7$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$PWR$clr(void){
#line 30
  /*HplSensirionSht11C.PWRM*/Msp430GpioC$5$GeneralIO$clr();
#line 30
}
#line 30
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$DATA$clr(void){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$clr();
#line 30
}
#line 30
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeInput(void)
#line 41
{
#line 41
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$DATA$makeInput(void){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeInput();
#line 33
}
#line 33
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$clr(void)
#line 38
{
#line 38
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$SCK$clr(void){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$clr();
#line 30
}
#line 30
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeInput(void)
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t *)34U &= ~(0x01 << 6);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeInput(void)
#line 41
{
#line 41
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void HplSensirionSht11P$SCK$makeInput(void){
#line 33
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeInput();
#line 33
}
#line 33
# 63 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline  error_t HplSensirionSht11P$SplitControl$stop(void)
#line 63
{
  HplSensirionSht11P$SCK$makeInput();
  HplSensirionSht11P$SCK$clr();
  HplSensirionSht11P$DATA$makeInput();
  HplSensirionSht11P$DATA$clr();
  HplSensirionSht11P$PWR$clr();
  HplSensirionSht11P$stopTask$postTask();
  return SUCCESS;
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stop(void){
#line 109
  unsigned char result;
#line 109

#line 109
  result = HplSensirionSht11P$SplitControl$stop();
#line 109

#line 109
  return result;
#line 109
}
#line 109
# 112 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$stop(void)
#line 112
{
  return SUCCESS;
}

# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static  error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$stop(void){
#line 84
  unsigned char result;
#line 84

#line 84
  result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$default$stop();
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 120 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline    void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$default$cleanup(void)
#line 120
{
}

# 52 "/opt/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
inline static   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$cleanup(void){
#line 52
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$default$cleanup();
#line 52
}
#line 52
# 68 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$runTask(void)
#line 68
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$PowerDownCleanup$cleanup();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$stop();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stop();
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$readDone(error_t arg_0x40624580, /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$val_t arg_0x40624708){
#line 63
  UBee430_APC$Temperature$readDone(arg_0x40624580, arg_0x40624708);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$release(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 113 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperature(uint8_t client)
#line 113
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$on) {
#line 114
      return EOFF;
    }
#line 115
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy) {
#line 115
      return EBUSY;
    }
  else 
#line 115
    {
#line 115
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = TRUE;
    }
#line 116
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_TEMPERATURE;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$performCommand();
}

# 61 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperature(void){
#line 61
  unsigned char result;
#line 61

#line 61
  result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperature(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 65 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$granted(void)
#line 65
{
  error_t result;

#line 67
  if ((result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperature()) != SUCCESS) {
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$release();
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$readDone(result, 0);
    }
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$readDone(error_t arg_0x40624580, /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$val_t arg_0x40624708){
#line 63
  UBee430_APC$Humidity$readDone(arg_0x40624580, arg_0x40624708);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$release(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$release(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 121 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline  error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidity(uint8_t client)
#line 121
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$on) {
#line 122
      return EOFF;
    }
#line 123
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy) {
#line 123
      return EBUSY;
    }
  else 
#line 123
    {
#line 123
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = TRUE;
    }
#line 124
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_HUMIDITY;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$performCommand();
}

# 76 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidity(void){
#line 76
  unsigned char result;
#line 76

#line 76
  result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidity(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY);
#line 76

#line 76
  return result;
#line 76
}
#line 76
# 85 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$granted(void)
#line 85
{
  error_t result;

#line 87
  if ((result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidity()) != SUCCESS) {
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$release();
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$readDone(result, 0);
    }
}

# 185 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$default$granted(uint8_t id)
#line 185
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$granted(uint8_t arg_0x40b9d968){
#line 92
  switch (arg_0x40b9d968) {
#line 92
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 92
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$granted();
#line 92
      break;
#line 92
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 92
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$default$granted(arg_0x40b9d968);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 199 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 199
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x40b9b4d8){
#line 49
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(arg_0x40b9b4d8);
#line 49
}
#line 49
# 173 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline  void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$runTask(void)
#line 173
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 174
    {
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$reqResId;
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_BUSY;
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$configure(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId);
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$granted(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId);
}

# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$disable(void){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set(void){
#line 29
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$set();
#line 29
}
#line 29




inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput(void){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeInput();
#line 33
}
#line 33
#line 30
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr(void){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$clr();
#line 30
}
#line 30
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeOutput(void)
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t *)34U |= 0x01 << 6;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$makeOutput(void){
#line 35
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$makeOutput();
#line 35
}
#line 35
# 220 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$initPins(void)
#line 220
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$disable();
}

# 88 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port15$disable(void)
#line 88
{
#line 88
  P1IE &= ~(1 << 5);
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$disable(void){
#line 36
  HplMsp430InterruptP$Port15$disable();
#line 36
}
#line 36
# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$set(void)
#line 37
{
#line 37
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set(void){
#line 29
  /*HplSensirionSht11C.SCKM*/Msp430GpioC$4$GeneralIO$set();
#line 29
}
#line 29
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput(void){
#line 35
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$makeOutput();
#line 35
}
#line 35
# 228 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$resetDevice(void)
#line 228
{
  uint8_t i;

#line 230
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  for (i = 0; i < 9; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
    }
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$clr(void){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$clr();
#line 30
}
#line 30
# 239 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$transmissionStart(void)
#line 239
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
}

static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sendCommand(uint8_t _cmd)
#line 251
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeByte(_cmd);
}

# 48 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$getRaw(void)
#line 48
{
#line 48
  return * (volatile uint8_t *)32U & (0x01 << 5);
}

#line 49
static inline   bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$get(void)
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$getRaw() != 0;
}

# 59 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   bool /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$get(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   bool /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$get(void)
#line 40
{
#line 40
  return /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$HplGeneralIO$get();
}

# 32 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$get(void){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplSensirionSht11C.DATAM*/Msp430GpioC$3$GeneralIO$get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 80 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port15$enable(void)
#line 80
{
#line 80
  P1IE |= 1 << 5;
}

# 31 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$enable(void){
#line 31
  HplMsp430InterruptP$Port15$enable();
#line 31
}
#line 31
# 137 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port15$edge(bool l2h)
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (l2h) {
#line 139
      P1IES &= ~(1 << 5);
      }
    else {
#line 140
      P1IES |= 1 << 5;
      }
  }
}

# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$edge(bool arg_0x40aceef8){
#line 56
  HplMsp430InterruptP$Port15$edge(arg_0x40aceef8);
#line 56
}
#line 56
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$edge(rising);
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}





static inline   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$enableFallingEdge(void)
#line 54
{
  return /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$enable(FALSE);
}

# 43 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$enableFallingEdge(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$enableFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 372 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$ack(void)
#line 372
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 50 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void)
#line 50
{
  return /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$isEmpty(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 58 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void)
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
        uint8_t id = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead;

#line 62
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead];
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
          }
#line 65
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[id] = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$dequeue(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 96 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$granted(void)
#line 96
{
  /* atomic removed: atomic calls only */
#line 97
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopping = TRUE;
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$postTask();
}

# 46 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted(void){
#line 46
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$granted();
#line 46
}
#line 46
# 201 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 201
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x40b9b4d8){
#line 55
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x40b9b4d8);
#line 55
}
#line 55
# 78 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumidityMetadata$getSignificantBits(void)
#line 78
{
#line 78
  return SHT11_HUMIDITY_BITS;
}

# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
inline static  uint8_t UBee430_APC$HumidityMetadata$getSignificantBits(void){
#line 41
  unsigned char result;
#line 41

#line 41
  result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumidityMetadata$getSignificantBits();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 58 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  uint8_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TemperatureMetadata$getSignificantBits(void)
#line 58
{
#line 58
  return SHT11_TEMPERATURE_BITS;
}

# 41 "/opt/tinyos-2.x/tos/interfaces/DeviceMetadata.nc"
inline static  uint8_t UBee430_APC$TemperatureMetadata$getSignificantBits(void){
#line 41
  unsigned char result;
#line 41

#line 41
  result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TemperatureMetadata$getSignificantBits();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 101 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stopDone(error_t error)
#line 101
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$requested == TRUE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$StdControl$start();
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$start();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 106
    {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$requested = FALSE;
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopping = FALSE;
    }
#line 109
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void HplSensirionSht11P$SplitControl$stopDone(error_t arg_0x4062a6e8){
#line 117
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$stopDone(arg_0x4062a6e8);
#line 117
}
#line 117
# 73 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline  void HplSensirionSht11P$stopTask$runTask(void)
#line 73
{
  HplSensirionSht11P$SplitControl$stopDone(SUCCESS);
}

# 99 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureHumidityDone(error_t result, uint16_t val)
#line 99
{
}

#line 93
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidityDone(error_t result, uint16_t val)
#line 93
{
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$release();
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$readDone(result, val);
}

# 408 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureHumidityDone(uint8_t client, error_t result, uint16_t val)
#line 408
{
}

# 84 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidityDone(uint8_t arg_0x40a5fc58, error_t arg_0x40a3c088, uint16_t arg_0x40a3c218){
#line 84
  switch (arg_0x40a5fc58) {
#line 84
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 84
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureHumidityDone(arg_0x40a3c088, arg_0x40a3c218);
#line 84
      break;
#line 84
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 84
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureHumidityDone(arg_0x40a3c088, arg_0x40a3c218);
#line 84
      break;
#line 84
    default:
#line 84
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureHumidityDone(arg_0x40a5fc58, arg_0x40a3c088, arg_0x40a3c218);
#line 84
      break;
#line 84
    }
#line 84
}
#line 84
# 73 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperatureDone(error_t result, uint16_t val)
#line 73
{
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$release();
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$readDone(result, val);
}

#line 104
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureTemperatureDone(error_t result, uint16_t val)
#line 104
{
}

# 407 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureTemperatureDone(uint8_t client, error_t result, uint16_t val)
#line 407
{
}

# 69 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperatureDone(uint8_t arg_0x40a5fc58, error_t arg_0x40a3e5e0, uint16_t arg_0x40a3e770){
#line 69
  switch (arg_0x40a5fc58) {
#line 69
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 69
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$measureTemperatureDone(arg_0x40a3e5e0, arg_0x40a3e770);
#line 69
      break;
#line 69
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 69
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$measureTemperatureDone(arg_0x40a3e5e0, arg_0x40a3e770);
#line 69
      break;
#line 69
    default:
#line 69
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$measureTemperatureDone(arg_0x40a5fc58, arg_0x40a3e5e0, arg_0x40a3e770);
#line 69
      break;
#line 69
    }
#line 69
}
#line 69
# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$stop(void){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(2U);
#line 67
}
#line 67
# 320 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$runTask(void)
#line 320
{
  uint16_t data = 0;
  uint8_t crc = 0;

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy == FALSE) {


      return;
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$stop();

  data = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte() << 8;
  data |= /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte();

  crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte();

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$endTransmission();

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, SUCCESS, data);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, SUCCESS, data);
      break;

      default: 
        break;
    }
}

# 101 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$writeStatusRegDone(error_t result)
#line 101
{
}



static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$writeStatusRegDone(error_t result)
#line 106
{
}

# 410 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$writeStatusRegDone(uint8_t client, error_t result)
#line 410
{
}

# 116 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$writeStatusRegDone(uint8_t arg_0x40a5fc58, error_t arg_0x40a3b7d8){
#line 116
  switch (arg_0x40a5fc58) {
#line 116
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 116
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$writeStatusRegDone(arg_0x40a3b7d8);
#line 116
      break;
#line 116
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 116
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$writeStatusRegDone(arg_0x40a3b7d8);
#line 116
      break;
#line 116
    default:
#line 116
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$writeStatusRegDone(arg_0x40a5fc58, arg_0x40a3b7d8);
#line 116
      break;
#line 116
    }
#line 116
}
#line 116
# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$readStatusRegDone(error_t result, uint8_t val)
#line 100
{
}



static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$readStatusRegDone(error_t result, uint8_t val)
#line 105
{
}

# 409 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$readStatusRegDone(uint8_t client, error_t result, uint8_t val)
#line 409
{
}

# 100 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$readStatusRegDone(uint8_t arg_0x40a5fc58, error_t arg_0x40a3cb70, uint8_t arg_0x40a3ccf8){
#line 100
  switch (arg_0x40a5fc58) {
#line 100
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 100
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$readStatusRegDone(arg_0x40a3cb70, arg_0x40a3ccf8);
#line 100
      break;
#line 100
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 100
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$readStatusRegDone(arg_0x40a3cb70, arg_0x40a3ccf8);
#line 100
      break;
#line 100
    default:
#line 100
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$readStatusRegDone(arg_0x40a5fc58, arg_0x40a3cb70, arg_0x40a3ccf8);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 388 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$runTask(void)
#line 388
{
  bool _writeFail = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeFail;

#line 390
  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_READ_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$readStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, SUCCESS, /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status);
      break;
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeFail = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$writeStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, _writeFail ? FAIL : SUCCESS);
      break;
      default: 

        break;
    }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x4092b598, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x4092b728){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(arg_0x4092b598, arg_0x4092b728);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t arg_0x40684258, uint32_t arg_0x406843e8){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(arg_0x40684258, arg_0x406843e8);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void)
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void)
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 88 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void)
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 123
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t UBee430_APC$rt_Send$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(UBee430_APC$rt_Send);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void)
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t *)49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void)
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$toggle(void){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle();
#line 31
}
#line 31
# 73 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led0Toggle(void)
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static   void UBee430_APC$Leds$led0Toggle(void){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 215 "UBee430_APC.nc"
static inline  void UBee430_APC$Timer0$fired(void)
#line 215
{
  UBee430_APC$Leds$led0Toggle();
  UBee430_APC$next$postTask();
  if (!UBee430_APC$boot_tx_flag) {
      UBee430_APC$boot_tx_flag = 1;
      UBee430_APC$rt_Send$postTask();
    }
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 126 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void)
#line 126
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$default_owner_id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_GRANTING) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$postTask();
              {
                unsigned char __nesc_temp = 
#line 131
                SUCCESS;

                {
#line 131
                  __nesc_atomic_end(__nesc_atomic); 
#line 131
                  return __nesc_temp;
                }
              }
            }
          else {
#line 133
            if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_IMM_GRANTING) {
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$reqResId;
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 136
                  SUCCESS;

                  {
#line 136
                    __nesc_atomic_end(__nesc_atomic); 
#line 136
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
#line 140
  return FAIL;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$release(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release();
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 92 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline  void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$startDone(error_t error)
#line 92
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$release();
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void HplSensirionSht11P$SplitControl$startDone(error_t arg_0x4062baf0){
#line 92
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$SplitControl$startDone(arg_0x4062baf0);
#line 92
}
#line 92
# 59 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline  void HplSensirionSht11P$Timer$fired(void)
#line 59
{
  HplSensirionSht11P$SplitControl$startDone(SUCCESS);
}

# 98 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$resetDone(error_t result)
#line 98
{
}



static inline  void /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$resetDone(error_t result)
#line 103
{
}

# 406 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$resetDone(uint8_t client, error_t result)
#line 406
{
}

# 54 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$resetDone(uint8_t arg_0x40a5fc58, error_t arg_0x40a3fd10){
#line 54
  switch (arg_0x40a5fc58) {
#line 54
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY:
#line 54
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Temp$resetDone(arg_0x40a3fd10);
#line 54
      break;
#line 54
    case /*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY:
#line 54
      /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Sht11Hum$resetDone(arg_0x40a3fd10);
#line 54
      break;
#line 54
    default:
#line 54
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$default$resetDone(arg_0x40a5fc58, arg_0x40a3fd10);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 287 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$fired(void)
#line 287
{

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_SOFT_RESET: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$resetDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, SUCCESS);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_TEMPERATURE: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, FAIL, 0);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_HUMIDITY: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$SensirionSht11$measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$currentClient, FAIL, 0);
      break;

      default: 

        break;
    }
}

# 220 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 
        Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 
        Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

          case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 

            default: 

              return;
    }
}

static inline  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
            Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(SUCCESS);
          }
        else {
#line 253
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
          }
#line 254
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
            Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(SUCCESS);
          }
        else {
#line 260
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
          }
#line 261
      break;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

        case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

          case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

            default: 

              return;
    }
}

# 189 "UBee430_APC.nc"
static inline  void UBee430_APC$DS28$EEPROM_WriteDone(void)
#line 189
{
}

# 12 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  void Ds28dg02P$Ds28dg02s$EEPROM_WriteDone(void){
#line 12
  UBee430_APC$DS28$EEPROM_WriteDone();
#line 12
}
#line 12
# 315 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  void Ds28dg02P$Timer0$fired(void)
{


  Ds28dg02P$Ds28dg02s$EEPROM_WriteDone();
}

# 192 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline   void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x409e58a8){
#line 72
  switch (arg_0x409e58a8) {
#line 72
    case 0U:
#line 72
      UBee430_APC$Timer0$fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      HplSensirionSht11P$Timer$fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      Msp430RefVoltGeneratorP$SwitchOnTimer$fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      Msp430RefVoltGeneratorP$SwitchOffTimer$fired();
#line 72
      break;
#line 72
    case 5U:
#line 72
      Ds28dg02P$Timer0$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(arg_0x409e58a8);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 127 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void)
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t UBee430_APC$Send$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(UBee430_APC$Send);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  error_t UBee430_APC$AdcZero$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static  error_t UBee430_APC$Photo$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$request(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$HUM_KEY);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 80 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$read(void)
#line 80
{
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$HumResource$request();
  return SUCCESS;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  error_t UBee430_APC$Humidity$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Humidity$read();
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$request(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$request(/*UBee430_APAppC.Sht11C.SensirionSht11C*/SensirionSht11C$0$TEMP_KEY);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 60 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline  error_t /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$read(void)
#line 60
{
  /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$TempResource$request();
  return SUCCESS;
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  error_t UBee430_APC$Temperature$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = /*UBee430_APAppC.Sht11C.SensirionSht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP$0$Temperature$read();
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static  error_t UBee430_APC$InternalTemp$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static  error_t UBee430_APC$InternalVolt$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 69 "UBee430_APC.nc"
static inline  void UBee430_APC$next$runTask(void)
#line 69
{
  UBee430_APC$count = 0;
  UBee430_APC$InternalVolt$read();
  UBee430_APC$InternalTemp$read();
  UBee430_APC$Temperature$read();
  UBee430_APC$Humidity$read();
  UBee430_APC$Photo$read();
  UBee430_APC$AdcZero$read();
  UBee430_APC$Send$postTask();
}

# 155 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline   uint8_t Msp430RefVoltArbiterImplP$ClientResource$isOwner(uint8_t client)
{
  return Msp430RefVoltArbiterImplP$AdcResource$isOwner(client);
}

# 327 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    bool AdcP$ResourceRead$default$isOwner(uint8_t client)
#line 327
{
#line 327
  return FALSE;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool AdcP$ResourceRead$isOwner(uint8_t arg_0x40c0a7c8){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x40c0a7c8) {
#line 118
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 118
      result = Msp430RefVoltArbiterImplP$ClientResource$isOwner(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 118
      result = Msp430RefVoltArbiterImplP$ClientResource$isOwner(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 118
      result = Msp430RefVoltArbiterImplP$ClientResource$isOwner(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 118
      result = Msp430RefVoltArbiterImplP$ClientResource$isOwner(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = AdcP$ResourceRead$default$isOwner(arg_0x40c0a7c8);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline   error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP$AdcResource$request(client);
}

# 324 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceRead$default$request(uint8_t client)
#line 324
{
#line 324
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceRead$request(uint8_t arg_0x40c0a7c8){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x40c0a7c8) {
#line 78
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = AdcP$ResourceRead$default$request(arg_0x40c0a7c8);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 187 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline    void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 187
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$requested(uint8_t arg_0x40b9c308){
#line 43
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$default$requested(arg_0x40b9c308);
#line 43
}
#line 43
# 54 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline   bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id)
#line 54
{
  return /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[id] != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY || /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qTail == id;
}

#line 72
static inline   error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(id)) {
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = id;
          }
        else {
#line 78
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qTail] = id;
          }
#line 79
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x40b828b0){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(arg_0x40b828b0);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 74 "/opt/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline   void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$requested(void)
#line 74
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopping == FALSE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$postTask();
    }
  else {
#line 78
    /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$requested = TRUE;
    }
}

# 73 "/opt/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static   void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$requested(void){
#line 73
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$ResourceDefaultOwner$requested();
#line 73
}
#line 73
# 69 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static  error_t UBee430_APC$AMSend$send(am_addr_t arg_0x406682c0, message_t *arg_0x40668470, uint8_t arg_0x406685f8){
#line 69
  unsigned char result;
#line 69

#line 69
  result = CC2420ActiveMessageP$AMSend$send(AM_MSG, arg_0x406682c0, arg_0x40668470, arg_0x406685f8);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 80 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  void *CC2420ActiveMessageP$AMSend$getPayload(am_id_t id, message_t *m)
#line 80
{
  return CC2420ActiveMessageP$Packet$getPayload(m, (void *)0);
}

# 125 "/opt/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static  void *UBee430_APC$AMSend$getPayload(message_t *arg_0x40665248){
#line 125
  void *result;
#line 125

#line 125
  result = CC2420ActiveMessageP$AMSend$getPayload(AM_MSG, arg_0x40665248);
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 191 "UBee430_APC.nc"
static inline  void UBee430_APC$rt_Send$runTask(void)
#line 191
{
  memcpy(UBee430_APC$AMSend$getPayload(&UBee430_APC$packet), &UBee430_APC$rtm, sizeof UBee430_APC$rtm);
  if (UBee430_APC$AMSend$send(AM_BROADCAST_ADDR, &UBee430_APC$packet, sizeof UBee430_APC$rtm) == SUCCESS) {
    }
}

# 160 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline  uint8_t CC2420ActiveMessageP$Packet$payloadLength(message_t *msg)
#line 160
{
  return __nesc_ntoh_leuint8((unsigned char *)&CC2420ActiveMessageP$CC2420PacketBody$getHeader(msg)->length) - CC2420ActiveMessageP$CC2420_SIZE;
}

# 272 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   uint16_t CC2420ControlP$CC2420Config$getPanAddr(void)
#line 272
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 273
    {
      unsigned int __nesc_temp = 
#line 273
      CC2420ControlP$m_pan;

      {
#line 273
        __nesc_atomic_end(__nesc_atomic); 
#line 273
        return __nesc_temp;
      }
    }
#line 275
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static   uint16_t CC2420ActiveMessageP$CC2420Config$getPanAddr(void){
#line 70
  unsigned int result;
#line 70

#line 70
  result = CC2420ControlP$CC2420Config$getPanAddr();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 251 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_hton_leuint8(void *target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

#line 281
static __inline uint16_t __nesc_hton_leuint16(void *target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 511 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$send(message_t *p_msg, bool cca)
#line 511
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 512
    {


      if ((
#line 513
      CC2420TransmitP$m_state == CC2420TransmitP$S_LOAD_CANCEL
       || CC2420TransmitP$m_state == CC2420TransmitP$S_CCA_CANCEL)
       || CC2420TransmitP$m_state == CC2420TransmitP$S_TX_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 516
            ECANCEL;

            {
#line 516
              __nesc_atomic_end(__nesc_atomic); 
#line 516
              return __nesc_temp;
            }
          }
        }
#line 519
      if (CC2420TransmitP$m_state != CC2420TransmitP$S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 520
            FAIL;

            {
#line 520
              __nesc_atomic_end(__nesc_atomic); 
#line 520
              return __nesc_temp;
            }
          }
        }
#line 523
      CC2420TransmitP$m_state = CC2420TransmitP$S_LOAD;
      CC2420TransmitP$m_cca = cca;
      CC2420TransmitP$m_msg = p_msg;
      CC2420TransmitP$totalCcaChecks = 0;
    }
#line 527
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP$acquireSpiResource() == SUCCESS) {
      CC2420TransmitP$loadTXFIFO();
    }

  return SUCCESS;
}

#line 172
static inline   error_t CC2420TransmitP$Send$send(message_t *p_msg, bool useCca)
#line 172
{
  return CC2420TransmitP$send(p_msg, useCca);
}

# 51 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static   error_t CC2420CsmaP$CC2420Transmit$send(message_t *arg_0x40efd620, bool arg_0x40efd7a8){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420TransmitP$Send$send(arg_0x40efd620, arg_0x40efd7a8);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 287 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline    void CC2420CsmaP$RadioBackoff$default$requestCca(am_id_t amId, 
message_t *msg)
#line 288
{
}

# 95 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static   void CC2420CsmaP$RadioBackoff$requestCca(am_id_t arg_0x40f0a320, message_t *arg_0x40e9ccf8){
#line 95
    CC2420CsmaP$RadioBackoff$default$requestCca(arg_0x40f0a320, arg_0x40e9ccf8);
#line 95
}
#line 95
# 57 "/opt/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static  am_addr_t CC2420CsmaP$AMPacket$address(void){
#line 57
  unsigned int result;
#line 57

#line 57
  result = CC2420ActiveMessageP$AMPacket$address();
#line 57

#line 57
  return result;
#line 57
}
#line 57
# 47 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_metadata_t *CC2420CsmaP$CC2420PacketBody$getMetadata(message_t *arg_0x40ead0d0){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketC$CC2420PacketBody$getMetadata(arg_0x40ead0d0);
#line 47

#line 47
  return result;
#line 47
}
#line 47
#line 42
inline static   cc2420_header_t *CC2420CsmaP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 124 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline  error_t CC2420CsmaP$Send$send(message_t *p_msg, uint8_t len)
#line 124
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 126
  cc2420_header_t *header = CC2420CsmaP$CC2420PacketBody$getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP$CC2420PacketBody$getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (!CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 131
            FAIL;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
#line 134
      CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_TRANSMITTING);
      CC2420CsmaP$m_msg = p_msg;
    }
#line 136
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8((unsigned char *)&header->length, len);
  (__nesc_temp42 = (unsigned char *)&header->fcf, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (1 << IEEE154_FCF_ACK_REQ)));
  (__nesc_temp43 = (unsigned char *)&header->fcf, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));
  __nesc_hton_leuint16((unsigned char *)&header->src, CC2420CsmaP$AMPacket$address());
  __nesc_hton_int8((unsigned char *)&metadata->ack, FALSE);
  __nesc_hton_uint8((unsigned char *)&metadata->rssi, 0);
  __nesc_hton_uint8((unsigned char *)&metadata->lqi, 0);
  __nesc_hton_uint16((unsigned char *)&metadata->time, 0);

  CC2420CsmaP$ccaOn = TRUE;
  CC2420CsmaP$RadioBackoff$requestCca(__nesc_ntoh_leuint8((unsigned char *)&((cc2420_header_t *)(CC2420CsmaP$m_msg->data - 
  sizeof(cc2420_header_t )))->type), CC2420CsmaP$m_msg);
  CC2420CsmaP$CC2420Transmit$send(CC2420CsmaP$m_msg, CC2420CsmaP$ccaOn);
  return SUCCESS;
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static  error_t UniqueSendP$SubSend$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790){
#line 64
  unsigned char result;
#line 64

#line 64
  result = CC2420CsmaP$Send$send(arg_0x40eb4608, arg_0x40eb4790);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 42 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static   cc2420_header_t *UniqueSendP$CC2420PacketBody$getHeader(message_t *arg_0x40eaeb68){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketC$CC2420PacketBody$getHeader(arg_0x40eaeb68);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 45 "/opt/tinyos-2.x/tos/interfaces/State.nc"
inline static   error_t UniqueSendP$State$requestState(uint8_t arg_0x40f27230){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP$State$requestState(2U, arg_0x40f27230);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 75 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline  error_t UniqueSendP$Send$send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP$State$requestState(UniqueSendP$S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8((unsigned char *)&UniqueSendP$CC2420PacketBody$getHeader(msg)->dsn, UniqueSendP$localSendId++);

      if ((error = UniqueSendP$SubSend$send(msg, len)) != SUCCESS) {
          UniqueSendP$State$toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 64 "/opt/tinyos-2.x/tos/interfaces/Send.nc"
inline static  error_t CC2420ActiveMessageP$SubSend$send(message_t *arg_0x40eb4608, uint8_t arg_0x40eb4790){
#line 64
  unsigned char result;
#line 64

#line 64
  result = UniqueSendP$Send$send(arg_0x40eb4608, arg_0x40eb4790);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 215 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number5(void)
#line 215
{
  uint8_t temp;

#line 217
  temp = Ds28dg02P$SPI_READ(0x0b, 0x19);
  return temp;
}

# 22 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number5(void){
#line 22
  unsigned char result;
#line 22

#line 22
  result = Ds28dg02P$Ds28dg02s$ReadID_number5();
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 220 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number4(void)
#line 220
{
  uint8_t temp;

#line 222
  temp = Ds28dg02P$SPI_READ(0x0b, 0x1a);
  return temp;
}

# 21 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number4(void){
#line 21
  unsigned char result;
#line 21

#line 21
  result = Ds28dg02P$Ds28dg02s$ReadID_number4();
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 225 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number3(void)
#line 225
{
  uint8_t temp;

#line 227
  temp = Ds28dg02P$SPI_READ(0x0b, 0x1b);
  return temp;
}

# 20 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number3(void){
#line 20
  unsigned char result;
#line 20

#line 20
  result = Ds28dg02P$Ds28dg02s$ReadID_number3();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 230 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number2(void)
#line 230
{
  uint8_t temp;

#line 232
  temp = Ds28dg02P$SPI_READ(0x0b, 0x1c);
  return temp;
}

# 19 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number2(void){
#line 19
  unsigned char result;
#line 19

#line 19
  result = Ds28dg02P$Ds28dg02s$ReadID_number2();
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 235 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number1(void)
#line 235
{
  uint8_t temp;

#line 237
  temp = Ds28dg02P$SPI_READ(0x0b, 0x1d);
  return temp;
}

# 18 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number1(void){
#line 18
  unsigned char result;
#line 18

#line 18
  result = Ds28dg02P$Ds28dg02s$ReadID_number1();
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 240 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadID_number0(void)
#line 240
{
  uint8_t temp;

#line 242
  temp = Ds28dg02P$SPI_READ(0x0b, 0x1e);
  return temp;
}

# 17 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadID_number0(void){
#line 17
  unsigned char result;
#line 17

#line 17
  result = Ds28dg02P$Ds28dg02s$ReadID_number0();
#line 17

#line 17
  return result;
#line 17
}
#line 17
# 60 "UBee430_APC.nc"
static inline void UBee430_APC$getID(void)
#line 60
{
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[0], UBee430_APC$DS28$ReadID_number0());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[1], UBee430_APC$DS28$ReadID_number1());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[2], UBee430_APC$DS28$ReadID_number2());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[3], UBee430_APC$DS28$ReadID_number3());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[4], UBee430_APC$DS28$ReadID_number4());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.ID[5], UBee430_APC$DS28$ReadID_number5());
}

# 282 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_years(void)
#line 282
{
  uint8_t temp;

#line 284
  temp = Ds28dg02P$SPI_READ(0x0b, 0x2f);
  return temp;
}

# 31 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_years(void){
#line 31
  unsigned char result;
#line 31

#line 31
  result = Ds28dg02P$Ds28dg02s$ReadRTC_years();
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 277 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_months(void)
#line 277
{
  uint8_t temp;

#line 279
  temp = Ds28dg02P$SPI_READ(0x0b, 0x2e);
  return temp;
}

# 30 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_months(void){
#line 30
  unsigned char result;
#line 30

#line 30
  result = Ds28dg02P$Ds28dg02s$ReadRTC_months();
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 272 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_date(void)
#line 272
{
  uint8_t temp;

#line 274
  temp = Ds28dg02P$SPI_READ(0x0b, 0x2d);
  return temp;
}

# 29 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_date(void){
#line 29
  unsigned char result;
#line 29

#line 29
  result = Ds28dg02P$Ds28dg02s$ReadRTC_date();
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 262 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_hours(void)
#line 262
{
  uint8_t temp;

#line 264
  temp = Ds28dg02P$SPI_READ(0x0b, 0x2b);
  return temp;
}

# 27 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_hours(void){
#line 27
  unsigned char result;
#line 27

#line 27
  result = Ds28dg02P$Ds28dg02s$ReadRTC_hours();
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 257 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_minutes(void)
#line 257
{
  uint8_t temp;

#line 259
  temp = Ds28dg02P$SPI_READ(0x0b, 0x2a);
  return temp;
}

# 26 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_minutes(void){
#line 26
  unsigned char result;
#line 26

#line 26
  result = Ds28dg02P$Ds28dg02s$ReadRTC_minutes();
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 252 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static inline  uint8_t Ds28dg02P$Ds28dg02s$ReadRTC_seconds(void)
#line 252
{
  uint8_t temp;

#line 254
  temp = Ds28dg02P$SPI_READ(0x0b, 0x29);
  return temp;
}

# 25 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02s.nc"
inline static  uint8_t UBee430_APC$DS28$ReadRTC_seconds(void){
#line 25
  unsigned char result;
#line 25

#line 25
  result = Ds28dg02P$Ds28dg02s$ReadRTC_seconds();
#line 25

#line 25
  return result;
#line 25
}
#line 25
# 51 "UBee430_APC.nc"
static inline void UBee430_APC$getRTC(void)
#line 51
{
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[5], UBee430_APC$DS28$ReadRTC_seconds());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[4], UBee430_APC$DS28$ReadRTC_minutes());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[3], UBee430_APC$DS28$ReadRTC_hours());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[2], UBee430_APC$DS28$ReadRTC_date());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[1], UBee430_APC$DS28$ReadRTC_months());
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.RTC[0], UBee430_APC$DS28$ReadRTC_years());
}

#line 80
static inline  void UBee430_APC$Send$runTask(void)
#line 80
{
  if (UBee430_APC$count < 6) {
    UBee430_APC$Send$postTask();
    }
  else 
#line 83
    {

      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.start, 0x02);
      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.etx, 0x03);
      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.msg_type, 'S');
      __nesc_hton_uint16((unsigned char *)&UBee430_APC$message.node_id, TOS_NODE_ID);
      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.device_type, TOS_NODE_ID);


      UBee430_APC$getRTC();
      UBee430_APC$getID();
      memcpy(UBee430_APC$AMSend$getPayload(&UBee430_APC$packet), &UBee430_APC$message, sizeof UBee430_APC$message);
      if (UBee430_APC$AMSend$send(AM_BROADCAST_ADDR, &UBee430_APC$packet, sizeof UBee430_APC$message) == SUCCESS) {
        }
#line 96
      UBee430_APC$locked = TRUE;
    }
}

# 113 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline uint8_t TOSH_READ_DS28DG02_SO_PIN(void)
#line 113
{
#line 113
   static volatile uint8_t r __asm ("0x0028");

#line 113
  return r & (1 << 1);
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4393 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void)
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(x);
}

#line 94
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void)
{
  * (volatile uint16_t *)386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl();
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline  error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void)
#line 45
{
  memset(/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ, /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY, sizeof /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$resQ);
  return SUCCESS;
}

# 111 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$stopConversion(void)
#line 111
{
  HplAdc12P$ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P$ADC12CTL0 &= ~0x0010;
}

# 123 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$stopConversion(void){
#line 123
  HplAdc12P$HplAdc12$stopConversion();
#line 123
}
#line 123
# 87 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline  error_t Msp430Adc12ImplP$Init$init(void)
{
  Msp430Adc12ImplP$HplAdc12$stopConversion();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void)
#line 51
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ);
  return SUCCESS;
}

# 82 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline   am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void)
#line 82
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 84
  myGroup = ActiveMessageAddressC$group;
  return myGroup;
}

# 53 "/opt/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static   am_group_t CC2420ControlP$ActiveMessageAddress$amGroup(void){
#line 53
  unsigned char result;
#line 53

#line 53
  result = ActiveMessageAddressC$ActiveMessageAddress$amGroup();
#line 53

#line 53
  return result;
#line 53
}
#line 53
#line 48
inline static   am_addr_t CC2420ControlP$ActiveMessageAddress$amAddress(void){
#line 48
  unsigned int result;
#line 48

#line 48
  result = ActiveMessageAddressC$ActiveMessageAddress$amAddress();
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)30U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP$29$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$VREN$makeOutput(void){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC$12$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)30U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$RSTN$makeOutput(void){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC$10$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)30U |= 0x01 << 2;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420ControlP$CSN$makeOutput(void){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$makeOutput();
#line 35
}
#line 35
# 116 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline  error_t CC2420ControlP$Init$init(void)
#line 116
{
  CC2420ControlP$CSN$makeOutput();
  CC2420ControlP$RSTN$makeOutput();
  CC2420ControlP$VREN$makeOutput();

  CC2420ControlP$m_short_addr = CC2420ControlP$ActiveMessageAddress$amAddress();
  CC2420ControlP$m_pan = CC2420ControlP$ActiveMessageAddress$amGroup();
  CC2420ControlP$m_tx_power = 31;
  CC2420ControlP$m_channel = 26;




  CC2420ControlP$autoAckEnabled = TRUE;





  CC2420ControlP$hwAutoAckDefault = FALSE;





  CC2420ControlP$addressRecognition = TRUE;


  return SUCCESS;
}

# 81 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static inline  error_t StateImplP$Init$init(void)
#line 81
{
  int i;

#line 83
  for (i = 0; i < 4U; i++) {
      StateImplP$state[i] = StateImplP$S_IDLE;
    }
  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline  error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void)
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$resQ);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$__nesc_unnamed4394 {
#line 46
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$compareControl(void)
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$CC2int(x);
}

#line 94
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$setControlAsCompare(void)
{
  * (volatile uint16_t *)390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$compareControl();
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$setControlAsCompare(void){
#line 36
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Init$init(void)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)30U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP$25$IO$makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$makeInput(void)
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$HplGeneralIO$makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420TransmitP$SFD$makeInput(void){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC$11$GeneralIO$makeInput();
#line 33
}
#line 33


inline static   void CC2420TransmitP$CSN$makeOutput(void){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC$7$GeneralIO$makeOutput();
#line 35
}
#line 35
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)34U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP$4$IO$makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$makeInput(void)
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$HplGeneralIO$makeInput();
}

# 33 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void CC2420TransmitP$CCA$makeInput(void){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC$6$GeneralIO$makeInput();
#line 33
}
#line 33
# 140 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline  error_t CC2420TransmitP$Init$init(void)
#line 140
{
  CC2420TransmitP$CCA$makeInput();
  CC2420TransmitP$CSN$makeOutput();
  CC2420TransmitP$SFD$makeInput();
  return SUCCESS;
}

# 112 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline  error_t CC2420ReceiveP$Init$init(void)
#line 112
{
  CC2420ReceiveP$m_p_rx_buf = &CC2420ReceiveP$m_rx_buf;
  return SUCCESS;
}

# 44 "/opt/tinyos-2.x/tos/system/RandomMlcgP.nc"
static inline  error_t RandomMlcgP$Init$init(void)
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgP$seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 41 "/opt/tinyos-2.x/tos/interfaces/Random.nc"
inline static   uint16_t UniqueSendP$Random$rand16(void){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgP$Random$rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 62 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline  error_t UniqueSendP$Init$init(void)
#line 62
{
  UniqueSendP$localSendId = UniqueSendP$Random$rand16();
  return SUCCESS;
}

# 71 "/opt/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline  error_t UniqueReceiveP$Init$init(void)
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP$receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP$receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t RealMainP$SoftwareInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = UniqueReceiveP$Init$init();
#line 51
  result = ecombine(result, UniqueSendP$Init$init());
#line 51
  result = ecombine(result, RandomMlcgP$Init$init());
#line 51
  result = ecombine(result, CC2420ReceiveP$Init$init());
#line 51
  result = ecombine(result, CC2420TransmitP$Init$init());
#line 51
  result = ecombine(result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC$1$Init$init());
#line 51
  result = ecombine(result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC$1$Init$init());
#line 51
  result = ecombine(result, StateImplP$Init$init());
#line 51
  result = ecombine(result, CC2420ControlP$Init$init());
#line 51
  result = ecombine(result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init());
#line 51
  result = ecombine(result, Msp430Adc12ImplP$Init$init());
#line 51
  result = ecombine(result, /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init());
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 69 "/opt/tinyos-2.x/tos/platforms/UBee430/hardware.h"
static inline void TOSH_MAKE_GIO57_OUTPUT(void)
#line 69
{
#line 69
   static volatile uint8_t r __asm ("0x0032");

#line 69
  r |= 1 << 7;
}

# 142 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void UBee430_APC$Timer0$startPeriodic(uint32_t arg_0x40686030){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(0U, arg_0x40686030);
#line 53
}
#line 53
# 100 "UBee430_APC.nc"
static inline  void UBee430_APC$Boot$booted(void)
#line 100
{
  UBee430_APC$SplitControl$start();
  UBee430_APC$Timer0$startPeriodic(100);
  TOSH_MAKE_GIO57_OUTPUT();
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.start, 0x02);


  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.msg_type, 'R');
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.device_type, TOS_NODE_ID);
  __nesc_hton_uint16((unsigned char *)&UBee430_APC$rtm.node_id, TOS_NODE_ID);
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.d0, '0');
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.d1, '0');
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.d2, '0');
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.d3, '0');
  __nesc_hton_uint8((unsigned char *)&UBee430_APC$rtm.etx, 0x03);
}

# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
inline static  void RealMainP$Boot$booted(void){
#line 49
  UBee430_APC$Boot$booted();
#line 49
}
#line 49
# 190 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 124 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline    mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void)
#line 124
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static   mcu_power_t McuSleepC$McuPowerOverride$lowestState(void){
#line 54
  unsigned char result;
#line 54

#line 54
  result = McuSleepC$McuPowerOverride$default$lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 66 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$getPowerState(void)
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM3;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 178 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 178
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC$computePowerState(void)
#line 104
{
  McuSleepC$powerState = mcombine(McuSleepC$getPowerState(), 
  McuSleepC$McuPowerOverride$lowestState());
}

static inline   void McuSleepC$McuSleep$sleep(void)
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC$dirty) {
      McuSleepC$computePowerState();
    }

  temp = McuSleepC$msp430PowerBits[McuSleepC$powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static   void SchedulerBasicP$McuSleep$sleep(void){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void)
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline  void SchedulerBasicP$Scheduler$taskLoop(void)
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  void RealMainP$Scheduler$taskLoop(void){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 183 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline   void CC2420ReceiveP$InterruptFIFOP$fired(void)
#line 183
{
  if (CC2420ReceiveP$m_state == CC2420ReceiveP$S_STARTED) {
      CC2420ReceiveP$beginReceive();
    }
  else {
      CC2420ReceiveP$m_missed_packets++;
    }
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$fired(void){
#line 57
  CC2420ReceiveP$InterruptFIFOP$fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline   void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$fired(void)
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$Interrupt$fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port10$fired(void){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC$2$HplInterrupt$fired();
#line 61
}
#line 61
# 92 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port11$clear(void)
#line 92
{
#line 92
  P1IFG &= ~(1 << 1);
}

#line 68
static inline    void HplMsp430InterruptP$Port11$default$fired(void)
#line 68
{
#line 68
  HplMsp430InterruptP$Port11$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port11$fired(void){
#line 61
  HplMsp430InterruptP$Port11$default$fired();
#line 61
}
#line 61
# 93 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port12$clear(void)
#line 93
{
#line 93
  P1IFG &= ~(1 << 2);
}

#line 69
static inline    void HplMsp430InterruptP$Port12$default$fired(void)
#line 69
{
#line 69
  HplMsp430InterruptP$Port12$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port12$fired(void){
#line 61
  HplMsp430InterruptP$Port12$default$fired();
#line 61
}
#line 61
# 94 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port13$clear(void)
#line 94
{
#line 94
  P1IFG &= ~(1 << 3);
}

#line 70
static inline    void HplMsp430InterruptP$Port13$default$fired(void)
#line 70
{
#line 70
  HplMsp430InterruptP$Port13$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port13$fired(void){
#line 61
  HplMsp430InterruptP$Port13$default$fired();
#line 61
}
#line 61
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t CC2420CsmaP$startDone_task$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$startDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 207 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline   void CC2420CsmaP$CC2420Power$startOscillatorDone(void)
#line 207
{
  CC2420CsmaP$startDone_task$postTask();
}

# 76 "/opt/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static   void CC2420ControlP$CC2420Power$startOscillatorDone(void){
#line 76
  CC2420CsmaP$CC2420Power$startOscillatorDone();
#line 76
}
#line 76
# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   error_t CC2420ControlP$InterruptCCA$disable(void){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 390 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline   void CC2420ControlP$InterruptCCA$fired(void)
#line 390
{
  CC2420ControlP$m_state = CC2420ControlP$S_XOSC_STARTED;
  CC2420ControlP$InterruptCCA$disable();
  CC2420ControlP$IOCFG1$write(0);
  CC2420ControlP$writeId();
  CC2420ControlP$CSN$set();
  CC2420ControlP$CSN$clr();
  CC2420ControlP$CC2420Power$startOscillatorDone();
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$fired(void){
#line 57
  CC2420ControlP$InterruptCCA$fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline   void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$fired(void)
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$Interrupt$fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port14$fired(void){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC$1$HplInterrupt$fired();
#line 61
}
#line 61
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 315 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline   void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$fired(void)
#line 315
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$disable();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$postTask();
}

# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$fired(void){
#line 57
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$fired();
#line 57
}
#line 57
# 96 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port15$clear(void)
#line 96
{
#line 96
  P1IFG &= ~(1 << 5);
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$clear(void){
#line 41
  HplMsp430InterruptP$Port15$clear();
#line 41
}
#line 41
# 66 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline   void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$fired(void)
#line 66
{
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$clear();
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$fired();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port15$fired(void){
#line 61
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$fired();
#line 61
}
#line 61
# 97 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port16$clear(void)
#line 97
{
#line 97
  P1IFG &= ~(1 << 6);
}

#line 73
static inline    void HplMsp430InterruptP$Port16$default$fired(void)
#line 73
{
#line 73
  HplMsp430InterruptP$Port16$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port16$fired(void){
#line 61
  HplMsp430InterruptP$Port16$default$fired();
#line 61
}
#line 61
# 98 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port17$clear(void)
#line 98
{
#line 98
  P1IFG &= ~(1 << 7);
}

#line 74
static inline    void HplMsp430InterruptP$Port17$default$fired(void)
#line 74
{
#line 74
  HplMsp430InterruptP$Port17$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port17$fired(void){
#line 61
  HplMsp430InterruptP$Port17$default$fired();
#line 61
}
#line 61
# 195 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port20$clear(void)
#line 195
{
#line 195
  P2IFG &= ~(1 << 0);
}

#line 171
static inline    void HplMsp430InterruptP$Port20$default$fired(void)
#line 171
{
#line 171
  HplMsp430InterruptP$Port20$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port20$fired(void){
#line 61
  HplMsp430InterruptP$Port20$default$fired();
#line 61
}
#line 61
# 196 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port21$clear(void)
#line 196
{
#line 196
  P2IFG &= ~(1 << 1);
}

#line 172
static inline    void HplMsp430InterruptP$Port21$default$fired(void)
#line 172
{
#line 172
  HplMsp430InterruptP$Port21$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port21$fired(void){
#line 61
  HplMsp430InterruptP$Port21$default$fired();
#line 61
}
#line 61
# 197 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port22$clear(void)
#line 197
{
#line 197
  P2IFG &= ~(1 << 2);
}

#line 173
static inline    void HplMsp430InterruptP$Port22$default$fired(void)
#line 173
{
#line 173
  HplMsp430InterruptP$Port22$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port22$fired(void){
#line 61
  HplMsp430InterruptP$Port22$default$fired();
#line 61
}
#line 61
# 198 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port23$clear(void)
#line 198
{
#line 198
  P2IFG &= ~(1 << 3);
}

#line 174
static inline    void HplMsp430InterruptP$Port23$default$fired(void)
#line 174
{
#line 174
  HplMsp430InterruptP$Port23$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port23$fired(void){
#line 61
  HplMsp430InterruptP$Port23$default$fired();
#line 61
}
#line 61
# 199 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port24$clear(void)
#line 199
{
#line 199
  P2IFG &= ~(1 << 4);
}

#line 175
static inline    void HplMsp430InterruptP$Port24$default$fired(void)
#line 175
{
#line 175
  HplMsp430InterruptP$Port24$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port24$fired(void){
#line 61
  HplMsp430InterruptP$Port24$default$fired();
#line 61
}
#line 61
# 200 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port25$clear(void)
#line 200
{
#line 200
  P2IFG &= ~(1 << 5);
}

#line 176
static inline    void HplMsp430InterruptP$Port25$default$fired(void)
#line 176
{
#line 176
  HplMsp430InterruptP$Port25$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port25$fired(void){
#line 61
  HplMsp430InterruptP$Port25$default$fired();
#line 61
}
#line 61
# 201 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port26$clear(void)
#line 201
{
#line 201
  P2IFG &= ~(1 << 6);
}

#line 177
static inline    void HplMsp430InterruptP$Port26$default$fired(void)
#line 177
{
#line 177
  HplMsp430InterruptP$Port26$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port26$fired(void){
#line 61
  HplMsp430InterruptP$Port26$default$fired();
#line 61
}
#line 61
# 202 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline   void HplMsp430InterruptP$Port27$clear(void)
#line 202
{
#line 202
  P2IFG &= ~(1 << 7);
}

#line 178
static inline    void HplMsp430InterruptP$Port27$default$fired(void)
#line 178
{
#line 178
  HplMsp430InterruptP$Port27$clear();
}

# 61 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static   void HplMsp430InterruptP$Port27$fired(void){
#line 61
  HplMsp430InterruptP$Port27$default$fired();
#line 61
}
#line 61
# 87 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i)
#line 87
{
  return *((uint16_t *)(int *)0x0140 + i);
}

# 89 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t arg_0x40c8ded8){
#line 89
  unsigned int result;
#line 89

#line 89
  result = HplAdc12P$HplAdc12$getMem(arg_0x40c8ded8);
#line 89

#line 89
  return result;
#line 89
}
#line 89
# 73 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl)
#line 73
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 75
  memCtlPtr += i;
  *memCtlPtr = * (uint8_t *)&memControl;
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t arg_0x40c8d1d8, adc12memctl_t arg_0x40c8d370){
#line 75
  HplAdc12P$HplAdc12$setMCtl(arg_0x40c8d1d8, arg_0x40c8d370);
#line 75
}
#line 75







inline static   adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t arg_0x40c8d928){
#line 82
  struct __nesc_unnamed4277 result;
#line 82

#line 82
  result = HplAdc12P$HplAdc12$getMCtl(arg_0x40c8d928);
#line 82

#line 82
  return result;
#line 82
}
#line 82
# 116 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$enableConversion(void)
#line 116
{
  HplAdc12P$ADC12CTL0 |= 0x0002;
}

# 133 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$enableConversion(void){
#line 133
  HplAdc12P$HplAdc12$enableConversion();
#line 133
}
#line 133
# 614 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 614
{
}

# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static   void Msp430Adc12ImplP$MultiChannel$dataReady(uint8_t arg_0x40c96068, uint16_t *arg_0x40c82e90, uint16_t arg_0x40c80068){
#line 110
    Msp430Adc12ImplP$MultiChannel$default$dataReady(arg_0x40c96068, arg_0x40c82e90, arg_0x40c80068);
#line 110
}
#line 110
# 617 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id)
#line 617
{
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static   void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(uint8_t arg_0x40c96818){
#line 54
    Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(arg_0x40c96818);
#line 54
}
#line 54
# 616 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id)
#line 616
{
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static   void Msp430Adc12ImplP$Overflow$memOverflow(uint8_t arg_0x40c96818){
#line 49
    Msp430Adc12ImplP$Overflow$default$memOverflow(arg_0x40c96818);
#line 49
}
#line 49
# 520 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv)
{
  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP$Overflow$memOverflow(Msp430Adc12ImplP$clientID);
        }
      else {
#line 526
        Msp430Adc12ImplP$Overflow$conversionTimeOverflow(Msp430Adc12ImplP$clientID);
        }
    }
#line 528
  switch (Msp430Adc12ImplP$state & Msp430Adc12ImplP$CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP$SINGLE_DATA: 
        Msp430Adc12ImplP$stopConversion();
      Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$HplAdc12$getMem(0));
      break;
      case Msp430Adc12ImplP$SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 537
          repeatContinue = Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, 
          Msp430Adc12ImplP$HplAdc12$getMem(0));
          if (repeatContinue == FAIL) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 541
          break;
        }

      case Msp430Adc12ImplP$MULTI_CHANNEL: 
        {
          uint16_t i = 0;

#line 547
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 549
          ++i < Msp430Adc12ImplP$numChannels);
          Msp430Adc12ImplP$resultBufferIndex += Msp430Adc12ImplP$numChannels;
          if (Msp430Adc12ImplP$resultBufferLength == Msp430Adc12ImplP$resultBufferIndex) {
              Msp430Adc12ImplP$stopConversion();
              Msp430Adc12ImplP$resultBuffer -= Msp430Adc12ImplP$resultBufferLength;
              Msp430Adc12ImplP$resultBufferIndex = 0;
              Msp430Adc12ImplP$MultiChannel$dataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$resultBuffer, Msp430Adc12ImplP$resultBufferLength);
            }
          else {
#line 556
            Msp430Adc12ImplP$HplAdc12$enableConversion();
            }
        }
#line 558
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 561
          uint16_t length;

#line 562
          if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 565
            length = Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex;
            }
#line 566
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 568
          ++i < length);
          Msp430Adc12ImplP$resultBufferIndex += length;

          if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 15) {
            return;
            }
          else {
#line 573
            if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 0) {
                adc12memctl_t memctl = Msp430Adc12ImplP$HplAdc12$getMCtl(0);

#line 575
                memctl.eos = 1;
                Msp430Adc12ImplP$HplAdc12$setMCtl(Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex, memctl);
              }
            else 
#line 577
              {
                Msp430Adc12ImplP$stopConversion();
                Msp430Adc12ImplP$resultBuffer -= Msp430Adc12ImplP$resultBufferLength;
                Msp430Adc12ImplP$resultBufferIndex = 0;
                Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$resultBuffer, Msp430Adc12ImplP$resultBufferLength);
              }
            }
        }
#line 584
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 588
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 590
          ++i < Msp430Adc12ImplP$resultBufferLength);

          Msp430Adc12ImplP$resultBuffer = Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, 
          Msp430Adc12ImplP$resultBuffer - Msp430Adc12ImplP$resultBufferLength, 
          Msp430Adc12ImplP$resultBufferLength);
          if (!Msp430Adc12ImplP$resultBuffer) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 597
          break;
        }
    }
}

# 274 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv)
#line 274
{
}

# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void HplAdc12P$HplAdc12$conversionDone(uint16_t arg_0x40cb7120){
#line 112
  Msp430RefVoltGeneratorP$HplAdc12$conversionDone(arg_0x40cb7120);
#line 112
  Msp430Adc12ImplP$HplAdc12$conversionDone(arg_0x40cb7120);
#line 112
}
#line 112
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 0);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 6);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 7);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc();
#line 85
}
#line 85
# 94 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$resetIFGs(void)
#line 94
{
  if (!HplAdc12P$ADC12IFG) {
    return;
    }
  else 
#line 97
    {

      uint8_t i;
      volatile uint16_t tmp;

#line 101
      for (i = 0; i < 16; i++) 
        tmp = HplAdc12P$HplAdc12$getMem(i);
    }
}

# 106 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$resetIFGs(void){
#line 106
  HplAdc12P$HplAdc12$resetIFGs();
#line 106
}
#line 106
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$readDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$readDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 334 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 334
{
}

# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
inline static   void AdcP$ReadNow$readDone(uint8_t arg_0x40c0e9a0, error_t arg_0x40bfdd98, AdcP$ReadNow$val_t arg_0x40bfdf20){
#line 65
    AdcP$ReadNow$default$readDone(arg_0x40c0e9a0, arg_0x40bfdd98, arg_0x40bfdf20);
#line 65
}
#line 65
# 318 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline   error_t AdcP$SingleChannelReadStream$singleDataReady(uint8_t streamClient, uint16_t data)
{

  return SUCCESS;
}

#line 186
static inline   uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t length)
{

  return 0;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$signalBufferDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$signalBufferDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 155 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$userId(void)
#line 155
{
  /* atomic removed: atomic calls only */
#line 156
  {
    unsigned char __nesc_temp = 
#line 156
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId;

#line 156
    return __nesc_temp;
  }
}

# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static   uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(void){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 349 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline   void HplMsp430Usart0P$Usart$disableRxIntr(void)
#line 349
{
  HplMsp430Usart0P$IE1 &= ~(1 << 6);
}

# 177 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableRxIntr(void){
#line 177
  HplMsp430Usart0P$Usart$disableRxIntr();
#line 177
}
#line 177
# 172 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$rxDone(uint8_t data)
#line 172
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$continueOp();
    }
  else 
#line 179
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone();
    }
}

# 65 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(uint8_t arg_0x41197908, uint8_t arg_0x410a88e8){
#line 54
  switch (arg_0x41197908) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$rxDone(arg_0x410a88e8);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(arg_0x41197908, arg_0x410a88e8);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 146 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static inline   bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$inUse(void)
#line 146
{
  return TRUE;
}

# 80 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static   bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse(void){
#line 80
  unsigned char result;
#line 80

#line 80
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ArbiterInfo$inUse();
#line 80

#line 80
  return result;
#line 80
}
#line 80
# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(), data);
    }
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static   void HplMsp430Usart0P$Interrupts$rxDone(uint8_t arg_0x410a88e8){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(arg_0x410a88e8);
#line 54
}
#line 54
# 55 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline   bool HplMsp430I2C0P$HplI2C$isI2C(void)
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P$U0CTL & 0x20 && HplMsp430I2C0P$U0CTL & 0x04 && HplMsp430I2C0P$U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static   bool HplMsp430Usart0P$HplI2C$isI2C(void){
#line 6
  unsigned char result;
#line 6

#line 6
  result = HplMsp430I2C0P$HplI2C$isI2C();
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 66 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$default$fired(uint8_t id)
#line 66
{
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$fired(uint8_t arg_0x41196010){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$default$fired(arg_0x41196010);
#line 39
}
#line 39
# 59 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawI2CInterrupts$fired(void)
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$I2CInterrupts$fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId());
    }
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static   void HplMsp430Usart0P$I2CInterrupts$fired(void){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawI2CInterrupts$fired();
#line 39
}
#line 39
# 190 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline   void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$txDone(void)
#line 190
{
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline    void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(uint8_t id)
#line 64
{
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(uint8_t arg_0x41197908){
#line 49
  switch (arg_0x41197908) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C$0$CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$UsartInterrupts$txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(arg_0x41197908);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline   void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void)
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId());
    }
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static   void HplMsp430Usart0P$Interrupts$txDone(void){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone();
#line 49
}
#line 49
# 210 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
 __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 212
    uint16_t __x;

#line 212
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 212
   & 0x0008) != 0;

#line 213
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

 void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(12))) void sig_TIMERA0_VECTOR(void)
#line 11
{
#line 11
  Msp430TimerCommonP$VectorTimerA0$fired();
}

# 169 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired();
    }
}

#line 169
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired();
    }
}

#line 169
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired();
    }
}

# 12 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(10))) void sig_TIMERA1_VECTOR(void)
#line 12
{
#line 12
  Msp430TimerCommonP$VectorTimerA1$fired();
}

#line 13
 __attribute((wakeup)) __attribute((interrupt(26))) void sig_TIMERB0_VECTOR(void)
#line 13
{
#line 13
  Msp430TimerCommonP$VectorTimerB0$fired();
}

# 135 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static    void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(uint8_t arg_0x40875be0){
#line 28
  switch (arg_0x40875be0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(arg_0x40875be0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static   error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void)
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )remaining << 5);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static   /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void)
{
  /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void)
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t *)400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t *)400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t *)400U;
    }
}

# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$enableCapture(uint8_t mode)
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$GeneralIO$selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430Capture$clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC$0$Msp430TimerControl$enableEvents();
  }
  return SUCCESS;
}

# 147 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   error_t CC2420SpiP$Resource$release(uint8_t id)
#line 147
{
  uint8_t i;

#line 149
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
    {
      if (CC2420SpiP$m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 151
            FAIL;

            {
#line 151
              __nesc_atomic_end(__nesc_atomic); 
#line 151
              return __nesc_temp;
            }
          }
        }
#line 154
      CC2420SpiP$m_holder = CC2420SpiP$NO_HOLDER;
      if (!CC2420SpiP$m_requests) {
          CC2420SpiP$WorkingState$toIdle();
          CC2420SpiP$attemptRelease();
        }
      else {
          for (i = CC2420SpiP$m_holder + 1; ; i++) {
              i %= CC2420SpiP$RESOURCE_COUNT;

              if (CC2420SpiP$m_requests & (1 << i)) {
                  CC2420SpiP$m_holder = i;
                  CC2420SpiP$m_requests &= ~(1 << i);
                  CC2420SpiP$grant$postTask();
                  {
                    unsigned char __nesc_temp = 
#line 167
                    SUCCESS;

                    {
#line 167
                      __nesc_atomic_end(__nesc_atomic); 
#line 167
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 173
    __nesc_atomic_end(__nesc_atomic); }
#line 173
  return SUCCESS;
}

#line 335
static error_t CC2420SpiP$attemptRelease(void)
#line 335
{


  if ((
#line 336
  CC2420SpiP$m_requests > 0
   || CC2420SpiP$m_holder != CC2420SpiP$NO_HOLDER)
   || !CC2420SpiP$WorkingState$isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP$release = TRUE;
  CC2420SpiP$ChipSpiResource$releasing();
  /* atomic removed: atomic calls only */
#line 344
  {
    if (CC2420SpiP$release) {
        CC2420SpiP$SpiResource$release();
        {
          unsigned char __nesc_temp = 
#line 347
          SUCCESS;

#line 347
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 133 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static   bool StateImplP$State$isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP$state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static   void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Alarm$startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$set_alarm(void)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$Counter$get();
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type expires;
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$AlarmFrom$startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC$1$from_size_type )remaining << 0);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static   /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type /*Counter32khz32C.Transform*/TransformCounterC$1$Counter$get(void)
{
  /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC$1$upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC$1$m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC$1$from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC$1$CounterFrom$get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC$1$to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC$1$LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC$1$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 683 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$signalDone(error_t err)
#line 683
{
  /* atomic removed: atomic calls only */
#line 684
  CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
  CC2420TransmitP$abortSpiRelease = FALSE;
  CC2420TransmitP$ChipSpiResource$attemptRelease();
  CC2420TransmitP$Send$sendDone(CC2420TransmitP$m_msg, err);
}

#line 624
static void CC2420TransmitP$congestionBackoff(void)
#line 624
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 625
    {
      CC2420TransmitP$RadioBackoff$requestCongestionBackoff(CC2420TransmitP$m_msg);
      CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$myCongestionBackoff);
    }
#line 628
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.x/tos/system/RandomMlcgP.nc"
static   uint32_t RandomMlcgP$Random$rand32(void)
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

  /* atomic removed: atomic calls only */
#line 62
  {
    tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgP$seed;
    q = tmpseed;
    q = q >> 1;
    p = tmpseed >> 32;
    mlcg = p + q;
    if (mlcg & 0x80000000) {
        mlcg = mlcg & 0x7FFFFFFF;
        mlcg++;
      }
    RandomMlcgP$seed = mlcg;
  }
  return mlcg;
}

# 631 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP$acquireSpiResource(void)
#line 631
{
  error_t error = CC2420TransmitP$SpiResource$immediateRequest();

#line 633
  if (error != SUCCESS) {
      CC2420TransmitP$SpiResource$request();
    }
  return error;
}

# 124 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   error_t CC2420SpiP$Resource$immediateRequest(uint8_t id)
#line 124
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (CC2420SpiP$WorkingState$requestState(CC2420SpiP$S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 129
            EBUSY;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP$SpiResource$isOwner()) {
          CC2420SpiP$m_holder = id;
          error = SUCCESS;
        }
      else {
#line 137
        if ((error = CC2420SpiP$SpiResource$immediateRequest()) == SUCCESS) {
            CC2420SpiP$m_holder = id;
          }
        else {
            CC2420SpiP$WorkingState$toIdle();
          }
        }
    }
#line 144
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return error;
}

# 96 "/opt/tinyos-2.x/tos/system/StateImplP.nc"
static   error_t StateImplP$State$requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP$S_IDLE || StateImplP$state[id] == StateImplP$S_IDLE) {
          StateImplP$state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 162 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static   uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$Resource$isOwner(uint8_t id)
#line 162
{
  /* atomic removed: atomic calls only */
#line 163
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId == id) {
        unsigned char __nesc_temp = 
#line 164
        TRUE;

#line 164
        return __nesc_temp;
      }
    else 
#line 165
      {
        unsigned char __nesc_temp = 
#line 165
        FALSE;

#line 165
        return __nesc_temp;
      }
  }
}

#line 126
static   error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$ResourceDefaultOwner$release(void)
#line 126
{
  /* atomic removed: atomic calls only */
#line 127
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$postTask();
            {
              unsigned char __nesc_temp = 
#line 131
              SUCCESS;

#line 131
              return __nesc_temp;
            }
          }
        else {
#line 133
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 136
                SUCCESS;

#line 136
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 140
  return FAIL;
}

# 265 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static   void HplMsp430Usart0P$Usart$setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P$Usart$resetUsart(TRUE);
      HplMsp430Usart0P$HplI2C$clearModeI2C();
      HplMsp430Usart0P$Usart$disableUart();
      HplMsp430Usart0P$configSpi(config);
      HplMsp430Usart0P$Usart$enableSpi();
      HplMsp430Usart0P$Usart$resetUsart(FALSE);
      HplMsp430Usart0P$Usart$clrIntr();
      HplMsp430Usart0P$Usart$disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 105 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   error_t CC2420SpiP$Resource$request(uint8_t id)
#line 105
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    {
      if (CC2420SpiP$WorkingState$requestState(CC2420SpiP$S_BUSY) == SUCCESS) {
          CC2420SpiP$m_holder = id;
          if (CC2420SpiP$SpiResource$isOwner()) {
              CC2420SpiP$grant$postTask();
            }
          else {
              CC2420SpiP$SpiResource$request();
            }
        }
      else {
          CC2420SpiP$m_requests |= 1 << id;
        }
    }
#line 120
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 583 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$attemptSend(void)
#line 583
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 587
    {
      if (CC2420TransmitP$m_state == CC2420TransmitP$S_TX_CANCEL) {
          CC2420TransmitP$SFLUSHTX$strobe();
          CC2420TransmitP$releaseSpiResource();
          CC2420TransmitP$CSN$set();
          CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
          {
#line 593
            __nesc_atomic_end(__nesc_atomic); 
#line 593
            return;
          }
        }

      CC2420TransmitP$CSN$clr();

      status = CC2420TransmitP$m_cca ? CC2420TransmitP$STXONCCA$strobe() : CC2420TransmitP$STXON$strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP$SNOP$strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP$m_state = congestion ? CC2420TransmitP$S_SAMPLE_CCA : CC2420TransmitP$S_SFD;
      CC2420TransmitP$CSN$set();
    }
#line 609
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP$totalCcaChecks = 0;
      CC2420TransmitP$releaseSpiResource();
      CC2420TransmitP$congestionBackoff();
    }
  else 
#line 615
    {
      CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$CC2420_ABORT_PERIOD);
    }
}

# 314 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   cc2420_status_t CC2420SpiP$Strobe$strobe(uint8_t addr)
#line 314
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 315
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 317
            0;

            {
#line 317
              __nesc_atomic_end(__nesc_atomic); 
#line 317
              return __nesc_temp;
            }
          }
        }
    }
#line 321
    __nesc_atomic_end(__nesc_atomic); }
#line 321
  return CC2420SpiP$SpiByte$write(addr);
}

# 98 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static   uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiByte$write(uint8_t tx)
#line 98
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$rx();

  return byte;
}

# 386 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static   uint8_t HplMsp430Usart0P$Usart$rx(void)
#line 386
{
  uint8_t value;

#line 388
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    value = U0RXBUF;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)29U |= 0x01 << 2;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static   void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP$26$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)29U &= ~(0x01 << 2);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)29U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static   void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP$30$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)29U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 14 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(24))) void sig_TIMERB1_VECTOR(void)
#line 14
{
#line 14
  Msp430TimerCommonP$VectorTimerB1$fired();
}

# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void)
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 160 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.x/tos/platforms/UBee430/MotePlatformC.nc"
static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 123 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static  bool SchedulerBasicP$Scheduler$runNextTask(void)
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 164
static   void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x405d4868){
#line 64
  switch (arg_0x405d4868) {
#line 64
    case UBee430_APC$Send:
#line 64
      UBee430_APC$Send$runTask();
#line 64
      break;
#line 64
    case UBee430_APC$rt_Send:
#line 64
      UBee430_APC$rt_Send$runTask();
#line 64
      break;
#line 64
    case UBee430_APC$next:
#line 64
      UBee430_APC$next$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readSensor$runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$runTask();
#line 64
      break;
#line 64
    case HplSensirionSht11P$stopTask:
#line 64
      HplSensirionSht11P$stopTask$runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask:
#line 64
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$startTask$runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP$0$stopTask$runTask();
#line 64
      break;
#line 64
    case AdcP$finishStreamRequest:
#line 64
      AdcP$finishStreamRequest$runTask();
#line 64
      break;
#line 64
    case AdcP$signalBufferDone:
#line 64
      AdcP$signalBufferDone$runTask();
#line 64
      break;
#line 64
    case AdcP$readDone:
#line 64
      AdcP$readDone$runTask();
#line 64
      break;
#line 64
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask:
#line 64
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask();
#line 64
      break;
#line 64
    case Msp430RefVoltArbiterImplP$switchOff:
#line 64
      Msp430RefVoltArbiterImplP$switchOff$runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP$startDone_task:
#line 64
      CC2420CsmaP$startDone_task$runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP$stopDone_task:
#line 64
      CC2420CsmaP$stopDone_task$runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP$sendDone_task:
#line 64
      CC2420CsmaP$sendDone_task$runTask();
#line 64
      break;
#line 64
    case CC2420ControlP$sync:
#line 64
      CC2420ControlP$sync$runTask();
#line 64
      break;
#line 64
    case CC2420ControlP$syncDone:
#line 64
      CC2420ControlP$syncDone$runTask();
#line 64
      break;
#line 64
    case CC2420SpiP$grant:
#line 64
      CC2420SpiP$grant$runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task:
#line 64
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP$1$grantedTask$runTask();
#line 64
      break;
#line 64
    case CC2420ReceiveP$receiveDone_task:
#line 64
      CC2420ReceiveP$receiveDone_task$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x405d4868);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 95 "/opt/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static   am_addr_t ActiveMessageAddressC$amAddress(void)
#line 95
{
  am_addr_t myAddr;

#line 97
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    myAddr = ActiveMessageAddressC$addr;
#line 97
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 132 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static  am_id_t CC2420ActiveMessageP$AMPacket$type(message_t *amsg)
#line 132
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 134
  return __nesc_ntoh_leuint8((unsigned char *)&header->type);
}

# 400 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP$waitForNextPacket(void)
#line 400
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 401
    {
      if (CC2420ReceiveP$m_state == CC2420ReceiveP$S_STOPPED) {
          CC2420ReceiveP$SpiResource$release();
          {
#line 404
            __nesc_atomic_end(__nesc_atomic); 
#line 404
            return;
          }
        }
      CC2420ReceiveP$receivingPacket = FALSE;
      if ((CC2420ReceiveP$m_missed_packets && CC2420ReceiveP$FIFO$get()) || !CC2420ReceiveP$FIFOP$get()) {

          if (CC2420ReceiveP$m_missed_packets) {
              CC2420ReceiveP$m_missed_packets--;
            }

          CC2420ReceiveP$beginReceive();
        }
      else {

          CC2420ReceiveP$m_state = CC2420ReceiveP$S_STARTED;
          CC2420ReceiveP$m_missed_packets = 0;
          CC2420ReceiveP$SpiResource$release();
        }
    }
#line 422
    __nesc_atomic_end(__nesc_atomic); }
}

#line 355
static void CC2420ReceiveP$beginReceive(void)
#line 355
{
  CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_LENGTH;
  /* atomic removed: atomic calls only */
  CC2420ReceiveP$receivingPacket = TRUE;
  if (CC2420ReceiveP$SpiResource$isOwner()) {
      CC2420ReceiveP$receive();
    }
  else {
#line 362
    if (CC2420ReceiveP$SpiResource$immediateRequest() == SUCCESS) {
        CC2420ReceiveP$receive();
      }
    else {
        CC2420ReceiveP$SpiResource$request();
      }
    }
}

#line 390
static void CC2420ReceiveP$receive(void)
#line 390
{
  CC2420ReceiveP$CSN$clr();
  CC2420ReceiveP$RXFIFO$beginRead((uint8_t *)CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf), 1);
}

# 187 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   cc2420_status_t CC2420SpiP$Fifo$beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 188
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 192
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 194
            status;

            {
#line 194
              __nesc_atomic_end(__nesc_atomic); 
#line 194
              return __nesc_temp;
            }
          }
        }
    }
#line 198
    __nesc_atomic_end(__nesc_atomic); }
#line 198
  CC2420SpiP$m_addr = addr | 0x40;

  status = CC2420SpiP$SpiByte$write(CC2420SpiP$m_addr);
  CC2420SpiP$Fifo$continueRead(addr, data, len);

  return status;
}

# 146 "/opt/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static   error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SpiPacket$send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 148
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$signalDone_task$postTask();
    }

  return SUCCESS;
}

#line 120
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$continueOp(void)
#line 120
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 125
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isTxIntrPending()) ;
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrTxIntr();
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos] : 0);
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$isRxIntrPending()) ;
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$clrRxIntr();
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$Usart$rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP$0$m_pos - 1] = tmp;
            }
        }
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 325 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   void CC2420SpiP$SpiPacket$sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 326
{
  if (CC2420SpiP$m_addr & 0x40) {
      CC2420SpiP$Fifo$readDone(CC2420SpiP$m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 329
    {
      CC2420SpiP$Fifo$writeDone(CC2420SpiP$m_addr, tx_buf, len, error);
    }
}

# 373 "/opt/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP$flush(void)
#line 373
{
  CC2420ReceiveP$reset_state();
  CC2420ReceiveP$CSN$set();
  CC2420ReceiveP$CSN$clr();
  CC2420ReceiveP$SFLUSHRX$strobe();
  CC2420ReceiveP$SFLUSHRX$strobe();
  CC2420ReceiveP$CSN$set();
  CC2420ReceiveP$SpiResource$release();
  CC2420ReceiveP$waitForNextPacket();
}

#line 428
static void CC2420ReceiveP$reset_state(void)
#line 428
{
  CC2420ReceiveP$m_bytes_left = CC2420ReceiveP$RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 430
  CC2420ReceiveP$receivingPacket = FALSE;
  CC2420ReceiveP$m_timestamp_head = 0;
  CC2420ReceiveP$m_timestamp_size = 0;
  CC2420ReceiveP$m_missed_packets = 0;
}

# 661 "/opt/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$loadTXFIFO(void)
#line 661
{
  cc2420_header_t *header = CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8((unsigned char *)&CC2420TransmitP$CC2420PacketBody$getMetadata(CC2420TransmitP$m_msg)->tx_power);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP$CSN$clr();

  if (CC2420TransmitP$m_tx_power != tx_power) {
      CC2420TransmitP$TXCTRL$write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP$m_tx_power = tx_power;

  CC2420TransmitP$TXFIFO$write((uint8_t *)header, __nesc_ntoh_leuint8((unsigned char *)&header->length) - 1);
}

# 301 "/opt/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static   cc2420_status_t CC2420SpiP$Reg$write(uint8_t addr, uint16_t data)
#line 301
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 302
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 304
            0;

            {
#line 304
              __nesc_atomic_end(__nesc_atomic); 
#line 304
              return __nesc_temp;
            }
          }
        }
    }
#line 308
    __nesc_atomic_end(__nesc_atomic); }
#line 307
  CC2420SpiP$SpiByte$write(addr);
  CC2420SpiP$SpiByte$write(data >> 8);
  return CC2420SpiP$SpiByte$write(data & 0xff);
}

# 428 "/opt/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP$writeFsctrl(void)
#line 428
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 431
    {
      channel = CC2420ControlP$m_channel;
    }
#line 433
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP$FSCTRL$write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}




static void CC2420ControlP$writeMdmctrl0(void)
#line 442
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 443
    {
      CC2420ControlP$MDMCTRL0$write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | (
      CC2420ControlP$addressRecognition << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP$autoAckEnabled && CC2420ControlP$hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 452
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP$writeId(void)
#line 461
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 464
    {
      __nesc_hton_leuint16((unsigned char *)&id[0], CC2420ControlP$m_pan);
      __nesc_hton_leuint16((unsigned char *)&id[1], CC2420ControlP$m_short_addr);
    }
#line 467
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP$PANID$write(0, (uint8_t *)&id, sizeof id);
}

# 56 "/opt/tinyos-2.x/tos/interfaces/State.nc"
static   void UniqueSendP$State$toIdle(void){
#line 56
  StateImplP$State$toIdle(2U);
#line 56
}
#line 56
# 79 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static void Ds28dg02P$SPI_INSTRUCTION_SET(uint8_t instruction)
#line 79
{
  uint8_t clk_count;
  uint8_t temp;

  Ds28dg02P$DS28DG02_SEL();
  Ds28dg02P$delay(10);
  temp = instruction;
  for (clk_count = 0; clk_count <= 7; clk_count++) {

      if (temp & 0x80) {
#line 88
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 89
        TOSH_CLR_DS28DG02_SI_PIN();
        }
#line 90
      Ds28dg02P$delay(4);
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }
  TOSH_SET_DS28DG02_SI_PIN();
  Ds28dg02P$delay(5);
  Ds28dg02P$DS28DG02_DSEL();
}

#line 70
static void Ds28dg02P$spi_clk(void)
#line 70
{
  TOSH_CLR_DS28DG02_SCK_PIN();
  TOSH_SET_DS28DG02_SCK_PIN();
  Ds28dg02P$delay(10);
  TOSH_CLR_DS28DG02_SCK_PIN();
}

#line 98
static void Ds28dg02P$SPI_WRITE(uint8_t w_instruction, uint8_t addr, uint8_t data)
#line 98
{
  uint8_t clk_count;
  uint8_t temp;

#line 101
  Ds28dg02P$DS28DG02_SEL();
  Ds28dg02P$delay(10);

  temp = w_instruction;
  for (clk_count = 0; clk_count <= 7; clk_count++) {
      if (temp & 0x80) {
#line 106
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 107
        TOSH_CLR_DS28DG02_SI_PIN();
        }
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }

  temp = addr;
  for (clk_count = 0; clk_count <= 7; clk_count++) {
      if (temp & 0x80) {
#line 115
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 116
        TOSH_CLR_DS28DG02_SI_PIN();
        }
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }

  temp = data;
  for (clk_count = 0; clk_count <= 7; clk_count++) {
      if (temp & 0x80) {
#line 124
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 125
        TOSH_CLR_DS28DG02_SI_PIN();
        }
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }
  TOSH_SET_DS28DG02_SI_PIN();
  Ds28dg02P$delay(5);
  Ds28dg02P$DS28DG02_DSEL();
}

# 83 "/opt/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static  error_t CC2420CsmaP$SplitControl$start(void)
#line 83
{
  if (CC2420CsmaP$SplitControlState$requestState(CC2420CsmaP$S_STARTING) == SUCCESS) {
      CC2420CsmaP$CC2420Power$startVReg();
      return SUCCESS;
    }
  else {
#line 88
    if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTED)) {
        return EALREADY;
      }
    else {
#line 91
      if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 95
  return EBUSY;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP$switchOff(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    {
      if (Msp430RefVoltGeneratorP$HplAdc12$isBusy()) 
        {
          unsigned char __nesc_temp = 
#line 82
          FAIL;

          {
#line 82
            __nesc_atomic_end(__nesc_atomic); 
#line 82
            return __nesc_temp;
          }
        }
      else 
#line 83
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP$HplAdc12$getCtl0();

#line 85
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 89
            SUCCESS;

            {
#line 89
              __nesc_atomic_end(__nesc_atomic); 
#line 89
              return __nesc_temp;
            }
          }
        }
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }
}

# 132 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

#line 135
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 70 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP$Config$getConfiguration(client);

#line 73
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

#line 76
      if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {



          Msp430RefVoltArbiterImplP$AdcResource$release(client);
          Msp430RefVoltArbiterImplP$AdcResource$request(client);
          return;
        }
      Msp430RefVoltArbiterImplP$syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
        started = Msp430RefVoltArbiterImplP$RefVolt_1_5V$start();
        }
      else {
#line 88
        started = Msp430RefVoltArbiterImplP$RefVolt_2_5V$start();
        }
#line 89
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;
          Msp430RefVoltArbiterImplP$AdcResource$release(client);
          Msp430RefVoltArbiterImplP$AdcResource$request(client);
        }
    }
  else {
#line 95
    Msp430RefVoltArbiterImplP$ClientResource$granted(client);
    }
}

#line 170
static    error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$release(uint8_t arg_0x40d76720){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40d76720) {
#line 110
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 110
      break;
#line 110
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = Msp430RefVoltArbiterImplP$AdcResource$default$release(arg_0x40d76720);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 97 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 65 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id)
#line 65
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] & (1 << id % 8);
}

# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static    error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client)
{
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$request(uint8_t arg_0x40d76720){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x40d76720) {
#line 78
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 78
      break;
#line 78
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = Msp430RefVoltArbiterImplP$AdcResource$default$request(arg_0x40d76720);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 71 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id)
#line 71
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 160 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static   void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client)
#line 160
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$ClientResource$granted(uint8_t arg_0x40d77d50){
#line 92
  switch (arg_0x40d77d50) {
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      AdcP$ResourceRead$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      AdcP$ResourceReadStream$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 92
      AdcP$SubResourceReadNow$granted(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 92
      AdcP$ResourceRead$granted(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 92
      AdcP$ResourceReadStream$granted(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 92
      AdcP$ResourceRead$granted(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT);
#line 92
      break;
#line 92
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 92
      AdcP$ResourceRead$granted(/*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT);
#line 92
      break;
#line 92
    default:
#line 92
      Msp430RefVoltArbiterImplP$ClientResource$default$granted(arg_0x40d77d50);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 104 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static  void AdcP$ResourceRead$granted(uint8_t client)
{

  error_t result = AdcP$configure(client);

#line 108
  if (result == SUCCESS) {
      AdcP$state = AdcP$STATE_READ;
      result = AdcP$SingleChannel$getData(client);
    }
  if (result != SUCCESS) {
      AdcP$ResourceRead$release(client);
      AdcP$Read$readDone(client, result, 0);
    }
}

#line 87
static error_t AdcP$configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t *config;

#line 91
  config = AdcP$Config$getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP$SingleChannel$configureSingle(client, config);
    }
#line 94
  return result;
}

# 166 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 174
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 176
          EBUSY;

          {
#line 176
            __nesc_atomic_end(__nesc_atomic); 
#line 176
            return __nesc_temp;
          }
        }
#line 177
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();

#line 194
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP$state = Msp430Adc12ImplP$SINGLE_DATA;
          Msp430Adc12ImplP$HplAdc12$setCtl0(ctl0);
          Msp430Adc12ImplP$HplAdc12$setCtl1(ctl1);
          Msp430Adc12ImplP$HplAdc12$setMCtl(0, memctl);
          Msp430Adc12ImplP$HplAdc12$setIEFlags(0x01);
          result = SUCCESS;
        }
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

#line 370
static   error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 372
    {
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP$resultBuffer) 
            {
              unsigned char __nesc_temp = 
#line 375
              EINVAL;

              {
#line 375
                __nesc_atomic_end(__nesc_atomic); 
#line 375
                return __nesc_temp;
              }
            }
#line 376
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 377
              EBUSY;

              {
#line 377
                __nesc_atomic_end(__nesc_atomic); 
#line 377
                return __nesc_temp;
              }
            }
#line 378
          Msp430Adc12ImplP$state |= Msp430Adc12ImplP$ADC_BUSY;
          Msp430Adc12ImplP$clientID = id;
          Msp430Adc12ImplP$configureAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
          Msp430Adc12ImplP$HplAdc12$startConversion();
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
            Msp430Adc12ImplP$startTimerA();
            }
#line 384
          {
            unsigned char __nesc_temp = 
#line 384
            SUCCESS;

            {
#line 384
              __nesc_atomic_end(__nesc_atomic); 
#line 384
              return __nesc_temp;
            }
          }
        }
    }
#line 388
    __nesc_atomic_end(__nesc_atomic); }
#line 387
  return FAIL;
}

# 79 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i)
#line 79
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 82
  memCtlPtr += i;
  x = * (adc12memctl_t *)memCtlPtr;
  return x;
}

# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 116 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client)
{
  error_t error;

#line 119
  if (Msp430RefVoltArbiterImplP$syncOwner == client) {
    Msp430RefVoltArbiterImplP$switchOff$postTask();
    }
#line 121
  error = Msp430RefVoltArbiterImplP$AdcResource$release(client);
#line 133
  return error;
}

# 328 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static   void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 328
{
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void AdcP$Read$readDone(uint8_t arg_0x40c0fb48, error_t arg_0x40624580, AdcP$Read$val_t arg_0x40624708){
#line 63
  switch (arg_0x40c0fb48) {
#line 63
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 63
      UBee430_APC$InternalVolt$readDone(arg_0x40624580, arg_0x40624708);
#line 63
      break;
#line 63
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT:
#line 63
      UBee430_APC$InternalTemp$readDone(arg_0x40624580, arg_0x40624708);
#line 63
      break;
#line 63
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT:
#line 63
      UBee430_APC$Photo$readDone(arg_0x40624580, arg_0x40624708);
#line 63
      break;
#line 63
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT:
#line 63
      UBee430_APC$AdcZero$readDone(arg_0x40624580, arg_0x40624708);
#line 63
      break;
#line 63
    default:
#line 63
      AdcP$Read$default$readDone(arg_0x40c0fb48, arg_0x40624580, arg_0x40624708);
#line 63
      break;
#line 63
    }
#line 63
}
#line 63
# 241 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static  void AdcP$ResourceReadStream$granted(uint8_t streamClient)
{
  error_t result;
  const msp430adc12_channel_config_t *config;
  struct AdcP$stream_entry_t *entry = AdcP$streamBuf[streamClient];

  if (!entry) {
    result = EINVAL;
    }
  else 
#line 249
    {
      config = AdcP$ConfigReadStream$getConfiguration(streamClient);
      if (config->inch == INPUT_CHANNEL_NONE) {
        result = EINVAL;
        }
      else 
#line 253
        {
          AdcP$owner = streamClient;
          AdcP$streamConfig = *config;
          AdcP$streamConfig.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
          AdcP$streamConfig.sampcon_id = SAMPCON_CLOCK_DIV_1;
          AdcP$streamBuf[streamClient] = entry->next;
          result = AdcP$SingleChannelReadStream$configureMultiple(streamClient, 
          &AdcP$streamConfig, (uint16_t *)entry, entry->count, AdcP$usPeriod[streamClient]);
          if (result == SUCCESS) {
            result = AdcP$SingleChannelReadStream$getData(streamClient);
            }
          else 
#line 263
            {
              AdcP$streamBuf[streamClient] = entry;
              AdcP$finishStreamRequest$postTask();
              return;
            }
        }
    }
  if (result != SUCCESS) {
      AdcP$ResourceReadStream$release(streamClient);
      AdcP$ReadStream$readDone(streamClient, FAIL, 0);
    }
  return;
}

# 257 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;

  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 266
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 268
          EBUSY;

          {
#line 268
            __nesc_atomic_end(__nesc_atomic); 
#line 268
            return __nesc_temp;
          }
        }
#line 269
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
#line 285
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();

#line 287
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP$state = Msp430Adc12ImplP$MULTIPLE_DATA;
          Msp430Adc12ImplP$resultBuffer = buf;
          Msp430Adc12ImplP$resultBufferLength = length;
          Msp430Adc12ImplP$resultBufferIndex = 0;
          Msp430Adc12ImplP$HplAdc12$setCtl0(ctl0);
          Msp430Adc12ImplP$HplAdc12$setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP$HplAdc12$setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP$HplAdc12$setMCtl(i, memctl);
          Msp430Adc12ImplP$HplAdc12$setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP$state |= Msp430Adc12ImplP$USE_TIMERA;
              Msp430Adc12ImplP$prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
#line 309
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 58 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP$switchOn(uint8_t level)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 60
    {
      if (Msp430RefVoltGeneratorP$HplAdc12$isBusy()) 
        {
          unsigned char __nesc_temp = 
#line 62
          FAIL;

          {
#line 62
            __nesc_atomic_end(__nesc_atomic); 
#line 62
            return __nesc_temp;
          }
        }
      else 
#line 63
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP$HplAdc12$getCtl0();

#line 65
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          ctl0.refon = 1;
          if (level == Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) {
            ctl0.r2_5v = 0;
            }
          else {
#line 71
            ctl0.r2_5v = 1;
            }
#line 72
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 73
            SUCCESS;

            {
#line 73
              __nesc_atomic_end(__nesc_atomic); 
#line 73
              return __nesc_temp;
            }
          }
        }
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t arg_0x40686600){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(3U, arg_0x40686600);
#line 62
}
#line 62
# 52 "/opt/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static  error_t HplSensirionSht11P$SplitControl$start(void)
#line 52
{
  HplSensirionSht11P$PWR$makeOutput();
  HplSensirionSht11P$PWR$set();
  HplSensirionSht11P$Timer$startOneShot(11);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)33U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}


static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeInput(void)
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t *)34U &= ~(0x01 << 5);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$clr(void)
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t *)33U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 149 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$performCommand(void)
#line 149
{

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$initPins();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$resetDevice();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$transmissionStart();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd &= 0x1F;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$sendCommand(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd);

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$waitForResponse() != SUCCESS) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$busy = FALSE;
      return FAIL;
    }

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_SOFT_RESET: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$TIMEOUT_RESET);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$TIMEOUT_12BIT);
        }
      else 
#line 173
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$TIMEOUT_14BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$TIMEOUT_8BIT);
        }
      else 
#line 184
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$TIMEOUT_12BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_READ_STATUS: 
        {
          uint8_t tempStatus;
          uint8_t crc;

          tempStatus = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte();
          crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte();
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$endTransmission();

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status = tempStatus;

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$postTask();
        }

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeByte(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$newStatus);

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$waitForResponse() != SUCCESS) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeFail = TRUE;
        }
      else 
#line 209
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$status = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$newStatus;
        }

      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$signalStatusDone$postTask();
    }


  return SUCCESS;
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)33U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static   error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$Interrupt$disable(void)
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC$0$HplInterrupt$clear();
    }
#line 62
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static   void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP$5$IO$makeOutput(void)
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t *)34U |= 0x01 << 5;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static   void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP$6$IO$set(void)
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t *)33U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 255 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$writeByte(uint8_t byte)
#line 255
{
  uint8_t i;

#line 257
  for (i = 0; i < 8; i++) {
      if (byte & 0x80) {
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
        }
      else {
#line 261
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$clr();
        }
#line 262
      byte = byte << 1;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
    }
}

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$waitForResponse(void)
#line 268
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$get()) {


      return FAIL;
    }
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
  return SUCCESS;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$Timer$startOneShot(uint32_t arg_0x40686600){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(2U, arg_0x40686600);
#line 62
}
#line 62
# 281 "/opt/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$enableInterrupt(void)
#line 281
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$InterruptDATA$enableFallingEdge();
}

#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$readByte(void)
#line 355
{
  uint8_t byte = 0;
  uint8_t i;

  for (i = 0; i < 8; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$get()) {
        byte |= 1;
        }
#line 363
      if (i != 7) {
        byte = byte << 1;
        }
#line 365
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$ack();
  return byte;
}










static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$endTransmission(void)
#line 381
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$DATA$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP$0$CLOCK$clr();
}

# 107 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$release(uint8_t id)
#line 107
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_BUSY && /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId == id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$isEmpty() == FALSE) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$reqResId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$dequeue();
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_GRANTING;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$grantedTask$postTask();
            }
          else {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$default_owner_id;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_CONTROLLED;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted();
            }
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(id);
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 153 "UBee430_APC.nc"
static  void UBee430_APC$Humidity$readDone(error_t result, uint16_t data)
#line 153
{
  uint8_t sig;
  uint16_t t;
#line 155
  uint16_t sorh;
#line 155
  uint16_t i;

#line 156
  if (result == SUCCESS) {
      sig = (uint8_t )UBee430_APC$HumidityMetadata$getSignificantBits();
      t = (1 << sig) - 1;
      sorh = (u_int16_t )(t & data);
      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.humid, (uint8_t )(-4 + 0.0405 * sorh - 2.8 * 0.000001 * sorh * sorh));
    }
  else {
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.humid, 0xFF);
    }
#line 164
  UBee430_APC$count++;
}

static  void UBee430_APC$Temperature$readDone(error_t result, uint16_t data)
#line 167
{
  uint8_t sig;
  uint16_t t;

#line 170
  if (result == SUCCESS) {
      sig = (uint8_t )UBee430_APC$TemperatureMetadata$getSignificantBits();
      t = (1 << sig) - 1;
      __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.temp, (uint8_t )(-39.55 + 0.01 * (t & data)));
    }
  else {
    __nesc_hton_uint8((unsigned char *)&UBee430_APC$message.temp, 0xFF);
    }
#line 177
  UBee430_APC$count++;
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(num);
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 97 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static  error_t AdcP$Read$read(uint8_t client)
{
  if (AdcP$ResourceRead$isOwner(client)) {
    return EBUSY;
    }
#line 101
  return AdcP$ResourceRead$request(client);
}

# 169 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static    bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(uint8_t client)
#line 169
{
#line 169
  return FALSE;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$AdcResource$isOwner(uint8_t arg_0x40d76720){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x40d76720) {
#line 118
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID);
#line 118
      break;
#line 118
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(arg_0x40d76720);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 143 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id)
#line 143
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 144
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId == id) {
          unsigned char __nesc_temp = 
#line 145
          TRUE;

          {
#line 145
            __nesc_atomic_end(__nesc_atomic); 
#line 145
            return __nesc_temp;
          }
        }
      else 
#line 146
        {
          unsigned char __nesc_temp = 
#line 146
          FALSE;

          {
#line 146
            __nesc_atomic_end(__nesc_atomic); 
#line 146
            return __nesc_temp;
          }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
}

# 76 "/opt/tinyos-2.x/tos/system/ArbiterP.nc"
static   error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Resource$request(uint8_t id)
#line 76
{
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceRequested$requested(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 78
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_CONTROLLED) {
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$RES_GRANTING;
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 83
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$Queue$enqueue(id);

          {
#line 83
            __nesc_atomic_end(__nesc_atomic); 
#line 83
            return __nesc_temp;
          }
        }
    }
#line 86
    __nesc_atomic_end(__nesc_atomic); }
#line 85
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP$0$ResourceDefaultOwner$requested();
  return SUCCESS;
}

# 172 "/opt/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static  void *CC2420ActiveMessageP$Packet$getPayload(message_t *msg, uint8_t *len)
#line 172
{
  if (len != (void *)0) {
      *len = CC2420ActiveMessageP$Packet$payloadLength(msg);
    }
  return msg->data;
}

#line 61
static  error_t CC2420ActiveMessageP$AMSend$send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 63
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(msg);

#line 65
  __nesc_hton_leuint8((unsigned char *)&header->type, id);
  __nesc_hton_leuint16((unsigned char *)&header->dest, addr);
  __nesc_hton_leuint16((unsigned char *)&header->destpan, CC2420ActiveMessageP$CC2420Config$getPanAddr());

  return CC2420ActiveMessageP$SubSend$send(msg, len + CC2420ActiveMessageP$CC2420_SIZE);
}

# 135 "/opt/tinyos-2.x/tos/platforms/UBee430/chips/ds28dg02/Ds28dg02P.nc"
static uint8_t Ds28dg02P$SPI_READ(uint8_t r_instruction, uint8_t addr)
#line 135
{
  uint8_t clk_count;
  uint8_t temp;
  uint16_t in_data = 0;

  Ds28dg02P$DS28DG02_SEL();
  Ds28dg02P$delay(10);

  temp = r_instruction;
  for (clk_count = 0; clk_count <= 7; clk_count++) {
      if (temp & 0x80) {
#line 145
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 146
        TOSH_CLR_DS28DG02_SI_PIN();
        }
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }

  temp = addr;
  for (clk_count = 0; clk_count <= 7; clk_count++) {
      if (temp & 0x80) {
#line 154
        TOSH_SET_DS28DG02_SI_PIN();
        }
      else {
#line 155
        TOSH_CLR_DS28DG02_SI_PIN();
        }
      Ds28dg02P$spi_clk();
      temp = temp << 1;
    }
  for (clk_count = 0; clk_count <= 15; clk_count++) {



      TOSH_SET_DS28DG02_SCK_PIN();
      Ds28dg02P$delay(5);


      if (TOSH_READ_DS28DG02_SO_PIN()) {
          in_data = in_data + 0x0001;
          if (clk_count < 15) {
#line 170
            in_data = in_data << 1;
            }
        }
      else 
#line 172
        {
          if (clk_count < 15) {
#line 173
            in_data = in_data << 1;
            }
        }
#line 175
      TOSH_CLR_DS28DG02_SCK_PIN();
      Ds28dg02P$delay(5);
    }
  Ds28dg02P$delay(5);
  Ds28dg02P$DS28DG02_DSEL();
  return (uint8_t )in_data;
}

# 53 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
 __attribute((wakeup)) __attribute((interrupt(8))) void sig_PORT1_VECTOR(void)
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 57
      HplMsp430InterruptP$Port10$fired();
#line 57
      return;
    }
#line 58
  if (n & (1 << 1)) {
#line 58
      HplMsp430InterruptP$Port11$fired();
#line 58
      return;
    }
#line 59
  if (n & (1 << 2)) {
#line 59
      HplMsp430InterruptP$Port12$fired();
#line 59
      return;
    }
#line 60
  if (n & (1 << 3)) {
#line 60
      HplMsp430InterruptP$Port13$fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 4)) {
#line 61
      HplMsp430InterruptP$Port14$fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 5)) {
#line 62
      HplMsp430InterruptP$Port15$fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 6)) {
#line 63
      HplMsp430InterruptP$Port16$fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 7)) {
#line 64
      HplMsp430InterruptP$Port17$fired();
#line 64
      return;
    }
}

#line 158
 __attribute((wakeup)) __attribute((interrupt(2))) void sig_PORT2_VECTOR(void)
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 162
      HplMsp430InterruptP$Port20$fired();
#line 162
      return;
    }
#line 163
  if (n & (1 << 1)) {
#line 163
      HplMsp430InterruptP$Port21$fired();
#line 163
      return;
    }
#line 164
  if (n & (1 << 2)) {
#line 164
      HplMsp430InterruptP$Port22$fired();
#line 164
      return;
    }
#line 165
  if (n & (1 << 3)) {
#line 165
      HplMsp430InterruptP$Port23$fired();
#line 165
      return;
    }
#line 166
  if (n & (1 << 4)) {
#line 166
      HplMsp430InterruptP$Port24$fired();
#line 166
      return;
    }
#line 167
  if (n & (1 << 5)) {
#line 167
      HplMsp430InterruptP$Port25$fired();
#line 167
      return;
    }
#line 168
  if (n & (1 << 6)) {
#line 168
      HplMsp430InterruptP$Port26$fired();
#line 168
      return;
    }
#line 169
  if (n & (1 << 7)) {
#line 169
      HplMsp430InterruptP$Port27$fired();
#line 169
      return;
    }
}

# 122 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
 __attribute((wakeup)) __attribute((interrupt(14))) void sig_ADC_VECTOR(void)
#line 122
{
  HplAdc12P$HplAdc12$conversionDone(HplAdc12P$ADC12IV);
}

# 475 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP$stopConversion(void)
{
  uint8_t i;

  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
    Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTI_CHANNEL) {
      ADC12IV = 0;
      for (i = 1; i < Msp430Adc12ImplP$numChannels; i++) 
        Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(i).inch);
    }
  Msp430Adc12ImplP$HplAdc12$stopConversion();
  Msp430Adc12ImplP$HplAdc12$resetIFGs();
  Msp430Adc12ImplP$state &= ~Msp430Adc12ImplP$ADC_BUSY;
}

#line 149
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectIOFunc();
#line 154
      break;
      case 1: Msp430Adc12ImplP$Port61$selectIOFunc();
#line 155
      break;
      case 2: Msp430Adc12ImplP$Port62$selectIOFunc();
#line 156
      break;
      case 3: Msp430Adc12ImplP$Port63$selectIOFunc();
#line 157
      break;
      case 4: Msp430Adc12ImplP$Port64$selectIOFunc();
#line 158
      break;
      case 5: Msp430Adc12ImplP$Port65$selectIOFunc();
#line 159
      break;
      case 6: Msp430Adc12ImplP$Port66$selectIOFunc();
#line 160
      break;
      case 7: Msp430Adc12ImplP$Port67$selectIOFunc();
#line 161
      break;
    }
}

#line 603
static    error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(uint8_t arg_0x40c97518, uint16_t arg_0x40c27250){
#line 206
  unsigned char result;
#line 206

#line 206
  switch (arg_0x40c97518) {
#line 206
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 206
      result = AdcP$SingleChannelReadStream$singleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 206
      result = AdcP$SingleChannelReadStream$singleDataReady(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT, arg_0x40c27250);
#line 206
      break;
#line 206
    default:
#line 206
      result = Msp430Adc12ImplP$SingleChannel$default$singleDataReady(arg_0x40c97518, arg_0x40c27250);
#line 206
      break;
#line 206
    }
#line 206

#line 206
  return result;
#line 206
}
#line 206
# 167 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static   error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP$state) 
    {
      case AdcP$STATE_READ: 
        AdcP$owner = client;
      AdcP$value = data;
      AdcP$readDone$postTask();
      break;
      case AdcP$STATE_READNOW: 
        AdcP$ReadNow$readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

# 608 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static    uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t length)
{
  return 0;
}

# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   uint16_t *Msp430Adc12ImplP$SingleChannel$multipleDataReady(uint8_t arg_0x40c97518, uint16_t arg_0x40c27a00[], uint16_t arg_0x40c27b98){
#line 227
  unsigned int *result;
#line 227

#line 227
  switch (arg_0x40c97518) {
#line 227
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC$0$CLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 227
      result = AdcP$SingleChannelReadStream$multipleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$2$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*UBee430_APAppC.VoltageC.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC$0$CLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$3$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadClientC*/AdcReadClientC$1$CLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$4$ID:
#line 227
      result = AdcP$SingleChannelReadStream$multipleDataReady(/*UBee430_APAppC.TemperatureC.Msp430InternalTemperatureC.AdcReadStreamClientC*/AdcReadStreamClientC$1$RSCLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.LightToVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$5$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*UBee430_APAppC.LightToVoltageC.AdcReadClientC*/AdcReadClientC$2$CLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    case /*UBee430_APAppC.AdcZeroC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$6$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*UBee430_APAppC.AdcZeroC.AdcReadClientC*/AdcReadClientC$3$CLIENT, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    default:
#line 227
      result = Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(arg_0x40c97518, arg_0x40c27a00, arg_0x40c27b98);
#line 227
      break;
#line 227
    }
#line 227

#line 227
  return result;
#line 227
}
#line 227
# 278 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  error_t nextRequest;

  if (!AdcP$resultBuf) {
      AdcP$value = length;
      AdcP$resultBuf = buf;
      AdcP$signalBufferDone$postTask();
      if (!AdcP$streamBuf[streamClient]) {
        AdcP$finishStreamRequest$postTask();
        }
      else 
#line 289
        {

          struct AdcP$stream_entry_t *entry = AdcP$streamBuf[streamClient];

#line 292
          AdcP$streamBuf[streamClient] = AdcP$streamBuf[streamClient]->next;
          nextRequest = AdcP$SingleChannelReadStream$configureMultiple(streamClient, 
          &AdcP$streamConfig, (uint16_t *)entry, entry->count, AdcP$usPeriod[streamClient]);
          if (nextRequest == SUCCESS) {
            nextRequest = AdcP$SingleChannelReadStream$getData(streamClient);
            }
#line 297
          if (nextRequest != SUCCESS) {
              AdcP$streamBuf[AdcP$owner] = entry;
              AdcP$finishStreamRequest$postTask();
            }
        }
    }
  else 
#line 302
    {

      struct AdcP$stream_entry_t *entry = (struct AdcP$stream_entry_t *)buf;

#line 305
      entry->next = AdcP$streamBuf[streamClient];
      AdcP$streamBuf[streamClient] = entry;
      AdcP$finishStreamRequest$postTask();
    }
  return 0;
}

# 96 "/opt/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
 __attribute((wakeup)) __attribute((interrupt(18))) void sig_UART0RX_VECTOR(void)
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P$Interrupts$rxDone(temp);
}

 __attribute((wakeup)) __attribute((interrupt(16))) void sig_UART0TX_VECTOR(void)
#line 101
{
  if (HplMsp430Usart0P$HplI2C$isI2C()) {
    HplMsp430Usart0P$I2CInterrupts$fired();
    }
  else {
#line 105
    HplMsp430Usart0P$Interrupts$txDone();
    }
}

