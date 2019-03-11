/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "r8c38a_lib.h"                 /* R8C/38A ���䃉�C�u����       */
#include "types3_beep.h"                /* �u�U�[�ǉ�                   */
#include "printf_lib.h"
#include "data_flash_lib.h"

/*======================================*/
/* �V���{����`                         */
/*======================================*/
/* �萔�ݒ� */

// �ύX�֎~:
#define     TRC_MOTOR_CYCLE     20000   /* ���O,�E�O���[�^PWM�̎���     */
/* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* ����,�E��,����Ӱ�PWM�̎���   */
/* 50[ns] * 20000 = 1.00[ms]    */
#define     FREE                1       /* ���[�^���[�h�@�t���[         */
#define     BRAKE               0       /* ���[�^���[�h�@�u���[�L       */

#define     HIGH        1   /* 5V����           */
#define     LOW         0   /* 0V����           */

#define     ON                  1
#define     OFF                 0

#define     RIGHT               4
#define     LEFT                7

#define A 3
#define B 4
#define C 5
#define D 6

/* �f�[�^�t���b�V���֘A */
#define DF_SIZE  256  	/* �ǂݏ����T�C�Y */
#define DF_CHECK   0  	/* �f�[�^�t���b�V���`�F�b�N */
#define DF_DATA    1	/* �f�[�^ */

// �ύXOK:
/* dipsw:�@4�`6 */
#define  KP   5      /* ���  */
#define  KI   6      /* �ϕ�  */
#define  KD  25      /* ����  */

#define 	SABUN_L 			15
#define 	SABUN_R 			320

#define     RUNNING_TIME        31000L  /* ���s����(ms)                 */

#define     AD_1DEG             3       /* 1�x�������A/D�l�̑���       */

#define  	SERVO_PWM_MAX   	95    /* �T�[�{�̍ő�PWM        */
#define		LANCER_PWM_MAX		100		/* ���̍ő�PWM  		*/

#define     CENTER_LNC          512		/* ���̒��S��A/D�l		*/         

#define     STAIR_LIMIT         545     /* �J�[�u�F����臒l(A/D�l)   */

#define     UP4BIT /* dipsw2��4bit */272L * ( dipsw_get2() >> 4 )  /* ���s�W�I�ɓ��Ă�܂ł̋���: */
#define     MIN4BIT/* dipsw2��4bit */272L * ( dipsw_get2()&0x0f )  /* �n�[�t���C�����o����ʉ߂܂ł̃p���X��(272.5*2.5=25cm) */


// �T�[�{�p
// Arduino nano�Ƃ̐ڑ�:(�o��)
#define     SERVO_ANGLE_PORT    p5_addr.bit.b3
#define     SERVO_ZERO_PORT     p5_addr.bit.b2
#define     SERVO_MODE_PORT     p5_addr.bit.b1

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
unsigned int sensor_inp(unsigned char sensor);
unsigned char center_inp( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char convertBCD( int data );
void led_out( unsigned char led );
void servoSet( int num );
void fullColor_out( unsigned char type );
void traceMain( void );
void motor_r( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
void servoControl( void );
void servoControl2( void );
void stm_go( int degree );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
int getServoAngle( void );
int getAnalogSensor( void );
int diff( int pwm );
int getLancerAngle( void );
void hyouteki_check( void );
void lancerPwmOut( int pwm );
void lancerControl( void );
long map( long x, long in_min, long in_max, long out_min, long out_max );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/

/* �f�[�^�t���b�V���֘A */
volatile signed char	  data_buff[ DF_SIZE ];	 /* �ꎞ�ۑ��G���A 				 */ 

volatile const char      *C_DATE = __DATE__;     /* �R���p�C���������t           */
volatile const char      *C_TIME = __TIME__;     /* �R���p�C����������           */

volatile int             pattern       = 0;      /* �}�C�R���J�[����p�^�[��     */
volatile unsigned long   cnt_run       = 0;      /* �^�C�}�p                     */
volatile unsigned long   cnt1          = 0;      /* �^�C�}�p                     */
volatile unsigned long   check_cross_cnt = 0;    /* �^�C�}�p                     */
volatile unsigned long   check_sen_cnt = 0;      /* �^�C�}�p                     */
volatile unsigned long   check_enc_cnt = 0;      /* �^�C�}�p                     */
volatile unsigned long   stm_cnt     = 0;      /* �^�C�}�p                     */
volatile int             hitcount      = 0;      /* �n�[�t���C����ǂ񂾉�     */
volatile int             hyouteki_flag = 0;      /* �W�I�����������s������������(���s�W�I:0 �����W�I:1)*/
volatile int             anchi_cross = 0;        /* 1:�O��ɕЕ��̃f�W�^���Z���T������ 0:�f�W�^���̔����Ȃ� */
volatile int             heikou  = 0;
volatile int             stair_flag  = 0;        /* 1:�傫���Ȃ����Ă��� 0: ���� */
volatile int             iLeftSensor0 = 0;
volatile int             iRightSensor0 = 0;

/* �G���R�[�_�֘A     */
volatile int             iTimer10     = 0;       /* 10ms�J�E���g�p               */
volatile int             iEncoder     = 0;       /* 10ms���̍ŐV�l               */
volatile int             iEncoderMax  = 0;       /* ���ݍő�l                   */
volatile long            lEncoderLine = 0;       /* ���C�����o���̐ώZ�l       */
volatile long            lEncoderTotal = 0;      /* �ώZ�l�ۑ��p                 */
volatile unsigned int    uEncoderBuff = 0;       /* �v�Z�p�@���荞�ݓ��Ŏg�p     */

/* �T�[�{�֘A       */
volatile int             iSensorBefore;          /* �O��̃Z���T�l�ۑ�           */
volatile int             iServoPwm;              /* �T�[�{PWM�l                */
volatile int             iAngle0;                /* ���S����A/D�l�ۑ�            */

/* �T�[�{�֘A2          */
volatile int             iSetAngle;
volatile int             iAngleBefore2;
volatile int             iServoPwm2;

/* ��(DC���[�^�ƃ{�����[��ad7) */
volatile int             iLancer0;				 /* ���S����A/D�l�ۑ� 			 */
volatile int			 iSetLancer;			 /* �ڕW��getLancerAngle()�̒l   */
volatile int 			 iSetLancerAD;			 /* �ڕW��AD�l					 */
volatile int			 iLancerPwm;			 
volatile int			 iLancerBefore;
volatile int             integral_lancer;

/* TRC���W�X�^�̃o�b�t�@ */
volatile unsigned int    trcgrb_buff;            /* TRCGRB�̃o�b�t�@             */
volatile unsigned int    trcgrd_buff;            /* TRCGRD�̃o�b�t�@             */
volatile unsigned int    trcgrc_buff;


/* ���[�^�h���C�u���TypeS Ver.3���LED�A�f�B�b�v�X�C�b�`���� */
volatile unsigned char   types_led;              /* LED�l�ݒ�                    */
volatile unsigned char   types_dipsw;            /* �f�B�b�v�X�C�b�`�l�ۑ�       */

/* ���֍��l�v�Z�p�@�e�}�C�R���J�[�ɍ��킹�čČv�Z���ĉ����� */
volatile const int revolution_difference[] = {   /* �p�x������ցA�O�։�]���v�Z */
  100, 98, 97, 95, 94,
  93, 91, 90, 88, 87,
  86, 84, 83, 82, 80,
  79, 78, 76, 75, 74,
  72, 71, 70, 68, 67,
  66, 65, 63, 62, 61,
  59, 58, 57, 55, 54,
  53, 51, 50, 49, 47,
  46, 45, 43, 42, 40,
  39
};

/* ���̑� */
volatile long    integral = 0;
volatile int     TH_R = 0;  			/* �f�W�^���F����臒l */
volatile int     TH_L = 0;
volatile long    sum_l = 0;
volatile long    sum_r = 0;
volatile long    avg_l = 0;
volatile long    avg_r = 0;

/* �t���J���[LED�p(p6) */
volatile const int fullColor_data[] = {
  //  0:OFF 1:��  2:�΁@3:��  4:��  5:��  6:���F 7:��
  0x00, 0x01, 0x02, 0x04, 0x03, 0x05, 0x06,  0x07
};


/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
  int i;
  unsigned char b;
  
  /* �f�[�^�t���b�V�������p */
  int r;
  unsigned char c;
  /**************************/
	
  /* �}�C�R���@�\�̏����� */
  init();                             /* ������                       */
  _asm(" FSET I ");                   /* �S�̂̊��荞�݋���           */
  initBeepS();                        /* �u�U�[�֘A����               */
  init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
  readDataFlash( 0x3000, data_buff, DF_SIZE );
  
  if( data_buff[ DF_CHECK ] != 0x38 ){  // 0x38�łȂ���΁A���߂�0x3000�Ԓn�̃f�[�^�t���b�V������f�[�^��ǂݍ��񂾂Ɣ��f
  		data_buff[ DF_CHECK ] = 0x38;
		data_buff[ DF_DATA ]  = 0; 
  }
	
  /* �}�C�R���J�[�̏�ԏ����� */
  motor_mode_f( BRAKE, BRAKE );
  motor_mode_r( BRAKE, BRAKE );
  motor2_f( 0, 0 );
  motor2_r( 0, 0 );
  servoPwmOut( 0 );
  fullColor_out( 0 );
  setBeepPatternS( 0x8000 );
  iSetLancer = CENTER_LNC;
  iLancer0 = getLancerAngle();
  
  timer_ms(10);
  
  servoSet( 0 );

  while ( 1 ) {
	  
    servoControl();
    if ( pattern >= 11 && pattern < 100 ) {
	  
	  lancerControl();  // ��	
	  lancerPwmOut( iLancerPwm );
      /* ���Ԃɂ���~(1��) */
      if ( cnt_run > RUNNING_TIME ) {
        pattern = 101;  // ���s�I��
      }
	  
	  if( iEncoder <= 1 ){
	  	if( check_enc_cnt > 1000 ){
			pattern = 101;	
		}
	  }
	  else{
		check_enc_cnt = 0;	  
	  }
    }
	else{
		lancerPwmOut( 0 );
	}

    switch ( pattern ) {
      case 0:
        /* �v�b�V���X�C�b�`�����҂� */
        servoPwmOut( 0 );
        if ( pushsw_get() ) {
          setBeepPatternS( 0xcc00 );
          cnt1 = 0;
          pattern = 1;
          break;
        }

        b = p3_0;
        fullColor_out( b * 5 ); // �G���R�[�_�̐M����LED�ɕ\��
        printf( "Left=%4d , "
                        "CL=%1d , CR=%1d , Right=%4d            \r",
                         get_ad(7), get_ad(13), get_ad(12), get_ad(4));
        if ( cnt1 >= 600 ) cnt1 = 0;
        if ( cnt1 < 300 ) {
          led_out( 0x08 );
        }
        else if ( cnt1 < 600 ) {
          led_out( 0x04 );
        }

        break;

      case 1:
        /* �X�^���o�C:�X�^�[�g3�b�O */

        led_out( 0x11 );
        setBeepPatternS( 0x8000 );  // 3
        timer_ms( 1000 );

        led_out( 0x33 );
        setBeepPatternS( 0x8000 );  // 2
        timer_ms( 1000 );

        led_out( 0x77 );
        setBeepPatternS( 0x8000 );  // 1
        timer_ms( 1000 );

        led_out( 0xff );
        setBeepPatternS( 0xffff );  // GO!
        timer_ms( 1000 );

        led_out( 0x00 );
		sum_l = 0;
		sum_r = 0; 
		avg_l = 0;
		avg_r = 0;
	
		// 100�񕪂̃Z���T�l���獇�v�l���o��
		for( i = 0; i < 50; i++ ){
			fullColor_out( 3 );
			sum_l += sensor_inp( LEFT  );
			timer_ms(1);
			fullColor_out( 2 );
			sum_r += sensor_inp( RIGHT );
			timer_ms(1);
		}
		
		//  ���v�𗘗p���ĕ��ς��o��
		avg_l = (int)( sum_l / 50 );
		avg_r = (int)( sum_r / 50 );
		
		// �f�W�^���F���p��臒l�����߂�
		// (臒l) = (�X�^�[�g�ʒu�̃Z���T���ϒl) - (�D�F�Ɣ��̍���)
		TH_L = avg_l - SABUN_L;
		TH_R = avg_r - SABUN_R;
		printf("\nsum_l= %4ld,  sum_r= %4ld\n", sum_l, sum_r); 
		printf("\navg_l= %4ld,  avg_r= %4ld\n", avg_l, avg_r); 
		printf("\nTH_L= %4d,  TH_R= %4d\n", TH_L, TH_R); 
	/*
		
		TH_L = sensor_inp( LEFT  ) - SABUN_L;
		TH_R = sensor_inp( RIGHT ) - SABUN_R;
	*/	
		
        anchi_cross = 0;
        check_sen_cnt = 0;
        check_enc_cnt = 0;
        lEncoderTotal = 0;
        lEncoderLine  = 0;
		hyouteki_flag = 0;
		iAngle0 = getServoAngle();  	/* 0�x�̈ʒu�L��          */
		iLancer0 = getLancerAngle();  	/* 0�x�̈ʒu�L��          */
		cnt_run = 0;
        cnt1 = 0;		
		pattern = 11;
        break;

      case 11:
        /* �ʏ�g���[�X */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 0 );
        servoSet( 0 );

        if ( check_crossline() ) { /* �J�[�u�`�F�b�N */
          cnt1 = 0;
          pattern = 20;
          break;
        }

        if ( check_rightline() && hyouteki_flag == 0 ) { /* �E�n�[�t���C���`�F�b�N:���s�W�I*/
          pattern = 70;
          cnt1 = 0;
          servoSet( 1 );  // ���s�U�邾��
          break;
        }

        if ( check_rightline() && hyouteki_flag == 1 ) { /* �E�n�[�t���C���`�F�b�N:�����W�I*/
          pattern = 30;
          cnt1 = 0;
          servoSet( A );  // ����A
          break;
        }
        break;

      case 20:
        /* �N���X���C�����o����̏��� */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 7 );
        servoSet( 0 );
        hyouteki_check();
        heikou = 0;
        cnt1 = 0;
        setBeepPatternS(0x8000);
        lEncoderLine = lEncoderTotal;
        pattern = 21;
        break;

      case 21:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
		fullColor_out( 7 );
        servoSet( 0 );
		if (lEncoderTotal - lEncoderLine >= (273L * 3)) { // �N���X���C���̓ǂݔ�΂�
          pattern = 22;
          lEncoderLine = lEncoderTotal;
          cnt1 = 0;
        }
        break;

      case 22:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
		fullColor_out( 7 );
		servoSet( 0 );
        if ( check_rightline() || check_leftline() ) {  // �J�[�u�o���`�F�b�N
          servoSet( 0 );
          cnt1 = 0;
          lEncoderLine = lEncoderTotal;
          pattern = 23;
        }
        break;

      case 23:
        servoPwmOut( iServoPwm );
		servoSet( 0 );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L*3)) {
          pattern = 11;
          cnt1 = 0;
          servoSet( 0 );
        }
        break;

      case 30:
        /* �����W�I(A) */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 1 );
        servoSet( A );
        lEncoderLine = lEncoderTotal;
        cnt1 = 0;
        pattern = 31;
        break;

      case 31:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          //pattern = 11;
          pattern = 32;
          lEncoderLine = lEncoderTotal;
		  cnt1 = 0;
		  break;
        }
       
        break;

      case 32:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 3) &&  check_leftline()  ) {
          //pattern = 11;
          pattern = 40;
		  break;
        }
		
        break;

      case 40:
        /* �����W�I(B) */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 2 );
        servoSet( B );
        lEncoderLine = lEncoderTotal;
        cnt1 = 0;
        pattern = 41;
        break;

      case 41:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          lEncoderLine = lEncoderTotal;
          pattern = 42;
		  cnt1 = 0;
        }
        break;

      case 42:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 3) && check_rightline()) {
          //pattern = 11;
          pattern = 50;
		  break;
        }
        
        break;

      case 50:
        /* �����W�I(C) */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 3 );
        servoSet( C );
        lEncoderLine = lEncoderTotal;
        cnt1 = 0;
        pattern = 51;
        break;

      case 51:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          lEncoderLine = lEncoderTotal;
          pattern = 52;
          cnt1 = 0;
		}
        break;

      case 52:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( lEncoderTotal - lEncoderLine >= (273L * 3) && check_leftline()) {
          //pattern = 11;
          pattern = 60;
		  break;
        }
		
       
        break;

      case 60:
        /* �����W�I(D) */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        fullColor_out( 4 );
        servoSet( D );
        lEncoderLine = lEncoderTotal;
        cnt1 = 0;
        pattern = 61;
        break;

      case 61:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        if ( check_crossline() ) { /* �J�[�u�`�F�b�N */
          cnt1 = 0;
          pattern = 20;
          break;
        }
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          pattern = 62;
          lEncoderLine = lEncoderTotal;
		  cnt1 = 0;
        }
        break;

      case 62:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
		if ( check_rightline() || check_leftline()) { /* �J�[�u�`�F�b�N */
            cnt1 = 0;
            pattern = 20;
			break;
          }
		
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          if ( check_rightline() ) { /* �J�[�u�`�F�b�N */
            cnt1 = 0;
            pattern = 20;
			break;
          }
        }
        break;

      case 70:
        /* ���s�W�I */
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        servoSet( 1 );    // �߂Â��邾��
		fullColor_out( 5 ); // ���F
        setBeepPatternS( 0x8000 );
        lEncoderLine = lEncoderTotal;
        cnt1 = 0;
        pattern = 71;
        break;

      case 71:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        servoSet( 1 );    // �߂Â��邾��
		if ( check_crossline() ) { /* �J�[�u�`�F�b�N */
          cnt1 = 0;
          pattern = 20;
          break;
        }
        if ( lEncoderTotal - lEncoderLine >= (273L * 2)) {
          pattern = 72;
          lEncoderLine = lEncoderTotal;
          cnt1 = 0;
        }
        break;

      case 72:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        servoSet( 1 );    // �߂Â��邾��
		if ( check_rightline() && check_leftline() ) { /* �J�[�u�`�F�b�N */
          cnt1 = 0;
          pattern = 20;
          break;
        }
        if (lEncoderTotal - lEncoderLine >= UP4BIT) {
          pattern = 73;
          lEncoderLine = lEncoderTotal;
          cnt1 = 0;
		  break;
        }
		
        break;

      case 73:
        servoPwmOut( iServoPwm );
        traceMain();      // ���C���g���[�X�֐�
        servoSet( 2 );    // ���Ăɍs��
		fullColor_out( 6 ); // ���F
        setBeepPatternS( 0x8800 );
		if( check_crossline() ){
			pattern = 20;
			cnt1 = 0;
			break;	
		}
        if ( lEncoderTotal - lEncoderLine >= MIN4BIT) { // ���ĂĂ���ʉ߂���܂ł̃G���R�[�_�l
          heikou++;
          if ( heikou >= 2 ) {
            heikou = 0;
            cnt1 = 0;
            pattern = 20;
            break;
          }
          else {
			pattern = 11;
			cnt1 = 0;
          }
        }
        break;

      case 101:
        /* ��~���� */
        servoPwmOut( iServoPwm );
        motor_mode_f( FREE, FREE );
        motor_mode_r( FREE, FREE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );
        led_out( 0x00 );
        fullColor_out( 4 );
        pattern = 102;
        cnt1 = 0;
        break;

      case 102:
        /* ��~���� */
        servoPwmOut( iServoPwm );
        if ( cnt1 > 1000 ) {
          servoPwmOut( 0 );
          motor_mode_f( FREE, FREE );
          motor_mode_r( FREE, FREE );
          motor2_f( 0, 0 );
          motor2_r( 0, 0 );
          pattern = 103;
          setBeepPatternS( 0xcc00 );
          cnt1 = 0;
          break;
        }
        break;

      case 103:
        /* ��~���� */
        servoPwmOut( iServoPwm );
        motor_mode_f( FREE, FREE );
        motor_mode_r( FREE, FREE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );
        if ( cnt1 > 500 ) {
          pattern = 104;
          cnt1 = 0;
          break;
        }
        break;

      case 104:
        /* �������Ȃ� */
        servoPwmOut( 0 );
        servoSet( 0 );    // �߂Â��邾��
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );

        for ( i = 1; i < 8; i++ ) {
          fullColor_out( i );
          timer_ms( 200 );
        }
        break;

      default:
        break;
    }
  }
}

/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void )
{
  int i;

  init_xin_clk();  // ���W�X�^�̏�����

  /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
  prc0  = 1;                          /* �v���e�N�g����               */
  cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
  cm05  = 0;                          /* XIN�N���b�N���U              */
  for (i = 0; i < 50; i++ );          /* ���肷��܂ŏ����҂�(��10ms) */
  ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
  prc0  = 0;

  /* �|�[�g�̓��o�͐ݒ� */

  /*  PWM(�\��)       ���OM_PMW       �E�OM_PWM       �u�U�[
      �Z���T���[      �Z���T����      �Z���T�E��      �Z���T�E�[  */
  p0   = 0x00;
  prc2 = 1;                           /* PD0�̃v���e�N�g����          */
  pd0  = 0xf0;

  /*  �Z���T���S      �����ް         RxD0            TxD0
      DIPSW3          DIPSW2          DIPSW1          DIPSW0         */
  pur0 |= 0x04;                       /* P1_3?P1_0�̃v���A�b�vON     */
  p1  = 0x00;
  pd1 = 0x10;

  /*  �E�OM_����      �X�e�AM_����    �X�e�AM_PWM     �E��M_PWM
      �E��M_����      ����M_PWM       ����M_����      ���OM_����      */
  p2  = 0x00;
  pd2 = 0xff;

  /* !---�ǉ��E�ύX---! */
  /*  Arduino(ANGLE)  none            none            none
      none            none            none            �G���R�[�_A��   */
  p3  = 0x00;
  pd3 = 0xfe;

  /*  XOUT            XIN             �{�[�h���LED   none
      none            VREF            none            none            */
  p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
  pd4 = 0xb8;

  /*  none            none            none            none
      none            none            none            none            */
  p5  = 0x00;
  pd5 = 0xff;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xff;

  /*  DC���[�^��]����1   DC���[�^��]����2       CN6.4����       CN6.5����
      none(��۸ޗ\��) �p�xVR          �Z���T_����۸�  �Z���T_�E��۸�  */
  p7  = 0x00;
  pd7 = 0xc0;

  /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
      DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
  pur2 |= 0x03;                       /* P8_7?P8_0�̃v���A�b�vON      */
  p8  = 0x00;
  pd8 = 0x00;

  /*  -               -               �߯������       P8����(LEDorSW)
      �E�OM_Free      ���OM_Free      �E��M_Free      ����M_Free      */
  p9  = 0x00;
  pd9 = 0x1f;
  pu23 = 1;   // P9_4,P9_5���v���A�b�v����

  /* �^�C�}RB�̐ݒ� */
  /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                  = 1 / (20*10^6) * 200        * 100
                  = 0.001[s] = 1[ms]
  */
  trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
  trbpre = 200 - 1;                   /* �v���X�P�[�����W�X�^         */
  trbpr  = 100 - 1;                   /* �v���C�}�����W�X�^           */
  trbic  = 0x06;                      /* ���荞�ݗD�惌�x���ݒ�       */
  trbcr  = 0x01;                      /* �J�E���g�J�n                 */

  /* A/D�R���o�[�^�̐ݒ� */
//  admod   = 0x33;                     /* �J��Ԃ��|�����[�h�ɐݒ�     */
//  adinsel = 0x90;                     /* ���͒[�qP7��4�[�q��I��      */
//  adcon1  = 0x30;                     /* A/D����\                  */
//  _asm(" NOP ");                      /* ��AD��1�T�C�N���E�G�C�g�����*/
//  adcon0  = 0x01;                     /* A/D�ϊ��X�^�[�g              */

  /* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ� */
  timsr = 0x40;                       /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
  trgcr = 0x15;                       /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
  trgmr = 0x80;                       /* TRG�̃J�E���g�J�n            */

  /* �^�C�}RC PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^) */
  trcpsr0 = 0x40;                     /* TRCIOA,B�[�q�̐ݒ�           */
  trcpsr1 = 0x33;                     /* TRCIOC,D�[�q�̐ݒ�           */
  trcmr   = 0x0f;                     /* PWM���[�h�I���r�b�g�ݒ�      */
  trccr1  = 0x8e;                     /* �������:f1,�����o�͂̐ݒ�    */
  trccr2  = 0x00;                     /* �o�̓��x���̐ݒ�             */
  trcgra  = TRC_MOTOR_CYCLE - 1;      /* �����ݒ�                     */
  trcgrb  = trcgrb_buff = trcgra;     /* P0_5�[�q��ON��(���O���[�^)   */
  trcgrc  = trcgrc_buff = trcgra;     /* P0_7�[�q��ON��(�\��)         */
  trcgrd  = trcgrd_buff = trcgra;     /* P0_6�[�q��ON��(�E�O���[�^)   */
  trcic   = 0x07;                     /* ���荞�ݗD�惌�x���ݒ�       */
  trcier  = 0x01;                     /* IMIA������                   */
  trcoer  = 0x01;                     /* �o�͒[�q�̑I��               */
  trcmr  |= 0x80;                     /* TRC�J�E���g�J�n              */

  /* �^�C�}RD ���Z�b�g����PWM���[�h�ݒ�(����Ӱ��A�E��Ӱ��A����Ӱ�) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0  = 0x20;                     /* �\�[�X�J�E���g�̑I��:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* �����ݒ�             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON��(���ヂ�[�^)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON��(�E�ヂ�[�^)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5�[�q��ON��(�T�[�{���[�^) */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
  unsigned int i;

  _asm(" FSET I ");                    /* �^�C�}RB�ȏ�̊��荞�݋���   */

  cnt1++;
  cnt_run++;
  check_sen_cnt++;
  check_cross_cnt++;


  /* �T�[�{���[�^���� */
  servoControl();
  servoControl2();

  /* �u�U�[���� */
  beepProcessS();
  
  if( check_leftline() || check_rightline() ){
	//setBeepPatternS( 0x8000 );	  
  }

  /* 10��1����s���鏈�� */
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* �G���R�[�_���� */
      i = trg;
      iEncoder       = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      if ( iEncoder > iEncoderMax ) {
        iEncoderMax = iEncoder;
      }
      uEncoderBuff   = i;
      break;

    case 2:
      /* �X�C�b�`�ǂݍ��ݏ��� */
      p9_4 = 0;                       /* LED�o��OFF                   */
      pd8  = 0x00;
      break;

    case 3:
      /* �X�C�b�`�ǂݍ��݁ALED�o�� */
      types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
      p8  = types_led;                /* ��ײ�ފ��TypeS Ver.3��LED�֏o��*/
      pd8 = 0xff;
      p9_4 = 1;                       /* LED�o��ON                    */
      break;

    case 4:
      if ( pattern == 1 ) {
        servoPwmOut( iServoPwm );
      }
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;

    case 9:
      break;

    case 10:
      /* iTimer10�ϐ��̏��� */
      iTimer10 = 0;
      break;
  }
}

/************************************************************************/
/* �^�C�}RC ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;  /* �t���O�N���A */

  /* �^�C�}RC�@�f���[�e�B��̐ݒ� */
  trcgrb = trcgrb_buff;
  trcgrc = trcgrc_buff;
  trcgrd = trcgrd_buff;
}

/************************************************************************/
/* �J�[�u���o����(�i�����̂�)                                           */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{
  int ret = 0;
  int angle;

  angle = getServoAngle();

  if ( abs( angle ) <= 2 ) { // �n���h�����قڒ����̂Ƃ�
    if ( check_leftline() && check_rightline()) { // ���E�̃f�W�^���Z���T���������Ă���΃J�[�u�F��
      ret = 1;
    }
  }

  return ret;
}

/************************************************************************/
/* �E�n�[�t���C�����o����                        */
/* ���� �Ȃ�                              */
/* �߂�l 0:�n�[�t���C���Ȃ� 1:����                  */
/************************************************************************/
int check_rightline( void )
{
  int ret = 0;

  if ( (sensor_inp(RIGHT)) < TH_R) {
    ret = 1;
  }
  return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:���n�[�t���C���Ȃ� 1:����                                   */
/************************************************************************/
int check_leftline( void )
{
  int ret = 0;

  if ( (sensor_inp(LEFT)) < TH_L) {
    ret = 1;
  }
  return ret;
}


/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃f�W�^���Z���T�l�ǂݍ���              */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���[�A�����A�E���A�E�[�̃f�W�^���Z���T 0:�� 1:��              */
/************************************************************************/
unsigned int sensor_inp(unsigned char sensor)
{
  return get_ad( sensor );
}

/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0?15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
  unsigned char sw;

  sw = p1 & 0x0f;                     /* P1_3?P1_0�ǂݍ���           */

  return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0?255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
  /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
  return types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
  unsigned char sw;

  sw = ~p9_5 & 0x01;

  return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
  /* ���ۂ̏o�͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
  types_led = led;
}

/************************************************************************/
/* �t���J���[LED�̐���                                          */
/* �����@ �_���p�^�[��                                             */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void fullColor_out( unsigned char type )
{
  p6 = ~fullColor_data[ type ];
}


/************************************************************************/
/* ��ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	
	//accele_l = -accele_l;
	
    /* ���ヂ�[�^ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
	
	accele_r = -accele_r;
	
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* ��ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* ���ヂ�[�^ */
	
	if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
	
	accele_r = -accele_r;
    
	if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* �O�ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
  int sw_data;

  sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
  accele_l = accele_l * sw_data / 20;
  accele_r = accele_r * sw_data / 20;

  /* ���O���[�^ */
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }

  /* �E�O���[�^ */
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}

/************************************************************************/
/* �O�ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
  /* ���O���[�^ */
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }

  /* �E�O���[�^ */
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}


/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_0 = 1;
  } else {
    p9_0 = 0;
  }
  if ( mode_r ) {
    p9_1 = 1;
  } else {
    p9_1 = 0;
  }
}

/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_2 = 1;
  } else {
    p9_2 = 0;
  }
  if ( mode_r ) {
    p9_3 = 1;
  } else {
    p9_3 = 0;
  }
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
  if ( pwm >= 0 ) {
    p2_6 = 0;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
  } else {
    p2_6 = 1;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -pwm ) / 100;
  }

}

//************************************************************************/
/* �T�[�{�p�x�擾                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( get_ad(14) - iAngle0 );
}

/************************************************************************/
/* �A�i���O�Z���T�l�擾                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
  int ret;

  //  = ��  -  �E
  ret = (get_ad(13)) - get_ad(12);                    /* �A�i���O�Z���T���擾       */

  return ret;
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ��                               */
/************************************************************************/
void servoControl( void )
{
  int i, iRet, iP, iD;

  i = getAnalogSensor();          /* �Z���T�l�擾                 */

  //error = getAnalogSensor();
  integral = i + integral * (1 / 2);
  iRet = KP * i + KI * integral + KD * (i - iSensorBefore);

  /* �T�[�{���[�^�pPWM�l�v�Z */
  //    iP = KP * i;                        /* ���                         */
  //    iD = KD * ( iSensorBefore - i );    /* ����(�ڈ���P��5?10�{)       */
  //    iRet = iP - iD;
  iRet /= 32;

  /* PWM�̏���̐ݒ� */
  if ( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX; /* �}�C�R���J�[�����肵����     */
  if ( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX; /* �����90���炢�ɂ��Ă������� */
  iServoPwm = iRet;

  iSensorBefore = i;                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/* �T�[�{���[�^���� �p�x�w��p                     */
/* ���� �Ȃ�                              */
/* �߂�l �O���[�o���ϐ� iServoPwm2 �ɑ��                 */
/************************************************************************/
void servoControl2( void )
{
  int i, j, iRet, iP, iD;

  i = iSetAngle;            /* �ݒ肵�����p�x  */
  j = getServoAngle();        /* ���݂̊p�x    */

  /* �T�[�{���[�^�pPWM�l�v�Z */
  iP = KP * (j - i);          /* ��� */
  iD = KD * (iAngleBefore2 - j);    /* ���� */

  iRet = iP - iD;
  iRet /= 2;

  if ( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX; /* �}�C�R���J�[�����肵����   */
  if ( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX; /* �����90���炢�ɂ��Ă������� */

  iServoPwm2 = iRet;
  iAngleBefore2 = j;
}

/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
  int i, ret;

  i  = getServoAngle() / AD_1DEG;           /* 1�x������̑����Ŋ���        */
  if ( i <  0 ) i = -i;
  if ( i > 45 ) i = 45;
  ret = revolution_difference[i] * pwm / 100;

  return ret;
}

/************************************************************************/
/* 10�i����BCD�l                            */
/* ����    BCD�ɂ������l(int�^)                     */
/* �߂�l  BCD�l                            */
/* ����    BCD�������͈͂�0?99�܂�                    */
/************************************************************************/
unsigned char convertBCD( int data )
{
  unsigned char b1, b2;
  unsigned char ret;

  b1 = data / 10;
  b2 = data % 10;
  ret = b1 * 0x10 + b2;

  return ret;
}
/************************************************************************/
/* ���C���g���[�X�֐��{��                                               */
/* ����   �f���[�e�B��(0?100)                                          */
/* �߂�l �Ȃ�                                                          */
/* ���l �@�̂ɍ��킹�ăp�����[�^�����������Ă�������                    */
/************************************************************************/

void traceMain( void )
{
  const int _3MS = 86;
  const int _2MS = 55;

  int i;
  i = getServoAngle();

  if ( i > 50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor_f( 60, diff(60) );
      motor_r( 60, diff(60) );
    }
  } else if ( i < -50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor_f( diff(60), 60 );
      motor_r( diff(60), 60 );
    }
  } else if ( i > 15 ) {
    if (iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( -50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s�ȏ�Ȃ�
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor_f( 75, 0 );  // ���ւ�0%�ɂ��āA�J�[�u���Ȃ���₷������
      motor_r( 75, 0 );
    } else {
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor_f( 70, diff(70) );
      motor_r( 70, diff(70) );
    }
  } else if ( i < -15 ) {
    if ( iEncoder >= _3MS ) {
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( -50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s�ȏ�Ȃ�
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor_f( 0, 75 );         // ���ւ�0%�ɂ��āA�J�[�u���Ȃ���₷������
      motor_r( 0, 75 );
    } else {
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor_f( diff(70), 70 );
      motor_r( diff(70), 70 );
    }
  } else {
    if ( iEncoder >= (dipsw_get() * 2 + 30) ) { // 50(2.0m/s)?138(5.0m/s)  // ����l:15*2+14 = 44
      // dip_sw�̒l��
      // 0��0*2+25=25(2.3m/s) 8�� 8*2+25=41(3.7m/s)
      // 1��1*2+25=27(2.5m/s) 9�� 9*2+25=43(3.9m/s)
      // 2��2*2+25=29(2.6m/s) 10��10*2+25=45(4.1m/s)
      // 3��3*2+25=31(2.8m/s) 11��11*2+25=47(4.3m/s)
      // 4��4*2+25=33(3.0m/s) 12��12*2+25=49(4.5m/s)
      // 5��5*2+25=35(3.2m/s) 13��13*2+25=51(4.6m/s)
      // 6��6*2+25=37(3.4m/s) 14��14*2+25=53(4.8m/s)
      // 7��7*2+25=39(3.5m/s) 15��15*2+25=55(5.0m/s)
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( 0, 0 );
      motor2_r( 0, 0 );

    } else {
      motor_mode_f( FREE, FREE );
      motor_mode_r( FREE, FREE );
      motor2_f( 100, 100 );
      motor2_r( 100, 100 );
    }
  }
}

/*
void traceMain( void ){
		int i;
		i = getServoAngle();
        if( i > 170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( diff(40), 40 );
            motor_r( diff(40), 40 );
        } else if( i > 25 ) {
      		motor_mode_f( FREE, BRAKE );
      		motor_mode_r( FREE, BRAKE );
			motor_f( diff(50), 50 );
            motor_r( diff(50), 50 );
        } else if( i < -170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( 40, diff(40) );
            motor_r( 40, diff(40) );
        } else if( i < -25 ) {
			motor_mode_f( BRAKE, FREE );
      		motor_mode_r( BRAKE, FREE );
			motor_f( 50, diff(50) );
            motor_r( 50, diff(50) );
        } else {	
      		motor_mode_f( FREE, FREE );
      		motor_mode_r( FREE, FREE );
            motor_f( 80, 80 );
            motor_r( 80, 80 );
        }	
}
*/

/************************************************************************/
/* �ߋ��̉����̌��o�񐔂ɉ����ĕW�I����������                           */
/* ����   �Ȃ�                                                          */
/* �߂�l �ϐ�hyouteki_flag��(���s�W�I:0 �����W�I:1)������              */
/************************************************************************/
void hyouteki_check( void ) {
  hitcount = 1 - hitcount;
  switch ( hitcount ) {
    case 0:
      hyouteki_flag = 0;
      break;

    case 1:
      hyouteki_flag = 1;
      break;

    default:
      break;

  }
}

/************************************************************************/
/* ���l������͈͂���ʂ͈̔͂ɕϊ�(Arduino��map�֐��Ɠ���)             */
/*                                                                      */
/* ����   x: �ϊ����������l                                             */
/*        in_min: ���݂͈̔͂̉���                                      */
/*        int_max: ���݂͈̔͂̏��                                     */
/*        out_min: �ϊ���͈̔͂̉���                                   */
/*        out_max: �ϊ���͈̔͂̏��                                   */
/*                                                                      */
/* �߂�l �ϊ���̐��l (long)                                           */
/************************************************************************/
long map( long x, long in_min, long in_max, long out_min, long out_max ) {

  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;

}

/************************************************************************/
/* ���p�x�擾	                                                        */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getLancerAngle( void )
{
    return( get_ad(15) - iLancer0 );  // TypeS���AN16(p7_4)��R13���O��
}

/************************************************************************/
/* �T�[�{���[�^����(��)			                                        */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void lancerPwmOut( int pwm )
{
    if( abs(pwm) < 1 ){
		p7_7 = 1;
		p7_6 = 1;
	}
	else{
	 
		if( pwm >= 0 ) {
			p7_7 = 1;
			p7_6 = 0;
    	} else {
			p7_7 = 0;
			p7_6 = 1;	
        	pwm = -pwm;
    	}
	    if( pwm <= 5 ) {
        	trcgrc = trcgrc_buff = trcgra;
    	} else {
    	    trcgrc_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * pwm / 100;
    	}
	}
}

/************************************************************************/
/* �T�[�{���[�^���� ��       											*/
/* ���� �Ȃ� 															*/
/* �߂�l �O���[�o���ϐ� iLancerPwm �ɑ�� 								*/
/************************************************************************/
void lancerControl( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!�ǉ��E�ύX!!! */
	// i = iSetAngle; 						/* �ݒ肵�����p�x 	*/
	// j = getServoAngle(); 				/* ���݂̊p�x 		*/
	
	
	i = iSetLancer; 						/* �ݒ肵�����p�x 	*/
	j = get_ad(15);				 			/* ���݂̊p�x 		*/
	
	/* �T�[�{���[�^�pPWM�l�v�Z */
	integral_lancer = j + integral_lancer * (1 / 2);
    
	/*     P       I                     D                      */
  	iRet = 3 * i + 3 * integral_lancer + 5 * (i - iLancerBefore);
    iRet /= 2;
	
	if( iRet >  LANCER_PWM_MAX ) iRet =  LANCER_PWM_MAX;	/* �}�C�R���J�[�����肵���� 	*/
	if( iRet < -LANCER_PWM_MAX ) iRet = -LANCER_PWM_MAX; 	/* �����90���炢�ɂ��Ă������� */
	
	iLancerPwm = iRet;
	iLancerBefore = j;
}

void servoSet( int num ) {
  if ( num != 0 ) {
    if ( num == 1 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 2 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 3 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 4 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 1;
    }
    if ( num == 5 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 1;
    }
    if ( num == 6 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 1;
    }
  }
  if ( (num == 0) || (num >= 7) ) {
    SERVO_ANGLE_PORT = 0;
    SERVO_ZERO_PORT  = 0;
    SERVO_MODE_PORT  = 0;
  }
}