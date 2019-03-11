/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     モータドライブ基板TypeS Ver.3・アナログセンサ基板TypeS Ver.2*/
/*              を使用したマイコンカートレースプログラム                    */
/* バージョン   Ver.1.01                                                    */
/* Date         2011.06.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*
本プログラムは、
●モータドライブ基板TypeS Ver.3
●アナログセンサ基板TypeS Ver.2
を使用したマイコンカーを動作させるプログラムです。
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "r8c38a_lib.h"                 /* R8C/38A 制御ライブラリ       */
#include "types3_beep.h"                /* ブザー追加                   */
#include "printf_lib.h"

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* 定数設定 */

// 変更禁止:
#define     TRC_MOTOR_CYCLE     20000   /* 左前,右前モータPWMの周期     */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* 左後,右後,ｻｰﾎﾞﾓｰﾀPWMの周期   */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     FREE                1       /* モータモード　フリー         */
#define     BRAKE               0       /* モータモード　ブレーキ       */

#define     HIGH				1		/* 5V入力						*/
#define		LOW					0		/* 0V入力						*/

#define     ON                  1
#define     OFF                 0

#define     RIGHT               4
#define     LEFT                7

#define A 3
#define B 4
#define C 5
#define D 6

// 変更OK:
				/* dipsw:　1?3 */
//#define  KP   5      /* 比例 	*/
//#define  KI   5      /* 積分	*/
//#define  KD  25      /* 微分 	*/

				/* dipsw:　4?6 */
//#define  KP   7      /* 比例 	*/
//#define  KI   7      /* 積分	*/
//#define  KD  35      /* 微分 	*/

				/* dipsw:　7?9 */
#define  KP   5      /* 比例 	*/
#define  KI   5      /* 積分	*/
#define  KD  25      /* 微分 	*/  

#define     TH_R        		500/* デジタル認識の閾値 */
#define     TH_L                500

#define     RUNNING_TIME        31000L  /* 走行時間(ms)                 */      

#define     AD_1DEG             3       /* 1度あたりのA/D値の増分       */

#define		SERVO_PWM_MAX		95		/* サーボの最大PWM				*/

#define     STAIR_LIMIT         545     /* カーブ認識の閾値(A/D値)		*/

#define     UP4BIT /* dipsw2上4bit */272L * ( dipsw_get2() >> 4 )  /* 平行標的に当てるまでの距離: */
#define     MIN4BIT/* dipsw2下4bit */272L * ( dipsw_get2()&0x0f )  /* ハーフライン検出から通過までのパルス数(272.5*2.5=25cm) */


// サーボ用
// Arduino nanoとの接続:(出力)
#define     SERVO_ANGLE_PORT    p5_addr.bit.b3
#define     SERVO_ZERO_PORT     p5_addr.bit.b2
#define     SERVO_MODE_PORT     p5_addr.bit.b1

/*======================================*/
/* プロトタイプ宣言                     */
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
void hyouteki_check( void );
long map( long x, long in_min, long in_max, long out_min, long out_max );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/


volatile const char      *C_DATE = __DATE__;     /* コンパイルした日付           */
volatile const char      *C_TIME = __TIME__;     /* コンパイルした時間           */

volatile int             pattern       = 0;      /* マイコンカー動作パターン     */
volatile unsigned long   cnt_run       = 0;      /* タイマ用                     */
volatile unsigned long   cnt1          = 0;      /* タイマ用                     */ 
volatile unsigned long   check_cross_cnt= 0;     /* タイマ用                     */
volatile unsigned long   check_sen_cnt = 0;      /* タイマ用                     */
volatile unsigned long   check_enc_cnt = 0;      /* タイマ用                     */
volatile unsigned long   stm_cnt 	   = 0;      /* タイマ用                     */
volatile int             hitcount      = 0;      /* ハーフラインを読んだ回数     */
volatile int             hyouteki_flag = 0;      /* 標的が垂直か平行かを見分ける(平行標的:0 垂直標的:1)*/
volatile int             anchi_cross = 0;        /* 1:前回に片方のデジタルセンサが反応 0:デジタルの反応なし */
volatile int             heikou  = 0;
volatile int             stair_flag  = 0;        /* 1:大きく曲がっている 0: 直線 */

/* エンコーダ関連 		 */
volatile int             iTimer10     = 0;       /* 10msカウント用               */
volatile int             iEncoder     = 0;       /* 10ms毎の最新値               */
volatile int             iEncoderMax  = 0;       /* 現在最大値                   */
volatile long            lEncoderLine = 0;       /* ライン検出時の積算値     	 */
volatile long            lEncoderTotal= 0;       /* 積算値保存用                 */
volatile unsigned int    uEncoderBuff = 0;       /* 計算用　割り込み内で使用     */

/* サーボ関連 			 */
volatile int             iSensorBefore;          /* 前回のセンサ値保存           */
volatile int             iServoPwm;              /* サーボPWM値              	 */
volatile int             iAngle0;                /* 中心時のA/D値保存            */

/* サーボ関連2 		     */
volatile int             iSetAngle;
volatile int             iAngleBefore2;
volatile int             iServoPwm2;

/* TRCレジスタのバッファ */
volatile unsigned int    trcgrb_buff;            /* TRCGRBのバッファ             */
volatile unsigned int    trcgrd_buff;            /* TRCGRDのバッファ             */

/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
volatile unsigned char   types_led;              /* LED値設定                    */
volatile unsigned char   types_dipsw;            /* ディップスイッチ値保存       */

/* 内輪差値計算用　各マイコンカーに合わせて再計算して下さい */
volatile const int revolution_difference[] = {   /* 角度から内輪、外輪回転差計算 */
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

/* ステッピングモータ用 */
volatile int stm_run = 0;  // 0:停止 1:動作
volatile int stm_nowAngle= 0;
volatile int stm_stdby = 1;
volatile int stm_angle = 0;
volatile int stm_targetAngle = 0;
volatile const int stm_data[] = {
	0x0c, 0x04, 0x00, 0x08
};

int integral = 0;

/* フルカラーLED用(p6) */
volatile const int fullColor_data[] = {
//  0:OFF 1:赤  2:緑　3:青  4:黄  5:紫  6:水色 7:白 
	0x00, 0x01, 0x02, 0x04, 0x03, 0x05, 0x06,  0x07
};


/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
	int i;
	unsigned char b;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    _asm(" FSET I ");                   /* 全体の割り込み許可           */
    initBeepS();                        /* ブザー関連処理               */
	init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
	
    /* マイコンカーの状態初期化 */
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );
    motor2_f( 0, 0 );
    motor2_r( 0, 0 );
    servoPwmOut( 0 );
	fullColor_out( 0 );
    setBeepPatternS( 0x8000 );
	servoSet( 0 );
	
    while( 1 ) {
	servoControl();
	if( pattern >= 11 && pattern < 100 ) {
		
		/* 時間による停止(1分) */
		if( cnt_run > RUNNING_TIME ){
			pattern = 101;  // 走行終了
		}	
	}
	
    switch( pattern ) {
    case 0:
        /* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
            setBeepPatternS( 0xcc00 );
            cnt1 = 0;
			pattern = 1;
            break;
        }
		
		b = p3_0;
		fullColor_out( b*5 );
		
		if( cnt1 >= 600 ) cnt1 = 0;
		if( cnt1 < 300 ){  // エンコーダの信号をLEDに表示
			led_out( 0x08 );
		}
		else if( cnt1 < 600 ){
		    led_out( 0x04 );
		}
		
		break;

    case 1:
        /* スタンバイ:スタート3秒前 */
			
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
		iAngle0 = getServoAngle();  /* 0度の位置記憶                */
		cnt_run = 0;
		cnt1 = 0;
		anchi_cross = 0;
		check_sen_cnt = 0;
		check_enc_cnt = 0;
		lEncoderTotal = 0;
		lEncoderLine  = 0;
		pattern = 11;
        break;

      case 11:
       	/* 通常トレース */
	   	servoPwmOut( iServoPwm );	
	   	traceMain();      // ライントレース関数		
	    fullColor_out( 0 );	
		servoSet( 0 );
		
	   	if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		 
		if( check_leftline() ) {  /* 左ハーフラインチェック:垂直標的*/
			cnt1 = 0;
			break;
		}
	   
       	if( check_rightline() && hyouteki_flag == 0 ) { /* 右ハーフラインチェック:平行標的*/
			pattern = 70;
			cnt1 = 0;
			servoSet( 1 );	// 平行振るだけ
			break; 
		}
		
		if( check_rightline() && hyouteki_flag == 1 ) { /* 右ハーフラインチェック:垂直標的*/
			pattern = 30;
			cnt1 = 0;
			servoSet( A );	// 垂直A
        	break;
		}
        break;
	
	case 20:
		/* クロスライン検出直後の処理 */
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数		
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
		traceMain();      // ライントレース関数		
	  	if(lEncoderTotal - lEncoderLine >= (273L*2)){  // クロスラインの読み飛ばし
			pattern = 22;
			lEncoderLine = lEncoderTotal;
			cnt1 = 0;	
		}
		break;
	
	case 22:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数		
	  	if( check_rightline() || check_leftline() ){
			servoSet( 0 );
			cnt1 = 0;
			lEncoderLine = lEncoderTotal;
			pattern = 23;
		}
		break;
	
	case 23:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数		
		if( lEncoderTotal - lEncoderLine >= (273L*2)){
			pattern = 11;
			cnt1 = 0;
			servoSet( 0 );
		} 
		break;
	
	case 30:
		/* 垂直標的(A) */
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	fullColor_out( 1 );
		servoSet( A );
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 31;
		break;
	
	case 31:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*2)){
			//pattern = 11;
			pattern = 32;
			lEncoderLine = lEncoderTotal;	
		}
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		break;
	
	case 32:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*2) && check_leftline()){
			//pattern = 11;
			pattern = 40;	
		}
		break;
	
	case 40:
		/* 垂直標的(B) */
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
	  	fullColor_out( 2 );
		servoSet( B );
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 41;
		break;
	
	case 41:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*2)){
			lEncoderLine = lEncoderTotal;
			pattern = 42;	
		}
		break;
	
	case 42:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*3) && check_rightline()){
			//pattern = 11;
			pattern = 50;	
		}
		 if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		break;
	
	case 50:
		/* 垂直標的(C) */
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
	  	fullColor_out( 3 );
		servoSet( C );
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 51;
		break;
	
	case 51:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
		if( lEncoderTotal - lEncoderLine >= (273L*2)){
			lEncoderLine = lEncoderTotal;
			pattern = 52;	
		}
		break;	
	
	case 52:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*3) && check_leftline()){
			//pattern = 11;
			pattern = 60;	
		}
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		break;
	
	case 60:
		/* 垂直標的(D) */
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
	  	fullColor_out( 4 );
		servoSet( D );
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 61;
		break;
	
	case 61:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}	
	  	if( lEncoderTotal - lEncoderLine >= (273L*2)){
			pattern = 62;
			lEncoderLine = lEncoderTotal;
		}
		break;
	
	case 62:
		servoPwmOut( iServoPwm );	
		traceMain();      // ライントレース関数	
	  	if( lEncoderTotal - lEncoderLine >= (273L*3)){
			if( check_rightline() || check_leftline() ){  /* カーブチェック */
				cnt1 = 0;
				pattern = 20;
		 	}		
		}
		break;
	
	case 70:
		/* 平行標的 */
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
		servoSet( 1 );    // 近づけるだけ
		setBeepPatternS( 0x8000 );
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 71;
		break;	
	
	case 71:
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		if( lEncoderTotal - lEncoderLine >= (273L*1) ){
			pattern = 72;
			lEncoderLine = lEncoderTotal;
			cnt1 = 0;		
		}
		break;
	
	case 72:
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		if( lEncoderTotal - lEncoderLine >= UP4BIT ){
			pattern = 73;
			lEncoderLine = lEncoderTotal;
			cnt1 = 0;		
		}
		break;
	
	case 73:
		servoPwmOut( iServoPwm );
		traceMain();      // ライントレース関数		
		servoSet( 2 );    // 近づけるだけ
		setBeepPatternS( 0xc000 );
		if( check_crossline() ){  /* カーブチェック */
			cnt1 = 0;
			pattern = 20;
			break;
	 	}
		if( lEncoderTotal - lEncoderLine >= MIN4BIT){  // 当ててから通過するまでのエンコーダ値
			heikou++;
			if( heikou > 2 ){
				heikou = 0;
				cnt1 = 0;
				pattern = 20;
				break;
			}
			else{
				pattern = 11;
			}
		}
		break;

	  case 101:
        /* 停止処理 */
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
 	    /* 停止処理 */
        servoPwmOut( iServoPwm );
        if( cnt1 > 1000 ) {
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
		/* 停止処理 */
        servoPwmOut( iServoPwm );
		motor_mode_f( FREE, FREE );
		motor_mode_r( FREE, FREE );
		motor2_f( 0, 0 );
        motor2_r( 0, 0 );
        if( cnt1 > 500 ) {
            pattern = 104;
            cnt1 = 0;
            break;
        }
        break;

    case 104:
        /* 何もしない */
		servoPwmOut( 0 );
		servoSet( 0 );    // 近づけるだけ
		motor_mode_f( BRAKE, BRAKE );
		motor_mode_r( BRAKE, BRAKE );
		motor2_f( 0, 0 );
       	motor2_r( 0, 0 );
		
		for( i = 1;i < 8; i++ ){
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
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void )
{
	int i;
    
	init_xin_clk();  // レジスタの初期化
	
	/* クロックをXINクロック(20MHz)に変更 */
    prc0  = 1;                          /* プロテクト解除               */
    cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
    cm05  = 0;                          /* XINクロック発振              */
    for(i=0; i<50; i++ );               /* 安定するまで少し待つ(約10ms) */
    ocd2  = 0;                          /* システムクロックをXINにする  */
    prc0  = 0;
    
	/* ポートの入出力設定 */

    /*  PWM(予備)       左前M_PMW       右前M_PWM       ブザー
        センサ左端      センサ左中      センサ右中      センサ右端  */
    p0   = 0x00;
    prc2 = 1;                           /* PD0のプロテクト解除          */
    pd0  = 0xf0;

    /*  センサ中心      ｽﾀｰﾄﾊﾞｰ         RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0         */
    pur0 |= 0x04;                       /* P1_3?P1_0のプルアップON     */
    p1  = 0x00;
    pd1 = 0x10;

    /*  右前M_方向      ステアM_方向    ステアM_PWM     右後M_PWM
        右後M_方向      左後M_PWM       左後M_方向      左前M_方向      */
    p2  = 0x00;
    pd2 = 0xff;
	
	/* !---追加・変更---! */
    /*  Arduino(ANGLE)  none            none            none
        none            none            none            エンコーダA相   */
    p3  = 0x00;
    pd3 = 0xfe;

    /*  XOUT            XIN             ボード上のLED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;

    /*  none            none            none            none
        none            none            none            none            */
    p5  = 0x00;
    pd5 = 0xff;

    /*  none            none            none            none
        none            none            Arduino(ZERO)   Arduino(MODE)   */
    p6  = 0x00;
    pd6 = 0xff;

    /*  CN6.2入力       CN6.3入力       CN6.4入力       CN6.5入力
        none(ｱﾅﾛｸﾞ予備) 角度VR          センサ_左ｱﾅﾛｸﾞ  センサ_右ｱﾅﾛｸﾞ  */
    p7  = 0x00;
    pd7 = 0x00;

    /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
        DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
    pur2 |= 0x03;                       /* P8_7?P8_0のプルアップON      */
    p8  = 0x00;
    pd8 = 0x00;

    /*  -               -               ﾌﾟｯｼｭｽｲｯﾁ       P8制御(LEDorSW)
        右前M_Free      左前M_Free      右後M_Free      左後M_Free      */
    p9  = 0x00;
    pd9 = 0x1f;
    pu23 = 1;   // P9_4,P9_5をプルアップする
	
    /* タイマRBの設定 */
    /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* 動作モード、分周比設定       */
    trbpre = 200-1;                     /* プリスケーラレジスタ         */
    trbpr  = 100-1;                     /* プライマリレジスタ           */
    trbic  = 0x06;                      /* 割り込み優先レベル設定       */
    trbcr  = 0x01;                      /* カウント開始                 */

    /* A/Dコンバータの設定 */
    admod   = 0x33;                     /* 繰り返し掃引モードに設定     */
    adinsel = 0x90;                     /* 入力端子P7の4端子を選択      */
    adcon1  = 0x30;                     /* A/D動作可能                  */
    _asm(" NOP ");                      /* φADの1サイクルウエイト入れる*/
    adcon0  = 0x01;                     /* A/D変換スタート              */

   /* タイマRG タイマモード(両エッジでカウント)の設定 */
    timsr = 0x40;                       /* TRGCLKA端子 P3_0に割り当てる */
    trgcr = 0x15;                       /* TRGCLKA端子の両エッジでカウント*/
    trgmr = 0x80;                       /* TRGのカウント開始            */

    /* タイマRC PWMモード設定(左前モータ、右前モータ) */
    trcpsr0 = 0x40;                     /* TRCIOA,B端子の設定           */
    trcpsr1 = 0x33;                     /* TRCIOC,D端子の設定           */
    trcmr   = 0x0f;                     /* PWMモード選択ビット設定      */
    trccr1  = 0x8e;                     /* ｿｰｽｶｳﾝﾄ:f1,初期出力の設定    */
    trccr2  = 0x00;                     /* 出力レベルの設定             */
    trcgra  = TRC_MOTOR_CYCLE - 1;      /* 周期設定                     */
    trcgrb  = trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
    trcgrc  = trcgra;                   /* P0_7端子のON幅(予備)         */
    trcgrd  = trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
    trcic   = 0x07;                     /* 割り込み優先レベル設定       */
    trcier  = 0x01;                     /* IMIAを許可                   */
    trcoer  = 0x01;                     /* 出力端子の選択               */
    trcmr  |= 0x80;                     /* TRCカウント開始              */

    /* タイマRD リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ、ｻｰﾎﾞﾓｰﾀ) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x20;                     /* ソースカウントの選択:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* 周期設定             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅(左後モータ)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅(右後モータ)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5端子のON幅(サーボモータ) */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/		
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
    unsigned int i;

    _asm(" FSET I ");                    /* タイマRB以上の割り込み許可   */

	cnt1++;
	cnt_run++;
	check_sen_cnt++;
	check_cross_cnt++;
	stm_cnt++;

    /* サーボモータ制御 */
    servoControl();
	servoControl2();

    /* ブザー処理 */
    beepProcessS();

    /* 10回中1回実行する処理 */
    iTimer10++;
    switch( iTimer10 ) {
    case 1:
        /* エンコーダ制御 */
        i = trg;
    	iEncoder       = i - uEncoderBuff;
	    lEncoderTotal += iEncoder;
		if( iEncoder > iEncoderMax ){
       		iEncoderMax = iEncoder;
		}
	    uEncoderBuff   = i;
        break;

    case 2:
        /* スイッチ読み込み準備 */
        p9_4 = 0;                       /* LED出力OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* スイッチ読み込み、LED出力 */
        types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
        p8  = types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のLEDへ出力*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED出力ON                    */
        break;

    case 4:
		if( pattern == 1 ){
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
        /* iTimer10変数の処理 */
        iTimer10 = 0;
        break;
    }
}

/************************************************************************/
/* タイマRC 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
    trcsr &= 0xfe;	/* フラグクリア */

    /* タイマRC　デューティ比の設定 */
    trcgrb = trcgrb_buff;
    trcgrd = trcgrd_buff;
}

/************************************************************************/
/* カーブ検出処理(進入時のみ)                                           */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
/************************************************************************/
int check_crossline( void )
{
  int ret = 0;
  int angle;

  angle = getServoAngle();

  if ( abs( angle ) <= 2 ) { // ハンドルがほぼ直線のとき
    if ( check_leftline() && check_rightline()) { // 左右のデジタルセンサが反応していればカーブ認識
      ret = 1;
    }
  }

  return ret;
}

/************************************************************************/
/* 右ハーフライン検出処理                        */
/* 引数 なし                              */
/* 戻り値 0:ハーフラインなし 1:あり                  */
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
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
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
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned int sensor_inp(unsigned char sensor)
{    
    return get_ad( sensor );
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2の中心デジタルセンサ読み込み            */
/* 引数　 なし                                                          */
/* 戻り値 中心デジタルセンサ 0:黒 1:白                                  */
/************************************************************************/
unsigned char center_inp( void )
{
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3?P1_0読み込み           */

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
    /* 実際の出力はタイマRB割り込み処理で実施 */
    types_led = led;
}

/************************************************************************/
/* フルカラーLEDの制御					                                */
/* 引数　 点灯パターン			                                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void fullColor_out( unsigned char type )
{
    p6 = ~fullColor_data[ type ];
}


/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
	if( accele_r >= 0 ) {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
    else{
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } 
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
    else{
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } 
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}


/************************************************************************/
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if( mode_r ) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}

/************************************************************************/
/* 前モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if( mode_r ) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{	
    if( pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -pwm ) / 100;
    }
	
}

/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( get_ad(14) - iAngle0 );
}

/************************************************************************/
/* アナログセンサ値取得                                                 */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
    int ret;
	
	//  = 左  -  右
    ret = (get_ad(13)) - get_ad(12);                    /* アナログセンサ情報取得       */
		
    return ret;
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl( void )
{
    int i, iRet, iP, iD;

    i = getAnalogSensor();          /* センサ値取得                 */
	
	//error = getAnalogSensor();
    integral = i + integral * (1/2);
    iRet = KP * i + KI * integral + KD * (i - iSensorBefore);	
	
    /* サーボモータ用PWM値計算 */
//    iP = KP * i;                        /* 比例                         */
//    iD = KD * ( iSensorBefore - i );    /* 微分(目安はPの5?10倍)       */
//    iRet = iP - iD;
    iRet /= 64;

    /* PWMの上限の設定 */
    if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;  /* マイコンカーが安定したら     */
    if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX;  /* 上限を90くらいにしてください */
    iServoPwm = iRet;

    iSensorBefore = i;                  /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ制御 角度指定用											*/
/* 引数 なし 															*/
/* 戻り値 グローバル変数 iServoPwm2 に代入 								*/
/************************************************************************/
void servoControl2( void )
{
	int i, j, iRet, iP, iD;
	
	i = iSetAngle; 						/* 設定したい角度 	*/
	j = getServoAngle(); 				/* 現在の角度 		*/
	
	/* サーボモータ用PWM値計算 */
	iP = KP * (j - i); 					/* 比例 */
	iD = KD * (iAngleBefore2 - j); 		/* 微分 */
	
	iRet = iP - iD;
	iRet /= 2;
	
	if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;	/* マイコンカーが安定したら 	*/
	if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX; 	/* 上限を90くらいにしてください */
	
	iServoPwm2 = iRet;
	iAngleBefore2 = j;
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
    int i, ret;

    i  = getServoAngle() / AD_1DEG;           /* 1度あたりの増分で割る        */
    if( i <  0 ) i = -i;
    if( i > 45 ) i = 45;
    ret = revolution_difference[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* 10進数→BCD値														*/
/* 引数    BCDにしたい値(int型)											*/
/* 戻り値  BCD値														*/
/* メモ	   BCD符号化範囲は0?99まで										*/
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
/* ライントレース関数本体                                               */
/* 引数   デューティ比(0?100)                                          */
/* 戻り値 なし                                                          */  
/* 備考 機体に合わせてパラメータを書き換えてください                    */
/************************************************************************/
void traceMain( void )
{
	int i;
		i = getServoAngle();
        if( i > 170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( diff(20), 20 );
            motor_r( diff(20), 20 );
        } else if( i > 25 ) {
      		motor_mode_f( FREE, BRAKE );
      		motor_mode_r( FREE, BRAKE );
			motor_f( diff(50), 50 );
            motor_r( diff(50), 50 );
        } else if( i < -170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( 20, diff(20) );
            motor_r( 20, diff(20) );
        } else if( i < -25 ) {
			motor_mode_f( BRAKE, FREE );
      		motor_mode_r( BRAKE, FREE );
			motor_f( 50, diff(50) );
            motor_r( 50, diff(50) );
        } else {	
      		motor_mode_f( FREE, FREE );
      		motor_mode_r( FREE, FREE );
            motor_f( 30, 30 );
            motor_r( 30, 30 );
        }	
	
}
/************************************************************************/
/* 過去の横線の検出回数に応じて標的を見分ける                           */
/* 引数   なし                                                          */
/* 戻り値 変数hyouteki_flagに(平行標的:0 垂直標的:1)が入る              */
/************************************************************************/
void hyouteki_check( void ){
	 hitcount = 1 - hitcount; 
	 switch( hitcount ){
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
/* 数値をある範囲から別の範囲に変換(Arduinoのmap関数と同等)             */
/*                                                                      */
/* 引数   x: 変換したい数値                                             */
/*        in_min: 現在の範囲の下限                                      */
/*        int_max: 現在の範囲の上限                                     */
/*        out_min: 変換後の範囲の下限                                   */
/*        out_max: 変換後の範囲の上限                                   */
/*                                                                      */
/* 戻り値 変換後の数値 (long)                                           */
/************************************************************************/
long map( long x, long in_min, long in_max, long out_min, long out_max ){
	
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
  
}

void servoSet( int num ){
	if( num != 0 ){
		if( num == 1 ){
			SERVO_ANGLE_PORT = 1;
			SERVO_ZERO_PORT  = 0;
			SERVO_MODE_PORT  = 0;	
		}
		if( num == 2 ){
			SERVO_ANGLE_PORT = 0;
			SERVO_ZERO_PORT  = 1;
			SERVO_MODE_PORT  = 0;		
		}
		if( num == 3 ){
			SERVO_ANGLE_PORT = 1;
			SERVO_ZERO_PORT  = 1;
			SERVO_MODE_PORT  = 0;		
		}
		if( num == 4 ){
			SERVO_ANGLE_PORT = 0;
			SERVO_ZERO_PORT  = 0;
			SERVO_MODE_PORT  = 1;		
		}
		if( num == 5 ){
			SERVO_ANGLE_PORT = 1;
			SERVO_ZERO_PORT  = 0;
			SERVO_MODE_PORT  = 1;		
		}
		if( num == 6 ){
			SERVO_ANGLE_PORT = 0;
			SERVO_ZERO_PORT  = 1;
			SERVO_MODE_PORT  = 1;		
		}
	}
	if( (num == 0) || (num >= 7) ){
		SERVO_ANGLE_PORT = 0;
		SERVO_ZERO_PORT  = 0;
		SERVO_MODE_PORT  = 0;	
	}
}