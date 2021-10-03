#ifndef _ARC_PCI_CFG_H
#define _ARC_PCI_CFG_H

#ifndef SHORT_PACKET
#define	SHORT_PACKET  (256)
#endif

#ifndef _MyID
#define	_MyID  (0xff)     /* アークネットボードのID No. */
#endif

#ifndef _SDID
#define	_SDID  (0xfe)     /* サーボドライバのID No. */
#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xec00)//(0xc400)	/* Acrnet board base address (coral) 2015 3/30 金澤追加 */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0x5000)//(0xc400)	/* Acrnet board base address (jade) 2015 3/30 金澤追加 */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xb000)//(0xc400)	/* Acrnet board base address (finger) 2015 3/30 金澤追加 */
//#endif

#ifndef ARC_ADDR
#define	ARC_ADDR    (0xc800)  /* アークネットボード(PCI3)・ベースアドレス */
#endif

/* #ifndef ARC_ADDR */
/* #define	ARC_ADDR_R  (0xe000)  /\* アークネットボード(PCI2)・ベースアドレス *\/ */
/* #endif */

#ifndef PI
#define	PI  (3.14159265358979323846)
#endif

#ifndef REZ_TO_DEG
#define	REZ_TO_DEG     (1.0/2275.55555)
#endif

#ifndef DEG_TO_RAD
#define	DEG_TO_RAD     (PI/180.0)
#endif

#ifndef TORQ_TO_DIGIT1
#define	TORQ_TO_DIGIT1  ((4096./4.60) / 50.)     /* 0.00112[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT2
#define	TORQ_TO_DIGIT2  ((4096./4.60) / 50.)     /* 0.00112[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT3
#define	TORQ_TO_DIGIT3  ((4096./2.00) / 50.)     /* 0.000488[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT4
#define	TORQ_TO_DIGIT4  ((4096./2.00) / 50.)     /* 0.000488[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT5
#define	TORQ_TO_DIGIT5  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT6
#define	TORQ_TO_DIGIT6  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT7
#define	TORQ_TO_DIGIT7  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TIMEOUT
#define	TIMEOUT	   (-1)
#endif
/*
#ifndef TIME_OUT
#define	TIME_OUT   (65535)
#endif
*/
#ifndef TIME_OUT
#define	TIME_OUT   (655350*2)
#endif
/*
#ifndef TIME_OUT2
#define	TIME_OUT2  (1048575)
#endif
*/
#ifndef TIME_OUT2
#define	TIME_OUT2  (10485750*2)
#endif


/*========== サーボドライバとの通信データフォーマット =======================*/

/*---------- 送信データ ----------*/

typedef struct {
    unsigned short  Status;        /* ステータス */
    short           Torq;          /* トルク指令値(電流値）*/
    short           Speed;         /* 関節速度指令値 */
} C_COM;

typedef struct {
    unsigned char   ChkNo;        /* ｼｰｹﾝｼｬﾙNo. */
    unsigned char   Code;         /* データの種類 */
    C_COM           Drive[7];     /* １～8軸指令値 */
    unsigned short  Do;           /* ﾃﾞｼﾞﾀﾙ出力：8ch */
} SEND_C;

/*---------- 受信データ ----------*/

typedef struct {
    unsigned short  Status;       /* ステータス */
    long            Rez;          /* 関節角度（レゾルバ値）*/
    short           Speed;        /* 現在速度値 */ 
    short           Torq;         /* 現在トルク値 */
} RC_COM;


typedef struct {
    unsigned char   ChkNo;        /* ｼｰｹﾝｼｬﾙNo.	*/
    unsigned char   Code;         /* データの種類 */
    RC_COM          sts[7];       /* 1～8軸ステータス */
    unsigned short  MStatus;     /* Masterステータス */
    unsigned short  Di;           /* ﾃﾞｼﾞﾀﾙ入力：8ch */
    unsigned short  Ai[4];        /* アナログ入力：4ch */
} RECV_C;

#endif  /* _ARC_PCI_CFG_H */
