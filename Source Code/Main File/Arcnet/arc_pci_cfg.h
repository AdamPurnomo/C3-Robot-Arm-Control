#ifndef _ARC_PCI_CFG_H
#define _ARC_PCI_CFG_H

#ifndef SHORT_PACKET
#define	SHORT_PACKET  (256)
#endif

#ifndef _MyID
#define	_MyID  (0xff)     /* �A�[�N�l�b�g�{�[�h��ID No. */
#endif

#ifndef _SDID
#define	_SDID  (0xfe)     /* �T�[�{�h���C�o��ID No. */
#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xec00)//(0xc400)	/* Acrnet board base address (coral) 2015 3/30 ���V�ǉ� */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0x5000)//(0xc400)	/* Acrnet board base address (jade) 2015 3/30 ���V�ǉ� */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xb000)//(0xc400)	/* Acrnet board base address (finger) 2015 3/30 ���V�ǉ� */
//#endif

#ifndef ARC_ADDR
#define	ARC_ADDR    (0xc800)  /* �A�[�N�l�b�g�{�[�h(PCI3)�E�x�[�X�A�h���X */
#endif

/* #ifndef ARC_ADDR */
/* #define	ARC_ADDR_R  (0xe000)  /\* �A�[�N�l�b�g�{�[�h(PCI2)�E�x�[�X�A�h���X *\/ */
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


/*========== �T�[�{�h���C�o�Ƃ̒ʐM�f�[�^�t�H�[�}�b�g =======================*/

/*---------- ���M�f�[�^ ----------*/

typedef struct {
    unsigned short  Status;        /* �X�e�[�^�X */
    short           Torq;          /* �g���N�w�ߒl(�d���l�j*/
    short           Speed;         /* �֐ߑ��x�w�ߒl */
} C_COM;

typedef struct {
    unsigned char   ChkNo;        /* ���ݼ��No. */
    unsigned char   Code;         /* �f�[�^�̎�� */
    C_COM           Drive[7];     /* �P�`8���w�ߒl */
    unsigned short  Do;           /* �޼��ُo�́F8ch */
} SEND_C;

/*---------- ��M�f�[�^ ----------*/

typedef struct {
    unsigned short  Status;       /* �X�e�[�^�X */
    long            Rez;          /* �֐ߊp�x�i���]���o�l�j*/
    short           Speed;        /* ���ݑ��x�l */ 
    short           Torq;         /* ���݃g���N�l */
} RC_COM;


typedef struct {
    unsigned char   ChkNo;        /* ���ݼ��No.	*/
    unsigned char   Code;         /* �f�[�^�̎�� */
    RC_COM          sts[7];       /* 1�`8���X�e�[�^�X */
    unsigned short  MStatus;     /* Master�X�e�[�^�X */
    unsigned short  Di;           /* �޼��ٓ��́F8ch */
    unsigned short  Ai[4];        /* �A�i���O���́F4ch */
} RECV_C;

#endif  /* _ARC_PCI_CFG_H */
