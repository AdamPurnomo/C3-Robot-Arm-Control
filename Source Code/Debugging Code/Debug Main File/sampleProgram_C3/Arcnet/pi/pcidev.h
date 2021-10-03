#ifndef _PCIDEV_
#define _PCIDEV_

#define BOOL  int
#define FALSE 0
#define TRUE  1
#define DWORD unsigned long
#define WORD  unsigned short
#define BYTE  unsigned char

//デバイスアクセス用構造体
struct PCI_dev_t {
	WORD	ven_id;
	WORD	dev_id;
	WORD	ioaddr[6];		// PCI規格でIOアドレス数は6個と決まっているらしい。
	WORD	reg[16];		//
	WORD	uioaddr;		// 使うIOアドレス
	BYTE	rev_id;			// ボードの個体を識別するためのID	nan 2011/8/24
};

int PCI_Init_new(int dev_num, WORD ven_id, WORD dev_id, char *dev_name, struct PCI_dev_t *buff);

BYTE inbyte (WORD port);

WORD inhword (WORD port);

DWORD inword (WORD port);

void outbyte (WORD port, BYTE value);

void outhword (WORD port, WORD value);

void outword (WORD port, DWORD value);

#endif
