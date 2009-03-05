#ifndef	_FLASHOP_H_
#define	_FLASHOP_H_
	
#include "stddef.h"
	
typedef int (*procref1)(unsigned int base);
typedef int (*procref2)(unsigned int base,int size);
typedef int (*procref3)(unsigned int base);
typedef int (*procref4)(unsigned int adr, char cVal);
typedef int (*procref5)(void);
typedef int (*procref6)(unsigned int base,int size,int verbose);
typedef unsigned int (*procref7)(unsigned int base);
	
typedef struct _Adam2Flash 
{
	procref1 FWBGetBlockSize; /* called to get block size */
	procref6 FWBErase;        /* called to erase blocks */
	procref3 FWBOpen;         /* called to open block writes */
	procref4 FWBWriteByte;    /* called to write block byte */
	procref5 FWBClose;        /* called to close block writes */
	procref2 FWBUnLock;       /* called to close block writes */
	procref2 FWBLock;         /* called to close block writes */
	procref7 FWBGetNextBlock;  /* called to get base of the next blk */
}Adam2Flash;
	
int FWBGetBlockSize(unsigned int base);           /*Get Flash Block Size            */
int FWBErase(unsigned int base,int size,int verbose); /*Erase flash                     */
int FWBOpen(unsigned int base);                   /*Prepare for flash writes        */
int FWBWriteByte(unsigned int adr, char cVal);    /*Write byte to flash             */
int FWBClose(void);                         /*Write any pending data to flash */
int FWBUnLock(unsigned int adr,int size);       /*Unlock Flash block(s)           */
int FWBLock(unsigned int from,int size);        /*Lock Flash block(s)             */

int FWBGet_flash_type(void); /* Valid for dual flash boards */
	
unsigned int FWBGetNextBlock(unsigned int base);  /* Get base addr of next block    */
	
void fix_vector_for_linux(void);
void AppCopyVectors(void);
void init_env(void);
void ShellAddPathToFileName(char *OutBuf,char *InBuf,char *Path);
unsigned int dm(int argc,char *argv[]);
int BadAddress(unsigned int adr);
int boot(void);
int fmt_extrn_dev_str(void);
	
#define SH_CMD_LEN      150
#define SH_ARGC_MAX     10
#define SH_PROMPT_SZ    24
#define PATH_COUNT_MAX  3
#define PRIV_ROOT  1
#define PRIV_USER  0
	
typedef int (*FUNC_PTR)(int, char**);
	
typedef struct SH_PATH {
	char     nm[50];
	char     sz;
} SH_PATH;
	
typedef struct sh_CB {
	char   cmd[SH_CMD_LEN];
	int    argc;
	char  *argv[SH_ARGC_MAX];
	char   path_count;
	SH_PATH  *path;
	char   prompt[SH_PROMPT_SZ + 1];
	unsigned char   priv_state;
} sh_CB;
	
typedef struct PSBL_REC_t {
	unsigned int psbl_size;
	unsigned int env_base;
	unsigned int env_size;
	unsigned int ffs_base;
	unsigned int ffs_size;
	struct sh_CB sh_cb;
	unsigned int ffs2_base;
	unsigned int ffs2_size;
}PSBL_REC;
	
typedef struct sh_cmd {
	const char      *name;
	FUNC_PTR        app_entry_pt;
	unsigned char   privilige;
	const char      *help;
} sh_cmd;
	
typedef enum fs_dev {
	e_Flash = 0,
	e_Sio,
} fs_dev;
	
typedef struct memwindow {
	unsigned int  base;
	unsigned int    end;
} memwindow;
	
void shell(void);
void sh_init(void);
	
#define EMIF_SDRAM_BASE           0x94000000
#define EMIF_SDRAM_MAX_SIZE       0x08000000 /*0x08000000 or 0x01000000*/
#define ASCII_DISP_BASE           0xbc4001c3
#define ASCII_DISP_OFFSET         8
#define SIO_OFFSET     4
#define SIO1_BASE     0xa8610f00
#define SIO1_RSTMASK  0x42
#define SIO0_BASE     0xa8610e00
#define SIO0_RSTMASK  0x01
#define CS0_BASE 0xb0000000
#define CS0_SIZE CONF_FLASH_SZ
#define CS1_BASE 0xb4000000
#define CS1_SIZE CONF_SDRAM_SZ        
#define CS3_BASE 0xbc000000
#define CS3_SIZE (16*1024*1024)
#define CS4_BASE 0xbd000000

#ifdef DUAL_FLASH
#define CS4_SIZE CS0_SIZE
#else
#define CS4_SIZE (16*1024*1024)
#endif
	
#define CS5_BASE 0xbe000000
#define CS5_SIZE (16*1024*1024)

#define EMIF_BASE        0xA8610800
#define EMIF_REV        (*(volatile unsigned int*)(EMIF_BASE+0x00))
#define EMIF_GASYNC     (*(volatile unsigned int*)(EMIF_BASE+0x04))
#define EMIF_DRAMCTL    (*(volatile unsigned int*)(EMIF_BASE+0x08))
#define EMIF_REFRESH    (*(volatile unsigned int*)(EMIF_BASE+0x0c))
#define EMIF_ASYNC_CS0  (*(volatile unsigned int*)(EMIF_BASE+0x10))
#define EMIF_ASYNC_CS3  (*(volatile unsigned int*)(EMIF_BASE+0x14))
#define EMIF_ASYNC_CS4  (*(volatile unsigned int*)(EMIF_BASE+0x18))
#define EMIF_ASYNC_CS5  (*(volatile unsigned int*)(EMIF_BASE+0x1c))

#define GPIO_BASE     0xa8610900
#ifdef TNETV1050_BOARD
/* These defines are for GPIO EN,OUT and DAT 0 */
#define GPIO_EN         (*(volatile unsigned int *)(GPIO_BASE+0x18))
#define GPIO_OUT   		(*(volatile unsigned int *)(GPIO_BASE+0x8))
#define GPIO_DIR   		(*(volatile unsigned int *)(GPIO_BASE+0x10)) 
#else
#define GPIO_EN         (*(volatile unsigned int *)(GPIO_BASE+0xc))
#define GPIO_OUT   		(*(volatile unsigned int *)(GPIO_BASE+0x4))
#define GPIO_DIR   		(*(volatile unsigned int *)(GPIO_BASE+0x8)) 
#endif

#define RESET_BASE 	0xa8611600
#define RESET_REG       (*(volatile unsigned *)(RESET_BASE+0x00))
#define RESET_STATUS    (*(volatile unsigned *)(RESET_BASE+0x08))

#define BOOTCR 		0xa8611a00
	
#ifdef DUAL_FLASH
#define SEC_FLASH_BASE          CS4_BASE
#define SEC_FLASH_SIZE          CS4_SIZE
#define _SEC_FLASH_SIZE         CONF_FLASH_SZ
#endif
	
#if defined (TNETD73XX_BOARD)
#define IS_OHIO_CHIP() (( (REG32_R( 0xA8610914,15,0 ) == 0x2b) || (REG32_R( 0xA8610914,15,0 ) == 0x18)) ? 1:0)
#define AVALANCHE_MII_SEL_REG         (0xa8611A08)
#endif
#endif /* _HW_H_ */
