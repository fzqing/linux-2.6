/*
 * Copyright (C) 2006 Texas Instruments Inc.
 *
 * Author: Manish Lachwani (mlachwani@mvista.com)
 * Copyright (C) 2006 Montavista Software Inc.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
	
#include "flashop.h"     

#define MAX_ENV_ENTRY 			(block_size/FLASH_ENV_ENTRY_SIZE)
	
unsigned int strlen(const char *str);
int strcmp(const char *A, const char *B);
char *strcpy(char *DST, const char *SRC);
int  EnvInit(void);
char *env_get_base(unsigned int *block_size);
int  sys_unsetenv(const char *);
int  sys_setenv(const char *, const char *);
char *sys_getenv(const char *);
void sh_printenv(void);
void init_env(void);
int sys_defragenv(void);
int sys_initenv(void);
char* sys_getienv(int var_num);
char* sys_getivar (int var_num);
int get_envstring(int index, char *buffer);
	
typedef struct t_env_var {
	char      var;
	char  *   val;
}t_env_var;
	
#ifdef FTP_SERVER_SUPPORT
#ifndef ENV_SPACE_SIZE /* Should be defined by bootcfg.mak file */
#define ENV_SPACE_SIZE      (10 * 1024)
#endif
#endif
	
#define AUTO_DEFRAG_ENVIRONMENT     1
	
typedef enum ENV_VARS {
	env_vars_start = 0,
	CPUFREQ,
	MEMSZ,
	FLASHSZ,
	MODETTY0,
	MODETTY1,
	PROMPT,
	BOOTCFG,
	HWA_0,
#if !defined (AVALANCHE) || defined(TNETC401B)     
	HWA_1,
#endif        
#if !defined(TNETV1020_BOARD)        
	HWA_RNDIS,
#endif    
#if defined (TNETD73XX_BOARD)    
	HWA_3,
#endif    
	IPA,
	IPA_SVR,
	BLINE_MAC0,
#if !defined (AVALANCHE) || defined(TNETC401B)         
	BLINE_MAC1,
#endif    
#if !defined(TNETV1020_BOARD)        
	BLINE_RNDIS,
#endif    
#if defined (TNETD73XX_BOARD)    
	BLINE_ATM,
#endif
#if !defined(TNETV1020_BOARD)            
	USB_PID,
	USB_VID,
	USB_EPPOLLI,
#endif    
	IPA_GATEWAY,
	SUBNET_MASK,
#if defined (TNETV1050_BOARD)    
	BLINE_ESWITCH,
#endif    
#if !defined(TNETV1020_BOARD)            
	USB_SERIAL,
	HWA_HRNDIS,      /* Host (PC) side RNDIS address */
#endif    
	REMOTE_USER,
	REMOTE_PASS,
	REMOTE_DIR,
	SYSFREQ,
	LINK_TIMEOUT,
#ifndef AVALANCHE     /* Avalanche boards use only one mac port */
	MAC_PORT,
#endif    
	PATH,
	HOSTNAME,
#ifdef WLAN
	 HW_REV_MAJOR,
	 HW_REV_MINOR,
	 HW_PATCH,
	 SW_PATCH,
	 SERIAL_NUMBER,
#endif
	TFTPCFG,
#if defined (TNETV1050_BOARD)    
	HWA_ESWITCH,
#endif
	BUILD_OPS,
	TFTP_FO_FNAME,
	TFTP_FO_PORTS,
	CONSOLE_STATE,
	MIPSFREQ,
	/*
	 * Add new env variables here.
	 * NOTE: New environment variables should always be placed at the end, ie 
	 *       just before env_vars_end.
	 */
		
	env_vars_end
} ENV_VARS;
	
typedef struct ENVDESC {
	ENV_VARS   idx;
	const char      *nm;
	char      *alias;
} ENVDESC;
	
#define ENVSTR(x)         #x
#define _ENV_ENTRY(x)  {x, ENVSTR(x), 0}
ENVDESC env_ns[] = {
	_ENV_ENTRY(env_vars_start), /* start. */
	_ENV_ENTRY(CPUFREQ),
	_ENV_ENTRY(MEMSZ),
	_ENV_ENTRY(FLASHSZ),
	_ENV_ENTRY(MODETTY0),
	_ENV_ENTRY(MODETTY1),
	_ENV_ENTRY(PROMPT),
	_ENV_ENTRY(BOOTCFG),
	_ENV_ENTRY(HWA_0),
#if !defined (AVALANCHE) || defined(TNETC401B)    
	_ENV_ENTRY(HWA_1),
#endif
#if !defined(TNETV1020_BOARD)        
	_ENV_ENTRY(HWA_RNDIS),
#endif    
#if defined (TNETD73XX_BOARD)    
	_ENV_ENTRY(HWA_3),
#endif    
	_ENV_ENTRY(IPA),
	_ENV_ENTRY(IPA_SVR),
	_ENV_ENTRY(IPA_GATEWAY),
	_ENV_ENTRY(SUBNET_MASK),
	_ENV_ENTRY(BLINE_MAC0),
#if !defined (AVALANCHE) || defined(TNETC401B)    
	_ENV_ENTRY(BLINE_MAC1),
#endif
#if !defined(TNETV1020_BOARD)    
	_ENV_ENTRY(BLINE_RNDIS),
#endif
#if defined (TNETD73XX_BOARD)    
	_ENV_ENTRY(BLINE_ATM),
#endif
#if !defined(TNETV1020_BOARD)            
	_ENV_ENTRY(USB_PID),
	_ENV_ENTRY(USB_VID),
	_ENV_ENTRY(USB_EPPOLLI),
#endif
#if defined (TNETV1050_BOARD)    
	_ENV_ENTRY(BLINE_ESWITCH),
#endif
#if !defined(TNETV1020_BOARD)            
	_ENV_ENTRY(USB_SERIAL),
	_ENV_ENTRY(HWA_HRNDIS),
#endif    
	_ENV_ENTRY(REMOTE_USER),
	_ENV_ENTRY(REMOTE_PASS),
	_ENV_ENTRY(REMOTE_DIR),
	_ENV_ENTRY(SYSFREQ),
	_ENV_ENTRY(LINK_TIMEOUT),
#ifndef AVALANCHE       /* Avalanche boards use only one mac port */
	_ENV_ENTRY(MAC_PORT),
#endif    
	_ENV_ENTRY(PATH),
	_ENV_ENTRY(HOSTNAME),
#ifdef WLAN
	_ENV_ENTRY(HW_REV_MAJOR),
	_ENV_ENTRY(HW_REV_MINOR),
	_ENV_ENTRY(HW_PATCH),
	_ENV_ENTRY(SW_PATCH),
	_ENV_ENTRY(SERIAL_NUMBER),
#endif
	_ENV_ENTRY(TFTPCFG),
#if defined (TNETV1050_BOARD)    
	_ENV_ENTRY(HWA_ESWITCH),    
#endif
	_ENV_ENTRY(BUILD_OPS),
	_ENV_ENTRY(TFTP_FO_FNAME),
	_ENV_ENTRY(TFTP_FO_PORTS),
	_ENV_ENTRY(CONSOLE_STATE),
	_ENV_ENTRY(MIPSFREQ),   
	/*
	 * Add new entries below this.
	 */
	/* Adam2 environment name alias. Temporary. */
	{IPA,     "my_ipaddress"},
	{CPUFREQ, "cpufrequency"},    
	{SYSFREQ, "sysfrequency"}, 
	{HWA_0,   "maca"},
#ifndef AVALANCHE    
	{HWA_1,   "macb"},
#endif
	{MEMSZ,   "memsize"},
	
	_ENV_ENTRY(env_vars_end) /* delimiter. */
};
	
static unsigned int env_size;
static unsigned int env_base;
	
/* TODO: remove this */
t_env_var env_vars[10];
	
static const char envVersion[] = { "TIENV0.8" };  
	
static const char envErrReadOnly[] = { "Env: %s is read-only.\n" };
	
#define ENV_CELL_SIZE           16
	
/* control field decode */
#define ENV_GARBAGE_BIT			0x01    /* Env is garbage if this bit is off */
#define ENV_DYNAMIC_BIT			0x02    /* Env is dynamic if this bit is off */
#define ENV_PERM_BIT            0x04    /* Env is a backup of a permanent value */
	
typedef struct ENV_VAR_t {
	unsigned char   varNum;
	unsigned char   ctrl;
	unsigned short  chksum;
	unsigned char   numCells;
	unsigned char   data[ENV_CELL_SIZE - 5];    
	/* The data section starts 
	 * here, continues for
	 * numCells.
	 */
}ENV_VAR;
	
	
static unsigned int MaxEnvVarsNum;     
static PSBL_REC* psbl_rec =  (PSBL_REC*)0x94000300;
	
/* Internal functions */
int IsReadOnlyVar( const char* env_nm );
static int IsPreDefinedVar(const char* var); 
static ENVDESC* GetEnvDescForEnvVar(const char* var);
static int __sys_setenv( const char *env_nm, const char *env_val, unsigned char perm );
	
/* Internal macros */
#define IsEnvGarbage(var)       (((var)->ctrl & ENV_GARBAGE_BIT) == 0)
#define IsEnvDynamic(var)       (((var)->ctrl & ENV_DYNAMIC_BIT) == 0)
#define IsEnvPerm(var)          (((var)->ctrl & ENV_PERM_BIT) == 0)
#define EnvGetNextBlock(var)    ((ENV_VAR*)( (char*)(var) + (var)->numCells * ENV_CELL_SIZE))
	
static int EnvMakeGarbage(ENV_VAR* pVar)
{
	int status;
	
	status = FWBOpen((int)&((pVar)->ctrl));
	FWBWriteByte((int)&((pVar)->ctrl), (pVar)->ctrl & ~ENV_GARBAGE_BIT);
	FWBClose();
	
	return (status == 0);
}
	
char* GetEnvBaseAndSize(unsigned int* size)
{
	if(size != 0)
	{
		*size = env_size;
	
#ifdef ENV_SPACE_SIZE
		if(*size > ENV_SPACE_SIZE){
			*size = ENV_SPACE_SIZE;
		}    
#endif
	}
	
	return( (char *) env_base);	
}
	
/* returns the variable number if pre-defined, else return 0 */
static int IsPreDefinedVar(const char* var)
{
	ENVDESC* env;
	
	if((env = GetEnvDescForEnvVar(var)) != 0) 
		return env->idx;
	
	return 0;  
}
	
static ENVDESC* GetEnvDescForEnvVar(const char* var)
{
	int ii;        
	/* go over the list of pre-defined environment variables */
	for (ii = env_vars_start; env_ns[ii].idx != env_vars_end; ii++) {   
		if (strcmp(env_ns[ii].nm, var) == 0) 
		{
			return  &env_ns[ii];
		}
	
		if(env_ns[ii].alias != 0) {
			if (strcmp(env_ns[ii].alias, var) == 0)
			{
				return &env_ns[ii];
			}			
		}
	}
	return 0;
}
	
static char* GetPreDefinedVarName(int index) 
{
	int ii;
	
	if ( index >= env_vars_end || index <= env_vars_start) {
		return 0;
	}
	for(ii = env_vars_start; env_ns[ii].idx != env_vars_end; ii++) {
		if(env_ns[ii].idx == index) {
			if(env_ns[ii].alias != 0) {
				return env_ns[ii].alias;
			} else {                        
				return ( char *)env_ns[ii].nm;
			}
		}
	}
	return 0;
}
	
/* Gives the nth non-garbage environment block. Indexed starting ZERO */
static ENV_VAR* GetEnvBlockByNumber(int index) 
{
	ENV_VAR* pVar;
	int count = 0; 
	unsigned int end_address, size;
	
	pVar = (ENV_VAR*)GetEnvBaseAndSize(&size);
	end_address = ( int ) pVar + size;
	
	/* skip first block */
	pVar++;	
	
	for(;( ( int )pVar < end_address ) && pVar->varNum!=0xFF; pVar = EnvGetNextBlock(pVar))
	{
		if(!IsEnvGarbage(pVar)) {
			if(count == index){
				return pVar; 
			}
			else count++;
		}
	}    
	
	return 0;
}

/*
 * Gets the name and value from a given environment block. Also checks the 
 * checksum while doing so. If it finds that the checksum is invalid, it 
 * marks the environment as garbage, and returns 0 for the name and value.
 */
static void GetNameAndValueFromEnvVar(ENV_VAR* pVar, char** ppName, char** ppValue)
{
	unsigned short chksum = 0;
	int i;
	
	chksum += (pVar->varNum + pVar->ctrl + pVar->numCells);        
	
	if(IsEnvDynamic(pVar)) {
		*ppName  = pVar->data;
		*ppValue = pVar->data + strlen(pVar->data) + 1;
	} else {
		*ppName  = GetPreDefinedVarName(pVar->varNum);
		*ppValue = pVar->data;                
	}        
	
	for(i = 0; i < strlen(*ppValue); i++) {
		chksum += (unsigned char) (*ppValue)[i];
	}
	
	if(IsEnvDynamic(pVar)) {
		for(i = 0; i < strlen(*ppName); i++) {
			chksum += (unsigned char) (*ppName)[i];
		}
	}    
	
	chksum += pVar->chksum;
	
	chksum  = ~(chksum);
	
	/* bad checksum */
	if(chksum != 0) {
		*ppName = *ppValue = 0;            
		EnvMakeGarbage(pVar);
	}                
	
	return;
}
	
/* returns the non-garbage block corresponding to the asked var. */
static ENV_VAR* GetEnvBlockByName(const char *var) 
{
	ENV_VAR* pVar;
	int index = IsPreDefinedVar(var);
	int i;
	
	for(i = 0; i < MaxEnvVarsNum; i++) {
		if( !(pVar = GetEnvBlockByNumber(i)) ) 
			return 0;
		if( (index) || !IsEnvDynamic( pVar ) ) { 
			if(pVar->varNum == index) {
				return pVar;           
			}
		} else {    /* Dynamic environment variables */
			if(!strcmp(var, pVar->data)) {
				return pVar;
			}
		}        
	}
	return 0;
}
	
static int FormatEnvBlock(void) 
{
	unsigned int size, i;
	unsigned char* pFlash = GetEnvBaseAndSize(&size);
	
#ifdef ENV_SPACE_SIZE
	char *pExtraSpace;
	
	if(!(pExtraSpace = vmalloc(env_size - size))) {
		return -1;
	}
	
	memset(pExtraSpace, 0xFF, env_size - size);
	memcpy(pExtraSpace, (char*)env_base + ENV_SPACE_SIZE, 
	env_size - size);    
#endif    
	
	/* If Erase returns 0 => Flash has gone bad. Return error */
	if(FWBErase((unsigned int)pFlash, size, 0) == 0) {
		return -1;
	}
	
	FWBOpen((int)pFlash);
	
	for (i = 0; i <= strlen(envVersion) ;i++) {
	    FWBWriteByte( (int)(pFlash++), envVersion[i]);
	}
	
#ifdef ENV_SPACE_SIZE
	pFlash = (char*)env_base + ENV_SPACE_SIZE;
	for (i = 0; i < env_size - size ;i++) {
		FWBWriteByte( (int)(pFlash++), pExtraSpace[i]);
	}
	vfree(pExtraSpace); 
#endif    
	
	FWBClose();	
	
return 0;
}
	
int EnvInit(void)
{
	static char after_init = 0;
	
	if (after_init)
		return 0;
	else
		after_init = 1;  
	
	env_size = psbl_rec->env_size;
	env_base = psbl_rec->env_base;
	
	MaxEnvVarsNum = (env_size)/(ENV_CELL_SIZE) - 1; /* Ignore the header */
	
	if(strcmp(( char *)env_base, envVersion) != 0) {
		FormatEnvBlock();            
	} 
	
	return 0;        
}
	
int sys_setenv(const char *env_nm, const char *env_val) 
{   
	if( ( !env_nm ) || ( !env_val ) )
		return -1;
	
	if( IsReadOnlyVar( env_nm ) )
	{
		printk(envErrReadOnly, env_nm);
		return 0;       
	}
	return __sys_setenv(env_nm, env_val, 0);
}
	
static int __sys_setenv( const char *env_nm, const char *env_val, unsigned char perm )
{
	ENV_VAR *pVar, new, *pBase, *pOldEnv = 0;
	unsigned int size, i, newsize, end_address;
	char *pTmp;
#if (AUTO_DEFRAG_ENVIRONMENT == 1)
	int IsGarbagePresent = 0;
#endif
	/* Check for pre-existance of the variable */
	if((pTmp = sys_getenv( (char *)env_nm))) 
	{
		/* Env Exists. See if the value to be set is same as old one */
		if(!strcmp(pTmp, env_val)) {
			return 0;
		}
		/* Env Exists but is a different value. Make old one garbage */ 
		pOldEnv = GetEnvBlockByName( (char *)env_nm);
#if (AUTO_DEFRAG_ENVIRONMENT == 1)
		IsGarbagePresent = 1;
#endif
	
		if( IsEnvPerm( pOldEnv ) ) {
			printk(envErrReadOnly, env_nm);
			return 0;       
		}  
	}
	
	pBase = pVar = (ENV_VAR*)GetEnvBaseAndSize(&size);
	end_address = ( unsigned int ) pVar + size;
	
	/* skip first block */
	pVar++;    
	
	/* Go to the end of Available Flash space */
	for( ;( ( int )pVar < end_address ) && pVar->varNum!=0xFF; pVar = EnvGetNextBlock(pVar)) {
#if (AUTO_DEFRAG_ENVIRONMENT == 1)
		if(IsEnvGarbage(pVar)) {
			IsGarbagePresent = 1;
		}
#endif        
	}
	
	memset((char*)&new, 0xFF, sizeof(new));
	new.chksum = 0;                
	
	if(!(new.varNum = IsPreDefinedVar(env_nm))) {       
		/* Dynamic variable */
		new.ctrl &= ~(ENV_DYNAMIC_BIT);
	}
	
#ifdef PERMANENT_VARIABLES
	if( perm )
	new.ctrl &= ~( ENV_PERM_BIT );
#endif
	
	new.chksum += (new.varNum + new.ctrl);
	
	newsize = sizeof(ENV_VAR) - sizeof(new.data) + strlen(env_val) + 1;
	
	if(IsEnvDynamic(&new)) {
		newsize  += strlen(env_nm) + 1;
		for(i=0; i < strlen(env_nm); i++) {
			new.chksum += (unsigned char) env_nm[i];
		}
	}
	
	new.numCells = ((newsize)/ENV_CELL_SIZE)+((newsize%ENV_CELL_SIZE)?1:0);
	new.chksum += new.numCells;
	
	for(i = 0; i <= strlen(env_val); i++) {
		new.chksum += (unsigned char) env_val[i];            
	}            
	
	new.chksum = ~(new.chksum);
	
	/* Check if enough space is available to store the env variable */
	if(((char*)pVar + (new.numCells*ENV_CELL_SIZE)) > ((char*)pBase + size)) {
#if (AUTO_DEFRAG_ENVIRONMENT == 1)
		if(IsGarbagePresent){
			if (sys_defragenv() == 0) {
				return __sys_setenv(env_nm, env_val, perm );
			}
		}
#endif        
		printk("Error: Out of Environment space\n");
		return -2;
	}
	
	/* Write to flash */
	FWBOpen((int)pVar);
	
	for(i = 0; i < sizeof(ENV_VAR) - sizeof(new.data); i++) {
		FWBWriteByte(((int)pVar)++, ((char*)&new)[i]);
	}
	
	if(IsEnvDynamic(&new)) {
		for(i = 0; i <= strlen(env_nm); i++) {
			FWBWriteByte(((int)pVar)++, env_nm[i]);
		}
	}
	
	for(i = 0; i <= strlen(env_val); i++) {
		FWBWriteByte(((int)pVar)++, env_val[i]);
	}        
	
	FWBClose();
	
	if(pOldEnv) {
		EnvMakeGarbage(pOldEnv);
	}
	
	
	return 0;
}
	
int sys_unsetenv(const char *env_nm)
{
	ENV_VAR* pVar;        
	
	if( IsReadOnlyVar( env_nm ) )
		goto fail;
	
	if( !(pVar = GetEnvBlockByName(env_nm)) ) 
		return -1;
	
#ifdef PERMANENT_VARIABLES
	if( !IsEnvPerm( pVar ) )
#endif
	{   
		EnvMakeGarbage(pVar); 
		return 0;  
	}  
	
fail:    
	printk(envErrReadOnly, env_nm);	
	return -1;       
}
	
void echo(int argc, char **argv)
{
	int ii;
	if (argc == 1) 
	{
		sh_printenv();
	} 
	else 
	{
		if (strcmp(argv[1], "envlist") == 0) 
		{
			printk("Pre-defined Variable list:\n\n");                
	
			for ( ii = env_vars_start + 1; ii < env_vars_end; ii++) 
			{
				printk("%-13s\n", GetPreDefinedVarName(ii));
			}
		}
		else 
			/* user gave some unsupported echo request */
			sh_printenv();
	}
}
	
int FWBGet_flash_type(void);
int sys_initenv(void)
{
	static char after_init = 0;
	
	if( after_init )
		return 1;
	else
		after_init = 1;  
	
	env_size = psbl_rec->env_size;
	env_base = psbl_rec->env_base;
	
	FWBGet_flash_type();
	
	MaxEnvVarsNum = (env_size)/(ENV_CELL_SIZE) - 1; /* Ignore the header */
	return 1;
}
	
void sh_printenv(void)
{
	ENV_VAR* pVar;
	int i;
	char *pName, *pValue;
	
	
	for(i = 0; i < MaxEnvVarsNum; i++) {
	
	if( !(pVar = GetEnvBlockByNumber(i)) ) goto out;
		GetNameAndValueFromEnvVar(pVar, &pName, &pValue);
		if(pName == 0) continue;
			printk("\n%-13s\t%s", pName, pValue);
	}	
out:    
	printk("\n");
	return;        
}
	
char* sys_getenv(const char *var)
{
	ENV_VAR* pVar;
	char *pName, *pValue;
	
	if( !(pVar = GetEnvBlockByName(var)) ) 
		return 0;
	
	GetNameAndValueFromEnvVar(pVar, &pName, &pValue);
	
	return pValue;
}
	
int sys_defragenv(void)
{
	char **ppRamStore = 0;
	char *pName, *pValue;
#ifdef PERMANENT_VARIABLES
	char *IsPerm = 0;
#endif
	unsigned int i;
	ENV_VAR* pVar;
	int defragFail=0;
	
	if( !(ppRamStore = (char**)vmalloc(sizeof(char*)*MaxEnvVarsNum)) ) {
		defragFail = 1;                
		goto defragerror;            
	}
#ifdef PERMANENT_VARIABLES
	if( !(IsPerm = (char*)vmalloc( MaxEnvVarsNum )) ) {
		defragFail = 1;                
		goto defragerror;            
	}
#endif
	
	memset ((char*)ppRamStore, 0, sizeof(char*)*MaxEnvVarsNum);
#ifdef PERMANENT_VARIABLES
	memset (IsPerm, 0, MaxEnvVarsNum );
#endif
	for(i = 0; i < MaxEnvVarsNum; i++) {
		if( !(pVar = GetEnvBlockByNumber(i)) ) 
			break;  

			GetNameAndValueFromEnvVar(pVar, &pName, &pValue);        
	
			if(pName == 0) 
				continue;
	
		if( !(ppRamStore[i] = vmalloc(strlen(pName) + strlen(pValue) + 2)) ) {
			defragFail = 1;                
			goto defragerror;            
		}
	
		/* store name and value in RAM */
		memcpy((char*)ppRamStore[i], pName, strlen(pName) + 1);
		memcpy((char*)ppRamStore[i] + strlen(pName) + 1, 
		pValue,strlen(pValue) + 1);
#ifdef PERMANENT_VARIABLES
		IsPerm[ i ] = IsEnvPerm( pVar );                 
#endif
	}
	
	if(FormatEnvBlock() != 0) {
		defragFail = 1;
	}
	
	defragerror:
	if(ppRamStore) {
		for(i = 0; i < MaxEnvVarsNum; i++) {
			if(ppRamStore[i]) {
#ifdef PERMANENT_VARIABLES
				__sys_setenv(ppRamStore[i], 
				ppRamStore[i]+strlen(ppRamStore[i]) + 1, IsPerm[ i ] );
#else
				__sys_setenv(ppRamStore[i], 
				ppRamStore[i]+strlen(ppRamStore[i]) + 1, 0 );
#endif
				vfree(ppRamStore[i]);
			}
		}
		vfree(ppRamStore);
	}
	
#ifdef PERMANENT_VARIABLES
	if( IsPerm )
		vfree(IsPerm);
#endif
	
	if(defragFail) {
		printk("Out of memory. Defragment aborted.\n");
		return -2;
	} 
	
	return 0;
}
	
int get_envstring(int index, char *buffer)
{        
	ENV_VAR* pVar;
	
	if( !(pVar = GetEnvBlockByNumber(index)) ) 	
		return 0;
	
	return sprintf(buffer, "%-13s\t%s\n", 
	sys_getivar(index), sys_getienv(index));
}
	
char* sys_getienv(int var_num)
	{
	ENV_VAR* pVar;
	char* pName, *pValue;
	
	if( !(pVar = GetEnvBlockByNumber(var_num)) ) 
		return 0;
	
	GetNameAndValueFromEnvVar(pVar, &pName, &pValue);
	
	return pValue;
}
	
char* sys_getivar (int var_num) 
{
	ENV_VAR* pVar;
	char* pName, *pValue;
	
	if( !(pVar = GetEnvBlockByNumber(var_num)) ) 
		return 0;
	
	GetNameAndValueFromEnvVar(pVar, &pName, &pValue);
	
	return pName;
}
	
int EnvAddAlias(const char* orig, char* alias)
{
	ENVDESC* env;
	
	if((env = GetEnvDescForEnvVar(orig)) != 0)
	{
		env->alias = alias;
		return 0;
	}
	return -1;
}
	
char* GetResolvedEnvName(const char *envName)
{
	ENVDESC* env;
	
	if((env = GetEnvDescForEnvVar(envName)) != 0)
	{
		if(env->alias != 0) {
			return env->alias;
		} else {
			return ( char *)env->nm;
		}
	}
	return 0;
}
	
int IsReadOnlyVar( const char* env_nm )
{
	if( (strcmp("CPUFREQ", env_nm) == 0) || 
		( strcmp("SYSFREQ", env_nm) == 0 ) || (strcmp("BUILD_OPS", env_nm) == 0) 
		|| (strcmp("MIPSFREQ", env_nm) == 0) )
			return 1;
	
	return 0;
}
