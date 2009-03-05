/*
 * $Id: pro_ti_titan_mtd.patch,v 1.1.2.1 2006/05/24 19:41:03 mlachwani Exp $
 *
 * Normal mappings of chips in physical memory
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/config.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/mtd/partitions.h>
#include <asm/mach-avalanche/pal.h>

#define MAX_NUM_PARTITIONS 4

#include <asm/mips-boards/prom.h>

struct avalanche_chip_info
{
	char *name;
	int addr;
	int size;
	int buswidth;
};

struct avalanche_mtd
{
	struct avalanche_chip_info *avalanche_chip_info;
	struct mtd_partition avalanche_partition_info[MAX_NUM_PARTITIONS];
	struct mtd_info *avalanche_mtd_info; 
	struct map_info avalanche_map;
	int num_of_partitions;
};

extern char *prom_getenv(char *name);

static int create_mtd_partitions(void);
static void __exit avalanche_mtd_cleanup(void);
	
struct avalanche_chip_info avalanche_chip_info[] = {   
	{ "Primary flash device", CONFIG_MTD_AVALANCHE_CHIP0_START, CONFIG_MTD_AVALANCHE_CHIP0_LEN, CONFIG_MTD_AVALANCHE_CHIP0_BUSWIDTH},
#ifdef CONFIG_MTD_AVALANCHE_SEC_FLASH
	{ "Secondary flash device", CONFIG_MTD_AVALANCHE_CHIP1_START, CONFIG_MTD_AVALANCHE_CHIP1_LEN, CONFIG_MTD_AVALANCHE_CHIP1_BUSWIDTH},
#endif
};

#define NUM_OF_MTD_DEVICES 1

struct avalanche_mtd avalanche_mtd[NUM_OF_MTD_DEVICES];

int avalanche_mtd_ready = 0;

int avalanche_flash_init(struct avalanche_mtd *dev)
{
	struct avalanche_chip_info *chip_info = dev->avalanche_chip_info;
	struct map_info *avalanche_map = &dev->avalanche_map;
	struct mtd_info *avalanche_mtd_info;
        
	avalanche_map->name = chip_info->name;
	avalanche_map->size = chip_info->size;
	avalanche_map->bankwidth = chip_info->buswidth;
	avalanche_map->phys = chip_info->addr;

	printk(KERN_NOTICE "avalanche flash device: 0x%lx at 0x%lx.\n",(unsigned long) chip_info->size, (unsigned long)chip_info->addr);

	avalanche_map->virt = (void *)ioremap_nocache(chip_info->addr, chip_info->size);

	if (!avalanche_map->virt) {
		printk(KERN_ERR "Failed to ioremap avalanche flash device.\n");
		return -EIO;
	}
	
	avalanche_mtd_info = do_map_probe("cfi_probe", avalanche_map);
	if (!avalanche_mtd_info) {
		printk(KERN_ERR "Error in do_map_probe on avalanche flash device.\n");
		return -ENXIO;
	}

	simple_map_init(avalanche_map);
	dev->avalanche_mtd_info = avalanche_mtd_info;	
	avalanche_mtd_info->owner = THIS_MODULE;
	
	return 0;	
}

int __init avalanche_mtd_init(void)
{      
	int i;
	
	for (i = 0; i < NUM_OF_MTD_DEVICES; i++) {
		int ret;

		avalanche_mtd[i].avalanche_chip_info = &avalanche_chip_info[i];
		ret = avalanche_flash_init(avalanche_mtd + i);	
		
		if (ret != 0)
			printk("failed initializing  flash dev %d\n",i);

	}

	create_mtd_partitions(); 
	avalanche_mtd_ready=1;
	ROOT_DEV = MKDEV(MTD_BLOCK_MAJOR, 0);
	return 0;
}

static char *strdup(char *str)
{
	int n = strlen(str)+1;
	char *s = kmalloc(n, GFP_KERNEL);

	if (!s) 
		return NULL;

	return strcpy(s, str);
}

static int avalanche_check_mtd(struct avalanche_mtd *dev, unsigned int flash_base, unsigned int flash_end)
{
	int window_start = dev->avalanche_chip_info->addr;
	int window_size = dev->avalanche_chip_info->size;

	flash_base = CPHYSADDR((void*)flash_base);	
	flash_end  = CPHYSADDR((void*)flash_end);	

	if ( (flash_base >= window_start && flash_base <= (window_start + window_size) )  &&
		 (flash_end >= window_start && flash_end <= (window_start + window_size)) )
        		return 1;

	return 0;
}

static void avalanche_add_partition(char * env_name, char * flash_base, char * flash_end)
{
	int i = 0;
	int found = 0;
	int mtd_partition;
	int offset;
	int size;
		
	struct avalanche_mtd *dev = NULL;
	struct mtd_partition *avalanche_partition_info;
	struct avalanche_chip_info *avalanche_chip_info;
 	struct mtd_info *avalanche_mtd_info;

	for (i = 0; i < NUM_OF_MTD_DEVICES; i++) {
		dev = &avalanche_mtd[i];
		if ( !avalanche_check_mtd(dev,(unsigned int)flash_base, (unsigned int)flash_end))
			continue;

		found = 1;
		break;
	}
                        
 	if (!found) {
                printk("avalanche_check_mtd returns error on partition %s\n", env_name);
		return;
        }

	avalanche_partition_info = dev->avalanche_partition_info;
	avalanche_chip_info = dev->avalanche_chip_info;
	avalanche_mtd_info = dev->avalanche_mtd_info;

	offset = CPHYSADDR(flash_base) - avalanche_chip_info->addr;
	size = flash_end - flash_base;
	
	printk("Found a %s image (0x%x), with size (0x%x).\n",env_name, offset, size);

	mtd_partition = dev->num_of_partitions;
	avalanche_partition_info[mtd_partition].name = strdup(env_name);
	avalanche_partition_info[mtd_partition].offset = offset;
	avalanche_partition_info[mtd_partition].size = size;
	avalanche_partition_info[mtd_partition].mask_flags = 0;
	add_mtd_partitions(avalanche_mtd_info, &avalanche_partition_info[mtd_partition], 1);
	dev->num_of_partitions++;
}

static int create_mtd_partitions(void)
{
	unsigned char *flash_base;
	unsigned char *flash_end;
	char env_ptr[50];
	char *p_env_ptr = env_ptr; /* cant modify array names, you see.. */
	char *base_ptr;
	char *end_ptr;
        int num_of_partitions = 0;

	do {
		char env_name[20];

		sprintf(env_name, "mtd%1u", num_of_partitions);
		printk("Looking for mtd device :%s:\n", env_name);
		if (prom_getenv(env_name) != NULL) {
			strcpy(env_ptr, prom_getenv(env_name));	
			p_env_ptr = env_ptr;
		} else {
			break;
		}
		
		strsep(&p_env_ptr, ",");
		base_ptr = env_ptr; 
		end_ptr = p_env_ptr; 
		if ((base_ptr == NULL) || (end_ptr == NULL)) {	
			printk("ERROR: Invalid %s start,end.\n", env_name);
			break;
		}

		flash_base = (unsigned char*) simple_strtol(base_ptr, NULL, 0);
		flash_end = (unsigned char*) simple_strtol(end_ptr, NULL, 0);
		if ((!flash_base) || (!flash_end)) {
			printk("ERROR: Invalid %s start,end.\n", env_name);
			break;
		}
                
		avalanche_add_partition(env_name,flash_base,flash_end);
		num_of_partitions++;

	} while (num_of_partitions < MAX_NUM_PARTITIONS);

	return num_of_partitions;
}

static void __exit avalanche_mtd_cleanup(void)
{
	int i;

	avalanche_mtd_ready = 0;
	for (i = 0; i < NUM_OF_MTD_DEVICES; i++) {
		struct mtd_info *avalanche_mtd_info = avalanche_mtd[i].avalanche_mtd_info;
		struct map_info *avalanche_map = &avalanche_mtd[i].avalanche_map;		

		if (avalanche_mtd_info) {
			del_mtd_partitions(avalanche_mtd_info);
			del_mtd_device(avalanche_mtd_info);
			map_destroy(avalanche_mtd_info);
		}

		if (avalanche_map->map_priv_1) {
			iounmap((void *)avalanche_map->map_priv_1);
			avalanche_map->map_priv_1 = 0;
		}
	}
}

module_init(avalanche_mtd_init);
module_exit(avalanche_mtd_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Avalanche CFI map driver");

