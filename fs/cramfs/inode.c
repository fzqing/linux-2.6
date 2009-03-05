/*
 * Compressed rom filesystem for Linux.
 *
 * Copyright (C) 1999 Linus Torvalds.
 *
 * This file is released under the GPL.
 */

/*
 * These are the VFS interfaces to the compressed rom filesystem.
 * The actual compression is based on zlib, see the other files.
 */

/* Linear Addressing code
 *
 * Copyright (C) 2000 Shane Nay.
 *
 * Allows you to have a linearly addressed cramfs filesystem.
 * Saves the need for buffer, and the munging of the buffer.
 * Savings a bit over 32k with default PAGE_SIZE, BUFFER_SIZE
 * etc.  Usefull on embedded platform with ROM :-).
 *
 * Downsides- Currently linear addressed cramfs partitions
 * don't co-exist with block cramfs partitions.
 *
 */

/*
 * 28-Dec-2000: XIP mode for linear cramfs
 * Copyright (C) 2000 Robert Leslie <rob@mars.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/blkdev.h>
#include <linux/cramfs_fs.h>
#include <linux/slab.h>
#include <linux/cramfs_fs_sb.h>
#include <linux/buffer_head.h>
#include <linux/vfs.h>
#include <asm/semaphore.h>

#include <asm/uaccess.h>
#include <asm/tlbflush.h>

static struct super_operations cramfs_ops;
static struct inode_operations cramfs_dir_inode_operations;
static struct file_operations cramfs_directory_operations;
static struct address_space_operations cramfs_aops;

static DECLARE_MUTEX(read_mutex);


/* These two macros may change in future, to provide better st_ino
   semantics. */
#define CRAMINO(x)	((x)->offset?(x)->offset<<2:1)
#define OFFSET(x)	((x)->i_ino)

 
#if defined(CONFIG_CRAMFS_XIP_DEBUGGABLE) && defined(CONFIG_PROC_FS)
#include <linux/proc_fs.h>

static struct proc_dir_entry *proc_xip_debug;
int xip_enable_debug;

static int read_xip_debug(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int ret;
	
	ret = sprintf(page + off, "%d\n", xip_enable_debug);
	*eof = 1;
	return ret;
}

static int write_xip_debug(struct file *file, const char *buffer,
			   unsigned long count, void *data)
{
	char buf[32];
	
	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count - 1] = '\0';
	if (!strcmp(buf, "0") || !strcmp(buf, "1"))
		xip_enable_debug = simple_strtoul(buf, NULL, 10);
	return count;
}

#define XIP_PROC_NAME "sys/fs/xip-debug"

static void __init create_cramfs_proc_entry(void)
{
	proc_xip_debug = create_proc_entry(XIP_PROC_NAME, 0644, NULL);
	
	if (proc_xip_debug) {
		proc_xip_debug->read_proc = read_xip_debug;
		proc_xip_debug->write_proc = write_xip_debug;
	}
}

static void __exit remove_cramfs_proc_entry(void)
{
	if (proc_xip_debug)
		remove_proc_entry(XIP_PROC_NAME, NULL);
}

#else

#define create_cramfs_proc_entry() do {} while (0)
#define remove_cramfs_proc_entry() do {} while (0)

#endif

#ifdef CONFIG_CRAMFS_LINEAR_XIP
static int cramfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long address, length;
	struct inode *inode = file->f_dentry->d_inode;
	struct super_block *sb = inode->i_sb;
	struct cramfs_sb_info *sbi = CRAMFS_SB(sb);

	/* this is only used in the case of read-only maps for XIP */

	if (vma->vm_flags & VM_WRITE)
		return generic_file_mmap(file, vma);

	if ((vma->vm_flags & VM_SHARED) && (vma->vm_flags & VM_MAYWRITE))
		return -EINVAL;

	address  = PAGE_ALIGN(sbi->linear_phys_addr + OFFSET(inode));
	address += vma->vm_pgoff << PAGE_SHIFT;

	length = vma->vm_end - vma->vm_start;

	if (length > inode->i_size)
		length = inode->i_size;

	length = PAGE_ALIGN(length);

	/*
	 * Don't dump addresses that are not real memory to a core file.
	 */
	vma->vm_flags |= (VM_IO | VM_XIP);
	flush_tlb_page(vma, address);
	if (remap_page_range(vma, vma->vm_start, address, length,
			     vma->vm_page_prot))
		return -EAGAIN;

#ifdef DEBUG_CRAMFS_XIP
	printk("cramfs_mmap: mapped %s at 0x%08lx, length %lu to vma 0x%08lx"
		", page_prot 0x%08lx\n",
		file->f_dentry->d_name.name, address, length,
		vma->vm_start, pgprot_val(vma->vm_page_prot));
#endif

	return 0;
}

static struct file_operations cramfs_linear_xip_fops = {
	read:	generic_file_read,
	mmap:	cramfs_mmap,
};

#define CRAMFS_INODE_IS_XIP(x) ((x)->i_mode & S_ISVTX)

#endif

static struct inode *get_cramfs_inode(struct super_block *sb, struct cramfs_inode * cramfs_inode)
{
	struct inode * inode = new_inode(sb);
	static struct timespec zerotime;

	if (inode) {
		inode->i_mode = cramfs_inode->mode;
		inode->i_uid = cramfs_inode->uid;
		inode->i_size = cramfs_inode->size;
		inode->i_blocks = (cramfs_inode->size - 1) / 512 + 1;
		inode->i_blksize = PAGE_CACHE_SIZE;
		inode->i_gid = cramfs_inode->gid;
		/* Struct copy intentional */
		inode->i_mtime = inode->i_atime = inode->i_ctime = zerotime;
		inode->i_ino = CRAMINO(cramfs_inode);
		/* inode->i_nlink is left 1 - arguably wrong for directories,
		   but it's the best we can do without reading the directory
	           contents.  1 yields the right result in GNU find, even
		   without -noleaf option. */
		insert_inode_hash(inode);
		if (S_ISREG(inode->i_mode)) {
#ifdef CONFIG_CRAMFS_LINEAR_XIP
			inode->i_fop = CRAMFS_INODE_IS_XIP(inode) ? &cramfs_linear_xip_fops : &generic_ro_fops;
#else
			inode->i_fop = &generic_ro_fops;
#endif
			inode->i_data.a_ops = &cramfs_aops;
		} else if (S_ISDIR(inode->i_mode)) {
			inode->i_op = &cramfs_dir_inode_operations;
			inode->i_fop = &cramfs_directory_operations;
		} else if (S_ISLNK(inode->i_mode)) {
			inode->i_op = &page_symlink_inode_operations;
			inode->i_data.a_ops = &cramfs_aops;
		} else {
			inode->i_size = 0;
			init_special_inode(inode, inode->i_mode,
				old_decode_dev(cramfs_inode->size));
		}
	}
	return inode;
}

#ifdef CONFIG_CRAMFS_LINEAR
/*
 * Return a pointer to the block in the linearly addressed cramfs image.
 */
static void *cramfs_read(struct super_block *sb, unsigned int offset, unsigned int len)
{
	struct cramfs_sb_info *sbi = CRAMFS_SB(sb);

	if (!len)
		return NULL;

	if ((offset + len > sbi->size) || (offset > sbi->size) || (len > sbi->size))
		return NULL;

	return (void*)(sbi->linear_virt_addr + offset);
}
#else /* Not linear addressing - aka regular block mode. */
/*
 * We have our own block cache: don't fill up the buffer cache
 * with the rom-image, because the way the filesystem is set
 * up the accesses should be fairly regular and cached in the
 * page cache and dentry tree anyway..
 *
 * This also acts as a way to guarantee contiguous areas of up to
 * BLKS_PER_BUF*PAGE_CACHE_SIZE, so that the caller doesn't need to
 * worry about end-of-buffer issues even when decompressing a full
 * page cache.
 */
#define READ_BUFFERS (2)
/* NEXT_BUFFER(): Loop over [0..(READ_BUFFERS-1)]. */
#define NEXT_BUFFER(_ix) ((_ix) ^ 1)

/*
 * BLKS_PER_BUF_SHIFT should be at least 2 to allow for "compressed"
 * data that takes up more space than the original and with unlucky
 * alignment.
 */
#define BLKS_PER_BUF_SHIFT	(2)
#define BLKS_PER_BUF		(1 << BLKS_PER_BUF_SHIFT)
#define BUFFER_SIZE		(BLKS_PER_BUF*PAGE_CACHE_SIZE)

static unsigned char read_buffers[READ_BUFFERS][BUFFER_SIZE];
static unsigned buffer_blocknr[READ_BUFFERS];
static struct super_block * buffer_dev[READ_BUFFERS];
static int next_buffer;

/*
 * Returns a pointer to a buffer containing at least LEN bytes of
 * filesystem starting at byte offset OFFSET into the filesystem.
 */
static void *cramfs_read(struct super_block *sb, unsigned int offset, unsigned int len)
{
	struct address_space *mapping = sb->s_bdev->bd_inode->i_mapping;
	struct page *pages[BLKS_PER_BUF];
	unsigned i, blocknr, buffer, unread;
	unsigned long devsize;
	char *data;

	if (!len)
		return NULL;
	blocknr = offset >> PAGE_CACHE_SHIFT;
	offset &= PAGE_CACHE_SIZE - 1;

	/* Check if an existing buffer already has the data.. */
	for (i = 0; i < READ_BUFFERS; i++) {
		unsigned int blk_offset;

		if (buffer_dev[i] != sb)
			continue;
		if (blocknr < buffer_blocknr[i])
			continue;
		blk_offset = (blocknr - buffer_blocknr[i]) << PAGE_CACHE_SHIFT;
		blk_offset += offset;
		if (blk_offset + len > BUFFER_SIZE)
			continue;
		return read_buffers[i] + blk_offset;
	}

	devsize = mapping->host->i_size >> PAGE_CACHE_SHIFT;

	/* Ok, read in BLKS_PER_BUF pages completely first. */
	unread = 0;
	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = NULL;

		if (blocknr + i < devsize) {
			page = read_cache_page(mapping, blocknr + i,
				(filler_t *)mapping->a_ops->readpage,
				NULL);
			/* synchronous error? */
			if (IS_ERR(page))
				page = NULL;
		}
		pages[i] = page;
	}

	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = pages[i];
		if (page) {
			wait_on_page_locked(page);
			if (!PageUptodate(page)) {
				/* asynchronous error */
				page_cache_release(page);
				pages[i] = NULL;
			}
		}
	}

	buffer = next_buffer;
	next_buffer = NEXT_BUFFER(buffer);
	buffer_blocknr[buffer] = blocknr;
	buffer_dev[buffer] = sb;

	data = read_buffers[buffer];
	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = pages[i];
		if (page) {
			memcpy(data, kmap(page), PAGE_CACHE_SIZE);
			kunmap(page);
			page_cache_release(page);
		} else
			memset(data, 0, PAGE_CACHE_SIZE);
		data += PAGE_CACHE_SIZE;
	}
	return read_buffers[buffer] + offset;
}
#endif /* CONFIG_CRAMFS_LINEAR */

static void cramfs_put_super(struct super_block *sb)
{
	kfree(sb->s_fs_info);
	sb->s_fs_info = NULL;
}

static int cramfs_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_RDONLY;
	return 0;
}

static int cramfs_fill_super(struct super_block *sb, void *data, int silent)
{
#ifndef CONFIG_CRAMFS_LINEAR
	int i;
#else
	char *p;
#endif
	struct cramfs_super super;
	unsigned long root_offset;
	struct cramfs_sb_info *sbi;
	struct inode *root;

	sb->s_flags |= MS_RDONLY;

	sbi = kmalloc(sizeof(struct cramfs_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;
	sb->s_fs_info = sbi;
	memset(sbi, 0, sizeof(struct cramfs_sb_info));

#ifndef CONFIG_CRAMFS_LINEAR
	/* Invalidate the read buffers on mount: think disk change.. */
	down(&read_mutex);
	for (i = 0; i < READ_BUFFERS; i++)
		buffer_blocknr[i] = -1;

#else /* CONFIG_CRAMFS_LINEAR */
	/*
	 * The physical location of the cramfs image is specified as
	 * a mount parameter.  This parameter is mandatory for obvious
	 * reasons.  Some validation is made on the phys address but this
	 * is not exhaustive and we count on the fact that someone using
	 * this feature is supposed to know what he/she's doing.
	 */
	if (!data || !(p = strstr((char *)data, "physaddr="))) {
		printk(KERN_ERR "cramfs: unknown physical address for linear cramfs image\n");
		goto out;
	}
	sbi->linear_phys_addr = simple_strtoul(p + 9, NULL, 0);
	if (sbi->linear_phys_addr & (PAGE_SIZE-1)) {
		printk(KERN_ERR "cramfs: physical address 0x%lx for linear cramfs isn't aligned to a page boundary\n",
		       sbi->linear_phys_addr);
		goto out;
	}
	if (sbi->linear_phys_addr == 0) {
		printk(KERN_ERR "cramfs: physical address for linear cramfs image can't be 0\n");
		goto out;
	}
	printk(KERN_INFO "cramfs: checking physical address 0x%lx for linear cramfs image\n",
	       sbi->linear_phys_addr);

	/* Map only one page for now.  Will remap it when fs size is known. */
	sbi->size = PAGE_SIZE;
	sbi->linear_virt_addr =
		ioremap(sbi->linear_phys_addr, sbi->size);
	if (!sbi->linear_virt_addr) {
		printk(KERN_ERR "cramfs: ioremap of the linear cramfs image failed\n");
		goto out;
	}

	down(&read_mutex);
#endif /* CONFIG_CRAMFS_LINEAR */

	/* Read the first block and get the superblock from it */
	memcpy(&super, cramfs_read(sb, 0, sizeof(super)), sizeof(super));
	up(&read_mutex);

	/* Do sanity checks on the superblock */
	if (super.magic != CRAMFS_MAGIC) {
		/* check at 512 byte offset */
		down(&read_mutex);
		memcpy(&super, cramfs_read(sb, 512, sizeof(super)), sizeof(super));
		up(&read_mutex);
		if (super.magic != CRAMFS_MAGIC) {
			if (!silent)
				printk(KERN_ERR "cramfs: wrong magic\n");
			goto out;
		}
	}

	/* get feature flags first */
	if (super.flags & ~CRAMFS_SUPPORTED_FLAGS) {
		printk(KERN_ERR "cramfs: unsupported filesystem features\n");
		goto out;
	}

	/* Check that the root inode is in a sane state */
	if (!S_ISDIR(super.root.mode)) {
		printk(KERN_ERR "cramfs: root is not a directory\n");
		goto out;
	}
	root_offset = super.root.offset << 2;
	if (super.flags & CRAMFS_FLAG_FSID_VERSION_2) {
		sbi->size=super.size;
		sbi->blocks=super.fsid.blocks;
		sbi->files=super.fsid.files;
	} else {
		sbi->size=1<<28;
		sbi->blocks=0;
		sbi->files=0;
	}
	sbi->magic=super.magic;
	sbi->flags=super.flags;
	if (root_offset == 0)
		printk(KERN_INFO "cramfs: empty filesystem");
	else if (!(super.flags & CRAMFS_FLAG_SHIFTED_ROOT_OFFSET) &&
		 ((root_offset != sizeof(struct cramfs_super)) &&
		  (root_offset != 512 + sizeof(struct cramfs_super))))
	{
		printk(KERN_ERR "cramfs: bad root offset %lu\n", root_offset);
		goto out;
	}

	/* Set it all up.. */
	sb->s_op = &cramfs_ops;
	root = get_cramfs_inode(sb, &super.root);
	if (!root)
		goto out;
	sb->s_root = d_alloc_root(root);
	if (!sb->s_root) {
		iput(root);
		goto out;
	}

#ifdef CONFIG_CRAMFS_LINEAR
	/* Remap the whole filesystem now */
	iounmap(sbi->linear_virt_addr);
	printk(KERN_INFO "cramfs: linear cramfs image appears to be %lu KB in size\n",
	       sbi->size/1024);
#ifdef CONFIG_ARM
	sbi->linear_virt_addr =
		__ioremap(sbi->linear_phys_addr, sbi->size, L_PTE_CACHEABLE,
			  1);
#else /* CONFIG_ARM */
	sbi->linear_virt_addr =
		ioremap(sbi->linear_phys_addr, sbi->size);
#endif /* CONFIG_ARM */
	if (!sbi->linear_virt_addr) {
		printk(KERN_ERR "cramfs: ioremap of the linear cramfs image failed\n");
		goto out;
	}
#endif /* CONFIG_CRAMFS_LINEAR */
	return 0;
out:
#ifdef CONFIG_CRAMFS_LINEAR
	if (sbi->linear_virt_addr)
		iounmap(sbi->linear_virt_addr);
#endif /* CONFIG_CRAMFS_LINEAR */
	kfree(sbi);
	sb->s_fs_info = NULL;
	return -EINVAL;
}

static int cramfs_statfs(struct super_block *sb, struct kstatfs *buf)
{
	buf->f_type = CRAMFS_MAGIC;
	buf->f_bsize = PAGE_CACHE_SIZE;
	buf->f_blocks = CRAMFS_SB(sb)->blocks;
	buf->f_bfree = 0;
	buf->f_bavail = 0;
	buf->f_files = CRAMFS_SB(sb)->files;
	buf->f_ffree = 0;
	buf->f_namelen = CRAMFS_MAXPATHLEN;
	return 0;
}

/*
 * Read a cramfs directory entry.
 */
static int cramfs_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	struct inode *inode = filp->f_dentry->d_inode;
	struct super_block *sb = inode->i_sb;
	char *buf;
	unsigned int offset;
	int copied;

	/* Offset within the thing. */
	offset = filp->f_pos;
	if (offset >= inode->i_size)
		return 0;
	/* Directory entries are always 4-byte aligned */
	if (offset & 3)
		return -EINVAL;

	buf = kmalloc(256, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	copied = 0;
	while (offset < inode->i_size) {
		struct cramfs_inode *de;
		unsigned long nextoffset;
		char *name;
		ino_t ino;
		mode_t mode;
		int namelen, error;

		down(&read_mutex);
		de = cramfs_read(sb, OFFSET(inode) + offset, sizeof(*de)+256);
		name = (char *)(de+1);

		/*
		 * Namelengths on disk are shifted by two
		 * and the name padded out to 4-byte boundaries
		 * with zeroes.
		 */
		namelen = de->namelen << 2;
		memcpy(buf, name, namelen);
		ino = CRAMINO(de);
		mode = de->mode;
		up(&read_mutex);
		nextoffset = offset + sizeof(*de) + namelen;
		for (;;) {
			if (!namelen) {
				kfree(buf);
				return -EIO;
			}
			if (buf[namelen-1])
				break;
			namelen--;
		}
		error = filldir(dirent, buf, namelen, offset, ino, mode >> 12);
		if (error)
			break;

		offset = nextoffset;
		filp->f_pos = offset;
		copied++;
	}
	kfree(buf);
	return 0;
}

/*
 * Lookup and fill in the inode data..
 */
static struct dentry * cramfs_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd)
{
	unsigned int offset = 0;
	int sorted;

	down(&read_mutex);
	sorted = CRAMFS_SB(dir->i_sb)->flags & CRAMFS_FLAG_SORTED_DIRS;
	while (offset < dir->i_size) {
		struct cramfs_inode *de;
		char *name;
		int namelen, retval;

		de = cramfs_read(dir->i_sb, OFFSET(dir) + offset, sizeof(*de)+256);
		name = (char *)(de+1);

		/* Try to take advantage of sorted directories */
		if (sorted && (dentry->d_name.name[0] < name[0]))
			break;

		namelen = de->namelen << 2;
		offset += sizeof(*de) + namelen;

		/* Quick check that the name is roughly the right length */
		if (((dentry->d_name.len + 3) & ~3) != namelen)
			continue;

		for (;;) {
			if (!namelen) {
				up(&read_mutex);
				return ERR_PTR(-EIO);
			}
			if (name[namelen-1])
				break;
			namelen--;
		}
		if (namelen != dentry->d_name.len)
			continue;
		retval = memcmp(dentry->d_name.name, name, namelen);
		if (retval > 0)
			continue;
		if (!retval) {
			struct cramfs_inode entry = *de;
			up(&read_mutex);
			d_add(dentry, get_cramfs_inode(dir->i_sb, &entry));
			return NULL;
		}
		/* else (retval < 0) */
		if (sorted)
			break;
	}
	up(&read_mutex);
	d_add(dentry, NULL);
	return NULL;
}

static int cramfs_readpage(struct file *file, struct page * page)
{
	struct inode *inode = page->mapping->host;
	u32 maxblock, bytes_filled;
	void *pgdata;

	maxblock = (inode->i_size + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	bytes_filled = 0;
	if (page->index < maxblock) {
		struct super_block *sb = inode->i_sb;
		u32 blkptr_offset = OFFSET(inode) + page->index*4;
		u32 start_offset, compr_len;

#ifdef CONFIG_CRAMFS_LINEAR_XIP
		if(CRAMFS_INODE_IS_XIP(inode)) {
			blkptr_offset = 
				PAGE_ALIGN(OFFSET(inode)) + 
				page->index * PAGE_CACHE_SIZE;
			down(&read_mutex);
			memcpy(page_address(page),
				cramfs_read(sb, blkptr_offset, PAGE_CACHE_SIZE),
				PAGE_CACHE_SIZE);
			up(&read_mutex);
			bytes_filled = PAGE_CACHE_SIZE;
			pgdata = kmap(page);
		} else {
#endif /* CONFIG_CRAMFS_LINEAR_XIP */
		start_offset = OFFSET(inode) + maxblock*4;
		down(&read_mutex);
		if (page->index)
			start_offset = *(u32 *) cramfs_read(sb, blkptr_offset-4, 4);
		compr_len = (*(u32 *) cramfs_read(sb, blkptr_offset, 4) - start_offset);
		up(&read_mutex);
		pgdata = kmap(page);
		if (compr_len == 0)
			; /* hole */
		else {
			down(&read_mutex);
			bytes_filled = cramfs_uncompress_block(pgdata,
				 PAGE_CACHE_SIZE,
				 cramfs_read(sb, start_offset, compr_len),
				 compr_len);
			up(&read_mutex);
		}
#ifdef CONFIG_CRAMFS_LINEAR_XIP
		}
#endif /* CONFIG_CRAMFS_LINEAR_XIP */
	} else
		pgdata = kmap(page);
	memset(pgdata + bytes_filled, 0, PAGE_CACHE_SIZE - bytes_filled);
	kunmap(page);
	flush_dcache_page(page);
	SetPageUptodate(page);
	unlock_page(page);
	return 0;
}

static struct address_space_operations cramfs_aops = {
	.readpage = cramfs_readpage
};

/*
 * Our operations:
 */

/*
 * A directory can only readdir
 */
static struct file_operations cramfs_directory_operations = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.readdir	= cramfs_readdir,
};

static struct inode_operations cramfs_dir_inode_operations = {
	.lookup		= cramfs_lookup,
};

static struct super_operations cramfs_ops = {
	.put_super	= cramfs_put_super,
	.remount_fs	= cramfs_remount,
	.statfs		= cramfs_statfs,
};

static struct super_block *cramfs_get_sb(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
#ifdef CONFIG_CRAMFS_LINEAR
	return get_sb_nodev(fs_type, flags, data, cramfs_fill_super);
#else
	return get_sb_bdev(fs_type, flags, dev_name, data, cramfs_fill_super);
#endif
}

static void cramfs_kill_super(struct super_block *sb)
{
#ifdef CONFIG_CRAMFS_LINEAR
	kill_anon_super(sb);
#else
	kill_block_super(sb);
#endif
}

static struct file_system_type cramfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "cramfs",
	.get_sb		= cramfs_get_sb,
	.kill_sb	= cramfs_kill_super,
#ifndef CONFIG_CRAMFS_LINEAR
	.fs_flags	= FS_REQUIRES_DEV,
#endif /* CONFIG_CRAMFS_LINEAR */
};

static int __init init_cramfs_fs(void)
{
	cramfs_uncompress_init();
	create_cramfs_proc_entry();
	return register_filesystem(&cramfs_fs_type);
}

static void __exit exit_cramfs_fs(void)
{
	remove_cramfs_proc_entry();
	cramfs_uncompress_exit();
	unregister_filesystem(&cramfs_fs_type);
}

module_init(init_cramfs_fs)
module_exit(exit_cramfs_fs)
MODULE_LICENSE("GPL");
