	/*
 * Copyright (C) 2004, 2005 - MontaVista Software, Inc. (source@mvista.com)
 *
 * This file is released  under the terms of the GNU GPL version 2.
 * This program  is licensed "as is" without any warranty of any kind,
 * whether express or implied.
 */

#ifndef _RELAYFS_TASKLET_H
#define _RELAYFS_TASKLET_H

#include <linux/relayfs_fs.h>

struct tasklet_node {
	struct tasklet_struct tasklet;
	struct list_head task_list;
	struct list_head kmem_list;
	struct tasklet_queue_head *head;
	void *private;
};

#ifdef RELAYFS_TASKLETS_POOL

#define MAX_TASKLETS_ACTIVE	16

static inline int alloc_tasklet_kmem(struct tasklet_queue_head *head)
{
	int i;
	unsigned long flags;
	struct tasklet_node *node, *next;
	
	atomic_set(&head->kmem_count, 0);
	INIT_LIST_HEAD(&head->kmem_list);
	for (i = 0; i < MAX_TASKLETS_ACTIVE; i++) {
		node = kmalloc(sizeof(struct tasklet_node), GFP_KERNEL);
		if (!node)
			goto failed;
		spin_lock_irqsave(&head->lock, flags);
		node->head = head;
		INIT_LIST_HEAD(&node->task_list);
		INIT_LIST_HEAD(&node->kmem_list);
		list_add(&node->kmem_list, &head->kmem_list);
		atomic_inc(&head->kmem_count);
		spin_unlock_irqrestore(&head->lock, flags);
	}
	return 0;
failed:
	list_for_each_entry_safe(node, next, &head->kmem_list, kmem_list) {
		spin_lock_irqsave(&head->lock, flags);
		list_del(&node->kmem_list);
		atomic_dec(&head->kmem_count);
		spin_unlock_irqrestore(&head->lock, flags);
		kfree(node);
	}
	return -ENOMEM;
}

static inline void free_tasklet_queue(struct tasklet_queue_head *head)
{
	unsigned long flags;
	struct tasklet_node *node, *next;

	list_for_each_entry_safe(node, next, &head->kmem_list, kmem_list) {
		spin_lock_irqsave(&head->lock, flags);
		list_del(&node->kmem_list);
		atomic_dec(&head->kmem_count);
		spin_unlock_irqrestore(&head->lock, flags);
		kfree(node);
	}

	list_for_each_entry_safe(node, next, &head->task_list, task_list) {
		tasklet_disable(&node->tasklet);
		spin_lock_irqsave(&head->lock, flags);
		list_del(&node->task_list);
		spin_unlock_irqrestore(&head->lock, flags);
		kfree(node);
	}
}

static inline int alloc_rchan_tasklets(struct rchan *rchan)
{
	int err = 0;

	if ( (err = alloc_tasklet_kmem(&rchan->wake_readers_tasklet)))
		return err;
	if ( (err = alloc_tasklet_kmem(&rchan->wake_writers_tasklet))) {
		free_tasklet_queue(&rchan->wake_readers_tasklet);
		return err;
	}
	if ( (err = alloc_tasklet_kmem(&rchan->work_tasklet))) {
		free_tasklet_queue(&rchan->wake_readers_tasklet);
		free_tasklet_queue(&rchan->wake_writers_tasklet);
		return err;
	}

	return err;
}

static inline void free_rchan_tasklets(struct rchan *rchan)
{
	free_tasklet_queue(&rchan->wake_readers_tasklet);
	free_tasklet_queue(&rchan->wake_writers_tasklet);
	free_tasklet_queue(&rchan->work_tasklet);
}

static inline struct tasklet_node *get_tasklet_kmem(struct tasklet_queue_head *head)
{
	unsigned long flags;
	struct tasklet_node *node = NULL;

	if (atomic_read(&head->kmem_count) == 0)
		return NULL;
	spin_lock_irqsave(&head->lock, flags);
	node = list_entry(head->kmem_list.next, struct tasklet_node, kmem_list);
	list_del_init(&node->kmem_list);
	atomic_dec(&head->kmem_count);
	spin_unlock_irqrestore(&head->lock, flags);

	return node;
}

static inline void put_tasklet_kmem(struct tasklet_node *node, struct tasklet_queue_head *head)
{
	unsigned long flags;

	spin_lock_irqsave(&head->lock, flags);
	list_add_tail(&node->kmem_list, &head->kmem_list);
	atomic_inc(&head->kmem_count);
	spin_unlock_irqrestore(&head->lock, flags);
}

static inline void unlink_tasklet_node(struct tasklet_node *node)
{
	unsigned long flags;

	spin_lock_irqsave(&node->head->lock, flags);
	list_del_init(&node->task_list);
	spin_unlock_irqrestore(&node->head->lock, flags);
}

static inline void relay_init_tasklets(struct rchan *rchan)
{
	INIT_LIST_HEAD(&rchan->wake_readers_tasklet.task_list);
	INIT_LIST_HEAD(&rchan->wake_readers_tasklet.kmem_list);
	INIT_LIST_HEAD(&rchan->wake_writers_tasklet.task_list);
	INIT_LIST_HEAD(&rchan->wake_writers_tasklet.kmem_list);
	INIT_LIST_HEAD(&rchan->work_tasklet.task_list);
	INIT_LIST_HEAD(&rchan->work_tasklet.kmem_list);

	spin_lock_init(&rchan->wake_readers_tasklet.lock);
	spin_lock_init(&rchan->wake_writers_tasklet.lock);
	spin_lock_init(&rchan->work_tasklet.lock);
}

static inline void __relay_tasklet_schedule(struct tasklet_node *node, 
					    void (*func)(unsigned long),
					    unsigned long data,
					    void *private)
{
	unsigned long flags;

	spin_lock_irqsave(&node->head->lock, flags);
	list_add(&node->task_list, &node->head->task_list);
	node->private = private;
	tasklet_init(&node->tasklet, func, data);
	tasklet_schedule(&node->tasklet);
	spin_unlock_irqrestore(&node->head->lock, flags);
}

static inline int relay_tasklet_schedule(struct tasklet_queue_head *queue, 
				void (*func)(unsigned long), void *private)
{
	struct tasklet_node *node;

	if ( (node = get_tasklet_kmem(queue)) == NULL)
		return -ENOMEM;
	__relay_tasklet_schedule(node, func, (unsigned long)node, private);

	return 0;
}

static inline void relay_tasklet_resched(struct tasklet_node *node)
{
	unsigned long flags;

	spin_lock_irqsave(&node->head->lock, flags);
	tasklet_schedule(&node->tasklet);
	spin_unlock_irqrestore(&node->head->lock, flags);
}

static inline int init_and_schedule_tasklet(struct tasklet_queue_head *tq,
				void (*func)(unsigned long), void *private)
{
	int err;
	struct tasklet_node *node;

	INIT_LIST_HEAD(&tq->task_list);
	INIT_LIST_HEAD(&tq->kmem_list);
	spin_lock_init(&tq->lock);

	if ( (err = alloc_tasklet_kmem(tq)) != 0)
		return err;
	if ( (node = get_tasklet_kmem(tq)) == NULL) {
		free_tasklet_queue(tq);
		return -ENOMEM;
	}
	__relay_tasklet_schedule(node, func, (unsigned long)node, private);

	return 0;
}

static inline void init_and_schedule_work(struct work_struct *work, 
				void (*func)(void *), void *private)
{
}

static inline void relay_init_workqueues(struct rchan *rchan)
{
}

#else

static inline int alloc_tasklet_kmem(struct tasklet_queue_head *head)
{
	return 0;
}

static inline int alloc_rchan_tasklets(struct rchan *rchan)
{
	return 0;
}

static inline void free_tasklet_queue(struct tasklet_queue_head *head)
{
}

static inline void free_rchan_tasklets(struct rchan *rchan)
{
}

static inline struct tasklet_node *get_tasklet_kmem(struct tasklet_queue_head *head)
{
	return NULL;
}

static inline void put_tasklet_kmem(struct tasklet_node *node, struct tasklet_queue_head *head)
{
}

static inline void unlink_tasklet_node(struct tasklet_node *node)
{
}

static inline void relay_init_tasklets(struct rchan *rchan)
{
}

static inline void relay_tasklet_resched(struct tasklet_node *node)
{
}

static inline void __relay_tasklet_schedule(struct tasklet_node *node,
					    void (*func)(unsigned long), 
					    unsigned long data,
					    void *private)
{
}

static inline int relay_tasklet_schedule(struct tasklet_queue_head *queue, 
				void (*func)(unsigned long), void *private)
{
	return 0;
}

static inline int init_and_schedule_tasklet(struct tasklet_queue_head *tq,
				void (*func)(unsigned long), void *private)
{
	return 0;
}

static inline void init_and_schedule_work(struct work_struct *work, 
				void (*func)(void *), void *data)
{
	INIT_WORK(work, func, data);
	schedule_delayed_work(work, 1);
}

static inline void relay_init_workqueues(struct rchan *rchan)
{
	INIT_WORK(&rchan->work, NULL, NULL);
}

#endif /* RELAYFS_TASKLETS_POOL */

#endif 
