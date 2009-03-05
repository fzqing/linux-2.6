#ifndef __USER_RMI_PHNX_USER_MAC_H
#define __USER_RMI_PHNX_USER_MAC_H

#include <asm/ioctl.h>

#define USER_MAC_IOC_MAGIC 'M'

#define    USER_MAC_IOC_SRXQ                _IOR(USER_MAC_IOC_MAGIC, 0, unsigned int)//
#define    USER_MAC_IOC_GSHMPHYS            _IOR(USER_MAC_IOC_MAGIC, 1, unsigned int)
#define    USER_MAC_IOC_GSHMVIRT            _IOR(USER_MAC_IOC_MAGIC, 2, unsigned int)
#define    USER_MAC_IOC_GSHMSIZE            _IOR(USER_MAC_IOC_MAGIC, 3, unsigned int)
#define    USER_MAC_IOC_GMMAP_START         _IOR(USER_MAC_IOC_MAGIC, 4, unsigned int)
#define    USER_MAC_IOC_SPERF_Q_THR         _IOR(USER_MAC_IOC_MAGIC, 5, unsigned int)//
#define    USER_MAC_IOC_SPERF_GLOBAL_THR    _IOR(USER_MAC_IOC_MAGIC, 6, unsigned int)//
#define    USER_MAC_IOC_SPERF_EVENT_0       _IOR(USER_MAC_IOC_MAGIC, 7, unsigned int)//
#define    USER_MAC_IOC_SPERF_EVENT_1       _IOR(USER_MAC_IOC_MAGIC, 8, unsigned int)//
#define    USER_MAC_IOC_SBALANCE_NUM_FLOWS  _IOR(USER_MAC_IOC_MAGIC, 9, unsigned int)//
#define    USER_MAC_IOC_SWRITE_REG          _IOR(USER_MAC_IOC_MAGIC, 10, unsigned int)//
#define    USER_MAC_IOC_GREAD_REG           _IOR(USER_MAC_IOC_MAGIC, 11, unsigned int)
#define    USER_MAC_IOC_SPERF               _IOR(USER_MAC_IOC_MAGIC, 12, unsigned int)//
#define    USER_MAC_IOC_GPHYS_CPU_PRESENT_MAP _IOR(USER_MAC_IOC_MAGIC, 13, unsigned int)
#define    USER_MAC_IOC_GCPU_ONLINE_MAP     _IOR(USER_MAC_IOC_MAGIC, 14, unsigned int)

#define PHNX_USER_MAC_CHRDEV_NAME "xlr_user_mac"

#define MAX_USER_MAC_PKTS 3072
#define USER_MAC_FIFO_SIZE 128
#define USER_MAC_PKT_BUF_SIZE 1600

#define USER_XGMAC_STRAIGHT 1
#define USER_XGMAC_CROSSOVER 2

struct ufifo {
  unsigned int data[USER_MAC_FIFO_SIZE];
  int       size;
  volatile int       head;
  volatile int       tail;
};
static __inline__ void ufifo_init (struct ufifo *ufifo)
{
  ufifo->head = ufifo->tail = 0;
  ufifo->size = USER_MAC_FIFO_SIZE;
}

/* TODO: Change all modulos to boolean arithmetic */
static __inline__ int ufifo_next_index(struct ufifo *ufifo, int index)
{
  //return (index+1) % ufifo->size;
  return (index+1) & (USER_MAC_FIFO_SIZE - 1);
}

static __inline__ int  ufifo_next_head(struct ufifo *ufifo) 
{ 
  //return (ufifo->head+1) % ufifo->size ; 
  return (ufifo->head+1) & (USER_MAC_FIFO_SIZE - 1); 
}

static __inline__ int  ufifo_next_tail(struct ufifo *ufifo) 
{ 
  //return (ufifo->tail+1) % ufifo->size ; 
  return (ufifo->tail+1) & (USER_MAC_FIFO_SIZE - 1);
}

static __inline__ int  ufifo_empty(struct ufifo *ufifo) 
{ return (ufifo->head == ufifo->tail); }

static __inline__ int  ufifo_full(struct ufifo *ufifo) 
{ return (ufifo_next_tail(ufifo) == ufifo->head); }

static __inline__ int  ufifo_count(struct ufifo *ufifo) 
{ 
  if (ufifo->head <= ufifo->tail)
    return ufifo->tail - ufifo->head;
  else
    return (ufifo->size - ufifo->head) + (ufifo->tail - 1);
}
static __inline__ int ufifo_dequeue(struct ufifo *ufifo, unsigned int *data)
{
  if (ufifo_empty(ufifo))
    return 0;

   *data = ufifo->data[ufifo->head];
  ufifo->head = ufifo_next_head(ufifo);

  return 1;
}
static __inline__ int ufifo_enqueue(struct ufifo *ufifo, unsigned int data) 
{
  if (ufifo_full(ufifo))
    return 0;

  ufifo->data[ufifo->tail] = data;
  ufifo->tail = ufifo_next_tail(ufifo);

  return 1;
}
static __inline__ int ufifo_head(struct ufifo *ufifo, unsigned int *data) 
{
  if (ufifo_empty(ufifo)) return 0;
  
  *data = ufifo->data[ufifo->head];
  return 1;
}

struct packet_data {
  unsigned char data[USER_MAC_PKT_BUF_SIZE];
};

struct packet_desc {
  unsigned int offset;
  int len;
  int port;
  int type;
  int xgmac; //ignore in gmac. 1 xgmac loopback, 2, xgmac crossover
  int device; //0 xgmac0, 1 xgmac1
  unsigned char priv[48];
  uint64_t priv_ptr;
};

#define USER_MAC_TXQ_FREE 1
#define USER_MAC_TXQ_TX 2

struct user_mac_time {
  unsigned int hi;
  unsigned int lo;
};

#define MAX_USER_KERNEL_APPL 4
#define MAX_USER_KERNEL_SYSCALL 32000
struct user_kernel_appl {
  char syscall_data[MAX_USER_KERNEL_SYSCALL];
};

struct user_mac_data {
  struct packet_data pkt_data[MAX_USER_MAC_PKTS];
  struct packet_desc pkt_desc[MAX_USER_MAC_PKTS];
  struct ufifo rxqs[32];
  struct ufifo txqs[32];
  struct user_mac_time time;
  struct user_kernel_appl appl[MAX_USER_KERNEL_APPL];
};

#ifndef CONFIG_USERSPACE
//Ramesh: API for Load Balancing...06/30/05
   
typedef struct conn_tuple {
        struct hlist_node  conn_node;
        struct hlist_node  timer_node;
        __u8  vcpu_id;
        __u32 src_ip;
        __u32 dst_ip;
        __u32 src_seq;
        __u32 dst_seq;
        __u16 src_port;
        __u16 dst_port;
        __u8  protocol;
        __u8  state_cli;
        __u8  state_ser;
} Conn_tuple;
  
typedef struct thread_stats {
        __u32 flows;
        __u32 packets;
        __u32 bytes;
} Thread_stats;
  

/* TCP FLAGS                */

#define TCP_FLAG_ACK    0x10
#define TCP_FLAG_PUSH   0x08
#define TCP_FLAG_RST    0x04
#define TCP_FLAG_SYN    0x02
#define TCP_FLAG_FIN    0x01
  
#define ICMP_PROTO_NUM  1
#define TCP_PROTO_NUM   6
#define UDP_PROTO_NUM   17

enum {
   TCP_ESTABLISHED = 0,
   TCP_FIN_WAIT_1,
   TCP_FIN_WAIT_2,
   TCP_CLOSING,
   TCP_CLOSE_WAIT,
   TCP_LAST_ACK,
   TCP_TIME_WAIT,
   TCP_CLOSED
};
        
#define CONN_HTABLE_SIZE (2*1024)
#define MAX_CONN_ENTRIES (2*1024)
#define MSL	 	 60
#define FLOW_TIMEOUT	 500000000
#define TICK		 (1000000000/FLOW_TIMEOUT)
#define TIMER_QUEUE_SIZE (TICK*2*MSL)

#endif /* CONFIG_USERSPACE */

#endif
