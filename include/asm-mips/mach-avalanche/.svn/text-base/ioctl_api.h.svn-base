#ifndef _IOCTL_API_H_
#define _IOCTL_API_H_

#define MIB2_TRUTH_VALUE_TRUE  		1
#define MIB2_TRUTH_VALUE_FALSE  	2
#define MIB2_STATUS_UP     	   1
#define MIB2_STATUS_DOWN   	   2
#define MIB2_STATUS_TEST   	   3
#define MIB2_STATUS_UNKNOWN	   4
#define MIB2_STATUS_DORMANT	   5

#ifndef OS_IOCTL_OFFSET 
#define OS_IOCTL_OFFSET 0
#endif

#define TI_SIOCGINTFCOUNTERS     (OS_IOCTL_OFFSET) + 0x01 /* Get Interface counters */
#define TI_SIOCGINTFPARAMS       (OS_IOCTL_OFFSET) + 0x02 /* Get Interface parameters */
#define TI_SIOCSINTFADMINSTATUS  (OS_IOCTL_OFFSET) + 0x03 /* Set Interface Admin. status */
#define TI_SIOCGETHERCOUNTERS     (OS_IOCTL_OFFSET) +0x04 /* Get Emac PHY counters */
#define TI_SIOCGETHERPARAMS       (OS_IOCTL_OFFSET) +0x05 /* Get Emac PHY parameters */

struct mib2_ifCounters
{
   unsigned long inBytesLow;          /* low 32-bit of total octets received from media */
   unsigned long inBytesHigh;         /* high 32-bit of total octets received from media */
   unsigned long inUnicastPktsLow;    /* low 32-bit of unicast packets delivered above */
   unsigned long inUnicastPktsHigh;   /* high 32-bit of unicast packets delivered above */
   unsigned long inMulticastPktsLow;  /* low 32-bit of muticast pkts delivered above */
   unsigned long inMulticastPktsHigh; /* high 32-bit of muticast pkts delivered above */
   unsigned long inBroadcastPktsLow;  /* low 32-bit of broadcast pkts delivered above */
   unsigned long inBroadcastPktsHigh; /* high 32-bit of broadcast pkts delivered above */
   unsigned long inDiscardPkts;       /* packets discarded due to resource limit */
   unsigned long inErrorPkts;         /* packets discarded due to format errors */
   unsigned long inUnknownProtPkts;   /* packets for unknown protocols */
   unsigned long outBytesLow;         /* low 32-bit of total octets sent on the media */
   unsigned long outBytesHigh;        /* high 32-bit of total octets sent on the media */
   unsigned long outUnicastPktsLow;   /* low 32-bit of unicast packets from above */
   unsigned long outUnicastPktsHigh;  /* high 32-bit of unicast packets from above */
   unsigned long outMulticastPktsLow; /* low 32-bit of multicast packets from above */
   unsigned long outMulticastPktsHigh;/* high 32-bit of multicast packets from above */
   unsigned long outBroadcastPktsLow; /* low 32-bit of broadcast packets from above */
   unsigned long outBroadcastPktsHigh;/* high 32-bit of broadcast packets from above */
   unsigned long outDiscardPkts;      /* packets discarded due to resource limit */
   unsigned long outErrorPkts;        /* packets discarded due to format errors */

};
struct mib2_ifHCCounters
{
   struct mib2_ifCounters Mib2IfCounter;
   unsigned long long inBytesHC;
   unsigned long long inUnicastPktsHC;
   unsigned long long inMulticastPktsHC;
   unsigned long long inBroadcastPktsHC;
   unsigned long long outBytesHC;
   unsigned long long outUnicastPktsHC;
   unsigned long long outMulticastPktsHC;   
   unsigned long long outBroadcastPktsHC;   
   unsigned long long inBytes;
   unsigned long long inUnicastPkts;
   unsigned long long inMulticastPkts;
   unsigned long long inBroadcastPkts;
   unsigned long long outBytes;
   unsigned long long outUnicastPkts;
   unsigned long long outMulticastPkts;   
   unsigned long long outBroadcastPkts;   	
};

struct mib2_ifParams
{
    unsigned long ifSpeed;            /* speed of the interface in bits per second */
    unsigned long ifHighSpeed;        /* speed of the interface in mega-bits per second */
    long          ifOperStatus;       /* current operational status */
    long          ifPromiscuousMode;  /* promiscuous mode interface status */
};

struct mib2_ifCommand
{
    long   ifAdminStatus;               /* desired interface state */
};

#define	MIB2_UNKNOWN_DUPLEX     1
#define	MIB2_HALF_DUPLEX 	2
#define	MIB2_FULL_DUPLEX	3

struct mib2_phyCounters
{
   unsigned long ethAlignmentErrors;
   unsigned long ethFCSErrors;
   unsigned long ethSingleCollisions;
   unsigned long ethMultipleCollisions;
   unsigned long ethSQETestErrors;
   unsigned long ethDeferredTxFrames;
   unsigned long ethLateCollisions;
   unsigned long ethExcessiveCollisions;
   unsigned long ethInternalMacTxErrors;
   unsigned long ethCarrierSenseErrors;
   unsigned long ethTooLongRxFrames;
   unsigned long ethInternalMacRxErrors;
   unsigned long ethSymbolErrors;
};

struct mib2_ethParams
{
  long           ethDuplexStatus;        /* current Emac duplex status */
};

typedef struct
{
    unsigned long cmd;
    unsigned long port;
    void *data;
} TI_SNMP_CMD_T;

#define SIOTIMIB2   SIOCDEVPRIVATE + 1

#endif
