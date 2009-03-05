/*************************************************************************
* Author: Cavium Networks info@caviumnetworks.com
*
* 2006 (c) Cavium Networks. This file is licensed under
* the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation. This program
* is licensed "as is" without any warranty of any kind, whether
* express or implied.
* This file may also be available under a different license from
* Cavium Networks.  Contact Cavium Networks for more details.
*************************************************************************/
/**
 * This header file defines the work queue entry (wqe) data structure.
 * Since this is a commonly used structure that depends on structures
 * from several hardware blocks, those definitions have been placed
 * in this file to create a single point of definition of the wqe
 * format.
 * Data structures are still named according to the block that they
 * relate to.
 */


#ifndef __CVMX_WQE_H__
#define __CVMX_WQE_H__

#define OCT_TAG_TYPE_STRING(x) (((x) == CVMX_POW_TAG_TYPE_ORDERED) ?  "ORDERED" : \
                                (((x) == CVMX_POW_TAG_TYPE_ATOMIC) ?  "ATOMIC" : \
                                (((x) == CVMX_POW_TAG_TYPE_NULL) ?  "NULL" : \
                                "NULL_NULL")))

/**
 * HW decode / err_code in work queue entry
 */
typedef union
{
    uint64_t                 u64;

    /** Use this struct if the hardware determines that the packet is IP */
    struct
    {
        uint64_t               bufs          : 8; /**< HW sets this to the number of buffers used by this packet */
        uint64_t               ip_offset     : 8; /**< HW sets to the number of L2 bytes prior to the IP */
        uint64_t               vlan_valid    : 1; /**< set to 1 if we found VLAN in the L2 */
        uint64_t               unassigned    : 2;
        uint64_t               vlan_cfi      : 1; /**< HW sets to the VLAN CFI flag (valid when vlan_valid) */
        uint64_t               vlan_id       :12; /**< HW sets to the VLAN_ID field (valid when vlan_valid) */

        uint64_t               unassigned2   :12;
        uint64_t               dec_ipcomp    : 1; /**< the packet needs to be decompressed */
        uint64_t               tcp_or_udp    : 1; /**< the packet is either TCP or UDP */
        uint64_t               dec_ipsec     : 1; /**< the packet needs to be decrypted (ESP or AH) */
        uint64_t               is_v6         : 1; /**< the packet is IPv6 */

        /* (rcv_error, not_IP, IP_exc, is_frag, L4_error, software, etc.) */

        uint64_t               software      : 1; /**< reserved for software use, hardware will clear on packet creation */
        /* exceptional conditions below */
        uint64_t               L4_error      : 1; /**< the receive interface hardware detected an L4 error (only applies if !is_frag)
                                                    (only applies if !rcv_error && !not_IP && !IP_exc && !is_frag)
                                                    failure indicated in err_code below, decode:
                                                    - 1  = TCP (UDP) packet not long enough to cover TCP (UDP) header
                                                    - 2  = illegal TCP/UDP port (either source or dest port is zero)
                                                    - 3  = TCP/UDP checksum failure
                                                    - 4  = TCP/UDP length check (TCP/UDP length does not match IP length)
                                                    - 8  = TCP flags = FIN only
                                                    - 9  = TCP flags = 0
                                                    - 10 = TCP flags = FIN+RST+*
                                                    - 11 = TCP flags = SYN+URG+*
                                                    - 12 = TCP flags = SYN+RST+*
                                                    - 13 = TCP flags = SYN+FIN+* */
        uint64_t               is_frag       : 1; /**< set if the packet is a fragment */
        uint64_t               IP_exc        : 1; /**< the receive interface hardware detected an IP error / exception
                                                    (only applies if !rcv_error && !not_IP) failure indicated in err_code below, decode:
                                                    - 1 = not IPv4 or IPv6
                                                    - 2 = IPv4 header checksum violation
                                                    - 3 = malformed (packet not long enough to cover IP hdr, or not long enough to cover len in IP hdr)
                                                    - 4 = TTL / hop count equal zero
                                                    - 5 = IPv4 options / IPv6 early extension headers */
        uint64_t               is_bcast      : 1; /**< set if the hardware determined that the packet is a broadcast */
        uint64_t               is_mcast      : 1; /**< set if the hardware determined that the packet is a multi-cast */
        uint64_t               not_IP        : 1; /**< set if the packet may not be IP (must be zero in this case) */
        uint64_t               rcv_error     : 1; /**< the receive interface hardware detected a receive error (must be zero in this case) */
        /* lower err_code = first-level descriptor of the work */
        /* zero for packet submitted by hardware that isn't on the slow path */

        uint64_t               err_code      : 8; /**< type is cvmx_pip_err_t */
    } s;

    /**< use this to get at the 16 vlan bits */
    struct
    {
        uint64_t               unused1       :16;
        uint64_t               vlan          :16;
        uint64_t               unused2       :32;
    } svlan;

    /**< use this struct if the hardware could not determine that the packet is ip */
    struct
    {
        uint64_t               bufs          : 8; /**< HW sets this to the number of buffers used by this packet */
        uint64_t               unused        : 8;
        uint64_t               vlan_valid    : 1; /**< set to 1 if we found VLAN in the L2 */
        uint64_t               unassigned    : 2;
        uint64_t               vlan_cfi      : 1; /**< HW sets to the VLAN CFI flag (valid when vlan_valid) */
        uint64_t               vlan_id       :12; /**< HW sets to the VLAN_ID field (valid when vlan_valid) */

        uint64_t               unassigned2   :16;
        uint64_t               software      : 1; /**< reserved for software use, hardware will clear on packet creation */
        uint64_t               unassigned3   : 1;
        uint64_t               is_rarp       : 1; /**< set if the hardware determined that the packet is rarp */
        uint64_t               is_arp        : 1; /**< set if the hardware determined that the packet is arp */
        uint64_t               is_bcast      : 1; /**< set if the hardware determined that the packet is a broadcast */
        uint64_t               is_mcast      : 1; /**< set if the hardware determined that the packet is a multi-cast */
        uint64_t               not_IP        : 1; /**< set if the packet may not be IP (must be one in this case) */
        uint64_t               rcv_error     : 1; /**< the receive interface hardware detected a receive error.
                                                    Failure indicated in err_code below, decode:
                                                    - 1 = min frame error (pkt len < min frame len)
                                                    - 2 = Frame carrier extend error
                                                    - 3 = max frame error (pkt len > max frame len)
                                                    - 4 = very long frame error - frame is truncated (pkt len > sys frame len)
                                                    - 5 = FCS error (GMX)
                                                    - 6 = nibble error (data not byte multiple - 100M and 10M only)
                                                    - 7 = length mismatch (len did not match len in L2 length/type)
                                                    - 8 = Frame error (some or all bits marked err)
                                                    - 9 = packet was not large enough to pass the skipper - no inspection could occur
                                                    - 10 = studder error (data not repeated - 100M and 10M only)
                                                    - 11 = partially received packet (buffering/bandwidth not adequate)
                                                    - 12 = FCS error (PIP)
                                                    - 15 = packet was not large enough to pass the skipper - no inspection could occur */
        /* lower err_code = first-level descriptor of the work */
        /* zero for packet submitted by hardware that isn't on the slow path */
        uint64_t               err_code       : 8; /* type is cvmx_pip_err_t (union, so can't use directly */
    } snoip;

} cvmx_pip_wqe_word2;








/**
 * Work queue entry format
 *
 * must be 8-byte aligned
 */
typedef struct
{

    /*****************************************************************
     * WORD 0
     *  HW WRITE: the following 64 bits are filled by HW when a packet arrives
     */

    /**
     * raw chksum result generated by the HW
     */
    uint16_t                   hw_chksum;
    /**
     * Field unused by hardware - available for software
     */
    uint8_t                    unused;
    /**
     * Next pointer used by hardware for list maintenance.
     * May be written/read by HW before the work queue
     *           entry is scheduled to a PP
     * (Only 36 bits used in Octeon 1)
     */
    uint64_t                   next_ptr      : 40;


    /*****************************************************************
     * WORD 1
     *  HW WRITE: the following 64 bits are filled by HW when a packet arrives
     */

    /**
     * HW sets to the total number of bytes in the packet
     */
    uint64_t                   len           :16;
    /**
     * HW sets this to input physical port
     */
    uint64_t                   ipprt         : 6;

    /**
     * HW sets this to what it thought the priority of the input packet was
     */
    uint64_t                   qos           : 3;

    /**
     * the group that the work queue entry will be scheduled to
     */
    uint64_t                   grp           : 4;
    /**
     * the type of the tag (ORDERED, ATOMIC, NULL)
     */
    cvmx_pow_tag_type_t        tag_type      : 3;
    /**
     * the synchronization/ordering tag
     */
    uint64_t                   tag           :32;

    /**
     * WORD 2
     *   HW WRITE: the following 64-bits are filled in by hardware when a packet arrives
     *   This indicates a variety of status and error conditions.
     */
    cvmx_pip_wqe_word2       word2;

    /**
     * Pointer to the first segment of the packet.
     */
    cvmx_buf_ptr_t             packet_ptr;

    /**
     *   HW WRITE: octeon will fill in a programmable amount from the
     *             packet, up to (at most, but perhaps less) the amount
     *             needed to fill the work queue entry to 128 bytes
     *   If the packet is recognized to be IP, the hardware starts (except that
     *   the IPv4 header is padded for appropriate alignment) writing here where
     *   the IP header starts.
     *   If the packet is not recognized to be IP, the hardware starts writing
     *   the beginning of the packet here.
     */
    uint8_t packet_data[96];


    /**
     * If desired, SW can make the work Q entry any length. For the
     * purposes of discussion here, Assume 128B always, as this is all that
     * the hardware deals with.
     *
     */

} cvmx_wqe_t  __attribute__ ((aligned (128)));

#endif /* __CVMX_WQE_H__ */
