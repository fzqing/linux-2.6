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
 * Interface to the hardware Input Packet Data unit.
 */


#ifndef __CVMX_IPD_H__
#define __CVMX_IPD_H__

/* CSR typedefs have been moved to cvmx-csr-*.h */

typedef cvmx_ipd_mbuff_first_skip_t cvmx_ipd_mbuff_not_first_skip_t;
typedef cvmx_ipd_first_next_ptr_back_t cvmx_ipd_second_next_ptr_back_t;


/**
 * Configure IPD
 *
 * @param mbuff_size Packets buffer size in 8 byte words
 * @param first_mbuff_skip
 *                   Number of 8 byte words to skip in the first buffer
 * @param not_first_mbuff_skip
 *                   Number of 8 byte words to skip in each following buffer
 * @param first_back Must be same as first_mbuff_skip / 128
 * @param second_back
 *                   Must be same as not_first_mbuff_skip / 128
 * @param wqe_fpa_pool
 *                   FPA pool to get work entries from
 * @param cache_mode
 * @param back_pres_enable_flag
 *                   Enable or disable port back pressure
 */
static inline void cvmx_ipd_config(uint64_t mbuff_size,
                                   uint64_t first_mbuff_skip,
                                   uint64_t not_first_mbuff_skip,
                                   uint64_t first_back,
                                   uint64_t second_back,
                                   uint64_t wqe_fpa_pool,
                                   cvmx_ipd_mode_t cache_mode,
                                   uint64_t back_pres_enable_flag
                                  )
{
    cvmx_ipd_mbuff_first_skip_t first_skip;
    cvmx_ipd_mbuff_not_first_skip_t not_first_skip;
    cvmx_ipd_mbuff_size_t size;
    cvmx_ipd_first_next_ptr_back_t first_back_struct;
    cvmx_ipd_second_next_ptr_back_t second_back_struct;
    cvmx_ipd_wqe_fpa_pool_t wqe_pool;
    cvmx_ipd_ctl_status_t ipd_ctl_reg;

    first_skip.u64 = 0;
    first_skip.s.skip_sz = first_mbuff_skip;
    cvmx_write_csr(CVMX_IPD_1ST_MBUFF_SKIP, first_skip.u64);

    not_first_skip.u64 = 0;
    not_first_skip.s.skip_sz = not_first_mbuff_skip;
    cvmx_write_csr(CVMX_IPD_NOT_1ST_MBUFF_SKIP, not_first_skip.u64);

    size.u64 = 0;
    size.s.mb_size = mbuff_size;
    cvmx_write_csr(CVMX_IPD_PACKET_MBUFF_SIZE, size.u64);

    first_back_struct.u64 = 0;
    first_back_struct.s.back = first_back;
    cvmx_write_csr(CVMX_IPD_1st_NEXT_PTR_BACK, first_back_struct.u64);

    second_back_struct.u64 = 0;
    second_back_struct.s.back = second_back;
    cvmx_write_csr(CVMX_IPD_2nd_NEXT_PTR_BACK,second_back_struct.u64);

    wqe_pool.u64 = 0;
    wqe_pool.s.wqe_pool = wqe_fpa_pool;
    cvmx_write_csr(CVMX_IPD_WQE_FPA_QUEUE, wqe_pool.u64);

    ipd_ctl_reg.u64 = cvmx_read_csr(CVMX_IPD_CTL_STATUS);
    ipd_ctl_reg.s.opc_mode = cache_mode;
    ipd_ctl_reg.s.pbp_en = back_pres_enable_flag;
    cvmx_write_csr(CVMX_IPD_CTL_STATUS, ipd_ctl_reg.u64);

#if 0
    /* Disable RED until Octeon Pass 2 - A chip bug causes performance to
        plumet once RED cuts in */

    /* Disable backpressure based on queued buffers. It needs SW support */
    cvmx_ipd_portx_bp_page_cnt_t page_cnt;
    page_cnt.u64 = 0;
    page_cnt.s.bp_enb = 0;
    page_cnt.s.page_cnt = 100;
    int port;
    for (port=0; port<36; port++)
        cvmx_write_csr(CVMX_IPD_PORTX_BP_PAGE_CNT(port), page_cnt.u64);

    /* Set RED to begin dropping packets when there are 1024 buffers left. It
        will linearly drop more packets until reaching 512 buffers */
    cvmx_ipd_qos_red_marks_t red_marks;
    red_marks.u64 = 0;
    red_marks.s.drop = 512;
    red_marks.s.pass = 1024;
    cvmx_write_csr(CVMX_IPD_QOS0_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS1_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS2_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS3_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS4_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS5_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS6_RED_MARKS, red_marks.u64);
    cvmx_write_csr(CVMX_IPD_QOS7_RED_MARKS, red_marks.u64);

    /* Use the actual queue 0 counter, not the average */
    cvmx_ipd_red_quex_param_t red_param;
    red_param.u64 = 0;
    red_param.s.prb_con = (255ul<<24) / (red_marks.s.pass - red_marks.s.drop);
    red_param.s.avg_con = 1;
    red_param.s.new_con = 255;
    red_param.s.use_pcnt = 1;
    cvmx_write_csr(CVMX_IPD_RED_QUE0_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE1_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE2_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE3_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE4_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE5_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE6_PARAM, red_param.u64);
    cvmx_write_csr(CVMX_IPD_RED_QUE7_PARAM, red_param.u64);

    /* Shutoff the dropping based on the per port page count. SW isn't
        decrementing it right now */
    cvmx_write_csr(CVMX_IPD_BP_PRT_RED_END, 0);

    cvmx_ipd_red_port_enable_t red_port_enable;
    red_port_enable.u64 = 0;
    red_port_enable.s.prt_enb = 0xfffffffffull;
    red_port_enable.s.avg_dly = 10000;
    red_port_enable.s.prb_dly = 10000;
    cvmx_write_csr(CVMX_IPD_RED_PORT_ENABLE, red_port_enable.u64);

#endif
}


/**
 * Enable IPD
 */
static inline void cvmx_ipd_enable(void)
{
    cvmx_ipd_ctl_status_t ipd_reg;
    ipd_reg.u64 = cvmx_read_csr(CVMX_IPD_CTL_STATUS);
    if (ipd_reg.s.ipd_en)
        printf("Warning: Enabling IPD when IPD already enabled.\n");
    ipd_reg.s.ipd_en = TRUE;
    cvmx_write_csr(CVMX_IPD_CTL_STATUS, ipd_reg.u64);
}


/**
 * Disable IPD
 */
static inline void cvmx_ipd_disable(void)
{
    cvmx_ipd_ctl_status_t ipd_reg;
    ipd_reg.u64 = cvmx_read_csr(CVMX_IPD_CTL_STATUS);
    ipd_reg.s.ipd_en = FALSE;
    cvmx_write_csr(CVMX_IPD_CTL_STATUS, ipd_reg.u64);
}

#endif  /*  __CVMX_IPD_H__ */
