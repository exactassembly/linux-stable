/*
 * Scsi Target Layer for MPT (Message Passing Technology) based controllers
 *
 * Copyright (C) 2015  Exact Assembly, LLC
 *  (mailto:support@xassembly.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NO WARRANTY
 * THE PROGRAM IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT
 * LIMITATION, ANY WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT,
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. Each Recipient is
 * solely responsible for determining the appropriateness of using and
 * distributing the Program and assumes all risks associated with its
 * exercise of rights under this Agreement, including but not limited to
 * the risks and costs of program errors, damage to or loss of data,
 * programs or equipment, and unavailability or interruption of operations.
 
 * DISCLAIMER OF LIABILITY
 * NEITHER RECIPIENT NOR ANY CONTRIBUTORS SHALL HAVE ANY LIABILITY FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING WITHOUT LIMITATION LOST PROFITS), HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OR DISTRIBUTION OF THE PROGRAM OR THE EXERCISE OF ANY RIGHTS GRANTED
 * HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES
 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/aer.h>

#include "mpt3sas_base.h"
#include "mpt3sas_stm.h"

#define DPRINTK(fmt, args...) if (mpt3stm_debug) printk(KERN_DEBUG "%s: " fmt, \
                                __FUNCTION__ , ## args)

#ifdef VERBOSE
#define VDPRINTK(fmt, args...) if (mpt3stm_debug) printk(KERN_DEBUG "%s: " fmt, \
                                __FUNCTION__ , ## args)
#else
#define VDPRINTK(fmt, args...)
#endif

MODULE_AUTHOR(MPT3STM_AUTHOR);
MODULE_DESCRIPTION(MPT3STM_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_VERSION(MPT3STM_DRIVER_VERSION);

int mpt3stm_debug = 0;
module_param(mpt3stm_debug, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(mpt3stm_debug, " enable debugging output ");

struct _mpt3stm_ta {
    struct _mpt3stm_ioc*    stmioc;
    u16                     IoIndex;
    
    u16                     smid;
    Mpi25TargetAssistRequest_t *mpi_request;
    
    struct scatterlist*     sg;
    int                     nents;
    enum dma_data_direction dir;
    struct list_head        chain_list;
    struct work_struct work_entry; // work_entry used by stmioc->recycleq add/remove
};

struct _mpt3stm_ioc {
    struct MPT3SAS_ADAPTER *ioc;
    void*               priv;   /* Client driver's pointer*/
    MPT3STM_DISPATCH*   dispatch;
    
    u8                  enabled;
    u8                  abort_all;
    
    struct list_head    phy_list;
    
    char                driver_name[MPT_NAME_LENGTH];
    
    u8                  stm_assist_cb_idx;
    
    u8                  stm_tm_cb_idx; /* task managment */
    u8                  stm_tm_imm_cb_idx; /* immediate TM request */
    u8                  stm_abort_cb_idx;
    struct _internal_cmd stm_tm_cmds; /* TM requests */
    
    u8                  stm_post_cb_idx; /* post all buffers */
    struct _internal_cmd stm_post_cmds; /* post cmd buffer request */
    
    u16                 ncmds;
    MPT3STM_CMD_BUFF*   cmdbufs;
    dma_addr_t          cmdbufs_dma;
    
    u16                 max_sges_in_ta_message;
    u16                 max_sges_in_chain_segment;
    spinlock_t          resource_lock;
    
    size_t              sge_chain_segment_size;
    struct dma_pool     *ta_chain_dma_pool;
    struct chain_tracker *ta_chain_lookup;
    struct list_head    ta_free_chain_list;
    
    struct _mpt3stm_ta* ta_lookup;
    struct workqueue_struct *recycleq; // work queue for returning commands to
                                       // IOC
    
    u8                  ta_max_sges;
    
};

static struct mpt3stm_ports _ports;

static u8       _num_iocs;
static struct _mpt3stm_ioc _iocs[MPT3STM_MAX_IOCS];


/**
 * _find_stmioc - turn the mpt3sas IOC pointer into our private target pointer
 *
 * Returns the private target pointer
 */
inline struct _mpt3stm_ioc*
_ioc_to_stmioc( struct MPT3SAS_ADAPTER * const ioc )
{
    return (struct _mpt3stm_ioc*) ioc->stmpriv;
}

/**
 * _to_iocidx - turn the private target pointer into an array index
 *
 * Returns the index of the stmioc pointer in the _iocs array
 */
inline u8
_to_iocidx( struct _mpt3stm_ioc * const stmioc )
{
    return (u8) ((stmioc - _iocs)/sizeof(struct _mpt3stm_ioc));
}

inline struct _mpt3stm_ioc *
_iocidx_to_ioc( u8 iocidx )
{
    BUG_ON( iocidx >= _num_iocs );
    return &_iocs[iocidx];
}


union _mpt25target_request_union {
    Mpi25TargetAssistRequest_t assist;
    Mpi2TargetStatusSendRequest_t status;
};





//int _build_sg(struct MPT3SAS_ADAPTER *ioc,
//              struct sg_table *sg_tbl, u32 smid, int rw, void *lsge, int main_sges)
//{
//    
//}


/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */
/*      IOC Houskeeping                                                        */
/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */

/**
 * @func _stm_msg_callback - command completion routine for oob (non I/O) messages
 * @ioc: per adapter object
 * @smid: system request message index
 * @msix_index: MSIX table index supplied by the OS
 * @reply: reply message frame(lower 32bit addr)
 *
 * Return 1 meaning mf should be freed from _base_interrupt
 *        0 means the mf is freed from this function.
 */
static u8
_stm_msg_callback(struct MPT3SAS_ADAPTER *ioc, u16 smid, u8 msix_index,
                  u32 reply)
{
    
    struct _mpt3stm_ioc *stmioc;
    MPI2DefaultReply_t *mpi_reply;
    
    stmioc = _ioc_to_stmioc(ioc);
    BUG_ON( !stmioc );
    
    if (stmioc->stm_post_cmds.status == MPT3_CMD_NOT_USED)
    {
        pr_err( MPT3SAS_FMT "%s that's odd, stm_post_cmds not used\n", ioc->name, __func__ );
        return 1;
    }
    
    mpi_reply = mpt3sas_base_get_reply_virt_addr(ioc, reply);
    if (unlikely(!mpi_reply)) {
        pr_err(MPT3SAS_FMT "mpi_reply not valid at %s:%d/%s()!\n",
               ioc->name, __FILE__, __LINE__, __func__);
        return 1;
    }
    
    switch ( mpi_reply->Function ) {
        case MPI2_FUNCTION_TARGET_CMD_BUF_BASE_POST:
        case MPI2_FUNCTION_TARGET_CMD_BUF_LIST_POST:
        case MPI2_FUNCTION_TARGET_MODE_ABORT:
            break;
        default:
            pr_err( MPT3SAS_FMT "%s: Unknown function %x\n",
                   ioc->name, __func__, mpi_reply->Function);
            return 1;
    }
    
    stmioc->stm_post_cmds.status &= ~MPT3_CMD_PENDING;
    stmioc->stm_post_cmds.status |= MPT3_CMD_COMPLETE;
    stmioc->stm_post_cmds.status |= MPT3_CMD_REPLY_VALID;
    stmioc->stm_post_cmds.reply = kzalloc(mpi_reply->MsgLength*4, GFP_ATOMIC);
    if ( stmioc->stm_post_cmds.reply )
        memcpy(stmioc->stm_post_cmds.reply, mpi_reply, mpi_reply->MsgLength*4);
    else
        pr_err( MPT3SAS_FMT "%s: failed kzalloc\n",
               ioc->name, __func__);
    
    complete(&stmioc->stm_post_cmds.done);
    return 1;
}


/*
 * @func _cmdbuf_base_post
 * @brief Send a Command Buffer Post request
 *
 * @returns 0 or error code
 */
static int
_cmdbuf_base_post(struct _mpt3stm_ioc *stmioc)
{
    struct MPT3SAS_ADAPTER *ioc;
    Mpi2TargetCmdBufferPostBaseRequest_t *mpi_request = NULL;
    U64* pBaseAddress;
    Mpi2TargetCmdBufferPostBaseListReply_t *mpi_reply = NULL;
    unsigned long timeleft;
    int r = 0;
    u16 smid;
    u16 ioc_status;
    
    ioc = stmioc->ioc;
    
    if ( !stmioc->ncmds )
        return -ENODEV;
    
    mutex_lock( &stmioc->stm_post_cmds.mutex );
    if ( stmioc->stm_post_cmds.status & MPT3_CMD_PENDING ) {
        pr_warn(MPT3SAS_FMT "%s(): internal command already in use?\n",
               ioc->name, __func__);
    }
    stmioc->stm_post_cmds.status = MPT3_CMD_PENDING;
    
    smid = mpt3sas_base_get_smid(ioc, stmioc->stm_post_cb_idx);
    if (!smid) {
        pr_err(MPT3SAS_FMT "%s(): failed obtaining a smid\n",
               ioc->name, __func__);
        r = -EAGAIN;
        goto out;
    }
    stmioc->stm_post_cmds.smid = smid;
    
    mpi_request = mpt3sas_base_get_msg_frame(ioc, smid);
    if (unlikely(!mpi_request)) {
        pr_err(MPT3SAS_FMT "%s(): mpi_request not valid\n",
               ioc->name, __func__);
        r = -ENOMEM;
        goto out;
    }
    
    memset(mpi_request, 0, sizeof(Mpi2TargetCmdBufferPostBaseRequest_t));
    mpi_request->Function = MPI2_FUNCTION_TARGET_CMD_BUF_BASE_POST;
    mpi_request->BufferPostFlags = MPI2_CMD_BUF_POST_BASE_FLAGS_AUTO_POST_ALL;
    mpi_request->TotalCmdBuffers = cpu_to_le16( stmioc->ncmds );
    mpi_request->CmdBufferLength = cpu_to_le16( sizeof(MPT3STM_CMD_BUFF) );
    pBaseAddress = (u64*) &(mpi_request->BaseAddressLow);
    *pBaseAddress =  cpu_to_le64( stmioc->cmdbufs_dma );
    
    init_completion(&stmioc->stm_post_cmds.done);
    mpt3sas_base_put_smid_default(ioc, smid);
    timeleft = wait_for_completion_timeout(&stmioc->stm_post_cmds.done,
                                           100*HZ);
    mutex_unlock( &stmioc->stm_post_cmds.mutex );
    
    if (!(stmioc->stm_post_cmds.status & MPT3_CMD_COMPLETE)) {
        pr_err(MPT3SAS_FMT "%s(): timeout on wait_for_completion_timeout\n",
               ioc->name, __func__);
        _debug_dump_mf(mpi_request,
                       sizeof(Mpi2PortEnableRequest_t)/4);
        if (stmioc->stm_post_cmds.status & MPT3_CMD_RESET)
            r = -EFAULT;
        else
            r = -ETIME;
        goto out;
    }
    
    mpi_reply = stmioc->stm_post_cmds.reply;
    BUG_ON(!mpi_reply);
    
    ioc_status = (le16_to_cpu(mpi_reply->IOCStatus) & MPI2_IOCSTATUS_MASK);
    if (ioc_status != MPI2_IOCSTATUS_SUCCESS) {
        pr_err(MPT3SAS_FMT "%s(): failed with (ioc_status=0x%08x)\n",
               ioc->name, __func__, ioc_status);
        r = -EFAULT;
        goto out;
    }
    
out:
    stmioc->stm_post_cmds.status = MPT3_CMD_NOT_USED;
    stmioc->stm_post_cmds.reply = NULL;
    VDPRINTK("complete\n");
    return r;
}

/**
 * @func _set_enable
 * @returns 0 or error
 */
int
_set_enable(struct _mpt3stm_ioc* stmioc,
            int enable)
{
    Mpi2SasIOUnitPage1_t *sas_page1;
    Mpi2ConfigReply_t mpi_reply;
    struct _mpt3stm_phy *pphy;
    int rc = 0;
    
    // check to see if needed
    if (enable == stmioc->enabled)
        return 0;
    
    sas_page1 = kzalloc(MPT3SAS_SASIOU1_SZ,
                        GFP_KERNEL);
    if ( !sas_page1 )
    {
        pr_err( MPT3SAS_FMT "%s(): failed kzmalloc\n",
               stmioc->ioc->name, __func__);
        return -ENOMEM;
    }
    
    rc = mpt3sas_config_get_sas_iounit_pg1( stmioc->ioc,
                                           &mpi_reply,
                                           sas_page1,
                                           MPT3SAS_SASIOU1_SZ);
    
    if (rc)
    {
        pr_err( MPT3SAS_FMT "%s(): failed mpt3sas_config_get_sas_iounit_pg1() with (iocstatus=0x%08x)\n",
               stmioc->ioc->name, __func__,le16_to_cpu(mpi_reply.IOCStatus));
        goto freeandexit;
    }
    
    list_for_each_entry(pphy,&stmioc->phy_list,list)
    {
        VDPRINTK(" %sabling phy: %d\n", (enable ? "En" : "Dis"),
                 pphy->phy_num);
        if (enable) {
            sas_page1->PhyData[pphy->phy_num].PhyFlags
            &= ~MPI2_SASIOUNIT1_PHYFLAGS_PHY_DISABLE;
        } else {
            sas_page1->PhyData[pphy->phy_num].PhyFlags
            |= MPI2_SASIOUNIT1_PHYFLAGS_PHY_DISABLE;
        }
    }
    
    memset( &mpi_reply,0,sizeof(Mpi2ConfigReply_t) );
    rc = mpt3sas_config_set_sas_iounit_pg1( stmioc->ioc,
                                           &mpi_reply,
                                           sas_page1,
                                           MPT3SAS_SASIOU1_SZ );
    
    if (rc)
    {
        pr_err( MPT3SAS_FMT "%s(): failed mpt3sas_config_set_sas_iounit_pg1() with (iocstatus=0x%08x)\n",
               stmioc->ioc->name, __func__,le16_to_cpu(mpi_reply.IOCStatus));
        goto freeandexit;
    }
    stmioc->enabled = enable;
    
freeandexit:
    kfree(sas_page1);
    return rc;
}


int
_abort_all(struct _mpt3stm_ioc* stmioc)
{
    Mpi2TargetModeAbort_t	*mpi_request;
    Mpi2TargetModeAbortReply_t *mpi_reply;
    u16 smid, ioc_status;
    int rc = 0;
    
    if ( !stmioc->ncmds )
        return -ENODEV;
    
    if ( stmioc->stm_post_cmds.status & MPT3_CMD_PENDING ) {
        pr_err(MPT3SAS_FMT "%s(): internal command already in use\n",
               stmioc->ioc->name, __func__);
        return -EAGAIN;
    }
    
    smid = mpt3sas_base_get_smid(stmioc->ioc, stmioc->stm_abort_cb_idx);
    if (!smid) {
        pr_err(MPT3SAS_FMT "%s(): failed obtaining a smid\n",
               stmioc->ioc->name, __func__);
        rc = -EAGAIN;
        goto out;
    }
    
    mpi_request = mpt3sas_base_get_msg_frame(stmioc->ioc, smid);
    if (unlikely(!mpi_request)) {
        pr_err(MPT3SAS_FMT "%s(): mpi_request not valid\n",
               stmioc->ioc->name, __func__);
        rc = -ENOMEM;
        goto out;
    }
    
    stmioc->stm_post_cmds.status = MPT3_CMD_PENDING;
    stmioc->stm_post_cmds.smid = smid;
    stmioc->abort_all = 1;
    
    memset(mpi_request,0,sizeof(Mpi2TargetModeAbort_t));
    mpi_request->Function = MPI2_FUNCTION_TARGET_MODE_ABORT;
    mpi_request->AbortType = MPI2_TARGET_MODE_ABORT_ALL_CMD_BUFFERS;
    
    init_completion(&stmioc->stm_post_cmds.done);
    mpt3sas_base_put_smid_default(stmioc->ioc, smid);
    rc = wait_for_completion_interruptible(&stmioc->stm_post_cmds.done);
    if (!(stmioc->stm_post_cmds.status & MPT3_CMD_COMPLETE)) {
        pr_err(MPT3SAS_FMT "%s(): interrupted on wait_for_completion_interruptible\n",
               stmioc->ioc->name, __func__);
        _debug_dump_mf(mpi_request,
                       sizeof(Mpi2TargetModeAbort_t)/4);
        if (stmioc->stm_post_cmds.status & MPT3_CMD_RESET)
            rc = -EFAULT;
        else
            rc = -ETIME;
        goto out;
    }
    
    stmioc->abort_all = 0;
    mpi_reply = stmioc->stm_post_cmds.reply;
    BUG_ON(!mpi_reply);
    
    ioc_status = (le16_to_cpu(mpi_reply->IOCStatus) & MPI2_IOCSTATUS_MASK);
    if (ioc_status != MPI2_IOCSTATUS_SUCCESS) {
        pr_err(MPT3SAS_FMT "%s(): failed with (ioc_status=0x%08x)\n",
               stmioc->ioc->name, __func__, ioc_status);
        rc = -EFAULT;
    }
    
out:
    stmioc->stm_post_cmds.status = MPT3_CMD_NOT_USED;
    stmioc->stm_post_cmds.reply = NULL;
    VDPRINTK("complete\n");
    return rc;
}

int
_post_command(struct _mpt3stm_ioc *stmioc, u16 IoIndex) {
    Mpi2TargetCmdBufferPostListRequest_t *mpi_request = NULL;
    Mpi2TargetCmdBufferPostBaseListReply_t *mpi_reply = NULL;
    unsigned long timeleft;
    int r = 0;
    u16 smid;
    u16 ioc_status;
    
    mutex_lock( &stmioc->stm_post_cmds.mutex );
    if ( stmioc->stm_post_cmds.status & MPT3_CMD_PENDING ) {
        pr_warn(MPT3SAS_FMT "%s(): internal command already in use?\n",
                stmioc->ioc->name, __func__);
    }
    stmioc->stm_post_cmds.status = MPT3_CMD_PENDING;
    
    smid = mpt3sas_base_get_smid(stmioc->ioc, stmioc->stm_post_cb_idx);
    if (!smid) {
        pr_err(MPT3SAS_FMT "%s(): failed obtaining a smid\n",
               stmioc->ioc->name, __func__);
        r = -EAGAIN;
        goto out;
    }
    stmioc->stm_post_cmds.smid = smid;
    
    mpi_request = mpt3sas_base_get_msg_frame(stmioc->ioc, smid);
    if (unlikely(!mpi_request)) {
        pr_err(MPT3SAS_FMT "%s(): mpi_request not valid\n",
               stmioc->ioc->name, __func__);
        r = -ENOMEM;
        goto out;
    }
    
    memset(mpi_request, 0, sizeof(Mpi2TargetCmdBufferPostBaseRequest_t));
    mpi_request->Function = MPI2_FUNCTION_TARGET_CMD_BUF_LIST_POST;
    //mpi_request->ChainOffset = 0;
    mpi_request->CmdBufferCount = 1;
    mpi_request->IoIndex[0] = IoIndex;
    
    init_completion(&stmioc->stm_post_cmds.done);
    mpt3sas_base_put_smid_default(stmioc->ioc, smid);
    timeleft = wait_for_completion_timeout(&stmioc->stm_post_cmds.done,
                                           100*HZ);
    mutex_unlock( &stmioc->stm_post_cmds.mutex );
    if (!(stmioc->stm_post_cmds.status & MPT3_CMD_COMPLETE)) {
        pr_err(MPT3SAS_FMT "%s(): timeout on wait_for_completion_timeout\n",
               stmioc->ioc->name, __func__);
        _debug_dump_mf(mpi_request,
                       sizeof(Mpi2PortEnableRequest_t)/4);
        if (stmioc->stm_post_cmds.status & MPT3_CMD_RESET)
            r = -EFAULT;
        else
            r = -ETIME;
        goto out;
    }
    
    mpi_reply = stmioc->stm_post_cmds.reply;
    BUG_ON(!mpi_reply);
    
    ioc_status = (le16_to_cpu(mpi_reply->IOCStatus) & MPI2_IOCSTATUS_MASK);
    if (ioc_status != MPI2_IOCSTATUS_SUCCESS) {
        pr_err(MPT3SAS_FMT "%s(): failed with (ioc_status=0x%08x)\n",
               stmioc->ioc->name, __func__, ioc_status);
        r = -EFAULT;
        goto out;
    }
    
out:
    stmioc->stm_post_cmds.status = MPT3_CMD_NOT_USED;
    stmioc->stm_post_cmds.reply = NULL;
    VDPRINTK("complete\n");
    return r;
}



/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */
/*      Exposed API to caller modules                                          */
/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */

void
mpt3stm_get_portmap( struct mpt3stm_ports * portmapbuffer )
{
    memcpy(portmapbuffer,&_ports,sizeof(_ports));
}
EXPORT_SYMBOL(mpt3stm_get_portmap);

MPT3STM_CMD_BUFF*
mpt3stm_get_cmdbuff(u8 iocidx,
                        u16 ioidx)
{
    return &_iocidx_to_ioc(iocidx)->cmdbufs[ioidx];
}
EXPORT_SYMBOL(mpt3stm_get_cmdbuff);


int mpt3stm_abort_all(u8 iocidx){
    return _abort_all(_iocidx_to_ioc(iocidx));
}
EXPORT_SYMBOL(mpt3stm_abort_all);

int mpt3stm_enable( u8 iocidx, u8 enable ) {
    return _set_enable( _iocidx_to_ioc(iocidx), enable );
}
EXPORT_SYMBOL(mpt3stm_enable);

int mpt3stm_is_enabled(u8 iocidx)
{
    return _iocidx_to_ioc(iocidx)->enabled;
}
EXPORT_SYMBOL(mpt3stm_is_enabled);

int mpt3stm_register( u8 iocidx,
                          MPT3STM_DISPATCH *dispatch ) {
    _iocidx_to_ioc(iocidx)->dispatch = dispatch;
    return 0;
}
EXPORT_SYMBOL(mpt3stm_register);

void mpt3stm_deregister( u8 iocidx ) {
    _iocidx_to_ioc(iocidx)->dispatch = NULL;
}
EXPORT_SYMBOL(mpt3stm_deregister);

u16 mpt3stm_get_ncmds(u8 iocidx) {
    return _iocidx_to_ioc(iocidx)->ncmds;
}
EXPORT_SYMBOL(mpt3stm_get_ncmds);

// maximum number of sges is chain size +1 immediate in REQ
u8 mpt3stm_get_max_sges(u8 iocidx) {
    return _iocidx_to_ioc(iocidx)->ta_max_sges;
}
EXPORT_SYMBOL(mpt3stm_get_max_sges);

u32 mpt3stm_get_iocstate(u8 iocidx, int cooked)
{
    struct _mpt3stm_ioc*    stmioc = _iocidx_to_ioc(iocidx);
    struct MPT3SAS_ADAPTER* ioc = stmioc->ioc;
    return mpt3sas_base_get_iocstate(ioc,cooked);
}
EXPORT_SYMBOL(mpt3stm_get_iocstate);

char *mpt3stm_get_adapter_name(u8 iocidx)
{
    return MPT3SAS_DRIVER_NAME;
}
EXPORT_SYMBOL(mpt3stm_get_adapter_name);

struct pci_dev *
mpt3stm_get_pci_dev(u8 iocidx)
{
    struct _mpt3stm_ioc*    stmioc = _iocidx_to_ioc(iocidx);
    struct MPT3SAS_ADAPTER* ioc = stmioc->ioc;
    BUG_ON(!ioc);
    return ioc->pdev;
}
EXPORT_SYMBOL(mpt3stm_get_pci_dev);

void* mpt3stm_ioc_get_priv( u8 iocidx )
{
    return _iocidx_to_ioc(iocidx)->priv;
}
EXPORT_SYMBOL(mpt3stm_ioc_get_priv);

void mpt3stm_ioc_set_priv( u8 iocidx, void* priv)
{
    _iocidx_to_ioc(iocidx)->priv = priv;
}
EXPORT_SYMBOL(mpt3stm_ioc_set_priv);



int
mpt3stm_remove_device(u8 iocidx, u16 DevHandle) {
    struct _mpt3stm_ioc*        stmioc;
    struct MPT3SAS_ADAPTER* ioc;
    Mpi2SasIoUnitControlRequest_t mpi_request;
    Mpi2SasIoUnitControlReply_t mpi_reply;
    int r = 0;
    
    stmioc = _iocidx_to_ioc(iocidx);
    ioc = stmioc->ioc;
    memset(&mpi_request,0,sizeof(Mpi2SasIoUnitControlRequest_t));
    
    mpi_request.Function = MPI2_FUNCTION_SAS_IO_UNIT_CONTROL;
    mpi_request.Operation = MPI2_SAS_OP_REMOVE_DEVICE;
    mpi_request.DevHandle = cpu_to_le16( DevHandle );
    
    r = mpt3sas_base_sas_iounit_control(ioc,
                                        &mpi_reply,
                                        &mpi_request);
    if (r) {
        pr_err(MPT3SAS_FMT "%s: failed with (ioc_status=0x%08x)\n",
               ioc->name, __func__, mpi_reply.IOCStatus);
        
        _debug_dump_mf(&mpi_request,sizeof(Mpi2SasIoUnitControlRequest_t)/4);
        _debug_dump_reply(&mpi_reply,mpi_reply.MsgLength);
        printk("==============\n");
    }
    
    return r;
}
EXPORT_SYMBOL(mpt3stm_remove_device);

int
mpt3stm_send_status( u8 iocidx,
                        u8 StatusFlags,
                        u16 QueueTag,
                        u16 InitiatorConnectionTag,
                        u16 IoIndex) {
    struct _mpt3stm_ioc*        stmioc;
    struct MPT3SAS_ADAPTER* ioc;
    u16 ts_smid;
    Mpi2TargetStatusSendRequest_t* mpi_request;
    int                     r = 0;

    // We cannot be run from the IRQ context (softirq callback) because this
    // interferes with interrupt enabling due to the use of an inline funciton
    // for _writeq with a shared flags varibale for spin_lock_saveirq()!!
    // we could fix the upstream LSI code in mpt3sas_base.c or just accept
    // this as a restriction as is done here with BUG_ON()
    BUG_ON(in_irq());
    
    stmioc = _iocidx_to_ioc(iocidx);
    ioc = stmioc->ioc;
    BUG_ON(!ioc);
    
    BUG_ON(!stmioc->stm_assist_cb_idx);
    
    ts_smid = mpt3sas_base_get_smid(ioc,stmioc->stm_assist_cb_idx);
    if ( !ts_smid ) {
        pr_err(MPT3SAS_FMT "%s: failed obtaining a smid\n",
               stmioc->ioc->name, __func__);
        r = -EAGAIN;
        goto out;
    }
    
    mpi_request = mpt3sas_base_get_msg_frame(ioc,ts_smid);
    if (unlikely(!mpi_request)) {
        pr_err(MPT3SAS_FMT "%s(): mpi_request not valid\n",
               ioc->name, __func__);
        mpt3sas_base_free_smid(ioc,
                               ts_smid);
        r = -ENOMEM;
        goto out;
    }
    
    memset(mpi_request, 0, sizeof(Mpi2TargetStatusSendRequest_t));
    mpi_request->Function = MPI2_FUNCTION_TARGET_STATUS_SEND;
    mpi_request->StatusFlags = StatusFlags;
    mpi_request->QueueTag = cpu_to_le16(QueueTag);
    mpi_request->InitiatorConnectionTag = cpu_to_le16(InitiatorConnectionTag);
    mpi_request->IoIndex = cpu_to_le16(IoIndex);
    mpi_request->SGLOffset0 = offsetof(Mpi2TargetStatusSendRequest_t, StatusDataSGE) / 4;
    
    mpi_request->StatusDataSGE.IeeeSimple.Simple64.Flags = MPI25_IEEE_SGE_FLAGS_END_OF_LIST;
    mpi_request->StatusDataSGE.IeeeSimple.Simple64.Length = cpu_to_le32(sizeof(union _rsp_frame));
    mpi_request->StatusDataSGE.IeeeSimple.Simple64.Address = cpu_to_le64(stmioc->cmdbufs_dma
        + (sizeof(MPT3STM_CMD_BUFF) * IoIndex) + offsetof(MPT3STM_CMD_BUFF,rsp));
        
    mpt3sas_base_put_smid_target( ioc,
                                  ts_smid,
                                  IoIndex );
    
out:
    return r;
}
EXPORT_SYMBOL(mpt3stm_send_status);

/**
 * _mpt3stm_get_chain_buffer_tracker - obtain chain tracker
 * @ioc: per adapter object
 * @smid: smid associated to an IO request
 *
 * Returns chain tracker(from ioc->free_chain_list)
 */
static struct chain_tracker *
_mpt3stm_get_chain_buffer_tracker(struct _mpt3stm_ioc *stmioc, u16 IoIndex)
{
    struct chain_tracker *chain_req;
    unsigned long flags;
    
    spin_lock_irqsave(&stmioc->resource_lock, flags);
    if (list_empty(&stmioc->ta_free_chain_list)) {
        spin_unlock_irqrestore(&stmioc->resource_lock, flags);
        pr_err(MPT3SAS_FMT "%s: chain buffers not available\n",
               stmioc->ioc->name, __func__);
        return NULL;
    }
    chain_req = list_entry(stmioc->ta_free_chain_list.next,
                           struct chain_tracker, tracker_list);
    list_del_init(&chain_req->tracker_list);
    list_add_tail(&chain_req->tracker_list,
                  &stmioc->ta_lookup[IoIndex].chain_list);
    spin_unlock_irqrestore(&stmioc->resource_lock, flags);
    return chain_req;
}

/**
 * _mpt3stm_add_sg_single_ieee - add sg element for IEEE format
 * @paddr: virtual address for SGE
 * @flags: SGE flags
 * @chain_offset: number of 128 byte elements from start of segment
 * @length: data transfer length
 * @dma_addr: Physical address
 *
 * Return nothing.
 */
static void
_mpt3stm_add_sg_single_ieee(void *paddr, u8 flags, u8 chain_offset, u32 length,
                         dma_addr_t dma_addr)
{
    Mpi25IeeeSgeChain64_t *sgel = paddr;
    
    sgel->Flags = flags;
    sgel->NextChainOffset = chain_offset;
    sgel->Length = cpu_to_le32(length);
    sgel->Address = cpu_to_le64(dma_addr);
}

static int
_mpt3stm_build_sg_targetassist(struct _mpt3stm_ta *ta)
{
    struct MPT3SAS_ADAPTER *ioc = ta->stmioc->ioc;
    dma_addr_t chain_dma;
    struct scatterlist *sg_tacmd;
    void *sg_local, *chain;
    u32 chain_offset;
    u32 chain_length;
    int sges_left;
    u32 sges_in_segment;
    u8 simple_sgl_flags;
    u8 simple_sgl_flags_last;
    u8 chain_sgl_flags;
    struct chain_tracker *chain_req;
    
    BUG_ON(!ioc);
    
    /* init scatter gather flags */
    simple_sgl_flags = MPI2_IEEE_SGE_FLAGS_SIMPLE_ELEMENT |
        MPI2_IEEE_SGE_FLAGS_SYSTEM_ADDR;
    simple_sgl_flags_last = simple_sgl_flags |
        MPI25_IEEE_SGE_FLAGS_END_OF_LIST;
    chain_sgl_flags = MPI2_IEEE_SGE_FLAGS_CHAIN_ELEMENT |
        MPI2_IEEE_SGE_FLAGS_SYSTEM_ADDR;
    
    sg_tacmd = ta->sg;
    sges_left = dma_map_sg(&ioc->pdev->dev,
                           ta->sg,
                           ta->nents,
                           ta->dir);
    if (!sges_left) {
        pr_err(MPT3SAS_FMT "%s: dma_map_sg failed\n",
               ioc->name, __func__);
        return -ENOMEM;
    }
    
    sg_local = &ta->mpi_request->SGL;
    sges_in_segment = ta->stmioc->max_sges_in_ta_message;
    
    if (sges_left <= sges_in_segment)
        goto fill_in_last_segment;
    
    ta->mpi_request->ChainOffset = (sges_in_segment - 1 /* chain element */) +
        (offsetof(Mpi25TargetAssistRequest_t, SGL)/ioc->sge_size_ieee);
    
    /* fill in main message segment when there is a chain following */
    while (sges_in_segment > 1) {
        _mpt3stm_add_sg_single_ieee(sg_local, simple_sgl_flags, 0,
                                 sg_dma_len(sg_tacmd), sg_dma_address(sg_tacmd));
        sg_tacmd = sg_next(sg_tacmd);
        sg_local += ioc->sge_size_ieee;
        sges_left--;
        sges_in_segment--;
    }
    
    /* initializing the pointers */
    chain_req = _mpt3stm_get_chain_buffer_tracker(ta->stmioc, ta->IoIndex);
    if (!chain_req)
        return -1;
    chain = chain_req->chain_buffer;
    chain_dma = chain_req->chain_buffer_dma;
    do {
        sges_in_segment = (sges_left <=
                           ta->stmioc->max_sges_in_chain_segment) ? sges_left :
                            ta->stmioc->max_sges_in_chain_segment;
        chain_offset = (sges_left == sges_in_segment) ?
                            0 : sges_in_segment;
        chain_length = sges_in_segment * ioc->sge_size_ieee;
        if (chain_offset)
            chain_length += ioc->sge_size_ieee;
        _mpt3stm_add_sg_single_ieee(sg_local, chain_sgl_flags,
                                 chain_offset, chain_length, chain_dma);
        
        sg_local = chain;
        if (!chain_offset)
            goto fill_in_last_segment;
        
        /* fill in chain segments */
        while (sges_in_segment) {
            _mpt3stm_add_sg_single_ieee(sg_local, simple_sgl_flags, 0,
                                     sg_dma_len(sg_tacmd), sg_dma_address(sg_tacmd));
            sg_tacmd = sg_next(sg_tacmd);
            sg_local += ioc->sge_size_ieee;
            sges_left--;
            sges_in_segment--;
        }
        
        chain_req = _mpt3stm_get_chain_buffer_tracker(ta->stmioc, ta->IoIndex);
        if (!chain_req)
            return -1;
        chain = chain_req->chain_buffer;
        chain_dma = chain_req->chain_buffer_dma;
    } while (1);
    
    
fill_in_last_segment:
    
    /* fill the last segment */
    while (sges_left) {
        if (sges_left == 1)
            _mpt3stm_add_sg_single_ieee(sg_local,
                                     simple_sgl_flags_last, 0, sg_dma_len(sg_tacmd),
                                     sg_dma_address(sg_tacmd));
        else
            _mpt3stm_add_sg_single_ieee(sg_local, simple_sgl_flags, 0,
                                     sg_dma_len(sg_tacmd), sg_dma_address(sg_tacmd));
        sg_tacmd = sg_next(sg_tacmd);
        sg_local += ioc->sge_size_ieee;
        sges_left--;
    }
    
    return 0;
}

void
_mpt3stm_free_sg_targetassist(struct _mpt3stm_ta *ta)
{
    struct chain_tracker *chain_req, *next;
    unsigned long flags;
    
    spin_lock_irqsave(&ta->stmioc->resource_lock, flags);
    if (!list_empty(&ta->chain_list)) {
        list_for_each_entry_safe(chain_req, next,
                                 &ta->chain_list, tracker_list) {
            list_del_init(&chain_req->tracker_list);
            list_add(&chain_req->tracker_list,
                     &ta->stmioc->ta_free_chain_list);
        }
    }
    spin_unlock_irqrestore(&ta->stmioc->resource_lock, flags);
    dma_unmap_sg(&ta->stmioc->ioc->pdev->dev,
                 ta->sg,
                 ta->nents,
                 ta->dir);
}

int
mpt3stm_assist( u8 iocidx, u8 AssistFlags, u16 QueueTag,
               u16 InitiatorConnectionTag, u16 IoIndex, u8 SequenceNumber,
               u32 SkipCount, u32 DataLength, struct scatterlist *sg,
               int nents, enum dma_data_direction dir)
{
    struct _mpt3stm_ioc* stmioc = NULL;
    struct _mpt3stm_ta *ta = NULL;
    int rc = 0;
    
    stmioc = _iocidx_to_ioc(iocidx);
    ta = &stmioc->ta_lookup[IoIndex];
    BUG_ON(!ta);
    
    ta->smid = mpt3sas_base_get_smid(stmioc->ioc, stmioc->stm_assist_cb_idx);
    if (unlikely(!ta->smid)) {
        pr_err(MPT3SAS_FMT "%s: failed obtaining a smid\n",
               stmioc->ioc->name, __func__);
        rc = -EAGAIN;
        goto out;
    }
    
    ta->mpi_request = mpt3sas_base_get_msg_frame(stmioc->ioc, ta->smid);
    BUG_ON(!ta->mpi_request);
    
    memset(ta->mpi_request, 0, sizeof(Mpi25TargetAssistRequest_t));
    ta->mpi_request->Function = MPI2_FUNCTION_TARGET_ASSIST;
    ta->mpi_request->TargetAssistFlags = AssistFlags;
    ta->mpi_request->QueueTag = cpu_to_le16(QueueTag);
    ta->mpi_request->InitiatorConnectionTag = cpu_to_le16(InitiatorConnectionTag);
    ta->mpi_request->IoIndex = cpu_to_le16(IoIndex);
    ta->mpi_request->SequenceNumber = SequenceNumber;
    ta->mpi_request->DMAFlags = MPI25_TA_DMAFLAGS_OP_D_D_D_D; // all SGLs are data
    ta->mpi_request->SGLOffset0 = offsetof(Mpi25TargetAssistRequest_t, SGL) / 4;
    ta->mpi_request->SkipCount = cpu_to_le32(SkipCount);
    ta->mpi_request->DataLength = cpu_to_le32(DataLength);

    ta->sg = sg;
    ta->nents = nents;
    ta->dir = dir;
    rc = _mpt3stm_build_sg_targetassist(ta);
    if ( !rc )
    {
//        printk("TARGET_ASSIST=======\n");
//        _debug_dump_mf(ta->mpi_request,
//                       sizeof(union _mpt25target_request_union)/4);
//        printk("=======TARGET_ASSIST\n");
    
        mpt3sas_base_put_smid_target(stmioc->ioc, ta->smid, IoIndex );
    }
out:
    return rc;
}
EXPORT_SYMBOL(mpt3stm_assist);

static void
_recycle_cmd_on_workq(struct work_struct *work) {
    struct _mpt3stm_ta* ta = container_of(work,struct _mpt3stm_ta,work_entry);
    VDPRINTK(MPT3SAS_FMT "%s: release I:%04x on workq\n",
             stmioc->ioc->name, __func__, ta->IoIndex);
    _post_command(ta->stmioc,ta->IoIndex);
}


int
mpt3stm_release_command(u8 iocidx, u16 IoIndex)
{
    struct _mpt3stm_ioc *stmioc = _iocidx_to_ioc(iocidx);
    struct _mpt3stm_ta *ta;
    BUG_ON(!stmioc);
    
    ta = &stmioc->ta_lookup[IoIndex];

    INIT_WORK(&ta->work_entry, _recycle_cmd_on_workq);
    queue_work(stmioc->recycleq, &ta->work_entry);
    return 0;
}
EXPORT_SYMBOL(mpt3stm_release_command);


/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */
/*      Interface with mpt3sas_base                                            */
/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */

void
_stm_watchdog_dispatch(struct MPT3SAS_ADAPTER *ioc)
{
    struct _mpt3stm_ioc *stmioc = _ioc_to_stmioc(ioc);
    BUG_ON( !stmioc );
    if (likely(stmioc->dispatch) && unlikely(stmioc->dispatch->stm_watchdog))
        stmioc->dispatch->stm_watchdog(_to_iocidx(stmioc));
    
}


/**
 * _stm_command_dispatch - command buffer posted by IOC
 * @ioc: per adapter object
 * @rpf: reply descriptor
 *
 */
void
_stm_command_dispatch(struct MPT3SAS_ADAPTER *ioc,
                      Mpi2TargetCommandBufferReplyDescriptor_t *rpf,
                      u8 msix_index) {
    struct _mpt3stm_ioc *stmioc;
    
    stmioc = _ioc_to_stmioc(ioc);
    BUG_ON( !stmioc );
    
    if (likely(stmioc->dispatch) && likely(stmioc->dispatch->stm_command)) {
        stmioc->dispatch->stm_command(_to_iocidx(stmioc),
                                      (MPI2_RPY_DESCRIPT_TCB_FLAGS_PHYNUM_MASK & rpf->Flags),
                                      le16_to_cpu(rpf->IoIndex), le16_to_cpu(rpf->InitiatorDevHandle));
    } else {
        pr_warn(MPT3SAS_FMT "%s: unable to dispatch command buffer!!\n",
                stmioc->ioc->name,__func__);
    }
    
}


/**
 * _stm_assist_err_callback - error completion for target assist messages (I/O)
 * @ioc: per adapter object
 * @smid: system request message index
 * @msix_index: MSIX table index supplied by the OS
 * @reply: reply message frame(lower 32bit addr)
 *
 * Return 1 meaning mf should be freed from _base_interrupt
 *        0 means the mf is freed from this function.
 */
static u8
_stm_assist_err_callback(struct MPT3SAS_ADAPTER *ioc, u16 smid, u8 msix_index,
                            u32 reply)
{
    Mpi2TargetErrorReply_t* mpi_reply = NULL;
    union _mpt25target_request_union* mpi_request = NULL;
    struct _mpt3stm_ioc*    stmioc;
    u16                     IoIndex;
    
    stmioc = _ioc_to_stmioc(ioc);
    mpi_request = mpt3sas_base_get_msg_frame(ioc,smid);
    
    mpi_reply = mpt3sas_base_get_reply_virt_addr(ioc, reply);
    BUG_ON(!mpi_reply);
    
    IoIndex = le16_to_cpu( mpi_reply->IoIndex );
    
    
    pr_warn( MPT3SAS_FMT "%s: assist error IoIndex:%x with (iocstatus=0x%08x\n",
            ioc->name, __func__,IoIndex,le16_to_cpu(mpi_reply->IOCStatus));
    _debug_dump_mf(mpi_request,
                   sizeof(union _mpt25target_request_union)/4);
    _debug_dump_reply(&mpi_reply,mpi_reply->MsgLength);
    
    if (likely(stmioc->dispatch)) {
        switch (mpi_request->assist.Function) {
            case MPI2_FUNCTION_TARGET_ASSIST:
                if (likely(stmioc->dispatch->stm_assist_complete))
                    _mpt3stm_free_sg_targetassist(&stmioc->ta_lookup[IoIndex]);
                    stmioc->dispatch->stm_assist_complete(_to_iocidx(stmioc),
                                                          IoIndex,
                                                          0, // no sequence number
                                                          mpi_reply);
                break;
            case MPI2_FUNCTION_TARGET_STATUS_SEND:
                if (likely(stmioc->dispatch->stm_status_complete))
                    stmioc->dispatch->stm_status_complete(_to_iocidx(stmioc),
                                                          IoIndex,
                                                          mpi_reply);
                break;
            default:
                pr_err( MPT3SAS_FMT "%s: invalid Function in callback error IoIndex:%x\n",
                        ioc->name, __func__,IoIndex);
                BUG_ON(1);
        }
    }
    else
    {
        pr_err( MPT3SAS_FMT "%s(): no dispatch installed!\n",
               ioc->name, __func__);
    }
    return 1;
}

/**
 * _stm_assist_success_dispatch - successful completion for TARGET_ASSIST or
 *                                  TARGET_STATUS message
 * @ioc: per adapter object
 * @rpf: reply descriptor frame
 *
 * Return 1 meaning mf should be freed from _base_interrupt
 *        0 means the mf is freed from this function.
 */
u8
_stm_assist_success_dispatch( struct MPT3SAS_ADAPTER *ioc,
                             Mpi2TargetAssistSuccessReplyDescriptor_t *rpf )
{
    MPI2RequestHeader_t* mpi_request = NULL;
    struct _mpt3stm_ioc *stmioc = _ioc_to_stmioc(ioc);
    u16 IoIndex = le16_to_cpu( rpf->IoIndex );
    BUG_ON(!stmioc);
    
    mpi_request = mpt3sas_base_get_msg_frame(ioc, rpf->SMID);
    BUG_ON(!mpi_request);
    
    VDPRINTK("assist_success IoIndex:%x\n",rpf->IoIndex);

    if (likely(stmioc->dispatch)) {
        switch (mpi_request->Function) {
            case MPI2_FUNCTION_TARGET_ASSIST:
                if (likely(stmioc->dispatch->stm_assist_complete))
                    _mpt3stm_free_sg_targetassist(&stmioc->ta_lookup[IoIndex]);
                    stmioc->dispatch->stm_assist_complete(_to_iocidx(stmioc),
                                                          IoIndex,
                                                          rpf->SequenceNumber,
                                                          NULL); // no err frame
                break;
            case MPI2_FUNCTION_TARGET_STATUS_SEND:
                if (likely(stmioc->dispatch->stm_status_complete))
                    stmioc->dispatch->stm_status_complete(_to_iocidx(stmioc),
                                                          IoIndex,
                                                          NULL); // no err frame
                break;
            default:
                pr_err( MPT3SAS_FMT "%s: invalid Function in callback error IoIndex:%x\n",
                       ioc->name, __func__,IoIndex);
                BUG_ON(1);
        }
    }
    else
    {
        pr_err( MPT3SAS_FMT "%s(): no dispatch installed!\n",
               ioc->name, __func__);
    }
    return 1;
}


/**
 * _stm_event_dispatch - EVENT notification, process, and pass along if necessary
 * @ioc: per adapter object
 * @msix_index: MSIX table index supplied by the OS
 * @reply: reply message frame(lower 32bit addr)
 *
 */
u8
_stm_event_dispatch( struct MPT3SAS_ADAPTER *ioc,
                    u8 msix_index,
                    u32 reply)
{
    Mpi2EventNotificationReply_t* mpi_reply;
    struct _mpt3stm_ioc *stmioc;
    
    /* events turned off due to host reset or driver unloading */
    if (ioc->remove_host || ioc->pci_error_recovery)
        return 0;
    
    
    stmioc = _ioc_to_stmioc(ioc);
    mpi_reply = mpt3sas_base_get_reply_virt_addr(ioc, reply);
    BUG_ON(!mpi_reply);

    if ( likely(stmioc->dispatch) && likely(stmioc->dispatch->stm_event) ) {
        stmioc->dispatch->stm_event(_to_iocidx(stmioc),
                                    mpi_reply);
    } else {
        pr_warn(MPT3SAS_FMT "%s: unable to dispatch event buffer!!\n",
                stmioc->ioc->name,__func__);
    }
    
    return 0;
}

/**
 * _stm_reset_dispatch - Forward reset lifecycle information to STM client
 *                          drivers
 * @ioc: per adapter object
 * @reset_phase: see mpt3sas_base.c
 */
void
_stm_reset_dispatch(struct MPT3SAS_ADAPTER *ioc, int reset_phase)
{
    struct _mpt3stm_ioc *stmioc = _ioc_to_stmioc(ioc);
    int rc;
    
    switch (reset_phase) {
        case MPT3_IOC_PRE_RESET:
            VDPRINTK("%s: MPT3_IOC_PRE_RESET\n",ioc->name);
            break;
        case MPT3_IOC_AFTER_RESET:
            VDPRINTK("%s: MPT3_IOC_AFTER_RESET\n",ioc->name);
            // terminate any CMD_BUFFER_POST
            if (stmioc->stm_post_cmds.status & MPT3_CMD_PENDING) {
                stmioc->stm_post_cmds.status |= MPT3_CMD_RESET;
                mpt3sas_base_free_smid(ioc, stmioc->stm_post_cmds.smid);
                complete(&stmioc->stm_post_cmds.done);
            }
            // terminate any TASK_MGMT
            if (stmioc->stm_tm_cmds.status & MPT3_CMD_PENDING) {
                stmioc->stm_tm_cmds.status |= MPT3_CMD_RESET;
                mpt3sas_base_free_smid(ioc, stmioc->stm_tm_cmds.smid);
                complete(&stmioc->stm_tm_cmds.done);
            }
            break;
        case MPT3_IOC_DONE_RESET:
            VDPRINTK("%s: MPT3_IOC_DONE_RESET\n",ioc->name);
            // re-post the command buffers, otherwise no incoming commands
            // can work
            rc = _cmdbuf_base_post(stmioc);
            if (rc)
            {
                pr_err(MPT3SAS_FMT "%s(): failed _cmdbuf_base_post()\n",
                       ioc->name, __func__);
            }
            else
            {
                // re-enable the port if necessary
                if (stmioc->enabled)
                {
                    _set_enable(stmioc,stmioc->enabled);
                }
            }
            break;
    }
    
    if ( likely(stmioc->dispatch) && likely(stmioc->dispatch->stm_reset) ) {
        stmioc->dispatch->stm_reset(_to_iocidx(stmioc),
                                    reset_phase);
    } else {
        pr_warn(MPT3SAS_FMT "%s: unable to dispatch reset activity!\n",
                ioc->name,__func__);
    }
}


/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */
/*      Standard module glue                                                   */
/* -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*- */

void _stm_exit( void )
{
    int i;
    struct _mpt3stm_ioc *stmioc;
    struct _mpt3stm_phy *pphy, *nphy;
    
    mpt3sas_base_stm_release_callback_handler();
    
    for (i=0; i < MPT3STM_MAX_IOCS; i++)
    {
        stmioc = _iocidx_to_ioc(i);
        
        if (stmioc->ioc)
        {
            _abort_all(stmioc);
            
            mpt3sas_base_release_callback_handler(stmioc->stm_abort_cb_idx);
            mpt3sas_base_release_callback_handler(stmioc->stm_post_cb_idx);
            mpt3sas_base_release_callback_handler(stmioc->stm_assist_cb_idx);
            
            if (stmioc->ta_chain_lookup) {
                for (i = 0; i < stmioc->ioc->chain_depth; i++) {
                    if (stmioc->ta_chain_lookup[i].chain_buffer)
                        pci_pool_free(stmioc->ta_chain_dma_pool,
                                      stmioc->ta_chain_lookup[i].chain_buffer,
                                      stmioc->ta_chain_lookup[i].chain_buffer_dma);
                }
                if (stmioc->ta_chain_dma_pool)
                    pci_pool_destroy(stmioc->ta_chain_dma_pool);
                kfree(stmioc->ta_chain_lookup);
                stmioc->ta_chain_lookup = NULL;
            }

            if (stmioc->ta_lookup)
                kfree(stmioc->ta_lookup);
            
            if (stmioc->stm_post_cmds.reply)
                kfree(stmioc->stm_post_cmds.reply);
            
            if (stmioc->cmdbufs)
                pci_free_consistent(stmioc->ioc->pdev,
                                    (stmioc->ncmds * sizeof(MPT3STM_CMD_BUFF)),
                                    stmioc->cmdbufs,
                                    stmioc->cmdbufs_dma );
            
            list_for_each_entry_safe(pphy,nphy,&stmioc->phy_list,list)
            {
                list_del(&pphy->list);
                kfree(pphy);
            }
            stmioc->ioc->stmpriv = NULL;
            stmioc->ioc = NULL;
        }
        
    }
}

struct STM_CALLBACK stm_callbacks = {
    .watchdog = _stm_watchdog_dispatch,
    .target_command = _stm_command_dispatch,
    .target_assist = _stm_assist_success_dispatch,
    .event_handler = _stm_event_dispatch,
    .reset_handler = _stm_reset_dispatch,
};

/**
 * @func _stm_init
 * @brief Standard module entry routine, parses the list of IOCs
 *
 * @returns 0 or error code
 */
int _stm_init( void )
{
    struct MPT3SAS_ADAPTER *ioc;
    struct _mpt3stm_ioc *stmioc;
    Mpi2ConfigReply_t mpi_reply;
    Mpi2SasIOUnitPage1_t *sas_page1 = NULL;
    struct _mpt3stm_phy *pphy = NULL;
    int i, rc = 0;
    int last_port;
    
    _num_iocs = 0;
    
    memset(&_iocs,0,sizeof(_iocs));
    mpt3sas_base_stm_register_callback_handler(stm_callbacks);
    
    list_for_each_entry(ioc, &mpt3sas_ioc_list, list) {

        if (! ioc->facts.MaxInitiators )
        {
            pr_info(MPT3SAS_FMT "Target-Mode: skipped non-target enabled ioc\n",
                    ioc->name);
        }
        else
        {
            stmioc = &_iocs[_num_iocs];
            stmioc->ioc = ioc;
            ioc->stmpriv = stmioc;
            
            if ( MPT3SAS_MAX_CMDBUFFS < ioc->pfacts[0].MaxPostedCmdBuffers )
                stmioc->ncmds = MPT3SAS_MAX_CMDBUFFS;
            else
                stmioc->ncmds = ioc->pfacts[0].MaxPostedCmdBuffers;
            
            pr_info(MPT3SAS_FMT "Target Mode: Max Commands: %d, Max Initiators: %d\n",
                    ioc->name, stmioc->ncmds, ioc->facts.MaxInitiators);
            
            INIT_LIST_HEAD( &stmioc->phy_list );
            
            // _internal_cmd structure used to control completions of MPI messages
            stmioc->stm_post_cmds.status = MPT3_CMD_NOT_USED;
            mutex_init(&stmioc->stm_post_cmds.mutex);
            stmioc->stm_post_cmds.reply = kzalloc(ioc->reply_sz, GFP_KERNEL);
            if ( !stmioc->stm_post_cmds.reply )
            {
                pr_err(MPT3SAS_FMT "%s(): failed kzmalloc\n",
                       ioc->name, __func__);
                rc = -ENOMEM;
                goto errout;
            }
            spin_lock_init(&stmioc->resource_lock);
            
            // BIG memory allocations for the command buffers (incoming SSP frames) and
            // outgoing returns (DMA addressable, PCI consistent)
            stmioc->cmdbufs = pci_alloc_consistent(ioc->pdev,
                                                   (stmioc->ncmds * sizeof(MPT3STM_CMD_BUFF)),
                                                   &stmioc->cmdbufs_dma );
            if (!stmioc->cmdbufs)
            {
                pr_err(MPT3SAS_FMT "%s(): failed pci_alloc_consistent ->cmdbufs\n",
                       ioc->name, __func__);
                rc = -ENOMEM;
                goto errout;
            }
            
            stmioc->recycleq = create_singlethread_workqueue( ioc->name ); // only one thing allowed to run at a time!
            
            stmioc->ta_lookup =
                kzalloc(stmioc->ncmds * sizeof(struct _mpt3stm_ta), GFP_KERNEL);
            if (!stmioc->ta_lookup)
            {
                pr_err(MPT3SAS_FMT "%s(): failed kzalloc ->ta_lookup\n",
                       ioc->name, __func__);
                rc = -ENOMEM;
                goto errout;
            }
            for ( i=0; i < stmioc->ncmds; i++)
            {
                INIT_LIST_HEAD(&stmioc->ta_lookup[i].chain_list);
                stmioc->ta_lookup[i].IoIndex = i;
                stmioc->ta_lookup[i].stmioc = stmioc;
            }
            
            // Another BIG memory allocation, this time for the
            // SGL chains needed to map scatter gather lists in memory
            if ( 0 == stmioc->ioc->facts.IOCMaxChainSegmentSize)
            {
                stmioc->sge_chain_segment_size = 0x160000;
                pr_info(MPT3SAS_FMT "%s: 1MBy chains!\n",
                        ioc->name, __func__);
            }
            else
            {
                stmioc->sge_chain_segment_size =
                    stmioc->ioc->facts.IOCMaxChainSegmentSize * 16;
                pr_info(MPT3SAS_FMT "%s: %04x chain segments (%x)\n",
                        ioc->name, __func__,
                        stmioc->sge_chain_segment_size,
                        ioc->request_sz);
            }
            
            stmioc->ta_chain_lookup =
                kzalloc((ioc->chain_depth * sizeof(struct chain_tracker)),
                    GFP_KERNEL);
            if (!stmioc->ta_chain_lookup)
            {
                pr_err(MPT3SAS_FMT "%s(): failed kzalloc ->ta_chain_lookup\n",
                    ioc->name, __func__);
                rc = -ENOMEM;
                goto errout;
            }
            
            stmioc->ta_chain_dma_pool = pci_pool_create("target chain pool", ioc->pdev,
                                                  stmioc->sge_chain_segment_size, 16, 0);
            if (!stmioc->ta_chain_dma_pool) {
                pr_err(MPT3SAS_FMT "ta_chain_dma_pool: pci_pool_create failed\n",
                       ioc->name);
                goto errout;
            }
            for (i = 0; i < ioc->chain_depth; i++) {
                stmioc->ta_chain_lookup[i].chain_buffer = pci_pool_alloc(
                    stmioc->ta_chain_dma_pool , GFP_KERNEL,
                    &stmioc->ta_chain_lookup[i].chain_buffer_dma);
                if (!stmioc->ta_chain_lookup[i].chain_buffer) {
                    pr_err(MPT3SAS_FMT "ta_chain_dma_pool: pci_pool_alloc failed\n",
                           ioc->name);
                    goto errout;
                }
                //                total_sz += ioc->request_sz;
            }
//            pr_info(MPT3SAS_FMT
//                "ta chain pool depth(%d), frame_size(%d), pool_size(%d kB)\n",
//                ioc->name, stmioc->ta_chain_depth, ioc->request_sz,
//                ((stmioc->ta_chain_depth *  ioc->request_sz))/1024);
            
            INIT_LIST_HEAD(&stmioc->ta_free_chain_list);
            for (i = 0; i < ioc->chain_depth; i++)
                list_add_tail(&stmioc->ta_chain_lookup[i].tracker_list,
                              &stmioc->ta_free_chain_list);
            
            // how many SGL entries fit in the TargetAssist msg
            stmioc->max_sges_in_ta_message = (ioc->request_sz -
                offsetof(Mpi25TargetAssistRequest_t, SGL))/sizeof(Mpi25SGEIOUnion_t);
            
            // how many SGL entries fit in the SGL chains, 1 SGE is set aside
            // for the chain element to cascade
            stmioc->max_sges_in_chain_segment =
                (stmioc->sge_chain_segment_size -
                    sizeof(Mpi25SGEIOUnion_t))/sizeof(Mpi25SGEIOUnion_t);
            
            stmioc->ta_max_sges = (stmioc->max_sges_in_ta_message - 1) +
                (stmioc->max_sges_in_chain_segment * ioc->facts.MaxChainDepth);
            
            pr_info(MPT3SAS_FMT "TA: Imm SGEs:%d, Max SGEs: %d\n",
                    ioc->name, stmioc->max_sges_in_ta_message, stmioc->ta_max_sges);

            
            // Use SAS_IOUNIT_PAGE1 to map the available target ports and phys
            sas_page1 = kzalloc(MPT3SAS_SASIOU1_SZ,
                                GFP_KERNEL);
            if ( !sas_page1 )
            {
                pr_err(MPT3SAS_FMT "%s(): failed kzmalloc\n",
                       ioc->name, __func__);
                rc = -ENOMEM;
                goto errout;
            }
            rc = mpt3sas_config_get_sas_iounit_pg1( ioc,
                                                   &mpi_reply,
                                                   sas_page1,
                                                   MPT3SAS_SASIOU1_SZ );
            if (rc)
            {
                pr_err(MPT3SAS_FMT "%s():failed config_get_iounit_pg1 with (iocstatus=0x%08x)\n",
                       ioc->name, __func__, le16_to_cpu(mpi_reply.IOCStatus));
                goto errout;
            }
            
            last_port = -1;
            for ( i=0; i < sas_page1->NumPhys; i++ )
            {
                if ( MPI2_SAS_DEVICE_INFO_SSP_TARGET & sas_page1->PhyData[i].ControllerPhyDeviceInfo )
                {
                    if ( MPI2_SASIOUNIT1_PORT_FLAGS_AUTO_PORT_CONFIG & sas_page1->PhyData[i].PortFlags )
                    {
                        pr_warn(" HBA is misconfigured, target phy %d on autoconf port - ignored!!\n",i);
                    }
                    else
                    {
                        // allocate a phy tracking object
                        pphy = kzalloc( sizeof(struct _mpt3stm_phy),
                                       GFP_KERNEL );
                        pphy->phy_num = i;
                        pphy->port_num = sas_page1->PhyData[i].Port;
                        pphy->phy_target_state = (MPI2_SASIOUNIT1_PHYFLAGS_PHY_DISABLE & sas_page1->PhyData[i].PhyFlags)?
                            MPT3STM_PORT_STATE_DISABLED:MPT3STM_PORT_STATE_ENABLED;
                        if (pphy->phy_target_state != stmioc->enabled)
                        {
                            VDPRINTK("setting ioc:%d phy:%d caused state change to %sbled\n",
                                    _num_iocs,i,
                                    (MPT3STM_PORT_STATE_ENABLED == pphy->phy_target_state)?"Ena":"Dis");
                            stmioc->enabled = pphy->phy_target_state;
                        }
                        list_add_tail( &pphy->list,
                                      &stmioc->phy_list );
                        VDPRINTK(" phy: %d added to target list\n",i);
                        if (sas_page1->PhyData[i].Port != last_port)
                        {
                            last_port = sas_page1->PhyData[i].Port;
                            _ports.port[_ports.num_ports].PhysPort = sas_page1->PhyData[i].Port;
                            _ports.port[_ports.num_ports].iocidx = _num_iocs;
                            pr_info(MPT3SAS_FMT " port: %d added on ioc %d\n",
                                    ioc->name,
                                    _ports.num_ports,
                                    _num_iocs);
                            _ports.num_ports++;
                        }
                    }
                }
            }
            kfree(sas_page1);
            sas_page1 = NULL;
            
            // Register our callback routines with the MPI driver so it can route
            // message completions to us
            stmioc->stm_post_cb_idx = mpt3sas_base_register_callback_handler(_stm_msg_callback);
            BUG_ON(!stmioc->stm_post_cb_idx);
            stmioc->stm_abort_cb_idx = mpt3sas_base_register_callback_handler(_stm_msg_callback);
            BUG_ON(!stmioc->stm_abort_cb_idx);
            stmioc->stm_assist_cb_idx = mpt3sas_base_register_callback_handler(_stm_assist_err_callback);
            BUG_ON(!stmioc->stm_assist_cb_idx);
            
            // Post the command buffer addresses to the IOC so that it can deliver
            // SSP frames to us!
            rc = _cmdbuf_base_post(stmioc);
            if (rc)
            {
                pr_err(MPT3SAS_FMT "%s(): failed _cmdbuf_base_post()\n",
                       ioc->name, __func__);
                rc = -EFAULT;
                goto errout;
            }
            
            _num_iocs++;
            
        }

    }
    
    return 0;
    
errout:
    if (ioc)
        ioc->stmpriv = NULL;
    if (stmioc)
        stmioc->ioc = NULL;
    
    if (sas_page1)
        kfree(sas_page1);
    
    _stm_exit();
    
    return rc;
}



module_init(_stm_init);
module_exit(_stm_exit);

