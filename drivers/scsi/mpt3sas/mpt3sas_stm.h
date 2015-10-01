/*
 * SCSI Target Layer for MPT2.5 (Message Passing Technology) based controllers
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

#ifndef MPT3SAS_TARGET_H_INCLUDED
#define MPT3SAS_TARGET_H_INCLUDED

#include "mpi/mpi2_type.h"
#include "mpi/mpi2.h"
#include "mpi/mpi2_ioc.h"
#include "mpi/mpi2_sas.h"
#include "mpi/mpi2_targ.h"

#include <scsi/scsi.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>

/* driver versioning info */
#define MPT3STM_DRIVER_NAME		"mpt3stm"
#define MPT3STM_AUTHOR          "Exact Assembly, LLC <info@xassembly.com>"
#define MPT3STM_DESCRIPTION         "LSI SAS 3.0 SCSI Target Driver"
#define MPT3STM_DRIVER_VERSION		"04.100.00.00"
#define MPT3STM_MAJOR_VERSION		4
#define MPT3STM_MINOR_VERSION		100
#define MPT3STM_BUILD_VERSION		0
#define MPT3STM_RELEASE_VERSION     00


#define SAS_DATAPRES_NODATA     0
#define SAS_DATAPRES_RESPONSE   1
#define SAS_DATAPRES_SENSE      2

#define SAS_SSP_FRAMETYPE_TASK  (0x16)
#define SAS_SSP_FRAMETYPE_CMD   (0x06)

/**
 * @struct MPT3STM_DISPATCH
 * @brief A copy of this struct is required for mpt3stm_register, and it consists of
 *      a set of function callbacks used to dispatch asynch firmware events,
 *      asynchronous target mode command buffers, and to process the returns from MPT3
 *      commands
 * @member stm_event - This callback occurs when a firmware (MPT2) event
 *      occurs, for example a TOPOLOGY_CHANGE event, allowing the TGT consumer to
 *      respond to firmware events and even filter them if necessary, the callback
 *      function returns a 0 to indicate that the event should be filtered, 1 otherwise
 * @member stm_command - This callback occurs when a remote host sends a
 *      command to us
 * @member stm_assist_complete - This callback occurs when a TARGET_ASSIST
 *                          command is completed, if the command failed then
 *                          the mpi_reply field is a pointer to the frame, NULL
 *                          otherwise
 * @member stm_status_complete - This callback occurs when a TARGET_STATUS
 *                          command is completed, if the command failed then
 *                          the mpi_reply field is a pointer to the frame, NULL
 *                          otherwise
 * @member stm_watchdog - The driver internal watchdog loop will dispatch to
 *                          this routine whenever it is called to allow the
 *                          client to detect internal lockups.
 * @member stm_reset- The driver internal reset logic notifies us when it is
 *                          invoked, pass along to client.
 *
 * @member target_msg_complete - This callback occurs when a TARGET_ABORT/REPOST command is
 *      completed (the reply frame is the originally submitted smid)
 * @member driver - human readable name of the attached driver
 */
typedef struct {
    void  (*stm_event) (u8 iocidx, MPI2_EVENT_NOTIFICATION_REPLY *pevent);
    void (*stm_command) (u8 iocidx, u8 PhyNum, u16 IoIndex,
                         u16 InitiatorDevHandle);
    void (*stm_assist_complete) (u8 iocidx, u16 IoIndex, u8 IOCSequenceNum,
                                 Mpi2TargetErrorReply_t* mpi_reply);
    void (*stm_status_complete) (u8 iocidx, u16 IoIndex,
                                 Mpi2TargetErrorReply_t* mpi_reply);
    void (*stm_watchdog) (u8 iocidx);
    void (*stm_reset) (u8 iocidx, int reset_phase);
    char driver[32];
} MPT3STM_DISPATCH;

// number of bytes in a command frame coming from IOC - see MPT2.5 spec
#define MPT3STM_CMD_BYTES   (64)
// number of bytes in a response buffer going to IOC - see MPT2.5 spec
#define MPT3STM_RSP_BYTES   (192)

/* The IOC writes either a COMMAND type frame or a TASK type frame when
   posting a SCSI target command buffer */
union _cmd_frame {
    u8 bytes[MPT3STM_CMD_BYTES];
    Mpi2TargetSspCmdBuffer  command;
    Mpi2TargetSspTaskBuffer task;
};

union _rsp_frame {
    u8 bytes[MPT3STM_RSP_BYTES];
    Mpi2TargetSspRspIu_t rspiu;
};

/* We allocate a set of command/response frames in DMA accessible/PCI memory
   so that the IOC can send us commands and we can send back reply (SCSI status) */
typedef struct {
    union _cmd_frame  cmd;
    union _rsp_frame  rsp;
    struct list_head   recycle_list;
} MPT3STM_CMD_BUFF;

#define MPT3STM_PORT_STATE_DISABLED    (0)
#define MPT3STM_PORT_STATE_ENABLED     (1)

#define MPT3STM_MAX_PORTS           (8)
#define MPT3STM_MAX_IOCS            (8)

struct mpt3stm_port {
    u8          PhysPort;
    u16         PhyNum;
    u8          iocidx;
};

struct mpt3stm_ports {
    uint        num_ports;
    struct mpt3stm_port port[MPT3STM_MAX_PORTS];
};

#define MPT3SAS_MAX_CMDBUFFS    (128)
#define MPT3SAS_SASIOU1_SZ (sizeof(Mpi2SasIOUnitPage1_t) + sizeof(MPI2_SAS_IO_UNIT1_PHY_DATA)*15)

// forward decl
struct _mpt3stm_ioc;

struct _mpt3stm_phy {
    u8 port_num;
    u8 phy_num;
    u8 phy_target_state;
    struct list_head list;
};

/* flags definitions */
#define MPT3STM_FLAGS_REPOST	0x01		/* repost command buffer */
#define MPT3STM_FLAGS_WRITE     0x02		/* this is a write request */
#define MPT3STM_FLAGS_STATUS	0x04		/* indicate good status */


/* SCSI target-mode API */
/**
 * @func mpt3stm_get_adapter_name
 * @brief Get our IOC adapter name
 *
 * @param iocidx - ioc index for dispatch
 *
 * @return pointer to adapter name of ioc
 */
char *mpt3stm_get_adapter_name(u8 iocidx);
/**
 * @func pli2tgt_get_pci_device
 * @brief Get our IOC adapter name
 *
 * @param iocidx - ioc index for dispatch
 *
 * @return pointer to struct pci_dev or NULL
 */
struct pci_dev *mpt3stm_get_pci_dev(u8 iocidx);


/**
 * @func mpt3sas_base_get_iocstate
 * @brief Get our IOC doorbell register
 *
 * @param iocidx - ioc index for dispatch
 * @param cooked 0 for raw value, 1 for masked (IOCState)
 *
 * @return value of doorbell register
 */
u32 mpt3stm_get_iocstate(u8 iocidx, int cooked);

/**
 * @func: mpt3stm_get_cmdbuff
 * @brief This function call gives the target driver access to the command
 *          buffer used for incoming command frames and outgoing response
 *          frames.
 *
 * @param ioidx The IOC assigned IoIndex
 * @returns a pointer to the command/response buffer
 */
MPT3STM_CMD_BUFF* mpt3stm_get_cmdbuff(u8 iocidx, u16 ioidx);

/**
 * @func mpt3stm_get_portmap
 * @brief Get our port mapping information message for the driver
 *
 * @param portmapbuffer pointer to an allocated buffer to hold the mapping
 */
void mpt3stm_get_portmap( struct mpt3stm_ports * portmapbuffer );

void* mpt3stm_ioc_get_priv( u8 iocidx );
void mpt3stm_ioc_set_priv( u8 iocidx, void* priv);

int mpt3stm_register(u8 iocidx, MPT3STM_DISPATCH *dispatch );
void mpt3stm_deregister( u8 iocidx );

/**
 * @func mpt3stm_enable
 * @brief Enable or disable STM functionality on an IOC
 *
 * @param iocidx the mpt3stm assigned IOC index number reported in portmap
 * @enable: 0 to disable, 1 to enable
 *
 * @returns 0 or error code
 */
int mpt3stm_enable(u8 iocidx, u8 enable );

/**
 * @func mpt3stm_is_enabled
 * @brief Check to see if target ports are enabled
 *
 * @param iocidx the mpt3stm assigned IOC index number reported in portmap
 *
 * @returns 1 if the IOC target PHYs have been enabled
 */
int mpt3stm_is_enabled(u8 iocidx);

/**
 * @func mpt3stm_get_iocfacts
 * @brief Get our IOC Facts message for the driver
 *
 * @param iocidx - ioc index for dispatch
 * @param *buff pointer to the MPI2_IOCFACTS_REPLY buffer
 *
 */
int mpt3stm_get_iocfacts(u8 iocidx, MPI2_IOC_FACTS_REPLY *pIOCFacts);

/**
 * @func mpt3stm_get_ncmds
 * @brief Get the number of command buffers needed
 *
 * @param iocidx the mpt3stm assigned IOC index number reported in portmap
 * @returns - the number of command buffers the IOC expects
 */
u16 mpt3stm_get_ncmds(u8 iocidx);

/**
 * @func mpt3stm_get_max_sges
 * @brief Get the maximum number of SGEs handleable in a TargetAssist
 *
 * @param iocidx the mpt3stm assigned IOC index number reported in portmap
 * @returns - the number of SGL entries the driver can support
 */
u8 mpt3stm_get_max_sges(u8 iocidx);

/**
 * @func mpt3stm_remove_device
 * @brief Issue the correct SAS IOUNIT CONTROL command to clear the IOC data
 *          structures associated with this DevHandle (required when a device
 *          is reported Not-Responding)
 * @param
 */
int mpt3stm_remove_device(u8 iocidx, u16 DevHandle);

/**
 * @func mpt3sas_stm_abort_all
 * @brief Send a Command Buffer Post request
 *
 * @param iocidx the mpt3stm assigned IOC index number reported in portmap
 * @returns 0 or error code
 */
int mpt3stm_abort_all(u8 iocidx);

/**
 * @func mpt3stm_release_command
 * @brief Return a target command buffer to the IOC for later use
 *
 * @param iocidx The STM assigned ioc ID
 * @param IoIndex the ID of the SSP frame this response corresponds to
 *
 * @returns 0 - success, else error
 */
int
mpt3stm_release_command(u8 iocidx, u16 IoIndex);

/**
 * @func mpt3stm_send_status
 * @brief Create and post a target status request
 *
 * @param portid The STM assigned port ID
 * @param StatusFlags MPT2.5 specified
 * @param QueueTag Defined by SCSI SAM-4 (set to 0 if not a tagged command)
 * @param InitiatorConnectionTag from the incoming SSP frame
 * @param IoIndex the ID of the SSP frame this response corresponds to
 *
 * @returns 0 - success, else error
 */
int
mpt3stm_send_status(u8 iocidx, u8 StatusFlags, u16 QueueTag,
    u16 InitiatorConnectionTag, u16 IoIndex );

/**
 * @func mpt3stm_assist
 * @brief Create and post a target assist request
 *
 * @param iocidx the index of the IOC as reported in port_map
 * @param StatusFlags
 * @param QueueTag
 * @param InitiatorConnectionTag
 * @param IoIndex
 * @param SequenceNumber
 * @param SkipCount
 * @param DataLength
 * @param sg
 * @param nents
 * @param dir
 *
 * @returns 0 - success, else error
 */
int
mpt3stm_assist( u8 iocidx, u8 AssistFlags, u16 QueueTag,
               u16 InitiatorConnectionTag, u16 IoIndex, u8 SequenceNumber,
               u32 SkipCount, u32 DataLength, struct scatterlist *sg,
               int nents, enum dma_data_direction dir);

#endif // MPT3SAS_TARGET_H_INCLUDED


