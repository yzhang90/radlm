/* vi: set ts=4 sw=4 : -*-  tab-width:4  c-basic-offset:4   -*-  */
/* vim: set comments= cinoptions=\:0,t0,+8,c4,C1 : */

/************************************************************
(C) Copyright 2005-2014 LynuxWorks, Inc. All rights reserved.
(C) Copyright 2008-2009 General Dynamics C4 Systems. All rights reserved.

$Date: 2014-05-15 16:35:06 -0700 (Thu, 15 May 2014) $
$Revision: 5401 $
$URL: svn://txx.lynx.com/svn/lynxsecure-svn/trunk/lynxsk/include/guestos/api.h $
************************************************************/

#if !defined(__guestos_api_h_)
#define __guestos_api_h_

/** @file
	Structures exposed to the guest OS. Note that all structures declared in
	this file must not include padding which would be treated differently by
	i386 and x86_64 ABIs.

	Mostly, this means that there shall be no 64-bit integers preceded by 4
	or more bytes of padding in x86_64 ABI. i386 would then align a 64-bit
	integer on a 32-bit boundary, resulting in different offsets for all
	the following fields compared to x86_64.

	The user of this file shall provide prior to including this file:
	- the definitions for fixed-width integer types (int8_t, uint8_t, ...,
	  int64_t, uint64_t)
*/

/*
	GCC may be configured to align 64-bit integers on either 4- or 8-byte
	boundary. To avoid conflicts between different compilers (e.g. between
	CDK and host versions), use naturally aligned 8-byte integers in
	potentially shared structures. A new type is declared for that, auint64_t.

	This file may be included into a source file that may or may not include
	the <common/types.h> header. If it was included, use auint64_t from that
	header. Otherwise, provide a local definition.
*/
#if !defined(__auint64_t_defined)
typedef uint64_t auint64_t __attribute__((__aligned__(8)));
#endif

/** Number of sub-branch IDs in the API ID */
#define LYNXSECURE_API_NSUB	0

/**
	128-byte LynxSecure API ID. Initializer for the api_id portion of the
	RO page.

	See the API Guide, "RO Page" chapter for the explanation of the release
	numbering scheme.
*/
typedef struct {
	uint16_t main;						///< Main (trunk) API ID
	uint16_t nsub;						///< Number of sub-IDs
	struct {
		uint8_t branch;					///< Unique ID for the branch from main API ID
		uint8_t rev;					///< Revision of the API on the branch
	} sub[LYNXSECURE_API_NSUB];			///< Sub-branch API ID

	/// Padding for the structure to end on 8-byte boundary.
	uint8_t __pad[6 - (2 * LYNXSECURE_API_NSUB + 2) % 8];
} ls_api_id_t;

/** Initializer for current API ID */
#define LYNXSECURE_API_ID { \
	.main = 0x604, \
	.nsub = LYNXSECURE_API_NSUB, \
	.sub = { \
	}, \
}


/**
	Exported Resource Name Identifier Type.
	Its use in the BCV indicates an associated entry in the exported resources
	table which contains a string identifier.
*/
typedef uint32_t ls_ername_id_t;

/** Known indices in the Base Address Register array */
enum {
	/// The start of regular BARs
	LSK_BAR_REGULAR			= 0,
	/// The Expansion ROM BAR
	LSK_BAR_EXPANSION_ROM	= 6,
	/// The start of SR-IOV VF BARs
	LSK_BAR_SRIOV			= 7,
	/// The maximum number of BARs per device
	MAX_BAR					= 13
};

#define MAX_INTRS 256					/**< Maximum number of interrupts */
#define LSK_MAX_INTRS MAX_INTRS
#define MAX_NAME_LEN 64

/** PCI BAR types */
enum {
	/** The BAR is not used OR it's the upper part of a 64-bit memory BAR */
	PCI_BAR_NOTUSED			= 0,
	PCI_IO_RES_TYPE			= 1,	/**< I/O BAR */
	PCI_MEM_RES_TYPE		= 2,	/**< 32-bit non-prefetchable memory BAR */
	PCI_MEM64_RES_TYPE		= 3,	/**< 64-bit non-prefetchable memory BAR */
	PCI_MEMPF_RES_TYPE		= 4,	/**< 32-bit prefetchable memory BAR */
	PCI_MEMPF64_RES_TYPE	= 5,	/**< 64-bit prefetchable memory BAR */
};

#define NO_IRQ				(-1)	/**< Device has no IRQ */

/** The BAR structure presented to the subject */
struct __attribute__((aligned(8))) s_bar {
	auint64_t host_addr;
	auint64_t guest_addr;
	auint64_t size;
	uint32_t type;
	uint32_t visible_value;
	uint32_t map_idx;
};

/** Subject-visible device types */
enum {
	LSK_DEV_TYPE_LEGACY			= 0,	///< [devices] Legacy device
	LSK_DEV_TYPE_PCI_END_DEV	= 1,	///< [devices] PCI endpoint
	LSK_DEV_TYPE_PCI_BRIDGE		= 2,	///< [vdevices] PCI or host bridge
	LSK_DEV_TYPE_PCI_VIRTUAL	= 4,	///< [vdevices] Emulated PCI device
	LSK_DEV_TYPE_PNP_VIRTUAL	= 5,	///< [vdevices] Emulated legacy device
};

/** Device flags */
enum {
	LSK_DEV_FLAG_RESET_ON_RESTART	= 1 << 0,	///< Reset on subject restart
};

/** INTx emulation */
enum emu_intx_e {
	EMU_INTX_NONE		= 0,	///< No emulation needed
	EMU_INTX_STATUS		= 1,	///< Emulate by polling status register
	EMU_INTX_MSI		= 2,	///< Emulate by MSI pending bits
};

/** MSI-X Extension Magic */
#define MSIX_EXT_MAGIC 0x534C

/** A subject-visible structure describing a physical device */
struct __attribute__((aligned(8))) s_device {
	struct s_bar bar[MAX_BAR];	///< BAR tracking data
	uint16_t vendor_id;			///< PCI vendor ID
	uint16_t device_id;			///< PCI device ID
	uint16_t cmd_reg;			///< PCI Command register guest value (cached)
	uint8_t device_type;		///< Device Type
	uint8_t bridge;				///< 0 - legacy, 1 - PCI
	uint8_t bus_no;				///< PCI bus number
	uint8_t dev_no;				///< PCI device number
	uint8_t func_no;			///< PCI function number
	uint8_t emulate_intx;		///< Must FV subject emulate INTx thru poll?
	uint8_t dev_flags;			///< Device flags
	int32_t irq;				///< IRQ; NO_IRQ if no interrupt
	uint32_t msi_irq_map[MAX_INTRS/32];	///< The bitmap of MSI IRQs
	struct {
		uint16_t command;		///< Initial PCI Command register
		uint8_t cacheline_size;	///< Initial PCI Cacheline Size
		uint8_t latency_timer;	///< Initial PCI Latency Timer
		uint16_t dev_ctrl;		///< Initial PCIe Device Control register
		uint16_t dev_ctrl2;		///< Initial PCIe Device Control 2 register
	} initial;					///< Initial guest values of some registers
	char name[MAX_NAME_LEN];	///< The HCV name of the device
};

/** A subject-visible structure describing a virtual device */
struct __attribute__((aligned(8))) s_vdevice {
	struct s_bar bar[MAX_BAR];	///< BAR tracking data
	uint16_t vendor_id;			///< PCI vendor ID
	uint16_t device_id;			///< PCI device ID
	uint8_t device_type;		///< Device Type
	uint8_t bridge;				///< 0 - legacy, 1 - PCI
	uint8_t bus_no;				///< PCI bus number
	uint8_t dev_no;				///< PCI device number
	uint8_t func_no;			///< PCI function number
	int32_t irq;				///< IRQ; NO_IRQ if no interrupt
	uint32_t obj_id;			///< BCV "object ID"
	/// The HCV name of the device, empty string if none
	char name[MAX_NAME_LEN];
	uint8_t config_space[256];	///< Virtual config space
};

/** Memory Map Descriptor */
typedef struct {
	auint64_t addr;		///< Virtual address
	auint64_t size;		///< Mapping size
	uint32_t idx;		///< Array index of corresponding mem_des_t or rmr_t
	uint8_t pad[4];		///< Pad to an 8-byte multiple
} mem_map_t;

/**
	The type for host memory region types.
	Individual bits correspond to different memory region types, and
	a set can be created by setting multiple bits.
*/
typedef enum {
	MEM_TYPE_PML4		= 1 <<  0,	///< Page Table PML4
	MEM_TYPE_PDPT		= 1 <<  1,	///< Page Directory Pointers
	MEM_TYPE_PDT		= 1 <<  2,	///< Page Directory
	MEM_TYPE_PTE		= 1 <<  3,	///< Page Table Entries
	MEM_TYPE_PROGRAM	= 1 <<  4,	///< Subject text, data etc.
	MEM_TYPE_IO			= 1 <<  5,	///< Physical device I/O memory
	MEM_TYPE_SHM		= 1 <<  6,	///< Shared Memory
	MEM_TYPE_MSG		= 1 <<  7,	///< Memory for ring descriptors etc.
	MEM_TYPE_HYPERVISOR	= 1 <<  8,	///< Memory used by hypervisor (internal)
	MEM_TYPE_ROPAGE		= 1 <<  9,	///< RO page mapped into the subject
	MEM_TYPE_ARGPAGE	= 1 << 10,	///< RO page with subject startup args
	MEM_TYPE_BOOT		= 1 << 12,	///< RO boot image
	MEM_TYPE_BOOTSTRAP	= 1 << 13,	///< RO boot loader
	MEM_TYPE_BIOS		= 1 << 15,	///< BIOS memory region
	MEM_TYPE_SHADOW		= 1 << 16,	///< Shadow page tables for FV subject
	MEM_TYPE_BITRESULTS	= 1 << 17,	///< RW region to contain bit results
	MEM_TYPE_DMA_ONLY	= 1 << 18,	///< Only accessible via device DMA (RMRR)
	MEM_TYPE_VIRTUAL_IO	= 1 << 19,	///< Virtual device I/O memory
	MEM_TYPE_SANSRC		= 1 << 20,	///< Auxiliary subject image source
	MEM_TYPE_SANDST		= 1 << 21,	///< Auxiliary subject image destination
	MEM_TYPE_SPECIAL	= 1 << 22,	///< Special memory - e.g. TPM, ePCI ACPI
	MEM_TYPE_ANY		= 0xFFFFFFFF///< All memory types
} mem_type_t;

/**
	Memory types for page table pools. For most types of subjects, these
	regions are mapped read-only regardless of the access mode specified in the
	configuration.
*/
#define MEM_TYPE_PTBL_POOL \
		(MEM_TYPE_PML4|MEM_TYPE_PDPT|MEM_TYPE_PDT|MEM_TYPE_PTE)

/** Legacy alias for MEM_TYPE_PTBL_POOL */
#define MEM_TYPE_MAP_RO MEM_TYPE_PTBL_POOL

/** Memory types that may be mapped into subject's memory by the subject */
#define MEM_TYPE_MAPPABLE ( \
		MEM_TYPE_PML4|MEM_TYPE_PDPT|MEM_TYPE_PDT|MEM_TYPE_PTE| \
		MEM_TYPE_PROGRAM|MEM_TYPE_IO|MEM_TYPE_SHM|MEM_TYPE_MSG| \
		MEM_TYPE_ROPAGE|MEM_TYPE_ARGPAGE| \
		MEM_TYPE_BOOT|MEM_TYPE_BOOTSTRAP|MEM_TYPE_BIOS|MEM_TYPE_BITRESULTS| \
		MEM_TYPE_VIRTUAL_IO|MEM_TYPE_SANSRC|MEM_TYPE_SANDST \
		)

/**
	Memory types that memory pointers passed as subject hypercall arguments may
	validly point to. This limitation is there for security reasons.
*/
#define MEM_TYPE_COPYABLE (MEM_TYPE_PROGRAM|MEM_TYPE_SHM|MEM_TYPE_ROPAGE|\
		MEM_TYPE_VIRTUAL_IO)

/** Memory types that subjects may only have read-only flows to */
#define MEM_TYPE_FLOW_RO (MEM_TYPE_ROPAGE|MEM_TYPE_BOOTSTRAP)

#define MEM_PERM_RO			0x1		/**< Read only memory. */
#define MEM_PERM_RW			0x2		/**< Read-Write memory. */

/** Memory caching types */
typedef enum {
	MEM_CACHING_DEFAULT			= 0,
	MEM_CACHING_UNCACHEABLE		= 1,
	MEM_CACHING_WRITE_COMBINING	= 2,
	MEM_CACHING_WRITE_THROUGH	= 3,
	MEM_CACHING_WRITE_PROTECTED	= 4,
	MEM_CACHING_WRITEBACK		= 5,
	MEM_CACHING_NUM
} mem_caching_t;

/** No fill value for a memory region */
#define MEM_NO_FILL ((uint16_t)-1)

/** No owner for a memory region */
#define MEM_NO_OWNER ((uint32_t)-1)

/**
	Subject Memory Region Descriptor.
	Structure to provide a global description of the memory regions exported
	by LynxSecure to a particular subject. Used in read-only pages.
*/
typedef struct rmr_s {
	/** Physical Start Address. */
	auint64_t start_phys;
	/** Region Size. Size of the segment in bytes. */
	auint64_t size;
	/** Size of the on-disk file read into this segment, if any */
	auint64_t filesize;
	/** Region Type. Not mem_type_t so that rmr_t is compiler-independent */
	uint32_t memory_type;
	/** Exported Resource ID */
	ls_ername_id_t ername_id;
	/** Fill value (optional) */
	uint16_t fill;
	/** Permissions */
	uint8_t perm;
	/** Caching type */
	uint8_t caching;
	/** Owner subject ID, or MEM_NO_OWNER if no owner subject */
	uint32_t owner;
} rmr_t;

/** Page pool type */
typedef enum {
	PT_PML4 = 0,
	PT_PDPT,
	PT_PDT,
	PT_PTE,
	PT_SHADOW,

	PT_MAX
} pool_type_t;

typedef struct ls_obj {
	char mem_descr_name[MAX_NAME_LEN];
	uint32_t index;
	uint8_t pad[4];
} ls_obj_t __attribute__((aligned(8)));

/* Exported Named Message Channel */
typedef struct vl_msg_chn {
	char	vl_msg_chn_name[MAX_NAME_LEN];
	uint32_t	vl_recv_subj_id;
	uint32_t	vl_send_subj_id;
	uint32_t	vl_msg_num;
	uint32_t	vl_msg_size;
	uint32_t	vl_synth_intr;
	ls_ername_id_t vl_ername_id;
	uint32_t	vl_channel_id;
	uint8_t		pad[4];
} vl_msg_chn_t __attribute__((aligned(8)));

/*	An ls_vlist stores a collection of objects
	of the same type, like external device objects.

	One uses a given ls_vlist by first adding the vlist's
	offset from the start of the RO page to the given
	index into that particular vlist:

		addr = ro_addr + vlist->offset + index * vlist->obj_sz

	Where the index ranges from 0 to count-1, where count
	is relative to the particular vlist in question.
*/
typedef struct ls_vlist {
	uint32_t count;		/* number of objects of same type */
						/* indexing done from 0 to count-1 */
	uint32_t offset;	/* the offset from RO page! */
	uint32_t obj_sz;	/* size of the object in this vlist */
} ls_vlist;

/** Host bridge resource description */
typedef struct {
	auint64_t addr;
	auint64_t size;
} ls_host_bridge_resource_t;

/** Host bridge description. */
typedef struct {
	uint8_t bus;		/**< Bus part of the BDF address */
	uint8_t dev;		/**< Device part of BDF address */
	uint8_t func;		/**< Function part of the BDF address */
	uint8_t bus_min;	/**< Start of the range of buses served */
	uint32_t bus_num;	/**< Size of the range of buses served */
	ls_vlist mem;		/**< Memory resources produced by the bridge */
	ls_vlist io;		/**< I/O resources produced by the bridge */
} ls_host_bridge_t;

/**
	Subject Type.
	An enumeration of the types of subjects supported by LynxSecure.
*/
typedef enum ls_subject_type_e {
	LS_32BIT_PARAVIRT = 0,
	LS_64BIT_PARAVIRT,
	LS_FULLVIRT,
	LS_MASTER_IBIT,
	LS_SLAVE_IBIT,
	LS_CBIT
} ls_subject_type_t;

/**
	States of the (optional) per-subject sanitization mode
	of a given subject.
*/
typedef enum {
	LS_SAN_INACTIVE = 0,
	LS_SAN_ACTIVE
} ls_san_mode_t;

/** Subject Physical Page Table types */
typedef enum {
	LYNXSK_SPPT_TYPE_NONE = 0,		/**< No SPPT or not visible to subject */
	LYNXSK_SPPT_TYPE_PAE = 1,		/**< SPPT is 64-bit PAE */
	LYNXSK_SPPT_TYPE_EPT = 2,		/**< SPPT is EPT */
} ls_subject_sppt_type_t;

/** Subject Physical Page Table Flags */
typedef enum {
	LYNXSK_SPPT_FLAG_APIC_PAGE = 1 << 0,	///< Uses an APIC access page
} ls_subject_sppt_flag_t;

/**
	Read Only Page Descriptor.
	Data structure for each subject conveying information necessary for
	virtualization.
*/
typedef struct ls_ro_page {
	ls_api_id_t api_id;				/**< LynxSecure API ID */
	char release_version[16];		/**< LynxSecure release version */
	ls_vlist mem_descriptors;		/**< Available memory regions */
	ls_vlist mem_maps;				/**< Initial mappings */
	ls_vlist name_object_maps;		/**< Object names */
	ls_vlist msg_channels;			/**< Message channels */
	ls_vlist devices;				/**< Available devices */
	ls_vlist vdevices;				/**< Available virtual devices */
	ls_vlist ecm;					/**< ECM H-to-S internal interface */
	ls_vlist vcpu_block;			/**< The VCPU module data block */
	uint32_t subject_id;			/**< The BCV identity of this subject */
	uint32_t subjects_num;			/**< The total number of subjects */
	uint32_t subject_type;			/**< The enumerated type of this subject */
	uint32_t cpus_num;				/**< The # of (V)CPUs in this subject */
	auint64_t tb_per_sec;			/**< Timebase ticks per second */
	uint32_t ticks_per_sec;			/**< PV subject clock ticks per second */
	uint32_t tsc_ticks;				/**< TSC ticks per clock tick */
	uint32_t irq_base;				/**< (PV) Vector corresponding to IRQ #0 */
	uint32_t used_pool[PT_MAX];		/**< Ptbl pool pages used for initial map */
	uint32_t ro_page_size;			/**< RO page size in bytes */
	auint64_t lapic_per_sec;		/**< Physical local APIC ticks per second */
	ls_san_mode_t san_mode;			/**< External sanitization bits */
	uint32_t subject_flags;			/**< bits to pass subject info */
	uint8_t ecm_registered;			/**< For FV subject: executing under ECM */
	struct {
		uint8_t sppt_type;			/**< Type of SPPT in this subject */
		uint8_t sppt_lgpg_mask;		/**< Large page support per SPPT level */
		uint8_t sppt_flags;			/**< SPPT flags */
	} sppt_info;
	struct {
		auint64_t mcfg_addr;		/**< GPA (with respect to bus 0) */
		uint8_t mcfg_start_bus;		/**< Start PCI BUS number */
		uint8_t mcfg_end_bus;		/**< End PCI BUS number */
	} pci_mcfg_info;				/**< PCI MCFG area info */
	ls_vlist host_bridges;			/**< Host bridges in the host */
	auint64_t mdm_addr;				/**< MDM address */
	auint64_t mdm_size;				/**< MDM size */
} ls_ro_page_t;

/** Bit definitions for subject_flags */
enum {
	SUBJECT_FLAG_RESTARTED		= 1 << 0,	///< Restarted at least once
	SUBJECT_FLAG_PCI_MCFG		= 1 << 1,	///< Has PCI MCFG
	SUBJECT_FLAG_NO_UNREAL_MODE	= 1 << 2,	///< No support for x86 Unreal Mode
};

/** Special interrupt vectors for the HVCALL_SEND_IPI hypercall */
enum {
	LYNXSK_SEND_IPI_INIT = (uint32_t)-1,	/**< Reset and halt a VCPU */
	LYNXSK_SEND_IPI_START = (uint32_t)-2,	/**< Resume a VCPU */
};

/** Special value for the HVCALL_SET_HRTIMER_VEC hypercall */
enum {
	LYNXSK_SET_HRTIMER_VEC_NONE = MAX_INTRS,	/**< Disable HR timer */
};

/** Subject EOI types */
typedef enum subject_eoi_type_e {
	/** Subject must issue non-spec EOI (default) */
	LYNXSK_EOI_EXPLICIT = 0,
	/**
		IRQ masked and EOI issued upon injection implicitly,
		subject will unmask later.
	*/
	LYNXSK_EOI_IMPLICIT,
} subject_eoi_type_t;

/** Special values for "PCI bus remap" hypercall */
enum {
	LYNXSK_BUS_UNREACHABLE = (uint32_t)-1,	/**< Ignore accesses to this bus */
};

/** Hypercall numbers */
typedef enum {
	HVCALL_GET_ROPAGE_INFO						= 0,
	/* MM */
	HVCALL_TEST_AND_SET_PTE_BITS_V				= 1,
	HVCALL_UPDATE_PML4							= 2,
	HVCALL_UPDATE_PDPTE							= 3,
	HVCALL_UPDATE_PDTE							= 4,
	HVCALL_UPDATE_PTE							= 5,
	HVCALL_SET_SPPT								= 6,
	HVCALL_MEMORY_COPY							= 7,
	/* SESM */
	HVCALL_SESM_STOP_SUBJECT					= 10,
	HVCALL_SESM_RESTART_SUBJECT					= 11,
	HVCALL_SESM_SUSPEND_SUBJECT					= 12,
	HVCALL_SESM_START_SUBJECT					= 13,
	HVCALL_SESM_RESUME_SUBJECT					= 14,
	HVCALL_SESM_GET_STATE_V						= 17,
	/* Time */
	HVCALL_TIME_SET								= 21,
	HVCALL_TCAL_SET								= 23,
	HVCALL_TIME_GET_V							= 24,
	HVCALL_TCAL_GET_V							= 25,
	HVCALL_TIME_GET_MONOTONIC					= 26,
	/* Message channels */
	HVCALL_MSG_RECV_V							= 32,
	HVCALL_MSG_SEND_V							= 33,
	/* Debug and testing */
	HVCALL_DEBUG								= 41,
	HVCALL_SKDB_ENTER							= 42,
	HVCALL_SKDB_EXECUTE							= 43,
	HVCALL_LT_COPY								= 47,
	HVCALL_SUBJECT_LOG							= 49,
	/* SSM state transitions */
	HVCALL_SSM_GET_STATE_V						= 50,
	HVCALL_SSM_SET_INITIATED_BIT				= 51,
	HVCALL_SSM_SET_LAST_STATE					= 52,
	HVCALL_SSM_SET_MAINTENANCE_INSECURE			= 53,
	HVCALL_SSM_SET_MAINTENANCE_SECURE			= 54,
	HVCALL_SSM_SET_OPERATION					= 55,
	HVCALL_SSM_SET_RESTART						= 56,
	HVCALL_SSM_SET_SHUTDOWN						= 57,
	HVCALL_SSM_SET_SUSPEND_TO_RAM				= 58,
	/* Platform-specific Hypercalls */
	HVCALL_X86_SET_TPR							= 60,
	/* Scheduling */
	HVCALL_FLEX_SCHD_RETURN						= 70,
	HVCALL_FLEX_SCHD_GIVE_ALL					= 71,
	HVCALL_FLEX_SCHD_GIVE_UNTIL_SCT				= 72,
	HVCALL_CHANGE_SCHD_POLICY					= 73,
	HVCALL_TIME_LEFT_MNR_V						= 75,
	HVCALL_GET_SCHD_POLICY_V					= 77,
	HVCALL_STROBE_WATCHDOG						= 79,
	/* Event Capture Mechanism */
	HVCALL_ECM_REGISTER							= 80,
	HVCALL_ECM_RESUME_GUEST						= 81,
	/* Virtual Real Mode */
	HVCALL_REGISTER_SUBJECT_VRM					= 85,
	HVCALL_LEAVE_VRM							= 86,
	/* Audit */
	HVCALL_RETRIEVE_AUDIT_RECORD_V				= 93,
	HVCALL_RETRIEVE_OVERFLOW_AUDIT_RECORD_V		= 94,
	HVCALL_STORE_AUDIT_RECORD_V					= 95,
	/* High-resolution timer */
	HVCALL_SET_HRTIMER							= 100,
	HVCALL_SET_HRTIMER_VEC						= 101,
	/* Interrupt Routing */
	HVCALL_SYNTH_INTR							= 110,
	HVCALL_APIC_EOI								= 111,
	HVCALL_SEND_IPI								= 112,
	HVCALL_ROUTE_INTR							= 113,
	HVCALL_SET_EOI_TYPE							= 114,
	HVCALL_UNMASK_VECTOR						= 115,
	/* Device Configuration Virtualization */
	HVCALL_PCI_CONF_READ						= 120,
	HVCALL_PCI_CONF_WRITE						= 121,
	HVCALL_PCI_BUS_REMAP						= 122,

	MAX_HYPERCALLS = 128
} hvcall_num_t;

/* Hypercall Number of Arguments */
#define _ARGS_HVCALL_GET_ROPAGE_INFO                    1
#define _ARGS_HVCALL_APIC_EOI                           0
#define _ARGS_HVCALL_SEND_IPI                           2
#define _ARGS_HVCALL_UPDATE_PML4                        3
#define _ARGS_HVCALL_UPDATE_PDPTE                       3
#define _ARGS_HVCALL_UPDATE_PDTE                        3
#define _ARGS_HVCALL_UPDATE_PTE                         3
#define _ARGS_HVCALL_SET_SPPT                           1
#define _ARGS_HVCALL_MEMORY_COPY                        3
#define _ARGS_HVCALL_TEST_AND_SET_PTE_BITS_V            3
#define _ARGS_HVCALL_SESM_GET_STATE_V                   2
#define _ARGS_HVCALL_SESM_STOP_SUBJECT                  1
#define _ARGS_HVCALL_SESM_RESTART_SUBJECT               1
#define _ARGS_HVCALL_SESM_SUSPEND_SUBJECT               1
#define _ARGS_HVCALL_SESM_START_SUBJECT                 1
#define _ARGS_HVCALL_SESM_RESUME_SUBJECT                1
#define _ARGS_HVCALL_CHANGE_SCHD_POLICY                 1
#define _ARGS_HVCALL_GET_SCHD_POLICY_V                  1
#define _ARGS_HVCALL_TIME_LEFT_MNR_V                    1
#define _ARGS_HVCALL_TIME_GET_V                         1
#define _ARGS_HVCALL_TIME_GET_MONOTONIC                 1
#define _ARGS_HVCALL_TIME_SET                           2
#define _ARGS_HVCALL_TCAL_GET_V                         1
#define _ARGS_HVCALL_TCAL_SET                           1
#define _ARGS_HVCALL_RETRIEVE_AUDIT_RECORD_V            1
#define _ARGS_HVCALL_RETRIEVE_OVERFLOW_AUDIT_RECORD_V   1
#define _ARGS_HVCALL_STORE_AUDIT_RECORD_V               1
#define _ARGS_HVCALL_MSG_RECV_V                         2
#define _ARGS_HVCALL_MSG_SEND_V                         2
#define _ARGS_HVCALL_SYNTH_INTR                         2
#define _ARGS_HVCALL_DEBUG                              3
#define _ARGS_HVCALL_SKDB_ENTER							0
#define _ARGS_HVCALL_SKDB_EXECUTE						1
#define _ARGS_HVCALL_LT_COPY							3
#define _ARGS_HVCALL_SUBJECT_LOG                        1
#define _ARGS_HVCALL_SSM_GET_STATE_V                    1
#define _ARGS_HVCALL_SSM_SET_INITIATED_BIT              0
#define _ARGS_HVCALL_SSM_SET_LAST_STATE                 0
#define _ARGS_HVCALL_SSM_SET_MAINTENANCE_INSECURE       0
#define _ARGS_HVCALL_SSM_SET_MAINTENANCE_SECURE         0
#define _ARGS_HVCALL_SSM_SET_OPERATION                  0
#define _ARGS_HVCALL_SSM_SET_RESTART                    0
#define _ARGS_HVCALL_SSM_SET_SHUTDOWN                   0
#define _ARGS_HVCALL_SSM_SET_SUSPEND_TO_RAM             0
#define _ARGS_HVCALL_REGISTER_SUBJECT_VRM               2
#define _ARGS_HVCALL_LEAVE_VRM                          0
#define _ARGS_HVCALL_FLEX_SCHD_RETURN                   0    /* return CPU to owner of minor frame */
#define _ARGS_HVCALL_FLEX_SCHD_GIVE_ALL                 1    /* give CPU indefinitely              */
#define _ARGS_HVCALL_FLEX_SCHD_GIVE_UNTIL_SCT           1    /* donate time until end of timeslice */
#define _ARGS_HVCALL_ECM_REGISTER                       2
#define _ARGS_HVCALL_ECM_RESUME_GUEST                   0
#define _ARGS_HVCALL_SET_HRTIMER                        3
#define _ARGS_HVCALL_SET_HRTIMER_VEC                    1
#define _ARGS_HVCALL_ROUTE_INTR                         3
#define _ARGS_HVCALL_SET_EOI_TYPE                       1
#define _ARGS_HVCALL_UNMASK_VECTOR                      1
#define _ARGS_HVCALL_STROBE_WATCHDOG					0
#define _ARGS_HVCALL_PCI_CONF_READ						3
#define _ARGS_HVCALL_PCI_CONF_WRITE						3
#define _ARGS_HVCALL_PCI_BUS_REMAP						2
#define _ARGS_HVCALL_X86_SET_TPR						1

/**
	All possible reasons for auditing.
	These must match the event_types for the HCV, BCV, and runtime code.
	I.e. these are defined by the runtime code and used by the HCV, the BCV,
	and SKH.

	ALL MODULES SHALL ADD THEIR AUDIT EVENT TYPES HERE.
*/
#define AUDIT_EVENTS \
   X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_PERM_DENY, \
                 "CHANGE_SCHED_DENIED",                 REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_UNKNOWN_POLICY, \
                 "CHANGE_SCHED_INVALID",                REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_SUCCESS, \
                 "CHANGE_SCHED_SUCCESS",                REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_ALREADY_IN_PROGRESS, \
                 "CHANGE_SCHED_IN_PROGRESS",            REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_NOT_ALLOWED, \
                 "CHANGE_SCHED_NOT_ALLOWED",            REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SECURITY_POLICY_PERM_DENY,  \
                 "CHANGE_SECURITY_DENIED",              REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SECURITY_POLICY_UNKNOWN_POLICY, \
                 "CHANGE_SECURITY_INVALID",             REQUIRED,   NOT_ALLOWED) \
   X(LYNXSK_ADTEVT_CHANGE_SECURITY_POLICY_SUCCESS, \
                 "CHANGE_SECURITY_SUCCESS",             REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_GET_SECURITY_POLICY_PERM_DENY, \
                 "GET_SECURITY_POLICY_DENIED",          REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_GET_SECURITY_POLICY_BAD_MEM,  \
                 "GET_SECURITY_POLICY_BAD_MEM",         REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_GET_SCHD_PERM_DENY, \
                 "GET_SCHED_DENIED",                    REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_GET_ABSOLUTE_TIME_PERM_DENY, \
                 "GET_ABS_TIME_DENIED",                 REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_ABSOLUTE_TIME_PERM_DENY, \
                 "SET_ABS_TIME_DENIED",                 REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_ABSOLUTE_TIME_INVALID_TIME, \
                 "SET_ABS_TIME_INVALID_TIME",           REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_ABSOLUTE_TIME_SUCCESS, \
                 "SET_ABS_TIME_SUCCESS",                REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_TIME_CALIBRATION_PERM_DENY, \
                 "SET_TIME_CAL_DENIED",                 REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_TIME_CALIBRATION_INVALID_CALIBRATION, \
                 "SET_TIME_CAL_INVALID_CAL",            REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SET_TIME_CALIBRATION_SUCCESS, \
                 "SET_TIME_CAL_SUCCESS",                REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_GET_TIME_CALIBRATION_PERM_DENY, \
                 "GET_TIME_CAL_DENIED",                 REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_INIT_TIME, \
                 "TIME_INIT",							NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_STOP_SUBJECT_PERM_DENY, \
                 "SESM_STOP_SUBJECT_DENIED",            REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_STOP_SUBJECT_FAILURE, \
                 "SESM_STOP_SUBJECT_FAILURE",			REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_SESM_START_SUBJECT_PERM_DENY, \
                 "SESM_START_SUBJECT_DENIED",           REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_START_SUBJECT_FAILURE, \
                 "SESM_START_SUBJECT_FAILURE",			REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_SESM_SUSPEND_SUBJECT_PERM_DENY, \
                 "SESM_SUSPEND_SUBJECT_DENIED",         REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_SUSPEND_SUBJECT_FAILURE, \
                 "SESM_SUSPEND_SUBJECT_FAILURE",		REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_SESM_RESUME_SUBJECT_PERM_DENY, \
                 "SESM_RESUME_SUBJECT_DENIED",          REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_RESUME_SUBJECT_FAILURE, \
                 "SESM_RESUME_SUBJECT_FAILURE",         REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_SESM_RESTART_SUBJECT_PERM_DENY, \
                 "SESM_RESTART_SUBJECT_DENIED",         REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_RESTART_SUBJECT_FAILURE, \
                 "SESM_RESTART_SUBJECT_FAILURE",		REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_SESM_STATUS_PERM_DENY, \
                 "SESM_STATUS_DENIED",                  REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SESM_GET_STATE_PERM_DENY, \
                 "SESM_GET_STATE_BAD_MEM",				REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_AUDIT_RETRIEVE_RECORD_PERM_DENY, \
                 "AUDIT_RETRIEVE_DENIED",               REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_AUDIT_RETRIEVE_OVERFLOW_PERM_DENY, \
                 "AUDIT_RETRIEVE_OVERFLOW_DENIED",      REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_AUDIT_SUBJECT_STORE_RECORD_PERM_DENY, \
                 "AUDIT_STORE_RECORD_DENIED",           REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_AUDIT_SUBJECT_STORE_RECORD_SUCCESS, \
                 "AUDIT_STORE_RECORD_SUCCESS",          REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_MESSAGE_SEND_BAD_MEM, \
                 "MESSAGE_SEND_BAD_MEM",                REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_MESSAGE_SEND_PERM_DENY, \
                 "MESSAGE_SEND_DENIED",                 REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_MESSAGE_SEND_UNKNOWN_CHANNEL, \
                 "MESSAGE_SEND_UNKNOWN_CHANNEL",        REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_MESSAGE_RECEIVE_BAD_MEM, \
                 "MESSAGE_RECEIVE_BAD_MEM",             REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_MESSAGE_RECEIVE_PERM_DENY, \
                 "MESSAGE_RECEIVE_DENIED",              REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_MESSAGE_RECEIVE_UNKNOWN_CHANNEL, \
                 "MESSAGE_RECEIVE_UNKNOWN_CHANNEL",     REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SEND_SYNTH_INTERRUPT_PERM_DENY, \
                 "SEND_SYNTH_INT_DENIED",               REQUIRED,   REQUIRED) \
	X(LYNXSK_ADTEVT_MAINT_MODE_HVCALL_PERM_DENY, \
                 "MAINT_MODE_HVCALL_DENIED",            REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SHUTDOWN_FULL_AUDIT_BUFFER, \
                 "SHUTDOWN_FULL_AUDIT_BUFF",            NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_MAINT_MODE_FULL_AUDIT_BUFFER, \
                 "MAINT_MODE_FULL_AUDIT_BUFF",          NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_CHANGE_SCHD_POLICY_FULL_AUDIT_BUFFER, \
                 "CHANGE_SCHD_FULL_AUDIT_BUFF",         NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_HYPERVISOR_UNKNOWN_HVCALL, \
                 "UNKNOWN_HVCALL",                      REQUIRED,   NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_AUDIT_STARTED, \
                 "AUDIT_STARTED",                       NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_FULL_AUDIT_BUFFER, \
                 "FULL_AUDIT_BUFFER",                   NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_VALIDATION_TRANSITION, \
                 "SSM_VALIDATION_TRANSITION",           NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_OPERATION_TRANSITION, \
                 "SSM_OPERATION_TRANSITION",            NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_MAINTENANCE_INSECURE_TRANSITION, \
                 "SSM_MAINTENANCE_INSECURE_TRANSITION", NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_MAINTENANCE_SECURE_TRANSITION, \
                 "SSM_MAINTENANCE_SECURE_TRANSITION",   NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_IBIT_TRANSITION, \
                 "SSM_IBIT_TRANSITION",					NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_SHUTDOWN_TRANSITION, \
                 "SSM_SHUTDOWN_TRANSITION",             NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_RESTART_TRANSITION, \
                 "SSM_RESTART_TRANSITION",              NOT_ALLOWED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_SSM_SUSPEND_TO_RAM_TRANSITION, \
                 "SSM_SUSPEND_TO_RAM_TRANSITION",       NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_GET_STATE_PERM_DENIED, \
                 "SSM_GET_STATE_HVCALL_DENIED",         REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_INITIATED_BIT_PERM_DENIED, \
                 "SSM_IBIT_HVCALL_DENIED",              REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_LAST_STATE_PERM_DENIED, \
                 "SSM_SET_LAST_STATE_PERM_DENIED",      REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_MAINT_INSECURE_PERM_DENIED, \
                 "SSM_MAINT_ISEC_PERM_DENIED",          REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_MAINT_SECURE_PERM_DENIED, \
                 "SSM_MAINT_SEC_PERM_DENIED",           REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_OPERATION_PERM_DENIED, \
                 "SSM_OPERATION_PERM_DENIED",           REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_RESTART_PERM_DENIED, \
                 "SSM_RESTART_PERM_DENIED",             REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SHUTDOWN_PERM_DENIED, \
                 "SSM_SHUTDOWN_PERM_DENIED",            REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SUSPEND_TO_RAM_PERM_DENIED, \
                 "SSM_SUSPEND_TO_RAM_PERM_DENIED",      REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_LAST_STATE_FAILURE, \
                 "SSM_SET_LAST_STATE_FAILURE",          REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_OPERATION_FAILURE, \
                 "SSM_SET_OPERATION_FAILURE",			REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_SHUTDOWN_FAILURE, \
                 "SSM_SET_SHUTDOWN_FAILURE",			REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_SUSPEND_TO_RAM_FAILURE, \
                 "SSM_SET_SUSPEND_TO_RAM_FAILURE",		REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_RESTART_FAILURE, \
                 "SSM_SET_RESTART_FAILURE",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_IBIT_FAILURE, \
                 "SSM_SET_IBIT_FAILURE",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_MAINT_SECURE_FAILURE, \
                 "SSM_SET_MAINT_SECURE_FAILURE",        REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SSM_SET_MAINT_INSECURE_FAILURE, \
                 "SSM_SET_MAINT_INSECURE_FAILURE",      REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_UPDATE_PDPTE_PERM_DENIED, \
                 "UPDATE_PDPTE_PERM_DENIED",			REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_UPDATE_PDTE_PERM_DENIED, \
                 "UPDATE_PDTE_PERM_DENIED",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_UPDATE_PML4_PERM_DENIED, \
                 "UPDATE_PML4_PERM_DENIED",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_UPDATE_PTE_PERM_DENIED, \
                 "UPDATE_PTE_PERM_DENIED",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_APIC_EOI_PERM_DENIED, \
                 "APIC_EOI_PERM_DENIED",				REQUIRED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_FLEX_SCHD_RETURN_PERM_DENY, \
                 "FLEX_SCHD_RETURN_HVCALL_DENIED",      REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_FLEX_SCHD_GIVE_ALL_PERM_DENY, \
                 "FLEX_SCHD_GIVE_ALL_HVCALL_DENIED",    REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_FLEX_SCHD_GIVE_SCT_PERM_DENY, \
                 "FLEX_SCHD_GIVE_SCT_HVCALL_DENIED",    REQUIRED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_MM_POLICY_VIOLATION, \
                 "MM_POLICY_VIOLATION",                 REQUIRED, REQUIRED) \
    X(LYNXSK_ADTEVT_MM_BAD_MMU_MODE, \
                 "MM_BAD_MMU_MODE",                     REQUIRED, NOT_ALLOWED) \
    X(LYNXSK_ADTEVT_IOMMU_POLICY_VIOLATION, \
                 "IOMMU_POLICY_VIOLATION",              REQUIRED, REQUIRED) \
	X(LYNXSK_ADTEVT_UNKNOWN_HVCALL_PERM_DENIED, \
                 "UNKNOWN_HVCALL_PERM_DENIED",          REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_HRTMR_SET_VEC_INVALID_VEC, \
                 "HRTMR_SET_VEC_INVALID_VEC",			REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_ROUTE_INTR_INVALID_ARG, \
                 "ROUTE_INTR_INVALID_ARG",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_ROUTE_INTR_ADDR_FAULT, \
                 "ROUTE_INTR_ADDR_FAULT",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_ROUTE_INTR_BAD_MEM, \
                 "ROUTE_INTR_BAD_MEM",					REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_ROUTE_INTR_FAILURE, \
                 "ROUTE_INTR_FAILURE",					REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SEND_IPI_INVALID_VCPU, \
                 "SEND_IPI_INVALID_VCPU",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SEND_IPI_INVALID_VEC, \
                 "SEND_IPI_INVALID_VEC",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_VRM_REGISTER_EP_BAD_MEM, \
                 "VRM_REG_EP_BAD_MEM",					REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_LEAVE_VRM_INVALID_CTX, \
                 "LEAVE_VRM_INVALID_CTX",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SUBJECT_LOG_PERM_DENIED, \
                 "SUBJECT_LOG_PERM_DENIED",				REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_CSP_MACHINE_CHECK,  \
                 "CSP_MACHINE_CHECK",                   NOT_ALLOWED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SUBJECT_TRIPLEFAULT,	\
                 "SUBJECT_TRIPLEFAULT",                 REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_SUBJECT_WATCHDOG,	\
                 "SUBJECT_WATCHDOG",                    REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_MDM_GENERIC,    \
	              "MDM_GENERIC",                        REQUIRED, NOT_ALLOWED) \
	X(LYNXSK_ADTEVT_BSP_SPURIOUS_INTERRUPT,    \
	             "BSP_SPURIOUS_INTERRUPT",              NOT_ALLOWED, NOT_ALLOWED) \
    X(NUM_AUDIT_EVENT_TYPES, \
                 "NUM_AUDIT_EVENT_TYPES",               NOT_ALLOWED, NOT_ALLOWED)

// Synonyms for backward compatibility
#define LYNXSK_ADTEVT_SEND_IPI_INVALID_SUBJECT \
			LYNXSK_ADTEVT_SEND_IPI_INVALID_VCPU

enum AUDIT {
#define X(a,b,c,d) a,
	AUDIT_EVENTS
#undef X
};


/** Hypercall return/error codes */

enum {
	/* Class codes */
	LYNXSK_GENERIC_CLASS					= 0x0000,
	LYNXSK_AUDIT_CLASS						= 0x1000,
	LYNXSK_MSP_CLASS						= 0x2000,
	LYNXSK_PDE_CLASS						= 0x4000,
	LYNXSK_HYPERVISOR_CLASS					= 0x5000,
	LYNXSK_SESM_CLASS						= 0x6000,

	/* Generic return values */
	LYNXSK_SUCCESS = LYNXSK_GENERIC_CLASS,							/// 0x0000
	LYNXSK_FAILURE,													/// 0x0001
	LYNXSK_NOT_IMPLEMENTED_INTERFACE_ONLY,							/// 0x0002
	LYNXSK_BAD_MEMORY,												/// 0x0003
	LYNXSK_ADDRESS_FAULT,											/// 0x0004
	LYNXSK_INVALID_ARG,												/// 0x0005
	LYNXSK_NO_SPACE,												/// 0x0006
	LYNXSK_NO_IP_UPDATE,											/// 0x0007
	LYNXSK_NO_HW_SUPPORT,											/// 0x0008

	/* Audit specific errors */
	LYNXSK_AUDIT_EMPTY_AUDIT_BUFFER = LYNXSK_AUDIT_CLASS,			/// 0x1000
	LYNXSK_AUDIT_FULL_AUDIT_BUFFER,									/// 0x1001
	LYNXSK_AUDIT_ACTION_UNKNOWN,									/// 0x1002
	LYNXSK_AUDIT_RINGBUFFER_WRITE_ERROR,							/// 0x1003
	LYNXSK_AUDIT_FILTER_ACTION_ERROR,								/// 0x1004
	LYNXSK_AUDIT_FULL_BUFFER_ACTION_ERROR,							/// 0x1005

	/* MSP specific errors */
	LYNXSK_MSP_UNKNOWN_POLICY = LYNXSK_MSP_CLASS,
	LYNXSK_MSP_CHANGE_ALREADY_IN_PROGRESS,
	LYNXSK_MSP_POLICY_NOT_ALLOWED,

	/* Policy Decision Engine specific errors */
	LYNXSK_PDE_PERMISSION_DENIED = LYNXSK_PDE_CLASS,
	LYNXSK_SSM_PERMISSION_DENIED,

	/* Hypervisor specific errors */
	LYNXSK_HYPERVISOR_BAD_MEMORY = LYNXSK_HYPERVISOR_CLASS,
	LYNXSK_HYPERVISOR_INVALID_ARG,
	LYNXSK_HYPERVISOR_BAD_HVCALL,

	/* SESM specific errors */
	LYNXSK_SESM_STATE_CHANGE_FAILED = LYNXSK_SESM_CLASS,
};

/** Structure used for reading the absolute time */
typedef struct abs_time {
	auint64_t seconds;
	auint64_t nanoseconds;
} abs_time_t;

/** Structure used for getting the RO page info */
typedef struct ropage_info_s {
	auint64_t paddr;		/**< Guest physical address */
	auint64_t initial_addr;	/**< Initially mapped at (GVA for PV, GPA for FV)*/
	auint64_t size;			/**< Size in bytes */
	auint64_t dummy;		/**< For future expansion */
} ropage_info_t;

/** Structure used for executing SKDB commands */
typedef struct skdb_exec_s {
	auint64_t input_addr;	/**< Address of the input buffer */
	auint64_t input_size;	/**< Size of the input buffer */
	auint64_t output_addr;	/**< Address of the output buffer */
	auint64_t output_size;	/**< Size of the output buffer */
} skdb_exec_t;

/**
	Enumeration representing each test contained in the BIT suite module.
*/
typedef enum {
	BIT_SUITE_PTE_MOD,
	BIT_SUITE_PTE_INVALIDMOD,
	BIT_SUITE_PTD_MOD,
	BIT_SUITE_PTD_INVALIDMOD,
	BIT_SUITE_PGFAULT_UNMAP,
	BIT_SUITE_PGFAULT_ROMEM,
	BIT_SUITE_TSF_CLEAN,
	BIT_SUITE_GETSEC_EXC,
	BIT_SUITE_XSETBF_EXC,
	BIT_SUITE_INVALIDHVC_EXC,
	BIT_SUITE_VMX_EXC,
	BIT_SUITE_INVALIDIO_RDBACK,
	BIT_SUITE_CR0_RW,
	BIT_SUITE_CR3_RW,
	BIT_SUITE_CR3MOV_EXC,
	BIT_SUITE_CR8MOVFROM_EXC,
	BIT_SUITE_CR8MOVTO_EXC,
	BIT_SUITE_RDMSR_EXC,
	BIT_SUITE_WRMSR_EXC,
	BIT_SUITE_ROMSR_EXC,
	BIT_SUITE_RDPMC_EXC,
	BIT_SUITE_VCPU_CLOCK,
	BIT_SUITE_SCHED_HCV,
	BIT_SUITE_HRTIMER,
	BIT_SUITE_NUM_TESTS
} bit_suite_tests_t;

/**
	Structure which holds the results of a single BIT test case.
*/
typedef struct {
	abs_time_t time;	/**< Time at which the test was executed. */
	uint8_t result;		/**< Test result: 0=PASS, 1=FAIL */
} bit_result_t;

/**
	Structure that holds results for the whole BIT test suite. Access to
	this structure is asynchronous with BIT updates; refer to User's
	Guide for safe reading procedure description.
*/
typedef struct {
	bit_result_t results[BIT_SUITE_NUM_TESTS];
	uint32_t cnt_start;	/**< Incremented counter when this set starts */
	uint32_t cnt_end;	/**< Incremented counter when this set ends */
} bit_set_t;

/**
	Layout of the BIT results memory region. There is one buffer for
	each of IBIT and CBIT.
*/
typedef struct {
	bit_set_t ibit;		/**< IBIT test results */
	bit_set_t cbit;		/**< CBIT test results */
} bit_results_mem_region_t;

/** Size of buffer passed to HVCALL_SUBJECT_LOG */
#define SUBJECT_LOG_SIZE	256

/** The number of low fractional bits in the time calibration value */
#define TCAL_FRAC_BITS		16

/** Subject execution states */
typedef enum ses_e {
	SUBJECT_NOT_INITIALIZED = 0,
	SUBJECT_RUNNING = 1,
	SUBJECT_STOPPED = 2,
	SUBJECT_SUSPENDED = 3,
} ses_t;

#define AUDIT_COMMENT_FIELD_SIZE	228
#define STORED_AUDIT_REC_SIZE_BYTE	256

/**
	An Audit Record that is stored within the Audit Buffer. This type of Audit
	Record is also used when a subject wishes to retrieve an Audit Record from
	the Audit Buffer. This data structure is exactly 256 bytes.
*/
typedef struct lynxsk_stored_audit_record_s {
	/**
		Stores the data for a subject generated Audit Record or stores extra
		information for Audit Records generated internally by LynxSecure
	*/
	char comment_field[AUDIT_COMMENT_FIELD_SIZE];
	/**
		Identifier of an exported resource or LynxSecure object that initiates
		an action which generates an Audit Record.
	*/
	ls_ername_id_t initiator_id;
	/**
		Identifier of an exported resource or LynxSecure object that is the
		target of an action which generates an Audit Record.
	*/
	ls_ername_id_t recipient_id;
	/** Specific event that caused the Audit Record to be generated */
	uint32_t audit_event_type;
	/**
		Structure retrieved from the absolute clock just before the Audit
		Record is stored in the Audit Buffer.
	*/
	abs_time_t timestamp;
} lynxsk_stored_audit_record_t;

/**
	Overflow Audit Record Structure.
	This structure is used by the audit module to store the number and types of
	audit events that have been dropped due to a full audit buffer
*/
typedef struct lynxsk_overflow_audit_record_s {
	/**
		Array of integers where each index corresponds to a unique audit event
		type. Each audit_event_type has a unique integer value that is also the
		index into this array.
	*/
	auint64_t num_overflows[NUM_AUDIT_EVENT_TYPES];
} lynxsk_overflow_audit_record_t;

/**
	SSM States.
	An enumeration of SSM states.
*/
typedef enum {
	SSM_INITIAL_STATE			= 0,
	SSM_STARTUP					= 1,
	SSM_VALIDATION				= 2,
	SSM_OPERATION				= 3,
	SSM_MAINTENANCE_INSECURE	= 4,
	SSM_MAINTENANCE_SECURE		= 5,
	SSM_INITIATED_BIT			= 6,
	SSM_SHUTDOWN				= 7,
	SSM_RESTART					= 8,
	SSM_SUSPEND_TO_RAM			= 9,
	SSM_NUM_STATES
} ssm_state_t;


/** PM power policies */
typedef enum {
	PM_CPU_PWR_POLICY_UNKNOWN,  ///< current policy is unknown
	PM_CPU_PWR_POLICY_SAVE,     ///< set CPU power policy to power save
	PM_CPU_PWR_POLICY_MAX,      ///< set CPU power policy to MAX performance
	PM_CPU_PWR_POLICY_BALANCE,  ///< set CPU power policy to balance
	PM_CPU_PWR_POLICY_CUSTOM,   ///< set CPU power policy user selected P-state
	PM_CPU_PWR_POLICY_ONDEMAND  ///< let LSK to take over CPU PM
} pm_cpu_pwr_policy_t;

/** PM get hypercall commands */
enum {
	PM_CPU_CURR_PWR_POLICY,   ///< get current power policy in use
	PM_CPU_CURR_PERF_STATE,   ///< get current P-state in use
	PM_CPU_TGT_PERF_STATE,    ///< get target P-states to be set
	PM_CPU_PERF_STATES_MAX,   ///< get MAX P-states supported by CPU
};


/*
	VMCALL framework
*/
#define _XARGS_SET_N(nr,args...)	_ARGS_SET_##nr(args)
#define _ARGS_SET_N(nr,args...)		_XARGS_SET_N(nr,args)

#if defined(__x86_64__) /* 64-bit code */

/*
	The hypercall number is passed via RCX.
	Arguments are passed via RDI, RSI and RDX.
	The return value is returned in RAX.
*/
#define _ARGS_SET_0()
#define _ARGS_SET_1(a1) , "D"((uint64_t)(a1))
#define _ARGS_SET_2(a1,a2) _ARGS_SET_1(a1), "S"((uint64_t)(a2))
#define _ARGS_SET_3(a1,a2,a3) _ARGS_SET_2(a1,a2), "d"((uint64_t)(a3))

#define VMCALL(num,args...) ({												\
		uint64_t ret;														\
		asm volatile ("vmcall" : "=a"(ret) :								\
				"c"(num) _ARGS_SET_N(_ARGS_##num, args) : "memory");		\
		ret;																\
	})

#else /* 32-bit code */

/*
	The hypercall number is passed via ECX.
	Arguments are passed via EDX:EAX, EDI:ESI and EBP:EBX.
	The return value is returned in EDX:EAX.
*/
#define _ARGS_SET_0()
#define _ARGS_SET_1(a1) , (a1)
#define _ARGS_SET_2(a1,a2) , (a1), (a2)
#define _ARGS_SET_3(a1,a2,a3) , (a1), (a2), (a3)

#define _XARGS_FNAME_N(nr) __vmcall_##nr##_args
#define _ARGS_FNAME_N(nr) _XARGS_FNAME_N(nr)
#define VMCALL(num,args...) \
		_ARGS_FNAME_N(_ARGS_##num)(num _ARGS_SET_N(_ARGS_##num, args))

static inline uint64_t
__vmcall_0_args(uint32_t hvc_num)
{
	uint64_t ret;

	asm volatile ("vmcall" : "=A"(ret) : "c"(hvc_num) : "memory");
	return ret;
}

static inline uint64_t
__vmcall_1_args(uint32_t hvc_num, uint64_t hvc_arg1)
{
	uint64_t ret;

	asm volatile ("vmcall" : "=A"(ret) : "c"(hvc_num), "A"(hvc_arg1)
			: "memory");
	return ret;
}

static inline uint64_t
__vmcall_2_args(uint32_t hvc_num, uint64_t hvc_arg1, uint64_t hvc_arg2)
{
	uint64_t ret;

	asm volatile ("vmcall" : "=A"(ret) : "c"(hvc_num),
			"A"(hvc_arg1),
			"S"((uint32_t)hvc_arg2),
			"D"((uint32_t)(hvc_arg2 >> 32))
			: "memory");
	return ret;
}

static inline uint64_t
__vmcall_3_args(uint32_t hvc_num,
		uint64_t hvc_arg1, uint64_t hvc_arg2, uint64_t hvc_arg3)
{
	uint64_t ret;
	uint32_t arg3_high = hvc_arg3 >> 32;

	/*
		A slightly more complicated case because we need to preserve EBP,
		which is used by GCC as the frame pointer, and the fact that there
		are no more available registers. In addition, the memory reference
		to the local variable arg3_high may be off ESP (if compiled with
		-fomit-frame-pointer, though it's normally off EBP), so we must use
		it before ESP is modified.
	*/
	asm volatile (
		"pushl	%[arg3_high]		\n"
		"push	%%ebp				\n"
		"movl	4(%%esp), %%ebp		\n"
		"vmcall						\n"
		"pop	%%ebp				\n"
		"add	$4, %%esp			\n"
		: "=A"(ret) : "c"(hvc_num),
			"A"(hvc_arg1),
			"S"((uint32_t)hvc_arg2),
			"D"((uint32_t)(hvc_arg2 >> 32)),
			"b"((uint32_t)hvc_arg3),
			[arg3_high]"m"(arg3_high)
			: "memory");

	return ret;
}
#endif /* 32-bit */

#endif
