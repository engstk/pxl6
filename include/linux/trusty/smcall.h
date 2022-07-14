/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2013-2014 Google Inc. All rights reserved
 *
 * Trusty and TF-A also have a copy of this header.
 * Please keep the copies in sync.
 */
#ifndef __LINUX_TRUSTY_SMCALL_H
#define __LINUX_TRUSTY_SMCALL_H

#define SMC_NUM_ENTITIES	64
#define SMC_NUM_ARGS		4
#define SMC_NUM_PARAMS		(SMC_NUM_ARGS - 1)

#define SMC_IS_FASTCALL(smc_nr)	((smc_nr) & 0x80000000)
#define SMC_IS_SMC64(smc_nr)	((smc_nr) & 0x40000000)
#define SMC_ENTITY(smc_nr)	(((smc_nr) & 0x3F000000) >> 24)
#define SMC_FUNCTION(smc_nr)	((smc_nr) & 0x0000FFFF)

#define SMC_NR(entity, fn, fastcall, smc64) ((((fastcall) & 0x1U) << 31) | \
					     (((smc64) & 0x1U) << 30) | \
					     (((entity) & 0x3FU) << 24) | \
					     ((fn) & 0xFFFFU) \
					    )

#define SMC_FASTCALL_NR(entity, fn)	SMC_NR((entity), (fn), 1, 0)
#define SMC_STDCALL_NR(entity, fn)	SMC_NR((entity), (fn), 0, 0)
#define SMC_FASTCALL64_NR(entity, fn)	SMC_NR((entity), (fn), 1, 1)
#define SMC_STDCALL64_NR(entity, fn)	SMC_NR((entity), (fn), 0, 1)

#define	SMC_ENTITY_ARCH			0	/* ARM Architecture calls */
#define	SMC_ENTITY_CPU			1	/* CPU Service calls */
#define	SMC_ENTITY_SIP			2	/* SIP Service calls */
#define	SMC_ENTITY_OEM			3	/* OEM Service calls */
#define	SMC_ENTITY_STD			4	/* Standard Service calls */
#define	SMC_ENTITY_RESERVED		5	/* Reserved for future use */
#define	SMC_ENTITY_TRUSTED_APP		48	/* Trusted Application calls */
#define	SMC_ENTITY_TRUSTED_OS		50	/* Trusted OS calls */
#define	SMC_ENTITY_LOGGING		51	/* Used for secure -> nonsecure logging */
#define	SMC_ENTITY_TEST			52	/* Used for secure -> nonsecure tests */
#define	SMC_ENTITY_SECURE_MONITOR	60	/* Trusted OS calls internal to secure monitor */

/* FC = Fast call, SC = Standard call */
#define SMC_SC_RESTART_LAST	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 0)
#define SMC_SC_LOCKED_NOP	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 1)

/**
 * SMC_SC_RESTART_FIQ - Re-enter trusty after it was interrupted by an fiq
 *
 * No arguments, no return value.
 *
 * Re-enter trusty after returning to ns to process an fiq. Must be called iff
 * trusty returns SM_ERR_FIQ_INTERRUPTED.
 *
 * Enable by selecting api version TRUSTY_API_VERSION_RESTART_FIQ (1) or later.
 */
#define SMC_SC_RESTART_FIQ	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 2)

/**
 * SMC_SC_NOP - Enter trusty to run pending work.
 *
 * No arguments.
 *
 * Returns SM_ERR_NOP_INTERRUPTED or SM_ERR_NOP_DONE.
 * If SM_ERR_NOP_INTERRUPTED is returned, the call must be repeated.
 *
 * Enable by selecting api version TRUSTY_API_VERSION_SMP (2) or later.
 */
#define SMC_SC_NOP		SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 3)

/*
 * Return from secure os to non-secure os with return value in r1
 */
#define SMC_SC_NS_RETURN	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 0)

#define SMC_FC_RESERVED		SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 0)
#define SMC_FC_FIQ_EXIT		SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 1)
#define SMC_FC_REQUEST_FIQ	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 2)

#define TRUSTY_IRQ_TYPE_NORMAL		(0)
#define TRUSTY_IRQ_TYPE_PER_CPU		(1)
#define TRUSTY_IRQ_TYPE_DOORBELL	(2)
#define SMC_FC_GET_NEXT_IRQ	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 3)

#define SMC_FC_CPU_SUSPEND	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 7)
#define SMC_FC_CPU_RESUME	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 8)

#define SMC_FC_AARCH_SWITCH	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 9)
#define SMC_FC_GET_VERSION_STR	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 10)

/**
 * SMC_FC_API_VERSION - Find and select supported API version.
 *
 * @r1: Version supported by client.
 *
 * Returns version supported by trusty.
 *
 * If multiple versions are supported, the client should start by calling
 * SMC_FC_API_VERSION with the largest version it supports. Trusty will then
 * return a version it supports. If the client does not support the version
 * returned by trusty and the version returned is less than the version
 * requested, repeat the call with the largest supported version less than the
 * last returned version.
 *
 * This call must be made before any calls that are affected by the api version.
 */
#define TRUSTY_API_VERSION_RESTART_FIQ	(1)
#define TRUSTY_API_VERSION_SMP		(2)
#define TRUSTY_API_VERSION_SMP_NOP	(3)
#define TRUSTY_API_VERSION_PHYS_MEM_OBJ	(4)
#define TRUSTY_API_VERSION_MEM_OBJ	(5)
#define TRUSTY_API_VERSION_CURRENT	(5)
#define SMC_FC_API_VERSION	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 11)

/* TRUSTED_OS entity calls */
#define SMC_SC_VIRTIO_GET_DESCR	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 20)
#define SMC_SC_VIRTIO_START	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 21)
#define SMC_SC_VIRTIO_STOP	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 22)

#define SMC_SC_VDEV_RESET	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 23)
#define SMC_SC_VDEV_KICK_VQ	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 24)
#define SMC_NC_VDEV_KICK_VQ	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 25)

#endif /* __LINUX_TRUSTY_SMCALL_H */
