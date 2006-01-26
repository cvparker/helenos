/*
 * Copyright (C) 2005 - 2006 Jakub Jermar
 * Copyright (C) 2006 Jakub Vana
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ia64_PAGE_H__
#define __ia64_PAGE_H__

#include <arch/mm/frame.h>
#include <genarch/mm/page_ht.h>
#include <arch/types.h>
#include <typedefs.h>
#include <debug.h>

#define PAGE_SIZE	FRAME_SIZE
#define PAGE_WIDTH	FRAME_WIDTH

#define KA2PA(x)	((__address) (x))
#define PA2KA(x)	((__address) (x))

#define GET_PTL0_ADDRESS_ARCH()			((pte_t *) 0)
#define SET_PTL0_ADDRESS_ARCH(ptl0)

/** Implementation of page hash table interface. */
#define HT_ENTRIES_ARCH			0
#define HT_HASH_ARCH(page, asid)	0
#define HT_COMPARE_ARCH(page, asid, t)	0
#define HT_SLOT_EMPTY_ARCH(t)		1
#define HT_INVALIDATE_SLOT_ARCH(t)
#define HT_GET_NEXT_ARCH(t)		0
#define HT_SET_NEXT_ARCH(t, s)
#define HT_SET_RECORD_ARCH(t, page, asid, frame, flags)

#define VRN_KERNEL	 		0
#define REGION_REGISTERS 		8

#define VHPT_WIDTH 			20         	/* 1M */
#define VHPT_SIZE 			(1<<VHPT_WIDTH)

#define VHPT_BASE 			page_ht		/* Must be aligned to VHPT_SIZE */

struct vhpt_tag_info {
	unsigned long long tag : 63;
	unsigned ti : 1;
} __attribute__ ((packed));

union vhpt_tag {
	struct vhpt_tag_info tag_info;
	unsigned tag_word;
};

struct vhpt_entry_present {
	/* Word 0 */
	unsigned p : 1;
	unsigned : 1;
	unsigned ma : 3;
	unsigned a : 1;
	unsigned d : 1;
	unsigned pl : 2;
	unsigned ar : 3;
	unsigned long long ppn : 38;
	unsigned : 2;
	unsigned ed : 1;
	unsigned ig1 : 11;
	
	/* Word 1 */
	unsigned : 2;
	unsigned ps : 6;
	unsigned key : 24;
	unsigned : 32;
	
	/* Word 2 */
	union vhpt_tag tag;
	
	/* Word 3 */													
	unsigned long long next : 64;	/**< Collision chain next pointer. */
} __attribute__ ((packed));

struct vhpt_entry_not_present {
	/* Word 0 */
	unsigned p : 1;
	unsigned long long ig0 : 52;
	unsigned ig1 : 11;
	
	/* Word 1 */
	unsigned : 2;
	unsigned ps : 6;
	unsigned long long ig2 : 56;

	/* Word 2 */
	union vhpt_tag tag;
	
	/* Word 3 */													
	unsigned long long next : 64;	/**< Collision chain next pointer. */
	
} __attribute__ ((packed));

typedef union vhpt_entry {
	struct vhpt_entry_present present;
	struct vhpt_entry_not_present not_present;
} vhpt_entry;

struct region_register_map {
	unsigned ve : 1;
	unsigned : 1;
	unsigned ps : 6;
	unsigned rid : 24;
	unsigned : 32;
} __attribute__ ((packed));

typedef union region_register {
	struct region_register_map map;
	unsigned long long word;
} region_register;

struct pta_register_map {
	unsigned ve : 1;
	unsigned : 1;
	unsigned size : 6;
	unsigned vf : 1;
	unsigned : 6;
	unsigned long long base : 49;
} __attribute__ ((packed));

typedef union pta_register {
	struct pta_register_map map;
	__u64 word;
} pta_register;

/** Return Translation Hashed Entry Address.
 *
 * VRN bits are used to read RID (ASID) from one
 * of the eight region registers registers.
 *
 * @param va Virtual address including VRN bits.
 *
 * @return Address of the head of VHPT collision chain.
 */
static inline __u64 thash(__u64 va)
{
	__u64 ret;

	__asm__ volatile ("thash %0 = %1\n" : "=r" (ret) : "r" (va));

	return ret;
}

/** Return Translation Hashed Entry Tag.
 *
 * VRN bits are used to read RID (ASID) from one
 * of the eight region registers.
 *
 * @param va Virtual address including VRN bits.
 *
 * @return The unique tag for VPN and RID in the collision chain returned by thash().
 */
static inline __u64 ttag(__u64 va)
{
	__u64 ret;

	__asm__ volatile ("ttag %0 = %1\n" : "=r" (ret) : "r" (va));

	return ret;
}

/** Read Region Register.
 *
 * @param i Region register index.
 *
 * @return Current contents of rr[i].
 */
static inline __u64 rr_read(index_t i)
{
	__u64 ret;
	
//	ASSERT(i < REGION_REGISTERS);
	__asm__ volatile ("mov %0 = rr[%1]\n" : "=r" (ret) : "r" (i));
	
	return ret;
}


/** Write Region Register.
 *
 * @param i Region register index.
 * @param v Value to be written to rr[i].
 */
static inline void rr_write(index_t i, __u64 v)
{
//	ASSERT(i < REGION_REGISTERS);
	__asm__ volatile ("mov rr[%0] = %1\n" : : "r" (i), "r" (v));
}
 
/** Read Page Table Register.
 *
 * @return Current value stored in PTA.
 */
static inline __u64 pta_read(void)
{
	__u64 ret;
	
	__asm__ volatile ("mov %0 = cr.pta\n" : "=r" (ret));
	
	return ret;
}

/** Write Page Table Register.
 *
 * @param v New value to be stored in PTA.
 */
static inline void pta_write(__u64 v)
{
	__asm__ volatile ("mov cr.pta = %0\n" : : "r" (v));
}

extern void page_arch_init(void);

#endif
