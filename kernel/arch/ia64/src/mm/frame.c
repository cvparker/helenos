/*
 * Copyright (c) 2005 Jakub Jermar
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

/** @addtogroup ia64mm	
 * @{
 */
/** @file
 */

#include <arch/mm/frame.h>
#include <mm/frame.h>
#include <config.h>
#include <panic.h>

/*
 * This is Ski-specific and certainly not sufficient
 * for real ia64 systems that provide memory map.
 */
#define MEMORY_SIZE	(64 * 1024 * 1024)
#define MEMORY_BASE	(0 * 64 * 1024 * 1024)

#define ONE_TO_ONE_MAPPING_SIZE (256*1048576) // Mapped at start

#define ROM_BASE	0xa0000               //For ski
#define ROM_SIZE	(384 * 1024)          //For ski
void poke_char(int x,int y,char ch, char c);

uintptr_t last_frame;

void frame_arch_init(void)
{

	if(config.cpu_active==1)
	{
		zone_create(MEMORY_BASE >> FRAME_WIDTH, SIZE2FRAMES(MEMORY_SIZE), (MEMORY_SIZE) >> FRAME_WIDTH, 0);
	
		/*
		* Blacklist ROM regions.
		*/
		//frame_mark_unavailable(ADDR2PFN(ROM_BASE), SIZE2FRAMES(ROM_SIZE));

		frame_mark_unavailable(ADDR2PFN(0), SIZE2FRAMES(1048576));
		last_frame=SIZE2FRAMES((VRN_KERNEL<<VRN_SHIFT)+ONE_TO_ONE_MAPPING_SIZE);	
	}	
}

/** @}
 */
