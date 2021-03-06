/*
 * Copyright (c) 2007, 2008 University of Tsukuba
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the University of Tsukuba nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef LOG_TO_GUEST
#include "current.h"
#include "initfunc.h"
#include "mm.h"
#include "putchar.h"
#include "vmmcall.h"

static putchar_func_t old;
u8 *logbuf;
static ulong bufsize, offset;



void
log_putchar (unsigned char c)
{
	if (logbuf) {
		logbuf[offset++] = c;
		if (offset >= bufsize)
			offset = 4;
		asm_lock_incl ((u32 *)(void *)logbuf);
	}
	old (c);
}

static void
log_set_buf (void)
{
	u16 cs;
	ulong physaddr;
    u8 *oldbuf;
    int i;

	current->vmctl.read_sreg_sel (SREG_CS, &cs);
	if (cs & 3)
		return;
	if (logbuf != NULL) {
		unmapmem (logbuf, bufsize);
		logbuf = NULL;
	}
	current->vmctl.read_general_reg (GENERAL_REG_RBX, &physaddr);
	current->vmctl.read_general_reg (GENERAL_REG_RCX, &bufsize);
	if (physaddr == 0 || bufsize == 0)
		return;
	offset = 4;
    oldbuf = logbuf;
	logbuf = mapmem_gphys (physaddr, bufsize, MAPMEM_WRITE);
	if (old == NULL)
		putchar_set_func (log_putchar, &old);


    printf("AAA \n") ;

    //for (i = 0; i< 100; i++)
      //  log_putchar('a');
    //logf("AAAAAAAAAAAAAAAAAAA \n" ) ;
    //panic("SSSSSSSSSSSSSS!!!! \n") ;
    /*
    if (logbuf != NULL) {
		unmapmem (logbuf, bufsize);
		logbuf = oldbuf;
	}
    */
}

static void
vmmcall_log_init (void)
{
	old = NULL;
	logbuf = NULL;
	vmmcall_register ("log_set_buf", log_set_buf);
    vmmcall_register_set_log_buf(log_set_buf) ;
}

INITFUNC ("vmmcal0", vmmcall_log_init);
#endif
