
#ifndef _ASM_X86_MSR_INDEX_H
#define _ASM_X86_MSR_INDEX_H


#define EPT_VIOLATION_EXIT_QUAL_WRITE_BIT 0x2
#define STAT_EXIT_REASON_MAX EXIT_REASON_XSETBV



#define VMCS_GUEST_IA32_PERF_GLOBAL_CTRL	 0x2c04	
#define VMCS_GUEST_IA32_PERF_GLOBAL_CTRL_HIGH 	0x2c05 

#define VMCS_HOST_IA32_PERF_GLOBAL_CTRL		 0x2808
#define VMCS_HOST_IA32_PERF_GLOBAL_CTRL_HIGH	 0x2809

#define MSR_ARCH_PERFMON_PERFCTR0 0xc1

#define LBR_KERNEL_BIT 0 /* do not capture at ring0 */
#define LBR_USER_BIT 1 /* do not capture at ring > 0 */
#define LBR_JCC_BIT 2 /* do not capture conditional branches */
#define LBR_REL_CALL_BIT 3 /* do not capture relative calls */
#define LBR_IND_CALL_BIT 4 /* do not capture indirect calls */
#define LBR_RETURN_BIT 5 /* do not capture near returns */
#define LBR_IND_JMP_BIT 6 /* do not capture indirect jumps */
#define LBR_REL_JMP_BIT 7 /* do not capture relative jumps */
#define LBR_FAR_BIT 8 /* do not capture far branches */

#define LBR_USER (1 << LBR_USER_BIT)
#define LBR_JCC (1 << LBR_JCC_BIT)
#define LBR_REL_CALL (1 << LBR_REL_CALL_BIT)
#define LBR_IND_CALL (1 << LBR_IND_CALL_BIT)
#define LBR_RETURN (1 << LBR_RETURN_BIT)
#define LBR_REL_JMP (1 << LBR_REL_JMP_BIT)
#define LBR_IND_JMP (1 << LBR_IND_JMP_BIT)
#define LBR_FAR (1 << LBR_FAR_BIT)

#define DEBUGCTLMSR_LBR (1UL <<  0) /* last branch recording */
#define DEBUGCTLMSR_BTF (1UL <<  1) /* single-step on branches */
#define DEBUGCTLMSR_TR (1UL <<  6)
#define DEBUGCTLMSR_BTS (1UL <<  7)
#define DEBUGCTLMSR_BTINT (1UL <<  8)
#define DEBUGCTLMSR_BTS_OFF_OS (1UL <<  9)
#define DEBUGCTLMSR_BTS_OFF_USR (1UL << 10)
#define DEBUGCTLMSR_FREEZE_LBRS_ON_PMI (1UL << 11)

#define MSR_CORE_PERF_GLOBAL_OVF_CTRL 0x00000390
#define MSR_P6_EVNTSEL0 0x00000186
#define MSR_P6_EVNTSEL1 0x00000187
#define MSR_CORE_PERF_GLOBAL_CTRL 0x0000038f
#define MSR_LBR_SELECT 0x000001c8
#define MSR_IA32_DEBUGCTLMSR     0x000001d9


#define VMCS_HOST_IA32_DEBUGCTL		0x2c04
#define VMCS_HOST_IA32_DEBUGCTL_HIGH	0x2c05


#define MSR_CORE_PERF_GLOBAL_STATUS 0x0000038e



#define MSR_LBR_TOS			0x000001c9
#define MSR_LBR_NHM_FROM		0x00000680
#define MSR_LBR_NHM_TO			0x000006c0
#define MSR_LBR_CORE_FROM		0x00000040
#define MSR_LBR_CORE_TO			0x00000060




#endif /* _ASM_X86_MSR_INDEX_H */
