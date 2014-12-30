#ifndef __MACH_K3V2_CORTEX_A9_H
#define __MACH_K3V2_CORTEX_A9_H

#include <mach/platform.h>

/*cortex A9 regs base*/
#define REG_CPU_A9GIC_BASE	(REG_BASE_A9PER + 0x100)
#define REG_CPU_A9GLBTIMER_BASE	(REG_BASE_A9PER + 0x200)
#define REG_CPU_A9PRVTIMER_BASE	(REG_BASE_A9PER + 0x600)
#define REG_CPU_A9PRVWDOG_BASE	(REG_BASE_A9PER + 0x600)
#define REG_CPU_A9GICDIST_BASE	(REG_BASE_A9PER + 0x1000)

#define CFG_TIMER_VABASE    IO_ADDRESS(REG_BASE_TIMER0)
#define CFG_TIMER_CONTROL  (TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC | TIMER_CTRL_IE | TIMER_CTRL_32BIT)

#define CFG_TIMER_PRESCALE      1
#define BUSCLK_TO_TIMER_RELOAD(busclk)  (((busclk)/CFG_TIMER_PRESCALE)/HZ)
#define BUSCLK_TO_TIMER0_CLK_HZ(busclk)  ((busclk)/CFG_TIMER_PRESCALE)

#endif /* _HISI_CORTEX_A9_H_ */
