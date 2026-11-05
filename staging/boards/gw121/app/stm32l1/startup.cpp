// Start-up for the 121GW app on the STM32L152ZD, entered from the stock
// bootloader: the vector table, Reset_Handler (copy .data, zero .bss,
// constructors, main) and the C++ run-time stubs a no-heap firmware needs.
#include <stdint.h>
#include <stddef.h>

extern "C" {
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
int main(void);
void Reset_Handler(void);
void SysTick_Handler(void);

void Default_Handler(void) { for (;;) {} }
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));

// 16 system entries and the STM32L152xD's 57 interrupts. Every interrupt is
// disabled at start (main.cpp), so only SysTick is used.
typedef void (*Vector)(void);
__attribute__((section(".isr_vector"), used)) const Vector g_vectors[16 + 57] = {
	reinterpret_cast<Vector>(&_estack), Reset_Handler, NMI_Handler, HardFault_Handler,
	MemManage_Handler, BusFault_Handler, UsageFault_Handler, nullptr, nullptr, nullptr, nullptr,
	SVC_Handler, DebugMon_Handler, nullptr, PendSV_Handler, SysTick_Handler,
	// 57 x Default_Handler
#define D4 Default_Handler, Default_Handler, Default_Handler, Default_Handler
	D4, D4, D4, D4, D4, D4, D4, D4, D4, D4, D4, D4, D4, D4, Default_Handler,
#undef D4
};

void Reset_Handler(void) {
	// The bootloader's stack pointer may not be ours.
	__asm volatile ("msr msp, %0" :: "r"(&_estack));
	uint32_t* s = &_sidata;
	for (uint32_t* d = &_sdata; d < &_edata;) *d++ = *s++;
	for (uint32_t* d = &_sbss; d < &_ebss;) *d++ = 0;
	for (auto f = __preinit_array_start; f < __preinit_array_end; f++) (*f)();
	for (auto f = __init_array_start; f < __init_array_end; f++) (*f)();
	main();
	for (;;) {}
}

// No heap, no exceptions: what the C++ run time would otherwise pull in.
void __cxa_pure_virtual(void) { for (;;) {} }
int __cxa_atexit(void (*)(void*), void*, void*) { return 0; }   // statics are never destroyed
void* __dso_handle = nullptr;
}

void operator delete(void*) noexcept {}
void operator delete(void*, size_t) noexcept {}
