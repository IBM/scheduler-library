#pragma once

#ifdef HPVM
extern "C" {
    extern void* __hpvm_launch(void*, ...);
    extern void __hpvm_wait(void*);
    extern void* __hpvm_parallel_section_begin();
    extern void __hpvm_parallel_section_end(void*);
    extern void* __hpvm_task_begin(unsigned,...);
    extern void __hpvm_task_end(void*);
    extern void __hpvm_parallel_loop(unsigned, ...);
    extern void* __hpvm_launch_begin(unsigned, ...);
    extern void __hpvm_launch_end(void*);
    extern void __hpvm_priv(unsigned, ...);
    extern void __hpvm__isNonZeroLoop(long, ...);
}
#endif
