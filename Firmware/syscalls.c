/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include <sys/unistd.h>
#include <board.h>


//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

extern char _end; // provided by the linker script: it's end of statically allocated section, which is where the heap starts.
extern char _heap_end_max; // provided by the linker script
void* _end_ptr = &_end;
void* _heap_end_max_ptr = &_heap_end_max;
void* heap_end_ptr = 0;

/* @brief Increments the program break (aka heap end)
*
* This is called internally by malloc once it runs out
* of heap space. Malloc might expect a contiguous heap,
* so we don't call the FreeRTOS pvPortMalloc here.
* If this function returns -1, malloc will return NULL.
* Note that if this function returns NULL, malloc does not
* consider this as an error and will return the pointer 0x8.
*
* You should still be careful with using malloc though,
* as it does not guarantee thread safety.
*
* @return A pointer to the newly allocated block on success
*         or -1 otherwise.
*/

/** 2023.09.01 zhuhuan
* _sbrk 它用于改变程序的堆栈边界。堆栈和堆是两种用于存储数据的内存区域，通常在 C 和 C++ 中使用。
*  堆是动态分配的，而栈是静态分配的。_sbrk 允许你的程序在运行时增加或减少堆的大小。
*/
intptr_t _sbrk(size_t size) {
    intptr_t ptr;
	{
        uint32_t mask = cpu_enter_critical(); // cpu_enter_critical 是一个函数，用于进入临界区（critical section）在计算机科学中，临界区是一段代码，其中包含对共享资源的访问，需要确保在任何时刻只有一个线程可以执行这段代码。
        if (!heap_end_ptr) // 首先检查 heap_end_ptr 是否为零。如果是，它会将其设置为 _end_ptr 的值
            heap_end_ptr = _end_ptr;
        if (heap_end_ptr + size > _heap_end_max_ptr) { // 然后检查堆是否已经达到了最大值。如果是，它会返回 -1
            ptr = -1;
        } else { // 如果堆还有空间，它会将堆的末尾指针增加 size 字节，并返回指向新分配的内存的指针
            ptr = (intptr_t)heap_end_ptr;
            heap_end_ptr += size;
        }
        cpu_exit_critical(mask); // cpu_exit_critical 是一个函数，用于退出临界区
	}
    return ptr;
}

// _write is defined in communication.cpp


