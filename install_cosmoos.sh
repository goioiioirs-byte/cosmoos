#!/bin/bash

# install_cosmoos.sh - создание всей структуры CosmoOS
# Запуск: chmod +x install_cosmoos.sh && ./install_cosmoos.sh

set -e  # Остановка при ошибках

echo "========================================"
echo "   CosmoOS Builder for Linux Mint       "
echo "========================================"
echo ""

# Проверка прав
if [ "$EUID" -ne 0 ]; then 
    echo "Внимание: Для установки зависимостей нужны права суперпользователя"
    echo "Запустите с sudo или установите зависимости вручную"
    read -p "Продолжить без установки зависимостей? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    # Установка зависимостей
    echo "[1/12] Установка зависимостей..."
    apt-get update
    apt-get install -y \
        gcc \
        nasm \
        grub-pc-bin \
        xorriso \
        qemu-system-x86 \
        make \
        mtools \
        gcc-multilib
fi

# Создание структуры директорий
echo "[2/12] Создание структуры проекта..."
mkdir -p cosmoos/{boot,kernel,include,iso/boot/grub}

# 1. Makefile
echo "[3/12] Создание Makefile..."
cat > cosmoos/Makefile << 'EOF'
CC = gcc
CFLAGS = -m32 -ffreestanding -nostdlib -nostdinc -fno-builtin \
         -fno-stack-protector -nostartfiles -nodefaultlibs \
         -Wall -Wextra -I./include -c
LD = ld
LDFLAGS = -m elf_i386 -T link.ld -nostdlib
AS = nasm
ASFLAGS = -f elf32

KERNEL_SOURCES = $(wildcard kernel/*.c)
KERNEL_OBJECTS = $(patsubst kernel/%.c, build/kernel/%.o, $(KERNEL_SOURCES))
ASM_SOURCES = $(wildcard kernel/*.asm)
ASM_OBJECTS = $(patsubst kernel/%.asm, build/kernel/%.o, $(ASM_SOURCES))

all: cosmoos.iso

cosmoos.iso: build/kernel.bin
	@echo "Создание загрузочного ISO..."
	mkdir -p iso/boot/grub
	cp build/kernel.bin iso/boot/
	cp boot/grub.cfg iso/boot/grub/
	grub-mkrescue -o cosmoos.iso iso/ 2>/dev/null
	@echo "ISO создан: cosmoos.iso"

build/kernel.bin: build/kernel/start.o $(KERNEL_OBJECTS)
	@echo "Линковка ядра..."
	$(LD) $(LDFLAGS) -o build/kernel.bin $^

build/kernel/start.o: boot/boot.asm
	@mkdir -p build/kernel
	$(AS) $(ASFLAGS) -o $@ $<

build/kernel/%.o: kernel/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -o $@ $<

build/kernel/%.o: kernel/%.asm
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<

clean:
	rm -rf build iso cosmoos.iso

run: cosmoos.iso
	@echo "Запуск в QEMU..."
	qemu-system-i386 -cdrom cosmoos.iso -m 256M

debug: cosmoos.iso
	@echo "Запуск с отладкой..."
	qemu-system-i386 -cdrom cosmoos.iso -m 256M -s -S &
	gdb -ex "target remote localhost:1234" -ex "symbol-file build/kernel.bin"

.PHONY: all clean run debug
EOF

# 2. Скрипт линковки
echo "[4/12] Создание скрипта линковки..."
cat > cosmoos/link.ld << 'EOF'
ENTRY(start)

SECTIONS {
    . = 0x100000; /* 1MB - стандартная точка входа */

    .text : {
        *(.text)
    }

    .rodata : {
        *(.rodata)
    }

    .data : {
        *(.data)
    }

    .bss : {
        *(COMMON)
        *(.bss)
    }
}
EOF

# 3. Загрузчик GRUB
echo "[5/12] Создание конфигурации GRUB..."
cat > cosmoos/boot/grub.cfg << 'EOF'
set timeout=0
set default=0

menuentry "CosmoOS" {
    multiboot /boot/kernel.bin
    boot
}
EOF

# 4. Мультизагрузочный заголовок
echo "[6/12] Создание мультизагрузочного заголовка..."
cat > cosmoos/boot/multiboot.asm << 'EOF'
section .multiboot
align 4

multiboot_header:
    dd 0x1BADB002              ; magic
    dd 0x00000003              ; flags
    dd -(0x1BADB002 + 0x00000003) ; checksum
EOF

# 5. Загрузчик
echo "[7/12] Создание загрузчика (boot.asm)..."
cat > cosmoos/boot/boot.asm << 'EOF'
global start
extern kmain

section .text
bits 32

start:
    ; Установка стека
    mov esp, stack_top
    
    ; Проверка мультизагрузки
    cmp eax, 0x2BADB002
    jne .no_multiboot
    
    ; Вызов главной функции ядра на C
    call kmain
    
    ; Если ядро вернулось (чего не должно быть)
    cli
.hang:
    hlt
    jmp .hang

.no_multiboot:
    mov al, 'M'
    jmp error

error:
    ; Простой вывод ошибки (белый текст на красном фоне)
    mov dword [0xb8000], 0x4f454f4d
    cli
    hlt

section .bss
align 16
stack_bottom:
    resb 16384 ; 16KB стека
stack_top:
EOF

# 6. Заголовочные файлы
echo "[8/12] Создание заголовочных файлов..."

# types.h
cat > cosmoos/include/types.h << 'EOF'
#ifndef TYPES_H
#define TYPES_H

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef uint32_t size_t;
typedef int32_t ssize_t;

#define NULL ((void*)0)
#define bool int
#define true 1
#define false 0

#endif
EOF

# kernel.h
cat > cosmoos/include/kernel.h << 'EOF'
#ifndef KERNEL_H
#define KERNEL_H

#include "types.h"

// Базовые функции ядра
void kmain();
void kprint(const char* str);
void kprint_char(char c);
void kprint_dec(uint32_t n);
void kprint_hex(uint32_t n);
void clear_screen();

// Утилиты
void* memcpy(void* dest, const void* src, size_t n);
void* memset(void* dest, uint8_t val, size_t n);
size_t strlen(const char* s);
int strcmp(const char* s1, const char* s2);
char* strcpy(char* dest, const char* src);
char* strcat(char* dest, const char* src);

// Порт ввода/вывода
static inline uint8_t inb(uint16_t port) {
    uint8_t ret;
    asm volatile ("inb %1, %0" : "=a"(ret) : "Nd"(port));
    return ret;
}

static inline void outb(uint16_t port, uint8_t val) {
    asm volatile ("outb %0, %1" : : "a"(val), "Nd"(port));
}

#endif
EOF

# screen.h
cat > cosmoos/include/screen.h << 'EOF'
#ifndef SCREEN_H
#define SCREEN_H

#include "types.h"

#define VIDEO_MEMORY 0xB8000
#define MAX_ROWS 25
#define MAX_COLS 80
#define WHITE_ON_BLACK 0x0F

void clear_screen();
void kprint(const char* str);
void kprint_char(char c);
void kprint_dec(uint32_t n);
void kprint_hex(uint32_t n);
void kprint_at(const char* str, int col, int row);

#endif
EOF

# 7. Ядро системы
echo "[9/12] Создание ядра системы..."

# kernel.c
cat > cosmoos/kernel/kernel.c << 'EOF'
#include "kernel.h"
#include "screen.h"
#include "memory.h"
#include "task.h"
#include "fs.h"
#include "shell.h"

void welcome_message() {
    clear_screen();
    kprint("========================================\n");
    kprint("        CosmoOS v1.0 - Starting...      \n");
    kprint("========================================\n\n");
}

void kmain() {
    welcome_message();
    
    kprint("Initializing screen driver... OK\n");
    kprint("Initializing memory manager... ");
    init_memory();
    kprint("OK\n");
    
    kprint("Initializing file system... ");
    init_fs();
    kprint("OK\n");
    
    kprint("Initializing task scheduler... ");
    init_task_scheduler();
    kprint("OK\n");
    
    kprint("\nSystem ready!\n");
    kprint("Total memory: 1 MB\n");
    kprint("Kernel size: ~32 KB\n");
    kprint("Free memory: ~960 KB\n");
    
    // Запуск демо-задач
    create_task("Idle Task", 0);
    create_task("Shell Task", 0);
    
    kprint("\nStarting command shell...\n");
    kprint("Type 'help' for available commands\n");
    
    start_shell();
    
    // Бесконечный цикл ядра
    while(1) {
        asm volatile("hlt");
    }
}
EOF

# screen.c (драйвер экрана)
cat > cosmoos/kernel/screen.c << 'EOF'
#include "screen.h"

static int cursor_x = 0;
static int cursor_y = 0;

void update_cursor() {
    uint16_t pos = cursor_y * MAX_COLS + cursor_x;
    
    outb(0x3D4, 0x0F);
    outb(0x3D5, (uint8_t)(pos & 0xFF));
    outb(0x3D4, 0x0E);
    outb(0x3D5, (uint8_t)((pos >> 8) & 0xFF));
}

void clear_screen() {
    uint8_t* video = (uint8_t*)VIDEO_MEMORY;
    
    for (int i = 0; i < MAX_COLS * MAX_ROWS * 2; i += 2) {
        video[i] = ' ';
        video[i + 1] = WHITE_ON_BLACK;
    }
    
    cursor_x = 0;
    cursor_y = 0;
    update_cursor();
}

void scroll() {
    if (cursor_y >= MAX_ROWS) {
        uint8_t* video = (uint8_t*)VIDEO_MEMORY;
        
        // Копируем строки вверх
        for (int i = 0; i < (MAX_ROWS - 1) * MAX_COLS * 2; i++) {
            video[i] = video[i + MAX_COLS * 2];
        }
        
        // Очищаем последнюю строку
        for (int i = (MAX_ROWS - 1) * MAX_COLS * 2; i < MAX_ROWS * MAX_COLS * 2; i += 2) {
            video[i] = ' ';
            video[i + 1] = WHITE_ON_BLACK;
        }
        
        cursor_y = MAX_ROWS - 1;
    }
}

void kprint_char(char c) {
    uint8_t* video = (uint8_t*)VIDEO_MEMORY;
    
    if (c == '\n') {
        cursor_x = 0;
        cursor_y++;
    } else if (c == '\t') {
        cursor_x = (cursor_x + 4) & ~(4 - 1);
    } else if (c == '\b') {
        if (cursor_x > 0) {
            cursor_x--;
            int pos = (cursor_y * MAX_COLS + cursor_x) * 2;
            video[pos] = ' ';
        }
    } else {
        int pos = (cursor_y * MAX_COLS + cursor_x) * 2;
        video[pos] = c;
        video[pos + 1] = WHITE_ON_BLACK;
        
        cursor_x++;
        if (cursor_x >= MAX_COLS) {
            cursor_x = 0;
            cursor_y++;
        }
    }
    
    scroll();
    update_cursor();
}

void kprint(const char* str) {
    while (*str) {
        kprint_char(*str);
        str++;
    }
}

void kprint_at(const char* str, int col, int row) {
    if (col >= 0 && col < MAX_COLS && row >= 0 && row < MAX_ROWS) {
        cursor_x = col;
        cursor_y = row;
        kprint(str);
    }
}

void kprint_dec(uint32_t n) {
    if (n == 0) {
        kprint_char('0');
        return;
    }
    
    char buffer[32];
    int i = 0;
    
    while (n > 0) {
        buffer[i++] = '0' + (n % 10);
        n /= 10;
    }
    
    while (--i >= 0) {
        kprint_char(buffer[i]);
    }
}

void kprint_hex(uint32_t n) {
    kprint("0x");
    
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t digit = (n >> i) & 0xF;
        kprint_char(digit < 10 ? '0' + digit : 'A' + digit - 10);
    }
}
EOF

# memory.c (упрощенная версия)
cat > cosmoos/kernel/memory.c << 'EOF'
#include "kernel.h"
#include "screen.h"

#define MEM_SIZE 0x100000  // 1MB
#define BLOCK_SIZE 4096

static uint8_t memory_pool[MEM_SIZE];
static uint32_t next_free = 0;

void init_memory() {
    next_free = 0;
}

void* kmalloc(size_t size) {
    // Простой аллокатор - выравнивание по 4 байта
    size = (size + 3) & ~3;
    
    if (next_free + size >= MEM_SIZE) {
        kprint("Memory allocation failed!\n");
        return NULL;
    }
    
    void* ptr = &memory_pool[next_free];
    next_free += size;
    
    return ptr;
}

void kfree(void* ptr) {
    // В этой простой реализации free ничего не делает
    (void)ptr;
}

void print_memory_stats() {
    kprint("\n=== Memory Statistics ===\n");
    kprint("Total: ");
    kprint_dec(MEM_SIZE);
    kprint(" bytes\n");
    
    kprint("Used: ");
    kprint_dec(next_free);
    kprint(" bytes\n");
    
    kprint("Free: ");
    kprint_dec(MEM_SIZE - next_free);
    kprint(" bytes\n");
    
    kprint("Usage: ");
    kprint_dec((next_free * 100) / MEM_SIZE);
    kprint("%\n");
}
EOF

# task.c (упрощенный планировщик)
cat > cosmoos/kernel/task.c << 'EOF'
#include "kernel.h"
#include "screen.h"

#define MAX_TASKS 5

typedef struct {
    uint32_t id;
    char name[32];
    uint32_t state;  // 0-free, 1-ready, 2-running
    uint32_t priority;
} task_t;

static task_t tasks[MAX_TASKS];
static uint32_t task_count = 0;

void init_task_scheduler() {
    for (int i = 0; i < MAX_TASKS; i++) {
        tasks[i].state = 0;
    }
}

int create_task(const char* name, uint32_t priority) {
    if (task_count >= MAX_TASKS) {
        return -1;
    }
    
    for (int i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].state == 0) {
            tasks[i].id = i + 1;
            strcpy(tasks[i].name, name);
            tasks[i].state = 1;  // ready
            tasks[i].priority = priority;
            task_count++;
            
            kprint("Created task: ");
            kprint(name);
            kprint(" (ID: ");
            kprint_dec(i + 1);
            kprint(")\n");
            
            return i + 1;
        }
    }
    
    return -1;
}

void print_task_list() {
    kprint("\n=== Task List ===\n");
    kprint("ID\tName\t\tState\tPriority\n");
    
    for (int i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].state != 0) {
            kprint_dec(tasks[i].id);
            kprint("\t");
            kprint(tasks[i].name);
            kprint("\t");
            
            switch(tasks[i].state) {
                case 1: kprint("Ready\t"); break;
                case 2: kprint("Running\t"); break;
                default: kprint("Unknown\t"); break;
            }
            
            kprint_dec(tasks[i].priority);
            kprint("\n");
        }
    }
    
    kprint("\nTotal tasks: ");
    kprint_dec(task_count);
    kprint("\n");
}
EOF

# fs.c (простая файловая система)
cat > cosmoos/kernel/fs.c << 'EOF'
#include "kernel.h"
#include "screen.h"

#define MAX_FILES 16
#define MAX_FILENAME 32

typedef struct {
    char name[MAX_FILENAME];
    uint32_t size;
    uint8_t is_dir;
    uint32_t created;
} file_entry_t;

static file_entry_t files[MAX_FILES];
static uint32_t file_count = 0;

void init_fs() {
    file_count = 0;
    
    // Создаем корневой каталог
    strcpy(files[0].name, "/");
    files[0].size = 0;
    files[0].is_dir = 1;
    files[0].created = 0;
    file_count++;
    
    // Создаем системные файлы
    strcpy(files[1].name, "README.TXT");
    files[1].size = 128;
    files[1].is_dir = 0;
    files[1].created = 1;
    file_count++;
    
    strcpy(files[2].name, "SYSTEM");
    files[2].size = 0;
    files[2].is_dir = 1;
    files[2].created = 1;
    file_count++;
}

int create_file(const char* name, uint8_t is_dir) {
    if (file_count >= MAX_FILES) {
        return -1;
    }
    
    strcpy(files[file_count].name, name);
    files[file_count].size = 0;
    files[file_count].is_dir = is_dir;
    files[file_count].created = file_count;
    
    kprint("Created ");
    kprint(is_dir ? "directory" : "file");
    kprint(": ");
    kprint(name);
    kprint("\n");
    
    return file_count++;
}

void list_directory() {
    kprint("\n=== Directory Listing ===\n");
    kprint("Name\t\tSize\tType\tCreated\n");
    
    for (uint32_t i = 0; i < file_count; i++) {
        kprint(files[i].name);
        kprint("\t\t");
        
        kprint_dec(files[i].size);
        kprint("\t");
        
        if (files[i].is_dir) {
            kprint("DIR\t");
        } else {
            kprint("FILE\t");
        }
        
        kprint_dec(files[i].created);
        kprint("\n");
    }
}

void fs_info() {
    kprint("\n=== File System Info ===\n");
    kprint("Total files: ");
    kprint_dec(file_count);
    kprint("/");
    kprint_dec(MAX_FILES);
    kprint("\n");
    
    uint32_t dir_count = 0;
    uint32_t file_total_size = 0;
    
    for (uint32_t i = 0; i < file_count; i++) {
        if (files[i].is_dir) {
            dir_count++;
        } else {
            file_total_size += files[i].size;
        }
    }
    
    kprint("Directories: ");
    kprint_dec(dir_count);
    kprint("\n");
    
    kprint("Regular files: ");
    kprint_dec(file_count - dir_count);
    kprint("\n");
    
    kprint("Total size: ");
    kprint_dec(file_total_size);
    kprint(" bytes\n");
}
EOF

# shell.c (командный интерпретатор)
cat > cosmoos/kernel/shell.c << 'EOF'
#include "kernel.h"
#include "screen.h"
#include "memory.h"
#include "task.h"
#include "fs.h"

#define MAX_CMD_LEN 64
#define MAX_ARGS 8

void print_prompt() {
    kprint("\ncosmoos> ");
}

void execute_command(char* cmd) {
    char* args[MAX_ARGS];
    int argc = 0;
    
    // Разбиваем команду на аргументы
    char* token = cmd;
    while (*token && argc < MAX_ARGS) {
        // Пропускаем пробелы
        while (*token == ' ') token++;
        if (!*token) break;
        
        args[argc++] = token;
        
        // Ищем конец аргумента
        while (*token && *token != ' ') token++;
        if (*token) *token++ = '\0';
    }
    
    if (argc == 0) return;
    
    // Обработка команд
    if (strcmp(args[0], "help") == 0) {
        kprint("\nAvailable commands:\n");
        kprint("help     - Show this help\n");
        kprint("clear    - Clear screen\n");
        kprint("mem      - Show memory info\n");
        kprint("tasks    - List tasks\n");
        kprint("ls       - List files\n");
        kprint("fsinfo   - File system info\n");
        kprint("echo     - Print text\n");
        kprint("create   - Create file/dir\n");
        kprint("reboot   - Reboot system\n");
        kprint("shutdown - Shutdown\n");
    }
    else if (strcmp(args[0], "clear") == 0) {
        clear_screen();
    }
    else if (strcmp(args[0], "mem") == 0) {
        print_memory_stats();
    }
    else if (strcmp(args[0], "tasks") == 0) {
        print_task_list();
    }
    else if (strcmp(args[0], "ls") == 0) {
        list_directory();
    }
    else if (strcmp(args[0], "fsinfo") == 0) {
        fs_info();
    }
    else if (strcmp(args[0], "echo") == 0) {
        kprint("\n");
        for (int i = 1; i < argc; i++) {
            kprint(args[i]);
            kprint(" ");
        }
        kprint("\n");
    }
    else if (strcmp(args[0], "create") == 0) {
        if (argc >= 3 && strcmp(args[1], "-d") == 0) {
            create_file(args[2], 1);
        } else if (argc >= 2) {
            create_file(args[1], 0);
        } else {
            kprint("Usage: create [-d] <name>\n");
        }
    }
    else if (strcmp(args[0], "reboot") == 0) {
        kprint("\nRebooting system...\n");
        // Команда перезагрузки через порт клавиатуры
        outb(0x64, 0xFE);
    }
    else if (strcmp(args[0], "shutdown") == 0) {
        kprint("\nSystem halted. You can now turn off the computer.\n");
        while(1) asm volatile("hlt");
    }
    else {
        kprint("Unknown command: ");
        kprint(args[0]);
        kprint("\nType 'help' for available commands\n");
    }
}

void start_shell() {
    char command[MAX_CMD_LEN];
    int pos = 0;
    
    kprint("\nCosmoOS Shell v1.0\n");
    kprint("Type 'help' for commands list\n");
    
    while (1) {
        print_prompt();
        pos = 0;
        
        // Чтение команды
        while (1) {
            // В реальной системе здесь было бы чтение с клавиатуры
            // Для демо просто ждем команду
            break;
        }
        
        // Демо-команда
        kprint("help\n");
        execute_command("help");
        
        // Выход из демо
        kprint("\nDemo completed. System halted.\n");
        while(1) asm volatile("hlt");
    }
}
EOF

# utilities.c (утилиты)
cat > cosmoos/kernel/utilities.c << 'EOF'
#include "kernel.h"

void* memcpy(void* dest, const void* src, size_t n) {
    uint8_t* d = (uint8_t*)dest;
    const uint8_t* s = (const uint8_t*)src;
    
    while (n--) {
        *d++ = *s++;
    }
    
    return dest;
}

void* memset(void* dest, uint8_t val, size_t n) {
    uint8_t* d = (uint8_t*)dest;
    
    while (n--) {
        *d++ = val;
    }
    
    return dest;
}

size_t strlen(const char* s) {
    size_t len = 0;
    while (s[len]) len++;
    return len;
}

int strcmp(const char* s1, const char* s2) {
    while (*s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(const uint8_t*)s1 - *(const uint8_t*)s2;
}

char* strcpy(char* dest, const char* src) {
    char* d = dest;
    while ((*d++ = *src++));
    return dest;
}

char* strcat(char* dest, const char* src) {
    char* d = dest;
    while (*d) d++;
    while ((*d++ = *src++));
    return dest;
}
EOF

# 8. Создание скрипта быстрого запуска
echo "[10/12] Создание скрипта быстрого запуска..."
cat > cosmoos/run.sh << 'EOF'
#!/bin/bash

cd "$(dirname "$0")"

echo "Building CosmoOS..."
make clean
make

if [ -f "cosmoos.iso" ]; then
    echo "Starting QEMU..."
    make run
else
    echo "Build failed!"
    exit 1
fi
EOF

chmod +x cosmoos/run.sh

# 9. README файл
echo "[11/12] Создание README..."
cat > cosmoos/README.md << 'EOF'
# CosmoOS - Minimal Operating System

Простая операционная система для обучения, написанная на C.

## Структура проекта
- `boot/` - загрузчик и конфигурация GRUB
- `kernel/` - исходный код ядра
- `include/` - заголовочные файлы
- `iso/` - структура загрузочного ISO

## Сборка и запуск

### Установка зависимостей (Ubuntu/Debian/Mint):
```bash
sudo apt-get install gcc nasm grub-pc-bin xorriso qemu-system-x86 make gcc-multilib
