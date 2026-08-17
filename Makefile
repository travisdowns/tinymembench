LDFLAGS ?= -pthread
CFLAGS  ?= -fopenmp -march=native -Wall # -ggdb -O0

all: tinymembench

ifdef WINDIR
	CC = gcc
endif

tinymembench: main.c util.o util.h asm-opt.h version.h asm-opt.o x86-sse2.o x86-avx2.o x86-avx512.o arm-neon.o mips-32.o aarch64-asm.o pmem.o sweep.o sweep.h
	${CC} -O2 ${CFLAGS} ${LDFLAGS} -o tinymembench main.c util.o asm-opt.o x86-sse2.o x86-avx2.o x86-avx512.o arm-neon.o mips-32.o aarch64-asm.o pmem.o sweep.o -lm

util.o: util.c util.h
	${CC} -O2 ${CFLAGS} -c util.c

asm-opt.o: asm-opt.c asm-opt.h x86-sse2.h x86-avx2.h arm-neon.h mips-32.h
	${CC} -O2 ${CFLAGS} -c asm-opt.c

x86-sse2.o: x86-sse2.S
	${CC} -O2 ${CFLAGS} -c x86-sse2.S

x86-avx2.o: x86-avx2.S
	${CC} -O2 ${CFLAGS} -c x86-avx2.S

x86-avx512.o: x86-avx512.S
	${CC} -O2 ${CFLAGS} -c x86-avx512.S

arm-neon.o: arm-neon.S
	${CC} -O2 ${CFLAGS} -c arm-neon.S

aarch64-asm.o: aarch64-asm.S
	${CC} -O2 ${CFLAGS} -c aarch64-asm.S

mips-32.o: mips-32.S
	${CC} -O2 ${CFLAGS} -c mips-32.S

pmem.o: pmem.c pmem.h
	${CC} -O2 ${CFLAGS} -c pmem.c

sweep.o: sweep.c sweep.h
	${CC} -O2 ${CFLAGS} -c sweep.c

clean:
	-rm -f tinymembench
	-rm -f tinymembench.exe
	-rm -f *.o
