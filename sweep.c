/* Measurement primitives for the working set sweep. See sweep.h. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif

#include "sweep.h"

#define SWEEP_LINE        64
#define SWEEP_MAX_THREADS 512

static double sweep_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

size_t sweep_slice(size_t bytes, int threads)
{
    if (threads < 1)
        threads = 1;
    return (bytes / (size_t)threads) & ~(size_t)(SWEEP_LINE - 1);
}

typedef struct
{
    char  *base;
    size_t bytes;
    int    cpu;
    int    pin;
    int    mode;      /* 0 = chase, 1 = streaming read */
    long   hops;
    long   warm;
    int    reps;
    double secs;      /* out */
    double ops;       /* out: hops, or bytes touched */
} sweep_work;

static pthread_barrier_t sweep_barrier;

static void sweep_build_cycle(char *buf, size_t bytes, unsigned seed)
{
    size_t n = bytes / SWEEP_LINE, i;
    size_t *order = (size_t *)malloc(n * sizeof(size_t));
    unsigned state = seed;

    if (!order)
        return;
    for (i = 0; i < n; i++)
        order[i] = i;
    for (i = n - 1; i > 0; i--)
    {
        size_t j, t;
        state = state * 1103515245u + 12345u;
        j = (size_t)((state >> 8) % i);
        t = order[i];
        order[i] = order[j];
        order[j] = t;
    }
    for (i = 0; i < n; i++)
        *(void **)(buf + order[i] * SWEEP_LINE) =
            (void *)(buf + order[(i + 1) % n] * SWEEP_LINE);
    free(order);
}

static void sweep_pin(int cpu)
{
#if defined(__linux__) && defined(CPU_SETSIZE)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#else
    (void)cpu;
#endif
}

static void *sweep_worker(void *arg)
{
    sweep_work *w = (sweep_work *)arg;

    if (w->pin)
        sweep_pin(w->cpu);

    if (w->mode == 0)
    {
        void **p = (void **)w->base;
        size_t lines = w->bytes / SWEEP_LINE;
        long warm = w->warm > 0 && (size_t)w->warm < lines ? w->warm : (long)lines;
        long i;

        for (i = 0; i < warm; i++)
            p = (void **)*p;
        pthread_barrier_wait(&sweep_barrier);
        {
            double t0 = sweep_now();
            for (i = 0; i < w->hops; i++)
                p = (void **)*p;
            w->secs = sweep_now() - t0;
        }
        __asm__ volatile("" :: "r"(p));
        w->ops = (double)w->hops;
    }
    else
    {
        uint64_t *p = (uint64_t *)w->base;
        size_t n = w->bytes / 8, i;
        uint64_t a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0, a7 = 0;
        int r;

        for (i = 0; i + 8 <= n; i += 64)
            a0 += p[i];
        pthread_barrier_wait(&sweep_barrier);
        {
            double t0 = sweep_now();
            for (r = 0; r < w->reps; r++)
                for (i = 0; i + 8 <= n; i += 8)
                {
                    a0 += p[i + 0]; a1 += p[i + 1];
                    a2 += p[i + 2]; a3 += p[i + 3];
                    a4 += p[i + 4]; a5 += p[i + 5];
                    a6 += p[i + 6]; a7 += p[i + 7];
                }
            w->secs = sweep_now() - t0;
        }
        {
            uint64_t sum = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
            __asm__ volatile("" :: "r"(sum));
        }
        w->ops = (double)w->bytes * (double)w->reps;
    }
    return NULL;
}

static double sweep_measure(char *buf, size_t bytes, int threads, int pin,
                            int mode, long hops, long warm, int reps)
{
    pthread_t thread[SWEEP_MAX_THREADS];
    sweep_work work[SWEEP_MAX_THREADS];
    size_t slice = sweep_slice(bytes, threads);
    int i;

    if (threads < 1)
        threads = 1;
    if (threads > SWEEP_MAX_THREADS)
        threads = SWEEP_MAX_THREADS;

    pthread_barrier_init(&sweep_barrier, NULL, threads);
    for (i = 0; i < threads; i++)
    {
        memset(work + i, 0, sizeof(work[i]));
        work[i].base  = buf + (size_t)i * slice;
        work[i].bytes = slice;
        work[i].cpu   = i;
        work[i].pin   = pin;
        work[i].mode  = mode;
        work[i].hops  = hops;
        work[i].warm  = warm;
        work[i].reps  = reps;
        if (pthread_create(thread + i, NULL, sweep_worker, work + i) != 0)
        {
            while (--i >= 0)
                pthread_join(thread[i], NULL);
            pthread_barrier_destroy(&sweep_barrier);
            return -1;
        }
    }
    for (i = 0; i < threads; i++)
        pthread_join(thread[i], NULL);
    pthread_barrier_destroy(&sweep_barrier);

    if (mode == 0)
    {
        double acc = 0;
        for (i = 0; i < threads; i++)
            acc += work[i].secs * 1e9 / work[i].ops;
        return acc / threads;
    }
    else
    {
        double total = 0, slowest = 0;
        for (i = 0; i < threads; i++)
        {
            total += work[i].ops;
            if (work[i].secs > slowest)
                slowest = work[i].secs;
        }
        return slowest > 0 ? total / slowest / 1e9 : -1;
    }
}

double sweep_chase_ns(char *buf, size_t bytes, int threads, int pin,
                      long hops, long warm)
{
    return sweep_measure(buf, bytes, threads, pin, 0, hops, warm, 0);
}

double sweep_read_gbs(char *buf, size_t bytes, int threads, int pin, int reps)
{
    return sweep_measure(buf, bytes, threads, pin, 1, 0, 0, reps);
}

char *sweep_alloc(size_t bytes, int threads, int hugepages)
{
    size_t slice = sweep_slice(bytes, threads);
    char *buf;
    int t;

#if defined(__linux__) && defined(MADV_HUGEPAGE)
    buf = (char *)mmap(NULL, bytes, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (buf == MAP_FAILED)
        return NULL;
    if (hugepages)
        madvise(buf, bytes, hugepages > 0 ? MADV_HUGEPAGE : MADV_NOHUGEPAGE);
#else
    (void)hugepages;
    if (!(buf = (char *)malloc(bytes)))
        return NULL;
#endif

    memset(buf, 0, bytes);
    if (threads < 1)
        threads = 1;
    for (t = 0; t < threads; t++)
        sweep_build_cycle(buf + (size_t)t * slice, slice,
                          12345u + (unsigned)t * 7919u);
    return buf;
}

void sweep_free(char *buf, size_t bytes)
{
#if defined(__linux__) && defined(MADV_HUGEPAGE)
    munmap(buf, bytes);
#else
    (void)bytes;
    free(buf);
#endif
}
