/*
 * Working-set sweep. See sweep.h for what this measures and why.
 *
 * Latency is a pointer chase around a single cycle that covers every cache line
 * in the thread's slice exactly once, so each line's reuse distance is the whole
 * slice and each hop is one dependent load. Linking a shuffled visiting order
 * end to end guarantees one cycle; pointing each line at an independently chosen
 * random line would instead produce several disjoint cycles, and a chase that
 * fell into a short one would stay cache resident and report a fast latency at
 * every footprint.
 *
 * Bandwidth is a streaming read with eight independent accumulators, which is
 * enough to keep the load units busy so the result reflects the memory system
 * rather than the dependency chain of the summation.
 *
 * With more than one thread the footprint is split into private per-thread
 * slices, so a point on the x axis is the aggregate pressure on the shared cache
 * levels, not the size each thread walks. At high thread counts a slice can fall
 * inside a core's private cache while the aggregate footprint looks large.
 */

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

#define SWEEP_LINE      64
#define SWEEP_MAX_THREADS 512

/* Dense between 16 and 64 MiB, where last level cache boundaries usually sit. */
static const size_t sweep_ladder_kib[] = {
    512, 1024, 2048, 4096, 6144, 8192, 12288, 16384, 20480, 24576, 28672,
    32768, 36864, 40960, 49152, 65536, 81920, 98304, 131072, 196608, 262144,
    524288, 1048576
};

static double sweep_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

typedef struct
{
    char  *base;
    size_t bytes;
    int    cpu;
    int    pin;
    int    mode;        /* 0 = latency, 1 = bandwidth */
    long   hops;
    long   warm_hops;
    int    reps;
    double secs;        /* out */
    double ops;         /* out: hops, or bytes touched */
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
        size_t j;
        state = state * 1103515245u + 12345u;
        j = (size_t)((state >> 8) % i);
        {
            size_t t = order[i];
            order[i] = order[j];
            order[j] = t;
        }
    }
    for (i = 0; i < n; i++)
        *(void **)(buf + order[i] * SWEEP_LINE) =
            (void *)(buf + order[(i + 1) % n] * SWEEP_LINE);
    free(order);
}

static void sweep_pin(int cpu)
{
#if defined(__linux__) && defined(CPU_SET)
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
        long warm = w->warm_hops > 0 && (size_t)w->warm_hops < lines
                        ? w->warm_hops : (long)lines;
        long i;

        /*
         * The warm pass leaves the chase pointing at lines that were touched
         * earliest, so the timed hops run against the coldest part of the
         * slice. A full traversal is ideal but costs one whole footprint of
         * misses per trial, which dominates the run at large sizes, so the
         * caller can cap it: a couple of cache fills is enough to reach the
         * same steady state.
         */
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

/* Latency returns mean ns per hop across threads; bandwidth returns GB/s. */
static double sweep_measure(char *buf, size_t total, int threads, int pin,
                            int mode, long hops, long warm_hops, int reps)
{
    pthread_t thread[SWEEP_MAX_THREADS];
    sweep_work work[SWEEP_MAX_THREADS];
    size_t chunk = (total / threads) & ~(size_t)(SWEEP_LINE - 1);
    int i;

    pthread_barrier_init(&sweep_barrier, NULL, threads);
    for (i = 0; i < threads; i++)
    {
        memset(work + i, 0, sizeof(work[i]));
        work[i].base      = buf + (size_t)i * chunk;
        work[i].bytes     = chunk;
        work[i].cpu       = i;
        work[i].pin       = pin;
        work[i].mode      = mode;
        work[i].hops      = hops;
        work[i].warm_hops = warm_hops;
        work[i].reps      = reps;
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
        double bytes = 0, slowest = 0;
        for (i = 0; i < threads; i++)
        {
            bytes += work[i].ops;
            if (work[i].secs > slowest)
                slowest = work[i].secs;
        }
        return slowest > 0 ? bytes / slowest / 1e9 : -1;
    }
}

static char *sweep_alloc(size_t bytes, int hugepages)
{
#if defined(__linux__) && defined(MADV_HUGEPAGE)
    char *buf = (char *)mmap(NULL, bytes, PROT_READ | PROT_WRITE,
                             MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (buf == MAP_FAILED)
        return NULL;
    if (hugepages)
        madvise(buf, bytes, hugepages > 0 ? MADV_HUGEPAGE : MADV_NOHUGEPAGE);
    return buf;
#else
    (void)hugepages;
    return (char *)malloc(bytes);
#endif
}

static void sweep_free(char *buf, size_t bytes)
{
#if defined(__linux__) && defined(MADV_HUGEPAGE)
    munmap(buf, bytes);
#else
    (void)bytes;
    free(buf);
#endif
}

void sweep_defaults(sweep_config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->threads   = 1;
    cfg->pin       = 1;
    cfg->trials    = 10;
    cfg->hops      = 1000000;
    cfg->warm_hops = 2000000;
    cfg->hugepages = 1;
}

int sweep_run(const sweep_config *cfg,
              int (*emit)(const sweep_result *result, void *ctx), void *ctx)
{
    int threads = cfg->threads;
    int emitted = 0;
    size_t i;

    if (threads <= 0)
    {
#ifdef __linux__
        threads = (int)sysconf(_SC_NPROCESSORS_ONLN);
#else
        threads = 1;
#endif
    }
    if (threads > SWEEP_MAX_THREADS)
        threads = SWEEP_MAX_THREADS;

    for (i = 0; i < sizeof(sweep_ladder_kib) / sizeof(sweep_ladder_kib[0]); i++)
    {
        size_t total = sweep_ladder_kib[i] * 1024;
        size_t chunk;
        char *buf;
        int reps, trial, t;

        if (cfg->min_size && total < cfg->min_size)
            continue;
        if (cfg->max_size && total > cfg->max_size)
            continue;

        chunk = (total / threads) & ~(size_t)(SWEEP_LINE - 1);
        /* A slice below 256 KiB says more about the L1 than about anything
         * shared, and the cycle needs enough lines to be worth building. */
        if (chunk < 256 * 1024)
            continue;

        if (!(buf = sweep_alloc(total, cfg->hugepages)))
        {
            fprintf(stderr, "sweep: cannot allocate %zu bytes\n", total);
            return -1;
        }
        memset(buf, 0, total);
        for (t = 0; t < threads; t++)
            sweep_build_cycle(buf + (size_t)t * chunk, chunk,
                              12345u + (unsigned)t * 7919u);

        reps = (int)(1073741824UL / chunk);
        if (reps < 3)
            reps = 3;

        for (trial = 1; trial <= cfg->trials; trial++)
        {
            sweep_result r;
            r.size    = total;
            r.threads = threads;
            r.trial   = trial;
            r.bw_gbs  = sweep_measure(buf, total, threads, cfg->pin, 1,
                                      0, 0, reps);
            r.lat_ns  = sweep_measure(buf, total, threads, cfg->pin, 0,
                                      cfg->hops, cfg->warm_hops, 0);
            if (r.bw_gbs < 0 || r.lat_ns < 0)
            {
                sweep_free(buf, total);
                fprintf(stderr, "sweep: unable to start %d threads\n", threads);
                return -1;
            }
            emitted++;
            if (emit && !emit(&r, ctx))
            {
                sweep_free(buf, total);
                return emitted;
            }
        }
        sweep_free(buf, total);
    }
    return emitted;
}
