/*
 * Copyright © 2011 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define _GNU_SOURCE
#include <getopt.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <omp.h>

#if !defined(_WIN64) && !defined(_WIN32)
#include <unistd.h>
#endif

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <semaphore.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#endif

#include "util.h"
#include "asm-opt.h"
#include "version.h"
#include "pmem.h"
#include "sweep.h"

#define SIZE (1024 * 1024 * 1024)
#define BLOCKSIZE 2048
#ifndef MAXREPEATS
#define MAXREPEATS 10
#endif
#ifndef LATBENCH_COUNT
#define LATBENCH_COUNT 10000000
#endif

char *progname;

#if defined(__aarch64__)
#define ARCH_NAME "aarch64"
#elif defined(__amd64__) || defined(__x86_64__)
#define ARCH_NAME "x86_64"
#elif defined(__i386__)
#define ARCH_NAME "i386"
#elif defined(__arm__)
#define ARCH_NAME "arm"
#elif defined(__mips__)
#define ARCH_NAME "mips"
#else
#define ARCH_NAME "unknown"
#endif

/*
 * Machine readable output. Results are collected as they are printed and the
 * whole document is written out once at the end of the run, so the emitter
 * needs no incremental bracket bookkeeping. Every string stored here is a
 * string literal, so the pointers stay valid without copying.
 */
#define JSON_MAX_RESULTS 512

typedef struct
{
    const char *group;
    const char *description;
    int threads;
    int use_tmpbuf;
    double speed;
    double sd_percent;
} json_bandwidth_result;

typedef struct
{
    const char *group;
    const char *variant;
    int block_size;
    double single_ns;
    double dual_ns;
} json_latency_result;

/*
 * When -B is given, only the bandwidth benchmarks named in it are run. The
 * match is exact rather than a substring so that a harness asking for one
 * benchmark can never silently start measuring a neighbouring one, and names
 * which match nothing are reported as an error rather than ignored.
 */
#define BENCH_FILTER_MAX 32

static char *bench_filter_names[BENCH_FILTER_MAX];
static int bench_filter_matched[BENCH_FILTER_MAX];
static int bench_filter_count = 0;

static void bench_filter_init(const char *list)
{
    char *p, *copy = strdup(list);

    if (!copy)
    {
        fprintf(stderr, "%s: out of memory\n", progname);
        exit(EXIT_FAILURE);
    }

    for (p = strtok(copy, ","); p; p = strtok(NULL, ","))
    {
        if (bench_filter_count == BENCH_FILTER_MAX)
        {
            fprintf(stderr, "%s: at most %d benchmarks may be selected\n",
                    progname, BENCH_FILTER_MAX);
            exit(EXIT_FAILURE);
        }
        bench_filter_names[bench_filter_count++] = p;
    }
}

static int bench_selected(const char *description)
{
    int i;

    if (bench_filter_count == 0)
        return 1;

    for (i = 0; i < bench_filter_count; i++)
    {
        if (strcmp(bench_filter_names[i], description) == 0)
        {
            bench_filter_matched[i] = 1;
            return 1;
        }
    }

    return 0;
}

/* Complains about every selected name which never matched, and counts them. */
static int bench_filter_unmatched(void)
{
    int i, unmatched = 0;

    for (i = 0; i < bench_filter_count; i++)
    {
        if (!bench_filter_matched[i])
        {
            fprintf(stderr, "%s: no benchmark named '%s'\n",
                    progname, bench_filter_names[i]);
            unmatched++;
        }
    }

    return unmatched;
}

/* Which section a result belongs to, set before each phase of the run. */
static const char *json_group = "dram";
static const char *json_variant = "default";

#define JSON_MAX_SWEEP 1024

typedef struct
{
    size_t size;
    int    threads;
    int    trial;
    double lat_ns;
    double bw_gbs;
} json_sweep_result;

static json_sweep_result json_sweep[JSON_MAX_SWEEP];
static int json_sweep_count = 0;
/*
 * Page size for every mode: 1 requests MADV_HUGEPAGE, -1 MADV_NOHUGEPAGE, 0
 * leaves system policy alone. page_mode_set records whether the user asked,
 * because unasked the latency test measures both variants and the sweep defaults
 * to hugepages, which is not the same thing as "default".
 */
static int page_mode = 0;
static int page_mode_set = 0;

static const char *page_mode_name(int mode)
{
    return mode > 0 ? "hugepage" : (mode < 0 ? "nohugepage" : "default");
}

/*
 * madvise() needs a page aligned range, and transparent huge pages only back a
 * 2 MiB aligned one, so trim to that. The pool comes from malloc and may start
 * anywhere.
 */
static void page_advise(void *base, size_t len)
{
#if defined(__linux__) && defined(MADV_HUGEPAGE)
    uintptr_t start = ((uintptr_t)base + 0x1FFFFF) & ~(uintptr_t)0x1FFFFF;
    uintptr_t end = ((uintptr_t)base + len) & ~(uintptr_t)0x1FFFFF;

    if (!page_mode || end <= start)
        return;
    if (madvise((void *)start, end - start,
                page_mode > 0 ? MADV_HUGEPAGE : MADV_NOHUGEPAGE) != 0)
        fprintf(stderr, "%s: madvise(%s) failed, continuing\n", progname,
                page_mode_name(page_mode));
#else
    (void)base;
    (void)len;
#endif
}

static json_bandwidth_result json_bandwidth[JSON_MAX_RESULTS];
static json_latency_result json_latency[JSON_MAX_RESULTS];
static int json_bandwidth_count = 0;
static int json_latency_count = 0;

static void json_add_bandwidth(const char *description, int threads,
                               int use_tmpbuf, double speed, double sd_percent)
{
    json_bandwidth_result *r;

    if (json_bandwidth_count >= JSON_MAX_RESULTS)
        return;

    r = json_bandwidth + json_bandwidth_count++;
    r->group = json_group;
    r->description = description;
    /* the 2-pass benchmarks are always driven by a single thread */
    r->threads = use_tmpbuf ? 1 : threads;
    r->use_tmpbuf = use_tmpbuf;
    r->speed = speed;
    r->sd_percent = sd_percent;
}

static void json_add_sweep(size_t size, int threads, int trial,
                           double lat_ns, double bw_gbs)
{
    json_sweep_result *out;

    if (json_sweep_count >= JSON_MAX_SWEEP)
        return;

    out = json_sweep + json_sweep_count++;
    out->size    = size;
    out->threads = threads;
    out->trial   = trial;
    out->lat_ns  = lat_ns;
    out->bw_gbs  = bw_gbs;
}

static void json_add_latency(int block_size, double single_ns, double dual_ns)
{
    json_latency_result *r;

    if (json_latency_count >= JSON_MAX_RESULTS)
        return;

    r = json_latency + json_latency_count++;
    r->group = json_group;
    r->variant = json_variant;
    r->block_size = block_size;
    r->single_ns = single_ns;
    r->dual_ns = dual_ns;
}

static void json_print_string(FILE *f, const char *s)
{
    fputc('"', f);
    for (; *s; s++)
    {
        if (*s == '"' || *s == '\\')
            fprintf(f, "\\%c", *s);
        else if ((unsigned char)*s < 0x20)
            fprintf(f, "\\u%04x", (unsigned char)*s);
        else
            fputc(*s, f);
    }
    fputc('"', f);
}

static int json_write(const char *path, int threads, int pin,
                      size_t bufsize, int blocksize,
                      size_t latbench_size, int latbench_count)
{
    FILE *f;
    int i;

    if (strcmp(path, "-") == 0)
    {
        f = stdout;
    }
    else if (!(f = fopen(path, "w")))
    {
        fprintf(stderr, "%s: unable to open %s for writing\n", progname, path);
        return 0;
    }

    fprintf(f, "{\n");
    fprintf(f, "  \"version\": \"" VERSION "\",\n");
    fprintf(f, "  \"arch\": \"" ARCH_NAME "\",\n");
    fprintf(f, "  \"threads\": %d,\n", threads);
    fprintf(f, "  \"pinned\": %s,\n", pin ? "true" : "false");
    fprintf(f, "  \"cpus_online\": %ld,\n", sysconf(_SC_NPROCESSORS_ONLN));
    fprintf(f, "  \"buffer_size\": %zu,\n", bufsize);
    fprintf(f, "  \"block_size\": %d,\n", blocksize);
    fprintf(f, "  \"latency_size\": %zu,\n", latbench_size);
    fprintf(f, "  \"latency_count\": %d,\n", latbench_count);
    fprintf(f, "  \"pages\": ");
    json_print_string(f, page_mode_set ? page_mode_name(page_mode) : "unset");
    fprintf(f, ",\n");

    fprintf(f, "  \"bandwidth\": [\n");
    for (i = 0; i < json_bandwidth_count; i++)
    {
        json_bandwidth_result *r = json_bandwidth + i;
        fprintf(f, "    {\"group\": ");
        json_print_string(f, r->group);
        fprintf(f, ", \"name\": ");
        json_print_string(f, r->description);
        fprintf(f, ", \"threads\": %d, \"two_pass\": %s",
                r->threads, r->use_tmpbuf ? "true" : "false");
        fprintf(f, ", \"mb_per_s\": %.1f, \"sd_percent\": %.3f}%s\n",
                r->speed, r->sd_percent,
                i + 1 < json_bandwidth_count ? "," : "");
    }
    fprintf(f, "  ],\n");

    fprintf(f, "  \"latency\": [\n");
    for (i = 0; i < json_latency_count; i++)
    {
        json_latency_result *r = json_latency + i;
        fprintf(f, "    {\"group\": ");
        json_print_string(f, r->group);
        fprintf(f, ", \"variant\": ");
        json_print_string(f, r->variant);
        fprintf(f, ", \"block_size\": %d", r->block_size);
        fprintf(f, ", \"single_read_ns\": %.1f, \"dual_read_ns\": %.1f}%s\n",
                r->single_ns, r->dual_ns,
                i + 1 < json_latency_count ? "," : "");
    }
    fprintf(f, "  ]");

    if (json_sweep_count > 0)
    {
        fprintf(f, ",\n  \"sweep\": [\n");
        for (i = 0; i < json_sweep_count; i++)
        {
            json_sweep_result *r = json_sweep + i;
            fprintf(f, "    {\"size\": %zu, \"threads\": %d, \"trial\": %d,"
                       " \"lat_ns\": %.3f, \"bw_gbs\": %.3f}%s\n",
                    r->size, r->threads, r->trial, r->lat_ns, r->bw_gbs,
                    i + 1 < json_sweep_count ? "," : "");
        }
        fprintf(f, "  ]\n");
    }
    else
    {
        fprintf(f, "\n");
    }

    fprintf(f, "}\n");

    if (f == stdout)
        return fflush(f) == 0;

    if (fclose(f) != 0)
    {
        fprintf(stderr, "%s: error writing %s\n", progname, path);
        return 0;
    }

    return 1;
}

#ifdef __linux__
static void *mmap_framebuffer(size_t *fbsize)
{
    int fd;
    void *p;
    struct fb_fix_screeninfo finfo;

    if ((fd = open("/dev/fb0", O_RDWR)) == -1)
        if ((fd = open("/dev/graphics/fb0", O_RDWR)) == -1)
            return NULL;

    if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo))
    {
        close(fd);
        return NULL;
    }

    p = mmap(0, finfo.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (p == (void *)-1)
        return NULL;

    *fbsize = finfo.smem_len;
    return p;
}

struct f_data
{
    void (*func)(int64_t *, int64_t *, size_t);
    int64_t *arg1;
    int64_t *arg2;
    int arg3;
};
pthread_cond_t p_ready;
pthread_cond_t p_start;
pthread_mutex_t p_lock;
pthread_t *p_worker = NULL;
struct f_data *worker_data = NULL;
int p_worker_not_ready;
int p_workers_ready;

void *thread_func(void *data)
{
    struct f_data *data_ptr = data;

    pthread_mutex_lock(&p_lock);
    --p_worker_not_ready;

    if (p_worker_not_ready == 0)
    {
        pthread_cond_signal(&p_ready);
    }
    while (p_workers_ready != 1)
    {
        pthread_cond_wait(&p_start, &p_lock);
    }
    pthread_mutex_unlock(&p_lock);

    (data_ptr->func)(data_ptr->arg1, data_ptr->arg2, data_ptr->arg3);

    pthread_exit(NULL);
}

static void parallel_run(void)
{
    pthread_mutex_lock(&p_lock);
    p_workers_ready = 1;
    pthread_mutex_unlock(&p_lock);
    pthread_cond_broadcast(&p_start);
}

static void parallel_init(int threads, int pin)
{
    int i;
    pthread_attr_t attr;
    cpu_set_t cpus;

    pthread_cond_init(&p_ready, NULL);
    pthread_cond_init(&p_start, NULL);
    pthread_mutex_init(&p_lock, NULL);
    p_worker_not_ready = threads;
    p_workers_ready = 0;
    pthread_attr_init(&attr);

    if (!p_worker || !worker_data)
    {
        p_worker = (pthread_t *)malloc(threads * sizeof(pthread_t));
        worker_data = (struct f_data *)malloc(threads * sizeof(struct f_data));
    }
    if (!p_worker || !worker_data)
    {
        printf("malloc failed\n");
        return;
    }

    for (i = 0; i < threads; ++i)
    {
        if (pin)
        {
            CPU_ZERO(&cpus);
            CPU_SET(i, &cpus);
            pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
        }
        pthread_create(p_worker + i, pin ? &attr : NULL,
                       thread_func, worker_data + i);
    }

    pthread_mutex_lock(&p_lock);
    while (p_worker_not_ready != 0)
    {
        pthread_cond_wait(&p_ready, &p_lock);
    }
    pthread_mutex_unlock(&p_lock);
}
#endif

static double bandwidth_bench_helper(int threads, int pin,
                                     int64_t *dstbuf, int64_t *srcbuf,
                                     int64_t *tmpbuf,
                                     int size, int blocksize,
                                     const char *indent_prefix,
                                     int use_tmpbuf,
                                     void (*f)(int64_t *, int64_t *, size_t),
                                     const char *description)
{
    int i, j, loopcount, innerloopcount, n;
    double t, t1, t2;
    double speed, maxspeed;
    double s, s0, s1, s2;
    int pt;

    /* do up to MAXREPEATS measurements */
    s = s0 = s1 = s2 = 0.0;
    maxspeed = 0.0;
    for (n = 0; n < MAXREPEATS; n++)
    {
#if 1
        parallel_init(threads, pin);
        for (pt = 0; pt < threads; ++pt)
        {
            (worker_data + pt)->func = f;
            (worker_data + pt)->arg1 = dstbuf + size * pt / sizeof(int64_t);
            (worker_data + pt)->arg2 = srcbuf + size * pt / sizeof(int64_t);
            (worker_data + pt)->arg3 = size;
        }
        parallel_run();
        for (pt = 0; pt < threads; ++pt)
            pthread_join(p_worker[pt], NULL);
#else
        f(dstbuf, srcbuf, size);
#endif
        loopcount = 0;
        innerloopcount = 1;
        t = 0.0;
        do
        {
            loopcount += innerloopcount;
            if (use_tmpbuf)
            {
                for (i = 0; i < innerloopcount; i++)
                {
                    t1 = gettime();
                    for (j = 0; j < size; j += blocksize)
                    {
                        f(tmpbuf, srcbuf + j / sizeof(int64_t), blocksize);
                        f(dstbuf + j / sizeof(int64_t), tmpbuf, blocksize);
                    }
                    t2 = gettime();
                    t += t2 - t1;
                }
            }
            else
            {
                for (i = 0; i < innerloopcount; i++)
                {
#if 1
                    parallel_init(threads, pin);
                    for (pt = 0; pt < threads; ++pt)
                    {
                        (worker_data + pt)->func = f;
                        (worker_data + pt)->arg1 = dstbuf + size * pt / sizeof(int64_t);
                        (worker_data + pt)->arg2 = srcbuf + size * pt / sizeof(int64_t);
                        (worker_data + pt)->arg3 = size;
                    }

                    t1 = gettime();
                    parallel_run();
                    for (pt = 0; pt < threads; ++pt)
                        pthread_join(p_worker[pt], NULL);
                    t2 = gettime();
#else
                    t1 = gettime();
                    f(dstbuf, srcbuf, size);
                    t2 = gettime();
#endif
                    t += t2 - t1;
                }
            }
            innerloopcount *= 2;
        } while (t < 0.5);
        speed = (double)size * (use_tmpbuf ? 1 : threads) * loopcount / t / 1000000.;

        s0 += 1.;
        s1 += speed;
        s2 += speed * speed;

        if (speed > maxspeed)
            maxspeed = speed;

        if (s0 > 2.)
        {
            s = sqrt((s0 * s2 - s1 * s1) / (s0 * (s0 - 1)));
            if (s < maxspeed / 1000.)
                break;
        }
    }

    json_add_bandwidth(description, threads, use_tmpbuf, maxspeed,
                       maxspeed > 0 ? s / maxspeed * 100. : 0.);

    if (maxspeed > 0 && s / maxspeed * 100. >= 0.1)
    {
        printf("%s%-52s : %8.1f MB/s (%.1f%%)",
               indent_prefix, description, maxspeed, s / maxspeed * 100.);
    }
    else
    {
        printf("%s%-52s : %8.1f MB/s       ", indent_prefix, description, maxspeed);
    }
    if (use_tmpbuf || threads == 1)
        printf("\n");
    else
        printf(" @ %d thread%c\n", (use_tmpbuf ? 1 : threads), (use_tmpbuf || threads == 1 ? ' ' : 's'));

    return maxspeed;
}

void memcpy_wrapper(int64_t *dst, int64_t *src, size_t size)
{
    memcpy(dst, src, size);
}

void memset_wrapper(int64_t *dst, int64_t *src, size_t size)
{
    memset(dst, src[0], size);
}

static bench_info c_stream_benchmarks[] =
    {
        {"C stream copy", 0, stream_copy},
        {NULL, 0, NULL}};

static bench_info c_benchmarks[] =
    {
        {"C copy backwards", 0, aligned_block_copy_backwards},
        {"C copy backwards (32 byte blocks)", 0, aligned_block_copy_backwards_bs32},
        {"C copy backwards (64 byte blocks)", 0, aligned_block_copy_backwards_bs64},
        {"C copy", 0, aligned_block_copy},
        {"C copy prefetched (32 bytes step)", 0, aligned_block_copy_pf32},
        {"C copy prefetched (64 bytes step)", 0, aligned_block_copy_pf64},
        {"C 2-pass copy", 1, aligned_block_copy},
        {"C 2-pass copy prefetched (32 bytes step)", 1, aligned_block_copy_pf32},
        {"C 2-pass copy prefetched (64 bytes step)", 1, aligned_block_copy_pf64},
        {"C fetch", 0, aligned_block_fetch},
        {"C fill", 0, aligned_block_fill},
        {"C fill (shuffle within 16 byte blocks)", 0, aligned_block_fill_shuffle16},
        {"C fill (shuffle within 32 byte blocks)", 0, aligned_block_fill_shuffle32},
        {"C fill (shuffle within 64 byte blocks)", 0, aligned_block_fill_shuffle64},
        {NULL, 0, NULL}};

static bench_info libc_benchmarks[] =
    {
        {"standard memcpy", 0, memcpy_wrapper},
        {"standard memset", 0, memset_wrapper},
        {NULL, 0, NULL}};

void bandwidth_bench(int threads, int pin,
                     int64_t *dstbuf, int64_t *srcbuf, int64_t *tmpbuf,
                     int size, int blocksize, const char *indent_prefix,
                     bench_info *bi)
{
    while (bi->f)
    {
        if (bench_selected(bi->description))
            bandwidth_bench_helper(threads, pin,
                                   dstbuf, srcbuf, tmpbuf,
                                   size, blocksize,
                                   indent_prefix, bi->use_tmpbuf,
                                   bi->f,
                                   bi->description);
        bi++;
    }
}

static void __attribute__((noinline)) random_read_test(char *zerobuffer,
                                                       int count, int nbits)
{
    uint32_t seed = 0;
    uintptr_t addrmask = (1 << nbits) - 1;
    uint32_t v;
    static volatile uint32_t dummy;

#ifdef __arm__
    uint32_t tmp;
    __asm__ volatile(
        "subs %[count], %[count],       #16\n"
        "blt  1f\n"
        "0:\n"
        "subs %[count], %[count],       #16\n"
        ".rept 16\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "and  %[v],     %[xFF],         %[seed],        lsr #16\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "and  %[tmp],   %[xFF00],       %[seed],        lsr #8\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "orr  %[v],     %[v],           %[tmp]\n"
        "and  %[tmp],   %[x7FFF0000],   %[seed]\n"
        "orr  %[v],     %[v],           %[tmp]\n"
        "and  %[v],     %[v],           %[addrmask]\n"
        "ldrb %[v],     [%[zerobuffer], %[v]]\n"
        "orr  %[seed],  %[seed],        %[v]\n"
        ".endr\n"
        "bge  0b\n"
        "1:\n"
        "add  %[count], %[count],       #16\n"
        : [count] "+&r"(count),
          [seed] "+&r"(seed), [v] "=&r"(v),
          [tmp] "=&r"(tmp)
        : [c1103515245] "r"(1103515245), [c12345] "r"(12345),
          [xFF00] "r"(0xFF00), [xFF] "r"(0xFF),
          [x7FFF0000] "r"(0x7FFF0000),
          [zerobuffer] "r"(zerobuffer),
          [addrmask] "r"(addrmask)
        : "cc");
#else
#define RANDOM_MEM_ACCESS()           \
    seed = seed * 1103515245 + 12345; \
    v = (seed >> 16) & 0xFF;          \
    seed = seed * 1103515245 + 12345; \
    v |= (seed >> 8) & 0xFF00;        \
    seed = seed * 1103515245 + 12345; \
    v |= seed & 0x7FFF0000;           \
    seed |= zerobuffer[v & addrmask];

    while (count >= 16)
    {
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        count -= 16;
    }
#endif
    dummy = seed;
    (void)dummy; // set but not used suppresion
#undef RANDOM_MEM_ACCESS
}

static void __attribute__((noinline)) random_dual_read_test(char *zerobuffer,
                                                            int count, int nbits)
{
    uint32_t seed = 0;
    uintptr_t addrmask = (1 << nbits) - 1;
    uint32_t v1, v2;
    static volatile uint32_t dummy;

#ifdef __arm__
    uint32_t tmp;
    __asm__ volatile(
        "subs %[count], %[count],       #16\n"
        "blt  1f\n"
        "0:\n"
        "subs %[count], %[count],       #16\n"
        ".rept 16\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "and  %[v1],    %[xFF00],       %[seed],        lsr #8\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "and  %[v2],    %[xFF00],       %[seed],        lsr #8\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "and  %[tmp],   %[x7FFF0000],   %[seed]\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "orr  %[v1],    %[v1],          %[tmp]\n"
        "and  %[tmp],   %[x7FFF0000],   %[seed]\n"
        "mla  %[seed],  %[c1103515245], %[seed],        %[c12345]\n"
        "orr  %[v2],    %[v2],          %[tmp]\n"
        "and  %[tmp],   %[xFF],         %[seed],        lsr #16\n"
        "orr  %[v2],    %[v2],          %[seed],        lsr #24\n"
        "orr  %[v1],    %[v1],          %[tmp]\n"
        "and  %[v2],    %[v2],          %[addrmask]\n"
        "eor  %[v1],    %[v1],          %[v2]\n"
        "and  %[v1],    %[v1],          %[addrmask]\n"
        "ldrb %[v2],    [%[zerobuffer], %[v2]]\n"
        "ldrb %[v1],    [%[zerobuffer], %[v1]]\n"
        "orr  %[seed],  %[seed],        %[v2]\n"
        "add  %[seed],  %[seed],        %[v1]\n"
        ".endr\n"
        "bge  0b\n"
        "1:\n"
        "add  %[count], %[count],       #16\n"
        : [count] "+&r"(count),
          [seed] "+&r"(seed), [v1] "=&r"(v1), [v2] "=&r"(v2),
          [tmp] "=&r"(tmp)
        : [c1103515245] "r"(1103515245), [c12345] "r"(12345),
          [xFF00] "r"(0xFF00), [xFF] "r"(0xFF),
          [x7FFF0000] "r"(0x7FFF0000),
          [zerobuffer] "r"(zerobuffer),
          [addrmask] "r"(addrmask)
        : "cc");
#else
#define RANDOM_MEM_ACCESS()           \
    seed = seed * 1103515245 + 12345; \
    v1 = (seed >> 8) & 0xFF00;        \
    seed = seed * 1103515245 + 12345; \
    v2 = (seed >> 8) & 0xFF00;        \
    seed = seed * 1103515245 + 12345; \
    v1 |= seed & 0x7FFF0000;          \
    seed = seed * 1103515245 + 12345; \
    v2 |= seed & 0x7FFF0000;          \
    seed = seed * 1103515245 + 12345; \
    v1 |= (seed >> 16) & 0xFF;        \
    v2 |= (seed >> 24);               \
    v2 &= addrmask;                   \
    v1 ^= v2;                         \
    seed |= zerobuffer[v2];           \
    seed += zerobuffer[v1 & addrmask];

    while (count >= 16)
    {
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        RANDOM_MEM_ACCESS();
        count -= 16;
    }
#endif
    dummy = seed;
    (void)dummy; // suppress set but not used
#undef RANDOM_MEM_ACCESS
}

static uint32_t rand32()
{
    static int seed = 0;
    uint32_t hi, lo;
    hi = (seed = seed * 1103515245 + 12345) >> 16;
    lo = (seed = seed * 1103515245 + 12345) >> 16;
    return (hi << 16) + lo;
}

static void latency_bench_with_buffer(void *buffer, size_t size, int count, const char *comment)
{
    double t, t2, t_before, t_after, t_noaccess, t_noaccess2;
    double xs, xs1, xs2;
    double ys, ys1, ys2;
    double min_t, min_t2;
    int nbits, n;

    printf("\nblock size : single random read / dual random read");

    if (NULL == comment)
    {
        printf("\n");
    }
    else
    {
        printf("%s\n", comment);
    }

    memset(buffer, 0, size);

    for (n = 1; n <= MAXREPEATS; n++)
    {
        t_before = gettime();
        random_read_test(buffer, count, 1);
        t_after = gettime();
        if (n == 1 || t_after - t_before < t_noaccess)
            t_noaccess = t_after - t_before;

        t_before = gettime();
        random_dual_read_test(buffer, count, 1);
        t_after = gettime();
        t_noaccess2 = 0.0;
        if (n == 1 || t_after - t_before < t_noaccess2)
            t_noaccess2 = t_after - t_before;
    }

    for (nbits = 10; (1 << nbits) <= size; nbits++)
    {
        int testsize = 1 << nbits;
        xs1 = xs2 = ys = ys1 = ys2 = 0;
        for (n = 1; n <= MAXREPEATS; n++)
        {
            /*
             * Select a random offset in order to mitigate the unpredictability
             * of cache associativity effects when dealing with different
             * physical memory fragmentation (for PIPT caches). We are reporting
             * the "best" measured latency, some offsets may be better than
             * the others.
             */
            int testoffs = (rand32() % (size / testsize)) * testsize;

            t_before = gettime();
            random_read_test(buffer + testoffs, count, nbits);
            t_after = gettime();
            t = t_after - t_before - t_noaccess;
            if (t < 0)
                t = 0;

            xs1 += t;
            xs2 += t * t;

            if (n == 1 || t < min_t)
                min_t = t;

            t_before = gettime();
            random_dual_read_test(buffer + testoffs, count, nbits);
            t_after = gettime();
            t2 = t_after - t_before - t_noaccess2;
            if (t2 < 0)
                t2 = 0;

            ys1 += t2;
            ys2 += t2 * t2;

            if (n == 1 || t2 < min_t2)
                min_t2 = t2;

            if (n > 2)
            {
                xs = sqrt((xs2 * n - xs1 * xs1) / (n * (n - 1)));
                ys = sqrt((ys2 * n - ys1 * ys1) / (n * (n - 1)));
                if (xs < min_t / 1000. && ys < min_t2 / 1000.)
                    break;
            }
        }
        printf("%10d : %6.1f ns          /  %6.1f ns \n", (1 << nbits),
               min_t * 1000000000. / count, min_t2 * 1000000000. / count);
        json_add_latency(1 << nbits, min_t * 1000000000. / count,
                         min_t2 * 1000000000. / count);
    }
}

static int pmem_latency_bench(size_t size, int count, int memfd)
{
    void *poolbuf = NULL;
    void *buffer = NULL;

    if (memfd < 0)
    {
        return 0;
    }

    poolbuf = alloc_four_pmem_buffers(&buffer, size, NULL, 0, NULL, 0, NULL, 0, memfd);
    if (NULL == poolbuf)
    {
        fprintf(stderr, "%s: failed\n", __func__);
        exit(1);
    }

    json_group = "pmem";
    json_variant = "file";
    latency_bench_with_buffer(buffer, size, count, ", [Using File]");
    json_group = "dram";

    free_pmem_buffers(poolbuf);
    poolbuf = NULL;

    return 1;
}

int latency_bench(size_t size, int count, int use_hugepage)
{
    char *buffer, *buffer_alloc;

#if !defined(__linux__) || !defined(MADV_HUGEPAGE)
    if (use_hugepage)
        return 0;
    buffer_alloc = (char *)malloc(size + 4095);
    if (!buffer_alloc)
        return 0;
    buffer = (char *)(((uintptr_t)buffer_alloc + 4095) & ~(uintptr_t)4095);
#else
    if (posix_memalign((void **)&buffer_alloc, 4 * 1024 * 1024, size) != 0)
        return 0;
    buffer = buffer_alloc;
    if (use_hugepage && madvise(buffer, size, use_hugepage > 0 ? MADV_HUGEPAGE : MADV_NOHUGEPAGE) != 0)
    {
        free(buffer_alloc);
        return 0;
    }
#endif
    if (use_hugepage > 0)
    {
        json_variant = "hugepage";
        latency_bench_with_buffer(buffer, size, count, ", [MADV_HUGEPAGE]");
    }
    else if (use_hugepage < 0)
    {
        json_variant = "nohugepage";
        latency_bench_with_buffer(buffer, size, count, ", [MADV_NOHUGEPAGE]");
    }
    else
    {
        json_variant = "default";
        latency_bench_with_buffer(buffer, size, count, "");
    }
    free(buffer_alloc);

    return 1;
}

static void memtest(int threads, int pin, void *dstbuf, void *srcbuf, void *tmpbuf, size_t bufsize, size_t blocksize, const char *comment)
{
    printf("\n");
    printf("==========================================================================\n");
    if (NULL != comment)
    {
        printf("== %30s                                               ==\n", comment);
    }
    printf("== Memory bandwidth tests                                               ==\n");
    printf("== size Bytes: %zu                                                ==\n", bufsize);
    printf("== blocksize Bytes: %zu                                                ==\n", blocksize);
    printf("==                                                                      ==\n");
    printf("== Note 1: 1MB = 1000000 bytes                                          ==\n");
    printf("== Note 2: Results for 'copy' tests show how many bytes can be          ==\n");
    printf("==         copied per second (adding together read and written          ==\n");
    printf("==         bytes would have provided twice higher numbers)              ==\n");
    printf("== Note 3: 2-pass copy means that we are using a small temporary buffer ==\n");
    printf("==         to first fetch data into it, and only then write it to the   ==\n");
    printf("==         destination (source -> L1 cache, L1 cache -> destination)    ==\n");
    printf("== Note 4: If sample standard deviation exceeds 0.1%%, it is shown in    ==\n");
    printf("==         brackets                                                     ==\n");
    printf("==========================================================================\n\n");

    if (pin)
        bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", c_stream_benchmarks);
    bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", c_benchmarks);
    printf(" ---\n");
    bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", libc_benchmarks);

    bench_info *bi = get_asm_benchmarks();
    if (bi && bi->f)
    {
        printf(" ---\n");
        bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", bi);
    }

    bench_info *bi_avx2 = get_avx2_benchmarks();
    if (bi_avx2 && bi_avx2->f)
    {
        printf(" ---\n");
        bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", bi_avx2);
    }

    bench_info *bi_avx512 = get_avx512_benchmarks();
    if (bi_avx512 && bi_avx512->f)
    {
        printf(" ---\n");
        bandwidth_bench(threads, pin, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", bi_avx512);
    }
}

static void
usage()
{
    fprintf(stderr, "usage: %s [-s buffer size -b blocksize -l latency max size -c latency count -j json file]\n",
            progname);
    fprintf(stderr, "\t-b Memory blocksize in Bytes <%d>\n", BLOCKSIZE);
    fprintf(stderr, "\t-s Memory buffer size in Bytes <%d>\n", SIZE);
    fprintf(stderr, "\t-l Latency test maximum buffer size in Bytes <%ld>\n", (size_t)SIZE * 2);
    fprintf(stderr, "\t-c Latency count <%d>\n", LATBENCH_COUNT);
    fprintf(stderr, "\t-m Memory map file (PMEM/DAX) <name>\n");
    // fprintf(stderr, "\t--run_sse Include SSE2 tests <false>\n");
    //  fprintf(stderr, "\t--run_sse Include AVX2 tests <false>\n");
    // fprintf(stderr, "\t--run_sse Include AVX512 tests <false>\n");
    fprintf(stderr, "\t-t Thread count, 0 means %ld (max)\n", sysconf(_SC_NPROCESSORS_ONLN));
    fprintf(stderr, "\t-u Run without pinning threads to CPUs\n");
    fprintf(stderr, "\t-j Also write the results as JSON to <file> ('-' for stdout)\n");
    fprintf(stderr, "\t-B Run only the named bandwidth benchmarks (comma separated, exact names)\n");
    fprintf(stderr, "\t-L Skip the memory latency test\n");
    fprintf(stderr, "\t-S Run the working set sweep instead of the standard benchmarks\n");
    fprintf(stderr, "\t   Footprints run from 512 KiB up to -l, trials use --sweep-trials.\n");
    fprintf(stderr, "\t   --sweep-trials <n> repeats per footprint <10>\n");
    fprintf(stderr, "\t   --sweep-hops <n>   timed loads per thread per trial <1000000>\n");
    fprintf(stderr, "\t   --sweep-warm <n>   cap on warm-pass loads, 0 for a full pass <2000000>\n");
    fprintf(stderr, "\t--hugepages <p> Page size for every mode: huge|2m, small|4k, default.\n");
    fprintf(stderr, "\t                Unset runs the latency test both ways and sweeps with huge.\n");
    exit(EXIT_FAILURE);
}

#if defined(__linux__)
static void set_linux_fifo_scheduler()
{
    /* Use FIFO scheduler to limit OS interference. Program must be run
     as root, and this works only for Linux kernels. */
    struct sched_param schedParam;
    schedParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &schedParam);
}
#endif

/*
 * Working set sweep, placed next to latency_bench because it answers the
 * question that one cannot: how much cache this process actually gets. It walks
 * a ladder of footprints and reports the cycle chase latency and streaming read
 * bandwidth at each, so the footprint where latency leaves its plateau is
 * visible. latency_bench masks a pseudorandom address, which restricts it to
 * power of two footprints and cannot sample between 32 and 64 MiB where a last
 * level cache boundary lands.
 *
 * The ladder is bounded by -l, so the same flag sizes both latency tests.
 */
static const size_t sweep_ladder_kib[] = {
    512, 1024, 2048, 4096, 6144, 8192, 12288, 16384, 20480, 24576, 28672,
    32768, 36864, 40960, 49152, 65536, 81920, 98304, 131072, 196608, 262144,
    524288, 1048576
};

static int sweep_cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

static double sweep_median(double *v, int n)
{
    qsort(v, n, sizeof(double), sweep_cmp_double);
    return n & 1 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

#define SWEEP_MAX_TRIALS 256

static int sweep_bench(int threads, int pin, size_t max_size, int trials,
                       long hops, long warm)
{
    int pages = page_mode_set ? page_mode : 1;
    size_t i;

    if (trials < 1)
        trials = 1;
    if (trials > SWEEP_MAX_TRIALS)
        trials = SWEEP_MAX_TRIALS;

    printf("\n");
    printf("==========================================================================\n");
    printf("== Working set sweep. Latency is a dependent load chase around a cycle   ==\n");
    printf("== covering every line of the footprint exactly once, so the size where  ==\n");
    printf("== it leaves its plateau is the cache this process really gets, which on ==\n");
    printf("== a shared socket can be a fraction of what the CPU advertises.         ==\n");
    printf("==                                                                      ==\n");
    printf("== With several threads the footprint is split into private slices, so   ==\n");
    printf("== a row is aggregate pressure and a slice may fit a private cache.      ==\n");
    printf("== pages: %-62s ==\n",
           pages > 0 ? "MADV_HUGEPAGE"
                     : (pages < 0 ? "MADV_NOHUGEPAGE, expect the TLB not the cache"
                                  : "system default"));
    printf("==========================================================================\n");
    printf("%12s : %9s   %12s%s\n", "footprint", "latency", "read",
           threads > 1 ? "   per thread" : "");

    for (i = 0; i < sizeof(sweep_ladder_kib) / sizeof(sweep_ladder_kib[0]); i++)
    {
        size_t bytes = sweep_ladder_kib[i] * 1024;
        size_t slice = sweep_slice(bytes, threads);
        double lat[SWEEP_MAX_TRIALS], bw[SWEEP_MAX_TRIALS];
        char *buf;
        int reps, trial;

        if (max_size && bytes > max_size)
            continue;
        /* A slice this small says more about the L1 than about anything shared. */
        if (slice < 256 * 1024)
            continue;

        if (!(buf = sweep_alloc(bytes, threads, pages)))
        {
            fprintf(stderr, "%s: cannot allocate %zu bytes for the sweep\n",
                    progname, bytes);
            return 0;
        }

        reps = (int)(1073741824UL / slice);
        if (reps < 3)
            reps = 3;

        for (trial = 0; trial < trials; trial++)
        {
            bw[trial] = sweep_read_gbs(buf, bytes, threads, pin, reps);
            lat[trial] = sweep_chase_ns(buf, bytes, threads, pin, hops, warm);
            if (bw[trial] < 0 || lat[trial] < 0)
            {
                sweep_free(buf, bytes);
                fprintf(stderr, "%s: unable to start %d sweep threads\n",
                        progname, threads);
                return 0;
            }
            json_add_sweep(bytes, threads, trial + 1, lat[trial], bw[trial]);
        }
        sweep_free(buf, bytes);

        if (bytes >= 1024 * 1024)
            printf("%8zu MiB", bytes / (1024 * 1024));
        else
            printf("%8zu KiB", bytes / 1024);
        printf(" : %6.1f ns   %9.1f MB/s",
               sweep_median(lat, trials), sweep_median(bw, trials) * 1000.0);
        if (threads > 1)
            printf("   %zu KiB", slice / 1024);
        printf("\n");
    }
    return 1;
}

int main(int argc, char *argv[])
{
    size_t latbench_size = (size_t)SIZE * 2;
    int latbench_count = LATBENCH_COUNT;
    int c;
    size_t bufsize = SIZE;
    size_t pmem_bufsize;
    int blocksize = BLOCKSIZE;
    static int run_sse2 = 1;
    static int run_avx2 = 1;
    static int run_avx512 = 1;
    void *poolbuf = NULL;
    int64_t *srcbuf, *dstbuf, *tmpbuf;
    const char *filename = NULL; // for DAX
    const char *json_path = NULL;
    int run_sweep = 0;
    int sweep_trials = 10;
    long sweep_hops = 0;
    long sweep_warm = -1;
    size_t json_bufsize;
    int run_latency = 1;
    int memfd = -1;
    int total_cpu = sysconf(_SC_NPROCESSORS_ONLN);
    int threads = -1;
    int pin_threads = 1;

    if (0 == geteuid())
    {
        printf("Using FIFO scheduler\n");
        set_linux_fifo_scheduler();
    }

    progname = argv[0];
    while (1)
    {
        static struct option long_options[] = {
            {"run_sse2", no_argument, &run_sse2, 1},
            {"run_avx2", no_argument, &run_avx2, 1},
            {"run_avx512", no_argument, &run_avx512, 1},
            {"json", required_argument, NULL, 'j'},
            {"bench", required_argument, NULL, 'B'},
            {"no-latency", no_argument, NULL, 'L'},
            {"sweep", no_argument, NULL, 'S'},
            {"sweep-trials", required_argument, NULL, 1001},
            {"sweep-hops", required_argument, NULL, 1002},
            {"sweep-warm", required_argument, NULL, 1003},
            {"hugepages", required_argument, NULL, 1004},
            {0, 0, 0, 0}};
        /* getopt_long stores the option index here. */
        int option_index = 0;
        c = getopt_long(argc, argv, "hb:c:l:s:m:t:uj:B:LS", long_options, &option_index);
        if (c == -1)
            break;
        switch (c)
        {
        case 0:
            if (long_options[option_index].flag != 0)
                break;
            printf("option %s", long_options[option_index].name);
            if (optarg)
                printf(" with arg %s", optarg);
            printf("\n");
            break;
        case 'b':
            blocksize = atoi(optarg);
            break;
        case 'c':
            latbench_count = atoi(optarg);
            break;
        case 'l':
            latbench_size = atoi(optarg);
            break;
        case 's':
            bufsize = atoi(optarg);
            break;
        case 'm':
            filename = strdup(optarg);
            break;
        case 't':
            threads = atoi(optarg);
            break;
        case 'u':
            pin_threads = 0;
            break;
        case 'j':
            json_path = optarg;
            break;
        case 'B':
            bench_filter_init(optarg);
            break;
        case 'L':
            run_latency = 0;
            break;
        case 'S':
            run_sweep = 1;
            break;
        case 1001:
            sweep_trials = atoi(optarg);
            break;
        case 1002:
            sweep_hops = atol(optarg);
            break;
        case 1003:
            sweep_warm = atol(optarg);
            break;
        case 1004:
            page_mode_set = 1;
            if (strcmp(optarg, "huge") == 0 || strcmp(optarg, "2m") == 0)
                page_mode = 1;
            else if (strcmp(optarg, "small") == 0 || strcmp(optarg, "4k") == 0)
                page_mode = -1;
            else if (strcmp(optarg, "default") == 0)
                page_mode = 0;
            else
            {
                fprintf(stderr, "%s: --hugepages expects huge, small or default\n",
                        progname);
                exit(EXIT_FAILURE);
            }
            break;
        case 'h':
        default:
            usage();
            exit(1);
        }
    }

    printf("tinymembench-pthread v" VERSION " (simple benchmark for memory throughput and latency)\n");

    if (threads < 0)
    {
        threads = 1;
        printf("Single thread test\n");
    }

    if (0 == threads)
    {
        threads = total_cpu;
    }

    if (threads > total_cpu)
    {
        printf("Reduce %d threads (to %d CPUs)\n", threads, total_cpu);
        threads = total_cpu;
    }
    printf("%d thread(s) on %d CPU (%s)\n", threads, total_cpu,
           pin_threads ? "pinned" : "unpinned");

    if (run_sweep)
    {
        int ok = sweep_bench(threads, pin_threads, latbench_size, sweep_trials,
                             sweep_hops > 0 ? sweep_hops : 1000000,
                             sweep_warm >= 0 ? sweep_warm : 2000000);

        if (ok && json_path)
            ok = json_write(json_path, threads, pin_threads, bufsize, blocksize,
                            latbench_size, latbench_count);
        return ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (NULL != filename)
    {
        memfd = open_pmem_device(filename);
        printf("Using memory device %s\n", filename);
        if (-1 == memfd)
        {
            printf("Unable to open %s (%d): %s\n", filename, errno, strerror(errno));
            exit(1);
        }

        if (1 != threads)
        {
            fprintf(stderr, "Currently, memory map only written to work with 1 thread\n");
            exit(1);
        }

        pmem_bufsize = align_up1gb(bufsize);

        poolbuf = alloc_four_pmem_buffers((void **)&srcbuf, pmem_bufsize * threads,
                                          (void **)&dstbuf, pmem_bufsize * threads,
                                          (void **)&tmpbuf, BLOCKSIZE * threads,
                                          NULL, 0, memfd);

        if (NULL == poolbuf)
        {
            fprintf(stderr, "alloc_four_pmem_buffers failed\n");
            exit(1);
        }

        // TODO: probably want to make this include the file name used
        json_group = "pmem";
        memtest(threads, pin_threads, dstbuf, srcbuf, tmpbuf, pmem_bufsize, blocksize, "TEST: FILE");
        json_group = "dram";

        free_pmem_buffers(poolbuf);
        poolbuf = NULL;
    }

    poolbuf = alloc_four_nonaliased_buffers((void **)&srcbuf, bufsize * threads,
                                            (void **)&dstbuf, bufsize * threads,
                                            (void **)&tmpbuf, BLOCKSIZE * threads,
                                            NULL, 0);
    page_advise(poolbuf, (size_t)bufsize * threads * 2 +
                             (size_t)BLOCKSIZE * threads);

    /* bufsize is clamped below by the framebuffer test, so remember it here */
    json_bufsize = bufsize;
    memtest(threads, pin_threads, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, "Test: DRAM");

#ifdef __linux__
    bench_info *bi = NULL;
    size_t fbsize = 0;
    int64_t *fbbuf = mmap_framebuffer(&fbsize);
    fbsize = (fbsize / blocksize) * blocksize;

    bi = get_asm_framebuffer_benchmarks();
    if (bi && bi->f && fbbuf)
    {
        printf("\n");
        printf("==========================================================================\n");
        printf("== Framebuffer read tests.                                              ==\n");
        printf("==                                                                      ==\n");
        printf("== Many ARM devices use a part of the system memory as the framebuffer, ==\n");
        printf("== typically mapped as uncached but with write-combining enabled.       ==\n");
        printf("== Writes to such framebuffers are quite fast, but reads are much       ==\n");
        printf("== slower and very sensitive to the alignment and the selection of      ==\n");
        printf("== CPU instructions which are used for accessing memory.                ==\n");
        printf("==                                                                      ==\n");
        printf("== Many x86 systems allocate the framebuffer in the GPU memory,         ==\n");
        printf("== accessible for the CPU via a relatively slow PCI-E bus. Moreover,    ==\n");
        printf("== PCI-E is asymmetric and handles reads a lot worse than writes.       ==\n");
        printf("==                                                                      ==\n");
        printf("== If uncached framebuffer reads are reasonably fast (at least 100 MB/s ==\n");
        printf("== or preferably >300 MB/s), then using the shadow framebuffer layer    ==\n");
        printf("== is not necessary in Xorg DDX drivers, resulting in a nice overall    ==\n");
        printf("== performance improvement. For example, the xf86-video-fbturbo DDX     ==\n");
        printf("== uses this trick.                                                     ==\n");
        printf("==========================================================================\n\n");

        srcbuf = fbbuf;
        if (bufsize > fbsize)
            bufsize = fbsize;
        json_group = "framebuffer";
        bandwidth_bench(1, pin_threads, dstbuf, srcbuf, tmpbuf, bufsize, blocksize, " ", bi);
        json_group = "dram";
    }
    /* TODO: add get_avx2_framebuffer_benchmarks and get_avx512_framebuffer_benchmarks */
#endif

    free(poolbuf);

    if (run_latency && NULL != filename)
    {
        if (-1 == memfd)
        {
            printf("Using memory device %s\n", filename);
            memfd = open_pmem_device(filename);
        }

        if (-1 == memfd)
        {
            printf("Unable to open %s (%d): %s\n", filename, errno, strerror(errno));
            exit(1);
        }

        if (0 == pmem_latency_bench(latbench_size, latbench_count, memfd))
        {
            printf("pmem_latency_bench failed\n");
        }
    }

    if (!run_latency)
        goto done;

    printf("\n");
    printf("==========================================================================\n");
    printf("== Memory latency test                                                  ==\n");
    printf("== latbench_size Bytes: %zu                                       ==\n", latbench_size);
    printf("== latbench_count: %d                                             ==\n", latbench_count);
    printf("==                                                                      ==\n");
    printf("== Average time is measured for random memory accesses in the buffers   ==\n");
    printf("== of different sizes. The larger is the buffer, the more significant   ==\n");
    printf("== are relative contributions of TLB, L1/L2 cache misses and SDRAM      ==\n");
    printf("== accesses. For extremely large buffer sizes we are expecting to see   ==\n");
    printf("== page table walk with several requests to SDRAM for almost every      ==\n");
    printf("== memory access (though 64MiB is not nearly large enough to experience ==\n");
    printf("== this effect to its fullest).                                         ==\n");
    printf("==                                                                      ==\n");
    printf("== Note 1: All the numbers are representing extra time, which needs to  ==\n");
    printf("==         be added to L1 cache latency. The cycle timings for L1 cache ==\n");
    printf("==         latency can be usually found in the processor documentation. ==\n");
    printf("== Note 2: Dual random read means that we are simultaneously performing ==\n");
    printf("==         two independent memory accesses at a time. In the case if    ==\n");
    printf("==         the memory subsystem can't handle multiple outstanding       ==\n");
    printf("==         requests, dual random read has the same timings as two       ==\n");
    printf("==         single reads performed one after another.                    ==\n");
    printf("==========================================================================\n");

    if (page_mode_set)
    {
        if (!latency_bench(latbench_size, latbench_count, page_mode))
            latency_bench(latbench_size, latbench_count, 0);
    }
    else if (!latency_bench(latbench_size, latbench_count, -1) ||
             !latency_bench(latbench_size, latbench_count, 1))
    {
        latency_bench(latbench_size, latbench_count, 0);
    }

done:
    if (bench_filter_unmatched() != 0)
        return 1;

    if (json_path && !json_write(json_path, threads, pin_threads, json_bufsize,
                                 blocksize, latbench_size, latbench_count))
    {
        return 1;
    }

    return 0;
}
