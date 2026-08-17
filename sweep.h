/*
 * Measurement primitives for the working set sweep in main.c.
 *
 * These exist because the latency test there cannot answer how much cache a
 * process actually gets. Its address generator masks a pseudorandom value, so
 * its footprints are powers of two and it cannot sample between 32 and 64 MiB
 * where a last level cache boundary lands; it also draws addresses with
 * replacement and runs on one thread.
 *
 * A chase around a cycle that covers every line exactly once gives an exact
 * working set at any size, one dependent load per hop, and no address
 * arithmetic in the timed loop. Linking a shuffled visiting order end to end
 * guarantees a single cycle; pointing each line at an independently chosen
 * random line would produce several disjoint cycles, and a chase that fell into
 * a short one would stay cache resident and look fast at every footprint.
 */

#ifndef __SWEEP_H__
#define __SWEEP_H__

#include <stddef.h>

/*
 * Allocates a buffer of the given size and lays an independent cycle inside
 * each of the per-thread slices it will be split into. hugepages is 1 for
 * MADV_HUGEPAGE, -1 for MADV_NOHUGEPAGE, 0 for system policy. Returns NULL on
 * failure.
 */
char *sweep_alloc(size_t bytes, int threads, int hugepages);
void  sweep_free(char *buf, size_t bytes);

/* Bytes each thread walks, given the aggregate footprint and thread count. */
size_t sweep_slice(size_t bytes, int threads);

/*
 * Both measurements split the footprint into private per-thread slices and
 * release the threads from a barrier so they apply pressure at the same time,
 * so a footprint is aggregate pressure on the shared cache levels rather than
 * the size any one thread walks.
 *
 * sweep_chase_ns returns the mean per-thread dependent load latency in ns.
 * warm caps the untimed pass: <= 0 walks the whole slice, which costs one
 * footprint of misses per call and dominates a large run, while a couple of
 * cache fills reach the same steady state.
 *
 * sweep_read_gbs returns aggregate streaming read bandwidth in GB/s, using
 * eight independent accumulators so the result is not limited by the
 * dependency chain of the summation.
 *
 * Both return a negative value if the threads could not be started.
 */
double sweep_chase_ns(char *buf, size_t bytes, int threads, int pin,
                      long hops, long warm);
double sweep_read_gbs(char *buf, size_t bytes, int threads, int pin, int reps);

#endif
