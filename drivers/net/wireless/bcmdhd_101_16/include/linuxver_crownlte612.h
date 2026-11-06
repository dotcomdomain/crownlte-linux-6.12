/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte: kernel API compatibility for Samsung bcmdhd 101.16 on Linux 6.12.
 *
 * Only mechanical renames of interfaces that were removed between 5.6 and
 * 6.12 live here; behavioural changes are made in the .c files and listed in
 * Documentation/crownlte/wifi-port.md. Included at the end of linuxver.h.
 */
#ifndef _LINUXVER_CROWNLTE612_H_
#define _LINUXVER_CROWNLTE612_H_

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)

#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/sched/clock.h>
#include <linux/string.h>

/* struct timeval left the in-kernel headers in 5.6; same layout as before */
#define timeval __kernel_old_timeval

/* ioremap() is uncached on arm64; ioremap_nocache() was removed in 5.6. */
#ifndef ioremap_nocache
#define ioremap_nocache(addr, size)	ioremap(addr, size)
#endif

/* include/linux/pci-dma-compat.h, removed in 5.18 */
#ifndef PCI_DMA_TODEVICE
#define PCI_DMA_BIDIRECTIONAL	DMA_BIDIRECTIONAL
#define PCI_DMA_TODEVICE	DMA_TO_DEVICE
#define PCI_DMA_FROMDEVICE	DMA_FROM_DEVICE
#define PCI_DMA_NONE		DMA_NONE
#endif

static inline void
pci_free_consistent(struct pci_dev *hwdev, size_t size, void *vaddr,
	dma_addr_t dma_handle)
{
	dma_free_coherent(&hwdev->dev, size, vaddr, dma_handle);
}

static inline dma_addr_t
pci_map_single(struct pci_dev *hwdev, void *ptr, size_t size, int direction)
{
	return dma_map_single(&hwdev->dev, ptr, size,
		(enum dma_data_direction)direction);
}

static inline void
pci_unmap_single(struct pci_dev *hwdev, dma_addr_t dma_addr, size_t size,
	int direction)
{
	dma_unmap_single(&hwdev->dev, dma_addr, size,
		(enum dma_data_direction)direction);
}

static inline int
pci_dma_mapping_error(struct pci_dev *pdev, dma_addr_t dma_addr)
{
	return dma_mapping_error(&pdev->dev, dma_addr);
}

static inline int
pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{
	return dma_set_mask(&dev->dev, mask);
}

static inline int
pci_set_consistent_dma_mask(struct pci_dev *dev, u64 mask)
{
	return dma_set_coherent_mask(&dev->dev, mask);
}

/* netif_rx() may be called from any context since 5.18 */
#define netif_rx_ni(skb)	netif_rx(skb)

/* PDE_DATA() was renamed in 5.17 */
#ifndef PDE_DATA
#define PDE_DATA(inode)		pde_data(inode)
#endif

/*
 * strlcpy() was removed in 6.8. DHD relies on its return value (length of
 * src), so keep the exact semantics instead of mapping it to strscpy().
 */
static inline size_t
dhd_compat_strlcpy(char *dest, const char *src, size_t size)
{
	size_t ret = strlen(src);

	if (size) {
		size_t len = (ret >= size) ? size - 1 : ret;

		memcpy(dest, src, len);
		dest[len] = '\0';
	}
	return ret;
}
#define strlcpy(d, s, n)	dhd_compat_strlcpy(d, s, n)

/* complete_and_exit() became kthread_complete_and_exit() in 5.17 */
#include <linux/kthread.h>
#define complete_and_exit(comp, code)	kthread_complete_and_exit(comp, code)

/*
 * set_fs()/get_fs() are gone (arm64 since 5.11). DHD only used them around
 * vfs_read/vfs_write, which are compiled out (no DHD_SUPPORT_VFS_CALL), so
 * keep the call sites and make the address-limit dance a no-op.
 */
typedef struct { int unused; } mm_segment_t;
#define KERNEL_DS		((mm_segment_t){ 0 })
#define get_fs()		((mm_segment_t){ 0 })
#define set_fs(x)		do { (void)(x); } while (0)

/* do_gettimeofday() was removed in 5.0; linuxver.h only covers 5.0..5.5 */
static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec / 1000;
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) */

/* struct completion waits on a simple wait queue since 5.7 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 7, 0)
#include <linux/swait.h>
#define DHD_PNO_COMPLETION_WAITERS(c)	swait_active(&(c)->wait)
#else
#define DHD_PNO_COMPLETION_WAITERS(c)	waitqueue_active(&(c)->wait)
#endif

#endif /* _LINUXVER_CROWNLTE612_H_ */
