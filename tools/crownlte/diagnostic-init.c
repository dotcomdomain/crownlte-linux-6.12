/* SPDX-License-Identifier: GPL-2.0 */
/* Source-only bring-up init. Never opens block devices or mounts phone data. */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static int kmsg = -1;
static char history[512 * 1024];
static size_t history_len;

static void announce(const char *fmt, ...)
{
        char buf[1024];
        va_list ap;
        int n;
        va_start(ap, fmt);
        n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n < 0)
                return;
        if ((size_t)n >= sizeof(buf))
                n = sizeof(buf) - 1;
        dprintf(STDERR_FILENO, "crownlte-diag: %s\n", buf);
        if (kmsg >= 0)
                dprintf(kmsg, "<6>crownlte-diag: %s\n", buf);
}

static int put(const char *path, const char *value)
{
        int fd = open(path, O_WRONLY | O_CLOEXEC);
        ssize_t n;
        if (fd < 0)
                return -errno;
        n = write(fd, value, strlen(value));
        close(fd);
        return n == (ssize_t)strlen(value) ? 0 : -EIO;
}

static void mk(const char *path)
{
        if (mkdir(path, 0755) && errno != EEXIST)
                announce("mkdir %s: %s", path, strerror(errno));
}

static int gadget(void)
{
        const char *base = "/sys/kernel/config/usb_gadget/crownlte_diag";
        struct dirent *de;
        DIR *udcs;
        char name[256] = {0}, path[512];
        udcs = opendir("/sys/class/udc");
        if (!udcs)
                return -ENODEV;
        while ((de = readdir(udcs))) {
                if (de->d_name[0] != '.') {
                        snprintf(name, sizeof(name), "%s", de->d_name);
                        break;
                }
        }
        closedir(udcs);
        if (!name[0])
                return -ENODEV;
        mk(base);
#define SET(attr, value) do { \
        snprintf(path, sizeof(path), "%s/%s", base, attr); \
        int err = put(path, value); \
        if (err) { announce("gadget %s failed: %d", attr, err); return err; } \
} while (0)
        SET("idVendor", "0x1d6b");
        SET("idProduct", "0x0104");
        /* CDC ACM class: Windows can use its existing USB serial driver. */
        SET("bDeviceClass", "0x02");
        SET("bDeviceSubClass", "0");
        SET("bDeviceProtocol", "0");
        mk("/sys/kernel/config/usb_gadget/crownlte_diag/strings/0x409");
        SET("strings/0x409/serialnumber", "crownlte-source-diagnostic");
        SET("strings/0x409/manufacturer", "Kernel bring-up");
        SET("strings/0x409/product", "Note9 kernel diagnostic serial");
        mk("/sys/kernel/config/usb_gadget/crownlte_diag/configs/c.1");
        mk("/sys/kernel/config/usb_gadget/crownlte_diag/configs/c.1/strings/0x409");
        SET("configs/c.1/strings/0x409/configuration", "Read-only diagnostic logs");
        SET("configs/c.1/MaxPower", "100");
        mk("/sys/kernel/config/usb_gadget/crownlte_diag/functions/acm.usb0");
        if (symlink("../../functions/acm.usb0",
                    "/sys/kernel/config/usb_gadget/crownlte_diag/configs/c.1/acm.usb0")
            && errno != EEXIST)
                return -errno;
        SET("UDC", name);
#undef SET
        announce("CDC ACM bound to %s; no ADB daemon or block access", name);
        return 0;
}

static void report(const char *path)
{
        char buf[4096];
        int fd = open(path, O_RDONLY | O_CLOEXEC);
        ssize_t n;
        if (fd < 0) {
                announce("read %s: %s", path, strerror(errno));
                return;
        }
        n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0) {
                buf[n] = 0;
                announce("%s:\n%s", path, buf);
        }
}

static size_t remember(const char *s, size_t n)
{
        size_t discarded = 0;
        if (n > sizeof(history)) {
                s += n - sizeof(history);
                n = sizeof(history);
        }
        if (history_len + n > sizeof(history)) {
                size_t discard = history_len + n - sizeof(history);
                memmove(history, history + discard, history_len - discard);
                history_len -= discard;
                discarded = discard;
        }
        memcpy(history + history_len, s, n);
        history_len += n;
        return discarded;
}

static int serial_open(void)
{
        struct termios tio;
        int fd = open("/dev/ttyGS0", O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
                return -1;
        if (!tcgetattr(fd, &tio)) {
                cfmakeraw(&tio);
                tio.c_cflag |= CLOCAL | CREAD;
                tcsetattr(fd, TCSANOW, &tio);
        }
        return fd;
}

int main(void)
{
        int console, reader, serial = -1, bound = 0;
        unsigned int ticks = 0;
        size_t sent = 0;
        char record[8192];
        struct utsname uts;
        const struct timespec delay = { .tv_sec = 0, .tv_nsec = 100000000 };
        mk("/proc"); mk("/sys"); mk("/dev");
        mount("proc", "/proc", "proc", MS_NOSUID | MS_NOEXEC | MS_NODEV, NULL);
        mount("sysfs", "/sys", "sysfs", MS_NOSUID | MS_NOEXEC | MS_NODEV, NULL);
        mount("devtmpfs", "/dev", "devtmpfs", MS_NOSUID, "mode=0755");
        console = open("/dev/console", O_RDWR | O_NOCTTY);
        if (console >= 0) {
                dup2(console, 0); dup2(console, 1); dup2(console, 2);
                if (console > 2) close(console);
        }
        kmsg = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);
        announce("PID1 running; storage DT nodes disabled; data stays inaccessible");
        if (!uname(&uts))
                announce("kernel %s %s %s", uts.release, uts.version, uts.machine);
        report("/proc/cmdline"); report("/proc/meminfo"); report("/proc/iomem");
        mk("/sys/kernel/config");
        if (mount("configfs", "/sys/kernel/config", "configfs",
                  MS_NOSUID | MS_NOEXEC | MS_NODEV, NULL) && errno != EBUSY)
                announce("configfs mount: %s", strerror(errno));
        reader = open("/dev/kmsg", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (reader < 0)
                announce("kernel log reader: %s", strerror(errno));
        for (;;) {
                ssize_t n;
                if (!bound && ticks % 10 == 0)
                        bound = gadget() == 0;
                if (serial < 0 && bound && ticks % 10 == 0) {
                        serial = serial_open();
                        sent = 0;
                }
                /* Limit work so USB retries and heartbeats remain responsive. */
                for (int i = 0; i < 64 && reader >= 0; i++) {
                        n = read(reader, record, sizeof(record));
                        if (n > 0) {
                                size_t dropped = remember(record, n);
                                sent = sent > dropped ? sent - dropped : 0;
                        }
                        else if (n < 0 && errno == EPIPE) continue;
                        else break;
                }
                if (serial >= 0 && sent < history_len) {
                        n = write(serial, history + sent, history_len - sent);
                        if (n > 0) sent += n;
                        else if (n < 0 && errno != EAGAIN && errno != EINTR) {
                                close(serial); serial = -1;
                        }
                }
                if (ticks % 150 == 0)
                        announce("alive %us, USB=%s, cached log bytes=%zu",
                                 ticks / 10, bound ? "bound" : "no UDC", history_len);
                ticks++;
                nanosleep(&delay, NULL);
        }
}
