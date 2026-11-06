// SPDX-License-Identifier: GPL-2.0
/*
 * Host fixture for the crownlte T3 blob guard (blob_guard.h, used verbatim).
 * Feeds the real vendor TA/driver blobs, modified copies and synthetic
 * headers through mc_blob_check() with the T3 pin lists.
 *
 * Build/run (from MobiCoreDriver/):
 *   cc -O1 -Wall -Wextra -Werror -I. tests/blob-guard-fixture.c -o /tmp/bgf -lcrypto
 *   /tmp/bgf <mounted vendor>/app/mcRegistry
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
#include <openssl/sha.h>

typedef uint8_t u8;
typedef uint32_t u32;

#include "public/mc_user.h"	/* struct mc_uuid_t, BIT() */
#include "mci/mcloadformat.h"	/* struct mclf_header_v2, SERVICE_TYPE_* */

static void sha256(const u8 *data, unsigned int len, u8 *out)
{
	SHA256(data, len, out);
}

#include "blob_guard.h"

_Static_assert(sizeof(struct mclf_header_v2) == 76, "MCLF v2 header size");
_Static_assert(offsetof(struct mclf_header_v2, service_type) == 16, "type offset");
_Static_assert(offsetof(struct mclf_header_v2, uuid) == 24, "uuid offset");
_Static_assert(sizeof(struct mc_blob_pin) == MC_PIN_LEN, "pin size");

struct blob {
	const char *file, *sha;
	u8 *data;
	size_t len;
};

static struct blob km = {.file = "ffffffff00000000000000000000003e.tlbin",
	.sha = "40dbc984ecd447995b7a0021a1e0c8e621573089422fe74814b42d5c84211ea7"};
static struct blob gk = {.file = "08130000000000000000000000000000.tlbin",
	.sha = "5062a6b0d27a8ebd891f51abd11533247bc6c6009c5f5ace0b98ffdbd07c3f57"};
static struct blob sec = {.file = "FFFFFFFF000000000000000000000001.drbin",
	.sha = "7f64cbdd78e2da8cb5f2d8b951b1cb3c7e5a3e15badc1263883bb9c973f21caf"};
static struct blob tima = {.file = "ffffffffd0000000000000000000000a.tlbin",
	.sha = "a78e0c874a99d79e96af5c464d093ae41f31f07e885c7de30cb4effab0d64fa8"};
static struct blob cm = {.file = "ffffffffd00000000000000000000016.tlbin",
	.sha = "baa9a8cd23afa8db5afec1a21a3f38fef3e2ae96776868b093740a7864667a1e"};
static struct blob vk = {.file = "ffffffffd00000000000000000000045.drbin",
	.sha = "67f4f9dfb761cc2a8b57a731f9b89d37e05f4cf4b8e23803bc59893047168d83"};
static struct blob other = {.file = "ffffffff000000000000000000000004.tlbin"};

static int fails, checks;

static void load(const char *dir, struct blob *b)
{
	char path[512];
	FILE *f;
	long n;

	snprintf(path, sizeof(path), "%s/%s", dir, b->file);
	f = fopen(path, "rb");
	if (!f || fseek(f, 0, SEEK_END) || (n = ftell(f)) <= 0 || fseek(f, 0, SEEK_SET)) {
		printf("FAIL cannot read %s\n", path);
		exit(1);
	}
	b->len = (size_t)n;
	b->data = malloc(b->len + 1);
	if (!b->data || fread(b->data, 1, b->len, f) != b->len) {
		printf("FAIL short read %s\n", path);
		exit(1);
	}
	fclose(f);
}

static void hex2bin(const char *hex, u8 *out, int n)
{
	for (int i = 0; i < n; i++)
		sscanf(hex + 2 * i, "%2hhx", &out[i]);
}

static struct mc_blob_pin pin(const struct blob *b)
{
	struct mc_blob_pin p;

	memcpy(p.uuid, b->data + 24, 16);
	hex2bin(b->sha, p.sha256, 32);
	return p;
}

static void expect(const char *name, const u8 *data, size_t len, u32 hl, u32 type,
		   const struct mc_blob_pin *pins, int n, enum mc_blob_verdict want)
{
	u8 d[32];
	const u8 *uuid;
	u32 t;
	enum mc_blob_verdict v = mc_blob_check(data, len, hl, type, pins, n, d, &uuid, &t);

	checks++;
	if (v != want) {
		fails++;
		printf("FAIL %-44s got %-16s want %s\n", name, mc_blob_why[v], mc_blob_why[want]);
	} else {
		printf("ok   %-44s %s\n", name, mc_blob_why[v]);
	}
}

int main(int argc, char **argv)
{
	struct blob *all[] = {&km, &gk, &sec, &tima, &cm, &vk, &other};
	struct mc_blob_pin tl[2], drv[3], gk_only[1];
	const u32 TL = SERVICE_TYPE_SYSTEM_TRUSTLET, DRV = SERVICE_TYPE_DRIVER;
	u8 *m;

	if (argc != 2) {
		printf("usage: %s <vendor/app/mcRegistry>\n", argv[0]);
		return 2;
	}
	for (unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++)
		load(argv[1], all[i]);
	/* Appendix A digests must be the real file digests. */
	for (unsigned i = 0; i < 6; i++) {
		u8 d[32], w[32];

		sha256(all[i]->data, all[i]->len, d);
		hex2bin(all[i]->sha, w, 32);
		checks++;
		if (memcmp(d, w, 32)) {
			fails++;
			printf("FAIL digest of %s differs from the pin\n", all[i]->file);
		}
	}
	tl[0] = pin(&km);
	tl[1] = pin(&gk);
	drv[0] = pin(&sec);
	drv[1] = pin(&tima);
	drv[2] = pin(&cm);
	gk_only[0] = pin(&gk);

	/* Pinned and correct type: allowed. */
	expect("KM TA as trustlet", km.data, km.len, 0, TL, tl, 2, MC_BLOB_ALLOWED);
	expect("GK TA as trustlet", gk.data, gk.len, 0, TL, tl, 2, MC_BLOB_ALLOWED);
	expect("sec driver 01 as driver", sec.data, sec.len, 0, DRV, drv, 3, MC_BLOB_ALLOWED);
	expect("TIMA d..0a as driver", tima.data, tima.len, 0, DRV, drv, 3, MC_BLOB_ALLOWED);
	expect("CryptoManager d..16 as driver", cm.data, cm.len, 0, DRV, drv, 3, MC_BLOB_ALLOWED);
	expect("CryptoManager only list", cm.data, cm.len, 0, DRV, &drv[2], 1, MC_BLOB_ALLOWED);
	/* Wrong path / list. */
	expect("KM TA via LOAD_DRIVER", km.data, km.len, 0, DRV, drv, 3, MC_BLOB_TYPE);
	expect("KM TA via LOAD_DRIVER, KM pinned as drv", km.data, km.len, 0, DRV, tl, 2, MC_BLOB_TYPE);
	expect("sec driver via open-trustlet", sec.data, sec.len, 0, TL, tl, 2, MC_BLOB_TYPE);
	expect("sec driver via open-trustlet, pinned", sec.data, sec.len, 0, TL, drv, 3, MC_BLOB_TYPE);
	expect("VaultKeeper (not pinned)", vk.data, vk.len, 0, DRV, drv, 3, MC_BLOB_UUID);
	expect("TIMA with CryptoManager-only list", tima.data, tima.len, 0, DRV, &drv[2], 1, MC_BLOB_UUID);
	expect("KM TA with GK-only list", km.data, km.len, 0, TL, gk_only, 1, MC_BLOB_UUID);
	expect("other TA ..04", other.data, other.len, 0, TL, tl, 2,
	       ((const struct mclf_header_v2 *)other.data)->service_type == TL ? MC_BLOB_UUID : MC_BLOB_TYPE);
	expect("KM TA, no trustlet pins", km.data, km.len, 0, TL, tl, 0, MC_BLOB_NO_PINS);
	expect("CM, no driver pins", cm.data, cm.len, 0, DRV, drv, 0, MC_BLOB_NO_PINS);

	/* Modified copies of a pinned blob. */
	m = malloc(km.len + 1);
	memcpy(m, km.data, km.len);
	m[km.len - 1] ^= 1;
	expect("KM TA last byte flipped", m, km.len, 0, TL, tl, 2, MC_BLOB_DIGEST);
	memcpy(m, km.data, km.len);
	m[km.len / 2] ^= 0x80;
	expect("KM TA middle byte flipped", m, km.len, 0, TL, tl, 2, MC_BLOB_DIGEST);
	memcpy(m, km.data, km.len);
	m[100] ^= 1;	/* past the 76-byte header */
	expect("KM TA byte 100 flipped", m, km.len, 0, TL, tl, 2, MC_BLOB_DIGEST);
	memcpy(m, km.data, km.len);
	expect("KM TA truncated by 1", m, km.len - 1, 0, TL, tl, 2, MC_BLOB_DIGEST);
	m[km.len] = 0;
	expect("KM TA plus 1 byte", m, km.len + 1, 0, TL, tl, 2, MC_BLOB_DIGEST);
	expect("KM TA as SP object (header_length 8)", m, km.len, 8, TL, tl, 2, MC_BLOB_SP);
	memcpy(m, km.data, km.len);
	m[16] = SERVICE_TYPE_SP_TRUSTLET;
	expect("KM TA retyped SP (2)", m, km.len, 0, TL, tl, 2, MC_BLOB_TYPE);
	m[16] = SERVICE_TYPE_DRIVER;
	expect("KM TA retyped driver (1) via LOAD_DRIVER", m, km.len, 0, DRV, tl, 2, MC_BLOB_DIGEST);
	memcpy(m, km.data, km.len);
	m[0] = 'X';
	expect("KM TA bad magic", m, km.len, 0, TL, tl, 2, MC_BLOB_MAGIC);
	memcpy(m, km.data, km.len);
	m[24] ^= 1;	/* UUID changed: no pin for it */
	expect("KM TA uuid changed", m, km.len, 0, TL, tl, 2, MC_BLOB_UUID);
	memcpy(m, gk.data, 24);
	memcpy(m + 24, km.data + 24, gk.len - 24);
	expect("GK header + KM uuid/body (spliced)", m, gk.len, 0, TL, tl, 2, MC_BLOB_DIGEST);
	expect("75-byte blob", km.data, 75, 0, TL, tl, 2, MC_BLOB_SHORT);
	expect("0-byte blob", km.data, 0, 0, TL, tl, 2, MC_BLOB_SHORT);
	free(m);

	/* DT list sizes. */
	{
		static const struct { int n, want; } c[] = {
			{0, -1}, {-22, -1}, {-61, -1}, {16, -1}, {47, -1}, {48, 1}, {49, -1},
			{96, 2}, {144, 3}, {192, 4}, {240, -1}, {32, -1},
		};
		for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
			checks++;
			if (mc_blob_pins_count(c[i].n) != c[i].want) {
				fails++;
				printf("FAIL pins_count(%d)=%d want %d\n", c[i].n,
				       mc_blob_pins_count(c[i].n), c[i].want);
			}
		}
	}
	printf("blob guard fixture: %s (%d checks, %d failures)\n",
	       fails ? "FAIL" : "PASS", checks, fails);
	return !!fails;
}
