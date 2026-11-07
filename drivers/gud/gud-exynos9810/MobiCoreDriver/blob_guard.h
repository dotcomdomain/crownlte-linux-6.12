/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte TEE staging (T3): decision logic of the blob-pinned guard.
 *
 * Pure function over the KERNEL's own copy of a TA/driver blob (struct
 * tee_object->data, the exact buffer later mapped for the SWd), so there is
 * no time-of-check/time-of-use window with userspace. Shared verbatim by
 * main.c and the host fixture tests/blob-guard-fixture.c; the includer
 * provides u8/u32, memcmp/memset, sha256(), struct mclf_header_v2 and
 * SERVICE_TYPE_* (mci/mcloadformat.h).
 *
 * DT list format (samsung,tee-trustlet-allow / samsung,tee-driver-allow):
 * [uuid(16) sha256(32)] x N, N <= MC_PIN_MAX. Anything else allows nothing.
 */
#ifndef _MC_BLOB_GUARD_H_
#define _MC_BLOB_GUARD_H_

#define MC_PIN_UUID_LEN	16
#define MC_PIN_SHA_LEN	32
#define MC_PIN_LEN	(MC_PIN_UUID_LEN + MC_PIN_SHA_LEN)
#define MC_PIN_MAX	4

struct mc_blob_pin {
	u8	uuid[MC_PIN_UUID_LEN];
	u8	sha256[MC_PIN_SHA_LEN];
};

enum mc_blob_verdict {
	MC_BLOB_ALLOWED = 0,
	MC_BLOB_NO_PINS,	/* no pin list for this path */
	MC_BLOB_SP,		/* SP trustlet (container header): never */
	MC_BLOB_SHORT,		/* shorter than an MCLF v2 header */
	MC_BLOB_MAGIC,		/* not MCLF */
	MC_BLOB_TYPE,		/* wrong service type for this path */
	MC_BLOB_UUID,		/* UUID not pinned */
	MC_BLOB_DIGEST,		/* UUID pinned, SHA-256 differs */
	MC_BLOB_REQ_UUID,	/* header UUID is not the requested UUID */
};

static const char * const mc_blob_why[] = {
	"allowed", "no pins", "SP trustlet", "short", "magic", "service type",
	"uuid not pinned", "sha256 mismatch", "uuid not the requested uuid",
};

/* Is there a pin for this UUID at all? */
static inline int mc_blob_pinned(const u8 *uuid, const struct mc_blob_pin *pins,
				 int npins)
{
	int i;

	/* MC_PIN_MAX: an explicit bound for FORTIFY_SOURCE (pin arrays hold 4) */
	for (i = 0; i < npins && i < MC_PIN_MAX; i++)
		if (!memcmp(pins[i].uuid, uuid, MC_PIN_UUID_LEN))
			return 1;
	return 0;
}

/* Byte length of a DT pin list -> number of pins, or -1 if malformed. */
static inline int mc_blob_pins_count(int nbytes)
{
	if (nbytes <= 0 || nbytes % MC_PIN_LEN || nbytes / MC_PIN_LEN > MC_PIN_MAX)
		return -1;
	return nbytes / MC_PIN_LEN;
}

/*
 * data/len/header_length: the tee_object. want_type: SERVICE_TYPE_DRIVER
 * (admin LOAD_DRIVER) or SERVICE_TYPE_SYSTEM_TRUSTLET (open-trustlet and the
 * guarded registry path). want_uuid: NULL for a client/daemon-chosen blob
 * (open-trustlet, LOAD_DRIVER); the requested UUID for a registry blob, whose
 * header must then name exactly that UUID.
 * Out: digest (zero unless hashed), *uuid (NULL if no header), *type.
 * The SHA-256 covers the whole copy (data[0..len)), i.e. the exact vendor
 * file bytes for a system trustlet or driver (header_length == 0).
 */
static inline enum mc_blob_verdict
mc_blob_check(const u8 *data, size_t len, u32 header_length, u32 want_type,
	      const u8 *want_uuid, const struct mc_blob_pin *pins, int npins,
	      u8 digest[MC_PIN_SHA_LEN], const u8 **uuid, u32 *type)
{
	const struct mclf_header_v2 *h = (const void *)data;
	int i, uuid_hit = 0;

	memset(digest, 0, MC_PIN_SHA_LEN);
	*uuid = NULL;
	*type = 0;
	if (npins <= 0)
		return MC_BLOB_NO_PINS;
	if (header_length)
		return MC_BLOB_SP;
	if (len < sizeof(*h))
		return MC_BLOB_SHORT;
	*uuid = h->uuid.value;
	*type = (u32)h->service_type;
	if (h->intro.magic != MC_SERVICE_HEADER_MAGIC_BE &&
	    h->intro.magic != MC_SERVICE_HEADER_MAGIC_LE)
		return MC_BLOB_MAGIC;
	if ((u32)h->service_type != want_type)
		return MC_BLOB_TYPE;
	if (want_uuid && memcmp(h->uuid.value, want_uuid, MC_PIN_UUID_LEN))
		return MC_BLOB_REQ_UUID;
	sha256(data, len, digest);
	for (i = 0; i < npins; i++) {
		if (memcmp(pins[i].uuid, h->uuid.value, MC_PIN_UUID_LEN))
			continue;
		uuid_hit = 1;
		if (!memcmp(pins[i].sha256, digest, MC_PIN_SHA_LEN))
			return MC_BLOB_ALLOWED;
	}
	return uuid_hit ? MC_BLOB_DIGEST : MC_BLOB_UUID;
}

#endif /* _MC_BLOB_GUARD_H_ */
