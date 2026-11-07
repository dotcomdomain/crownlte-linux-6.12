/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte TEE staging (T3): the guarded daemon-registry path.
 *
 * OPEN_SESSION and GP open-session (client_open_session(),
 * client_gp_open_session()) do not carry a blob: the kernel asks the daemon
 * for one (tee_object_get(), admin GET_TRUSTLET) and maps whatever bytes come
 * back. While the session guard is on, and after the UUID allow list has
 * accepted the requested UUID, the object handed to the SWd comes from here:
 *
 *  - UUID with a trustlet pin (samsung,tee-trustlet-allow): the daemon's
 *    bytes, accepted only as a plain MCLF system trustlet whose header UUID
 *    is the requested UUID and whose SHA-256 matches a pin for it. A fetch
 *    error is returned as is; there is no fallback to SWd selection.
 *  - UUID without a pin (FSD2, DebugSession): SWd selection only
 *    (tee_object_select(): a bare header holding the UUID, the SWd starts its
 *    own built-in). The daemon is never asked. Without TA-auth support
 *    (selection unavailable) the request is refused.
 *
 * Shared verbatim by main.c and tests/blob-guard-fixture.c. The includer
 * provides blob_guard.h, struct tee_object, struct mc_uuid_t,
 * tee_object_get/select/free, IS_ERR/PTR_ERR/ERR_PTR, EPERM and
 * SERVICE_TYPE_SYSTEM_TRUSTLET.
 */
#ifndef _MC_REGISTRY_GUARD_H_
#define _MC_REGISTRY_GUARD_H_

enum mc_reg_source {
	MC_REG_REFUSED,		/* no pin and no SWd selection: nothing asked */
	MC_REG_SWD_SELECT,	/* no pin: SWd selection, daemon not asked */
	MC_REG_PINNED,		/* pinned: daemon bytes, checked */
};

struct mc_reg_result {
	enum mc_reg_source	source;
	enum mc_blob_verdict	verdict;	/* MC_REG_PINNED with err == 0 */
	int			err;		/* MC_REG_PINNED: fetch error */
	int			have_uuid;	/* blob header UUID below valid */
	u32			type;
	u32			length;
	u8			uuid[MC_PIN_UUID_LEN];
	u8			digest[MC_PIN_SHA_LEN];
};

/*
 * Returns the object to map (caller frees it with tee_object_free()) or an
 * ERR_PTR; *r says which source was used and why, for the caller's log line.
 */
static inline struct tee_object *
mc_registry_object(const struct mc_uuid_t *uuid, bool gp, bool ta_auth,
		   const struct mc_blob_pin *pins, int npins,
		   struct mc_reg_result *r)
{
	struct tee_object *obj;
	const u8 *buuid;

	memset(r, 0, sizeof(*r));
	r->verdict = MC_BLOB_NO_PINS;
	if (!mc_blob_pinned(uuid->value, pins, npins)) {
		if (!ta_auth) {
			r->source = MC_REG_REFUSED;
			return ERR_PTR(-EPERM);
		}
		r->source = MC_REG_SWD_SELECT;
		return tee_object_select(uuid);
	}

	r->source = MC_REG_PINNED;
	obj = tee_object_get(uuid, gp);
	if (IS_ERR(obj)) {
		r->err = (int)PTR_ERR(obj);
		return obj;
	}

	r->length = obj->length;
	r->verdict = mc_blob_check(obj->data, obj->length, obj->header_length,
				   SERVICE_TYPE_SYSTEM_TRUSTLET, uuid->value,
				   pins, npins, r->digest, &buuid, &r->type);
	if (buuid) {
		memcpy(r->uuid, buuid, MC_PIN_UUID_LEN);
		r->have_uuid = 1;
	}
	if (r->verdict == MC_BLOB_ALLOWED)
		return obj;

	tee_object_free(obj);
	return ERR_PTR(-EPERM);
}

#endif /* _MC_REGISTRY_GUARD_H_ */
