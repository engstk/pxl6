/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.  Please contact Qorvo to inquire about licensing terms.
 */

#ifndef NET_MCPS802154_FIRA_CRYPTO_H
#define NET_MCPS802154_FIRA_CRYPTO_H

#include <crypto/aes.h>
#include <linux/types.h>

#include "fira_aead_impl.h"

struct fira_local;
struct fira_session;

/**
 * struct fira_crypto - Crypto context for sessions. This contains sensitive data
 * and must be handled specially to avoid leaking information.
 */
struct fira_crypto {
	/**
	 * @session_key: Session key. This is a constant for static STS. Size is
	 * given by @key_size.
	 */
	u8 session_key[AES_KEYSIZE_256];
	/**
	 * @data_protection_key: Data protection key, used to derive other
	 * material. Size is given by @key_size.
	 */
	u8 data_protection_key[AES_KEYSIZE_256];
	/**
	 * @sts_v: STS V, composed of the derived authentication initialization
	 * vector, V upper 64 (for static STS) and STS index, used for STS generation.
	 *
	 * STS index must be updated for each frame.
	 */
	u8 sts_v[AES_BLOCK_SIZE];
	/**
	 * @derived_authentication_key: Derived authentication key, used for STS
	 * generation.
	 */
	u8 derived_authentication_key[AES_KEYSIZE_128];
	/**
	 * @derived_payload_key: Derived payload key, used to encrypt frame
	 * payload.
	 */
	u8 derived_payload_key[AES_KEYSIZE_128];
	/**
	 * @config_digest: Digest of the configuration, used as input for key
	 * derivation.
	 */
	u8 config_digest[AES_BLOCK_SIZE];
	/**
	 * @sts_index_init: Initial value of the STS index, ignore MSB.
	 */
	u32 sts_index_init;
	/**
	 * @key_size: Size of the session key and data protection key. All other
	 * keys are 128 bit.
	 */
	int key_size;
	/**
	 * @aead: Context for payload encryption/decryption.
	 */
	struct fira_aead aead;
};

/**
 * fira_crypto_derive_per_session() - Prepare crypto material per session.
 * @local: FiRa context.
 * @session: Session.
 *
 * Prepare everything which is generated once per session.
 *
 * Return: 0 or error.
 */
int fira_crypto_derive_per_session(struct fira_local *local,
				   struct fira_session *session);

/**
 * fira_crypto_derive_per_rotation() - Prepare crypto material per rotation.
 * @local: FiRa context.
 * @session: Session.
 * @sts_index: STS index at time of rotation. Ignored for static STS.
 *
 * Prepare keys which are generated at initialization and on key rotation.
 *
 * Return: 0 or error.
 */
int fira_crypto_derive_per_rotation(struct fira_local *local,
				    struct fira_session *session,
				    u32 sts_index);

#ifndef CONFIG_MCPS802154_DISABLE_AUTO_TEST

/**
 * fira_crypto_test() - Autotest for crypto.
 *
 * Return: 0 or error.
 */
int fira_crypto_test(void);

#else

static inline int fira_crypto_test(void)
{
	return 0;
}

#endif

#endif /* NET_MCPS802154_FIRA_CRYPTO_H */
