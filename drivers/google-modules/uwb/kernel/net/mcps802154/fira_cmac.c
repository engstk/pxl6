/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#include "fira_cmac.h"

#include <asm/unaligned.h>
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <linux/err.h>

int fira_digest(const u8 *key, unsigned int key_len, const u8 *data,
		unsigned int data_len, u8 *out)
{
	struct crypto_shash *tfm;
	int r;

	tfm = crypto_alloc_shash("cmac(aes)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	r = crypto_shash_setkey(tfm, key, key_len);
	if (r)
		goto out;

	do {
		/* tfm need to be allocated for kernel < 4.20, so don't remove
		 * this do..while block. */
		SHASH_DESC_ON_STACK(desc, tfm);
		desc->tfm = tfm;

		r = crypto_shash_init(desc);
		if (r)
			goto out;

		r = crypto_shash_finup(desc, data, data_len, out);
	} while (0);

out:
	crypto_free_shash(tfm);
	return r;
}

int fira_kdf(const u8 *input_key, unsigned int input_key_len, const char *label,
	     const u8 *context, u8 *output_key, unsigned int output_key_len)
{
	u8 derivation_data[sizeof(u32) + FIRA_KDF_LABEL_LEN +
			   FIRA_KDF_CONTEXT_LEN + sizeof(u32)];
	u8 *p;

	if (output_key_len != AES_KEYSIZE_128)
		return -1;

	p = derivation_data;
	put_unaligned_be32(1, p);
	p += sizeof(u32);
	memcpy(p, label, FIRA_KDF_LABEL_LEN);
	p += FIRA_KDF_LABEL_LEN;
	memcpy(p, context, FIRA_KDF_CONTEXT_LEN);
	p += FIRA_KDF_CONTEXT_LEN;
	put_unaligned_be32(output_key_len * 8, p);

	return fira_digest(input_key, input_key_len, derivation_data,
			   sizeof(derivation_data), output_key);
}
