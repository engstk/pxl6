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

#include "fira_aead_impl.h"

#include <asm/unaligned.h>
#include <crypto/aes.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/ieee802154.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <net/mcps802154_frame.h>

#define FIRA_AEAD_AUTHSIZE 8

int fira_aead_set_key(struct fira_aead *aead, const u8 *key)
{
	struct crypto_aead *tfm;
	int r;

	crypto_free_aead(aead->tfm);
	aead->tfm = NULL;

	tfm = crypto_alloc_aead("ccm(aes)", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	r = crypto_aead_setkey(tfm, key, AES_KEYSIZE_128);
	if (r)
		goto err_free_tfm;

	r = crypto_aead_setauthsize(tfm, FIRA_AEAD_AUTHSIZE);
	if (r)
		goto err_free_tfm;

	aead->tfm = tfm;

	return 0;

err_free_tfm:
	crypto_free_aead(tfm);
	return r;
}

static void fira_aead_fill_iv(u8 *iv, __le16 src_short_addr, u32 counter)
{
	u8 *ivp;

	ivp = iv;
	*ivp++ = sizeof(u16) - 1; /* Only set L', rest is filled by CCM. */
	memset(ivp, 0,
	       IEEE802154_EXTENDED_ADDR_LEN - IEEE802154_SHORT_ADDR_LEN);
	ivp += IEEE802154_EXTENDED_ADDR_LEN - IEEE802154_SHORT_ADDR_LEN;
	put_unaligned_be16(le16_to_cpu(src_short_addr), ivp);
	ivp += IEEE802154_SHORT_ADDR_LEN;
	put_unaligned_be32(counter, ivp);
	ivp += sizeof(u32);
	*ivp++ = IEEE802154_SCF_SECLEVEL_ENC_MIC64;
	/* Filled by CCM. */
	*ivp++ = 0;
	*ivp++ = 0;
}

int fira_aead_encrypt(struct fira_aead *aead, struct sk_buff *skb,
		      unsigned int header_len, __le16 src_short_addr,
		      u32 counter)
{
	u8 iv[AES_BLOCK_SIZE];
	struct scatterlist sg;
	struct aead_request *req;
	int r;

	if (skb_tailroom(skb) < FIRA_AEAD_AUTHSIZE)
		return -ENOBUFS;

	fira_aead_fill_iv(iv, src_short_addr, counter);

	req = aead_request_alloc(aead->tfm, GFP_ATOMIC);
	if (!req)
		return -ENOMEM;

	skb->data[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN] =
		IEEE802154_SCF_SECLEVEL_ENC_MIC64 |
		IEEE802154_SCF_NO_FRAME_COUNTER;

	sg_init_one(&sg, skb->data, skb->len + FIRA_AEAD_AUTHSIZE);

	aead_request_set_callback(req, 0, NULL, NULL);
	aead_request_set_crypt(req, &sg, &sg, skb->len - header_len, iv);
	aead_request_set_ad(req, header_len);

	r = crypto_aead_encrypt(req);

	aead_request_free(req);

	if (!r)
		skb_put(skb, FIRA_AEAD_AUTHSIZE);
	else
		skb->data[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN] =
			IEEE802154_SCF_NO_FRAME_COUNTER;

	return r;
}

bool fira_aead_decrypt_scf_check(u8 scf)
{
	return scf == (IEEE802154_SCF_SECLEVEL_ENC_MIC64 |
		       IEEE802154_SCF_NO_FRAME_COUNTER);
}

int fira_aead_decrypt_prepare(struct sk_buff *skb)
{
	if (skb->len < FIRA_AEAD_AUTHSIZE)
		return -EBADMSG;
	skb_trim(skb, skb->len - FIRA_AEAD_AUTHSIZE);
	return 0;
}

int fira_aead_decrypt(struct fira_aead *aead, struct sk_buff *skb,
		      unsigned int header_len, __le16 src_short_addr,
		      u32 counter)
{
	u8 iv[AES_BLOCK_SIZE];
	struct scatterlist sg;
	struct aead_request *req;
	u8 *header;
	int r, payload_auth_len;

	payload_auth_len = skb->len + FIRA_AEAD_AUTHSIZE;

	fira_aead_fill_iv(iv, src_short_addr, counter);

	req = aead_request_alloc(aead->tfm, GFP_ATOMIC);
	if (!req)
		return -ENOMEM;

	header = skb->data - header_len;
	sg_init_one(&sg, header, header_len + payload_auth_len);

	aead_request_set_callback(req, 0, NULL, NULL);
	aead_request_set_crypt(req, &sg, &sg, payload_auth_len, iv);
	aead_request_set_ad(req, header_len);

	r = crypto_aead_decrypt(req);

	aead_request_free(req);

	if (!r) {
		header[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN] =
			IEEE802154_SCF_NO_FRAME_COUNTER;
	}

	return r;
}

void fira_aead_destroy(struct fira_aead *aead)
{
	crypto_free_aead(aead->tfm);
}
