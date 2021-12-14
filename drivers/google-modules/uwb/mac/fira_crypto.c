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

#include "fira_crypto.h"

#include "fira_cmac.h"
#include "fira_region.h"
#include "fira_session.h"

#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <net/mcps802154_frame.h>

#define FIRA_STATIC_STS_SESSION_KEY "StaticTSStaticTS"

/**
 * fira_crypto_config_digest() - Compute session config digest.
 * @local: FiRa context.
 * @session: Session.
 *
 * Return: 0 or error.
 */
static int fira_crypto_config_digest(struct fira_local *local,
				     struct fira_session *session)
{
	u8 derivation_data[17];
	u8 *p = derivation_data;
	int slot_duration_us;
	static const u8 zero_key[AES_KEYSIZE_128];
	const struct mcps802154_channel *channel;

	channel = mcps802154_get_current_channel(local->llhw);

	slot_duration_us = session->params.slot_duration_dtu * 1000 /
			   (local->llhw->dtu_freq_hz / 1000);

	*p++ = session->params.ranging_round_usage;
	*p++ = session->params.sts_config;
	*p++ = session->params.multi_node_mode;
	*p++ = session->params.channel_number ?: channel->channel;
	put_unaligned_be16(slot_duration_us, p);
	p += sizeof(u16);
	*p++ = session->params.mac_fcs_type;
	*p++ = session->params.rframe_config;
	*p++ = session->params.preamble_code_index ?: channel->preamble_code;
	*p++ = session->params.sfd_id;
	*p++ = session->params.psdu_data_rate;
	*p++ = session->params.preamble_duration;
	*p++ = 3; /* Constant. */
	put_unaligned_be32(session->id, p);

	return fira_digest(zero_key, sizeof(zero_key), derivation_data,
			   sizeof(derivation_data),
			   session->crypto.config_digest);
}

int fira_crypto_derive_per_session(struct fira_local *local,
				   struct fira_session *session)
{
	struct fira_crypto *crypto = &session->crypto;
	u8 sts_index_init_tmp[AES_KEYSIZE_128];
	int r;

	if (session->params.sts_config != FIRA_STS_CONFIG_STATIC)
		return -EOPNOTSUPP;

	r = fira_crypto_config_digest(local, session);
	if (r)
		goto out;

	memcpy(session->crypto.session_key, FIRA_STATIC_STS_SESSION_KEY,
	       AES_KEYSIZE_128);
	session->crypto.key_size = AES_KEYSIZE_128;

	r = fira_kdf(crypto->session_key, crypto->key_size, "DataPrtK",
		     crypto->config_digest, crypto->data_protection_key,
		     crypto->key_size);
	if (r)
		goto out;

	r = fira_kdf(crypto->data_protection_key, crypto->key_size, "StsIndIn",
		     crypto->config_digest, sts_index_init_tmp,
		     AES_KEYSIZE_128);
	if (r)
		goto out;
	crypto->sts_index_init =
		get_unaligned_be32(sts_index_init_tmp + AES_KEYSIZE_128 -
				   sizeof(u32)) &
		0x7fffffff;
out:
	memzero_explicit(sts_index_init_tmp, sizeof(sts_index_init_tmp));
	return r;
}

int fira_crypto_derive_per_rotation(struct fira_local *local,
				    struct fira_session *session, u32 sts_index)
{
	struct fira_crypto *crypto = &session->crypto;
	u8 context[AES_BLOCK_SIZE];
	u8 derived_authentication_iv[AES_BLOCK_SIZE];
	u32 crypto_sts_index;
	u32 sts_v_counter;
	u8 *sts_v;
	int r;

	/* Zero for Static STS. */
	crypto_sts_index = 0;

	memcpy(context, crypto->config_digest + sizeof(u32),
	       AES_BLOCK_SIZE - sizeof(u32));
	put_unaligned_be32(crypto_sts_index,
			   context + AES_BLOCK_SIZE - sizeof(u32));

	r = fira_kdf(crypto->data_protection_key, crypto->key_size, "DerAuthI",
		     context, derived_authentication_iv, AES_KEYSIZE_128);
	if (r)
		goto out;
	sts_v = crypto->sts_v;
	memcpy(sts_v, session->params.vupper64, FIRA_VUPPER64_SIZE);
	sts_v += FIRA_VUPPER64_SIZE;
	memset(sts_v, 0, sizeof(u32));
	sts_v += sizeof(u32);
	sts_v_counter = get_unaligned_be32(derived_authentication_iv +
					   AES_BLOCK_SIZE - sizeof(u32)) &
			0x7fffffff;
	put_unaligned_be32(sts_v_counter, sts_v);

	r = fira_kdf(crypto->data_protection_key, crypto->key_size, "DerAuthK",
		     context, crypto->derived_authentication_key,
		     AES_KEYSIZE_128);
	if (r)
		goto out;

	r = fira_kdf(crypto->data_protection_key, crypto->key_size, "DerPaylK",
		     context, crypto->derived_payload_key, AES_KEYSIZE_128);
	if (r)
		goto out;

	r = fira_aead_set_key(&crypto->aead, crypto->derived_payload_key);

out:
	memzero_explicit(context, sizeof(context));
	memzero_explicit(derived_authentication_iv,
			 sizeof(derived_authentication_iv));
	return r;
}

#ifndef CONFIG_MCPS802154_DISABLE_AUTO_TEST

int fira_crypto_test(void)
{
	/* LCOV_EXCL_START */
	static const u8 zero_key[AES_KEYSIZE_128];
	struct sk_buff *skb = NULL;
	int r;
	struct fira_round_hopping_sequence round_hopping_sequence;

	static const u8 digest_data[] = { 0x02, 0x00, 0x00, 0x09, 0x07, 0xD0,
					  0x00, 0x03, 0x0a, 0x02, 0x00, 0x01,
					  0x03, 0x01, 0x23, 0x45, 0x67 };
	u8 digest[AES_BLOCK_SIZE];
	static const u8 digest_expect[] = { 0xa0, 0x43, 0x90, 0xcf, 0x8a, 0x33,
					    0xf6, 0xeb, 0x7e, 0x2f, 0xc3, 0x78,
					    0x87, 0xb6, 0xb2, 0xa3 };

	static const u8 frame_key[] = { 0xa5, 0x5f, 0xab, 0x83, 0xb6, 0x20,
					0xf9, 0xf6, 0xa4, 0x7c, 0xdb, 0x72,
					0x91, 0x7c, 0x73, 0x8a };
	static const u8 frame[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x20, 0x13, 0x00, 0xff, 0x18, 0x5a,
		0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x67, 0x45,
		0x23, 0x01, 0x78, 0xbe, 0x9b, 0x0b, 0x00, 0x3f, 0x1b, 0x90,
		0xff, 0x18, 0x5a, 0x03, 0x05, 0x00, 0x00, 0x03, 0x42, 0x55,
		0x01, 0x04, 0x44, 0x55, 0x03, 0x07, 0x42, 0x55, 0x05, 0x09,
		0x42, 0x55, 0x09, 0x0a, 0x44, 0x55, 0x0b
	};
	static const u8 frame_enc[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff, 0x18, 0x5a,
		0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x67, 0x45,
		0x23, 0x01, 0x78, 0xbe, 0x9b, 0x0b, 0x00, 0x3f, 0xcb, 0xa4,
		0xfd, 0x37, 0xd1, 0x99, 0x44, 0x88, 0x7c, 0x2b, 0xec, 0x2e,
		0x1a, 0x99, 0x8e, 0x80, 0x61, 0x7c, 0x44, 0xb5, 0xe8, 0xe3,
		0xf3, 0x35, 0x3a, 0xb9, 0xf2, 0x29, 0x1b, 0x80, 0x4b, 0xba,
		0xe1, 0xa9, 0x2a, 0x20, 0x28
	};
	const unsigned int frame_header_len = 28;
	const __le16 frame_src_short_addr = 0xaaa1;
	const u32 frame_counter = 0;
	struct fira_aead aead = {};

	/* Test digest. */
	r = fira_digest(zero_key, AES_KEYSIZE_128, digest_data,
			sizeof(digest_data), digest);
	if (r != 0 || memcmp(digest, digest_expect, sizeof(digest)) != 0) {
		pr_err("fira_digest test failed: r = %d\n", r);
		print_hex_dump(KERN_ERR, "digest:        ", DUMP_PREFIX_NONE,
			       16, 1, digest, sizeof(digest), false);
		print_hex_dump(KERN_ERR, "digest_expect: ", DUMP_PREFIX_NONE,
			       16, 1, digest_expect, sizeof(digest_expect),
			       false);
		return -EINVAL;
	}

	/* Test AEAD. */
	r = fira_aead_set_key(&aead, frame_key);
	if (r != 0) {
		pr_err("fira_aead_set_key test failed: r = %d\n", r);
		return -EINVAL;
	}

	/* AEAD enc. */
	skb = alloc_skb(sizeof(frame_enc), GFP_KERNEL);
	if (!skb) {
		r = -ENOMEM;
		goto out;
	}
	skb_put_data(skb, frame, sizeof(frame));

	r = fira_aead_encrypt(&aead, skb, frame_header_len,
			      frame_src_short_addr, frame_counter);
	if (r != 0 || skb->len != sizeof(frame_enc) ||
	    memcmp(skb->data, frame_enc, sizeof(frame_enc)) != 0) {
		pr_err("fira_aead_encrypt test failed: r = %d\n", r);
		print_hex_dump(KERN_ERR, "frame_enc: ", DUMP_PREFIX_NONE, 16, 1,
			       skb->data, skb->len, false);
		print_hex_dump(KERN_ERR, "expect:    ", DUMP_PREFIX_NONE, 16, 1,
			       frame_enc, sizeof(frame_enc), false);
		r = -EINVAL;
		goto out;
	}

	/* AEAD dec. */
	kfree_skb(skb);
	skb = alloc_skb(sizeof(frame_enc), GFP_KERNEL);
	if (!skb) {
		r = -ENOMEM;
		goto out;
	}
	skb_put_data(skb, frame_enc, sizeof(frame_enc));
	skb_pull(skb, frame_header_len);

	/* Prepare cannot fail. */
	fira_aead_decrypt_prepare(skb);
	r = fira_aead_decrypt(&aead, skb, frame_header_len,
			      frame_src_short_addr, frame_counter);
	skb_push(skb, frame_header_len);
	if (r != 0 || skb->len != sizeof(frame) ||
	    memcmp(skb->data, frame, sizeof(frame)) != 0) {
		pr_err("fira_aead_decrypt test failed: r = %d\n", r);
		print_hex_dump(KERN_ERR, "frame:  ", DUMP_PREFIX_NONE, 16, 1,
			       skb->data, skb->len, false);
		print_hex_dump(KERN_ERR, "expect: ", DUMP_PREFIX_NONE, 16, 1,
			       frame, sizeof(frame), false);
		r = -EINVAL;
		goto out;
	}

	/* AEAD dec bad tag. */
	kfree_skb(skb);
	skb = alloc_skb(sizeof(frame_enc), GFP_KERNEL);
	if (!skb) {
		r = -ENOMEM;
		goto out;
	}
	skb_put_data(skb, frame_enc, sizeof(frame_enc));
	skb_pull(skb, frame_header_len);
	skb->data[skb->len - 1]++;

	/* Prepare cannot fail. */
	fira_aead_decrypt_prepare(skb);
	r = fira_aead_decrypt(&aead, skb, frame_header_len,
			      frame_src_short_addr, frame_counter);
	if (r != -EBADMSG) {
		pr_err("fira_aead_decrypt bad msg test failed: r = %d\n", r);
		r = -EINVAL;
		goto out;
	}

	/* Test ecb(aes) presence for hopping. */
	r = fira_round_hopping_crypto_init(&round_hopping_sequence);
	if (r)
		goto out;
	fira_round_hopping_crypto_destroy(&round_hopping_sequence);

	r = 0;
out:
	kfree_skb(skb);
	fira_aead_destroy(&aead);

	/* LCOV_EXCL_STOP */
	return r;
}

#endif /* !CONFIG_MCPS802154_DISABLE_AUTO_TEST */
