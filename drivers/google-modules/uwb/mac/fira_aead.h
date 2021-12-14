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

#ifndef FIRA_AEAD_H
#define FIRA_AEAD_H

#include <linux/types.h>
#include <linux/skbuff.h>

struct fira_aead;

/**
 * fira_aead_set_key() - Set key used for encryption/decryption.
 * @aead: Context.
 * @key: AES payload key (key size 128).
 *
 * Return: 0 or error.
 */
int fira_aead_set_key(struct fira_aead *aead, const u8 *key);

/**
 * fira_aead_encrypt() - Encrypt payload.
 * @aead: Context.
 * @skb: Buffer containing the frame to encrypt.
 * @header_len: Length of the MAC header, used for authentication.
 * @src_short_addr: Source short address.
 * @counter: Counter used for the nonce.
 *
 * Return: 0 or error.
 */
int fira_aead_encrypt(struct fira_aead *aead, struct sk_buff *skb,
		      unsigned int header_len, __le16 src_short_addr,
		      u32 counter);

/**
 * fira_aead_decrypt_scf_check() - Check security control field before
 * decryption.
 * @scf: Security control field.
 *
 * Return: true if good.
 */
bool fira_aead_decrypt_scf_check(u8 scf);

/**
 * fira_aead_decrypt_prepare() - Prepare the frame before decryption.
 * @skb: Buffer containing the frame to decrypt.
 *
 * Return: 0 or error.
 */
int fira_aead_decrypt_prepare(struct sk_buff *skb);

/**
 * fira_aead_decrypt() - Decrypt payload.
 * @aead: Context.
 * @skb: Buffer containing the frame to decrypt. MAC header should be present
 * before data.
 * @header_len: Length of the MAC header, used for authentication.
 * @src_short_addr: Source short address.
 * @counter: Counter used for the nonce.
 *
 * NOTE: This function must be called after calling fira_aead_decrypt_prepare.
 *
 * Return: 0 or error. In particular, -EBADMSG is returned if authentication tag
 * is wrong.
 */
int fira_aead_decrypt(struct fira_aead *aead, struct sk_buff *skb,
		      unsigned int header_len, __le16 src_short_addr,
		      u32 counter);

/**
 * fira_aead_destroy() - Release memory used for payload encryption/decryption.
 * @aead: Context to destroy.
 */
void fira_aead_destroy(struct fira_aead *aead);

#endif /* FIRA_AEAD_H */
