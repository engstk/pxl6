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

#ifndef FIRA_CMAC_H
#define FIRA_CMAC_H

#include <crypto/aes.h>
#include <linux/types.h>

#define FIRA_KDF_LABEL_LEN 8
#define FIRA_KDF_CONTEXT_LEN 16

/**
 * fira_digest() - Compute a digest using cmac(aes).
 * @key: AES key.
 * @key_len: Length of AES key (AES_KEYSIZE_x).
 * @data: Input data.
 * @data_len: Input data length in octets.
 * @out: Output hash, with length AES_BLOCK_SIZE.
 *
 * Return: 0 or error.
 */
int fira_digest(const u8 *key, unsigned int key_len, const u8 *data,
		unsigned int data_len, u8 *out);

/**
 * fira_kdf() - Key derivation function.
 * @input_key: AES input key.
 * @input_key_len: Length of AES input key.
 * @label: KDF label (8 bytes).
 * @context: KDF context (16 bytes).
 * @output_key: AES output key.
 * @output_key_len: Length of AES output key.
 *
 * Return: 0 or error.
 */
int fira_kdf(const u8 *input_key, unsigned int input_key_len, const char *label,
	     const u8 *context, u8 *output_key, unsigned int output_key_len);

#endif /* FIRA_CMAC_H */
