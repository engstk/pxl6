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
#include "linux/kernel.h"
#include "linux/limits.h"
#include "utils_fixed_point.h"

static const s16 pi = 6434; /* Same as round((M_PI * K)). */
static const s16 pi2 = 3217; /* Same as round((M_PI * K) / 2). */

s16 sat_fp(s32 x)
{
	if (x > S16_MAX)
		return S16_MAX;
	else if (x < S16_MIN)
		return S16_MIN;
	else
		return x;
}

s16 div_fp(s16 a, s16 b)
{
	/* Pre-multiply by the base (upscale to Q16 so that the result will be
	 * in Q format). */
	s32 temp = (s32)a * K;

	/* Rounding: mid values are rounded up (down for negative values). */
	if ((temp >= 0 && b >= 0) || (temp < 0 && b < 0))
		temp += K / 2;
	else
		temp -= K / 2;
	return temp / b;
}

s16 mult_fp(s16 a, s16 b)
{
	s32 temp;

	temp = (s32)a * (s32)b;
	/* Rounding */
	if (temp >= 0) {
		temp += K / 2;
	} else {
		temp -= K / 2;
	}
	/* Correct by dividing by base and saturate result. */
	return sat_fp(temp / K);
}

s16 pow_fp(s16 x, int n)
{
	int i;
	s16 y = K;

	for (i = 1; i <= n; i++)
		y = mult_fp(y, x);

	return y;
}

s16 sqrt_fp(s16 x)
{
	uint32_t t, q, b, r;

	r = x;
	b = 1 << (Q + 2);
	q = 0;
	while (b > 0) {
		t = q + b;
		if (r >= t) {
			r -= t;
			q = t + b; /* Equivalent to q += 2*b. */
		}
		r <<= 1;
		b >>= 1;
	}
	q >>= 2;
	return q;
}

s16 asin_fp(s16 x)
{
	static const s16 a0 = 3217; /* Same as pi2. */
	static const s16 a1 = -434; /* Same as round(-0.2121144 * K). */
	static const s16 a2 = 152; /* Same as round(0.074261 * K). */
	static const s16 a3 = -38; /* Same as (-0.0187293 * K). */
	s16 xx, y;

	/* Same as : abs(x) in stdlib, which is not available. */
	xx = x < 0 ? -x : x;
	y = pi2 - mult_fp(sqrt_fp(K - xx), a0 + mult_fp(a1, xx) +
						   mult_fp(a2, pow_fp(xx, 2)) +
						   mult_fp(a3, pow_fp(xx, 3)));

	if (x < 0)
		return -y;
	return y;
}

s16 map_q11_to_2pi(s16 x)
{
	s32 temp = (s32)(x * S16_MAX);
	temp /= pi;

	return sat_fp(temp);
}
