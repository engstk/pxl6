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

#ifndef NET_UTILS_FIXED_POINT_H
#define NET_UTILS_FIXED_POINT_H

#include <linux/types.h>

/* Number of fixed digits after the radix point. */
#define Q 11
/* Scale of the fixed point. */
#define K (1 << Q)

/**
 * sat_fp() - Saturate the range of fixed-point
 * @x: fixed-point value on s32.
 *
 * Return: value saturate to s16 range
 */
s16 sat_fp(s32 x);

/**
 * div_fp() - Division between two fixed-point
 * @a: numerator fixed-point value.
 * @b: denominator fixed-point value.
 *
 * Return: result of the division
 */
s16 div_fp(s16 a, s16 b);

/**
 * mult_fp() - Multiplication between two fixed-point.
 * @a: first fixed-point value.
 * @b: second fixed-point value.
 *
 * Return: result of the multiplication
 */
s16 mult_fp(s16 a, s16 b);

/**
 * pow_fp() - x raised to the power of n with fixed-point.
 * @x: fixed-point base value.
 * @n: power value.
 *
 * Return: x raised to the power of n.
 */
s16 pow_fp(s16 x, int n);

/**
 * sqrt_fp() - Compute square root with fixed-point.
 * @x: x must be positive.
 *
 * Return: the s16 point square root of x (s16).
 *
 * Code imported/adapted from:
 * https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
 */
s16 sqrt_fp(s16 x);

/**
 * asin_fp() - asin function with fixed-point.
 * @x: fixed_point value.
 *
 * Return: radian angle in fixed-point.
 */
s16 asin_fp(s16 x);

/**
 * map_q11_to_2pi() - Map a Fixed Point azimuth angle
 * to an signed 16 bit interger
 * @x: angle as Q11 fixed_point value in range [-PI, PI]
 *
 * Return: the angle mapped to [INT16_MIN, INT16_MAX]
 */
s16 map_q11_to_2pi(s16 x);

#endif /* NET_UTILS_FIXED_POINT_H */
