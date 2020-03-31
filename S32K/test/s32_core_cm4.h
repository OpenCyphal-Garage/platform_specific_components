/*
 * Copyright (c) 2020, NXP. All rights reserved.
 * Distributed under The MIT License.
 */

/**
 * @file
 * Dummy file to support unit testing.
 */

#ifndef S32_CORE_CM4_H_INCLUDED
#define S32_CORE_CM4_H_INCLUDED

#define REV_BYTES_32(a, b) \
    (b = ((a & 0xFF000000U) >> 24U) | ((a & 0xFF0000U) >> 8U) | ((a & 0xFF00U) << 8U) | ((a & 0xFFU) << 24U))

#endif // S32_CORE_CM4_H_INCLUDED
