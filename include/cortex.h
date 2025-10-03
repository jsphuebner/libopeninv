/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2025 David J. Fiddes <D.J@fiddes.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CORTEX_H
#define CORTEX_H

#include <stdint.h>

/** Do not mask any interrupts */
#define CM_BASEPRI_ENABLE_INTERRUPTS 0

/** \brief Set the BASEPRI register to the given priority level */
inline __attribute__((always_inline)) void cm_set_basepriority(uint32_t new_priority)
{
    __asm__ volatile("msr basepri, %0 " ::"r"(new_priority) : "memory");
}

#endif // CORTEX_H
