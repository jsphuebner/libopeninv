/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2024 David J. Fiddes <D.J@fiddes.net>
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

// Minimal project parameters to test libopeninv
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    VALUE_ENTRY(amp,            "dig",   2013 ) \
    VALUE_ENTRY(pot,            "dig",   2015 ) \
    PARAM_ENTRY("inverter",   ocurlim,     "A",       -65536, 65536,  100,    22  )

extern const char* errorListString;