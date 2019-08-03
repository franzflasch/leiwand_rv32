/*
 *   This file is part of leiwand_rv32.
 *   Copyright (c) 2019 Franz Flasch.
 *
 *   leiwand_rv32 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   leiwand_rv32 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with leiwand_rv32.  If not, see <https://www.gnu.org/licenses/>.
 */

`define BITS_TO_FIT(N) ($clog2(N+1))

/* Gives the number of bits needed to represent the number given in the argument */
`define HIGH_BIT_TO_FIT(N) (`BITS_TO_FIT(N)-1)
