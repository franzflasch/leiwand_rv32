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

`define NR_RV_REGS 32

//`define RV64

`ifdef RV64
    `define XLEN 64
    `define XLEN_BYTES 8
`else
    `define XLEN 32
    `define XLEN_BYTES 4
`endif

`define MEM_WIDTH 32
`define MEM_WIDTH_BYTES 4
`define MEM_OFFSET 2

/* Interrupt Concept taken from SiFive Coreplex Manual - Chapter 6 Interrupts */
/* https://static.dev.sifive.com/E31-Coreplex.pdf */
/* Machine Software Interrupt */
`define MSI_BIT 3
/* Machine Timer Interrupt */
`define MTI_BIT 7
/* Machine External Interrupt */
`define MEI_BIT 11
