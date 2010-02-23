/*
 * Copyright (c) 2010 Jiri Svoboda
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LEX_T_H_
#define LEX_T_H_

/** Lexical element class */
typedef enum {
	lc_invalid,
	lc_eof,

	lc_ident,
	lc_lit_int,
	lc_lit_string,

	/* Keywords */
	lc_class,
	lc_constructor,
	lc_do,
	lc_else,
	lc_end,
	lc_except,
	lc_finally,
	lc_for,
	lc_fun,
	lc_new,
	lc_get,
	lc_if,
	lc_in,
	lc_int,
	lc_interface,
	lc_is,
	lc_override,
	lc_private,
	lc_prop,
	lc_protected,
	lc_public,
	lc_raise,
	lc_return,
	lc_set,
	lc_static,
	lc_string,
	lc_struct,
	lc_then,
	lc_this,
	lc_var,
	lc_with,
	lc_while,
	lc_yield,

	/* Operators */
	lc_period,
	lc_slash,
	lc_lparen,
	lc_rparen,
	lc_lsbr,
	lc_rsbr,
	lc_equal,
	lc_notequal,
	lc_lt,
	lc_gt,
	lc_lt_equal,
	lc_gt_equal,
	lc_assign,
	lc_plus,
	lc_increase,

	/* Punctuators */
	lc_comma,
	lc_colon,
	lc_scolon,

	lc__limit
} lclass_t;

typedef struct {
	/* String ID */
	int sid;
} lem_ident_t;

typedef struct {
	/* Integer value */
	int value;
} lem_lit_int_t;

typedef struct {
	/* String value */
	char *value;
} lem_lit_string_t;

/** Lexical element */
typedef struct {
	/* Lexical element class */
	lclass_t lclass;

	union {
		lem_ident_t ident;
		lem_lit_int_t lit_int;
		lem_lit_string_t lit_string;
	} u;

	/** Coordinates of this lexical element */
	int line_no, col_0;
} lem_t;

/** Lexer state object */
typedef struct lex {
	/** Input object */
	struct input *input;

	/** Lexing buffer */
	char *inbuf;

	/** Pointer to current position in lexing buffer */
	char *ibp;

	/** Number of the line currently in inbuf */
	int ib_line;

	/** Column number adjustment (due to tabs) */
	int col_adj;

	/** Curent lem */
	lem_t current;
} lex_t;

#endif
