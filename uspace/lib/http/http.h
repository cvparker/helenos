/*
 * Copyright (c) 2013 Martin Sucha
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

/** @addtogroup http
 * @{
 */
/**
 * @file
 */

#ifndef HTTP_HTTP_H_
#define HTTP_HTTP_H_

#include <net/socket.h>
#include <adt/list.h>
#include <inet/addr.h>

typedef struct {
	char *host;
	uint16_t port;
	inet_addr_t addr;

	bool connected;
	int conn_sd;
	
	size_t buffer_size;
	char *recv_buffer;
	size_t recv_buffer_in;
	size_t recv_buffer_out;
} http_t;

typedef struct {
	uint8_t minor;
	uint8_t major;
} http_version_t;

typedef struct {
	link_t link;
	char *name;
	char *value;
} http_header_t;

typedef struct {
	char *method;
	char *path;
	list_t headers;
} http_request_t;

typedef struct {
	http_version_t version;
	uint16_t status;
	char *message;
	list_t headers;
} http_response_t;

extern http_t *http_create(const char *, uint16_t);
extern int http_connect(http_t *);
extern http_header_t *http_header_create(const char *, const char *);
extern http_header_t *http_header_create_no_copy(char *, char *);
extern void http_header_destroy(http_header_t *);
extern http_request_t *http_request_create(const char *, const char *);
extern void http_request_destroy(http_request_t *);
extern int http_request_format(http_request_t *, char **, size_t *);
extern int http_send_request(http_t *, http_request_t *);
extern int http_parse_status(const char *, http_version_t *, uint16_t *,
    char **);
extern int http_parse_header(const char *, char **, char **);
extern int http_receive_response(http_t *, http_response_t **);
extern int http_receive_body(http_t *, void *, size_t);
extern void http_response_destroy(http_response_t *);
extern int http_close(http_t *);
extern void http_destroy(http_t *);

#endif

/** @}
 */