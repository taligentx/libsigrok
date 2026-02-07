/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Nikhil Choudhary <nikhilc@taligentx.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Parser for the Zeeweii DSO3D12, an oscilloscope with an integrated
 * SD7501-based multimeter. The firmware parses the DMM chip output
 * to an ASCII string.
 */

#include <config.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "hardware/serial-dmm/protocol.h"

#undef LOG_PREFIX
#define LOG_PREFIX "zeeweii_dso3d12"

/*
 * Custom scan function to initialize the device instance without opening the
 * serial port to ensure the serial port is only initialized once at data
 * acquisition start.
 *
 * The Zeeweii DSO3D12 requires holding down the power button at startup to
 * enable the USB-serial controller, and resets if the serial port
 * is later re-initialized (as during data acquisition start).
 */
SR_PRIV GSList *sr_zeeweii_dso3d12_scan(struct sr_dev_driver *di, GSList *options)
{
	struct dmm_info *dmm;
	struct sr_config *src;
	GSList *l, *devices;
	const char *conn, *serialcomm;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	size_t ch_idx;
	char ch_name[12];

	dmm = (struct dmm_info *)di;
	conn = dmm->conn;
	serialcomm = dmm->serialcomm;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		case SR_CONF_SERIALCOMM:
			serialcomm = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (!conn)
		return NULL;

	if (dmm->dmm_state_init)
		dmm->dmm_state = dmm->dmm_state_init();

	/* Setup the device instance. */
	sdi = g_malloc0(sizeof(*sdi));
	sdi->status = SR_ST_INACTIVE;
	sdi->vendor = g_strdup(dmm->vendor);
	sdi->model = g_strdup(dmm->device);
	devc = g_malloc0(sizeof(*devc));
	sr_sw_limits_init(&devc->limits);
	sdi->inst_type = SR_INST_SERIAL;
	sdi->conn = sr_serial_dev_inst_new(conn, serialcomm);
	sdi->priv = devc;

	/* Create channel. */
	dmm->channel_count = 1;
	for (ch_idx = 0; ch_idx < dmm->channel_count; ch_idx++) {
		size_t ch_num;
		const char *fmt;
		fmt = "P%zu";
		if (dmm->channel_formats && dmm->channel_formats[ch_idx])
			fmt = dmm->channel_formats[ch_idx];
		ch_num = ch_idx + 1;
		snprintf(ch_name, sizeof(ch_name), fmt, ch_num);
		sr_channel_new(sdi, ch_idx, SR_CHANNEL_ANALOG, TRUE, ch_name);
	}

	/* Add device to result set. */
	devices = NULL;
	devices = g_slist_append(devices, sdi);

	return std_scan_complete(di, devices);
}

SR_PRIV int sr_zeeweii_dso3d12_packet_valid(void *st,
	const uint8_t *buf, size_t len, size_t *pkt_len)
{
	(void)st;
	gboolean has_digit = FALSE;
	gboolean has_letter = FALSE;

	/* Checks for newline-terminated data */
	for (size_t i = 0; i < len; i++) {
		if (buf[i] == '\n') {
			size_t current_len = i + 1;

			/* Checks for plausible data (at least 1 digit and 1 letter) */
			for (size_t j = 0; j < current_len; j++) {
				if (g_ascii_isdigit(buf[j]))
					has_digit = TRUE;
				if (g_ascii_isalpha(buf[j]))
					has_letter = TRUE;
				if (has_digit && has_letter)
					break;
			}

			if (has_digit && has_letter) {
				if (pkt_len)
					*pkt_len = current_len;
				return SR_PACKET_VALID;
			} else {
				sr_dbg("Discarding invalid packet.");
				return SR_PACKET_INVALID;
			}
		}
	}

	if (len > 16)
		return SR_PACKET_INVALID;

	/* Newline not found, request more data */
	return SR_PACKET_NEED_RX;
}

SR_PRIV int sr_zeeweii_dso3d12_parse(void *st, const uint8_t *buf, size_t len,
	double *val, struct sr_datafeed_analog *analog, void *info)
{
	(void)st; (void)len; (void)info;
	char s[16], *p, *endptr = NULL;
	double multiplier = 1.0;
	int digits = 0;

	/* Sanitizes data and null-terminates */
	size_t i = 0;
	while (i < sizeof(s) - 1 && buf[i] && buf[i] != '\n') {
		s[i] = (char)buf[i];
		i++;
	}
	s[i] = '\0';
	g_strstrip(s);

	/* Parses value */
	char *p_ol_decimal = strstr(s, "0.L");
	char *p_ol_regular = strstr(s, "0L");
	if (p_ol_decimal || p_ol_regular) {
		sr_spew("Received OL");
		*val = INFINITY;
		p = p_ol_decimal ? (p_ol_decimal + 3) : (p_ol_regular + 2);
	} else {
		*val = g_ascii_strtod(s, &endptr);
		if (endptr == s) {
			sr_err("Failed to parse numeric value from string: \"%s\"", s);
			return SR_ERR;
		}
		p = endptr;

		/* Calculate digits based on decimal point */
		char *dot = strchr(s, '.');
		if (dot) {
			for (char *q = dot + 1; *q && g_ascii_isdigit(*q); q++)
				digits++;
		}

		sr_spew("Parsed number: %.6f, hardware digits: %d, remaining string: \"%s\"",
			   *val, digits, p);
	}

	/* Multiplier and MQ */
	multiplier = 1.0;
	analog->meaning->mq = 0;
	analog->meaning->unit = 0;

	while (*p && (g_ascii_isspace((unsigned char)*p) || g_ascii_isdigit(*p) || *p == '.'))
		p++;

	/* Sets multiplier and digits based on prefix */
	if (*p == 'n') {multiplier = 1e-9; digits += 9; p++;}
	else if (*p == 'u') {multiplier = 1e-6; digits += 6; p++;}
	else if (*p == 'm') {multiplier = 1e-3; digits += 3; p++;}
	else if (*p == 'K') {multiplier = 1e3; digits -= 3; p++;}
	else if (*p == 'M') {multiplier = 1e6; digits -= 6; p++;}

	/* Parses measurement quantity */
	if (*p == 'V') {
		analog->meaning->mq = SR_MQ_VOLTAGE;
		analog->meaning->unit = SR_UNIT_VOLT;
	} else if (*p == 'A') {
		analog->meaning->mq = SR_MQ_CURRENT;
		analog->meaning->unit = SR_UNIT_AMPERE;
	} else if (*p == 'R') {
		analog->meaning->mq = SR_MQ_RESISTANCE;
		analog->meaning->unit = SR_UNIT_OHM;
	} else if (*p == 'F') {
		analog->meaning->mq = SR_MQ_CAPACITANCE;
		analog->meaning->unit = SR_UNIT_FARAD;
	}

	/* Apply multiplier */
	if (!isinf(*val)) {
		*val *= multiplier;
	}

	sr_dbg("Final value: %.9f, Total digits: %d, MQ: %d",
		   *val, digits, analog->meaning->mq);

	analog->encoding->is_float = TRUE;
	analog->encoding->is_signed = TRUE;
	analog->encoding->unitsize = sizeof(double);
	analog->encoding->digits = digits;
	analog->spec->spec_digits = digits;

	return SR_OK;
}
