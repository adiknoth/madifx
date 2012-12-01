#ifndef __SOUND_HDSPM_H
#define __SOUND_HDSPM_H
/*
 *   Copyright (C) 2003 Winfried Ritsch (IEM)
 *   based on hdsp.h from Thomas Charbonnel (thomas@undata.org)
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Maximum channels is 64 even on 56Mode you have 64playbacks to matrix */
#define HDSPM_MAX_CHANNELS      196

enum madifx_io_type {
	MADI,
	MADIface,
	AIO,
	AES32,
	RayDAT,
	MADIFX
};

enum madifx_speed {
	ss = 0,
	ds = 1,
	qs = 2
};

/* -------------------- IOCTL Peak/RMS Meters -------------------- */

struct madifx_level_buffer {
	uint32_t rms_out_pre[2 * 256];
	uint32_t peak_out_pre[256];

	uint32_t rms_in[2 * 256];
	uint32_t peak_in[256];

	uint32_t rms_play[2 * 256];
	uint32_t peak_play[256];

	uint32_t rms_out[2 * 256];
	uint32_t peak_out[256];

	uint32_t rms_in_pre[2 * 256];
	uint32_t peak_in_pre[256];

	uint8_t speed; /* enum {ss, ds, qs} */
};

#define SNDRV_MADIFX_GET_LEVELBUFFER \
	_IOR('H', 0x42, struct madifx_level_buffer)

/* ------------ CONFIG block IOCTL ---------------------- */

struct madifx_config {
	unsigned char pref_sync_ref;
	unsigned char wordclock_sync_check;
	unsigned char madi_sync_check;
	unsigned int system_sample_rate;
	unsigned int autosync_sample_rate;
	unsigned char system_clock_mode;
	unsigned char clock_source;
	unsigned char autosync_ref;
	unsigned char line_out;
	unsigned int passthru;
	unsigned int analog_out;
};

#define SNDRV_HDSPM_IOCTL_GET_CONFIG \
	_IOR('H', 0x41, struct madifx_config)

/**
 * If there's a TCO (TimeCode Option) board installed,
 * there are further options and status data available.
 * The madifx_ltc structure contains the current SMPTE
 * timecode and some status information and can be
 * obtained via SNDRV_HDSPM_IOCTL_GET_LTC or in the
 * madifx_status struct.
 **/

enum madifx_ltc_format {
	format_invalid,
	fps_24,
	fps_25,
	fps_2997,
	fps_30
};

enum madifx_ltc_frame {
	frame_invalid,
	drop_frame,
	full_frame
};

enum madifx_ltc_input_format {
	ntsc,
	pal,
	no_video
};

struct madifx_ltc {
	unsigned int ltc;

	enum madifx_ltc_format format;
	enum madifx_ltc_frame frame;
	enum madifx_ltc_input_format input_format;
};

#define SNDRV_HDSPM_IOCTL_GET_LTC _IOR('H', 0x46, struct madifx_mixer_ioctl)

/**
 * The status data reflects the device's current state
 * as determined by the card's configuration and
 * connection status.
 **/

enum madifx_sync {
	madifx_sync_no_lock = 0,
	madifx_sync_lock = 1,
	madifx_sync_sync = 2
};

enum madifx_madi_input {
	madifx_input_optical = 0,
	madifx_input_coax = 1
};

enum madifx_madi_channel_format {
	madifx_format_ch_64 = 0,
	madifx_format_ch_56 = 1
};

enum madifx_madi_frame_format {
	madifx_frame_48 = 0,
	madifx_frame_96 = 1
};

enum madifx_syncsource {
	syncsource_madi1 = 0,
	syncsource_madi2 = 1,
	syncsource_madi3 = 2,
	syncsource_aes = 3,
	syncsource_wc = 4,
	syncsource_none = 5
};

struct madifx_status {
	uint8_t card_type; /* enum madifx_io_type */
	enum madifx_syncsource autosync_source;

	uint64_t card_clock;
	uint32_t master_period;

	union {
		struct {
			uint8_t sync_wc; /* enum madifx_sync */
			uint8_t sync_madi; /* enum madifx_sync */
			uint8_t sync_tco; /* enum madifx_sync */
			uint8_t sync_in; /* enum madifx_sync */
			uint8_t madi_input; /* enum madifx_madi_input */
			uint8_t channel_format; /* enum madifx_madi_channel_format */
			uint8_t frame_format; /* enum madifx_madi_frame_format */
		} madi;
	} card_specific;
};

#define SNDRV_HDSPM_IOCTL_GET_STATUS \
	_IOR('H', 0x47, struct madifx_status)

/**
 * Get information about the card and its add-ons.
 **/

#define HDSPM_ADDON_TCO 1

struct madifx_version {
	uint8_t card_type; /* enum madifx_io_type */
	char cardname[20];
	unsigned int serial;
	unsigned short firmware_rev;
	int addons;
};

#define SNDRV_HDSPM_IOCTL_GET_VERSION _IOR('H', 0x48, struct madifx_version)

/* ------------- get Matrix Mixer IOCTL --------------- */

/* MADI mixer: 64inputs+64playback in 64outputs = 8192 => *4Byte =
 * 32768 Bytes
 */

/* organisation is 64 channelfader in a continuous memory block */
/* equivalent to hardware definition, maybe for future feature of mmap of
 * them
 */
/* each of 64 outputs has 64 infader and 64 outfader:
   Ins to Outs mixer[out].in[in], Outstreams to Outs mixer[out].pb[pb] */

#define HDSPM_MIXER_CHANNELS HDSPM_MAX_CHANNELS
#define MADIFX_LIST_LENGTH 4096
#define MADIFX_NUM_OUTPUT_GAINS 198 
#define MADIFX_NUM_LEVEL_PAGES 5
#define MADIFX_LEVEL_BUFFER_SIZE (MADIFX_NUM_LEVEL_PAGES * 4096)

struct madifx_channelfader {
	unsigned int in[HDSPM_MIXER_CHANNELS];
	unsigned int pb[HDSPM_MIXER_CHANNELS];
};

struct madifx_mixer {
	struct madifx_channelfader ch[HDSPM_MIXER_CHANNELS];
};

/* FIXME: maybe move to .c file */
struct madifx_newmixer {
	uint32_t listVol[MADIFX_LIST_LENGTH];
	uint32_t listCh[MADIFX_LIST_LENGTH];
	uint32_t output_gain[MADIFX_NUM_OUTPUT_GAINS];
};

struct madifx_mixer_ioctl {
	struct madifx_mixer *mixer;
};

/* use indirect access due to the limit of ioctl bit size */
#define SNDRV_HDSPM_IOCTL_GET_MIXER _IOR('H', 0x44, struct madifx_mixer_ioctl)

/* typedefs for compatibility to user-space */
typedef struct madifx_peak_rms madifx_peak_rms_t;
typedef struct madifx_config_info madifx_config_info_t;
typedef struct madifx_version madifx_version_t;
typedef struct madifx_channelfader snd_madifx_channelfader_t;
typedef struct madifx_mixer madifx_mixer_t;


#endif
