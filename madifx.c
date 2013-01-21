/*
 *   ALSA driver for RME Hammerfall DSP MADI FX audio interface(s)
 *
 *      Based on hdspm.c
 *      Copyright (c) 2003 Winfried Ritsch (IEM)
 *      code based on hdsp.c   Paul Davis
 *                             Marcus Andersson
 *                             Thomas Charbonnel
 *                             Florian Faber
 *                             Adrian Knoth
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
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/math64.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <sound/asoundef.h>
#include <sound/rawmidi.h>
#include <sound/hwdep.h>
#include <sound/initval.h>

#include "madifx.h"

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	  /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	  /* ID for this card */
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;/* Enable this card */

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for RME HDSPM interface.");

module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for RME HDSPM interface.");

module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable/disable specific HDSPM soundcards.");


MODULE_AUTHOR
(
	"Adrian Knoth <adi@drcomp.erfurt.thur.de>"
);
MODULE_DESCRIPTION("RME MADIFX");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{RME HDSPM-MADIFX}}");

/* --- Write registers. ---
  These are defined as byte-offsets from the iobase value.  */

#define MADIFX_CONTROL_REG	(0*4)
#define MADIFX_IRQ_ACK		(3*4)
#define MADIFX_FREQ_REG		(1*4)
#define MADIFX_SETTINGS_REG	(2*4)
#define MADIFX_START_LEVEL	(6*4)
#define MADIFX_midi_out0_data   (8*4)
#define MADIFX_midi_out1_data   (9*4)
#define MADIFX_midi_out2_data	(10*4)
#define MADIFX_midi_out3_data	(11*4)
#define MADIFX_ENABLE_OUTPUT	(64*4)
#define MADIFX_ENABLE_INPUT	(96*4)
#define MADIFX_MIXER_LIST_VOL	(16384*4)
#define MADIFX_MIXER_LIST_CH	(20480*4)
#define MADIFX_WR_OUTPUT_GAIN	((24576+256)*4)

#define MADIFX_SAMPLE_FRAMES_PER_BUFFER		8192

#define MADIFX_PAGE_ADDRESS_LIST   (8192*4)

/* page table size in entries, multiply by 4 to get byte offset */
#define MADIFX_MAX_PAGE_TABLE_SIZE	4096
#define MADIFX_LPTI_HMFX    (MADIFX_MAX_PAGE_TABLE_SIZE/2+25*32768*8/4096)
#define MADIFX_LPTI_MFXT    (MADIFX_MAX_PAGE_TABLE_SIZE/2+26*32768*8/4096)

#define HDSPM_MADI_mixerBase    32768	/* 32768-65535 for 2x64x64 Fader */

#define HDSPM_MATRIX_MIXER_SIZE  8192	/* = 2*64*64 * 4 Byte => 32kB */

/* --- Read registers. ---
   These are defined as byte-offsets from the iobase value */

#define MADIFX_RD_STATUS		(0*4)
#define MADIFX_RD_INP_STATUS	(1*4)
#define MADIFX_RD_INP_FREQ		(2*4)
#define MADIFX_RD_PLL_FREQ		(3*4)
#define MADIFX_RD_VERSION		(4*4) /* card_type(7..0) & "0000" & build(19..0) */
#define MADIFX_RD_FLASH			(5*4)
#define MADIFX_RD_BARCODE0		(6*4)
#define MADIFX_RD_BARCODE1		(7*4)
#define MADIFX_RD_DSP_DATA		(8*4)
#define MADIFX_RD_DSP_STATUS	(9*4)
#define MADIFX_midi_in0_data	(12*4)
#define MADIFX_midi_in1_data	(13*4)
#define MADIFX_midi_in2_data	(14*4)
#define MADIFX_midi_in3_data	(15*4)
#define MADIFX_midi_out0_status	(16*4)
#define MADIFX_midi_out1_status	(17*4)
#define MADIFX_midi_out2_status	(18*4)
#define MADIFX_midi_out3_status	(19*4)
#define MADIFX_midi_in0_status	(20*4)
#define MADIFX_midi_in1_status	(21*4)
#define MADIFX_midi_in2_status	(22*4)
#define MADIFX_midi_in3_status	(23*4)

/* input status */

#define MADIFX_madi1_lock		0x0001
#define MADIFX_madi2_lock		0x0002
#define MADIFX_madi3_lock		0x0004
#define MADIFX_aes_lock		0x0008
#define MADIFX_word_lock		0x0010
#define MADIFX_madi1_sync		0x0020
#define MADIFX_madi2_sync		0x0040
#define MADIFX_madi3_sync		0x0080
#define MADIFX_word_sync		0x0100
#define MADIFX_aes_sync		0x0200
#define MADIFX_madi1_rx_64ch	0x0400
#define MADIFX_madi2_rx_64ch	0x0800
#define MADIFX_madi3_rx_64ch	0x1000
#define MADIFX_SelSyncRef0		0x2000
#define MADIFX_SelSyncRef1		0x4000
#define MADIFX_SelSyncRef2		0x8000
#define MADIFX_MADIInput0		0x10000
#define MADIFX_MADIInput1		0x20000

/* control register bits */

#define MADIFX_START				0x00000001
#define MADIFX_freq0				0x00000002
#define MADIFX_freq1				0x00000004
#define MADIFX_freq2				0x00000008
#define MADIFX_freq3				0x00000010
#define MADIFX_BUF_SIZ_0			0x00000020
#define MADIFX_BUF_SIZ_1			0x00000040
#define MADIFX_BUF_SIZ_2			0x00000080
#define MADIFX_LAT_0				0x00000100
#define MADIFX_LAT_1				0x00000200
#define MADIFX_LAT_2				0x00000400
#define MADIFX_LAT_3				0x00000800
#define MADIFX_IE_AUDIO			0x00001000
#define MADIFX_IEN0				0x00002000
#define MADIFX_IEN1				0x00004000
#define MADIFX_IEN2				0x00008000
#define MADIFX_IEN3				0x00010000
#define MADIFX_float_format		0x00020000
#define MADIFX_CLR_TMS				0x00040000
#define MADIFX_Dolby				0x00080000

#define MADIFX_kFrequencyMask	(MADIFX_freq0 + MADIFX_freq1 + MADIFX_freq2 + MADIFX_freq3)
#define MADIFX_kBufferPositionMask	0xFFF0

enum {
	MADIFX_kFrequency32kHz		= 0,
	MADIFX_kFrequency44_1kHz	= MADIFX_freq0,
	MADIFX_kFrequency48kHz		= MADIFX_freq1,
	MADIFX_kFrequency64kHz		= MADIFX_freq2 + 0,
	MADIFX_kFrequency88_2kHz	= MADIFX_freq2 + MADIFX_freq0,
	MADIFX_kFrequency96kHz		= MADIFX_freq2 + MADIFX_freq1,
	MADIFX_kFrequency128kHz		= MADIFX_freq3 + MADIFX_freq2 + 0,
	MADIFX_kFrequency176_4kHz	= MADIFX_freq3 + MADIFX_freq2 + MADIFX_freq0,
	MADIFX_kFrequency192kHz		= MADIFX_freq3 + MADIFX_freq2 + MADIFX_freq1
};

/* settings register bits */

#define MADIFX_SyncRef0		0x00000001
#define MADIFX_SyncRef1		0x00000002
#define MADIFX_SyncRef2		0x00000004
#define MADIFX_PRO		0x00000008
#define MADIFX_DSP_EN		0x00000010
#define MADIFX_WCK_TERM		0x00000020
#define MADIFX_WCK48		0x00000040
#define MADIFX_madi1_tx_64ch	0x00000080
#define MADIFX_madi2_tx_64ch	0x00000100
#define MADIFX_madi3_tx_64ch	0x00000200
#define MADIFX_madi1_smux	0x00000400
#define MADIFX_madi2_smux	0x00000800
#define MADIFX_madi3_smux	0x00001000
#define MADIFX_redundancy_mode	0x00002000
#define MADIFX_mirror_madi_out	0x00004000

#define MADIFX_SyncRefMask	(MADIFX_SyncRef0 | MADIFX_SyncRef1 | MADIFX_SyncRef2)

/* input freq register bits */

#define MADIFX_madi1_freq0	0x00001
#define MADIFX_madi1_freq1	0x00002
#define MADIFX_madi1_freq2	0x00004
#define MADIFX_madi1_freq3	0x00008
#define MADIFX_madi2_freq0	0x00010
#define MADIFX_madi2_freq1	0x00020
#define MADIFX_madi2_freq2	0x00040
#define MADIFX_madi2_freq3	0x00080
#define MADIFX_madi3_freq0	0x00100
#define MADIFX_madi3_freq1	0x00200
#define MADIFX_madi3_freq2	0x00400
#define MADIFX_madi3_freq3	0x00800
#define MADIFX_aes_freq0	0x01000
#define MADIFX_aes_freq1	0x02000
#define MADIFX_aes_freq2	0x04000
#define MADIFX_aes_freq3	0x08000
#define MADIFX_word_freq0	0x10000
#define MADIFX_word_freq1	0x20000
#define MADIFX_word_freq2	0x40000
#define MADIFX_word_freq3	0x80000

/* Index to DMA level buffer in uint32_t units */
#define MADIFX_RD_RMS_IN	(0*1)
#define MADIFX_RD_PEAK_IN	(512*1)
#define MADIFX_RD_RMS_PLAY	(1024*1)
#define MADIFX_RD_PEAK_PLAY	(1536*1)
#define MADIFX_RD_RMS_OUT	(2048*1)
#define MADIFX_RD_PEAK_OUT	(2560*1)
#define MADIFX_RD_RMS_IN_PRE	(3072*1)
#define MADIFX_RD_PEAK_IN_PRE	(3584*1)
#define MADIFX_RD_RMS_OUT_PRE	(4096*1)
#define MADIFX_RD_PEAK_OUT_PRE	(4608*1)

/* MADIFX MIDI Interrupt enable */
#define MADIFX_IEN0				0x00002000
#define MADIFX_IEN1				0x00004000
#define MADIFX_IEN2				0x00008000
#define MADIFX_IEN3				0x00010000

/* status register, MIDI IRQ Pending */
#define MADIFX_mIRQ0		0x10000000
#define MADIFX_mIRQ1		0x20000000
#define MADIFX_mIRQ2		0x40000000
#define MADIFX_mIRQ3		0x80000000

/* --- bit helper defines */
#define MADIFX_LatencyMask  (MADIFX_LAT_0|MADIFX_LAT_1|MADIFX_LAT_2|MADIFX_LAT_3)

#define madifx_encode_latency(x)       (((x)<<8) & MADIFX_LatencyMask)
#define madifx_decode_latency(x)       ((((x) & MADIFX_LatencyMask)>>8))

/* speemode is enum 0,1,2 for ss/ds/qs, so (1<<speedmode) returns 1, 2, 4. */
#define madifx_speed_multiplier(x)	(1<<(x)->speedmode)


#define HDSPM_audioIRQPending    (1<<0)	/* IRQ is high and pending */



/* Mixer Values */
#define UNITY_GAIN          32768	/* = 65536/2 */
#define MINUS_INFINITY_GAIN 0

/* Number of channels for different Speed Modes */
#define MADIFX_SS_IN_CHANNELS       194
#define MADIFX_DS_IN_CHANNELS       98
#define MADIFX_QS_IN_CHANNELS       50

#define MADIFX_SS_OUT_CHANNELS       196
#define MADIFX_DS_OUT_CHANNELS       100
#define MADIFX_QS_OUT_CHANNELS       52


#define HDSPM_MADIFX_REV	213

/* speed factor modes */
#define HDSPM_SPEED_SINGLE 0
#define HDSPM_SPEED_DOUBLE 1
#define HDSPM_SPEED_QUAD   2

/* DMA buffers in byte; 8192 samples per channel, each 4 bytes wide */
#define NUM_INPUTS_S_MFXT	(64*3+4)
#define NUM_OUTPUTS_S_MFXT	(64*3+6)
#define INPUT_DMA_BUFFER_SIZE (NUM_INPUTS_S_MFXT*32768)
#define OUTPUT_DMA_BUFFER_SIZE (NUM_OUTPUTS_S_MFXT*32768)



/* names for speed modes */
static char *madifx_speed_names[] = { "single", "double", "quad" };

static char *texts_madifx_clock_source[] = {
	"Internal",
	"AES In",
	"Word Clock",
	"MADI 1 In",
	"MADI 2 In",
	"MADI 3 In"
};


static char *texts_freq[] = {
	"No Lock",
	"32 kHz",
	"44.1 kHz",
	"48 kHz",
	"64 kHz",
	"88.2 kHz",
	"96 kHz",
	"128 kHz",
	"176.4 kHz",
	"192 kHz"
};


struct madifx_midi {
	struct hdspm *hdspm;
	int id;
	struct snd_rawmidi *rmidi;
	struct snd_rawmidi_substream *input;
	struct snd_rawmidi_substream *output;
	char istimer;		/* timer in use */
	struct timer_list timer;
	spinlock_t lock;
	int pending;
	int dataIn;
	int statusIn;
	int dataOut;
	int statusOut;
	int ie;
	int irq;
};


struct hdspm {
	spinlock_t lock;
	/* only one playback and/or capture stream */
	struct snd_pcm_substream *capture_substream;
	struct snd_pcm_substream *playback_substream;

	char *card_name;	     /* for procinfo */
	unsigned short firmware_rev;

	uint8_t io_type;

	int monitor_outs;	/* set up monitoring outs init flag */

	u32 control_register;	/* cached value */
	u32 control2_register;	/* cached value */
	u32 settings_register;

	struct madifx_midi midi[4];
	struct tasklet_struct midi_tasklet;

	size_t period_bytes;
	unsigned char ss_in_channels;
	unsigned char ds_in_channels;
	unsigned char qs_in_channels;
	unsigned char ss_out_channels;
	unsigned char ds_out_channels;
	unsigned char qs_out_channels;

	unsigned char max_channels_in;
	unsigned char max_channels_out;

	char **port_names_in;
	char **port_names_out;

	char **port_names_in_ss, **port_names_in_ds, **port_names_in_qs;
	char **port_names_out_ss, **port_names_out_ds, **port_names_out_qs;

	unsigned char *playback_buffer;	/* suitably aligned address */
	unsigned char *capture_buffer;	/* suitably aligned address */
	u32 *level_buffer;	/* suitably aligned address */

	pid_t capture_pid;	/* process id which uses capture */
	pid_t playback_pid;	/* process id which uses capture */
	int running;		/* running status */

	int last_external_sample_rate;	/* samplerate mystic ... */
	int last_internal_sample_rate;
	int system_sample_rate;

	int dev;		/* Hardware vars... */
	int irq;
	unsigned long port;
	void __iomem *iobase;

	int irq_count;		/* for debug */
	int midiPorts;

	struct snd_card *card;	/* one card */
	struct snd_pcm *pcm;		/* has one pcm */
	struct snd_hwdep *hwdep;	/* and a hwdep for additional ioctl */
	struct pci_dev *pci;	/* and an pci info */

	/* Mixer vars */
	/* fast alsa mixer */
	struct snd_kcontrol *playback_mixer_ctls[HDSPM_MAX_CHANNELS];
	/* but input to much, so not used */
	struct snd_kcontrol *input_mixer_ctls[HDSPM_MAX_CHANNELS];
	/* full mixer accessible over mixer ioctl or hwdep-device */
	struct madifx_mixer *mixer;
	struct madifx_newmixer *newmixer;
	dma_addr_t *dmaPageTable;
	struct snd_dma_buffer dmaLevelBuffer;

	char **texts_clocksource;
	int texts_autosync_items;
	int texts_clocksource_items;

	cycles_t last_interrupt;

	unsigned int serial;

	int speedmode;

	struct madifx_level_buffer peak_rms;

};


static DEFINE_PCI_DEVICE_TABLE(snd_madifx_ids) = {
	{
	 .vendor = PCI_VENDOR_ID_XILINX,
	 .device = 0x3fc7,
	 .subvendor = PCI_ANY_ID,
	 .subdevice = PCI_ANY_ID,
	 .class = 0,
	 .class_mask = 0,
	 .driver_data = 0},
	{0,}
};

MODULE_DEVICE_TABLE(pci, snd_madifx_ids);

/* prototypes */
static int __devinit snd_madifx_create_alsa_devices(struct snd_card *card,
						   struct hdspm *hdspm);
static int __devinit snd_madifx_create_pcm(struct snd_card *card,
					  struct hdspm *hdspm);

static inline void snd_madifx_initialize_midi_flush(struct hdspm *hdspm);
static int madifx_update_simple_mixer_controls(struct hdspm *hdspm);
static int madifx_autosync_ref(struct hdspm *hdspm);
static int madifx_external_freq_index(struct hdspm *hdspm, enum madifx_syncsource port);
static int madifx_get_clock_select(struct hdspm *hdspm);
static int snd_madifx_set_defaults(struct hdspm *hdspm);
static int madifx_system_clock_mode(struct hdspm *hdspm);

static inline int HDSPM_bit2freq(int n)
{
	static const int bit2freq_tab[] = {
		0, 32000, 44100, 48000, 64000, 88200,
		96000, 128000, 176400, 192000 };
	if (n < 1 || n > 9)
		return 0;
	return bit2freq_tab[n];
}

/* Write/read to/from HDSPM with Adresses in Bytes
   not words but only 32Bit writes are allowed */

static inline void madifx_write(struct hdspm *hdspm, unsigned int reg,
			       unsigned int val)
{
	writel(val, hdspm->iobase + reg);
}

static inline unsigned int madifx_read(struct hdspm *hdspm, unsigned int reg)
{
	return readl(hdspm->iobase + reg);
}

/* for each output channel (chan) I have an Input (in) and Playback (pb) Fader
   mixer is write only on hardware so we have to cache him for read
   each fader is a u32, but uses only the first 16 bit */

static inline int madifx_read_in_gain(struct hdspm *hdspm, unsigned int chan,
				     unsigned int in)
{
	if (chan >= HDSPM_MIXER_CHANNELS || in >= HDSPM_MIXER_CHANNELS)
		return 0;

	return hdspm->mixer->ch[chan].in[in];
}

static inline int madifx_read_pb_gain(struct hdspm *hdspm, unsigned int chan,
				     unsigned int pb)
{
	if (chan >= HDSPM_MIXER_CHANNELS || pb >= HDSPM_MIXER_CHANNELS)
		return 0;
	return hdspm->mixer->ch[chan].pb[pb];
}

static int madifx_write_in_gain(struct hdspm *hdspm, unsigned int chan,
				      unsigned int in, unsigned short data)
{
	if (chan >= HDSPM_MIXER_CHANNELS || in >= HDSPM_MIXER_CHANNELS)
		return -1;

	madifx_write(hdspm,
		    HDSPM_MADI_mixerBase +
		    ((in + 128 * chan) * sizeof(u32)),
		    (hdspm->mixer->ch[chan].in[in] = data & 0xFFFF));
	return 0;
}

static int madifx_write_pb_gain(struct hdspm *hdspm, unsigned int chan,
				      unsigned int pb, unsigned short data)
{
	if (chan >= HDSPM_MIXER_CHANNELS || pb >= HDSPM_MIXER_CHANNELS)
		return -1;

	madifx_write(hdspm,
		    HDSPM_MADI_mixerBase +
		    ((64 + pb + 128 * chan) * sizeof(u32)),
		    (hdspm->mixer->ch[chan].pb[pb] = data & 0xFFFF));
	return 0;
}


/* enable DMA for specific channels, now available for DSP-MADI */
static inline void snd_madifx_enable_in(struct hdspm *hdspm, int i, int v)
{
	madifx_write(hdspm, MADIFX_ENABLE_INPUT + (4 * i), v);
}

static inline void snd_madifx_enable_out(struct hdspm *hdspm, int i, int v)
{
	madifx_write(hdspm, MADIFX_ENABLE_OUTPUT + (4 * i), v);
}

/* check if same process is writing and reading */
static int snd_madifx_use_is_exclusive(struct hdspm *hdspm)
{
	unsigned long flags;
	int ret = 1;

	spin_lock_irqsave(&hdspm->lock, flags);
	if ((hdspm->playback_pid != hdspm->capture_pid) &&
	    (hdspm->playback_pid >= 0) && (hdspm->capture_pid >= 0)) {
		ret = 0;
	}
	spin_unlock_irqrestore(&hdspm->lock, flags);
	return ret;
}


/* return latency in samples per period */
static int madifx_get_latency(struct hdspm *hdspm)
{
	int n;

	n = madifx_decode_latency(hdspm->control_register);

	return 1 << (n + 5);
}

/* Latency function */
static inline void madifx_compute_period_size(struct hdspm *hdspm)
{
	hdspm->period_bytes = 4 * madifx_get_latency(hdspm);
}


/* position of the hardware pointer in the buffer */
static snd_pcm_uframes_t madifx_hw_pointer(struct hdspm *hdspm)
{
	u32 position;

	position = madifx_read(hdspm, MADIFX_RD_STATUS);

	position &= MADIFX_kBufferPositionMask;
	position >>= 4;
	position *= 4;
#if 0
	position -= 4; /* safety offset */
#endif
	position &= (MADIFX_SAMPLE_FRAMES_PER_BUFFER-1);

	return position;
}

static inline void madifx_start_audio(struct hdspm *s)
{
	s->control_register |= (MADIFX_IE_AUDIO | MADIFX_START);
	madifx_write(s, MADIFX_CONTROL_REG, s->control_register);
}

static inline void madifx_stop_audio(struct hdspm *s)
{
	s->control_register &= ~(MADIFX_START | MADIFX_IE_AUDIO);
	madifx_write(s, MADIFX_CONTROL_REG, s->control_register);
}

static void madifx_silence_playback(struct hdspm *hdspm)
{
	void *buf = hdspm->playback_buffer;

	if (buf == NULL)
		return;

	memset(buf, 0, OUTPUT_DMA_BUFFER_SIZE);
}

static int madifx_set_interrupt_interval(struct hdspm *s, unsigned int frames)
{
	int n;
	snd_printk(KERN_INFO "MADIFX: latency request for %i frames\n", frames);

	spin_lock_irq(&s->lock);

	/* FIXME: We have four bits, but we don't know the mapping to frames,
	 * yet.
	 * LAT_3 is 2048
	 * LAT_2 is 128
	 * LAT_1 is 32
	 * LAT_0 is 16
	 *
	 * 2^(n+4), encode n to 4 bits
	 */
	n = 0;
	frames >>= 6;
	while (frames) {
		n++;
		frames >>= 1;
	}

	snd_printk(KERN_INFO "MADIFX: setting %i frames equals %i is %i\n", frames,
			n, madifx_decode_latency((madifx_encode_latency(n))));

	s->control_register &= ~MADIFX_LatencyMask;
	s->control_register |= madifx_encode_latency(n);

	snd_printk(KERN_INFO "MADIFX: setting LAT_0 to %i\n", s->control_register & MADIFX_LAT_0);
	snd_printk(KERN_INFO "MADIFX: setting LAT_1 to %i\n", s->control_register & MADIFX_LAT_1);
	snd_printk(KERN_INFO "MADIFX: setting LAT_2 to %i\n", s->control_register & MADIFX_LAT_2);
	snd_printk(KERN_INFO "MADIFX: setting LAT_3 to %i\n", s->control_register & MADIFX_LAT_3);

	madifx_write(s, MADIFX_CONTROL_REG, s->control_register);

	madifx_compute_period_size(s);

	snd_printk(KERN_INFO "MADIFX: final latency %i\n", madifx_get_latency(s));

	spin_unlock_irq(&s->lock);

	return 0;
}

static u64 madifx_calc_dds_value(struct hdspm *hdspm, u64 period)
{
	u64 freq_const;

	if (period == 0)
		return 0;

	switch (hdspm->io_type) {
	case MADIFX:
		freq_const = 131072000000000ULL;
		break;
	default:
		snd_BUG();
		return 0;
	}

	return div_u64(freq_const, period);
}


static void madifx_set_dds_value(struct hdspm *hdspm, int rate)
{
	u64 n;

	if (rate >= 112000)
		rate /= 4;
	else if (rate >= 56000)
		rate /= 2;

	switch (hdspm->io_type) {
	case MADIFX:
		n = 131072000000000ULL;  /* 125 MHz */
		break;
	default:
		snd_BUG();
		return;
	}

	n = div_u64(n, rate);
	/* n should be less than 2^32 for being written to FREQ register */
	snd_BUG_ON(n >> 32);
	madifx_write(hdspm, MADIFX_FREQ_REG, (u32)n);
}

static int madifx_get_external_rate(struct hdspm *hdspm)
{
	int current_clock = madifx_get_clock_select(hdspm);

	switch (current_clock) {
	case 0:
		/* Master. Should not happen */
		break;
	case 1:
	case 2:
		/* 1 == AES, 2 == WC; map to enum madifx_syncsource,
		 * so that 3 == AES, 4 == WC */
		current_clock += 2;
		break;
	case 3:
	case 4:
	case 5:
		/* MADI1 == 3, MADI2 == 4, MADI3 == 5 map to
		 * MADI1 == 0, MADI2 = 1, MADI3 == 2
		 */
		current_clock -= 3;
		break;
	default:
		snd_printk(KERN_ERR "MADIFX: Unknown clock source\n");
		return 0;
	}



	return HDSPM_bit2freq(madifx_external_freq_index(hdspm, current_clock));
}

/* dummy set rate lets see what happens */
static int madifx_set_rate(struct hdspm *hdspm, int rate, int called_internally)
{
	int current_rate;
	int rate_bits;
	int not_set = 0;
	int current_speed, target_speed;

	/* ASSUMPTION: hdspm->lock is either set, or there is no need for
	   it (e.g. during module initialization).
	 */

	if (1 == madifx_system_clock_mode(hdspm)) {

		/* SLAVE --- */
		if (called_internally) {

			/* request from ctl or card initialization
			   just make a warning an remember setting
			   for future master mode switching */

			snd_printk(KERN_WARNING
				"MADIFX: Warning: device is not running as a clock master.\n");
			not_set = 1;
		} else {
			int external_freq = madifx_get_external_rate(hdspm);



			if (rate != external_freq) {
				snd_printk(KERN_WARNING
				    "MADIFX: Warning: No AutoSync source for requested rate\n");
				not_set = 1;
			}
		}
	}

	current_rate = hdspm->system_sample_rate;

	/* Changing between Singe, Double and Quad speed is not
	   allowed if any substreams are open. This is because such a change
	   causes a shift in the location of the DMA buffers and a reduction
	   in the number of available buffers.

	   Note that a similar but essentially insoluble problem exists for
	   externally-driven rate changes. All we can do is to flag rate
	   changes in the read/write routines.
	 */

	if (current_rate <= 56000)
		current_speed = HDSPM_SPEED_SINGLE;
	else if (current_rate <= 96000)
		current_speed = HDSPM_SPEED_DOUBLE;
	else
		current_speed = HDSPM_SPEED_QUAD;

	if (rate <= 48000)
		target_speed = HDSPM_SPEED_SINGLE;
	else if (rate <= 112000)
		target_speed = HDSPM_SPEED_DOUBLE;
	else
		target_speed = HDSPM_SPEED_QUAD;

	switch (rate) {
	case 32000:
		rate_bits = MADIFX_kFrequency32kHz;
		break;
	case 44100:
		rate_bits = MADIFX_kFrequency44_1kHz;
		break;
	case 48000:
		rate_bits = MADIFX_kFrequency48kHz;
		break;
	case 64000:
		rate_bits = MADIFX_kFrequency64kHz;
		break;
	case 88200:
		rate_bits = MADIFX_kFrequency88_2kHz;
		break;
	case 96000:
		rate_bits = MADIFX_kFrequency96kHz;
		break;
	case 128000:
		rate_bits = MADIFX_kFrequency128kHz;
		break;
	case 176400:
		rate_bits = MADIFX_kFrequency176_4kHz;
		break;
	case 192000:
		rate_bits = MADIFX_kFrequency192kHz;
		break;
	default:
		return -EINVAL;
	}

	if (current_speed != target_speed
	    && (hdspm->capture_pid >= 0 || hdspm->playback_pid >= 0)) {
		snd_printk
		    (KERN_ERR "HDSPM: "
		     "cannot change from %s speed to %s speed mode "
		     "(capture PID = %d, playback PID = %d)\n",
		     madifx_speed_names[current_speed],
		     madifx_speed_names[target_speed],
		     hdspm->capture_pid, hdspm->playback_pid);
		return -EBUSY;
	}

	madifx_set_dds_value(hdspm, rate);

	hdspm->control_register &= ~MADIFX_kFrequencyMask;
	hdspm->control_register |= rate_bits;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);



	hdspm->system_sample_rate = rate;

	if (rate <= 56000) {
		hdspm->max_channels_in = hdspm->ss_in_channels;
		hdspm->max_channels_out = hdspm->ss_out_channels;
		hdspm->port_names_in = hdspm->port_names_in_ss;
		hdspm->port_names_out = hdspm->port_names_out_ss;
		hdspm->speedmode = ss;
	} else if (rate <= 112000) {
		hdspm->max_channels_in = hdspm->ds_in_channels;
		hdspm->max_channels_out = hdspm->ds_out_channels;
		hdspm->port_names_in = hdspm->port_names_in_ds;
		hdspm->port_names_out = hdspm->port_names_out_ds;
		hdspm->speedmode = ds;
	} else {
		hdspm->max_channels_in = hdspm->qs_in_channels;
		hdspm->max_channels_out = hdspm->qs_out_channels;
		hdspm->port_names_in = hdspm->port_names_in_qs;
		hdspm->port_names_out = hdspm->port_names_out_qs;
		hdspm->speedmode = qs;
	}

	if (not_set != 0)
		return -1;

	return 0;
}

/* mainly for init to 0 on load */
static void all_in_all_mixer(struct hdspm *hdspm, int sgain)
{
	int i, j;
	unsigned int gain;

	/* FIXME: MADI FX unsupported, yet. */
	if (MADIFX == hdspm->io_type)
		return;

	if (sgain > UNITY_GAIN)
		gain = UNITY_GAIN;
	else if (sgain < 0)
		gain = 0;
	else
		gain = sgain;

	for (i = 0; i < HDSPM_MIXER_CHANNELS; i++)
		for (j = 0; j < HDSPM_MIXER_CHANNELS; j++) {
			madifx_write_in_gain(hdspm, i, j, gain);
			madifx_write_pb_gain(hdspm, i, j, gain);
		}
}

/*----------------------------------------------------------------------------
   MIDI
  ----------------------------------------------------------------------------*/

static inline unsigned char snd_madifx_midi_read_byte(struct hdspm *hdspm,
						      int id)
{
	/* the hardware already does the relevant bit-mask with 0xff */
	return madifx_read(hdspm, hdspm->midi[id].dataIn);
}

static inline void snd_madifx_midi_write_byte(struct hdspm *hdspm, int id,
					      int val)
{
	/* the hardware already does the relevant bit-mask with 0xff */
	return madifx_write(hdspm, hdspm->midi[id].dataOut, val);
}

static inline int snd_madifx_midi_input_available(struct hdspm *hdspm, int id)
{
	return madifx_read(hdspm, hdspm->midi[id].statusIn) & 0xFF;
}

static inline int snd_madifx_midi_output_possible(struct hdspm *hdspm, int id)
{
	int fifo_bytes_used;

	fifo_bytes_used = madifx_read(hdspm, hdspm->midi[id].statusOut) & 0xFF;

	if (fifo_bytes_used < 128)
		return  128 - fifo_bytes_used;
	else
		return 0;
}

static void snd_madifx_flush_midi_input(struct hdspm *hdspm, int id)
{
	while (snd_madifx_midi_input_available(hdspm, id))
		snd_madifx_midi_read_byte(hdspm, id);
}

static int snd_madifx_midi_output_write(struct madifx_midi *hmidi)
{
	unsigned long flags;
	int n_pending;
	int to_write;
	int i;
	unsigned char buf[128];

	/* Output is not interrupt driven */

	spin_lock_irqsave(&hmidi->lock, flags);
	if (hmidi->output &&
	    !snd_rawmidi_transmit_empty(hmidi->output)) {
		n_pending = snd_madifx_midi_output_possible(hmidi->hdspm,
							    hmidi->id);
		if (n_pending > 0) {
			if (n_pending > (int)sizeof(buf))
				n_pending = sizeof(buf);

			to_write = snd_rawmidi_transmit(hmidi->output, buf,
							 n_pending);
			if (to_write > 0) {
				for (i = 0; i < to_write; ++i)
					snd_madifx_midi_write_byte(hmidi->hdspm,
								   hmidi->id,
								   buf[i]);
			}
		}
	}
	spin_unlock_irqrestore(&hmidi->lock, flags);
	return 0;
}

static int snd_madifx_midi_input_read(struct madifx_midi *hmidi)
{
	unsigned char buf[128]; /* this buffer is designed to match the MIDI
				 * input FIFO size
				 */
	unsigned long flags;
	int n_pending;
	int i;

	spin_lock_irqsave(&hmidi->lock, flags);
	n_pending = snd_madifx_midi_input_available(hmidi->hdspm, hmidi->id);
	if (n_pending > 0) {
		if (hmidi->input) {
			if (n_pending > (int)sizeof(buf))
				n_pending = sizeof(buf);
			for (i = 0; i < n_pending; ++i)
				buf[i] = snd_madifx_midi_read_byte(hmidi->hdspm,
								   hmidi->id);
			if (n_pending)
				snd_rawmidi_receive(hmidi->input, buf,
						     n_pending);
		} else {
			/* flush the MIDI input FIFO */
			while (n_pending--)
				snd_madifx_midi_read_byte(hmidi->hdspm,
							  hmidi->id);
		}
	}
	hmidi->pending = 0;
	spin_unlock_irqrestore(&hmidi->lock, flags);

	spin_lock_irqsave(&hmidi->hdspm->lock, flags);
	hmidi->hdspm->control_register |= hmidi->ie;
	madifx_write(hmidi->hdspm, MADIFX_CONTROL_REG,
		    hmidi->hdspm->control_register);
	spin_unlock_irqrestore(&hmidi->hdspm->lock, flags);

	return snd_madifx_midi_output_write(hmidi);
}

static void
snd_madifx_midi_input_trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct hdspm *hdspm;
	struct madifx_midi *hmidi;
	unsigned long flags;

	hmidi = substream->rmidi->private_data;
	hdspm = hmidi->hdspm;

	spin_lock_irqsave(&hdspm->lock, flags);
	if (up) {
		if (!(hdspm->control_register & hmidi->ie)) {
			snd_madifx_flush_midi_input(hdspm, hmidi->id);
			hdspm->control_register |= hmidi->ie;
		}
	} else {
		hdspm->control_register &= ~hmidi->ie;
	}

	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);
	spin_unlock_irqrestore(&hdspm->lock, flags);
}

static void snd_madifx_midi_output_timer(unsigned long data)
{
	struct madifx_midi *hmidi = (struct madifx_midi *) data;
	unsigned long flags;

	snd_madifx_midi_output_write(hmidi);
	spin_lock_irqsave(&hmidi->lock, flags);

	/* this does not bump hmidi->istimer, because the
	   kernel automatically removed the timer when it
	   expired, and we are now adding it back, thus
	   leaving istimer wherever it was set before.
	*/

	if (hmidi->istimer) {
		hmidi->timer.expires = 1 + jiffies;
		add_timer(&hmidi->timer);
	}

	spin_unlock_irqrestore(&hmidi->lock, flags);
}

static void
snd_madifx_midi_output_trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct madifx_midi *hmidi;
	unsigned long flags;

	hmidi = substream->rmidi->private_data;
	spin_lock_irqsave(&hmidi->lock, flags);
	if (up) {
		if (!hmidi->istimer) {
			init_timer(&hmidi->timer);
			hmidi->timer.function = snd_madifx_midi_output_timer;
			hmidi->timer.data = (unsigned long) hmidi;
			hmidi->timer.expires = 1 + jiffies;
			add_timer(&hmidi->timer);
			hmidi->istimer++;
		}
	} else {
		if (hmidi->istimer && --hmidi->istimer <= 0)
			del_timer(&hmidi->timer);
	}
	spin_unlock_irqrestore(&hmidi->lock, flags);
	if (up)
		snd_madifx_midi_output_write(hmidi);
}

static int snd_madifx_midi_input_open(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	hmidi = substream->rmidi->private_data;
	spin_lock_irq(&hmidi->lock);
	snd_madifx_flush_midi_input(hmidi->hdspm, hmidi->id);
	hmidi->input = substream;
	spin_unlock_irq(&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_output_open(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	hmidi = substream->rmidi->private_data;
	spin_lock_irq(&hmidi->lock);
	hmidi->output = substream;
	spin_unlock_irq(&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_input_close(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	snd_madifx_midi_input_trigger(substream, 0);

	hmidi = substream->rmidi->private_data;
	spin_lock_irq(&hmidi->lock);
	hmidi->input = NULL;
	spin_unlock_irq(&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_output_close(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	snd_madifx_midi_output_trigger(substream, 0);

	hmidi = substream->rmidi->private_data;
	spin_lock_irq(&hmidi->lock);
	hmidi->output = NULL;
	spin_unlock_irq(&hmidi->lock);

	return 0;
}

static struct snd_rawmidi_ops snd_madifx_midi_output = {
	.open =		snd_madifx_midi_output_open,
	.close =	snd_madifx_midi_output_close,
	.trigger =	snd_madifx_midi_output_trigger,
};

static struct snd_rawmidi_ops snd_madifx_midi_input = {
	.open =		snd_madifx_midi_input_open,
	.close =	snd_madifx_midi_input_close,
	.trigger =	snd_madifx_midi_input_trigger,
};

static int __devinit snd_madifx_create_midi(struct snd_card *card,
					    struct hdspm *hdspm, int id)
{
	int err;
	char buf[32];

	hdspm->midi[id].id = id;
	hdspm->midi[id].hdspm = hdspm;
	spin_lock_init(&hdspm->midi[id].lock);

	switch (id) {
	case 0:
		hdspm->midi[0].dataIn = MADIFX_midi_in0_data;
		hdspm->midi[0].statusIn = MADIFX_midi_in0_status;
		hdspm->midi[0].dataOut = MADIFX_midi_out0_data;
		hdspm->midi[0].statusOut = MADIFX_midi_out0_status;
		hdspm->midi[0].ie = MADIFX_IEN0;
		hdspm->midi[0].irq = MADIFX_mIRQ0;
		break;

	case 1:
		hdspm->midi[1].dataIn = MADIFX_midi_in1_data;
		hdspm->midi[1].statusIn = MADIFX_midi_in1_status;
		hdspm->midi[1].dataOut = MADIFX_midi_out1_data;
		hdspm->midi[1].statusOut = MADIFX_midi_out1_status;
		hdspm->midi[1].ie = MADIFX_IEN1;
		hdspm->midi[1].irq = MADIFX_mIRQ1;
		break;

	case 2:
		hdspm->midi[2].dataIn = MADIFX_midi_in2_data;
		hdspm->midi[2].statusIn = MADIFX_midi_in2_status;
		hdspm->midi[2].dataOut = MADIFX_midi_out2_data;
		hdspm->midi[2].statusOut = MADIFX_midi_out2_status;
		hdspm->midi[2].ie = MADIFX_IEN2;
		hdspm->midi[2].irq = MADIFX_mIRQ2;
		break;

	case 3:
		hdspm->midi[3].dataIn = MADIFX_midi_in3_data;
		hdspm->midi[3].statusIn = MADIFX_midi_in3_status;
		hdspm->midi[3].dataOut = MADIFX_midi_out3_data;
		hdspm->midi[3].statusOut = MADIFX_midi_out3_status;
		hdspm->midi[3].ie = MADIFX_IEN3;
		hdspm->midi[3].irq = MADIFX_mIRQ3;
		break;

	default:
		snd_printk(KERN_ERR "MADIFX: Unknown MIDI port %i\n", id);
		return -EINVAL;

	}

	sprintf(buf, "%s MIDIoverMADI %d", card->shortname, id+1);

	err = snd_rawmidi_new(card, buf, id, 1, 1,
			&hdspm->midi[id].rmidi);
	if (err < 0)
		return err;

	sprintf(hdspm->midi[id].rmidi->name, "%s MIDI %d",
			card->id, id+1);
	hdspm->midi[id].rmidi->private_data = &hdspm->midi[id];

	snd_rawmidi_set_ops(hdspm->midi[id].rmidi,
			SNDRV_RAWMIDI_STREAM_OUTPUT,
			&snd_madifx_midi_output);
	snd_rawmidi_set_ops(hdspm->midi[id].rmidi,
			SNDRV_RAWMIDI_STREAM_INPUT,
			&snd_madifx_midi_input);

	hdspm->midi[id].rmidi->info_flags |=
		SNDRV_RAWMIDI_INFO_OUTPUT |
		SNDRV_RAWMIDI_INFO_INPUT |
		SNDRV_RAWMIDI_INFO_DUPLEX;

	return 0;
}


static void madifx_midi_tasklet(unsigned long arg)
{
	struct hdspm *hdspm = (struct hdspm *)arg;
	int i = 0;

	while (i < hdspm->midiPorts) {
		if (hdspm->midi[i].pending)
			snd_madifx_midi_input_read(&hdspm->midi[i]);

		i++;
	}
}


/*-----------------------------------------------------------------------------
  Status Interface
  ----------------------------------------------------------------------------*/

/* get the system sample rate which is set */


/**
 * Calculate the real sample rate from the
 * current DDS value.
 **/
static int madifx_get_system_sample_rate(struct hdspm *hdspm)
{
	unsigned int period, rate;

	period = madifx_read(hdspm, MADIFX_RD_PLL_FREQ);
	rate = madifx_calc_dds_value(hdspm, period) *
		madifx_speed_multiplier(hdspm);

	return rate;
}


#define HDSPM_SYSTEM_SAMPLE_RATE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_system_sample_rate, \
	.put = snd_madifx_put_system_sample_rate, \
	.get = snd_madifx_get_system_sample_rate \
}

static int snd_madifx_info_system_sample_rate(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 27000;
	uinfo->value.integer.max = 207000;
	uinfo->value.integer.step = 1;
	return 0;
}


static int snd_madifx_get_system_sample_rate(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *
					    ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = madifx_get_system_sample_rate(hdspm);
	return 0;
}

static int snd_madifx_put_system_sample_rate(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *
					    ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	madifx_set_dds_value(hdspm, ucontrol->value.enumerated.item[0]);
	return 0;
}



static int madifx_external_freq_index(struct hdspm *hdspm, enum madifx_syncsource port)
{
	int i = 0;
	int freq0_bit;
	int inp_freq;
	int lock_bit;
	int inp_status;

	inp_status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);
	inp_freq = madifx_read(hdspm, MADIFX_RD_INP_FREQ);

	switch (port) {
	case syncsource_madi1:
		lock_bit = MADIFX_madi1_lock;
		freq0_bit = MADIFX_madi1_freq0;
		break;
	case syncsource_madi2:
		lock_bit = MADIFX_madi2_lock;
		freq0_bit = MADIFX_madi2_freq0;
		break;
	case syncsource_madi3:
		lock_bit = MADIFX_madi3_lock;
		freq0_bit = MADIFX_madi3_freq0;
		break;
	case syncsource_aes:
		lock_bit = MADIFX_aes_lock;
		freq0_bit = MADIFX_aes_freq0;
		break;
	case syncsource_wc:
		lock_bit = MADIFX_word_lock;
		freq0_bit = MADIFX_word_freq0;
		break;
	default:
		snd_printk(KERN_ERR "MADIFX: Unknown external port ID %i\n", port);
		return 0;
	}

	if (!(inp_status & lock_bit)) {
		i = 0;
	} else {
		int freq_bits = inp_freq & (freq0_bit*15);
		if      (freq_bits == freq0_bit * 1)
			i = 1;
		else if (freq_bits == freq0_bit * 2)
			i = 2;
		else if (freq_bits == freq0_bit * 3)
			i = 3;
		else if (freq_bits == freq0_bit * 4)
			i = 4;
		else if (freq_bits == freq0_bit * 5)
			i = 5;
		else if (freq_bits == freq0_bit * 6)
			i = 6;
		else if (freq_bits == freq0_bit * 7)
			i = 7;
		else if (freq_bits == freq0_bit * 8)
			i = 8;
		else if (freq_bits == freq0_bit * 9)
			i = 9;
		else
			i = 0;
	}
	return i;
}


#define HDSPM_AUTOSYNC_SAMPLE_RATE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.private_value = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READ, \
	.info = snd_madifx_info_autosync_sample_rate, \
	.get = snd_madifx_get_autosync_sample_rate \
}


static int snd_madifx_info_autosync_sample_rate(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 10;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item = uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
			texts_freq[uinfo->value.enumerated.item]);
	return 0;
}


static int snd_madifx_get_autosync_sample_rate(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value *
					      ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	switch (hdspm->io_type) {
	case MADIFX:
		ucontrol->value.enumerated.item[0] =
			madifx_external_freq_index(hdspm, kcontrol->private_value);
		break;

	default:
		break;
	}

	return 0;
}

#define MADIFX_MADI_CHANNELCOUNT(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.private_value = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READ, \
	.info = snd_madifx_info_channelcount, \
	.get = snd_madifx_get_channelcount \
}


static int snd_madifx_info_channelcount(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "64", "56", "32", "28", "16", "14", "No lock" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 7;
	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int madifx_get_madichannelcount(struct hdspm *hdspm, int idx)
{
	int rate_index;
	int i = 0;
	int inp_status;
	int rx_64ch_bit = (MADIFX_madi1_rx_64ch << idx);

	inp_status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);


	/* Check for speed. If rate_index is zero, there's no lock */
	rate_index = madifx_external_freq_index(hdspm, idx);
	if (0 == rate_index)
		i = 6;
	else {
		if (inp_status & rx_64ch_bit)
			i = 0;
		else
			i = 1;

		/* I know this is ugly */
		if (rate_index > 3)
			i += 2; /* Double speed */
		if (rate_index > 6)
			i += 2; /* Quad speed */
	}

	return i;

}


static int snd_madifx_get_channelcount(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_value *
					      ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int idx = kcontrol->private_value;


	ucontrol->value.enumerated.item[0] =
		madifx_get_madichannelcount(hdspm, idx);

	return 0;
}



/**
 * Returns the system clock mode for the given card.
 * @returns 0 - master, 1 - slave
 **/
static int madifx_system_clock_mode(struct hdspm *hdspm)
{
	u32 status;

	status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);
	if ((status & (MADIFX_SelSyncRef0 * 7)) == (MADIFX_SelSyncRef0 * 7))
		return 0;

	return 1;
}


#define HDSPM_INTERNAL_CLOCK(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_clock_source, \
	.get = snd_madifx_get_clock_source, \
	.put = snd_madifx_put_clock_source \
}


static int madifx_clock_source(struct hdspm *hdspm)
{
	switch (hdspm->system_sample_rate) {
	case 32000: return 0;
	case 44100: return 1;
	case 48000: return 2;
	case 64000: return 3;
	case 88200: return 4;
	case 96000: return 5;
	case 128000: return 6;
	case 176400: return 7;
	case 192000: return 8;
	}

	return -1;
}

static int madifx_set_clock_source(struct hdspm *hdspm, int mode)
{
	int rate;
	switch (mode) {
	case 0:
		rate = 32000; break;
	case 1:
		rate = 44100; break;
	case 2:
		rate = 48000; break;
	case 3:
		rate = 64000; break;
	case 4:
		rate = 88200; break;
	case 5:
		rate = 96000; break;
	case 6:
		rate = 128000; break;
	case 7:
		rate = 176400; break;
	case 8:
		rate = 192000; break;
	default:
		rate = 48000;
	}
	madifx_set_rate(hdspm, rate, 1);
	return 0;
}

static int snd_madifx_info_clock_source(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 9;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
	       texts_freq[uinfo->value.enumerated.item+1]);

	return 0;
}

static int snd_madifx_get_clock_source(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = madifx_clock_source(hdspm);
	return 0;
}

static int snd_madifx_put_clock_source(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.enumerated.item[0];
	if (val < 0)
		val = 0;
	if (val > 9)
		val = 9;
	spin_lock_irq(&hdspm->lock);
	if (val != madifx_clock_source(hdspm))
		change = (madifx_set_clock_source(hdspm, val) == 0) ? 1 : 0;
	else
		change = 0;
	spin_unlock_irq(&hdspm->lock);
	return change;
}


static int madifx_get_clock_select(struct hdspm *hdspm)
{
	switch (hdspm->io_type) {
	case MADIFX:
		{
			int i;
			u32 status;

			status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);

			switch (status & (MADIFX_SelSyncRef0 * 7)) {
			case MADIFX_SelSyncRef0 * 0:
				i = 0; break;
			case MADIFX_SelSyncRef0 * 1:
				i = 1; break;
			case MADIFX_SelSyncRef0 * 2:
				i = 2; break;
			case MADIFX_SelSyncRef0 * 3:
				i = 3; break;
			case MADIFX_SelSyncRef0 * 4:
				i = 4; break;
			case MADIFX_SelSyncRef0 * 5:
				i = 5; break;
			default:
						     /* We are master */
						     i = 0;
						     break;
			}
			return i;
		}
		break;
	default:
		break;
	}

	return -1;
}

static int madifx_set_clock_select(struct hdspm *hdspm, int val)
{
	hdspm->settings_register &= ~MADIFX_SyncRefMask;
	hdspm->settings_register |= MADIFX_SyncRef0 * val;
	madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);

	if (val > 0) {
		/* switched to slave mode */
		hdspm->system_sample_rate = madifx_get_external_rate(hdspm);
	}

	return 0;
}



#define MADIFX_CLOCK_SELECT(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_clock_select, \
	.get = snd_madifx_get_clock_select, \
	.put = snd_madifx_put_clock_select \
}


static int snd_madifx_info_clock_select(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = hdspm->texts_clocksource_items;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			hdspm->texts_clocksource[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_clock_select(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int psf = madifx_get_clock_select(hdspm);

	if (psf >= 0) {
		ucontrol->value.enumerated.item[0] = psf;
		return 0;
	}

	return -1;
}

static int snd_madifx_put_clock_select(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int val, change = 0;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;

	val = ucontrol->value.enumerated.item[0];

	if (val < 0)
		val = 0;
	else if (val >= hdspm->texts_clocksource_items)
		val = hdspm->texts_clocksource_items-1;

	spin_lock_irq(&hdspm->lock);
	if (val != madifx_get_clock_select(hdspm))
		change = (0 == madifx_set_clock_select(hdspm, val)) ? 1 : 0;

	spin_unlock_irq(&hdspm->lock);
	return change;
}



#define MADIFX_TOGGLE_SETTING(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.private_value = xindex, \
	.info = snd_madifx_info_toggle_setting, \
	.get = snd_madifx_get_toggle_setting, \
	.put = snd_madifx_put_toggle_setting \
}

static int madifx_read_toggle_setting(struct hdspm *hdspm, u32 reg)
{
	return (hdspm->settings_register & (reg)) ? 1 : 0;
}

static int madifx_set_toggle_setting(struct hdspm *hdspm, u32 reg, int out)
{
	if (out)
		hdspm->settings_register |= (reg);
	else
		hdspm->settings_register &= ~(reg);
	madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);

	return 0;
}

#define snd_madifx_info_toggle_setting		snd_ctl_boolean_mono_info

static int snd_madifx_get_toggle_setting(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] = madifx_read_toggle_setting(hdspm,
			kcontrol->private_value);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_toggle_setting(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	u32 reg = kcontrol->private_value;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_read_toggle_setting(hdspm, reg);
	madifx_set_toggle_setting(hdspm, reg, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}



#define HDSPM_MIXER(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_HWDEP, \
	.name = xname, \
	.index = xindex, \
	.device = 0, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE | \
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_mixer, \
	.get = snd_madifx_get_mixer, \
	.put = snd_madifx_put_mixer \
}

static int snd_madifx_info_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;
	uinfo->value.integer.step = 1;
	return 0;
}

static int snd_madifx_get_mixer(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int source;
	int destination;

	source = ucontrol->value.integer.value[0];
	if (source < 0)
		source = 0;
	else if (source >= 2 * HDSPM_MAX_CHANNELS)
		source = 2 * HDSPM_MAX_CHANNELS - 1;

	destination = ucontrol->value.integer.value[1];
	if (destination < 0)
		destination = 0;
	else if (destination >= HDSPM_MAX_CHANNELS)
		destination = HDSPM_MAX_CHANNELS - 1;

	spin_lock_irq(&hdspm->lock);
	if (source >= HDSPM_MAX_CHANNELS)
		ucontrol->value.integer.value[2] =
		    madifx_read_pb_gain(hdspm, destination,
				       source - HDSPM_MAX_CHANNELS);
	else
		ucontrol->value.integer.value[2] =
		    madifx_read_in_gain(hdspm, destination, source);

	spin_unlock_irq(&hdspm->lock);

	return 0;
}

static int snd_madifx_put_mixer(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	int source;
	int destination;
	int gain;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;

	source = ucontrol->value.integer.value[0];
	destination = ucontrol->value.integer.value[1];

	if (source < 0 || source >= 2 * HDSPM_MAX_CHANNELS)
		return -1;
	if (destination < 0 || destination >= HDSPM_MAX_CHANNELS)
		return -1;

	gain = ucontrol->value.integer.value[2];

	spin_lock_irq(&hdspm->lock);

	if (source >= HDSPM_MAX_CHANNELS)
		change = gain != madifx_read_pb_gain(hdspm, destination,
						    source -
						    HDSPM_MAX_CHANNELS);
	else
		change = gain != madifx_read_in_gain(hdspm, destination,
						    source);

	if (change) {
		if (source >= HDSPM_MAX_CHANNELS)
			madifx_write_pb_gain(hdspm, destination,
					    source - HDSPM_MAX_CHANNELS,
					    gain);
		else
			madifx_write_in_gain(hdspm, destination, source,
					    gain);
	}
	spin_unlock_irq(&hdspm->lock);

	return change;
}

/* The simple mixer control(s) provide gain control for the
   basic 1:1 mappings of playback streams to output
   streams.
*/

#define HDSPM_PLAYBACK_MIXER \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE | \
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_playback_mixer, \
	.get = snd_madifx_get_playback_mixer, \
	.put = snd_madifx_put_playback_mixer \
}

static int snd_madifx_info_playback_mixer(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 64;
	uinfo->value.integer.step = 1;
	return 0;
}

static int snd_madifx_get_playback_mixer(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int channel;

	channel = ucontrol->id.index - 1;

	if (snd_BUG_ON(channel < 0 || channel >= HDSPM_MAX_CHANNELS))
		return -EINVAL;

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] =
	  (madifx_read_pb_gain(hdspm, channel, channel)*64)/UNITY_GAIN;
	spin_unlock_irq(&hdspm->lock);

	return 0;
}

static int snd_madifx_put_playback_mixer(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	int channel;
	int gain;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;

	channel = ucontrol->id.index - 1;

	if (snd_BUG_ON(channel < 0 || channel >= HDSPM_MAX_CHANNELS))
		return -EINVAL;

	gain = ucontrol->value.integer.value[0]*UNITY_GAIN/64;

	spin_lock_irq(&hdspm->lock);
	change =
	    gain != madifx_read_pb_gain(hdspm, channel,
				       channel);
	if (change)
		madifx_write_pb_gain(hdspm, channel, channel,
				    gain);
	spin_unlock_irq(&hdspm->lock);
	return change;
}

#define HDSPM_SYNC_CHECK(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.private_value = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_sync_check, \
	.get = snd_madifx_get_sync_check \
}


static int snd_madifx_info_sync_check(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "No Lock", "Lock", "Sync", "N/A" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 4;
	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);
	return 0;
}


static int madifx_sync_check(struct hdspm *hdspm, int idx)
{
	int status, lock, sync;

	status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);

	lock = (status & (MADIFX_madi1_lock << idx)) ? 1 : 0;
	sync = (status & (MADIFX_madi1_sync << idx)) ? 1 : 0;

	if (lock && sync)
		return 2;
	else if (lock)
		return 1;
	return 0;
}




static int snd_madifx_get_sync_check(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int val = -1;

	switch (hdspm->io_type) {
	case MADIFX:
		val = madifx_sync_check(hdspm, kcontrol->private_value);
		break;
	}

	if (-1 == val)
		val = 3;

	ucontrol->value.enumerated.item[0] = val;
	return 0;
}



static struct snd_kcontrol_new snd_madifx_controls_madi[] = {
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYNC_CHECK("MADI 1 SyncCheck", 0),
	HDSPM_SYNC_CHECK("MADI 2 SyncCheck", 1),
	HDSPM_SYNC_CHECK("MADI 3 SyncCheck", 2),
	HDSPM_SYNC_CHECK("WC SyncCheck", 3),
	HDSPM_SYNC_CHECK("AES SyncCheck", 4),
	HDSPM_AUTOSYNC_SAMPLE_RATE("MADI 1 Frequency", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("MADI 2 Frequency", 1),
	HDSPM_AUTOSYNC_SAMPLE_RATE("MADI 3 Frequency", 2),
	HDSPM_AUTOSYNC_SAMPLE_RATE("WC Frequency", 3),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES Frequency", 4),
	MADIFX_MADI_CHANNELCOUNT("MADI 1 RX #ch", 0),
	MADIFX_MADI_CHANNELCOUNT("MADI 2 RX #ch", 1),
	MADIFX_MADI_CHANNELCOUNT("MADI 3 RX #ch", 2),
	MADIFX_TOGGLE_SETTING("MADI 1 TX 64ch mode", MADIFX_madi1_tx_64ch),
	MADIFX_TOGGLE_SETTING("MADI 2 TX 64ch mode", MADIFX_madi2_tx_64ch),
	MADIFX_TOGGLE_SETTING("MADI 3 TX 64ch mode", MADIFX_madi3_tx_64ch),
	MADIFX_TOGGLE_SETTING("MADI 1 SMUX mode", MADIFX_madi1_smux),
	MADIFX_TOGGLE_SETTING("MADI 2 SMUX mode", MADIFX_madi2_smux),
	MADIFX_TOGGLE_SETTING("MADI 3 SMUX mode", MADIFX_madi3_smux),
	MADIFX_TOGGLE_SETTING("WC Term", MADIFX_WCK_TERM),
	MADIFX_TOGGLE_SETTING("WC single speed", MADIFX_WCK48),
	MADIFX_TOGGLE_SETTING("AES professional", MADIFX_PRO),
	MADIFX_TOGGLE_SETTING("Redundancy mode", MADIFX_redundancy_mode),
	MADIFX_TOGGLE_SETTING("Mirror MADI out", MADIFX_mirror_madi_out),
	MADIFX_CLOCK_SELECT("Clock Selection", 0)
};


static struct snd_kcontrol_new snd_madifx_playback_mixer = HDSPM_PLAYBACK_MIXER;


static int madifx_update_simple_mixer_controls(struct hdspm *hdspm)
{
	int i;
	snd_printk(KERN_WARNING "MADIFX: updating broken mixer\n");

	for (i = hdspm->ds_out_channels; i < hdspm->ss_out_channels; ++i) {
		if (hdspm->system_sample_rate > 48000) {
			hdspm->playback_mixer_ctls[i]->vd[0].access =
				SNDRV_CTL_ELEM_ACCESS_INACTIVE |
				SNDRV_CTL_ELEM_ACCESS_READ |
				SNDRV_CTL_ELEM_ACCESS_VOLATILE;
		} else {
			hdspm->playback_mixer_ctls[i]->vd[0].access =
				SNDRV_CTL_ELEM_ACCESS_READWRITE |
				SNDRV_CTL_ELEM_ACCESS_VOLATILE;
		}
		snd_ctl_notify(hdspm->card, SNDRV_CTL_EVENT_MASK_VALUE |
				SNDRV_CTL_EVENT_MASK_INFO,
				&hdspm->playback_mixer_ctls[i]->id);
	}

	return 0;
}


static int snd_madifx_create_controls(struct snd_card *card,
					struct hdspm *hdspm)
{
	unsigned int idx, limit;
	int err;
	struct snd_kcontrol *kctl;
	struct snd_kcontrol_new *list = NULL;

	switch (hdspm->io_type) {
	case MADIFX:
		list = snd_madifx_controls_madi;
		limit = ARRAY_SIZE(snd_madifx_controls_madi);
		break;
	}

	if (NULL != list) {
		for (idx = 0; idx < limit; idx++) {
			err = snd_ctl_add(card,
					snd_ctl_new1(&list[idx], hdspm));
			if (err < 0)
				return err;
		}
	}


	/* FIXME: MADI FX, we don't know how to mix, yet */
#if 0
	/* create simple 1:1 playback mixer controls */
	snd_madifx_playback_mixer.name = "Chn";
	if (hdspm->system_sample_rate >= 128000) {
		limit = hdspm->qs_out_channels;
	} else if (hdspm->system_sample_rate >= 64000) {
		limit = hdspm->ds_out_channels;
	} else {
		limit = hdspm->ss_out_channels;
	}
	for (idx = 0; idx < limit; ++idx) {
		snd_madifx_playback_mixer.index = idx + 1;
		kctl = snd_ctl_new1(&snd_madifx_playback_mixer, hdspm);
		err = snd_ctl_add(card, kctl);
		if (err < 0)
			return err;
		hdspm->playback_mixer_ctls[idx] = kctl;
	}
#endif


	return 0;
}

/*------------------------------------------------------------
   /proc interface
 ------------------------------------------------------------*/

static void
snd_madifx_proc_read_madifx(struct snd_info_entry *entry,
			 struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	u32 status, inp_status, control, freq;

	char *system_clock_mode;
	int x;

	unsigned int period;
	u64 freq_const = 0;
	u32 rate;

	status = madifx_read(hdspm, MADIFX_RD_STATUS);
	inp_status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);
	control = hdspm->control_register;
	freq = madifx_read(hdspm, MADIFX_RD_INP_FREQ);

#if 0
	snd_iprintf(buffer, "%s (Card #%d) Rev.%x Status2first3bits: %x\n",
			hdspm->card_name, hdspm->card->number + 1,
			hdspm->firmware_rev,
			(status2 & HDSPM_version0) |
			(status2 & HDSPM_version1) | (status2 &
				HDSPM_version2));
#endif
	snd_iprintf(buffer, "HW Serial: 0x%x\n", madifx_read(hdspm,
				MADIFX_RD_VERSION));

	snd_iprintf(buffer, "IRQ: %d Registers bus: 0x%lx VM: 0x%lx\n",
			hdspm->irq, hdspm->port, (unsigned long)hdspm->iobase);

	snd_iprintf(buffer, "--- System ---\n");

	snd_iprintf(buffer,
		"IRQ Pending: Audio=%d, MIDI0=%d, MIDI1=%d, MIDI2=%d MIDI3=%d IRQcount=%d\n",
		status & HDSPM_audioIRQPending,
		(status & MADIFX_mIRQ0) ? 1 : 0,
		(status & MADIFX_mIRQ1) ? 1 : 0,
		(status & MADIFX_mIRQ2) ? 1 : 0,
		(status & MADIFX_mIRQ3) ? 1 : 0,
		hdspm->irq_count);

	snd_iprintf(buffer,
		"MIDI FIFO: Out0=0x%x, Out1=0x%x, Out2=0x%x, Out3=0x%x\n",
		madifx_read(hdspm, MADIFX_midi_out0_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_out1_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_out2_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_out3_status) & 0xFF);
	snd_iprintf(buffer,
		"MIDI FIFO: in0=0x%x, in1=0x%x, in2=0x%x, in3=0x%x\n",
		madifx_read(hdspm, MADIFX_midi_in0_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_in1_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_in2_status) & 0xFF,
		madifx_read(hdspm, MADIFX_midi_in3_status) & 0xFF);
	snd_iprintf(buffer,
		"Register:\ncontrol=0x%x, settings=0x%x, status=0x%x, "
		"input=0x%x inp_freq=0x%x\n",
		hdspm->control_register, hdspm->settings_register,
		status, inp_status, freq);

		switch (hdspm->io_type) {
		case MADIFX:
			freq_const = 131072000000000ULL;
			break;
		}

		period = madifx_read(hdspm, MADIFX_RD_PLL_FREQ);
		snd_iprintf(buffer, "    period: %u\n", period);


		/* rate = freq_const/period; */
		rate = div_u64(freq_const, period);

		rate *= madifx_speed_multiplier(hdspm);


		snd_iprintf(buffer, "  Frequency: %u Hz\n",
				(unsigned int) rate);


	snd_iprintf(buffer, "--- Settings ---\n");

	x = madifx_get_latency(hdspm);

	snd_iprintf(buffer,
		"Size (Latency): %d samples\n", x);

#if 0
	snd_iprintf(buffer, "Line out: %s\n",
		(hdspm->control_register & HDSPM_LineOut) ? "on " : "off");

	switch (hdspm->control_register & HDSPM_InputMask) {
	case HDSPM_InputOptical:
		insel = "Optical";
		break;
	case HDSPM_InputCoaxial:
		insel = "Coaxial";
		break;
	default:
		insel = "Unknown";
	}

	snd_iprintf(buffer,
		"ClearTrackMarker = %s, Transmit in %s Channel Mode, "
		"Auto Input %s\n",
		(hdspm->control_register & HDSPM_clr_tms) ? "on" : "off",
		(hdspm->control_register & HDSPM_TX_64ch) ? "64" : "56",
		(hdspm->control_register & HDSPM_AutoInp) ? "on" : "off");
#endif


	if (1 == madifx_system_clock_mode(hdspm))
		system_clock_mode = "Slave";
	else
		system_clock_mode = "Master";
	snd_iprintf(buffer, "AutoSync Reference: %s\n", system_clock_mode);

	snd_iprintf(buffer, "Selected clock source: %s\n",
			hdspm->texts_clocksource[madifx_get_clock_select(hdspm)]);

	snd_iprintf(buffer, "System Clock Frequency: %d\n",
			hdspm->system_sample_rate);


#if 0
	snd_iprintf(buffer, "--- Status:\n");

	x = status & HDSPM_madiSync;
	x2 = status2 & HDSPM_wcSync;

	snd_iprintf(buffer, "Inputs MADI=%s, WordClock=%s\n",
			(status & HDSPM_madiLock) ? (x ? "Sync" : "Lock") :
			"NoLock",
			(status2 & HDSPM_wcLock) ? (x2 ? "Sync" : "Lock") :
			"NoLock");

	switch (madifx_autosync_ref(hdspm)) {
	case HDSPM_AUTOSYNC_FROM_SYNC_IN:
		autosync_ref = "Sync In";
		break;
	case HDSPM_AUTOSYNC_FROM_TCO:
		autosync_ref = "TCO";
		break;
	case HDSPM_AUTOSYNC_FROM_WORD:
		autosync_ref = "Word Clock";
		break;
	case HDSPM_AUTOSYNC_FROM_MADI:
		autosync_ref = "MADI Sync";
		break;
	case HDSPM_AUTOSYNC_FROM_NONE:
		autosync_ref = "Input not valid";
		break;
	default:
		autosync_ref = "---";
		break;
	}
	snd_iprintf(buffer,
		"AutoSync: Reference= %s, Freq=%d (MADI = %d, Word = %d)\n",
		autosync_ref, madifx_external_sample_rate(hdspm),
		(status & HDSPM_madiFreqMask) >> 22,
		(status2 & HDSPM_wcFreqMask) >> 5);

	snd_iprintf(buffer, "Input: %s, Mode=%s\n",
		(status & HDSPM_AB_int) ? "Coax" : "Optical",
		(status & HDSPM_RX_64ch) ? "64 channels" :
		"56 channels");
#endif

	snd_iprintf(buffer, "\n");
}


#ifdef CONFIG_SND_DEBUG
static void
snd_madifx_proc_read_debug(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;

	int j, i;

	for (i = 0; i < 256 /* 1024*64 */; i += j) {
		snd_iprintf(buffer, "0x%08X: ", i);
		for (j = 0; j < 16; j += 4)
			snd_iprintf(buffer, "%08X ", madifx_read(hdspm, i + j));
		snd_iprintf(buffer, "\n");
	}
}
#endif


static void snd_madifx_proc_ports_in(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	int i;

	snd_iprintf(buffer, "# generated by hdspm\n");

	for (i = 0; i < hdspm->max_channels_in; i++)
		snd_iprintf(buffer, "%d=%s\n", i+1, hdspm->port_names_in[i]);
}

static void snd_madifx_proc_ports_out(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	int i;

	snd_iprintf(buffer, "# generated by hdspm\n");

	for (i = 0; i < hdspm->max_channels_out; i++)
		snd_iprintf(buffer, "%d=%s\n", i+1, hdspm->port_names_out[i]);
}


static void __devinit snd_madifx_proc_init(struct hdspm *hdspm)
{
	struct snd_info_entry *entry;

	if (!snd_card_proc_new(hdspm->card, "madifx", &entry)) {
		switch (hdspm->io_type) {
		case MADIFX:
			snd_info_set_text_ops(entry, hdspm,
					snd_madifx_proc_read_madifx);
			break;
		}
	}

	if (!snd_card_proc_new(hdspm->card, "ports.in", &entry))
		snd_info_set_text_ops(entry, hdspm, snd_madifx_proc_ports_in);

	if (!snd_card_proc_new(hdspm->card, "ports.out", &entry))
		snd_info_set_text_ops(entry, hdspm, snd_madifx_proc_ports_out);

#ifdef CONFIG_SND_DEBUG
	/* debug file to read all hdspm registers */
	if (!snd_card_proc_new(hdspm->card, "debug", &entry))
		snd_info_set_text_ops(entry, hdspm,
				snd_madifx_proc_read_debug);
#endif
}

/*------------------------------------------------------------
   hdspm intitialize
 ------------------------------------------------------------*/

static int snd_madifx_set_defaults(struct hdspm *hdspm)
{
	/* ASSUMPTION: hdspm->lock is either held, or there is no need to
	   hold it (e.g. during module initialization).
	   */

	/* set defaults:       */

	hdspm->settings_register = 0;

	switch (hdspm->io_type) {
	case MADIFX:
		/* OSX: LAT_3+BUF_SIZ_1+BUF_SIZ_2+freq1; */
		hdspm->control_register = MADIFX_LAT_3 + MADIFX_BUF_SIZ_1 +
			MADIFX_BUF_SIZ_2 + MADIFX_freq1;
		/* PRO+madi1_tx_64ch+madi2_tx_64ch+madi3_tx_64ch; */
		hdspm->settings_register = 0x8 + 0x80 + 0x100 + 0x200;
		break;
	}

	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	madifx_compute_period_size(hdspm);

	/* silence everything */

	/* FIXME */
	/* all_in_all_mixer(hdspm, 0 * UNITY_GAIN); */

	madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);

	/* set a default rate so that the channel map is set up. */
	madifx_set_rate(hdspm, 48000, 1);

	return 0;
}


/*------------------------------------------------------------
   interrupt
 ------------------------------------------------------------*/

static irqreturn_t snd_madifx_interrupt(int irq, void *dev_id)
{
	struct hdspm *hdspm = (struct hdspm *) dev_id;
	unsigned int status;
	int i, audio, midi, schedule = 0;
	/* cycles_t now; */

	status = madifx_read(hdspm, MADIFX_RD_STATUS);

	audio = status & HDSPM_audioIRQPending;
	midi = status & (MADIFX_mIRQ0 | MADIFX_mIRQ1 |
			MADIFX_mIRQ2 | MADIFX_mIRQ3);

	/* now = get_cycles(); */
	/**
	 *   LAT_2..LAT_0 period  counter (win)  counter (mac)
	 *          6       4096   ~256053425     ~514672358
	 *          5       2048   ~128024983     ~257373821
	 *          4       1024    ~64023706     ~128718089
	 *          3        512    ~32005945      ~64385999
	 *          2        256    ~16003039      ~32260176
	 *          1        128     ~7998738      ~16194507
	 *          0         64     ~3998231       ~8191558
	 **/
	/*
	   snd_printk(KERN_INFO "snd_madifx_interrupt %llu @ %llx\n",
	   now-hdspm->last_interrupt, status & 0xFFC0);
	   hdspm->last_interrupt = now;
	*/

	if (!audio && !midi)
		return IRQ_NONE;

	madifx_write(hdspm, MADIFX_IRQ_ACK, 0);
	hdspm->irq_count++;


	if (audio) {
		if (hdspm->capture_substream)
			snd_pcm_period_elapsed(hdspm->capture_substream);

		if (hdspm->playback_substream)
			snd_pcm_period_elapsed(hdspm->playback_substream);
	}

	if (midi) {
		i = 0;
		while (i < hdspm->midiPorts) {
			if ((madifx_read(hdspm,
				hdspm->midi[i].statusIn) & 0xff) &&
					(status & hdspm->midi[i].irq)) {
				/* we disable interrupts for this input until
				 * processing is done
				 */
				hdspm->control_register &= ~hdspm->midi[i].ie;
				madifx_write(hdspm, MADIFX_CONTROL_REG,
						hdspm->control_register);
				hdspm->midi[i].pending = 1;
				schedule = 1;
			}

			i++;
		}

		if (schedule)
			tasklet_hi_schedule(&hdspm->midi_tasklet);
	}

	return IRQ_HANDLED;
}

/*------------------------------------------------------------
   pcm interface
  ------------------------------------------------------------*/


static snd_pcm_uframes_t snd_madifx_hw_pointer(struct snd_pcm_substream
					      *substream)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	return madifx_hw_pointer(hdspm);
}


static int snd_madifx_reset(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	struct snd_pcm_substream *other;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		other = hdspm->capture_substream;
	else
		other = hdspm->playback_substream;

	if (hdspm->running)
		runtime->status->hw_ptr = madifx_hw_pointer(hdspm);
	else
		runtime->status->hw_ptr = 0;
	if (other) {
		struct snd_pcm_substream *s;
		struct snd_pcm_runtime *oruntime = other->runtime;
		snd_pcm_group_for_each_entry(s, substream) {
			if (s == other) {
				oruntime->status->hw_ptr =
					runtime->status->hw_ptr;
				break;
			}
		}
	}
	return 0;
}

static int snd_madifx_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	int err;
	int i;
	pid_t this_pid;
	pid_t other_pid;

	spin_lock_irq(&hdspm->lock);

	if (substream->pstr->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		this_pid = hdspm->playback_pid;
		other_pid = hdspm->capture_pid;
	} else {
		this_pid = hdspm->capture_pid;
		other_pid = hdspm->playback_pid;
	}

	if (other_pid > 0 && this_pid != other_pid) {

		/* The other stream is open, and not by the same
		   task as this one. Make sure that the parameters
		   that matter are the same.
		   */

		if (params_rate(params) != hdspm->system_sample_rate) {
			spin_unlock_irq(&hdspm->lock);
			_snd_pcm_hw_param_setempty(params,
					SNDRV_PCM_HW_PARAM_RATE);
			return -EBUSY;
		}

		if (params_period_size(params) != hdspm->period_bytes / 4) {
			spin_unlock_irq(&hdspm->lock);
			_snd_pcm_hw_param_setempty(params,
					SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
			return -EBUSY;
		}

	}
	/* We're fine. */
	spin_unlock_irq(&hdspm->lock);

	/* how to make sure that the rate matches an externally-set one ?   */

	spin_lock_irq(&hdspm->lock);
	err = madifx_set_rate(hdspm, params_rate(params), 0);
	if (err < 0) {
		snd_printk(KERN_INFO "err on madifx_set_rate: %d\n", err);
		spin_unlock_irq(&hdspm->lock);
		_snd_pcm_hw_param_setempty(params,
				SNDRV_PCM_HW_PARAM_RATE);
		return err;
	}
	spin_unlock_irq(&hdspm->lock);

	err = madifx_set_interrupt_interval(hdspm,
			params_period_size(params));
	if (err < 0) {
		snd_printk(KERN_INFO "err on madifx_set_interrupt_interval: %d\n", err);
		_snd_pcm_hw_param_setempty(params,
				SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
		return err;
	}

	/* Memory allocation, takashi's method, dont know if we should
	 * spinlock
	 */
	/* malloc all buffer even if not enabled to get sure */
	/* Update for MADI rev 204: we need to allocate for all channels,
	 * otherwise it doesn't work at 96kHz */

#define NUM_AES_PAGES (32768*2/4096)
#define NUM_MADI_PAGES (32768*192/4096)
#define NUM_DMA_CH_PAGES (32768*8/4096)
#define MADIFX_HARDWARE_PAGE_SIZE 4096

	{
		int wanted;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			wanted = OUTPUT_DMA_BUFFER_SIZE;
		else
			wanted = INPUT_DMA_BUFFER_SIZE;

		err = snd_pcm_lib_malloc_pages(substream, wanted);
		if (err < 0) {
			snd_printk(KERN_INFO "err on snd_pcm_lib_malloc_pages: %d\n", err);
			return err;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* initialise default DMA table. Will be
		 * overwritten in a second. */
		for (i = 0; i < MADIFX_MAX_PAGE_TABLE_SIZE/2; i++)
			hdspm->dmaPageTable[i] = snd_pcm_sgbuf_get_addr(substream, 0);

		/* AES Out, stereo */
		for (i = 0; i < NUM_AES_PAGES; i++)
			hdspm->dmaPageTable[i] = snd_pcm_sgbuf_get_addr(substream,
					i * MADIFX_HARDWARE_PAGE_SIZE);

		/* Phones Out, stereo */
		for (i = 0; i < NUM_AES_PAGES; i++)
			hdspm->dmaPageTable[i+1*NUM_DMA_CH_PAGES] =
				snd_pcm_sgbuf_get_addr(substream,
						(i+1*NUM_AES_PAGES) * MADIFX_HARDWARE_PAGE_SIZE);

		/* MADI Out, 192 channels */
		for (i = 0; i < NUM_MADI_PAGES; i++)
			hdspm->dmaPageTable[i+2*NUM_DMA_CH_PAGES] =
				snd_pcm_sgbuf_get_addr(substream,
						(i+2*NUM_AES_PAGES) * MADIFX_HARDWARE_PAGE_SIZE);

		for (i = 0; i < MADIFX_MAX_PAGE_TABLE_SIZE/2; i++)
			madifx_write(hdspm, MADIFX_PAGE_ADDRESS_LIST + (4 * i),
					hdspm->dmaPageTable[i]);

		for (i = 0; i < 32; ++i)
			snd_madifx_enable_out(hdspm, i, 1);

		hdspm->playback_buffer =
			(unsigned char *) substream->runtime->dma_area;
		snd_printdd("Allocated sample buffer for playback at %p\n",
				hdspm->playback_buffer);
	} else {
		/* initialise default DMA table. Will be
		 * overwritten in a second. */
		for (i = MADIFX_MAX_PAGE_TABLE_SIZE/2;
				i < MADIFX_MAX_PAGE_TABLE_SIZE; i++) {
			hdspm->dmaPageTable[i] = snd_pcm_sgbuf_get_addr(substream, 0);
		}

		/* setup DMA page table */
		/* AES In, stereo */
		for (i = 0; i < NUM_AES_PAGES; i++) {
			hdspm->dmaPageTable[i+MADIFX_MAX_PAGE_TABLE_SIZE/2] =
				snd_pcm_sgbuf_get_addr(substream,
						i * MADIFX_HARDWARE_PAGE_SIZE);
		}

		/* MADI In, 192 channels */
		for (i = 0; i < NUM_MADI_PAGES; i++) {
			hdspm->dmaPageTable[i + MADIFX_MAX_PAGE_TABLE_SIZE / 2 + NUM_DMA_CH_PAGES] =
				snd_pcm_sgbuf_get_addr(substream,
						(i + NUM_AES_PAGES) * MADIFX_HARDWARE_PAGE_SIZE);
		}

		for (i = MADIFX_MAX_PAGE_TABLE_SIZE/2;
				i < MADIFX_MAX_PAGE_TABLE_SIZE; i++) {
			madifx_write(hdspm, MADIFX_PAGE_ADDRESS_LIST + (4 * i),
					hdspm->dmaPageTable[i]);
		}

		for (i = 0; i < 32; ++i)
			snd_madifx_enable_in(hdspm, i, 1);

		hdspm->capture_buffer =
			(unsigned char *) substream->runtime->dma_area;
		snd_printdd("Allocated sample buffer for capture at %p\n",
				hdspm->capture_buffer);
	}

	/* Switch to native float format if requested */
	if (SNDRV_PCM_FORMAT_FLOAT_LE == params_format(params)) {
		if (!(hdspm->control_register & MADIFX_float_format))
			snd_printk(KERN_INFO "hdspm: Switching to native 32bit LE float format.\n");

		hdspm->control_register |= MADIFX_float_format;
	} else if (SNDRV_PCM_FORMAT_S32_LE == params_format(params)) {
		if (hdspm->control_register & MADIFX_float_format)
			snd_printk(KERN_INFO "hdspm: Switching to native 32bit LE integer format.\n");

		hdspm->control_register &= ~MADIFX_float_format;
	}
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

static int snd_madifx_hw_free(struct snd_pcm_substream *substream)
{
	int i;
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* params_channels(params) should be enough,
		   but to get sure in case of error */
		for (i = 0; i < 32; ++i)
			snd_madifx_enable_out(hdspm, i, 0);

		hdspm->playback_buffer = NULL;
	} else {
		for (i = 0; i < 32; ++i)
			snd_madifx_enable_in(hdspm, i, 0);

		hdspm->capture_buffer = NULL;

	}

	snd_pcm_lib_free_pages(substream);

	return 0;
}


static int snd_madifx_channel_info(struct snd_pcm_substream *substream,
		struct snd_pcm_channel_info *info)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		int last_madi_channel = 193;

		if (snd_BUG_ON(info->channel >= hdspm->max_channels_out)) {
			snd_printk(KERN_INFO
				"snd_madifx_channel_info: output channel out of range (%d)\n",
				info->channel);
			return -EINVAL;
		}

		switch (hdspm->speedmode) {
		case ss:
			/* MADI FX Playback channel map
			   Outputstream 0 with 2 channels at byte offset 0          AES
			   Outputstream 1 with 8 channels at byte offset 131072     MADI
			   Outputstream 2 with 8 channels at byte offset 393216
			   Outputstream 3 with 8 channels at byte offset 655360
			   Outputstream 4 with 8 channels at byte offset 917504
			   Outputstream 5 with 8 channels at byte offset 1179648
			   Outputstream 6 with 8 channels at byte offset 1441792
			   Outputstream 7 with 8 channels at byte offset 1703936
			   Outputstream 8 with 8 channels at byte offset 1966080
			   Outputstream 9 with 8 channels at byte offset 2228224
			   Outputstream 10 with 8 channels at byte offset 2490368
			   Outputstream 11 with 8 channels at byte offset 2752512
			   Outputstream 12 with 8 channels at byte offset 3014656
			   Outputstream 13 with 8 channels at byte offset 3276800
			   Outputstream 14 with 8 channels at byte offset 3538944
			   Outputstream 15 with 8 channels at byte offset 3801088
			   Outputstream 16 with 8 channels at byte offset 4063232
			   Outputstream 17 with 8 channels at byte offset 4325376
			   Outputstream 18 with 8 channels at byte offset 4587520
			   Outputstream 19 with 8 channels at byte offset 4849664
			   Outputstream 20 with 8 channels at byte offset 5111808
			   Outputstream 21 with 8 channels at byte offset 5373952
			   Outputstream 22 with 8 channels at byte offset 5636096
			   Outputstream 23 with 8 channels at byte offset 5898240
			   Outputstream 24 with 8 channels at byte offset 6160384
			   Outputstream 25 with 2 channels at byte offset 65536     Phones
			   */

			/* Note: channels start at zero */
			last_madi_channel = 193;
			break;
		case ds:
			last_madi_channel = 97;
			break;
		case qs:
			last_madi_channel = 49;
			break;
		}
	info->offset = (info->channel < 2) ?
	    0 : ((info->channel > last_madi_channel) ? 65536 :
		    131072 + 8 * 4 * 8192 * ((info->channel-2)/8));
	info->first = (info->channel < 2 || info->channel > last_madi_channel) ?
	    32 * info->channel : 32 * ((info->channel-2) % 8);
	info->step = (info->channel < 2 || info->channel > last_madi_channel) ?
	    64 : 256;
	} else {
		if (snd_BUG_ON(info->channel >= hdspm->max_channels_in)) {
			snd_printk(KERN_INFO
				"snd_madifx_channel_info: input channel out of range (%d)\n",
					info->channel);
			return -EINVAL;
		}

		switch (hdspm->speedmode) {
			/* MADI FX Input channel map
			   Inputstream 0 with 2 channels at byte offset 0           AES
			   Inputstream 1 with 8 channels at byte offset 65536       MADI
			   Inputstream 2 with 8 channels at byte offset 327680
			   Inputstream 3 with 8 channels at byte offset 589824
			   Inputstream 4 with 8 channels at byte offset 851968
			   Inputstream 5 with 8 channels at byte offset 1114112
			   Inputstream 6 with 8 channels at byte offset 1376256
			   Inputstream 7 with 8 channels at byte offset 1638400
			   Inputstream 8 with 8 channels at byte offset 1900544
			   Inputstream 9 with 8 channels at byte offset 2162688
			   Inputstream 10 with 8 channels at byte offset 2424832
			   Inputstream 11 with 8 channels at byte offset 2686976
			   Inputstream 12 with 8 channels at byte offset 2949120
			   Inputstream 13 with 8 channels at byte offset 3211264
			   Inputstream 14 with 8 channels at byte offset 3473408
			   Inputstream 15 with 8 channels at byte offset 3735552
			   Inputstream 16 with 8 channels at byte offset 3997696
			   Inputstream 17 with 8 channels at byte offset 4259840
			   Inputstream 18 with 8 channels at byte offset 4521984
			   Inputstream 19 with 8 channels at byte offset 4784128
			   Inputstream 20 with 8 channels at byte offset 5046272
			   Inputstream 21 with 8 channels at byte offset 5308416
			   Inputstream 22 with 8 channels at byte offset 5570560
			   Inputstream 23 with 8 channels at byte offset 5832704
			   Inputstream 24 with 8 channels at byte offset 6094848
			   */
		case ss:
		case ds:
		case qs:
			info->offset = (info->channel < 2) ? 0 : 65536 + 8 * 4 * 8192 *
				((info->channel-2)/8);
			info->first = (info->channel < 2) ? 32 * info->channel :
				32 * ((info->channel-2) % 8);
			info->step = (info->channel < 2) ? 64 : 256;
			break;
		}
	}

	return 0;
}


static int snd_madifx_ioctl(struct snd_pcm_substream *substream,
		unsigned int cmd, void *arg)
{
	switch (cmd) {
	case SNDRV_PCM_IOCTL1_RESET:
		return snd_madifx_reset(substream);

	case SNDRV_PCM_IOCTL1_CHANNEL_INFO:
		{
			struct snd_pcm_channel_info *info = arg;
			return snd_madifx_channel_info(substream, info);
		}
	default:
		break;
	}

	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int snd_madifx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	struct snd_pcm_substream *other;
	int running;

	spin_lock(&hdspm->lock);
	running = hdspm->running;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		running |= 1 << substream->stream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		running &= ~(1 << substream->stream);
		break;
	default:
		snd_BUG();
		spin_unlock(&hdspm->lock);
		return -EINVAL;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		other = hdspm->capture_substream;
	else
		other = hdspm->playback_substream;

	if (other) {
		struct snd_pcm_substream *s;
		snd_pcm_group_for_each_entry(s, substream) {
			if (s == other) {
				snd_pcm_trigger_done(s, substream);
				if (cmd == SNDRV_PCM_TRIGGER_START)
					running |= 1 << s->stream;
				else
					running &= ~(1 << s->stream);
				goto _ok;
			}
		}
		if (cmd == SNDRV_PCM_TRIGGER_START) {
			if (!(running & (1 << SNDRV_PCM_STREAM_PLAYBACK))
					&& substream->stream ==
					SNDRV_PCM_STREAM_CAPTURE)
				madifx_silence_playback(hdspm);
		} else {
			if (running &&
				substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				madifx_silence_playback(hdspm);
		}
	} else {
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			madifx_silence_playback(hdspm);
	}
_ok:
	snd_pcm_trigger_done(substream, substream);
	if (!hdspm->running && running)
		madifx_start_audio(hdspm);
	else if (hdspm->running && !running)
		madifx_stop_audio(hdspm);
	hdspm->running = running;
	spin_unlock(&hdspm->lock);

	return 0;
}

static int snd_madifx_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_hardware snd_madifx_playback_subinfo = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_COMPLEX |
		 SNDRV_PCM_INFO_SYNC_START | SNDRV_PCM_INFO_DOUBLE),
	.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_FLOAT_LE,
	.rates = (SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 |
		  SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_64000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000),
	.rate_min = 32000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 196,
	.buffer_bytes_max = OUTPUT_DMA_BUFFER_SIZE,
	.period_bytes_min = (32 * 4),
	.period_bytes_max = OUTPUT_DMA_BUFFER_SIZE,
	.periods_min = 2,
	.periods_max = 1024,
	.fifo_size = 0
};

static struct snd_pcm_hardware snd_madifx_capture_subinfo = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_COMPLEX |
		 SNDRV_PCM_INFO_SYNC_START),
	.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_FLOAT_LE,
	.rates = (SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 |
		  SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_64000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000),
	.rate_min = 32000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 194,
	.buffer_bytes_max = INPUT_DMA_BUFFER_SIZE,
	.period_bytes_min = (32 * 4),
	.period_bytes_max = INPUT_DMA_BUFFER_SIZE,
	.periods_min = 2,
	.periods_max = 1024,
	.fifo_size = 0
};

static int snd_madifx_hw_rule_in_channels_rate(struct snd_pcm_hw_params *params,
					   struct snd_pcm_hw_rule *rule)
{
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *r =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	if (r->min > 96000 && r->max <= 192000) {
		struct snd_interval t = {
			.min = hdspm->qs_in_channels,
			.max = hdspm->qs_in_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	} else if (r->min > 48000 && r->max <= 96000) {
		struct snd_interval t = {
			.min = hdspm->ds_in_channels,
			.max = hdspm->ds_in_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	} else if (r->max < 64000) {
		struct snd_interval t = {
			.min = hdspm->ss_in_channels,
			.max = hdspm->ss_in_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	}

	return 0;
}

static int snd_madifx_hw_rule_out_channels_rate(struct snd_pcm_hw_params *params,
					   struct snd_pcm_hw_rule *rule)
{
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *r =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	if (r->min > 96000 && r->max <= 192000) {
		struct snd_interval t = {
			.min = hdspm->qs_out_channels,
			.max = hdspm->qs_out_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	} else if (r->min > 48000 && r->max <= 96000) {
		struct snd_interval t = {
			.min = hdspm->ds_out_channels,
			.max = hdspm->ds_out_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	} else if (r->max < 64000) {
		struct snd_interval t = {
			.min = hdspm->ss_out_channels,
			.max = hdspm->ss_out_channels,
			.integer = 1,
		};
		return snd_interval_refine(c, &t);
	} else {
	}
	return 0;
}

static int snd_madifx_hw_rule_rate_in_channels(struct snd_pcm_hw_params *params,
					   struct snd_pcm_hw_rule *rule)
{
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *r =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	if (c->min >= hdspm->ss_in_channels) {
		struct snd_interval t = {
			.min = 32000,
			.max = 48000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	} else if (c->max <= hdspm->qs_in_channels) {
		struct snd_interval t = {
			.min = 128000,
			.max = 192000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	} else if (c->max <= hdspm->ds_in_channels) {
		struct snd_interval t = {
			.min = 64000,
			.max = 96000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	}

	return 0;
}
static int snd_madifx_hw_rule_rate_out_channels(struct snd_pcm_hw_params *params,
					   struct snd_pcm_hw_rule *rule)
{
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *r =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	if (c->min >= hdspm->ss_out_channels) {
		struct snd_interval t = {
			.min = 32000,
			.max = 48000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	} else if (c->max <= hdspm->qs_out_channels) {
		struct snd_interval t = {
			.min = 128000,
			.max = 192000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	} else if (c->max <= hdspm->ds_out_channels) {
		struct snd_interval t = {
			.min = 64000,
			.max = 96000,
			.integer = 1,
		};
		return snd_interval_refine(r, &t);
	}

	return 0;
}

static int snd_madifx_hw_rule_in_channels(struct snd_pcm_hw_params *params,
				      struct snd_pcm_hw_rule *rule)
{
	unsigned int list[3];
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	list[0] = hdspm->qs_in_channels;
	list[1] = hdspm->ds_in_channels;
	list[2] = hdspm->ss_in_channels;
	return snd_interval_list(c, 3, list, 0);
}

static int snd_madifx_hw_rule_out_channels(struct snd_pcm_hw_params *params,
				      struct snd_pcm_hw_rule *rule)
{
	unsigned int list[3];
	struct hdspm *hdspm = rule->private;
	struct snd_interval *c = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	list[0] = hdspm->qs_out_channels;
	list[1] = hdspm->ds_out_channels;
	list[2] = hdspm->ss_out_channels;
	return snd_interval_list(c, 3, list, 0);
}


static int snd_madifx_playback_open(struct snd_pcm_substream *substream)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	spin_lock_irq(&hdspm->lock);

	snd_pcm_set_sync(substream);


	runtime->hw = snd_madifx_playback_subinfo;

	if (hdspm->capture_substream == NULL)
		madifx_stop_audio(hdspm);

	hdspm->playback_pid = current->pid;
	hdspm->playback_substream = substream;

	spin_unlock_irq(&hdspm->lock);

	snd_pcm_hw_constraint_msbits(runtime, 0, 32, 24);
	snd_pcm_hw_constraint_pow2(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);

	switch (hdspm->io_type) {
	case MADIFX:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     32, 4096);
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
					     8192, 8192);
		break;

	default:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     64, 8192);
		break;
	}

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			snd_madifx_hw_rule_rate_out_channels, hdspm,
			SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd_madifx_hw_rule_out_channels, hdspm,
			SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd_madifx_hw_rule_out_channels_rate, hdspm,
			SNDRV_PCM_HW_PARAM_RATE, -1);

	return 0;
}

static int snd_madifx_playback_release(struct snd_pcm_substream *substream)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);

	spin_lock_irq(&hdspm->lock);

	hdspm->playback_pid = -1;
	hdspm->playback_substream = NULL;

	spin_unlock_irq(&hdspm->lock);

	return 0;
}


static int snd_madifx_capture_open(struct snd_pcm_substream *substream)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	spin_lock_irq(&hdspm->lock);
	snd_pcm_set_sync(substream);
	runtime->hw = snd_madifx_capture_subinfo;

	if (hdspm->playback_substream == NULL)
		madifx_stop_audio(hdspm);

	hdspm->capture_pid = current->pid;
	hdspm->capture_substream = substream;

	spin_unlock_irq(&hdspm->lock);

	snd_pcm_hw_constraint_msbits(runtime, 0, 32, 24);
	snd_pcm_hw_constraint_pow2(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);

	switch (hdspm->io_type) {
	case MADIFX:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     32, 4096);
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
					     8192, 8192);
		break;

	default:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     64, 8192);
		break;
	}

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			snd_madifx_hw_rule_rate_in_channels, hdspm,
			SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd_madifx_hw_rule_in_channels, hdspm,
			SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd_madifx_hw_rule_in_channels_rate, hdspm,
			SNDRV_PCM_HW_PARAM_RATE, -1);

	return 0;
}

static int snd_madifx_capture_release(struct snd_pcm_substream *substream)
{
	struct hdspm *hdspm = snd_pcm_substream_chip(substream);

	spin_lock_irq(&hdspm->lock);

	hdspm->capture_pid = -1;
	hdspm->capture_substream = NULL;

	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_hwdep_dummy_op(struct snd_hwdep *hw, struct file *file)
{
	/* we have nothing to initialize but the call is required */
	return 0;
}

static inline int copy_u32_le(void __user *dest, void __iomem *src)
{
	u32 val = readl(src);
	return copy_to_user(dest, &val, 4);
}

static int snd_madifx_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct hdspm *hdspm = hw->private_data;
	struct madifx_mixer_ioctl mixer;
	struct madifx_config info;
	struct madifx_status status;
	struct madifx_level_buffer *levels;
	long unsigned int s;
	int i = 0;

	switch (cmd) {

	case SNDRV_MADIFX_IOCTL_GET_LEVEL:
		{
			int row;

			levels = &(hdspm->peak_rms);
			for (row = 1; row <= 5 ; row++) {
				int rms_index, peak_index;
				u32 *target_rms, *target_peak;

				switch (row) {
				case 1:
					rms_index = MADIFX_RD_RMS_IN;
					peak_index = MADIFX_RD_PEAK_IN;
					target_rms = levels->rms_in;
					target_peak = levels->peak_in;
					break;
				case 2:
					rms_index = MADIFX_RD_RMS_PLAY;
					peak_index = MADIFX_RD_PEAK_PLAY;
					target_rms = levels->rms_play;
					target_peak = levels->peak_play;
					break;
				case 3:
					rms_index = MADIFX_RD_RMS_OUT;
					peak_index = MADIFX_RD_PEAK_OUT;
					target_rms = levels->rms_out;
					target_peak = levels->peak_out;
					break;
				case 4:
					rms_index = MADIFX_RD_RMS_IN_PRE;
					peak_index = MADIFX_RD_PEAK_IN_PRE;
					target_rms = levels->rms_in_pre;
					target_peak = levels->peak_in_pre;
					break;
				default:
					rms_index = MADIFX_RD_RMS_OUT_PRE;
					peak_index = MADIFX_RD_PEAK_OUT_PRE;
					target_rms = levels->rms_out_pre;
					target_peak = levels->peak_out_pre;
					break;
				}

				for (i = 0; i < 2 * 256; i++)
					*(target_rms + i) = hdspm->level_buffer[rms_index + i];

				for (i = 0; i < 256; i++)
					*(target_peak + i) = hdspm->level_buffer[peak_index + i];
			}

		}


		levels->speed = hdspm->speedmode;

		s = copy_to_user(argp, levels, sizeof(struct madifx_level_buffer));
		if (0 != s) {
			/* snd_printk(KERN_ERR "copy_to_user(.., .., %lu): %lu
			 [Levels]\n", sizeof(struct madifx_peak_rms), s);
			 */
			return -EFAULT;
		}

		madifx_write(hdspm, MADIFX_START_LEVEL, 0);

		break;


	case SNDRV_MADIFX_IOCTL_GET_CONFIG:

		memset(&info, 0, sizeof(info));
		spin_lock_irq(&hdspm->lock);

		for (i = 0; i < ARRAY_SIZE(info.madi_tx_64); i++) {
			info.madi_tx_64[i] = madifx_read_toggle_setting(hdspm,
					(MADIFX_madi1_tx_64ch << i));

			info.madi_smux[i] = madifx_read_toggle_setting(hdspm,
					(MADIFX_madi1_smux << i));
		}

		info.wcterm = madifx_read_toggle_setting(hdspm,
				MADIFX_WCK_TERM);

		info.wck48 = madifx_read_toggle_setting(hdspm, MADIFX_WCK48);

		info.aespro = madifx_read_toggle_setting(hdspm, MADIFX_PRO);

		info.redundancy_mode = madifx_read_toggle_setting(hdspm,
				MADIFX_redundancy_mode);

		info.mirror_madi_out = madifx_read_toggle_setting(hdspm,
				MADIFX_mirror_madi_out);


		spin_unlock_irq(&hdspm->lock);
		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;

	case SNDRV_MADIFX_IOCTL_GET_STATUS:
		memset(&status, 0, sizeof(status));

		status.card_type = hdspm->io_type;

		status.clock_selection = madifx_get_clock_select(hdspm);

		status.system_sample_rate =
			madifx_get_system_sample_rate(hdspm);


		for (i = 0; i < ARRAY_SIZE(status.sync_check); i++) {
			status.sync_check[i] = madifx_sync_check(hdspm, i);
			status.external_sample_rates[i] =
				HDSPM_bit2freq(madifx_external_freq_index(hdspm, i));
		}

		for (i = 0; i < ARRAY_SIZE(status.madi_channelcount); i++) {
			status.madi_channelcount[i] =
				madifx_get_madichannelcount(hdspm, i);
		}


		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;


		break;

	case SNDRV_HDSPM_IOCTL_GET_MIXER:
		if (copy_from_user(&mixer, argp, sizeof(mixer)))
			return -EFAULT;
		if (copy_to_user((void __user *)mixer.mixer, hdspm->mixer,
					sizeof(struct madifx_mixer)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static struct snd_pcm_ops snd_madifx_playback_ops = {
	.open = snd_madifx_playback_open,
	.close = snd_madifx_playback_release,
	.ioctl = snd_madifx_ioctl,
	.hw_params = snd_madifx_hw_params,
	.hw_free = snd_madifx_hw_free,
	.prepare = snd_madifx_prepare,
	.trigger = snd_madifx_trigger,
	.pointer = snd_madifx_hw_pointer,
	.page = snd_pcm_sgbuf_ops_page,
};

static struct snd_pcm_ops snd_madifx_capture_ops = {
	.open = snd_madifx_capture_open,
	.close = snd_madifx_capture_release,
	.ioctl = snd_madifx_ioctl,
	.hw_params = snd_madifx_hw_params,
	.hw_free = snd_madifx_hw_free,
	.prepare = snd_madifx_prepare,
	.trigger = snd_madifx_trigger,
	.pointer = snd_madifx_hw_pointer,
	.page = snd_pcm_sgbuf_ops_page,
};

static int __devinit snd_madifx_create_hwdep(struct snd_card *card,
					    struct hdspm *hdspm)
{
	struct snd_hwdep *hw;
	int err;

	err = snd_hwdep_new(card, "MADIFX hwdep", 0, &hw);
	if (err < 0)
		return err;

	hdspm->hwdep = hw;
	hw->private_data = hdspm;
	strcpy(hw->name, "MADIFX hwdep interface");

	hw->ops.open = snd_madifx_hwdep_dummy_op;
	hw->ops.ioctl = snd_madifx_hwdep_ioctl;
	hw->ops.ioctl_compat = snd_madifx_hwdep_ioctl;
	hw->ops.release = snd_madifx_hwdep_dummy_op;

	return 0;
}


/*------------------------------------------------------------
   memory interface
 ------------------------------------------------------------*/
static int __devinit snd_madifx_preallocate_memory(struct hdspm *hdspm)
{
	int err;
	int i;
	int lpti; /* level page table index */
	struct snd_pcm *pcm;
	size_t wanted;
	dma_addr_t levelPageTable[MADIFX_NUM_LEVEL_PAGES];

	pcm = hdspm->pcm;


	wanted = max(INPUT_DMA_BUFFER_SIZE, OUTPUT_DMA_BUFFER_SIZE);

	hdspm->dmaPageTable = kzalloc(sizeof(dma_addr_t) *
			MADIFX_MAX_PAGE_TABLE_SIZE, GFP_KERNEL);

	if (!hdspm->dmaPageTable) {
		snd_printk(KERN_ERR
				"MADIFX: unable to kmalloc dmaPageTable memory\n");
		return -ENOMEM;
	}

	err =
	     snd_pcm_lib_preallocate_pages_for_all(pcm,
						   SNDRV_DMA_TYPE_DEV_SG,
						   snd_dma_pci_data(hdspm->pci),
						   wanted,
						   wanted);
	if (err < 0) {
		snd_printdd("Could not preallocate %zd Bytes\n", wanted);

		return err;
	} else
		snd_printdd(" Preallocated %zd Bytes\n", wanted);

	/* allocate level buffer */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_SG,
			snd_dma_pci_data(hdspm->pci),
			MADIFX_LEVEL_BUFFER_SIZE, &hdspm->dmaLevelBuffer);
	if (err < 0) {
		snd_printk(KERN_ERR
				"MADIFX: Unable to allocate DMA level buffer\n");
		return -ENOMEM;
	}

	/* Fill level page table */
	for (i = 0; i < MADIFX_NUM_LEVEL_PAGES; i++) {
		levelPageTable[i] = snd_sgbuf_get_addr(&(hdspm->dmaLevelBuffer),
				i * MADIFX_HARDWARE_PAGE_SIZE);

	}

	/* Write level page table to device */
	lpti = (MADIFX == hdspm->io_type) ? MADIFX_LPTI_HMFX :
		MADIFX_LPTI_MFXT;

	for (i = 0; i < MADIFX_NUM_LEVEL_PAGES; i++) {
		madifx_write(hdspm, MADIFX_PAGE_ADDRESS_LIST + (4 * (lpti + i)),
				levelPageTable[i]);
	}

	hdspm->level_buffer = snd_sgbuf_get_ptr(&(hdspm->dmaLevelBuffer), 0);

	memset(hdspm->level_buffer, 0, MADIFX_LEVEL_BUFFER_SIZE);


	return 0;
}


/* ------------- ALSA Devices ---------------------------- */
static int __devinit snd_madifx_create_pcm(struct snd_card *card,
					  struct hdspm *hdspm)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(card, hdspm->card_name, 0, 1, 1, &pcm);
	if (err < 0)
		return err;

	hdspm->pcm = pcm;
	pcm->private_data = hdspm;
	strcpy(pcm->name, hdspm->card_name);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_madifx_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_madifx_capture_ops);

	pcm->info_flags = SNDRV_PCM_INFO_JOINT_DUPLEX;

	err = snd_madifx_preallocate_memory(hdspm);
	if (err < 0)
		return err;

	return 0;
}

static inline void snd_madifx_initialize_midi_flush(struct hdspm *hdspm)
{
	int i;

	for (i = 0; i < hdspm->midiPorts; i++)
		snd_madifx_flush_midi_input(hdspm, i);
}

static int __devinit snd_madifx_create_alsa_devices(struct snd_card *card,
						   struct hdspm *hdspm)
{
	int err, i;

	snd_printdd("Create card...\n");
	err = snd_madifx_create_pcm(card, hdspm);
	if (err < 0)
		return err;

	i = 0;
	while (i < hdspm->midiPorts) {
		err = snd_madifx_create_midi(card, hdspm, i);
		if (err < 0)
			return err;
		i++;
	}

	err = snd_madifx_create_controls(card, hdspm);
	if (err < 0)
		return err;

	err = snd_madifx_create_hwdep(card, hdspm);
	if (err < 0)
		return err;

	snd_printdd("proc init...\n");
	snd_madifx_proc_init(hdspm);

	hdspm->system_sample_rate = -1;
	hdspm->last_external_sample_rate = -1;
	hdspm->last_internal_sample_rate = -1;
	hdspm->playback_pid = -1;
	hdspm->capture_pid = -1;
	hdspm->capture_substream = NULL;
	hdspm->playback_substream = NULL;

	snd_printdd("Set defaults...\n");
	err = snd_madifx_set_defaults(hdspm);
	if (err < 0)
		return err;

	snd_printdd("Update mixer controls...\n");
#if 0
	/* FIXME: MADI FX disable, old mixer is broken */
	madifx_update_simple_mixer_controls(hdspm);
#endif

	snd_printdd("Initializeing complete ???\n");

	err = snd_card_register(card);
	if (err < 0) {
		snd_printk(KERN_ERR "MADIFX: error registering card\n");
		return err;
	}

	snd_printdd("... yes now\n");

	return 0;
}

static int __devinit snd_madifx_create(struct snd_card *card,
		struct hdspm *hdspm) {

	struct pci_dev *pci = hdspm->pci;
	int err;
	unsigned long io_extent;

	hdspm->irq = -1;
	hdspm->card = card;

	spin_lock_init(&hdspm->lock);

	pci_read_config_word(hdspm->pci,
			PCI_CLASS_REVISION, &hdspm->firmware_rev);

	strcpy(card->mixername, "Xilinx FPGA");
	strcpy(card->driver, "MADIFX");

	switch (hdspm->firmware_rev) {
	case HDSPM_MADIFX_REV:
		hdspm->io_type = MADIFX;
		hdspm->card_name = "RME MADI FX";
		hdspm->midiPorts = 4;
		break;
	default:
		snd_printk(KERN_ERR
				"MADIFX: unknown firmware revision %x\n",
				hdspm->firmware_rev);
		return -ENODEV;
	}

	err = pci_enable_device(pci);
	if (err < 0)
		return err;

	pci_set_master(hdspm->pci);

	err = pci_request_regions(pci, "hdspm");
	if (err < 0)
		return err;

	hdspm->port = pci_resource_start(pci, 0);
	io_extent = pci_resource_len(pci, 0);

	snd_printdd("grabbed memory region 0x%lx-0x%lx\n",
			hdspm->port, hdspm->port + io_extent - 1);

	hdspm->iobase = ioremap_nocache(hdspm->port, io_extent);
	if (!hdspm->iobase) {
		snd_printk(KERN_ERR
				"HDSPM: unable to remap region 0x%lx-0x%lx\n",
				hdspm->port, hdspm->port + io_extent - 1);
		return -EBUSY;
	}
	snd_printdd("remapped region (0x%lx) 0x%lx-0x%lx\n",
			(unsigned long)hdspm->iobase, hdspm->port,
			hdspm->port + io_extent - 1);

	if (request_irq(pci->irq, snd_madifx_interrupt,
			IRQF_SHARED, KBUILD_MODNAME, hdspm)) {
		snd_printk(KERN_ERR "HDSPM: unable to use IRQ %d\n", pci->irq);
		return -EBUSY;
	}

	snd_printdd("use IRQ %d\n", pci->irq);

	hdspm->irq = pci->irq;

	snd_printdd("kmalloc Mixer memory of %zd Bytes\n",
			sizeof(struct madifx_mixer));

	hdspm->newmixer = kzalloc(sizeof(struct madifx_newmixer), GFP_KERNEL);
	if (!hdspm->newmixer) {
		snd_printk(KERN_ERR
				"HDSPM: unable to kmalloc Mixer memory of %d Bytes\n",
				(int)sizeof(struct madifx_newmixer));
		return -ENOMEM;
	}

	/* FIXME: Drop this later when mixer is correctly replaced */
	hdspm->mixer = kzalloc(sizeof(struct madifx_mixer), GFP_KERNEL);
	if (!hdspm->mixer) {
		snd_printk(KERN_ERR
				"HDSPM: unable to kmalloc Mixer memory of %d Bytes\n",
				(int)sizeof(struct madifx_mixer));
		return -ENOMEM;
	}


	{
		/* This somehow initialises the mixer. I have no idea what it
		 * does exactly.
		 */
		int i;
		for (i = 0; i < MADIFX_NUM_OUTPUT_GAINS; i++)
			hdspm->newmixer->output_gain[i] = 0x9000;

		for (i = 0; i < MADIFX_LIST_LENGTH; i++) {
			hdspm->newmixer->listCh[i] = 0;
			hdspm->newmixer->listVol[i] = 0;
		}

		for (i = 0; i < 196; i++) {
			hdspm->newmixer->listCh[i] = (256 + i) | (i << 9);
			hdspm->newmixer->listVol[i] = 32768+(32768>>3);
		}

		/* Of course, the data has to be written to the device before
		 * something can happen.
		 */
		for (i = 0; i < MADIFX_LIST_LENGTH; i++) {
			madifx_write(hdspm, MADIFX_MIXER_LIST_CH + (4 * i),
					hdspm->newmixer->listCh[i]);
			madifx_write(hdspm, MADIFX_MIXER_LIST_VOL + (4 * i),
					hdspm->newmixer->listVol[i]);
		}

		for (i = 0; i < MADIFX_NUM_OUTPUT_GAINS; i++) {
			madifx_write(hdspm, MADIFX_WR_OUTPUT_GAIN + (4 * i),
					hdspm->newmixer->output_gain[i]);
		}
	}


	hdspm->port_names_in = NULL;
	hdspm->port_names_out = NULL;

	switch (hdspm->io_type) {
	case MADIFX:
		hdspm->ss_in_channels = MADIFX_SS_IN_CHANNELS;
		hdspm->ds_in_channels = MADIFX_DS_IN_CHANNELS;
		hdspm->qs_in_channels = MADIFX_QS_IN_CHANNELS;
		hdspm->ss_out_channels = MADIFX_SS_OUT_CHANNELS;
		hdspm->ds_out_channels = MADIFX_DS_OUT_CHANNELS;
		hdspm->qs_out_channels = MADIFX_QS_OUT_CHANNELS;
		/* FIXME: portnames and stuff missing */
		break;
	}


	/* texts */
	switch (hdspm->io_type) {
	/* Keep the switch if MFXT will be different */
	case MADIFX:
		hdspm->texts_clocksource = texts_madifx_clock_source;
		hdspm->texts_clocksource_items = 6;
		break;
	}

	tasklet_init(&hdspm->midi_tasklet,
			madifx_midi_tasklet, (unsigned long) hdspm);


	sprintf(card->id, "MADIFXtest");
	snd_card_set_id(card, card->id);

	snd_printdd("create alsa devices.\n");
	err = snd_madifx_create_alsa_devices(card, hdspm);
	if (err < 0)
		return err;

	snd_madifx_initialize_midi_flush(hdspm);

	return 0;
}


static int snd_madifx_free(struct hdspm *hdspm)
{

	if (hdspm->port) {

		/* stop th audio, and cancel all interrupts */
		hdspm->control_register &=
		    ~(MADIFX_START | MADIFX_IE_AUDIO |
		      MADIFX_IEN0 | MADIFX_IEN1 |
		      MADIFX_IEN2 | MADIFX_IEN3);
		madifx_write(hdspm, MADIFX_CONTROL_REG,
			    hdspm->control_register);
		madifx_write(hdspm, MADIFX_START_LEVEL, 0);
	}

	if (hdspm->irq >= 0)
		free_irq(hdspm->irq, (void *) hdspm);

	kfree(hdspm->mixer);
	kfree(hdspm->dmaPageTable);
	snd_dma_free_pages(&(hdspm->dmaLevelBuffer));

	if (hdspm->iobase)
		iounmap(hdspm->iobase);

	if (hdspm->port)
		pci_release_regions(hdspm->pci);

	pci_disable_device(hdspm->pci);
	return 0;
}


static void snd_madifx_card_free(struct snd_card *card)
{
	struct hdspm *hdspm = card->private_data;

	if (hdspm)
		snd_madifx_free(hdspm);
}


static int __devinit snd_madifx_probe(struct pci_dev *pci,
				     const struct pci_device_id *pci_id)
{
	static int dev;
	struct hdspm *hdspm;
	struct snd_card *card;
	int err;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	err = snd_card_create(index[dev], id[dev],
			THIS_MODULE, sizeof(struct hdspm), &card);
	if (err < 0)
		return err;

	hdspm = card->private_data;
	card->private_free = snd_madifx_card_free;
	hdspm->dev = dev;
	hdspm->pci = pci;

	snd_card_set_dev(card, &pci->dev);

	err = snd_madifx_create(card, hdspm);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	sprintf(card->shortname, "%s_%x",
			hdspm->card_name,
			hdspm->serial);
	sprintf(card->longname, "%s S/N 0x%x at 0x%lx, irq %d",
			hdspm->card_name,
			hdspm->serial,
			hdspm->port, hdspm->irq);

	err = snd_card_register(card);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	pci_set_drvdata(pci, card);

	madifx_write(hdspm, MADIFX_START_LEVEL, 1);

	dev++;
	return 0;
}

static void __devexit snd_madifx_remove(struct pci_dev *pci)
{
	snd_card_free(pci_get_drvdata(pci));
	pci_set_drvdata(pci, NULL);
}

static struct pci_driver madifx_driver = {
	.name = KBUILD_MODNAME,
	.id_table = snd_madifx_ids,
	.probe = snd_madifx_probe,
	.remove = __devexit_p(snd_madifx_remove),
};

module_pci_driver(madifx_driver);
