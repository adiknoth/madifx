/*
 *   ALSA driver for RME Hammerfall DSP MADI audio interface(s)
 *
 *      Copyright (c) 2003 Winfried Ritsch (IEM)
 *      code based on hdsp.c   Paul Davis
 *                             Marcus Andersson
 *                             Thomas Charbonnel
 *      Modified 2006-06-01 for AES32 support by Remy Bruno
 *                                               <remy.bruno@trinnov.com>
 *
 *      Modified 2009-04-13 for proper metering by Florian Faber
 *                                               <faber@faberman.de>
 *
 *      Modified 2009-04-14 for native float support by Florian Faber
 *                                               <faber@faberman.de>
 *
 *      Modified 2009-04-26 fixed bug in rms metering by Florian Faber
 *                                               <faber@faberman.de>
 *
 *      Modified 2009-04-30 added hw serial number support by Florian Faber
 *
 *      Modified 2011-01-14 added S/PDIF input on RayDATs by Adrian Knoth
 *
 *	Modified 2011-01-25 variable period sizes on RayDAT/AIO by Adrian Knoth
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
#include <asm/io.h>

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
	"Winfried Ritsch <ritsch_AT_iem.at>, "
	"Paul Davis <paul@linuxaudiosystems.com>, "
	"Marcus Andersson, Thomas Charbonnel <thomas@undata.org>, "
	"Remy Bruno <remy.bruno@trinnov.com>, "
	"Florian Faber <faberman@linuxproaudio.org>, "
	"Adrian Knoth <adi@drcomp.erfurt.thur.de>"
);
MODULE_DESCRIPTION("RME MADIFX");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{RME HDSPM-MADIFX}}");

/* --- Write registers. ---
  These are defined as byte-offsets from the iobase value.  */

#define MADIFX_FREQ_REG		(1*4)
#define MADIFX_SETTINGS_REG	(2*4)
#define MADIFX_MIXER_LIST_VOL	(16384*4)
#define MADIFX_MIXER_LIST_CH	(20480*4)
#define MADIFX_WR_OUTPUT_GAIN	((24576+256)*4)

#define MADIFX_SAMPLE_FRAMES_PER_BUFFER		8192


#define MADIFX_CONTROL_REG	     0
#define MADIFX_IRQ_ACK           (3*4)
#define HDSPM_control2Reg	     256  /* not in specs ???????? */
#define HDSPM_midiDataOut0	     352  /* just believe in old code */
#define HDSPM_midiDataOut1	     356
#define HDSPM_eeprom_wr		     384  /* for AES32 */

/* DMA enable for 64 channels, only Bit 0 is relevant */
#define HDSPM_outputEnableBase       384
#define HDSPM_inputEnableBase        256

#define MADIFX_PAGE_ADDRESS_LIST   (8192*4)

/* page table size in entries, multiply by 4 to get byte offset */
#define MADIFX_MAX_PAGE_TABLE_SIZE	4096

#define HDSPM_MADI_mixerBase    32768	/* 32768-65535 for 2x64x64 Fader */

#define HDSPM_MATRIX_MIXER_SIZE  8192	/* = 2*64*64 * 4 Byte => 32kB */

/* --- Read registers. ---
   These are defined as byte-offsets from the iobase value */
#define HDSPM_statusRegister    0
/*#define HDSPM_statusRegister2  96 */
/* after RME Windows driver sources, status2 is 4-byte word # 48 = word at
 * offset 192, for AES32 *and* MADI
 * => need to check that offset 192 is working on MADI */
#define HDSPM_statusRegister2  192
#define HDSPM_timecodeRegister 128

/* AIO, RayDAT */
#define HDSPM_RD_STATUS_0 0
#define HDSPM_RD_STATUS_1 64
#define HDSPM_RD_STATUS_2 128
#define HDSPM_RD_STATUS_3 192

#define HDSPM_RD_TCO           256
#define HDSPM_RD_PLL_FREQ      512
#define HDSPM_WR_TCO           128

#define HDSPM_TCO1_TCO_lock			0x00000001
#define HDSPM_TCO1_WCK_Input_Range_LSB		0x00000002
#define HDSPM_TCO1_WCK_Input_Range_MSB		0x00000004
#define HDSPM_TCO1_LTC_Input_valid		0x00000008
#define HDSPM_TCO1_WCK_Input_valid		0x00000010
#define HDSPM_TCO1_Video_Input_Format_NTSC	0x00000020
#define HDSPM_TCO1_Video_Input_Format_PAL	0x00000040

#define HDSPM_TCO1_set_TC			0x00000100
#define HDSPM_TCO1_set_drop_frame_flag		0x00000200
#define HDSPM_TCO1_LTC_Format_LSB		0x00000400
#define HDSPM_TCO1_LTC_Format_MSB		0x00000800

#define HDSPM_TCO2_TC_run			0x00010000
#define HDSPM_TCO2_WCK_IO_ratio_LSB		0x00020000
#define HDSPM_TCO2_WCK_IO_ratio_MSB		0x00040000
#define HDSPM_TCO2_set_num_drop_frames_LSB	0x00080000
#define HDSPM_TCO2_set_num_drop_frames_MSB	0x00100000
#define HDSPM_TCO2_set_jam_sync			0x00200000
#define HDSPM_TCO2_set_flywheel			0x00400000

#define HDSPM_TCO2_set_01_4			0x01000000
#define HDSPM_TCO2_set_pull_down		0x02000000
#define HDSPM_TCO2_set_pull_up			0x04000000
#define HDSPM_TCO2_set_freq			0x08000000
#define HDSPM_TCO2_set_term_75R			0x10000000
#define HDSPM_TCO2_set_input_LSB		0x20000000
#define HDSPM_TCO2_set_input_MSB		0x40000000
#define HDSPM_TCO2_set_freq_from_app		0x80000000


#define HDSPM_midiDataOut0    352
#define HDSPM_midiDataOut1    356
#define HDSPM_midiDataOut2    368

#define HDSPM_midiDataIn0     360
#define HDSPM_midiDataIn1     364
#define HDSPM_midiDataIn2     372
#define HDSPM_midiDataIn3     376

/* status is data bytes in MIDI-FIFO (0-128) */
#define HDSPM_midiStatusOut0  384
#define HDSPM_midiStatusOut1  388
#define HDSPM_midiStatusOut2  400

#define HDSPM_midiStatusIn0   392
#define HDSPM_midiStatusIn1   396
#define HDSPM_midiStatusIn2   404
#define HDSPM_midiStatusIn3   408

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


/* the meters are regular i/o-mapped registers, but offset
   considerably from the rest. the peak registers are reset
   when read; the least-significant 4 bits are full-scale counters;
   the actual peak value is in the most-significant 24 bits.
*/

#define HDSPM_MADI_INPUT_PEAK		4096
#define HDSPM_MADI_PLAYBACK_PEAK	4352
#define HDSPM_MADI_OUTPUT_PEAK		4608

#define HDSPM_MADI_INPUT_RMS_L		6144
#define HDSPM_MADI_PLAYBACK_RMS_L	6400
#define HDSPM_MADI_OUTPUT_RMS_L		6656

#define HDSPM_MADI_INPUT_RMS_H		7168
#define HDSPM_MADI_PLAYBACK_RMS_H	7424
#define HDSPM_MADI_OUTPUT_RMS_H		7680

/* --- Control Register bits --------- */

#define HDSPM_ClockModeMaster      (1<<4) /* 1=Master, 0=Autosync */
#define HDSPM_c0Master		0x1    /* Master clock bit in settings
					  register [RayDAT, AIO] */


#define HDSPM_Frequency0  (1<<6)  /* 0=44.1kHz/88.2kHz 1=48kHz/96kHz */
#define HDSPM_Frequency1  (1<<7)  /* 0=32kHz/64kHz */
#define HDSPM_DoubleSpeed (1<<8)  /* 0=normal speed, 1=double speed */
#define HDSPM_QuadSpeed   (1<<31) /* quad speed bit */

#define HDSPM_Professional (1<<9) /* Professional */ /* AES32 ONLY */
#define HDSPM_TX_64ch     (1<<10) /* Output 64channel MODE=1,
				     56channelMODE=0 */ /* MADI ONLY*/
#define HDSPM_Emphasis    (1<<10) /* Emphasis */ /* AES32 ONLY */

#define HDSPM_AutoInp     (1<<11) /* Auto Input (takeover) == Safe Mode,
                                     0=off, 1=on  */ /* MADI ONLY */
#define HDSPM_Dolby       (1<<11) /* Dolby = "NonAudio" ?? */ /* AES32 ONLY */

#define HDSPM_InputSelect0 (1<<14) /* Input select 0= optical, 1=coax
				    * -- MADI ONLY
				    */
#define HDSPM_InputSelect1 (1<<15) /* should be 0 */

#define HDSPM_SyncRef2     (1<<13)
#define HDSPM_SyncRef3     (1<<25)

#define HDSPM_SMUX         (1<<18) /* Frame ??? */ /* MADI ONY */
#define HDSPM_clr_tms      (1<<19) /* clear track marker, do not use
                                      AES additional bits in
				      lower 5 Audiodatabits ??? */
#define HDSPM_taxi_reset   (1<<20) /* ??? */ /* MADI ONLY ? */
#define HDSPM_WCK48        (1<<20) /* Frame ??? = HDSPM_SMUX */ /* AES32 ONLY */


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


#define HDSPM_LineOut (1<<24) /* Analog Out on channel 63/64 on=1, mute=0 */

#define HDSPM_DS_DoubleWire (1<<26) /* AES32 ONLY */
#define HDSPM_QS_DoubleWire (1<<27) /* AES32 ONLY */
#define HDSPM_QS_QuadWire   (1<<28) /* AES32 ONLY */

#define HDSPM_wclk_sel (1<<30)

/* --- bit helper defines */
#define MADIFX_LatencyMask  (MADIFX_LAT_0|MADIFX_LAT_1|MADIFX_LAT_2|MADIFX_LAT_3)
#define HDSPM_FrequencyMask  (HDSPM_Frequency0|HDSPM_Frequency1|\
			      HDSPM_DoubleSpeed|HDSPM_QuadSpeed)
#define HDSPM_InputMask      (HDSPM_InputSelect0|HDSPM_InputSelect1)
#define HDSPM_InputOptical   0
#define HDSPM_InputCoaxial   (HDSPM_InputSelect0)
#define HDSPM_SyncRefMask    (HDSPM_SyncRef0|HDSPM_SyncRef1|\
			      HDSPM_SyncRef2|HDSPM_SyncRef3)

#define HDSPM_c0_SyncRef0      0x2
#define HDSPM_c0_SyncRef1      0x4
#define HDSPM_c0_SyncRef2      0x8
#define HDSPM_c0_SyncRef3      0x10
#define HDSPM_c0_SyncRefMask   (HDSPM_c0_SyncRef0 | HDSPM_c0_SyncRef1 |\
				HDSPM_c0_SyncRef2 | HDSPM_c0_SyncRef3)

#define HDSPM_SYNC_FROM_WORD    0	/* Preferred sync reference */
#define HDSPM_SYNC_FROM_MADI    1	/* choices - used by "pref_sync_ref" */
#define HDSPM_SYNC_FROM_TCO     2
#define HDSPM_SYNC_FROM_SYNC_IN 3

#define HDSPM_Frequency32KHz    HDSPM_Frequency0
#define HDSPM_Frequency44_1KHz  HDSPM_Frequency1
#define HDSPM_Frequency48KHz   (HDSPM_Frequency1|HDSPM_Frequency0)
#define HDSPM_Frequency64KHz   (HDSPM_DoubleSpeed|HDSPM_Frequency0)
#define HDSPM_Frequency88_2KHz (HDSPM_DoubleSpeed|HDSPM_Frequency1)
#define HDSPM_Frequency96KHz   (HDSPM_DoubleSpeed|HDSPM_Frequency1|\
				HDSPM_Frequency0)
#define HDSPM_Frequency128KHz   (HDSPM_QuadSpeed|HDSPM_Frequency0)
#define HDSPM_Frequency176_4KHz   (HDSPM_QuadSpeed|HDSPM_Frequency1)
#define HDSPM_Frequency192KHz   (HDSPM_QuadSpeed|HDSPM_Frequency1|\
				 HDSPM_Frequency0)


/* Synccheck Status */
#define HDSPM_SYNC_CHECK_NO_LOCK 0
#define HDSPM_SYNC_CHECK_LOCK    1
#define HDSPM_SYNC_CHECK_SYNC	 2

/* AutoSync References - used by "autosync_ref" control switch */
#define HDSPM_AUTOSYNC_FROM_WORD      0
#define HDSPM_AUTOSYNC_FROM_MADI      1
#define HDSPM_AUTOSYNC_FROM_TCO       2
#define HDSPM_AUTOSYNC_FROM_SYNC_IN   3
#define HDSPM_AUTOSYNC_FROM_NONE      4

/* Possible sources of MADI input */
#define HDSPM_OPTICAL 0		/* optical   */
#define HDSPM_COAXIAL 1		/* BNC */

#define madifx_encode_latency(x)       (((x)<<8) & MADIFX_LatencyMask)
#define madifx_decode_latency(x)       ((((x) & MADIFX_LatencyMask)>>8))

#define madifx_encode_in(x) (((x)&0x3)<<14)
#define madifx_decode_in(x) (((x)>>14)&0x3)

/* --- control2 register bits --- */
#define HDSPM_TMS             (1<<0)
#define HDSPM_TCK             (1<<1)
#define HDSPM_TDI             (1<<2)
#define HDSPM_JTAG            (1<<3)
#define HDSPM_PWDN            (1<<4)
#define HDSPM_PROGRAM	      (1<<5)
#define HDSPM_CONFIG_MODE_0   (1<<6)
#define HDSPM_CONFIG_MODE_1   (1<<7)
/*#define HDSPM_VERSION_BIT     (1<<8) not defined any more*/
#define HDSPM_BIGENDIAN_MODE  (1<<9)
#define HDSPM_RD_MULTIPLE     (1<<10)

/* --- Status Register bits --- */ /* MADI ONLY */ /* Bits defined here and
     that do not conflict with specific bits for AES32 seem to be valid also
     for the AES32
 */
#define HDSPM_audioIRQPending    (1<<0)	/* IRQ is high and pending */
#define HDSPM_RX_64ch            (1<<1)	/* Input 64chan. MODE=1, 56chn MODE=0 */
#define HDSPM_AB_int             (1<<2)	/* InputChannel Opt=0, Coax=1
					 * (like inp0)
					 */

#define HDSPM_madiLock           (1<<3)	/* MADI Locked =1, no=0 */
#define HDSPM_madiSync          (1<<18) /* MADI is in sync */

#define HDSPM_tcoLock    0x00000020 /* Optional TCO locked status FOR HDSPe MADI! */
#define HDSPM_tcoSync    0x10000000 /* Optional TCO sync status */

#define HDSPM_syncInLock 0x00010000 /* Sync In lock status FOR HDSPe MADI! */
#define HDSPM_syncInSync 0x00020000 /* Sync In sync status FOR HDSPe MADI! */

#define HDSPM_BufferPositionMask 0x000FFC0 /* Bit 6..15 : h/w buffer pointer */
			/* since 64byte accurate, last 6 bits are not used */



#define HDSPM_DoubleSpeedStatus (1<<19) /* (input) card in double speed */

#define HDSPM_madiFreq0         (1<<22)	/* system freq 0=error */
#define HDSPM_madiFreq1         (1<<23)	/* 1=32, 2=44.1 3=48 */
#define HDSPM_madiFreq2         (1<<24)	/* 4=64, 5=88.2 6=96 */
#define HDSPM_madiFreq3         (1<<25)	/* 7=128, 8=176.4 9=192 */

#define HDSPM_BufferID          (1<<26)	/* (Double)Buffer ID toggles with
					 * Interrupt
					 */
#define HDSPM_tco_detect         0x08000000
#define HDSPM_tco_lock	         0x20000000

#define HDSPM_s2_tco_detect      0x00000040
#define HDSPM_s2_AEBO_D          0x00000080
#define HDSPM_s2_AEBI_D          0x00000100


#define HDSPM_midi0IRQPending    0x40000000
#define HDSPM_midi1IRQPending    0x80000000
#define HDSPM_midi2IRQPending    0x20000000
#define HDSPM_midi2IRQPendingAES 0x00000020
#define HDSPM_midi3IRQPending    0x00200000

/* --- status bit helpers */
#define HDSPM_madiFreqMask  (HDSPM_madiFreq0|HDSPM_madiFreq1|\
			     HDSPM_madiFreq2|HDSPM_madiFreq3)
#define HDSPM_madiFreq32    (HDSPM_madiFreq0)
#define HDSPM_madiFreq44_1  (HDSPM_madiFreq1)
#define HDSPM_madiFreq48    (HDSPM_madiFreq0|HDSPM_madiFreq1)
#define HDSPM_madiFreq64    (HDSPM_madiFreq2)
#define HDSPM_madiFreq88_2  (HDSPM_madiFreq0|HDSPM_madiFreq2)
#define HDSPM_madiFreq96    (HDSPM_madiFreq1|HDSPM_madiFreq2)
#define HDSPM_madiFreq128   (HDSPM_madiFreq0|HDSPM_madiFreq1|HDSPM_madiFreq2)
#define HDSPM_madiFreq176_4 (HDSPM_madiFreq3)
#define HDSPM_madiFreq192   (HDSPM_madiFreq3|HDSPM_madiFreq0)

/* Status2 Register bits */ /* MADI ONLY */

#define HDSPM_version0 (1<<0)	/* not really defined but I guess */
#define HDSPM_version1 (1<<1)	/* in former cards it was ??? */
#define HDSPM_version2 (1<<2)

#define HDSPM_wcLock (1<<3)	/* Wordclock is detected and locked */
#define HDSPM_wcSync (1<<4)	/* Wordclock is in sync with systemclock */

#define HDSPM_wc_freq0 (1<<5)	/* input freq detected via autosync  */
#define HDSPM_wc_freq1 (1<<6)	/* 001=32, 010==44.1, 011=48, */
#define HDSPM_wc_freq2 (1<<7)	/* 100=64, 101=88.2, 110=96, */
/* missing Bit   for               111=128, 1000=176.4, 1001=192 */

#define HDSPM_SyncRef0 0x10000  /* Sync Reference */
#define HDSPM_SyncRef1 0x20000

#define HDSPM_SelSyncRef0 (1<<8)	/* AutoSync Source */
#define HDSPM_SelSyncRef1 (1<<9)	/* 000=word, 001=MADI, */
#define HDSPM_SelSyncRef2 (1<<10)	/* 111=no valid signal */

#define HDSPM_wc_valid (HDSPM_wcLock|HDSPM_wcSync)

#define HDSPM_wcFreqMask  (HDSPM_wc_freq0|HDSPM_wc_freq1|HDSPM_wc_freq2)
#define HDSPM_wcFreq32    (HDSPM_wc_freq0)
#define HDSPM_wcFreq44_1  (HDSPM_wc_freq1)
#define HDSPM_wcFreq48    (HDSPM_wc_freq0|HDSPM_wc_freq1)
#define HDSPM_wcFreq64    (HDSPM_wc_freq2)
#define HDSPM_wcFreq88_2  (HDSPM_wc_freq0|HDSPM_wc_freq2)
#define HDSPM_wcFreq96    (HDSPM_wc_freq1|HDSPM_wc_freq2)

#define HDSPM_status1_F_0 0x0400000
#define HDSPM_status1_F_1 0x0800000
#define HDSPM_status1_F_2 0x1000000
#define HDSPM_status1_F_3 0x2000000
#define HDSPM_status1_freqMask (HDSPM_status1_F_0|HDSPM_status1_F_1|HDSPM_status1_F_2|HDSPM_status1_F_3)


#define HDSPM_SelSyncRefMask       (HDSPM_SelSyncRef0|HDSPM_SelSyncRef1|\
				    HDSPM_SelSyncRef2)
#define HDSPM_SelSyncRef_WORD      0
#define HDSPM_SelSyncRef_MADI      (HDSPM_SelSyncRef0)
#define HDSPM_SelSyncRef_TCO       (HDSPM_SelSyncRef1)
#define HDSPM_SelSyncRef_SyncIn    (HDSPM_SelSyncRef0|HDSPM_SelSyncRef1)
#define HDSPM_SelSyncRef_NVALID    (HDSPM_SelSyncRef0|HDSPM_SelSyncRef1|\
				    HDSPM_SelSyncRef2)

/*
   For AES32, bits for status, status2 and timecode are different
*/
/* status */
#define HDSPM_AES32_wcLock	0x0200000
#define HDSPM_AES32_wcFreq_bit  22
/* (status >> HDSPM_AES32_wcFreq_bit) & 0xF gives WC frequency (cf function
  HDSPM_bit2freq */
#define HDSPM_AES32_syncref_bit  16
/* (status >> HDSPM_AES32_syncref_bit) & 0xF gives sync source */

#define HDSPM_AES32_AUTOSYNC_FROM_WORD 0
#define HDSPM_AES32_AUTOSYNC_FROM_AES1 1
#define HDSPM_AES32_AUTOSYNC_FROM_AES2 2
#define HDSPM_AES32_AUTOSYNC_FROM_AES3 3
#define HDSPM_AES32_AUTOSYNC_FROM_AES4 4
#define HDSPM_AES32_AUTOSYNC_FROM_AES5 5
#define HDSPM_AES32_AUTOSYNC_FROM_AES6 6
#define HDSPM_AES32_AUTOSYNC_FROM_AES7 7
#define HDSPM_AES32_AUTOSYNC_FROM_AES8 8
#define HDSPM_AES32_AUTOSYNC_FROM_NONE 9

/*  status2 */
/* HDSPM_LockAES_bit is given by HDSPM_LockAES >> (AES# - 1) */
#define HDSPM_LockAES   0x80
#define HDSPM_LockAES1  0x80
#define HDSPM_LockAES2  0x40
#define HDSPM_LockAES3  0x20
#define HDSPM_LockAES4  0x10
#define HDSPM_LockAES5  0x8
#define HDSPM_LockAES6  0x4
#define HDSPM_LockAES7  0x2
#define HDSPM_LockAES8  0x1
/*
   Timecode
   After windows driver sources, bits 4*i to 4*i+3 give the input frequency on
   AES i+1
 bits 3210
      0001  32kHz
      0010  44.1kHz
      0011  48kHz
      0100  64kHz
      0101  88.2kHz
      0110  96kHz
      0111  128kHz
      1000  176.4kHz
      1001  192kHz
  NB: Timecode register doesn't seem to work on AES32 card revision 230
*/

/* Mixer Values */
#define UNITY_GAIN          32768	/* = 65536/2 */
#define MINUS_INFINITY_GAIN 0

/* Number of channels for different Speed Modes */
#define MADI_SS_CHANNELS       64
#define MADI_DS_CHANNELS       32
#define MADI_QS_CHANNELS       16

#define MADIFX_SS_IN_CHANNELS       194
#define MADIFX_DS_IN_CHANNELS       98
#define MADIFX_QS_IN_CHANNELS       50

#define MADIFX_SS_OUT_CHANNELS       196
#define MADIFX_DS_OUT_CHANNELS       100
#define MADIFX_QS_OUT_CHANNELS       52

#define RAYDAT_SS_CHANNELS     36
#define RAYDAT_DS_CHANNELS     20
#define RAYDAT_QS_CHANNELS     12

#define AIO_IN_SS_CHANNELS        14
#define AIO_IN_DS_CHANNELS        10
#define AIO_IN_QS_CHANNELS        8
#define AIO_OUT_SS_CHANNELS        16
#define AIO_OUT_DS_CHANNELS        12
#define AIO_OUT_QS_CHANNELS        10

#define AES32_CHANNELS		16

/* the size of a substream (1 mono data stream) */
#define HDSPM_CHANNEL_BUFFER_SAMPLES  (16*1024)
#define HDSPM_CHANNEL_BUFFER_BYTES    (4*HDSPM_CHANNEL_BUFFER_SAMPLES)

/* the size of the area we need to allocate for DMA transfers. the
   size is the same regardless of the number of channels, and
   also the latency to use.
   for one direction !!!
*/
#define HDSPM_DMA_AREA_BYTES (HDSPM_MAX_CHANNELS * HDSPM_CHANNEL_BUFFER_BYTES)
#define HDSPM_DMA_AREA_KILOBYTES (HDSPM_DMA_AREA_BYTES/1024)

#define HDSPM_RAYDAT_REV	211
#define HDSPM_AIO_REV		212
/* FIXME: MADIFACE_REV set to zero to avoid collision with MADIFX */
#define HDSPM_MADIFACE_REV	0
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

static char *texts_autosync_aes_tco[] = { "Word Clock",
					  "AES1", "AES2", "AES3", "AES4",
					  "AES5", "AES6", "AES7", "AES8",
					  "TCO" };
static char *texts_autosync_aes[] = { "Word Clock",
				      "AES1", "AES2", "AES3", "AES4",
				      "AES5", "AES6", "AES7", "AES8" };
static char *texts_autosync_madi_tco[] = { "Word Clock",
					   "MADI", "TCO", "Sync In" };
static char *texts_autosync_madi[] = { "Word Clock",
				       "MADI", "Sync In" };

static char *texts_autosync_raydat_tco[] = {
	"Word Clock",
	"ADAT 1", "ADAT 2", "ADAT 3", "ADAT 4",
	"AES", "SPDIF", "TCO", "Sync In"
};
static char *texts_autosync_raydat[] = {
	"Word Clock",
	"ADAT 1", "ADAT 2", "ADAT 3", "ADAT 4",
	"AES", "SPDIF", "Sync In"
};
static char *texts_autosync_aio_tco[] = {
	"Word Clock",
	"ADAT", "AES", "SPDIF", "TCO", "Sync In"
};
static char *texts_autosync_aio[] = { "Word Clock",
				      "ADAT", "AES", "SPDIF", "Sync In" };

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

static char *texts_ports_madi[] = {
	"MADI.1", "MADI.2", "MADI.3", "MADI.4", "MADI.5", "MADI.6",
	"MADI.7", "MADI.8", "MADI.9", "MADI.10", "MADI.11", "MADI.12",
	"MADI.13", "MADI.14", "MADI.15", "MADI.16", "MADI.17", "MADI.18",
	"MADI.19", "MADI.20", "MADI.21", "MADI.22", "MADI.23", "MADI.24",
	"MADI.25", "MADI.26", "MADI.27", "MADI.28", "MADI.29", "MADI.30",
	"MADI.31", "MADI.32", "MADI.33", "MADI.34", "MADI.35", "MADI.36",
	"MADI.37", "MADI.38", "MADI.39", "MADI.40", "MADI.41", "MADI.42",
	"MADI.43", "MADI.44", "MADI.45", "MADI.46", "MADI.47", "MADI.48",
	"MADI.49", "MADI.50", "MADI.51", "MADI.52", "MADI.53", "MADI.54",
	"MADI.55", "MADI.56", "MADI.57", "MADI.58", "MADI.59", "MADI.60",
	"MADI.61", "MADI.62", "MADI.63", "MADI.64",
};


static char *texts_ports_raydat_ss[] = {
	"ADAT1.1", "ADAT1.2", "ADAT1.3", "ADAT1.4", "ADAT1.5", "ADAT1.6",
	"ADAT1.7", "ADAT1.8", "ADAT2.1", "ADAT2.2", "ADAT2.3", "ADAT2.4",
	"ADAT2.5", "ADAT2.6", "ADAT2.7", "ADAT2.8", "ADAT3.1", "ADAT3.2",
	"ADAT3.3", "ADAT3.4", "ADAT3.5", "ADAT3.6", "ADAT3.7", "ADAT3.8",
	"ADAT4.1", "ADAT4.2", "ADAT4.3", "ADAT4.4", "ADAT4.5", "ADAT4.6",
	"ADAT4.7", "ADAT4.8",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R"
};

static char *texts_ports_raydat_ds[] = {
	"ADAT1.1", "ADAT1.2", "ADAT1.3", "ADAT1.4",
	"ADAT2.1", "ADAT2.2", "ADAT2.3", "ADAT2.4",
	"ADAT3.1", "ADAT3.2", "ADAT3.3", "ADAT3.4",
	"ADAT4.1", "ADAT4.2", "ADAT4.3", "ADAT4.4",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R"
};

static char *texts_ports_raydat_qs[] = {
	"ADAT1.1", "ADAT1.2",
	"ADAT2.1", "ADAT2.2",
	"ADAT3.1", "ADAT3.2",
	"ADAT4.1", "ADAT4.2",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R"
};


static char *texts_ports_aio_in_ss[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4", "ADAT.5", "ADAT.6",
	"ADAT.7", "ADAT.8"
};

static char *texts_ports_aio_out_ss[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4", "ADAT.5", "ADAT.6",
	"ADAT.7", "ADAT.8",
	"Phone.L", "Phone.R"
};

static char *texts_ports_aio_in_ds[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4"
};

static char *texts_ports_aio_out_ds[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4",
	"Phone.L", "Phone.R"
};

static char *texts_ports_aio_in_qs[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4"
};

static char *texts_ports_aio_out_qs[] = {
	"Analogue.L", "Analogue.R",
	"AES.L", "AES.R",
	"SPDIF.L", "SPDIF.R",
	"ADAT.1", "ADAT.2", "ADAT.3", "ADAT.4",
	"Phone.L", "Phone.R"
};

static char *texts_ports_aes32[] = {
	"AES.1", "AES.2", "AES.3", "AES.4", "AES.5", "AES.6", "AES.7",
	"AES.8", "AES.9.", "AES.10", "AES.11", "AES.12", "AES.13", "AES.14",
	"AES.15", "AES.16"
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

struct madifx_tco {
	int input;
	int framerate;
	int wordclock;
	int samplerate;
	int pull;
	int term; /* 0 = off, 1 = on */
};

struct hdspm {
        spinlock_t lock;
	/* only one playback and/or capture stream */
        struct snd_pcm_substream *capture_substream;
        struct snd_pcm_substream *playback_substream;

	char *card_name;	     /* for procinfo */
	unsigned short firmware_rev; /* dont know if relevant (yes if AES32)*/

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

	struct madifx_tco *tco;  /* NULL if no TCO detected */

	char **texts_autosync;
	int texts_autosync_items;

	cycles_t last_interrupt;

	unsigned int serial;

	int speedmode;

	struct madifx_peak_rms peak_rms;
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
						   struct hdspm * hdspm);
static int __devinit snd_madifx_create_pcm(struct snd_card *card,
					  struct hdspm * hdspm);

static inline void snd_madifx_initialize_midi_flush(struct hdspm *hdspm);
static int madifx_update_simple_mixer_controls(struct hdspm *hdspm);
static int madifx_autosync_ref(struct hdspm *hdspm);
static int snd_madifx_set_defaults(struct hdspm *hdspm);
static int madifx_system_clock_mode(struct hdspm *hdspm);
static void madifx_set_sgbuf(struct hdspm *hdspm,
			    struct snd_pcm_substream *substream,
			     unsigned int reg, int offset);

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

static inline void madifx_write(struct hdspm * hdspm, unsigned int reg,
			       unsigned int val)
{
	writel(val, hdspm->iobase + reg);
}

static inline unsigned int madifx_read(struct hdspm * hdspm, unsigned int reg)
{
	return readl(hdspm->iobase + reg);
}

/* for each output channel (chan) I have an Input (in) and Playback (pb) Fader
   mixer is write only on hardware so we have to cache him for read
   each fader is a u32, but uses only the first 16 bit */

static inline int madifx_read_in_gain(struct hdspm * hdspm, unsigned int chan,
				     unsigned int in)
{
	if (chan >= HDSPM_MIXER_CHANNELS || in >= HDSPM_MIXER_CHANNELS)
		return 0;

	return hdspm->mixer->ch[chan].in[in];
}

static inline int madifx_read_pb_gain(struct hdspm * hdspm, unsigned int chan,
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
static inline void snd_madifx_enable_in(struct hdspm * hdspm, int i, int v)
{
	madifx_write(hdspm, HDSPM_inputEnableBase + (4 * i), v);
}

static inline void snd_madifx_enable_out(struct hdspm * hdspm, int i, int v)
{
	madifx_write(hdspm, HDSPM_outputEnableBase + (4 * i), v);
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

/* check for external sample rate */
static int madifx_external_sample_rate(struct hdspm *hdspm)
{
	unsigned int status, status2, timecode;
	int syncref, rate = 0, rate_bits;

	switch (hdspm->io_type) {
	case AES32:
		status2 = madifx_read(hdspm, HDSPM_statusRegister2);
		status = madifx_read(hdspm, HDSPM_statusRegister);
		timecode = madifx_read(hdspm, HDSPM_timecodeRegister);

		syncref = madifx_autosync_ref(hdspm);

		if (syncref == HDSPM_AES32_AUTOSYNC_FROM_WORD &&
				status & HDSPM_AES32_wcLock)
			return HDSPM_bit2freq((status >> HDSPM_AES32_wcFreq_bit) & 0xF);

		if (syncref >= HDSPM_AES32_AUTOSYNC_FROM_AES1 &&
				syncref <= HDSPM_AES32_AUTOSYNC_FROM_AES8 &&
				status2 & (HDSPM_LockAES >>
				(syncref - HDSPM_AES32_AUTOSYNC_FROM_AES1)))
			return HDSPM_bit2freq((timecode >> (4*(syncref-HDSPM_AES32_AUTOSYNC_FROM_AES1))) & 0xF);
		return 0;
		break;

	case MADIface:
		status = madifx_read(hdspm, HDSPM_statusRegister);

		if (!(status & HDSPM_madiLock)) {
			rate = 0;  /* no lock */
		} else {
			switch (status & (HDSPM_status1_freqMask)) {
			case HDSPM_status1_F_0*1:
				rate = 32000; break;
			case HDSPM_status1_F_0*2:
				rate = 44100; break;
			case HDSPM_status1_F_0*3:
				rate = 48000; break;
			case HDSPM_status1_F_0*4:
				rate = 64000; break;
			case HDSPM_status1_F_0*5:
				rate = 88200; break;
			case HDSPM_status1_F_0*6:
				rate = 96000; break;
			case HDSPM_status1_F_0*7:
				rate = 128000; break;
			case HDSPM_status1_F_0*8:
				rate = 176400; break;
			case HDSPM_status1_F_0*9:
				rate = 192000; break;
			default:
				rate = 0; break;
			}
		}

		break;

	case MADI:
	case AIO:
	case RayDAT:
	case MADIFX:
		/* FIXME: MADIFX is just a wild guess */
		status2 = madifx_read(hdspm, HDSPM_statusRegister2);
		status = madifx_read(hdspm, HDSPM_statusRegister);
		rate = 0;

		/* if wordclock has synced freq and wordclock is valid */
		if ((status2 & HDSPM_wcLock) != 0 &&
				(status2 & HDSPM_SelSyncRef0) == 0) {

			rate_bits = status2 & HDSPM_wcFreqMask;


			switch (rate_bits) {
			case HDSPM_wcFreq32:
				rate = 32000;
				break;
			case HDSPM_wcFreq44_1:
				rate = 44100;
				break;
			case HDSPM_wcFreq48:
				rate = 48000;
				break;
			case HDSPM_wcFreq64:
				rate = 64000;
				break;
			case HDSPM_wcFreq88_2:
				rate = 88200;
				break;
			case HDSPM_wcFreq96:
				rate = 96000;
				break;
			default:
				rate = 0;
				break;
			}
		}

		/* if rate detected and Syncref is Word than have it,
		 * word has priority to MADI
		 */
		if (rate != 0 &&
		(status2 & HDSPM_SelSyncRefMask) == HDSPM_SelSyncRef_WORD)
			return rate;

		/* maybe a madi input (which is taken if sel sync is madi) */
		if (status & HDSPM_madiLock) {
			rate_bits = status & HDSPM_madiFreqMask;

			switch (rate_bits) {
			case HDSPM_madiFreq32:
				rate = 32000;
				break;
			case HDSPM_madiFreq44_1:
				rate = 44100;
				break;
			case HDSPM_madiFreq48:
				rate = 48000;
				break;
			case HDSPM_madiFreq64:
				rate = 64000;
				break;
			case HDSPM_madiFreq88_2:
				rate = 88200;
				break;
			case HDSPM_madiFreq96:
				rate = 96000;
				break;
			case HDSPM_madiFreq128:
				rate = 128000;
				break;
			case HDSPM_madiFreq176_4:
				rate = 176400;
				break;
			case HDSPM_madiFreq192:
				rate = 192000;
				break;
			default:
				rate = 0;
				break;
			}

			/* QS and DS rates normally can not be detected
			 * automatically by the card. Only exception is MADI
			 * in 96k frame mode.
			 *
			 * So if we read SS values (32 .. 48k), check for
			 * user-provided DS/QS bits in the control register
			 * and multiply the base frequency accordingly.
			 */
			if (rate <= 48000) {
				if (hdspm->control_register & HDSPM_QuadSpeed)
					rate *= 4;
				else if (hdspm->control_register &
						HDSPM_DoubleSpeed)
					rate *= 2;
			}
		}
		break;
	}

	return rate;
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
	position /= 4; /* Bytes per sample */
	position >>= 4;
	position *= 4; /* this might be wrong */
#if 0
	position -= 4; /* safety offset */
#endif
	position &= (MADIFX_SAMPLE_FRAMES_PER_BUFFER-1);

	return position;
}

static inline void madifx_start_audio(struct hdspm * s)
{
	s->control_register |= (MADIFX_IE_AUDIO | MADIFX_START);
	madifx_write(s, MADIFX_CONTROL_REG, s->control_register);
}

static inline void madifx_stop_audio(struct hdspm * s)
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

	spin_lock_irq(&s->lock);

	/* FIXME: We have four bits, but we don't know the mapping to frames,
	 * yet.
	 */
	n = 0;
	while (frames) {
		n++;
		frames >>= 1;
	}


	s->control_register &= ~MADIFX_LatencyMask;
	s->control_register |= madifx_encode_latency(n);

	madifx_write(s, MADIFX_CONTROL_REG, s->control_register);

	madifx_compute_period_size(s);

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
	case MADI:
	case AES32:
		freq_const = 110069313433624ULL;
		break;
	case RayDAT:
	case AIO:
		freq_const = 104857600000000ULL;
		break;
	case MADIface:
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
	case MADIface:
		n = 131072000000000ULL;  /* 125 MHz */
		break;
	case MADI:
	case AES32:
		n = 110069313433624ULL;  /* 105 MHz */
		break;
	case RayDAT:
	case AIO:
		n = 104857600000000ULL;  /* 100 MHz */
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

/* dummy set rate lets see what happens */
static int madifx_set_rate(struct hdspm * hdspm, int rate, int called_internally)
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

			snd_printk(KERN_WARNING "MADIFX: "
				   "Warning: device is not running "
				   "as a clock master.\n");
			not_set = 1;
		} else {

			/* hw_param request while in AutoSync mode */
			int external_freq =
			    madifx_external_sample_rate(hdspm);

			if (madifx_autosync_ref(hdspm) ==
			    HDSPM_AUTOSYNC_FROM_NONE) {

				snd_printk(KERN_WARNING "MADIFX: "
					   "Detected no Externel Sync \n");
				not_set = 1;

			} else if (rate != external_freq) {

				snd_printk(KERN_WARNING "MADIFX: "
					   "Warning: No AutoSync source for "
					   "requested rate\n");
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

	if (current_rate <= 48000)
		current_speed = HDSPM_SPEED_SINGLE;
	else if (current_rate <= 96000)
		current_speed = HDSPM_SPEED_DOUBLE;
	else
		current_speed = HDSPM_SPEED_QUAD;

	if (rate <= 48000)
		target_speed = HDSPM_SPEED_SINGLE;
	else if (rate <= 96000)
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

#if 0
	/* Maybe we need to set single-speed. Maybe not. */
	if (HDSPM_SPEED_SINGLE == target_speed) {
		hdspm->settings_register |= MADIFX_WCK48;
		madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);
	}
#endif

	madifx_set_dds_value(hdspm, rate);

	hdspm->control_register &= ~MADIFX_kFrequencyMask;
	hdspm->control_register |= rate_bits;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);



	hdspm->system_sample_rate = rate;

	if (rate <= 48000) {
		hdspm->max_channels_in = hdspm->ss_in_channels;
		hdspm->max_channels_out = hdspm->ss_out_channels;
		hdspm->port_names_in = hdspm->port_names_in_ss;
		hdspm->port_names_out = hdspm->port_names_out_ss;
		hdspm->speedmode = ss;
	} else if (rate <= 96000) {
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
static void all_in_all_mixer(struct hdspm * hdspm, int sgain)
{
	int i, j;
	unsigned int gain;

	/* FIXME: MADI FX unsupported, yet. */
	if (MADIFX == hdspm->io_type) {
		return;
	}

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

static inline unsigned char snd_madifx_midi_read_byte (struct hdspm *hdspm,
						      int id)
{
	/* the hardware already does the relevant bit-mask with 0xff */
	return madifx_read(hdspm, hdspm->midi[id].dataIn);
}

static inline void snd_madifx_midi_write_byte (struct hdspm *hdspm, int id,
					      int val)
{
	/* the hardware already does the relevant bit-mask with 0xff */
	return madifx_write(hdspm, hdspm->midi[id].dataOut, val);
}

static inline int snd_madifx_midi_input_available (struct hdspm *hdspm, int id)
{
	return madifx_read(hdspm, hdspm->midi[id].statusIn) & 0xFF;
}

static inline int snd_madifx_midi_output_possible (struct hdspm *hdspm, int id)
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
	while (snd_madifx_midi_input_available (hdspm, id))
		snd_madifx_midi_read_byte (hdspm, id);
}

static int snd_madifx_midi_output_write (struct madifx_midi *hmidi)
{
	unsigned long flags;
	int n_pending;
	int to_write;
	int i;
	unsigned char buf[128];

	/* Output is not interrupt driven */

	spin_lock_irqsave (&hmidi->lock, flags);
	if (hmidi->output &&
	    !snd_rawmidi_transmit_empty (hmidi->output)) {
		n_pending = snd_madifx_midi_output_possible (hmidi->hdspm,
							    hmidi->id);
		if (n_pending > 0) {
			if (n_pending > (int)sizeof (buf))
				n_pending = sizeof (buf);

			to_write = snd_rawmidi_transmit (hmidi->output, buf,
							 n_pending);
			if (to_write > 0) {
				for (i = 0; i < to_write; ++i)
					snd_madifx_midi_write_byte (hmidi->hdspm,
								   hmidi->id,
								   buf[i]);
			}
		}
	}
	spin_unlock_irqrestore (&hmidi->lock, flags);
	return 0;
}

static int snd_madifx_midi_input_read (struct madifx_midi *hmidi)
{
	unsigned char buf[128]; /* this buffer is designed to match the MIDI
				 * input FIFO size
				 */
	unsigned long flags;
	int n_pending;
	int i;

	spin_lock_irqsave (&hmidi->lock, flags);
	n_pending = snd_madifx_midi_input_available (hmidi->hdspm, hmidi->id);
	if (n_pending > 0) {
		if (hmidi->input) {
			if (n_pending > (int)sizeof (buf))
				n_pending = sizeof (buf);
			for (i = 0; i < n_pending; ++i)
				buf[i] = snd_madifx_midi_read_byte (hmidi->hdspm,
								   hmidi->id);
			if (n_pending)
				snd_rawmidi_receive (hmidi->input, buf,
						     n_pending);
		} else {
			/* flush the MIDI input FIFO */
			while (n_pending--)
				snd_madifx_midi_read_byte (hmidi->hdspm,
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

	return snd_madifx_midi_output_write (hmidi);
}

static void
snd_madifx_midi_input_trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct hdspm *hdspm;
	struct madifx_midi *hmidi;
	unsigned long flags;

	hmidi = substream->rmidi->private_data;
	hdspm = hmidi->hdspm;

	spin_lock_irqsave (&hdspm->lock, flags);
	if (up) {
		if (!(hdspm->control_register & hmidi->ie)) {
			snd_madifx_flush_midi_input (hdspm, hmidi->id);
			hdspm->control_register |= hmidi->ie;
		}
	} else {
		hdspm->control_register &= ~hmidi->ie;
	}

	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);
	spin_unlock_irqrestore (&hdspm->lock, flags);
}

static void snd_madifx_midi_output_timer(unsigned long data)
{
	struct madifx_midi *hmidi = (struct madifx_midi *) data;
	unsigned long flags;

	snd_madifx_midi_output_write(hmidi);
	spin_lock_irqsave (&hmidi->lock, flags);

	/* this does not bump hmidi->istimer, because the
	   kernel automatically removed the timer when it
	   expired, and we are now adding it back, thus
	   leaving istimer wherever it was set before.
	*/

	if (hmidi->istimer) {
		hmidi->timer.expires = 1 + jiffies;
		add_timer(&hmidi->timer);
	}

	spin_unlock_irqrestore (&hmidi->lock, flags);
}

static void
snd_madifx_midi_output_trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct madifx_midi *hmidi;
	unsigned long flags;

	hmidi = substream->rmidi->private_data;
	spin_lock_irqsave (&hmidi->lock, flags);
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
			del_timer (&hmidi->timer);
	}
	spin_unlock_irqrestore (&hmidi->lock, flags);
	if (up)
		snd_madifx_midi_output_write(hmidi);
}

static int snd_madifx_midi_input_open(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	hmidi = substream->rmidi->private_data;
	spin_lock_irq (&hmidi->lock);
	snd_madifx_flush_midi_input (hmidi->hdspm, hmidi->id);
	hmidi->input = substream;
	spin_unlock_irq (&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_output_open(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	hmidi = substream->rmidi->private_data;
	spin_lock_irq (&hmidi->lock);
	hmidi->output = substream;
	spin_unlock_irq (&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_input_close(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	snd_madifx_midi_input_trigger (substream, 0);

	hmidi = substream->rmidi->private_data;
	spin_lock_irq (&hmidi->lock);
	hmidi->input = NULL;
	spin_unlock_irq (&hmidi->lock);

	return 0;
}

static int snd_madifx_midi_output_close(struct snd_rawmidi_substream *substream)
{
	struct madifx_midi *hmidi;

	snd_madifx_midi_output_trigger (substream, 0);

	hmidi = substream->rmidi->private_data;
	spin_lock_irq (&hmidi->lock);
	hmidi->output = NULL;
	spin_unlock_irq (&hmidi->lock);

	return 0;
}

static struct snd_rawmidi_ops snd_madifx_midi_output =
{
	.open =		snd_madifx_midi_output_open,
	.close =	snd_madifx_midi_output_close,
	.trigger =	snd_madifx_midi_output_trigger,
};

static struct snd_rawmidi_ops snd_madifx_midi_input =
{
	.open =		snd_madifx_midi_input_open,
	.close =	snd_madifx_midi_input_close,
	.trigger =	snd_madifx_midi_input_trigger,
};

static int __devinit snd_madifx_create_midi (struct snd_card *card,
					    struct hdspm *hdspm, int id)
{
	int err;
	char buf[32];

	hdspm->midi[id].id = id;
	hdspm->midi[id].hdspm = hdspm;
	spin_lock_init (&hdspm->midi[id].lock);

	if (0 == id) {
		if (MADIface == hdspm->io_type) {
			/* MIDI-over-MADI on HDSPe MADIface */
			hdspm->midi[0].dataIn = HDSPM_midiDataIn2;
			hdspm->midi[0].statusIn = HDSPM_midiStatusIn2;
			hdspm->midi[0].dataOut = HDSPM_midiDataOut2;
			hdspm->midi[0].statusOut = HDSPM_midiStatusOut2;
			hdspm->midi[0].ie = MADIFX_IEN2;
			hdspm->midi[0].irq = HDSPM_midi2IRQPending;
		} else {
			hdspm->midi[0].dataIn = HDSPM_midiDataIn0;
			hdspm->midi[0].statusIn = HDSPM_midiStatusIn0;
			hdspm->midi[0].dataOut = HDSPM_midiDataOut0;
			hdspm->midi[0].statusOut = HDSPM_midiStatusOut0;
			hdspm->midi[0].ie = MADIFX_IEN0;
			hdspm->midi[0].irq = HDSPM_midi0IRQPending;
		}
	} else if (1 == id) {
		hdspm->midi[1].dataIn = HDSPM_midiDataIn1;
		hdspm->midi[1].statusIn = HDSPM_midiStatusIn1;
		hdspm->midi[1].dataOut = HDSPM_midiDataOut1;
		hdspm->midi[1].statusOut = HDSPM_midiStatusOut1;
		hdspm->midi[1].ie = MADIFX_IEN1;
		hdspm->midi[1].irq = HDSPM_midi1IRQPending;
	} else if ((2 == id) && (MADI == hdspm->io_type)) {
		/* MIDI-over-MADI on HDSPe MADI */
		hdspm->midi[2].dataIn = HDSPM_midiDataIn2;
		hdspm->midi[2].statusIn = HDSPM_midiStatusIn2;
		hdspm->midi[2].dataOut = HDSPM_midiDataOut2;
		hdspm->midi[2].statusOut = HDSPM_midiStatusOut2;
		hdspm->midi[2].ie = MADIFX_IEN2;
		hdspm->midi[2].irq = HDSPM_midi2IRQPending;
	} else if (2 == id) {
		/* TCO MTC, read only */
		hdspm->midi[2].dataIn = HDSPM_midiDataIn2;
		hdspm->midi[2].statusIn = HDSPM_midiStatusIn2;
		hdspm->midi[2].dataOut = -1;
		hdspm->midi[2].statusOut = -1;
		hdspm->midi[2].ie = MADIFX_IEN2;
		hdspm->midi[2].irq = HDSPM_midi2IRQPendingAES;
	} else if (3 == id) {
		/* TCO MTC on HDSPe MADI */
		hdspm->midi[3].dataIn = HDSPM_midiDataIn3;
		hdspm->midi[3].statusIn = HDSPM_midiStatusIn3;
		hdspm->midi[3].dataOut = -1;
		hdspm->midi[3].statusOut = -1;
		hdspm->midi[3].ie = MADIFX_IEN3;
		hdspm->midi[3].irq = HDSPM_midi3IRQPending;
	}

	if ((id < 2) || ((2 == id) && ((MADI == hdspm->io_type) ||
					(MADIface == hdspm->io_type)))) {
		if ((id == 0) && (MADIface == hdspm->io_type)) {
			sprintf(buf, "%s MIDIoverMADI", card->shortname);
		} else if ((id == 2) && (MADI == hdspm->io_type)) {
			sprintf(buf, "%s MIDIoverMADI", card->shortname);
		} else {
			sprintf(buf, "%s MIDI %d", card->shortname, id+1);
		}
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
	} else {
		/* TCO MTC, read only */
		sprintf(buf, "%s MTC %d", card->shortname, id+1);
		err = snd_rawmidi_new(card, buf, id, 1, 1,
				&hdspm->midi[id].rmidi);
		if (err < 0)
			return err;

		sprintf(hdspm->midi[id].rmidi->name,
				"%s MTC %d", card->id, id+1);
		hdspm->midi[id].rmidi->private_data = &hdspm->midi[id];

		snd_rawmidi_set_ops(hdspm->midi[id].rmidi,
				SNDRV_RAWMIDI_STREAM_INPUT,
				&snd_madifx_midi_input);

		hdspm->midi[id].rmidi->info_flags |= SNDRV_RAWMIDI_INFO_INPUT;
	}

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

	period = madifx_read(hdspm, HDSPM_RD_PLL_FREQ);
	rate = madifx_calc_dds_value(hdspm, period);

	if (rate > 207000) {
		/* Unreasonable high sample rate as seen on PCI MADI cards. */
		if (0 == madifx_system_clock_mode(hdspm)) {
			/* master mode, return internal sample rate */
			rate = hdspm->system_sample_rate;
		} else {
			/* slave mode, return external sample rate */
			rate = madifx_external_sample_rate(hdspm);
		}
	}

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


/**
 * Returns the WordClock sample rate class for the given card.
 **/
static int madifx_get_wc_sample_rate(struct hdspm *hdspm)
{
	int status;

	switch (hdspm->io_type) {
	case RayDAT:
	case AIO:
		status = madifx_read(hdspm, HDSPM_RD_STATUS_1);
		return (status >> 16) & 0xF;
		break;
	default:
		break;
	}


	return 0;
}


/**
 * Returns the TCO sample rate class for the given card.
 **/
static int madifx_get_tco_sample_rate(struct hdspm *hdspm)
{
	int status;

	if (hdspm->tco) {
		switch (hdspm->io_type) {
		case RayDAT:
		case AIO:
			status = madifx_read(hdspm, HDSPM_RD_STATUS_1);
			return (status >> 20) & 0xF;
			break;
		default:
			break;
		}
	}

	return 0;
}


/**
 * Returns the SYNC_IN sample rate class for the given card.
 **/
static int madifx_get_sync_in_sample_rate(struct hdspm *hdspm)
{
	int status;

	if (hdspm->tco) {
		switch (hdspm->io_type) {
		case RayDAT:
		case AIO:
			status = madifx_read(hdspm, HDSPM_RD_STATUS_2);
			return (status >> 12) & 0xF;
			break;
		default:
			break;
		}
	}

	return 0;
}


/**
 * Returns the sample rate class for input source <idx> for
 * 'new style' cards like the AIO and RayDAT.
 **/
static int madifx_get_s1_sample_rate(struct hdspm *hdspm, unsigned int idx)
{
	int status = madifx_read(hdspm, HDSPM_RD_STATUS_2);

	return (status >> (idx*4)) & 0xF;
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
	case RayDAT:
		switch (kcontrol->private_value) {
		case 0:
			ucontrol->value.enumerated.item[0] =
				madifx_get_wc_sample_rate(hdspm);
			break;
		case 7:
			ucontrol->value.enumerated.item[0] =
				madifx_get_tco_sample_rate(hdspm);
			break;
		case 8:
			ucontrol->value.enumerated.item[0] =
				madifx_get_sync_in_sample_rate(hdspm);
			break;
		default:
			ucontrol->value.enumerated.item[0] =
				madifx_get_s1_sample_rate(hdspm,
						kcontrol->private_value-1);
		}
		break;

	case AIO:
		switch (kcontrol->private_value) {
		case 0: /* WC */
			ucontrol->value.enumerated.item[0] =
				madifx_get_wc_sample_rate(hdspm);
			break;
		case 4: /* TCO */
			ucontrol->value.enumerated.item[0] =
				madifx_get_tco_sample_rate(hdspm);
			break;
		case 5: /* SYNC_IN */
			ucontrol->value.enumerated.item[0] =
				madifx_get_sync_in_sample_rate(hdspm);
			break;
		default:
			ucontrol->value.enumerated.item[0] =
				madifx_get_s1_sample_rate(hdspm,
						ucontrol->id.index-1);
		}
		break;

	case AES32:

		switch (kcontrol->private_value) {
		case 0: /* WC */
			ucontrol->value.enumerated.item[0] =
				madifx_get_wc_sample_rate(hdspm);
			break;
		case 9: /* TCO */
			ucontrol->value.enumerated.item[0] =
				madifx_get_tco_sample_rate(hdspm);
			break;
		case 10: /* SYNC_IN */
			ucontrol->value.enumerated.item[0] =
				madifx_get_sync_in_sample_rate(hdspm);
			break;
		default: /* AES1 to AES8 */
			ucontrol->value.enumerated.item[0] =
				madifx_get_s1_sample_rate(hdspm,
						kcontrol->private_value-1);
			break;
		}
		break;

	case MADI:
	case MADIface:
		{
			int rate = madifx_external_sample_rate(hdspm);
			int i, selected_rate = 0;
			for (i = 1; i < 10; i++)
				if (HDSPM_bit2freq(i) == rate) {
					selected_rate = i;
					break;
				}
			ucontrol->value.enumerated.item[0] = selected_rate;
		}
		break;

	default:
		break;
	}

	return 0;
}


#define HDSPM_SYSTEM_CLOCK_MODE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_system_clock_mode, \
	.get = snd_madifx_get_system_clock_mode, \
	.put = snd_madifx_put_system_clock_mode, \
}


/**
 * Returns the system clock mode for the given card.
 * @returns 0 - master, 1 - slave
 **/
static int madifx_system_clock_mode(struct hdspm *hdspm)
{
	u32 status;

	status = madifx_read(hdspm, MADIFX_RD_INP_STATUS);
	if ((status & (MADIFX_SelSyncRef0 * 7)) == (MADIFX_SelSyncRef0 * 7)) {
		snd_printk(KERN_INFO "MADFIX: We are clock master\n");
		return 0;
	}

	return 1;
}



/**
 * Sets the system clock mode.
 * @param mode 0 - master, 1 - slave
 **/
static void madifx_set_system_clock_mode(struct hdspm *hdspm, int mode)
{
	switch (hdspm->io_type) {
	case AIO:
	case RayDAT:
		if (0 == mode)
			hdspm->settings_register |= HDSPM_c0Master;
		else
			hdspm->settings_register &= ~HDSPM_c0Master;

		madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);
		break;
	case MADIFX:
		/* FIXME: we don't know yet how to set the clock mode */
		break;

	default:
		if (0 == mode)
			hdspm->control_register |= HDSPM_ClockModeMaster;
		else
			hdspm->control_register &= ~HDSPM_ClockModeMaster;

		madifx_write(hdspm, MADIFX_CONTROL_REG,
				hdspm->control_register);
	}
}


static int snd_madifx_info_system_clock_mode(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "Master", "AutoSync" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);
	return 0;
}

static int snd_madifx_get_system_clock_mode(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = madifx_system_clock_mode(hdspm);
	return 0;
}

static int snd_madifx_put_system_clock_mode(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;

	val = ucontrol->value.enumerated.item[0];
	if (val < 0)
		val = 0;
	else if (val > 1)
		val = 1;

	madifx_set_system_clock_mode(hdspm, val);

	return 0;
}


#define HDSPM_INTERNAL_CLOCK(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_clock_source, \
	.get = snd_madifx_get_clock_source, \
	.put = snd_madifx_put_clock_source \
}


static int madifx_clock_source(struct hdspm * hdspm)
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

static int madifx_set_clock_source(struct hdspm * hdspm, int mode)
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


#define HDSPM_PREF_SYNC_REF(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_pref_sync_ref, \
	.get = snd_madifx_get_pref_sync_ref, \
	.put = snd_madifx_put_pref_sync_ref \
}


/**
 * Returns the current preferred sync reference setting.
 * The semantics of the return value are depending on the
 * card, please see the comments for clarification.
 **/
static int madifx_pref_sync_ref(struct hdspm * hdspm)
{
	switch (hdspm->io_type) {
	case AES32:
		switch (hdspm->control_register & HDSPM_SyncRefMask) {
		case 0: return 0;  /* WC */
		case HDSPM_SyncRef0: return 1; /* AES 1 */
		case HDSPM_SyncRef1: return 2; /* AES 2 */
		case HDSPM_SyncRef1+HDSPM_SyncRef0: return 3; /* AES 3 */
		case HDSPM_SyncRef2: return 4; /* AES 4 */
		case HDSPM_SyncRef2+HDSPM_SyncRef0: return 5; /* AES 5 */
		case HDSPM_SyncRef2+HDSPM_SyncRef1: return 6; /* AES 6 */
		case HDSPM_SyncRef2+HDSPM_SyncRef1+HDSPM_SyncRef0:
						    return 7; /* AES 7 */
		case HDSPM_SyncRef3: return 8; /* AES 8 */
		case HDSPM_SyncRef3+HDSPM_SyncRef0: return 9; /* TCO */
		}
		break;

	case MADI:
	case MADIface:
		if (hdspm->tco) {
			switch (hdspm->control_register & HDSPM_SyncRefMask) {
			case 0: return 0;  /* WC */
			case HDSPM_SyncRef0: return 1;  /* MADI */
			case HDSPM_SyncRef1: return 2;  /* TCO */
			case HDSPM_SyncRef1+HDSPM_SyncRef0:
					     return 3;  /* SYNC_IN */
			}
		} else {
			switch (hdspm->control_register & HDSPM_SyncRefMask) {
			case 0: return 0;  /* WC */
			case HDSPM_SyncRef0: return 1;  /* MADI */
			case HDSPM_SyncRef1+HDSPM_SyncRef0:
					     return 2;  /* SYNC_IN */
			}
		}
		break;

	case RayDAT:
		if (hdspm->tco) {
			switch ((hdspm->settings_register &
				HDSPM_c0_SyncRefMask) / HDSPM_c0_SyncRef0) {
			case 0: return 0;  /* WC */
			case 3: return 1;  /* ADAT 1 */
			case 4: return 2;  /* ADAT 2 */
			case 5: return 3;  /* ADAT 3 */
			case 6: return 4;  /* ADAT 4 */
			case 1: return 5;  /* AES */
			case 2: return 6;  /* SPDIF */
			case 9: return 7;  /* TCO */
			case 10: return 8; /* SYNC_IN */
			}
		} else {
			switch ((hdspm->settings_register &
				HDSPM_c0_SyncRefMask) / HDSPM_c0_SyncRef0) {
			case 0: return 0;  /* WC */
			case 3: return 1;  /* ADAT 1 */
			case 4: return 2;  /* ADAT 2 */
			case 5: return 3;  /* ADAT 3 */
			case 6: return 4;  /* ADAT 4 */
			case 1: return 5;  /* AES */
			case 2: return 6;  /* SPDIF */
			case 10: return 7; /* SYNC_IN */
			}
		}

		break;

	case AIO:
		if (hdspm->tco) {
			switch ((hdspm->settings_register &
				HDSPM_c0_SyncRefMask) / HDSPM_c0_SyncRef0) {
			case 0: return 0;  /* WC */
			case 3: return 1;  /* ADAT */
			case 1: return 2;  /* AES */
			case 2: return 3;  /* SPDIF */
			case 9: return 4;  /* TCO */
			case 10: return 5; /* SYNC_IN */
			}
		} else {
			switch ((hdspm->settings_register &
				HDSPM_c0_SyncRefMask) / HDSPM_c0_SyncRef0) {
			case 0: return 0;  /* WC */
			case 3: return 1;  /* ADAT */
			case 1: return 2;  /* AES */
			case 2: return 3;  /* SPDIF */
			case 10: return 4; /* SYNC_IN */
			}
		}

		break;
	}

	return -1;
}


/**
 * Set the preferred sync reference to <pref>. The semantics
 * of <pref> are depending on the card type, see the comments
 * for clarification.
 **/
static int madifx_set_pref_sync_ref(struct hdspm * hdspm, int pref)
{
	int p = 0;

	switch (hdspm->io_type) {
	case AES32:
		hdspm->control_register &= ~HDSPM_SyncRefMask;
		switch (pref) {
		case 0: /* WC  */
			break;
		case 1: /* AES 1 */
			hdspm->control_register |= HDSPM_SyncRef0;
			break;
		case 2: /* AES 2 */
			hdspm->control_register |= HDSPM_SyncRef1;
			break;
		case 3: /* AES 3 */
			hdspm->control_register |=
				HDSPM_SyncRef1+HDSPM_SyncRef0;
			break;
		case 4: /* AES 4 */
			hdspm->control_register |= HDSPM_SyncRef2;
			break;
		case 5: /* AES 5 */
			hdspm->control_register |=
				HDSPM_SyncRef2+HDSPM_SyncRef0;
			break;
		case 6: /* AES 6 */
			hdspm->control_register |=
				HDSPM_SyncRef2+HDSPM_SyncRef1;
			break;
		case 7: /* AES 7 */
			hdspm->control_register |=
				HDSPM_SyncRef2+HDSPM_SyncRef1+HDSPM_SyncRef0;
			break;
		case 8: /* AES 8 */
			hdspm->control_register |= HDSPM_SyncRef3;
			break;
		case 9: /* TCO */
			hdspm->control_register |=
				HDSPM_SyncRef3+HDSPM_SyncRef0;
			break;
		default:
			return -1;
		}

		break;

	case MADI:
	case MADIface:
		hdspm->control_register &= ~HDSPM_SyncRefMask;
		if (hdspm->tco) {
			switch (pref) {
			case 0: /* WC */
				break;
			case 1: /* MADI */
				hdspm->control_register |= HDSPM_SyncRef0;
				break;
			case 2: /* TCO */
				hdspm->control_register |= HDSPM_SyncRef1;
				break;
			case 3: /* SYNC_IN */
				hdspm->control_register |=
					HDSPM_SyncRef0+HDSPM_SyncRef1;
				break;
			default:
				return -1;
			}
		} else {
			switch (pref) {
			case 0: /* WC */
				break;
			case 1: /* MADI */
				hdspm->control_register |= HDSPM_SyncRef0;
				break;
			case 2: /* SYNC_IN */
				hdspm->control_register |=
					HDSPM_SyncRef0+HDSPM_SyncRef1;
				break;
			default:
				return -1;
			}
		}

		break;

	case RayDAT:
		if (hdspm->tco) {
			switch (pref) {
			case 0: p = 0; break;  /* WC */
			case 1: p = 3; break;  /* ADAT 1 */
			case 2: p = 4; break;  /* ADAT 2 */
			case 3: p = 5; break;  /* ADAT 3 */
			case 4: p = 6; break;  /* ADAT 4 */
			case 5: p = 1; break;  /* AES */
			case 6: p = 2; break;  /* SPDIF */
			case 7: p = 9; break;  /* TCO */
			case 8: p = 10; break; /* SYNC_IN */
			default: return -1;
			}
		} else {
			switch (pref) {
			case 0: p = 0; break;  /* WC */
			case 1: p = 3; break;  /* ADAT 1 */
			case 2: p = 4; break;  /* ADAT 2 */
			case 3: p = 5; break;  /* ADAT 3 */
			case 4: p = 6; break;  /* ADAT 4 */
			case 5: p = 1; break;  /* AES */
			case 6: p = 2; break;  /* SPDIF */
			case 7: p = 10; break; /* SYNC_IN */
			default: return -1;
			}
		}
		break;

	case AIO:
		if (hdspm->tco) {
			switch (pref) {
			case 0: p = 0; break;  /* WC */
			case 1: p = 3; break;  /* ADAT */
			case 2: p = 1; break;  /* AES */
			case 3: p = 2; break;  /* SPDIF */
			case 4: p = 9; break;  /* TCO */
			case 5: p = 10; break; /* SYNC_IN */
			default: return -1;
			}
		} else {
			switch (pref) {
			case 0: p = 0; break;  /* WC */
			case 1: p = 3; break;  /* ADAT */
			case 2: p = 1; break;  /* AES */
			case 3: p = 2; break;  /* SPDIF */
			case 4: p = 10; break; /* SYNC_IN */
			default: return -1;
			}
		}
		break;
	}

	switch (hdspm->io_type) {
	case RayDAT:
	case AIO:
		hdspm->settings_register &= ~HDSPM_c0_SyncRefMask;
		hdspm->settings_register |= HDSPM_c0_SyncRef0 * p;
		madifx_write(hdspm, MADIFX_SETTINGS_REG, hdspm->settings_register);
		break;

	case MADI:
	case MADIface:
	case AES32:
		madifx_write(hdspm, MADIFX_CONTROL_REG,
				hdspm->control_register);
	}

	return 0;
}


static int snd_madifx_info_pref_sync_ref(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = hdspm->texts_autosync_items;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			hdspm->texts_autosync[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_pref_sync_ref(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int psf = madifx_pref_sync_ref(hdspm);

	if (psf >= 0) {
		ucontrol->value.enumerated.item[0] = psf;
		return 0;
	}

	return -1;
}

static int snd_madifx_put_pref_sync_ref(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int val, change = 0;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;

	val = ucontrol->value.enumerated.item[0];

	if (val < 0)
		val = 0;
	else if (val >= hdspm->texts_autosync_items)
		val = hdspm->texts_autosync_items-1;

	spin_lock_irq(&hdspm->lock);
	if (val != madifx_pref_sync_ref(hdspm))
		change = (0 == madifx_set_pref_sync_ref(hdspm, val)) ? 1 : 0;

	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_AUTOSYNC_REF(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READ, \
	.info = snd_madifx_info_autosync_ref, \
	.get = snd_madifx_get_autosync_ref, \
}

static int madifx_autosync_ref(struct hdspm *hdspm)
{
	if (AES32 == hdspm->io_type) {
		unsigned int status = madifx_read(hdspm, HDSPM_statusRegister);
		unsigned int syncref =
			(status >> HDSPM_AES32_syncref_bit) & 0xF;
		if (syncref == 0)
			return HDSPM_AES32_AUTOSYNC_FROM_WORD;
		if (syncref <= 8)
			return syncref;
		return HDSPM_AES32_AUTOSYNC_FROM_NONE;
	} else if (MADI == hdspm->io_type) {
		/* This looks at the autosync selected sync reference */
		unsigned int status2 = madifx_read(hdspm, HDSPM_statusRegister2);

		switch (status2 & HDSPM_SelSyncRefMask) {
		case HDSPM_SelSyncRef_WORD:
			return HDSPM_AUTOSYNC_FROM_WORD;
		case HDSPM_SelSyncRef_MADI:
			return HDSPM_AUTOSYNC_FROM_MADI;
		case HDSPM_SelSyncRef_TCO:
			return HDSPM_AUTOSYNC_FROM_TCO;
		case HDSPM_SelSyncRef_SyncIn:
			return HDSPM_AUTOSYNC_FROM_SYNC_IN;
		case HDSPM_SelSyncRef_NVALID:
			return HDSPM_AUTOSYNC_FROM_NONE;
		default:
			return 0;
		}

	}
	return 0;
}


static int snd_madifx_info_autosync_ref(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (AES32 == hdspm->io_type) {
		static char *texts[] = { "WordClock", "AES1", "AES2", "AES3",
			"AES4",	"AES5", "AES6", "AES7", "AES8", "None"};

		uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
		uinfo->count = 1;
		uinfo->value.enumerated.items = 10;
		if (uinfo->value.enumerated.item >=
		    uinfo->value.enumerated.items)
			uinfo->value.enumerated.item =
				uinfo->value.enumerated.items - 1;
		strcpy(uinfo->value.enumerated.name,
				texts[uinfo->value.enumerated.item]);
	} else if (MADI == hdspm->io_type) {
		static char *texts[] = {"Word Clock", "MADI", "TCO",
			"Sync In", "None" };

		uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
		uinfo->count = 1;
		uinfo->value.enumerated.items = 5;
		if (uinfo->value.enumerated.item >=
				uinfo->value.enumerated.items)
			uinfo->value.enumerated.item =
				uinfo->value.enumerated.items - 1;
		strcpy(uinfo->value.enumerated.name,
				texts[uinfo->value.enumerated.item]);
	}
	return 0;
}

static int snd_madifx_get_autosync_ref(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = madifx_autosync_ref(hdspm);
	return 0;
}


#define HDSPM_LINE_OUT(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_line_out, \
	.get = snd_madifx_get_line_out, \
	.put = snd_madifx_put_line_out \
}

static int madifx_line_out(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_LineOut) ? 1 : 0;
}


static int madifx_set_line_output(struct hdspm * hdspm, int out)
{
	if (out)
		hdspm->control_register |= HDSPM_LineOut;
	else
		hdspm->control_register &= ~HDSPM_LineOut;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_line_out		snd_ctl_boolean_mono_info

static int snd_madifx_get_line_out(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] = madifx_line_out(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_line_out(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_line_out(hdspm);
	madifx_set_line_output(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_TX_64(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_tx_64, \
	.get = snd_madifx_get_tx_64, \
	.put = snd_madifx_put_tx_64 \
}

static int madifx_tx_64(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_TX_64ch) ? 1 : 0;
}

static int madifx_set_tx_64(struct hdspm * hdspm, int out)
{
	if (out)
		hdspm->control_register |= HDSPM_TX_64ch;
	else
		hdspm->control_register &= ~HDSPM_TX_64ch;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_tx_64		snd_ctl_boolean_mono_info

static int snd_madifx_get_tx_64(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] = madifx_tx_64(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_tx_64(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_tx_64(hdspm);
	madifx_set_tx_64(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_C_TMS(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_c_tms, \
	.get = snd_madifx_get_c_tms, \
	.put = snd_madifx_put_c_tms \
}

static int madifx_c_tms(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_clr_tms) ? 1 : 0;
}

static int madifx_set_c_tms(struct hdspm * hdspm, int out)
{
	if (out)
		hdspm->control_register |= HDSPM_clr_tms;
	else
		hdspm->control_register &= ~HDSPM_clr_tms;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_c_tms		snd_ctl_boolean_mono_info

static int snd_madifx_get_c_tms(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] = madifx_c_tms(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_c_tms(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_c_tms(hdspm);
	madifx_set_c_tms(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_SAFE_MODE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_safe_mode, \
	.get = snd_madifx_get_safe_mode, \
	.put = snd_madifx_put_safe_mode \
}

static int madifx_safe_mode(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_AutoInp) ? 1 : 0;
}

static int madifx_set_safe_mode(struct hdspm * hdspm, int out)
{
	if (out)
		hdspm->control_register |= HDSPM_AutoInp;
	else
		hdspm->control_register &= ~HDSPM_AutoInp;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_safe_mode	snd_ctl_boolean_mono_info

static int snd_madifx_get_safe_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.integer.value[0] = madifx_safe_mode(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_safe_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_safe_mode(hdspm);
	madifx_set_safe_mode(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_EMPHASIS(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_emphasis, \
	.get = snd_madifx_get_emphasis, \
	.put = snd_madifx_put_emphasis \
}

static int madifx_emphasis(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_Emphasis) ? 1 : 0;
}

static int madifx_set_emphasis(struct hdspm * hdspm, int emp)
{
	if (emp)
		hdspm->control_register |= HDSPM_Emphasis;
	else
		hdspm->control_register &= ~HDSPM_Emphasis;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_emphasis		snd_ctl_boolean_mono_info

static int snd_madifx_get_emphasis(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_emphasis(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_emphasis(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_emphasis(hdspm);
	madifx_set_emphasis(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_DOLBY(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_dolby, \
	.get = snd_madifx_get_dolby, \
	.put = snd_madifx_put_dolby \
}

static int madifx_dolby(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_Dolby) ? 1 : 0;
}

static int madifx_set_dolby(struct hdspm * hdspm, int dol)
{
	if (dol)
		hdspm->control_register |= HDSPM_Dolby;
	else
		hdspm->control_register &= ~HDSPM_Dolby;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_dolby		snd_ctl_boolean_mono_info

static int snd_madifx_get_dolby(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_dolby(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_dolby(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_dolby(hdspm);
	madifx_set_dolby(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_PROFESSIONAL(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_professional, \
	.get = snd_madifx_get_professional, \
	.put = snd_madifx_put_professional \
}

static int madifx_professional(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_Professional) ? 1 : 0;
}

static int madifx_set_professional(struct hdspm * hdspm, int dol)
{
	if (dol)
		hdspm->control_register |= HDSPM_Professional;
	else
		hdspm->control_register &= ~HDSPM_Professional;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

#define snd_madifx_info_professional	snd_ctl_boolean_mono_info

static int snd_madifx_get_professional(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_professional(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_professional(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_professional(hdspm);
	madifx_set_professional(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}

#define HDSPM_INPUT_SELECT(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_input_select, \
	.get = snd_madifx_get_input_select, \
	.put = snd_madifx_put_input_select \
}

static int madifx_input_select(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_InputSelect0) ? 1 : 0;
}

static int madifx_set_input_select(struct hdspm * hdspm, int out)
{
	if (out)
		hdspm->control_register |= HDSPM_InputSelect0;
	else
		hdspm->control_register &= ~HDSPM_InputSelect0;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

static int snd_madifx_info_input_select(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "optical", "coaxial" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_input_select(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_input_select(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_input_select(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_input_select(hdspm);
	madifx_set_input_select(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_DS_WIRE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_ds_wire, \
	.get = snd_madifx_get_ds_wire, \
	.put = snd_madifx_put_ds_wire \
}

static int madifx_ds_wire(struct hdspm * hdspm)
{
	return (hdspm->control_register & HDSPM_DS_DoubleWire) ? 1 : 0;
}

static int madifx_set_ds_wire(struct hdspm * hdspm, int ds)
{
	if (ds)
		hdspm->control_register |= HDSPM_DS_DoubleWire;
	else
		hdspm->control_register &= ~HDSPM_DS_DoubleWire;
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

static int snd_madifx_info_ds_wire(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "Single", "Double" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_ds_wire(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_ds_wire(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_ds_wire(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	unsigned int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irq(&hdspm->lock);
	change = (int) val != madifx_ds_wire(hdspm);
	madifx_set_ds_wire(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}


#define HDSPM_QS_WIRE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_qs_wire, \
	.get = snd_madifx_get_qs_wire, \
	.put = snd_madifx_put_qs_wire \
}

static int madifx_qs_wire(struct hdspm * hdspm)
{
	if (hdspm->control_register & HDSPM_QS_DoubleWire)
		return 1;
	if (hdspm->control_register & HDSPM_QS_QuadWire)
		return 2;
	return 0;
}

static int madifx_set_qs_wire(struct hdspm * hdspm, int mode)
{
	hdspm->control_register &= ~(HDSPM_QS_DoubleWire | HDSPM_QS_QuadWire);
	switch (mode) {
	case 0:
		break;
	case 1:
		hdspm->control_register |= HDSPM_QS_DoubleWire;
		break;
	case 2:
		hdspm->control_register |= HDSPM_QS_QuadWire;
		break;
	}
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

static int snd_madifx_info_qs_wire(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "Single", "Double", "Quad" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_qs_wire(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_qs_wire(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_qs_wire(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0];
	if (val < 0)
		val = 0;
	if (val > 2)
		val = 2;
	spin_lock_irq(&hdspm->lock);
	change = val != madifx_qs_wire(hdspm);
	madifx_set_qs_wire(hdspm, val);
	spin_unlock_irq(&hdspm->lock);
	return change;
}

#define HDSPM_MADI_SPEEDMODE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_madifx_info_madi_speedmode, \
	.get = snd_madifx_get_madi_speedmode, \
	.put = snd_madifx_put_madi_speedmode \
}

static int madifx_madi_speedmode(struct hdspm *hdspm)
{
	if (hdspm->control_register & HDSPM_QuadSpeed)
		return 2;
	if (hdspm->control_register & HDSPM_DoubleSpeed)
		return 1;
	return 0;
}

static int madifx_set_madi_speedmode(struct hdspm *hdspm, int mode)
{
	hdspm->control_register &= ~(HDSPM_DoubleSpeed | HDSPM_QuadSpeed);
	switch (mode) {
	case 0:
		break;
	case 1:
		hdspm->control_register |= HDSPM_DoubleSpeed;
		break;
	case 2:
		hdspm->control_register |= HDSPM_QuadSpeed;
		break;
	}
	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	return 0;
}

static int snd_madifx_info_madi_speedmode(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "Single", "Double", "Quad" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
		    uinfo->value.enumerated.items - 1;
	strcpy(uinfo->value.enumerated.name,
	       texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_madi_speedmode(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	spin_lock_irq(&hdspm->lock);
	ucontrol->value.enumerated.item[0] = madifx_madi_speedmode(hdspm);
	spin_unlock_irq(&hdspm->lock);
	return 0;
}

static int snd_madifx_put_madi_speedmode(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int change;
	int val;

	if (!snd_madifx_use_is_exclusive(hdspm))
		return -EBUSY;
	val = ucontrol->value.integer.value[0];
	if (val < 0)
		val = 0;
	if (val > 2)
		val = 2;
	spin_lock_irq(&hdspm->lock);
	change = val != madifx_madi_speedmode(hdspm);
	madifx_set_madi_speedmode(hdspm, val);
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

static int madifx_wc_sync_check(struct hdspm *hdspm)
{
	int status, status2;

	switch (hdspm->io_type) {
	case AES32:
		status = madifx_read(hdspm, HDSPM_statusRegister);
		if (status & HDSPM_wcSync)
			return 2;
		else if (status & HDSPM_wcLock)
			return 1;
		return 0;
		break;

	case MADI:
		status2 = madifx_read(hdspm, HDSPM_statusRegister2);
		if (status2 & HDSPM_wcLock) {
			if (status2 & HDSPM_wcSync)
				return 2;
			else
				return 1;
		}
		return 0;
		break;

	case RayDAT:
	case AIO:
		status = madifx_read(hdspm, HDSPM_statusRegister);

		if (status & 0x2000000)
			return 2;
		else if (status & 0x1000000)
			return 1;
		return 0;

		break;

	case MADIface:
		break;
	}


	return 3;
}


static int madifx_madi_sync_check(struct hdspm *hdspm)
{
	int status = madifx_read(hdspm, HDSPM_statusRegister);
	if (status & HDSPM_madiLock) {
		if (status & HDSPM_madiSync)
			return 2;
		else
			return 1;
	}
	return 0;
}


static int madifx_s1_sync_check(struct hdspm *hdspm, int idx)
{
	int status, lock, sync;

	status = madifx_read(hdspm, HDSPM_RD_STATUS_1);

	lock = (status & (0x1<<idx)) ? 1 : 0;
	sync = (status & (0x100<<idx)) ? 1 : 0;

	if (lock && sync)
		return 2;
	else if (lock)
		return 1;
	return 0;
}


static int madifx_sync_in_sync_check(struct hdspm *hdspm)
{
	int status, lock = 0, sync = 0;

	switch (hdspm->io_type) {
	case RayDAT:
	case AIO:
		status = madifx_read(hdspm, HDSPM_RD_STATUS_3);
		lock = (status & 0x400) ? 1 : 0;
		sync = (status & 0x800) ? 1 : 0;
		break;

	case MADI:
		status = madifx_read(hdspm, HDSPM_statusRegister);
		lock = (status & HDSPM_syncInLock) ? 1 : 0;
		sync = (status & HDSPM_syncInSync) ? 1 : 0;
		break;

	case AES32:
		status = madifx_read(hdspm, HDSPM_statusRegister2);
		lock = (status & 0x100000) ? 1 : 0;
		sync = (status & 0x200000) ? 1 : 0;
		break;

	case MADIface:
		break;
	}

	if (lock && sync)
		return 2;
	else if (lock)
		return 1;

	return 0;
}

static int madifx_aes_sync_check(struct hdspm *hdspm, int idx)
{
	int status2, lock, sync;
	status2 = madifx_read(hdspm, HDSPM_statusRegister2);

	lock = (status2 & (0x0080 >> idx)) ? 1 : 0;
	sync = (status2 & (0x8000 >> idx)) ? 1 : 0;

	if (sync)
		return 2;
	else if (lock)
		return 1;
	return 0;
}


static int madifx_tco_sync_check(struct hdspm *hdspm)
{
	int status;

	if (hdspm->tco) {
		switch (hdspm->io_type) {
		case MADI:
		case AES32:
			status = madifx_read(hdspm, HDSPM_statusRegister);
			if (status & HDSPM_tcoLock) {
				if (status & HDSPM_tcoSync)
					return 2;
				else
					return 1;
			}
			return 0;

			break;

		case RayDAT:
		case AIO:
			status = madifx_read(hdspm, HDSPM_RD_STATUS_1);

			if (status & 0x8000000)
				return 2; /* Sync */
			if (status & 0x4000000)
				return 1; /* Lock */
			return 0; /* No signal */
			break;

		default:
			break;
		}
	}

	return 3; /* N/A */
}


static int snd_madifx_get_sync_check(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);
	int val = -1;

	switch (hdspm->io_type) {
	case RayDAT:
		switch (kcontrol->private_value) {
		case 0: /* WC */
			val = madifx_wc_sync_check(hdspm); break;
		case 7: /* TCO */
			val = madifx_tco_sync_check(hdspm); break;
		case 8: /* SYNC IN */
			val = madifx_sync_in_sync_check(hdspm); break;
		default:
			val = madifx_s1_sync_check(hdspm,
					kcontrol->private_value-1);
		}
		break;

	case AIO:
		switch (kcontrol->private_value) {
		case 0: /* WC */
			val = madifx_wc_sync_check(hdspm); break;
		case 4: /* TCO */
			val = madifx_tco_sync_check(hdspm); break;
		case 5: /* SYNC IN */
			val = madifx_sync_in_sync_check(hdspm); break;
		default:
			val = madifx_s1_sync_check(hdspm, ucontrol->id.index-1);
		}
		break;

	case MADI:
		switch (kcontrol->private_value) {
		case 0: /* WC */
			val = madifx_wc_sync_check(hdspm); break;
		case 1: /* MADI */
			val = madifx_madi_sync_check(hdspm); break;
		case 2: /* TCO */
			val = madifx_tco_sync_check(hdspm); break;
		case 3: /* SYNC_IN */
			val = madifx_sync_in_sync_check(hdspm); break;
		}
		break;

	case MADIface:
		val = madifx_madi_sync_check(hdspm); /* MADI */
		break;

	case AES32:
		switch (kcontrol->private_value) {
		case 0: /* WC */
			val = madifx_wc_sync_check(hdspm); break;
		case 9: /* TCO */
			val = madifx_tco_sync_check(hdspm); break;
		case 10 /* SYNC IN */:
			val = madifx_sync_in_sync_check(hdspm); break;
		default: /* AES1 to AES8 */
			 val = madifx_aes_sync_check(hdspm,
					 kcontrol->private_value-1);
		}
		break;

	}

	if (-1 == val)
		val = 3;

	ucontrol->value.enumerated.item[0] = val;
	return 0;
}



/**
 * TCO controls
 **/
static void madifx_tco_write(struct hdspm *hdspm)
{
	unsigned int tc[4] = { 0, 0, 0, 0};

	switch (hdspm->tco->input) {
	case 0:
		tc[2] |= HDSPM_TCO2_set_input_MSB;
		break;
	case 1:
		tc[2] |= HDSPM_TCO2_set_input_LSB;
		break;
	default:
		break;
	}

	switch (hdspm->tco->framerate) {
	case 1:
		tc[1] |= HDSPM_TCO1_LTC_Format_LSB;
		break;
	case 2:
		tc[1] |= HDSPM_TCO1_LTC_Format_MSB;
		break;
	case 3:
		tc[1] |= HDSPM_TCO1_LTC_Format_MSB +
			HDSPM_TCO1_set_drop_frame_flag;
		break;
	case 4:
		tc[1] |= HDSPM_TCO1_LTC_Format_LSB +
			HDSPM_TCO1_LTC_Format_MSB;
		break;
	case 5:
		tc[1] |= HDSPM_TCO1_LTC_Format_LSB +
			HDSPM_TCO1_LTC_Format_MSB +
			HDSPM_TCO1_set_drop_frame_flag;
		break;
	default:
		break;
	}

	switch (hdspm->tco->wordclock) {
	case 1:
		tc[2] |= HDSPM_TCO2_WCK_IO_ratio_LSB;
		break;
	case 2:
		tc[2] |= HDSPM_TCO2_WCK_IO_ratio_MSB;
		break;
	default:
		break;
	}

	switch (hdspm->tco->samplerate) {
	case 1:
		tc[2] |= HDSPM_TCO2_set_freq;
		break;
	case 2:
		tc[2] |= HDSPM_TCO2_set_freq_from_app;
		break;
	default:
		break;
	}

	switch (hdspm->tco->pull) {
	case 1:
		tc[2] |= HDSPM_TCO2_set_pull_up;
		break;
	case 2:
		tc[2] |= HDSPM_TCO2_set_pull_down;
		break;
	case 3:
		tc[2] |= HDSPM_TCO2_set_pull_up + HDSPM_TCO2_set_01_4;
		break;
	case 4:
		tc[2] |= HDSPM_TCO2_set_pull_down + HDSPM_TCO2_set_01_4;
		break;
	default:
		break;
	}

	if (1 == hdspm->tco->term) {
		tc[2] |= HDSPM_TCO2_set_term_75R;
	}

	madifx_write(hdspm, HDSPM_WR_TCO, tc[0]);
	madifx_write(hdspm, HDSPM_WR_TCO+4, tc[1]);
	madifx_write(hdspm, HDSPM_WR_TCO+8, tc[2]);
	madifx_write(hdspm, HDSPM_WR_TCO+12, tc[3]);
}


#define HDSPM_TCO_SAMPLE_RATE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_sample_rate, \
	.get = snd_madifx_get_tco_sample_rate, \
	.put = snd_madifx_put_tco_sample_rate \
}

static int snd_madifx_info_tco_sample_rate(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "44.1 kHz", "48 kHz" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_tco_sample_rate(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->samplerate;

	return 0;
}

static int snd_madifx_put_tco_sample_rate(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->samplerate != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->samplerate = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}


#define HDSPM_TCO_PULL(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_pull, \
	.get = snd_madifx_get_tco_pull, \
	.put = snd_madifx_put_tco_pull \
}

static int snd_madifx_info_tco_pull(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "0", "+ 0.1 %", "- 0.1 %", "+ 4 %", "- 4 %" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 5;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_tco_pull(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->pull;

	return 0;
}

static int snd_madifx_put_tco_pull(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->pull != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->pull = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}

#define HDSPM_TCO_WCK_CONVERSION(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_wck_conversion, \
	.get = snd_madifx_get_tco_wck_conversion, \
	.put = snd_madifx_put_tco_wck_conversion \
}

static int snd_madifx_info_tco_wck_conversion(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "1:1", "44.1 -> 48", "48 -> 44.1" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_tco_wck_conversion(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->wordclock;

	return 0;
}

static int snd_madifx_put_tco_wck_conversion(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->wordclock != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->wordclock = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}


#define HDSPM_TCO_FRAME_RATE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_frame_rate, \
	.get = snd_madifx_get_tco_frame_rate, \
	.put = snd_madifx_put_tco_frame_rate \
}

static int snd_madifx_info_tco_frame_rate(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "24 fps", "25 fps", "29.97fps",
		"29.97 dfps", "30 fps", "30 dfps" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 6;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_tco_frame_rate(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->framerate;

	return 0;
}

static int snd_madifx_put_tco_frame_rate(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->framerate != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->framerate = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}


#define HDSPM_TCO_SYNC_SOURCE(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_sync_source, \
	.get = snd_madifx_get_tco_sync_source, \
	.put = snd_madifx_put_tco_sync_source \
}

static int snd_madifx_info_tco_sync_source(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[] = { "LTC", "Video", "WCK" };
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;

	if (uinfo->value.enumerated.item >= uinfo->value.enumerated.items)
		uinfo->value.enumerated.item =
			uinfo->value.enumerated.items - 1;

	strcpy(uinfo->value.enumerated.name,
			texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_madifx_get_tco_sync_source(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->input;

	return 0;
}

static int snd_madifx_put_tco_sync_source(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->input != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->input = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}


#define HDSPM_TCO_WORD_TERM(xname, xindex) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |\
			SNDRV_CTL_ELEM_ACCESS_VOLATILE, \
	.info = snd_madifx_info_tco_word_term, \
	.get = snd_madifx_get_tco_word_term, \
	.put = snd_madifx_put_tco_word_term \
}

static int snd_madifx_info_tco_word_term(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}


static int snd_madifx_get_tco_word_term(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.enumerated.item[0] = hdspm->tco->term;

	return 0;
}


static int snd_madifx_put_tco_word_term(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct hdspm *hdspm = snd_kcontrol_chip(kcontrol);

	if (hdspm->tco->term != ucontrol->value.enumerated.item[0]) {
		hdspm->tco->term = ucontrol->value.enumerated.item[0];

		madifx_tco_write(hdspm);

		return 1;
	}

	return 0;
}




static struct snd_kcontrol_new snd_madifx_controls_madi[] = {
	HDSPM_MIXER("Mixer", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYSTEM_CLOCK_MODE("System Clock Mode", 0),
	HDSPM_PREF_SYNC_REF("Preferred Sync Reference", 0),
	HDSPM_AUTOSYNC_REF("AutoSync Reference", 0),
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("External Rate", 0),
	HDSPM_SYNC_CHECK("WC SyncCheck", 0),
	HDSPM_SYNC_CHECK("MADI SyncCheck", 1),
	HDSPM_SYNC_CHECK("TCO SyncCheck", 2),
	HDSPM_SYNC_CHECK("SYNC IN SyncCheck", 3),
	HDSPM_LINE_OUT("Line Out", 0),
	HDSPM_TX_64("TX 64 channels mode", 0),
	HDSPM_C_TMS("Clear Track Marker", 0),
	HDSPM_SAFE_MODE("Safe Mode", 0),
	HDSPM_INPUT_SELECT("Input Select", 0),
	HDSPM_MADI_SPEEDMODE("MADI Speed Mode", 0)
};


static struct snd_kcontrol_new snd_madifx_controls_madiface[] = {
	HDSPM_MIXER("Mixer", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYSTEM_CLOCK_MODE("System Clock Mode", 0),
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("External Rate", 0),
	HDSPM_SYNC_CHECK("MADI SyncCheck", 0),
	HDSPM_TX_64("TX 64 channels mode", 0),
	HDSPM_C_TMS("Clear Track Marker", 0),
	HDSPM_SAFE_MODE("Safe Mode", 0),
	HDSPM_MADI_SPEEDMODE("MADI Speed Mode", 0)
};

static struct snd_kcontrol_new snd_madifx_controls_aio[] = {
	HDSPM_MIXER("Mixer", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYSTEM_CLOCK_MODE("System Clock Mode", 0),
	HDSPM_PREF_SYNC_REF("Preferred Sync Reference", 0),
	HDSPM_AUTOSYNC_REF("AutoSync Reference", 0),
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("External Rate", 0),
	HDSPM_SYNC_CHECK("WC SyncCheck", 0),
	HDSPM_SYNC_CHECK("AES SyncCheck", 1),
	HDSPM_SYNC_CHECK("SPDIF SyncCheck", 2),
	HDSPM_SYNC_CHECK("ADAT SyncCheck", 3),
	HDSPM_SYNC_CHECK("TCO SyncCheck", 4),
	HDSPM_SYNC_CHECK("SYNC IN SyncCheck", 5),
	HDSPM_AUTOSYNC_SAMPLE_RATE("WC Frequency", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES Frequency", 1),
	HDSPM_AUTOSYNC_SAMPLE_RATE("SPDIF Frequency", 2),
	HDSPM_AUTOSYNC_SAMPLE_RATE("ADAT Frequency", 3),
	HDSPM_AUTOSYNC_SAMPLE_RATE("TCO Frequency", 4),
	HDSPM_AUTOSYNC_SAMPLE_RATE("SYNC IN Frequency", 5)

		/*
		   HDSPM_INPUT_SELECT("Input Select", 0),
		   HDSPM_SPDIF_OPTICAL("SPDIF Out Optical", 0),
		   HDSPM_PROFESSIONAL("SPDIF Out Professional", 0);
		   HDSPM_SPDIF_IN("SPDIF In", 0);
		   HDSPM_BREAKOUT_CABLE("Breakout Cable", 0);
		   HDSPM_INPUT_LEVEL("Input Level", 0);
		   HDSPM_OUTPUT_LEVEL("Output Level", 0);
		   HDSPM_PHONES("Phones", 0);
		   */
};

static struct snd_kcontrol_new snd_madifx_controls_raydat[] = {
	HDSPM_MIXER("Mixer", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYSTEM_CLOCK_MODE("Clock Mode", 0),
	HDSPM_PREF_SYNC_REF("Pref Sync Ref", 0),
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_SYNC_CHECK("WC SyncCheck", 0),
	HDSPM_SYNC_CHECK("AES SyncCheck", 1),
	HDSPM_SYNC_CHECK("SPDIF SyncCheck", 2),
	HDSPM_SYNC_CHECK("ADAT1 SyncCheck", 3),
	HDSPM_SYNC_CHECK("ADAT2 SyncCheck", 4),
	HDSPM_SYNC_CHECK("ADAT3 SyncCheck", 5),
	HDSPM_SYNC_CHECK("ADAT4 SyncCheck", 6),
	HDSPM_SYNC_CHECK("TCO SyncCheck", 7),
	HDSPM_SYNC_CHECK("SYNC IN SyncCheck", 8),
	HDSPM_AUTOSYNC_SAMPLE_RATE("WC Frequency", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES Frequency", 1),
	HDSPM_AUTOSYNC_SAMPLE_RATE("SPDIF Frequency", 2),
	HDSPM_AUTOSYNC_SAMPLE_RATE("ADAT1 Frequency", 3),
	HDSPM_AUTOSYNC_SAMPLE_RATE("ADAT2 Frequency", 4),
	HDSPM_AUTOSYNC_SAMPLE_RATE("ADAT3 Frequency", 5),
	HDSPM_AUTOSYNC_SAMPLE_RATE("ADAT4 Frequency", 6),
	HDSPM_AUTOSYNC_SAMPLE_RATE("TCO Frequency", 7),
	HDSPM_AUTOSYNC_SAMPLE_RATE("SYNC IN Frequency", 8)
};

static struct snd_kcontrol_new snd_madifx_controls_aes32[] = {
	HDSPM_MIXER("Mixer", 0),
	HDSPM_INTERNAL_CLOCK("Internal Clock", 0),
	HDSPM_SYSTEM_CLOCK_MODE("System Clock Mode", 0),
	HDSPM_PREF_SYNC_REF("Preferred Sync Reference", 0),
	HDSPM_AUTOSYNC_REF("AutoSync Reference", 0),
	HDSPM_SYSTEM_SAMPLE_RATE("System Sample Rate", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("External Rate", 0),
	HDSPM_SYNC_CHECK("WC Sync Check", 0),
	HDSPM_SYNC_CHECK("AES1 Sync Check", 1),
	HDSPM_SYNC_CHECK("AES2 Sync Check", 2),
	HDSPM_SYNC_CHECK("AES3 Sync Check", 3),
	HDSPM_SYNC_CHECK("AES4 Sync Check", 4),
	HDSPM_SYNC_CHECK("AES5 Sync Check", 5),
	HDSPM_SYNC_CHECK("AES6 Sync Check", 6),
	HDSPM_SYNC_CHECK("AES7 Sync Check", 7),
	HDSPM_SYNC_CHECK("AES8 Sync Check", 8),
	HDSPM_SYNC_CHECK("TCO Sync Check", 9),
	HDSPM_SYNC_CHECK("SYNC IN Sync Check", 10),
	HDSPM_AUTOSYNC_SAMPLE_RATE("WC Frequency", 0),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES1 Frequency", 1),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES2 Frequency", 2),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES3 Frequency", 3),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES4 Frequency", 4),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES5 Frequency", 5),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES6 Frequency", 6),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES7 Frequency", 7),
	HDSPM_AUTOSYNC_SAMPLE_RATE("AES8 Frequency", 8),
	HDSPM_AUTOSYNC_SAMPLE_RATE("TCO Frequency", 9),
	HDSPM_AUTOSYNC_SAMPLE_RATE("SYNC IN Frequency", 10),
	HDSPM_LINE_OUT("Line Out", 0),
	HDSPM_EMPHASIS("Emphasis", 0),
	HDSPM_DOLBY("Non Audio", 0),
	HDSPM_PROFESSIONAL("Professional", 0),
	HDSPM_C_TMS("Clear Track Marker", 0),
	HDSPM_DS_WIRE("Double Speed Wire Mode", 0),
	HDSPM_QS_WIRE("Quad Speed Wire Mode", 0),
};



/* Control elements for the optional TCO module */
static struct snd_kcontrol_new snd_madifx_controls_tco[] = {
	HDSPM_TCO_SAMPLE_RATE("TCO Sample Rate", 0),
	HDSPM_TCO_PULL("TCO Pull", 0),
	HDSPM_TCO_WCK_CONVERSION("TCO WCK Conversion", 0),
	HDSPM_TCO_FRAME_RATE("TCO Frame Rate", 0),
	HDSPM_TCO_SYNC_SOURCE("TCO Sync Source", 0),
	HDSPM_TCO_WORD_TERM("TCO Word Term", 0)
};


static struct snd_kcontrol_new snd_madifx_playback_mixer = HDSPM_PLAYBACK_MIXER;


static int madifx_update_simple_mixer_controls(struct hdspm * hdspm)
{
	int i;
    snd_printk(KERN_WARNING "MADIFX: "
            "updating broken mixer\n");

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
	case MADI:
	case MADIFX:
		list = snd_madifx_controls_madi;
		limit = ARRAY_SIZE(snd_madifx_controls_madi);
		break;
	case MADIface:
		list = snd_madifx_controls_madiface;
		limit = ARRAY_SIZE(snd_madifx_controls_madiface);
		break;
	case AIO:
		list = snd_madifx_controls_aio;
		limit = ARRAY_SIZE(snd_madifx_controls_aio);
		break;
	case RayDAT:
		list = snd_madifx_controls_raydat;
		limit = ARRAY_SIZE(snd_madifx_controls_raydat);
		break;
	case AES32:
		list = snd_madifx_controls_aes32;
		limit = ARRAY_SIZE(snd_madifx_controls_aes32);
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


	if (hdspm->tco) {
		/* add tco control elements */
		list = snd_madifx_controls_tco;
		limit = ARRAY_SIZE(snd_madifx_controls_tco);
		for (idx = 0; idx < limit; idx++) {
			err = snd_ctl_add(card,
					snd_ctl_new1(&list[idx], hdspm));
			if (err < 0)
				return err;
		}
	}

	return 0;
}

/*------------------------------------------------------------
   /proc interface
 ------------------------------------------------------------*/

static void
snd_madifx_proc_read_madi(struct snd_info_entry * entry,
			 struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	unsigned int status, status2, control, freq;

	char *pref_sync_ref;
	char *autosync_ref;
	char *system_clock_mode;
	char *insel;
	int x, x2;

	/* TCO stuff */
	int a, ltc, frames, seconds, minutes, hours;
	unsigned int period;
	u64 freq_const = 0;
	u32 rate;

	status = madifx_read(hdspm, HDSPM_statusRegister);
	status2 = madifx_read(hdspm, HDSPM_statusRegister2);
	control = hdspm->control_register;
	freq = madifx_read(hdspm, HDSPM_timecodeRegister);

	snd_iprintf(buffer, "%s (Card #%d) Rev.%x Status2first3bits: %x\n",
			hdspm->card_name, hdspm->card->number + 1,
			hdspm->firmware_rev,
			(status2 & HDSPM_version0) |
			(status2 & HDSPM_version1) | (status2 &
				HDSPM_version2));

	snd_iprintf(buffer, "HW Serial: 0x%06x%06x\n",
			(madifx_read(hdspm, HDSPM_midiStatusIn1)>>8) & 0xFFFFFF,
			hdspm->serial);

	snd_iprintf(buffer, "IRQ: %d Registers bus: 0x%lx VM: 0x%lx\n",
			hdspm->irq, hdspm->port, (unsigned long)hdspm->iobase);

	snd_iprintf(buffer, "--- System ---\n");

	snd_iprintf(buffer,
		"IRQ Pending: Audio=%d, MIDI0=%d, MIDI1=%d, IRQcount=%d\n",
		status & HDSPM_audioIRQPending,
		(status & HDSPM_midi0IRQPending) ? 1 : 0,
		(status & HDSPM_midi1IRQPending) ? 1 : 0,
		hdspm->irq_count);
	snd_iprintf(buffer,
		"HW pointer: id = %d, rawptr = %d (%d->%d) "
		"estimated= %ld (bytes)\n",
		((status & HDSPM_BufferID) ? 1 : 0),
		(status & HDSPM_BufferPositionMask),
		(status & HDSPM_BufferPositionMask) %
		(2 * (int)hdspm->period_bytes),
		((status & HDSPM_BufferPositionMask) - 64) %
		(2 * (int)hdspm->period_bytes),
		(long) madifx_hw_pointer(hdspm) * 4);

	snd_iprintf(buffer,
		"MIDI FIFO: Out1=0x%x, Out2=0x%x, In1=0x%x, In2=0x%x \n",
		madifx_read(hdspm, HDSPM_midiStatusOut0) & 0xFF,
		madifx_read(hdspm, HDSPM_midiStatusOut1) & 0xFF,
		madifx_read(hdspm, HDSPM_midiStatusIn0) & 0xFF,
		madifx_read(hdspm, HDSPM_midiStatusIn1) & 0xFF);
	snd_iprintf(buffer,
		"MIDIoverMADI FIFO: In=0x%x, Out=0x%x \n",
		madifx_read(hdspm, HDSPM_midiStatusIn2) & 0xFF,
		madifx_read(hdspm, HDSPM_midiStatusOut2) & 0xFF);
	snd_iprintf(buffer,
		"Register: ctrl1=0x%x, ctrl2=0x%x, status1=0x%x, "
		"status2=0x%x\n",
		hdspm->control_register, hdspm->control2_register,
		status, status2);
	if (status & HDSPM_tco_detect) {
		snd_iprintf(buffer, "TCO module detected.\n");
		a = madifx_read(hdspm, HDSPM_RD_TCO+4);
		if (a & HDSPM_TCO1_LTC_Input_valid) {
			snd_iprintf(buffer, "  LTC valid, ");
			switch (a & (HDSPM_TCO1_LTC_Format_LSB |
						HDSPM_TCO1_LTC_Format_MSB)) {
			case 0:
				snd_iprintf(buffer, "24 fps, ");
				break;
			case HDSPM_TCO1_LTC_Format_LSB:
				snd_iprintf(buffer, "25 fps, ");
				break;
			case HDSPM_TCO1_LTC_Format_MSB:
				snd_iprintf(buffer, "29.97 fps, ");
				break;
			default:
				snd_iprintf(buffer, "30 fps, ");
				break;
			}
			if (a & HDSPM_TCO1_set_drop_frame_flag) {
				snd_iprintf(buffer, "drop frame\n");
			} else {
				snd_iprintf(buffer, "full frame\n");
			}
		} else {
			snd_iprintf(buffer, "  no LTC\n");
		}
		if (a & HDSPM_TCO1_Video_Input_Format_NTSC) {
			snd_iprintf(buffer, "  Video: NTSC\n");
		} else if (a & HDSPM_TCO1_Video_Input_Format_PAL) {
			snd_iprintf(buffer, "  Video: PAL\n");
		} else {
			snd_iprintf(buffer, "  No video\n");
		}
		if (a & HDSPM_TCO1_TCO_lock) {
			snd_iprintf(buffer, "  Sync: lock\n");
		} else {
			snd_iprintf(buffer, "  Sync: no lock\n");
		}

		switch (hdspm->io_type) {
		case MADI:
		case AES32:
			freq_const = 110069313433624ULL;
			break;
		case RayDAT:
		case AIO:
			freq_const = 104857600000000ULL;
			break;
		case MADIFX:
			freq_const = 131072000000000ULL;
		case MADIface:
			break; /* no TCO possible */
		}

		period = madifx_read(hdspm, HDSPM_RD_PLL_FREQ);
		snd_iprintf(buffer, "    period: %u\n", period);


		/* rate = freq_const/period; */
		rate = div_u64(freq_const, period);

		if (control & HDSPM_QuadSpeed) {
			rate *= 4;
		} else if (control & HDSPM_DoubleSpeed) {
			rate *= 2;
		}

		snd_iprintf(buffer, "  Frequency: %u Hz\n",
				(unsigned int) rate);

		ltc = madifx_read(hdspm, HDSPM_RD_TCO);
		frames = ltc & 0xF;
		ltc >>= 4;
		frames += (ltc & 0x3) * 10;
		ltc >>= 4;
		seconds = ltc & 0xF;
		ltc >>= 4;
		seconds += (ltc & 0x7) * 10;
		ltc >>= 4;
		minutes = ltc & 0xF;
		ltc >>= 4;
		minutes += (ltc & 0x7) * 10;
		ltc >>= 4;
		hours = ltc & 0xF;
		ltc >>= 4;
		hours += (ltc & 0x3) * 10;
		snd_iprintf(buffer,
			"  LTC In: %02d:%02d:%02d:%02d\n",
			hours, minutes, seconds, frames);

	} else {
		snd_iprintf(buffer, "No TCO module detected.\n");
	}

	snd_iprintf(buffer, "--- Settings ---\n");

	x = madifx_get_latency(hdspm);

	snd_iprintf(buffer,
		"Size (Latency): %d samples (2 periods of %lu bytes)\n",
		x, (unsigned long) hdspm->period_bytes);

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


	if (!(hdspm->control_register & HDSPM_ClockModeMaster))
		system_clock_mode = "AutoSync";
	else
		system_clock_mode = "Master";
	snd_iprintf(buffer, "AutoSync Reference: %s\n", system_clock_mode);

	switch (madifx_pref_sync_ref(hdspm)) {
	case HDSPM_SYNC_FROM_WORD:
		pref_sync_ref = "Word Clock";
		break;
	case HDSPM_SYNC_FROM_MADI:
		pref_sync_ref = "MADI Sync";
		break;
	case HDSPM_SYNC_FROM_TCO:
		pref_sync_ref = "TCO";
		break;
	case HDSPM_SYNC_FROM_SYNC_IN:
		pref_sync_ref = "Sync In";
		break;
	default:
		pref_sync_ref = "XXXX Clock";
		break;
	}
	snd_iprintf(buffer, "Preferred Sync Reference: %s\n",
			pref_sync_ref);

	snd_iprintf(buffer, "System Clock Frequency: %d\n",
			hdspm->system_sample_rate);


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

	snd_iprintf(buffer, "\n");
}

static void
snd_madifx_proc_read_aes32(struct snd_info_entry * entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	unsigned int status;
	unsigned int status2;
	unsigned int timecode;
	int pref_syncref;
	char *autosync_ref;
	int x;

	status = madifx_read(hdspm, HDSPM_statusRegister);
	status2 = madifx_read(hdspm, HDSPM_statusRegister2);
	timecode = madifx_read(hdspm, HDSPM_timecodeRegister);

	snd_iprintf(buffer, "%s (Card #%d) Rev.%x\n",
		    hdspm->card_name, hdspm->card->number + 1,
		    hdspm->firmware_rev);

	snd_iprintf(buffer, "IRQ: %d Registers bus: 0x%lx VM: 0x%lx\n",
		    hdspm->irq, hdspm->port, (unsigned long)hdspm->iobase);

	snd_iprintf(buffer, "--- System ---\n");

	snd_iprintf(buffer,
		    "IRQ Pending: Audio=%d, MIDI0=%d, MIDI1=%d, IRQcount=%d\n",
		    status & HDSPM_audioIRQPending,
		    (status & HDSPM_midi0IRQPending) ? 1 : 0,
		    (status & HDSPM_midi1IRQPending) ? 1 : 0,
		    hdspm->irq_count);
	snd_iprintf(buffer,
		    "HW pointer: id = %d, rawptr = %d (%d->%d) "
		    "estimated= %ld (bytes)\n",
		    ((status & HDSPM_BufferID) ? 1 : 0),
		    (status & HDSPM_BufferPositionMask),
		    (status & HDSPM_BufferPositionMask) %
		    (2 * (int)hdspm->period_bytes),
		    ((status & HDSPM_BufferPositionMask) - 64) %
		    (2 * (int)hdspm->period_bytes),
		    (long) madifx_hw_pointer(hdspm) * 4);

	snd_iprintf(buffer,
		    "MIDI FIFO: Out1=0x%x, Out2=0x%x, In1=0x%x, In2=0x%x \n",
		    madifx_read(hdspm, HDSPM_midiStatusOut0) & 0xFF,
		    madifx_read(hdspm, HDSPM_midiStatusOut1) & 0xFF,
		    madifx_read(hdspm, HDSPM_midiStatusIn0) & 0xFF,
		    madifx_read(hdspm, HDSPM_midiStatusIn1) & 0xFF);
	snd_iprintf(buffer,
		    "MIDIoverMADI FIFO: In=0x%x, Out=0x%x \n",
		    madifx_read(hdspm, HDSPM_midiStatusIn2) & 0xFF,
		    madifx_read(hdspm, HDSPM_midiStatusOut2) & 0xFF);
	snd_iprintf(buffer,
		    "Register: ctrl1=0x%x, ctrl2=0x%x, status1=0x%x, "
		    "status2=0x%x\n",
		    hdspm->control_register, hdspm->control2_register,
		    status, status2);

	snd_iprintf(buffer, "--- Settings ---\n");

	x = madifx_get_latency(hdspm);

	snd_iprintf(buffer,
		    "Size (Latency): %d samples (2 periods of %lu bytes)\n",
		    x, (unsigned long) hdspm->period_bytes);

	snd_iprintf(buffer, "Line out: %s\n",
		    (hdspm->
		     control_register & HDSPM_LineOut) ? "on " : "off");

	snd_iprintf(buffer,
		    "ClearTrackMarker %s, Emphasis %s, Dolby %s\n",
		    (hdspm->
		     control_register & HDSPM_clr_tms) ? "on" : "off",
		    (hdspm->
		     control_register & HDSPM_Emphasis) ? "on" : "off",
		    (hdspm->
		     control_register & HDSPM_Dolby) ? "on" : "off");


	pref_syncref = madifx_pref_sync_ref(hdspm);
	if (pref_syncref == 0)
		snd_iprintf(buffer, "Preferred Sync Reference: Word Clock\n");
	else
		snd_iprintf(buffer, "Preferred Sync Reference: AES%d\n",
				pref_syncref);

	snd_iprintf(buffer, "System Clock Frequency: %d\n",
		    hdspm->system_sample_rate);

	snd_iprintf(buffer, "Double speed: %s\n",
			hdspm->control_register & HDSPM_DS_DoubleWire?
			"Double wire" : "Single wire");
	snd_iprintf(buffer, "Quad speed: %s\n",
			hdspm->control_register & HDSPM_QS_DoubleWire?
			"Double wire" :
			hdspm->control_register & HDSPM_QS_QuadWire?
			"Quad wire" : "Single wire");

	snd_iprintf(buffer, "--- Status:\n");

	snd_iprintf(buffer, "Word: %s  Frequency: %d\n",
		    (status & HDSPM_AES32_wcLock) ? "Sync   " : "No Lock",
		    HDSPM_bit2freq((status >> HDSPM_AES32_wcFreq_bit) & 0xF));

	for (x = 0; x < 8; x++) {
		snd_iprintf(buffer, "AES%d: %s  Frequency: %d\n",
			    x+1,
			    (status2 & (HDSPM_LockAES >> x)) ?
			    "Sync   " : "No Lock",
			    HDSPM_bit2freq((timecode >> (4*x)) & 0xF));
	}

	switch (madifx_autosync_ref(hdspm)) {
	case HDSPM_AES32_AUTOSYNC_FROM_NONE:
		autosync_ref = "None"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_WORD:
		autosync_ref = "Word Clock"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES1:
		autosync_ref = "AES1"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES2:
		autosync_ref = "AES2"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES3:
		autosync_ref = "AES3"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES4:
		autosync_ref = "AES4"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES5:
		autosync_ref = "AES5"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES6:
		autosync_ref = "AES6"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES7:
		autosync_ref = "AES7"; break;
	case HDSPM_AES32_AUTOSYNC_FROM_AES8:
		autosync_ref = "AES8"; break;
	default:
		autosync_ref = "---"; break;
	}
	snd_iprintf(buffer, "AutoSync ref = %s\n", autosync_ref);

	snd_iprintf(buffer, "\n");
}

static void
snd_madifx_proc_read_raydat(struct snd_info_entry *entry,
			 struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	unsigned int status1, status2, status3, control, i;
	unsigned int lock, sync;

	status1 = madifx_read(hdspm, HDSPM_RD_STATUS_1); /* s1 */
	status2 = madifx_read(hdspm, HDSPM_RD_STATUS_2); /* freq */
	status3 = madifx_read(hdspm, HDSPM_RD_STATUS_3); /* s2 */

	control = hdspm->control_register;

	snd_iprintf(buffer, "STATUS1: 0x%08x\n", status1);
	snd_iprintf(buffer, "STATUS2: 0x%08x\n", status2);
	snd_iprintf(buffer, "STATUS3: 0x%08x\n", status3);


	snd_iprintf(buffer, "\n*** CLOCK MODE\n\n");

	snd_iprintf(buffer, "Clock mode      : %s\n",
		(madifx_system_clock_mode(hdspm) == 0) ? "master" : "slave");
	snd_iprintf(buffer, "System frequency: %d Hz\n",
		madifx_get_system_sample_rate(hdspm));

	snd_iprintf(buffer, "\n*** INPUT STATUS\n\n");

	lock = 0x1;
	sync = 0x100;

	for (i = 0; i < 8; i++) {
		snd_iprintf(buffer, "s1_input %d: Lock %d, Sync %d, Freq %s\n",
				i,
				(status1 & lock) ? 1 : 0,
				(status1 & sync) ? 1 : 0,
				texts_freq[(status2 >> (i * 4)) & 0xF]);

		lock = lock<<1;
		sync = sync<<1;
	}

	snd_iprintf(buffer, "WC input: Lock %d, Sync %d, Freq %s\n",
			(status1 & 0x1000000) ? 1 : 0,
			(status1 & 0x2000000) ? 1 : 0,
			texts_freq[(status1 >> 16) & 0xF]);

	snd_iprintf(buffer, "TCO input: Lock %d, Sync %d, Freq %s\n",
			(status1 & 0x4000000) ? 1 : 0,
			(status1 & 0x8000000) ? 1 : 0,
			texts_freq[(status1 >> 20) & 0xF]);

	snd_iprintf(buffer, "SYNC IN: Lock %d, Sync %d, Freq %s\n",
			(status3 & 0x400) ? 1 : 0,
			(status3 & 0x800) ? 1 : 0,
			texts_freq[(status2 >> 12) & 0xF]);

}

#ifdef CONFIG_SND_DEBUG
static void
snd_madifx_proc_read_debug(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;

	int j,i;

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

	for (i = 0; i < hdspm->max_channels_in; i++) {
		snd_iprintf(buffer, "%d=%s\n", i+1, hdspm->port_names_in[i]);
	}
}

static void snd_madifx_proc_ports_out(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	struct hdspm *hdspm = entry->private_data;
	int i;

	snd_iprintf(buffer, "# generated by hdspm\n");

	for (i = 0; i < hdspm->max_channels_out; i++) {
		snd_iprintf(buffer, "%d=%s\n", i+1, hdspm->port_names_out[i]);
	}
}


static void __devinit snd_madifx_proc_init(struct hdspm *hdspm)
{
	struct snd_info_entry *entry;

	if (!snd_card_proc_new(hdspm->card, "madifx", &entry)) {
		switch (hdspm->io_type) {
		case AES32:
			snd_info_set_text_ops(entry, hdspm,
					snd_madifx_proc_read_aes32);
			break;
		case MADI:
			snd_info_set_text_ops(entry, hdspm,
					snd_madifx_proc_read_madi);
			break;
		case MADIface:
			/* snd_info_set_text_ops(entry, hdspm,
			 snd_madifx_proc_read_madiface); */
			break;
		case RayDAT:
			snd_info_set_text_ops(entry, hdspm,
					snd_madifx_proc_read_raydat);
			break;
		case AIO:
			break;
		case MADIFX:
			/* FIXME: MADI FX missing */
			break;
		}
	}

	if (!snd_card_proc_new(hdspm->card, "ports.in", &entry)) {
		snd_info_set_text_ops(entry, hdspm, snd_madifx_proc_ports_in);
	}

	if (!snd_card_proc_new(hdspm->card, "ports.out", &entry)) {
		snd_info_set_text_ops(entry, hdspm, snd_madifx_proc_ports_out);
	}

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

static int snd_madifx_set_defaults(struct hdspm * hdspm)
{
	/* ASSUMPTION: hdspm->lock is either held, or there is no need to
	   hold it (e.g. during module initialization).
	   */

	/* set defaults:       */

	hdspm->settings_register = 0;

	switch (hdspm->io_type) {
	case MADI:
	case MADIface:
		hdspm->control_register =
			0x2 + 0x8 + 0x10 + 0x80 + 0x400 + 0x4000 + 0x1000000;
		break;

	case RayDAT:
	case AIO:
		hdspm->settings_register = 0x1 + 0x1000;
		/* Magic values are: LAT_0, LAT_2, Master, freq1, tx64ch, inp_0,
		 * line_out */
		hdspm->control_register =
			0x2 + 0x8 + 0x10 + 0x80 + 0x400 + 0x4000 + 0x1000000;
		break;

	case AES32:
		hdspm->control_register =
			HDSPM_ClockModeMaster |	/* Master Cloack Mode on */
			madifx_encode_latency(7) | /* latency max=8192samples */
			HDSPM_SyncRef0 |	/* AES1 is syncclock */
			HDSPM_LineOut |	/* Analog output in */
			HDSPM_Professional;  /* Professional mode */
		break;
	case MADIFX:
		/* LAT_3+BUF_SIZ_1+BUF_SIZ_2+freq1; */
		hdspm->control_register = 0x800 + 0x40 + 0x80 + 0x4;
		/* PRO+madi1_tx_64ch+madi2_tx_64ch+madi3_tx_64ch; */
		hdspm->settings_register = 0x8 + 0x80 + 0x100 + 0x200;
		break;
	}

	madifx_write(hdspm, MADIFX_CONTROL_REG, hdspm->control_register);

	madifx_compute_period_size(hdspm);

	/* silence everything */

	/* FIXME */
	//all_in_all_mixer(hdspm, 0 * UNITY_GAIN);

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

	status = madifx_read(hdspm, HDSPM_statusRegister);

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

#define NUM_AES_PAGES 32768*2/4096
#define NUM_MADI_PAGES 32768*192/4096
#define NUM_DMA_CH_PAGES 32768*8/4096
#define MADIFX_HARDWARE_PAGE_SIZE 4096

	{
		int wanted;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			wanted = OUTPUT_DMA_BUFFER_SIZE;
		} else {
			wanted = INPUT_DMA_BUFFER_SIZE;
		}

		err = snd_pcm_lib_malloc_pages(substream, wanted);
		if (err < 0) {
			snd_printk(KERN_INFO "err on snd_pcm_lib_malloc_pages: %d\n", err);
			return err;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* initialise default DMA table. Will be
		 * overwritten in a second. */
		for (i = 0; i < MADIFX_MAX_PAGE_TABLE_SIZE/2; i++) {
			hdspm->dmaPageTable[i] = snd_pcm_sgbuf_get_addr(substream, 0);
		}

		/* AES Out, stereo */
		for (i = 0; i < NUM_AES_PAGES; i++) {
			hdspm->dmaPageTable[i] = snd_pcm_sgbuf_get_addr(substream,
					i * MADIFX_HARDWARE_PAGE_SIZE);
		}

		/* Phones Out, stereo */
		for (i = 0; i < NUM_AES_PAGES; i++) {
			hdspm->dmaPageTable[i+1*NUM_DMA_CH_PAGES] =
				snd_pcm_sgbuf_get_addr(substream,
						(i+1*NUM_AES_PAGES)* MADIFX_HARDWARE_PAGE_SIZE);
		}

		/* MADI Out, 192 channels */
		for (i = 0; i < NUM_MADI_PAGES; i++) {
			hdspm->dmaPageTable[i+2*NUM_DMA_CH_PAGES] =
				snd_pcm_sgbuf_get_addr(substream,
						(i+2*NUM_AES_PAGES) * MADIFX_HARDWARE_PAGE_SIZE);
		}

		for (i = 0; i < MADIFX_MAX_PAGE_TABLE_SIZE/2; i++) {
			madifx_write(hdspm, MADIFX_PAGE_ADDRESS_LIST + (4 * i),
					hdspm->dmaPageTable[i]);
		}

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

#if 0
		madifx_set_sgbuf(hdspm, substream, HDSPM_pageAddressBufferIn,
				params_channels(params));
#endif

		for (i = 0; i < 32; ++i)
			snd_madifx_enable_in(hdspm, i, 1);

		hdspm->capture_buffer =
			(unsigned char *) substream->runtime->dma_area;
		snd_printdd("Allocated sample buffer for capture at %p\n",
				hdspm->capture_buffer);
	}

	/*
	   snd_printdd("Allocated sample buffer for %s at 0x%08X\n",
	   substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	   "playback" : "capture",
	   snd_pcm_sgbuf_get_addr(substream, 0));
	   */
	/*
	   snd_printdd("set_hwparams: %s %d Hz, %d channels, bs = %d\n",
	   substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	   "playback" : "capture",
	   params_rate(params), params_channels(params),
	   params_buffer_size(params));
	   */


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
		if (snd_BUG_ON(info->channel >= hdspm->max_channels_out)) {
			snd_printk(KERN_INFO "snd_madifx_channel_info: output channel out of range (%d)\n", info->channel);
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
			info->offset = (info->channel < 2) ?
				0 : ((info->channel > 193) ? 65536 :
						131072 + 8 * 4 * 8192 * ((info->channel-2)/8));
			info->first = (info->channel < 2 || info->channel > 193) ?
				32 * info->channel : 32 * ((info->channel-2) % 8);
			info->step = (info->channel < 2 || info->channel > 193) ? 64 : 256;
			break;
		case ds:
		case qs:
			/* FIXME: No idea if really needed */
			break;
		}
	} else {
		if (snd_BUG_ON(info->channel >= hdspm->max_channels_in)) {
			snd_printk(KERN_INFO "snd_madifx_channel_info: input channel out of range (%d)\n", info->channel);
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
			info->offset = (info->channel < 2) ? 0 : 65536 + 8 * 4 * 8192 *
				((info->channel-2)/8);
			info->first = (info->channel < 2) ? 32 * info->channel :
				32 * ((info->channel-2) % 8);
			info->step = (info->channel < 2) ? 64 : 256;
			break;
		case ds:
		case qs:
			/* FIXME: No idea if really needed */
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
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = (SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 |
		  SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_64000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000 ),
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
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
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
					   struct snd_pcm_hw_rule * rule)
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
					   struct snd_pcm_hw_rule * rule)
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


static unsigned int madifx_aes32_sample_rates[] = {
	32000, 44100, 48000, 64000, 88200, 96000, 128000, 176400, 192000
};

static struct snd_pcm_hw_constraint_list
madifx_hw_constraints_aes32_sample_rates = {
	.count = ARRAY_SIZE(madifx_aes32_sample_rates),
	.list = madifx_aes32_sample_rates,
	.mask = 0
};

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
	case AIO:
	case RayDAT:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     32, 4096);
		/* RayDAT & AIO have a fixed buffer of 16384 samples per channel */
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
					     16384, 16384);
		break;
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

	if (AES32 == hdspm->io_type) {
		runtime->hw.rates |= SNDRV_PCM_RATE_KNOT;
		snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				&madifx_hw_constraints_aes32_sample_rates);
	} else {
		snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				snd_madifx_hw_rule_rate_out_channels, hdspm,
				SNDRV_PCM_HW_PARAM_CHANNELS, -1);
	}

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
	case AIO:
	case RayDAT:
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     32, 4096);
		snd_pcm_hw_constraint_minmax(runtime,
					     SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
					     16384, 16384);
		break;
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

	if (AES32 == hdspm->io_type) {
		runtime->hw.rates |= SNDRV_PCM_RATE_KNOT;
		snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				&madifx_hw_constraints_aes32_sample_rates);
	} else {
		snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				snd_madifx_hw_rule_rate_in_channels, hdspm,
				SNDRV_PCM_HW_PARAM_CHANNELS, -1);
	}

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
	struct madifx_version madifx_version;
	struct madifx_peak_rms *levels;
	struct madifx_ltc ltc;
	unsigned int statusregister;
	long unsigned int s;
	int i = 0;

	switch (cmd) {

	case SNDRV_HDSPM_IOCTL_GET_PEAK_RMS:
		levels = &hdspm->peak_rms;
		for (i = 0; i < HDSPM_MAX_CHANNELS; i++) {
			levels->input_peaks[i] =
				readl(hdspm->iobase +
						HDSPM_MADI_INPUT_PEAK + i*4);
			levels->playback_peaks[i] =
				readl(hdspm->iobase +
						HDSPM_MADI_PLAYBACK_PEAK + i*4);
			levels->output_peaks[i] =
				readl(hdspm->iobase +
						HDSPM_MADI_OUTPUT_PEAK + i*4);

			levels->input_rms[i] =
				((uint64_t) readl(hdspm->iobase +
					HDSPM_MADI_INPUT_RMS_H + i*4) << 32) |
				(uint64_t) readl(hdspm->iobase +
						HDSPM_MADI_INPUT_RMS_L + i*4);
			levels->playback_rms[i] =
				((uint64_t)readl(hdspm->iobase +
					HDSPM_MADI_PLAYBACK_RMS_H+i*4) << 32) |
				(uint64_t)readl(hdspm->iobase +
					HDSPM_MADI_PLAYBACK_RMS_L + i*4);
			levels->output_rms[i] =
				((uint64_t)readl(hdspm->iobase +
					HDSPM_MADI_OUTPUT_RMS_H + i*4) << 32) |
				(uint64_t)readl(hdspm->iobase +
						HDSPM_MADI_OUTPUT_RMS_L + i*4);
		}

		if (hdspm->system_sample_rate > 96000) {
			levels->speed = qs;
		} else if (hdspm->system_sample_rate > 48000) {
			levels->speed = ds;
		} else {
			levels->speed = ss;
		}
		levels->status2 = madifx_read(hdspm, HDSPM_statusRegister2);

		s = copy_to_user(argp, levels, sizeof(struct madifx_peak_rms));
		if (0 != s) {
			/* snd_printk(KERN_ERR "copy_to_user(.., .., %lu): %lu
			 [Levels]\n", sizeof(struct madifx_peak_rms), s);
			 */
			return -EFAULT;
		}
		break;

	case SNDRV_HDSPM_IOCTL_GET_LTC:
		ltc.ltc = madifx_read(hdspm, HDSPM_RD_TCO);
		i = madifx_read(hdspm, HDSPM_RD_TCO + 4);
		if (i & HDSPM_TCO1_LTC_Input_valid) {
			switch (i & (HDSPM_TCO1_LTC_Format_LSB |
				HDSPM_TCO1_LTC_Format_MSB)) {
			case 0:
				ltc.format = fps_24;
				break;
			case HDSPM_TCO1_LTC_Format_LSB:
				ltc.format = fps_25;
				break;
			case HDSPM_TCO1_LTC_Format_MSB:
				ltc.format = fps_2997;
				break;
			default:
				ltc.format = 30;
				break;
			}
			if (i & HDSPM_TCO1_set_drop_frame_flag) {
				ltc.frame = drop_frame;
			} else {
				ltc.frame = full_frame;
			}
		} else {
			ltc.format = format_invalid;
			ltc.frame = frame_invalid;
		}
		if (i & HDSPM_TCO1_Video_Input_Format_NTSC) {
			ltc.input_format = ntsc;
		} else if (i & HDSPM_TCO1_Video_Input_Format_PAL) {
			ltc.input_format = pal;
		} else {
			ltc.input_format = no_video;
		}

		s = copy_to_user(argp, &ltc, sizeof(struct madifx_ltc));
		if (0 != s) {
			/*
			 snd_printk(KERN_ERR "copy_to_user(.., .., %lu): %lu [LTC]\n", sizeof(struct madifx_ltc), s); */
			return -EFAULT;
		}

		break;

	case SNDRV_HDSPM_IOCTL_GET_CONFIG:

		memset(&info, 0, sizeof(info));
		spin_lock_irq(&hdspm->lock);
		info.pref_sync_ref = madifx_pref_sync_ref(hdspm);
		info.wordclock_sync_check = madifx_wc_sync_check(hdspm);

		info.system_sample_rate = hdspm->system_sample_rate;
		info.autosync_sample_rate =
			madifx_external_sample_rate(hdspm);
		info.system_clock_mode = madifx_system_clock_mode(hdspm);
		info.clock_source = madifx_clock_source(hdspm);
		info.autosync_ref = madifx_autosync_ref(hdspm);
		info.line_out = madifx_line_out(hdspm);
		info.passthru = 0;
		spin_unlock_irq(&hdspm->lock);
		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;

	case SNDRV_HDSPM_IOCTL_GET_STATUS:
		memset(&status, 0, sizeof(status));

		status.card_type = hdspm->io_type;

		status.autosync_source = madifx_autosync_ref(hdspm);

		status.card_clock = 110069313433624ULL;
		status.master_period = madifx_read(hdspm, HDSPM_RD_PLL_FREQ);

		switch (hdspm->io_type) {
		case MADI:
		case MADIface:
			status.card_specific.madi.sync_wc =
				madifx_wc_sync_check(hdspm);
			status.card_specific.madi.sync_madi =
				madifx_madi_sync_check(hdspm);
			status.card_specific.madi.sync_tco =
				madifx_tco_sync_check(hdspm);
			status.card_specific.madi.sync_in =
				madifx_sync_in_sync_check(hdspm);

			statusregister =
				madifx_read(hdspm, HDSPM_statusRegister);
			status.card_specific.madi.madi_input =
				(statusregister & HDSPM_AB_int) ? 1 : 0;
			status.card_specific.madi.channel_format =
				(statusregister & HDSPM_RX_64ch) ? 1 : 0;
			/* TODO: Mac driver sets it when f_s>48kHz */
			status.card_specific.madi.frame_format = 0;

		default:
			break;
		}

		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;


		break;

	case SNDRV_HDSPM_IOCTL_GET_VERSION:
		memset(&madifx_version, 0, sizeof(madifx_version));

		madifx_version.card_type = hdspm->io_type;
		strncpy(madifx_version.cardname, hdspm->card_name,
				sizeof(madifx_version.cardname));
		madifx_version.serial = hdspm->serial;
		madifx_version.firmware_rev = hdspm->firmware_rev;
		madifx_version.addons = 0;
		if (hdspm->tco)
			madifx_version.addons |= HDSPM_ADDON_TCO;

		if (copy_to_user(argp, &madifx_version,
					sizeof(madifx_version)))
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
					    struct hdspm * hdspm)
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
	struct snd_pcm *pcm;
	size_t wanted;

	pcm = hdspm->pcm;


	wanted = max(INPUT_DMA_BUFFER_SIZE, OUTPUT_DMA_BUFFER_SIZE);

	hdspm->dmaPageTable = kzalloc(sizeof(dma_addr_t) *
			MADIFX_MAX_PAGE_TABLE_SIZE, GFP_KERNEL);

	if (!hdspm->dmaPageTable) {
		snd_printk(KERN_ERR "MADIFX: "
				"unable to kmalloc dmaPageTable memory\n");
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



	return 0;
}


static void madifx_set_sgbuf(struct hdspm *hdspm,
			    struct snd_pcm_substream *substream,
			     unsigned int reg, int offset)
{
	/* continuous memory segment */
		madifx_write(hdspm, reg + 4 * offset,
				snd_pcm_sgbuf_get_addr(substream, 4096 * offset));
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

static inline void snd_madifx_initialize_midi_flush(struct hdspm * hdspm)
{
	int i;

	for (i = 0; i < hdspm->midiPorts; i++)
		snd_madifx_flush_midi_input(hdspm, i);
}

static int __devinit snd_madifx_create_alsa_devices(struct snd_card *card,
						   struct hdspm * hdspm)
{
	int err, i;

	snd_printdd("Create card...\n");
	err = snd_madifx_create_pcm(card, hdspm);
	if (err < 0)
		return err;

	i = 0;
	while (i < hdspm->midiPorts) {
		err = snd_madifx_create_midi(card, hdspm, i);
		if (err < 0) {
			return err;
		}
		i++;
	}

	err = snd_madifx_create_controls(card, hdspm);
	if (err < 0)
		return err;

	err = snd_madifx_create_hwdep(card, hdspm);
	if (err < 0)
		return err;

	snd_printdd("proc init...\n");
	/* FIXME: MADI FX disable, no proc so far */
	//snd_madifx_proc_init(hdspm);

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
	/* FIXME: MADI FX disable, old mixer is broken */
	madifx_update_simple_mixer_controls(hdspm);

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
		hdspm->midiPorts = 0;  /* FIXME: MIDI handling broken atm */
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
		snd_printk(KERN_ERR "HDSPM: "
				"unable to remap region 0x%lx-0x%lx\n",
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
		snd_printk(KERN_ERR "HDSPM: "
				"unable to kmalloc Mixer memory of %d Bytes\n",
				(int)sizeof(struct madifx_newmixer));
		return -ENOMEM;
	}

	/* FIXME: Drop this later when mixer is correctly replaced */
	hdspm->mixer = kzalloc(sizeof(struct madifx_mixer), GFP_KERNEL);
	if (!hdspm->mixer) {
		snd_printk(KERN_ERR "HDSPM: "
				"unable to kmalloc Mixer memory of %d Bytes\n",
				(int)sizeof(struct madifx_mixer));
		return -ENOMEM;
	}


	{
		/* This somehow initialises the mixer. I have no idea what it
		 * does exactly.
		 */
		int i;
		for (i = 0; i < MADIFX_NUM_OUTPUT_GAINS; i++) {
			hdspm->newmixer->output_gain[i] = 0x9000;
		}

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
	case AES32:
		hdspm->ss_in_channels = hdspm->ss_out_channels = AES32_CHANNELS;
		hdspm->ds_in_channels = hdspm->ds_out_channels = AES32_CHANNELS;
		hdspm->qs_in_channels = hdspm->qs_out_channels = AES32_CHANNELS;

		hdspm->port_names_in_ss = hdspm->port_names_out_ss =
			texts_ports_aes32;
		hdspm->port_names_in_ds = hdspm->port_names_out_ds =
			texts_ports_aes32;
		hdspm->port_names_in_qs = hdspm->port_names_out_qs =
			texts_ports_aes32;

		hdspm->max_channels_out = hdspm->max_channels_in =
			AES32_CHANNELS;
		hdspm->port_names_in = hdspm->port_names_out =
			texts_ports_aes32;

		break;

	case MADI:
	case MADIface:
		hdspm->ss_in_channels = hdspm->ss_out_channels =
			MADI_SS_CHANNELS;
		hdspm->ds_in_channels = hdspm->ds_out_channels =
			MADI_DS_CHANNELS;
		hdspm->qs_in_channels = hdspm->qs_out_channels =
			MADI_QS_CHANNELS;

		hdspm->port_names_in_ss = hdspm->port_names_out_ss =
			texts_ports_madi;
		hdspm->port_names_in_ds = hdspm->port_names_out_ds =
			texts_ports_madi;
		hdspm->port_names_in_qs = hdspm->port_names_out_qs =
			texts_ports_madi;
		break;

	case MADIFX:
		hdspm->ss_in_channels = MADIFX_SS_IN_CHANNELS;
		hdspm->ds_in_channels = MADIFX_DS_IN_CHANNELS;
		hdspm->qs_in_channels = MADIFX_QS_IN_CHANNELS;
		hdspm->ss_out_channels = MADIFX_SS_OUT_CHANNELS;
		hdspm->ds_out_channels = MADIFX_DS_OUT_CHANNELS;
		hdspm->qs_out_channels = MADIFX_QS_OUT_CHANNELS;
		/* FIXME: portnames and stuff missing */
		break;

	case AIO:
		if (0 == (madifx_read(hdspm, HDSPM_statusRegister2) & HDSPM_s2_AEBI_D)) {
			snd_printk(KERN_INFO "HDSPM: AEB input board found, but not supported\n");
		}

		hdspm->ss_in_channels = AIO_IN_SS_CHANNELS;
		hdspm->ds_in_channels = AIO_IN_DS_CHANNELS;
		hdspm->qs_in_channels = AIO_IN_QS_CHANNELS;
		hdspm->ss_out_channels = AIO_OUT_SS_CHANNELS;
		hdspm->ds_out_channels = AIO_OUT_DS_CHANNELS;
		hdspm->qs_out_channels = AIO_OUT_QS_CHANNELS;

		hdspm->port_names_in_ss = texts_ports_aio_in_ss;
		hdspm->port_names_out_ss = texts_ports_aio_out_ss;
		hdspm->port_names_in_ds = texts_ports_aio_in_ds;
		hdspm->port_names_out_ds = texts_ports_aio_out_ds;
		hdspm->port_names_in_qs = texts_ports_aio_in_qs;
		hdspm->port_names_out_qs = texts_ports_aio_out_qs;

		break;

	case RayDAT:
		hdspm->ss_in_channels = hdspm->ss_out_channels =
			RAYDAT_SS_CHANNELS;
		hdspm->ds_in_channels = hdspm->ds_out_channels =
			RAYDAT_DS_CHANNELS;
		hdspm->qs_in_channels = hdspm->qs_out_channels =
			RAYDAT_QS_CHANNELS;

		hdspm->max_channels_in = RAYDAT_SS_CHANNELS;
		hdspm->max_channels_out = RAYDAT_SS_CHANNELS;

		hdspm->port_names_in_ss = hdspm->port_names_out_ss =
			texts_ports_raydat_ss;
		hdspm->port_names_in_ds = hdspm->port_names_out_ds =
			texts_ports_raydat_ds;
		hdspm->port_names_in_qs = hdspm->port_names_out_qs =
			texts_ports_raydat_qs;


		break;

	}

	/* TCO detection */
	/* FIXME: TCO detection on MADIFX missing */
	switch (hdspm->io_type) {
	case AIO:
	case RayDAT:
		if (madifx_read(hdspm, HDSPM_statusRegister2) &
				HDSPM_s2_tco_detect) {
			hdspm->midiPorts++;
			hdspm->tco = kzalloc(sizeof(struct madifx_tco),
					GFP_KERNEL);
			if (NULL != hdspm->tco) {
				madifx_tco_write(hdspm);
			}
			snd_printk(KERN_INFO "HDSPM: AIO/RayDAT TCO module found\n");
		} else {
			hdspm->tco = NULL;
		}
		break;

	case MADI:
		if (madifx_read(hdspm, HDSPM_statusRegister) & HDSPM_tco_detect) {
			hdspm->midiPorts++;
			hdspm->tco = kzalloc(sizeof(struct madifx_tco),
					GFP_KERNEL);
			if (NULL != hdspm->tco) {
				madifx_tco_write(hdspm);
			}
			snd_printk(KERN_INFO "HDSPM: MADI TCO module found\n");
		} else {
			hdspm->tco = NULL;
		}
		break;

	default:
		hdspm->tco = NULL;
	}

	/* texts */
	switch (hdspm->io_type) {
	case AES32:
		if (hdspm->tco) {
			hdspm->texts_autosync = texts_autosync_aes_tco;
			hdspm->texts_autosync_items = 10;
		} else {
			hdspm->texts_autosync = texts_autosync_aes;
			hdspm->texts_autosync_items = 9;
		}
		break;

	case MADI:
	case MADIFX:
		if (hdspm->tco) {
			hdspm->texts_autosync = texts_autosync_madi_tco;
			hdspm->texts_autosync_items = 4;
		} else {
			hdspm->texts_autosync = texts_autosync_madi;
			hdspm->texts_autosync_items = 3;
		}
		break;

	case MADIface:

		break;

	case RayDAT:
		if (hdspm->tco) {
			hdspm->texts_autosync = texts_autosync_raydat_tco;
			hdspm->texts_autosync_items = 9;
		} else {
			hdspm->texts_autosync = texts_autosync_raydat;
			hdspm->texts_autosync_items = 8;
		}
		break;

	case AIO:
		if (hdspm->tco) {
			hdspm->texts_autosync = texts_autosync_aio_tco;
			hdspm->texts_autosync_items = 6;
		} else {
			hdspm->texts_autosync = texts_autosync_aio;
			hdspm->texts_autosync_items = 5;
		}
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


static int snd_madifx_free(struct hdspm * hdspm)
{

	if (hdspm->port) {

		/* stop th audio, and cancel all interrupts */
		hdspm->control_register &=
		    ~(MADIFX_START | MADIFX_IE_AUDIO |
		      MADIFX_IEN0 | MADIFX_IEN1 |
		      MADIFX_IEN2 | MADIFX_IEN3);
		madifx_write(hdspm, MADIFX_CONTROL_REG,
			    hdspm->control_register);
	}

	if (hdspm->irq >= 0)
		free_irq(hdspm->irq, (void *) hdspm);

	kfree(hdspm->mixer);
	kfree(hdspm->dmaPageTable);

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

	if (hdspm->io_type != MADIface) {
		sprintf(card->shortname, "%s_%x",
			hdspm->card_name,
			hdspm->serial);
		sprintf(card->longname, "%s S/N 0x%x at 0x%lx, irq %d",
			hdspm->card_name,
			hdspm->serial,
			hdspm->port, hdspm->irq);
	} else {
		sprintf(card->shortname, "%s", hdspm->card_name);
		sprintf(card->longname, "%s at 0x%lx, irq %d",
				hdspm->card_name, hdspm->port, hdspm->irq);
	}

	err = snd_card_register(card);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	pci_set_drvdata(pci, card);

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
