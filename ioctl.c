/* compile with gcc -o madifx_testtool ioctl.c -lasound */
#include <stdio.h>
#include <sys/ioctl.h>
#include <alsa/asoundlib.h>
#include <stdint.h>
#include "madifx.h"

int main(void) {
    int err;
    snd_hwdep_t *hw;
    snd_hwdep_info_t *info;
    snd_hwdep_info_alloca(&info);
    int i;
    struct madifx_level_buffer mybuf;
    struct madifx_config myconfig;

    if ((err = snd_hwdep_open(&hw, "hw:MADIFXtest", SND_HWDEP_OPEN_DUPLEX)) != 0) {
        fprintf(stderr, "Error opening hwdep device\n");
        return 1; 
    }

    if ((err = snd_hwdep_ioctl(hw, SNDRV_MADIFX_IOCTL_GET_LEVEL, &mybuf)) < 0) {
        fprintf(stderr, "Hwdep ioctl error\n");
        snd_hwdep_close(hw);
        return 2; 
    }

    if ((err = snd_hwdep_ioctl(hw, SNDRV_MADIFX_IOCTL_GET_CONFIG, &myconfig)) < 0) {
        fprintf(stderr, "Hwdep ioctl error\n");
        snd_hwdep_close(hw);
        return 3; 
    }

    for (i=0 ; i < 16; i++) {
	    printf ("%i ", mybuf.rms_out[i]);
    }
    printf ("\n");

    for (i=0; i < 256; i++) {
	    if (mybuf.rms_in[i] != 0) {
		    printf ("%i at %i\n", mybuf.rms_in[i], i);
	    }
    }
    printf ("\n");

    for (i=0; i < 3; i++) {
	    printf ("MADI %i TX64: %i SMUX: %i\n", i+1, myconfig.madi_tx_64[i], myconfig.madi_smux[i]);
    }

    printf ("WCK48: %i\nWCTERM: %i\n", myconfig.wck48, myconfig.wcterm);
    printf ("AES Pro: %i\n", myconfig.aespro);
    printf ("Redundancy mode: %i\nMirror MADI out: %i\n", myconfig.redundancy_mode,
		    myconfig.mirror_madi_out);


    snd_hwdep_close(hw);
}

