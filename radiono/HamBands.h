// HamBands.h

#ifndef HAMBANDS_H
#define HAMBANDS_H

    
    // The Number of Ham Bands
    // Need to adjust these for UK 60m, 137 and 472 as well so actually 12 bands and 60m is a bit different to US
    #define BANDS_BASE (9)  // 9 Standard HF Band
    #define BANDS_60M (5)   // 5 60m Freq's which are treated as bands here
    
    #ifdef USE_10M_SECTIONS
        #define LOWER_FREQ_10M_SECTION_01 (28.0 * MHz) // Bottom of Band
        #define UPPER_FREQ_10M_SECTION_01 (28.3 * MHz)
        #define UPPER_FREQ_10M_SECTION_02 (29.0 * MHz)
        #define UPPER_FREQ_10M_SECTION_03 (29.7 * MHz) // Top of Band
        #define ADDITIONAL_SECTIONS_IN_10M (2) // Additional 10m Sections
    #else
        #define ADDITIONAL_SECTIONS_IN_10M (0)
    #endif
    
    #ifdef USE_80M_SECTIONS
        #define LOWER_FREQ_80M_SECTION_01 (3.5 * MHz) // Bottom of Band
        #define UPPER_FREQ_80M_SECTION_01 (3.6 * MHz)
        #define UPPER_FREQ_80M_SECTION_02 (3.8 * MHz) // Top of Band
        #define ADDITIONAL_SECTIONS_IN_80M (1) // Additional 80m Sections
    #else
        #define ADDITIONAL_SECTIONS_IN_80M (0)
    #endif
    
    // BANDS is used many places in the code
    #define BANDS (BANDS_BASE + BANDS_60M + ADDITIONAL_SECTIONS_IN_10M + ADDITIONAL_SECTIONS_IN_80M)   
    
    
    
    // Externally Available Variables    
    // PROGMEM is used to avoid using the small available variable space
    extern const unsigned long bandLimits[BANDS*2] PROGMEM;
    
    // An Array to save: A-VFO & B-VFO
    extern unsigned long freqCache[BANDS*2];
    extern byte sideBandModeCache[BANDS*2];
    extern byte inBand;
    extern int hamBands[BANDS];
    extern boolean operate60m;
    
    // Externally Available Functions
    extern int inBandLimits(unsigned long freq);
    extern void decodeBandUpDown(int dir);

#endif

// End
