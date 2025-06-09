#ifndef TEST_H
#define TEST_H

#include <stdint.h>

/* WAV file data array - replace this with your actual WAV data */
/* Note: This is a placeholder - you should replace the data array with your actual WAV file bytes */
static const uint8_t wav_file_data[] = {
    /* RIFF header */
    'R', 'I', 'F', 'F',
    0x66, 0x23, 0x07, 0x00,  // File size - 8 (approximately 500KB)
    'W', 'A', 'V', 'E',
    
    /* fmt chunk */
    'f', 'm', 't', ' ',
    0x10, 0x00, 0x00, 0x00,  // fmt chunk size (16)
    0x01, 0x00,              // Audio format (PCM)
    0x02, 0x00,              // Channels (2)
    0x44, 0xAC, 0x00, 0x00,  // Sample rate (44100)
    0x10, 0xB1, 0x02, 0x00,  // Byte rate
    0x04, 0x00,              // Block align
    0x10, 0x00,              // Bits per sample (16)
    
    /* data chunk */
    'd', 'a', 't', 'a',
    0x42, 0x23, 0x07, 0x00,  // Data size (approximately 500KB - 44 bytes)
    
    /*
     * TODO: Replace this placeholder data with your actual WAV file data
     * The array should contain approximately 500KB of audio data
     * You can generate this by converting your WAV file to a C array
     */
    
    /* Placeholder audio data - replace with your actual WAV data */
    0x00, 0x00, 0x00, 0x00, 0x55, 0xAA, 0x55, 0xAA,
    0xFF, 0x00, 0xFF, 0x00, 0x33, 0xCC, 0x33, 0xCC,
    0x80, 0x7F, 0x80, 0x7F, 0x40, 0xBF, 0x40, 0xBF,
    0xC0, 0x3F, 0xC0, 0x3F, 0x20, 0xDF, 0x20, 0xDF,
    0xE0, 0x1F, 0xE0, 0x1F, 0x10, 0xEF, 0x10, 0xEF,
    0xF0, 0x0F, 0xF0, 0x0F, 0x08, 0xF7, 0x08, 0xF7,
    0xF8, 0x07, 0xF8, 0x07, 0x04, 0xFB, 0x04, 0xFB,
    0xFC, 0x03, 0xFC, 0x03, 0x02, 0xFD, 0x02, 0xFD,
    
    /* Add your actual WAV data here - this is just a small sample */
    /* The total array should be approximately 500KB (512000 bytes) */
};

/* Macros for size calculation */
#define WAV_DATA_SIZE           sizeof(wav_file_data)
#define WAV_DATA_HEADER_SIZE    44  /* Standard WAV header size */
#define WAV_DATA_AUDIO_SIZE     (WAV_DATA_SIZE - WAV_DATA_HEADER_SIZE)

/* Macro to get WAV data pointer */
#define GET_WAV_DATA()          wav_file_data
#define GET_WAV_DATA_SIZE()     WAV_DATA_SIZE

/* Validation macros */
#define IS_VALID_WAV_SIZE(size) ((size) <= MAX_WAV_SIZE)
#define CHUNKS_NEEDED(size)     (((size) + WAV_CHUNK_SIZE - sizeof(chunk_header_t) - 1) / (WAV_CHUNK_SIZE - sizeof(chunk_header_t)))

/* Runtime size validation function */
static inline int validate_wav_data_size(void) {
    if (WAV_DATA_SIZE > (512 * 1024)) {  /* MAX_WAV_SIZE */
        return -1;  /* Size too large */
    }
    return 0;  /* Size OK */
}

#endif /* TEST_H */