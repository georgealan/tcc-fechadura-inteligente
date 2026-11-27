#ifndef AUDIOS_FIXOS_H
#define AUDIOS_FIXOS_H

#include <stddef.h>
#include <stdint.h>
#include "audio_i2s.h"

extern const uint8_t facial_inicio_wav_start[]      asm("_binary_facial_inicio_wav_start");
extern const uint8_t facial_inicio_wav_end[]        asm("_binary_facial_inicio_wav_end");

extern const uint8_t facial_processando_wav_start[] asm("_binary_facial_processando_wav_start");
extern const uint8_t facial_processando_wav_end[]   asm("_binary_facial_processando_wav_end");

extern const uint8_t senha_incorreta_wav_start[]    asm("_binary_senha_incorreta_wav_start");
extern const uint8_t senha_incorreta_wav_end[]      asm("_binary_senha_incorreta_wav_end");

extern const uint8_t porta_aberta_wav_start[]       asm("_binary_porta_aberta_wav_start");
extern const uint8_t porta_aberta_wav_end[]         asm("_binary_porta_aberta_wav_end");

extern const uint8_t teclado_bloqueado_wav_start[]  asm("_binary_teclado_bloqueado_wav_start");
extern const uint8_t teclado_bloqueado_wav_end[]    asm("_binary_teclado_bloqueado_wav_end");

static inline void audio_falar_facial_inicio(void) {
    audio_i2s_tocar_wav(facial_inicio_wav_start,
                        (size_t)(facial_inicio_wav_end - facial_inicio_wav_start));
}

static inline void audio_falar_facial_processando(void) {
    audio_i2s_tocar_wav(facial_processando_wav_start,
                        (size_t)(facial_processando_wav_end - facial_processando_wav_start));
}

static inline void audio_falar_senha_incorreta(void) {
    audio_i2s_tocar_wav(senha_incorreta_wav_start,
                        (size_t)(senha_incorreta_wav_end - senha_incorreta_wav_start));
}

static inline void audio_falar_porta_aberta(void) {
    audio_i2s_tocar_wav(porta_aberta_wav_start,
                        (size_t)(porta_aberta_wav_end - porta_aberta_wav_start));
}

static inline void audio_falar_teclado_bloqueado(void) {
    audio_i2s_tocar_wav(teclado_bloqueado_wav_start,
                        (size_t)(teclado_bloqueado_wav_end - teclado_bloqueado_wav_start));
}

#endif // AUDIOS_FIXOS_H
