/*
 * FIPS 180-2 SHA-224/256/384/512 implementation
 * Last update: 02/02/2007
 * Issue date:  04/30/2005
 *
 * Copyright (C) 2013, Con Kolivas <kernel@kolivas.org>
 * Copyright (C) 2005, 2007 Olivier Gay <olivier.gay@a3.epfl.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <string.h>
#ifdef USE_CRYPTO_EXT
#include <arm_neon.h>
#endif

#include "sha2.h"

#define UNPACK32(x, str)                      \
{                                             \
    *((str) + 3) = (uint8_t) ((x)      );       \
    *((str) + 2) = (uint8_t) ((x) >>  8);       \
    *((str) + 1) = (uint8_t) ((x) >> 16);       \
    *((str) + 0) = (uint8_t) ((x) >> 24);       \
}

#define PACK32(str, x)                        \
{                                             \
    *(x) =   ((uint32_t) *((str) + 3)      )    \
           | ((uint32_t) *((str) + 2) <<  8)    \
           | ((uint32_t) *((str) + 1) << 16)    \
           | ((uint32_t) *((str) + 0) << 24);   \
}

#define SHA256_SCR(i)                         \
{                                             \
    w[i] =  SHA256_F4(w[i -  2]) + w[i -  7]  \
          + SHA256_F3(w[i - 15]) + w[i - 16]; \
}

uint32_t sha256_h0[8] =
            {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
             0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

uint32_t sha256_k[64] =
            {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
             0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
             0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
             0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
             0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
             0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
             0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
             0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
             0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
             0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
             0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
             0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
             0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
             0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
             0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
             0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

/* SHA-256 functions */

void sha256_transf(sha256_ctx *ctx, const unsigned char *message,
                   unsigned int block_nb)
{
    uint32_t w[64];
    uint32_t wv[8];
    uint32_t t1, t2;
    const unsigned char *sub_block;
    int i;

    int j;

    for (i = 0; i < (int) block_nb; i++) {
        sub_block = message + (i << 6);

        for (j = 0; j < 16; j++) {
            PACK32(&sub_block[j << 2], &w[j]);
        }

        for (j = 16; j < 64; j++) {
            SHA256_SCR(j);
        }

        for (j = 0; j < 8; j++) {
            wv[j] = ctx->h[j];
        }

        for (j = 0; j < 64; j++) {
            t1 = wv[7] + SHA256_F2(wv[4]) + CH(wv[4], wv[5], wv[6])
                + sha256_k[j] + w[j];
            t2 = SHA256_F1(wv[0]) + MAJ(wv[0], wv[1], wv[2]);
            wv[7] = wv[6];
            wv[6] = wv[5];
            wv[5] = wv[4];
            wv[4] = wv[3] + t1;
            wv[3] = wv[2];
            wv[2] = wv[1];
            wv[1] = wv[0];
            wv[0] = t1 + t2;
        }

        for (j = 0; j < 8; j++) {
            ctx->h[j] += wv[j];
        }
    }
}

void sha256(const unsigned char *message, unsigned int len, unsigned char *digest)
{
    sha256_ctx ctx;

    sha256_init(&ctx);
    sha256_update(&ctx, message, len);
    sha256_final(&ctx, digest);
}

void sha256_init(sha256_ctx *ctx)
{
    int i;
    for (i = 0; i < 8; i++) {
        ctx->h[i] = sha256_h0[i];
    }

    ctx->len = 0;
    ctx->tot_len = 0;
}

#ifdef USE_CRYPTO_EXT
static __attribute__ ((target("+crypto")))
void SHA256_Transform(uint32_t state[8], const uint8_t data[64], uint32_t W[64],
                 uint32_t S[8])
{
	uint32x4_t STATE0, STATE1, ABEF_SAVE, CDGH_SAVE;
	uint32x4_t MSG0, MSG1, MSG2, MSG3;
	uint32x4_t TMP0, TMP1, TMP2;

	/* Hack for single block */
	unsigned int length = 64;

	/* Load state */
	STATE0 = vld1q_u32(&state[0]);
	STATE1 = vld1q_u32(&state[4]);

	while (length >= 64)
	{
		/* Save state */
		ABEF_SAVE = STATE0;
		CDGH_SAVE = STATE1;

		/* Load message */
		MSG0 = vld1q_u32((const uint32_t *)(data +  0));
		MSG1 = vld1q_u32((const uint32_t *)(data + 16));
		MSG2 = vld1q_u32((const uint32_t *)(data + 32));
		MSG3 = vld1q_u32((const uint32_t *)(data + 48));

		/* Reverse for little endian */
		MSG0 = vreinterpretq_u32_u8(vrev32q_u8(vreinterpretq_u8_u32(MSG0)));
		MSG1 = vreinterpretq_u32_u8(vrev32q_u8(vreinterpretq_u8_u32(MSG1)));
		MSG2 = vreinterpretq_u32_u8(vrev32q_u8(vreinterpretq_u8_u32(MSG2)));
		MSG3 = vreinterpretq_u32_u8(vrev32q_u8(vreinterpretq_u8_u32(MSG3)));

		TMP0 = vaddq_u32(MSG0, vld1q_u32(&sha256_k[0x00]));

		/* Rounds 0-3 */
		MSG0 = vsha256su0q_u32(MSG0, MSG1);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG1, vld1q_u32(&sha256_k[0x04]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG0 = vsha256su1q_u32(MSG0, MSG2, MSG3);

		/* Rounds 4-7 */
		MSG1 = vsha256su0q_u32(MSG1, MSG2);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG2, vld1q_u32(&sha256_k[0x08]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG1 = vsha256su1q_u32(MSG1, MSG3, MSG0);

		/* Rounds 8-11 */
		MSG2 = vsha256su0q_u32(MSG2, MSG3);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG3, vld1q_u32(&sha256_k[0x0c]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG2 = vsha256su1q_u32(MSG2, MSG0, MSG1);

		/* Rounds 12-15 */
		MSG3 = vsha256su0q_u32(MSG3, MSG0);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG0, vld1q_u32(&sha256_k[0x10]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG3 = vsha256su1q_u32(MSG3, MSG1, MSG2);

		/* Rounds 16-19 */
		MSG0 = vsha256su0q_u32(MSG0, MSG1);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG1, vld1q_u32(&sha256_k[0x14]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG0 = vsha256su1q_u32(MSG0, MSG2, MSG3);

		/* Rounds 20-23 */
		MSG1 = vsha256su0q_u32(MSG1, MSG2);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG2, vld1q_u32(&sha256_k[0x18]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG1 = vsha256su1q_u32(MSG1, MSG3, MSG0);

		/* Rounds 24-27 */
		MSG2 = vsha256su0q_u32(MSG2, MSG3);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG3, vld1q_u32(&sha256_k[0x1c]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG2 = vsha256su1q_u32(MSG2, MSG0, MSG1);

		/* Rounds 28-31 */
		MSG3 = vsha256su0q_u32(MSG3, MSG0);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG0, vld1q_u32(&sha256_k[0x20]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG3 = vsha256su1q_u32(MSG3, MSG1, MSG2);

		/* Rounds 32-35 */
		MSG0 = vsha256su0q_u32(MSG0, MSG1);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG1, vld1q_u32(&sha256_k[0x24]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG0 = vsha256su1q_u32(MSG0, MSG2, MSG3);

		/* Rounds 36-39 */
		MSG1 = vsha256su0q_u32(MSG1, MSG2);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG2, vld1q_u32(&sha256_k[0x28]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG1 = vsha256su1q_u32(MSG1, MSG3, MSG0);

		/* Rounds 40-43 */
		MSG2 = vsha256su0q_u32(MSG2, MSG3);
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG3, vld1q_u32(&sha256_k[0x2c]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);
		MSG2 = vsha256su1q_u32(MSG2, MSG0, MSG1);

		/* Rounds 44-47 */
		MSG3 = vsha256su0q_u32(MSG3, MSG0);
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG0, vld1q_u32(&sha256_k[0x30]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);
		MSG3 = vsha256su1q_u32(MSG3, MSG1, MSG2);

		/* Rounds 48-51 */
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG1, vld1q_u32(&sha256_k[0x34]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);

		/* Rounds 52-55 */
		TMP2 = STATE0;
		TMP0 = vaddq_u32(MSG2, vld1q_u32(&sha256_k[0x38]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);

		/* Rounds 56-59 */
		TMP2 = STATE0;
		TMP1 = vaddq_u32(MSG3, vld1q_u32(&sha256_k[0x3c]));
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP0);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP0);

		/* Rounds 60-63 */
		TMP2 = STATE0;
		STATE0 = vsha256hq_u32(STATE0, STATE1, TMP1);
		STATE1 = vsha256h2q_u32(STATE1, TMP2, TMP1);

		/* Combine state */
		STATE0 = vaddq_u32(STATE0, ABEF_SAVE);
		STATE1 = vaddq_u32(STATE1, CDGH_SAVE);

		data += 64;
		length -= 64;
	}

	/* Save state */
	vst1q_u32(&state[0], STATE0);
	vst1q_u32(&state[4], STATE1);
}

static const uint8_t PAD[64] = { 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define STORE64_BE(DST, W) store64_be((DST), (W))
static inline void
store64_be(uint8_t dst[8], uint64_t w)
{
#ifdef NATIVE_BIG_ENDIAN
    memcpy(dst, &w, sizeof w);
#else
    dst[7] = (uint8_t) w; w >>= 8;
    dst[6] = (uint8_t) w; w >>= 8;
    dst[5] = (uint8_t) w; w >>= 8;
    dst[4] = (uint8_t) w; w >>= 8;
    dst[3] = (uint8_t) w; w >>= 8;
    dst[2] = (uint8_t) w; w >>= 8;
    dst[1] = (uint8_t) w; w >>= 8;
    dst[0] = (uint8_t) w;
#endif
}

static void
SHA256_Pad(sha256_ctx *ctx, uint32_t tmp32[64 + 8])
{
    unsigned int r;
    unsigned int i;

    r = (unsigned int) ((ctx->len >> 3) & 0x3f);
    if (r < 56) {
        for (i = 0; i < 56 - r; i++) {
            ctx->block[r + i] = PAD[i];
        }
    } else {
        for (i = 0; i < 64 - r; i++) {
            ctx->block[r + i] = PAD[i];
        }
        SHA256_Transform(ctx->h, ctx->block, &tmp32[0], &tmp32[64]);
        memset(&ctx->block[0], 0, 56);
    }
    STORE64_BE(&ctx->block[56], ctx->len);
    SHA256_Transform(ctx->h, ctx->block, &tmp32[0], &tmp32[64]);
}

#define sodium_memzero(a, b)
int ex_sha256_update(sha256_ctx *ctx, const unsigned char *in, unsigned long long inlen)
{
    uint32_t           tmp32[64 + 8];
    unsigned long long i;
    unsigned long long r;

    if (inlen <= 0U) {
        return 0;
    }
    r = (unsigned long long) ((ctx->len >> 3) & 0x3f);

    ctx->len += ((uint64_t) inlen) << 3;
    if (inlen < 64 - r) {
        for (i = 0; i < inlen; i++) {
            ctx->block[r + i] = in[i];
        }
        return 0;
    }
    for (i = 0; i < 64 - r; i++) {
        ctx->block[r + i] = in[i];
    }
    SHA256_Transform(ctx->h, ctx->block, &tmp32[0], &tmp32[64]);
    in += 64 - r;
    inlen -= 64 - r;

    while (inlen >= 64) {
        SHA256_Transform(ctx->h, in, &tmp32[0], &tmp32[64]);
        in += 64;
        inlen -= 64;
    }
    inlen &= 63;
    for (i = 0; i < inlen; i++) {
        ctx->block[i] = in[i];
    }
    sodium_memzero((void *) tmp32, sizeof tmp32);

    return 0;
}

#define STORE32_BE(DST, W) store32_be((DST), (W))
static inline void
store32_be(uint8_t dst[4], uint32_t w)
{
#ifdef NATIVE_BIG_ENDIAN
    memcpy(dst, &w, sizeof w);
#else
    dst[3] = (uint8_t) w; w >>= 8;
    dst[2] = (uint8_t) w; w >>= 8;
    dst[1] = (uint8_t) w; w >>= 8;
    dst[0] = (uint8_t) w;
#endif
}

static void
be32enc_vect(unsigned char *dst, const uint32_t *src, size_t len)
{
    size_t i;

    for (i = 0; i < len / 4; i++) {
        STORE32_BE(dst + i * 4, src[i]);
    }
}
#endif

#ifdef USE_CRYPTO_EXT
void sha256_update(sha256_ctx *ctx, const unsigned char *message,
                   unsigned int len)
{
	ex_sha256_update(ctx, message, len);
}
#else
void sha256_update(sha256_ctx *ctx, const unsigned char *message,
                   unsigned int len)
{
    unsigned int block_nb;
    unsigned int new_len, rem_len, tmp_len;
    const unsigned char *shifted_message;

    tmp_len = SHA256_BLOCK_SIZE - ctx->len;
    rem_len = len < tmp_len ? len : tmp_len;

    memcpy(&ctx->block[ctx->len], message, rem_len);

    if (ctx->len + len < SHA256_BLOCK_SIZE) {
        ctx->len += len;
        return;
    }

    new_len = len - rem_len;
    block_nb = new_len / SHA256_BLOCK_SIZE;

    shifted_message = message + rem_len;

    sha256_transf(ctx, ctx->block, 1);
    sha256_transf(ctx, shifted_message, block_nb);

    rem_len = new_len % SHA256_BLOCK_SIZE;

    memcpy(ctx->block, &shifted_message[block_nb << 6],
           rem_len);

    ctx->len = rem_len;
    ctx->tot_len += (block_nb + 1) << 6;
}
#endif

#ifdef USE_CRYPTO_EXT
void sha256_final(sha256_ctx *ctx, unsigned char *digest)
{
    uint32_t tmp32[64 + 8];

    SHA256_Pad(ctx, tmp32);
    be32enc_vect(digest, ctx->h, 32);
    sodium_memzero((void *) tmp32, sizeof tmp32);
    sodium_memzero((void *) ctx, sizeof *ctx);
}
#else
void sha256_final(sha256_ctx *ctx, unsigned char *digest)
{
    unsigned int block_nb;
    unsigned int pm_len;
    unsigned int len_b;

    int i;

    block_nb = (1 + ((SHA256_BLOCK_SIZE - 9)
                     < (ctx->len % SHA256_BLOCK_SIZE)));

    len_b = (ctx->tot_len + ctx->len) << 3;
    pm_len = block_nb << 6;

    memset(ctx->block + ctx->len, 0, pm_len - ctx->len);
    ctx->block[ctx->len] = 0x80;
    UNPACK32(len_b, ctx->block + pm_len - 4);

    sha256_transf(ctx, ctx->block, block_nb);

    for (i = 0 ; i < 8; i++) {
        UNPACK32(ctx->h[i], &digest[i << 2]);
    }
}
#endif
