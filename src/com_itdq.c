/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#include "com_def.h"
#include "com_tbl.h"

int com_get_inverse_trans_shift(int log2_size, int type, int bit_depth)
{
#if PARTITIONING_OPT
    assert(log2_size <= MAX_TR_LOG2);
#else
    assert(log2_size <= 6);
#endif
    return ((type == 0) ? 5 : (15 + 5 - bit_depth));
}

/******************   DCT-2   ******************************************/

static void itx_dct2_pb2(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    int j;
    int E, O;
    int add = shift == 0 ? 0 : 1<<(shift-1);

    for(j = 0; j < line; j++)
    {
        /* E and O */
        E = src[0 * line + j] + src[1 * line + j];
        O = src[0 * line + j] - src[1 * line + j];
        dst[j * 2 + 0] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (com_tbl_tm2[DCT2][0][0] * E + add) >> shift);
        dst[j * 2 + 1] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (com_tbl_tm2[DCT2][1][0] * O + add) >> shift);
    }
}

static void itx_dct2_pb4(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    int j;
    int E[2], O[2];
    int add = 1 << (shift - 1);

    for(j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        O[0] = com_tbl_tm4[DCT2][1][0] * src[1 * line + j] + com_tbl_tm4[DCT2][3][0] * src[3 * line + j];
        O[1] = com_tbl_tm4[DCT2][1][1] * src[1 * line + j] + com_tbl_tm4[DCT2][3][1] * src[3 * line + j];
        E[0] = com_tbl_tm4[DCT2][0][0] * src[0 * line + j] + com_tbl_tm4[DCT2][2][0] * src[2 * line + j];
        E[1] = com_tbl_tm4[DCT2][0][1] * src[0 * line + j] + com_tbl_tm4[DCT2][2][1] * src[2 * line + j];
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        dst[j * 4 + 0] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[0] + O[0] + add) >> shift);
        dst[j * 4 + 1] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[1] + O[1] + add) >> shift);
        dst[j * 4 + 2] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[1] - O[1] + add) >> shift);
        dst[j * 4 + 3] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[0] - O[0] + add) >> shift);
    }
}

static void itx_dct2_pb8(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for(k = 0; k < 4; k++)
        {
            O[k] = com_tbl_tm8[DCT2][1][k] * src[1 * line + j] + com_tbl_tm8[DCT2][3][k] * src[3 * line + j] + com_tbl_tm8[DCT2][5][k] * src[5 * line + j] + com_tbl_tm8[DCT2][7][k] * src[7 * line + j];
        }
        EO[0] = com_tbl_tm8[DCT2][2][0] * src[2 * line + j] + com_tbl_tm8[DCT2][6][0] * src[6 * line + j];
        EO[1] = com_tbl_tm8[DCT2][2][1] * src[2 * line + j] + com_tbl_tm8[DCT2][6][1] * src[6 * line + j];
        EE[0] = com_tbl_tm8[DCT2][0][0] * src[0 * line + j] + com_tbl_tm8[DCT2][4][0] * src[4 * line + j];
        EE[1] = com_tbl_tm8[DCT2][0][1] * src[0 * line + j] + com_tbl_tm8[DCT2][4][1] * src[4 * line + j];
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        E[0] = EE[0] + EO[0];
        E[3] = EE[0] - EO[0];
        E[1] = EE[1] + EO[1];
        E[2] = EE[1] - EO[1];
        for(k = 0; k < 4; k++)
        {
            dst[j * 8 + k] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[k] + O[k] + add) >> shift);
            dst[j * 8 + k + 4] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[3 - k] - O[3 - k] + add) >> shift);
        }
    }
}

static void itx_dct2_pb16(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];
    int add = 1 << (shift - 1);

    for(j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for(k = 0; k < 8; k++)
        {
            O[k] = com_tbl_tm16[DCT2][1][k] * src[1 * line + j] + com_tbl_tm16[DCT2][3][k] * src[3 * line + j] + com_tbl_tm16[DCT2][5][k] * src[5 * line + j] + com_tbl_tm16[DCT2][7][k] * src[7 * line + j] +
                   com_tbl_tm16[DCT2][9][k] * src[9 * line + j] + com_tbl_tm16[DCT2][11][k] * src[11 * line + j] + com_tbl_tm16[DCT2][13][k] * src[13 * line + j] + com_tbl_tm16[DCT2][15][k] * src[15 * line + j];
        }
        for(k = 0; k < 4; k++)
        {
            EO[k] = com_tbl_tm16[DCT2][2][k] * src[2 * line + j] + com_tbl_tm16[DCT2][6][k] * src[6 * line + j] + com_tbl_tm16[DCT2][10][k] * src[10 * line + j] + com_tbl_tm16[DCT2][14][k] * src[14 * line + j];
        }
        EEO[0] = com_tbl_tm16[DCT2][4][0] * src[4 * line + j] + com_tbl_tm16[DCT2][12][0] * src[12 * line + j];
        EEE[0] = com_tbl_tm16[DCT2][0][0] * src[0 * line + j] + com_tbl_tm16[DCT2][8][0] * src[8 * line + j];
        EEO[1] = com_tbl_tm16[DCT2][4][1] * src[4 * line + j] + com_tbl_tm16[DCT2][12][1] * src[12 * line + j];
        EEE[1] = com_tbl_tm16[DCT2][0][1] * src[0 * line + j] + com_tbl_tm16[DCT2][8][1] * src[8 * line + j];
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        for(k = 0; k < 2; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 2] = EEE[1 - k] - EEO[1 - k];
        }
        for(k = 0; k < 4; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 4] = EE[3 - k] - EO[3 - k];
        }
        for(k = 0; k < 8; k++)
        {
            dst[j * 16 + k] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[k] + O[k] + add) >> shift);
            dst[j * 16 + k + 8] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[7 - k] - O[7 - k] + add) >> shift);
        }
    }
}

static void itx_dct2_pb32(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];
    int add = 1 << (shift - 1);

    for(j = 0; j < line; j++)
    {
        for(k = 0; k < 16; k++)
        {
            O[k] = com_tbl_tm32[DCT2][ 1][k]*src[ 1*line+j] + \
                   com_tbl_tm32[DCT2][ 3][k]*src[ 3*line+j] + \
                   com_tbl_tm32[DCT2][ 5][k]*src[ 5*line+j] + \
                   com_tbl_tm32[DCT2][ 7][k]*src[ 7*line+j] + \
                   com_tbl_tm32[DCT2][ 9][k]*src[ 9*line+j] + \
                   com_tbl_tm32[DCT2][11][k]*src[11*line+j] + \
                   com_tbl_tm32[DCT2][13][k]*src[13*line+j] + \
                   com_tbl_tm32[DCT2][15][k]*src[15*line+j] + \
                   com_tbl_tm32[DCT2][17][k]*src[17*line+j] + \
                   com_tbl_tm32[DCT2][19][k]*src[19*line+j] + \
                   com_tbl_tm32[DCT2][21][k]*src[21*line+j] + \
                   com_tbl_tm32[DCT2][23][k]*src[23*line+j] + \
                   com_tbl_tm32[DCT2][25][k]*src[25*line+j] + \
                   com_tbl_tm32[DCT2][27][k]*src[27*line+j] + \
                   com_tbl_tm32[DCT2][29][k]*src[29*line+j] + \
                   com_tbl_tm32[DCT2][31][k]*src[31*line+j];
        }
        for(k = 0; k < 8; k++)
        {
            EO[k] = com_tbl_tm32[DCT2][ 2][k]*src[ 2*line+j] + \
                    com_tbl_tm32[DCT2][ 6][k]*src[ 6*line+j] + \
                    com_tbl_tm32[DCT2][10][k]*src[10*line+j] + \
                    com_tbl_tm32[DCT2][14][k]*src[14*line+j] + \
                    com_tbl_tm32[DCT2][18][k]*src[18*line+j] + \
                    com_tbl_tm32[DCT2][22][k]*src[22*line+j] + \
                    com_tbl_tm32[DCT2][26][k]*src[26*line+j] + \
                    com_tbl_tm32[DCT2][30][k]*src[30*line+j];
        }
        for(k = 0; k < 4; k++)
        {
            EEO[k] = com_tbl_tm32[DCT2][ 4][k]*src[ 4*line+j] + \
                     com_tbl_tm32[DCT2][12][k]*src[12*line+j] + \
                     com_tbl_tm32[DCT2][20][k]*src[20*line+j] + \
                     com_tbl_tm32[DCT2][28][k]*src[28*line+j];
        }
        EEEO[0] = com_tbl_tm32[DCT2][8][0]*src[8*line+j] + com_tbl_tm32[DCT2][24][0]*src[24*line+j];
        EEEO[1] = com_tbl_tm32[DCT2][8][1]*src[8*line+j] + com_tbl_tm32[DCT2][24][1]*src[24*line+j];
        EEEE[0] = com_tbl_tm32[DCT2][0][0]*src[0*line+j] + com_tbl_tm32[DCT2][16][0]*src[16*line+j];
        EEEE[1] = com_tbl_tm32[DCT2][0][1]*src[0*line+j] + com_tbl_tm32[DCT2][16][1]*src[16*line+j];
        EEE[0] = EEEE[0] + EEEO[0];
        EEE[3] = EEEE[0] - EEEO[0];
        EEE[1] = EEEE[1] + EEEO[1];
        EEE[2] = EEEE[1] - EEEO[1];
        for (k=0; k<4; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k+4] = EEE[3-k] - EEO[3-k];
        }
        for (k=0; k<8; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k+8] = EE[7-k] - EO[7-k];
        }
        for (k=0; k<16; k++)
        {
            dst[j*32+k]    = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[k] + O[k] + add)>>shift);
            dst[j*32+k+16] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[15-k] - O[15-k] + add)>>shift);
        }
    }
}

static void itx_dct2_pb64(s16 *src, s16 *dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    const int tx_size = 64;
    const s8 *tm = com_tbl_tm64[DCT2][0];
    int j, k;
    int E[32], O[32];
    int EE[16], EO[16];
    int EEE[8], EEO[8];
    int EEEE[4], EEEO[4];
    int EEEEE[2], EEEEO[2];
    int add = 1 << (shift - 1);

    for(j = 0; j < line; j++)
    {
        for(k = 0; k < 32; k++)
        {
            O[k] = tm[ 1*64+k]*src[   line] + tm[ 3*64+k]*src[  3*line ] + tm[ 5*64+k]*src[ 5*line] + tm[ 7*64+k]*src[ 7*line] +
                   tm[ 9*64+k]*src[ 9*line] + tm[11*64+k]*src[ 11*line ] + tm[13*64+k]*src[13*line] + tm[15*64+k]*src[15*line] +
                   tm[17*64+k]*src[17*line] + tm[19*64+k]*src[ 19*line ] + tm[21*64+k]*src[21*line] + tm[23*64+k]*src[23*line] +
                   tm[25*64+k]*src[25*line] + tm[27*64+k]*src[ 27*line ] + tm[29*64+k]*src[29*line] + tm[31*64+k]*src[31*line] +
                   tm[33*64+k]*src[33*line] + tm[35*64+k]*src[ 35*line ] + tm[37*64+k]*src[37*line] + tm[39*64+k]*src[39*line] +
                   tm[41*64+k]*src[41*line] + tm[43*64+k]*src[ 43*line ] + tm[45*64+k]*src[45*line] + tm[47*64+k]*src[47*line] +
                   tm[49*64+k]*src[49*line] + tm[51*64+k]*src[ 51*line ] + tm[53*64+k]*src[53*line] + tm[55*64+k]*src[55*line] +
                   tm[57*64+k]*src[57*line] + tm[59*64+k]*src[ 59*line ] + tm[61*64+k]*src[61*line] + tm[63*64+k]*src[63*line];
        }
        for(k = 0; k < 16; k++)
        {
            EO[k] = tm[ 2*64+k]*src[ 2*line] + tm[ 6*64+k]*src[ 6*line] + tm[10*64+k]*src[10*line] + tm[14*64+k]*src[14*line] +
                    tm[18*64+k]*src[18*line] + tm[22*64+k]*src[22*line] + tm[26*64+k]*src[26*line] + tm[30*64+k]*src[30*line] +
                    tm[34*64+k]*src[34*line] + tm[38*64+k]*src[38*line] + tm[42*64+k]*src[42*line] + tm[46*64+k]*src[46*line] +
                    tm[50*64+k]*src[50*line] + tm[54*64+k]*src[54*line] + tm[58*64+k]*src[58*line] + tm[62*64+k]*src[62*line];
        }
        for(k = 0; k < 8; k++)
        {
            EEO[k] = tm[ 4*64+k]*src[ 4*line] + tm[12*64+k]*src[12*line] + tm[20*64+k]*src[20*line] + tm[28*64+k]*src[28*line] +
                     tm[36*64+k]*src[36*line] + tm[44*64+k]*src[44*line] + tm[52*64+k]*src[52*line] + tm[60*64+k]*src[60*line];
        }
        for (k=0; k<4; k++)
        {
            EEEO[k] = tm[8*64+k]*src[8*line] + tm[24*64+k]*src[24*line] + tm[40*64+k]*src[40*line] + tm[56*64+k]*src[56*line];
        }
        EEEEO[0] = tm[16*64+0]*src[16*line] + tm[48*64+0]*src[48*line];
        EEEEO[1] = tm[16*64+1]*src[16*line] + tm[48*64+1]*src[48*line];
        EEEEE[0] = tm[ 0*64+0]*src[ 0     ] + tm[32*64+0]*src[32*line];
        EEEEE[1] = tm[ 0*64+1]*src[ 0     ] + tm[32*64+1]*src[32*line];
        for(k = 0; k < 2; k++)
        {
            EEEE[k] = EEEEE[k] + EEEEO[k];
            EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
        }
        for(k = 0; k < 4; k++)
        {
            EEE[k] = EEEE[k] + EEEO[k];
            EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
        }
        for(k = 0; k < 8; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 8] = EEE[7 - k] - EEO[7 - k];
        }
        for(k = 0; k < 16; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 16] = EE[15 - k] - EO[15 - k];
        }
        for(k = 0; k < 32; k++)
        {
            dst[k] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[k] + O[k] + add) >> shift);
            dst[k + 32] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[31 - k] - O[31 - k] + add) >> shift);
        }
        src++;
        dst += tx_size;
    }
}

#if PARTITIONING_OPT
static void itx_dct2_pb128(s16* src, s16* dst, int shift, int line, int max_tr_val, int min_tr_val)
{
    const int tx_size = 128;
    const s8* tm = com_tbl_tm128[0];
    int j, k;
    int E[64], O[64];
    int EE[32], EO[32];
    int EEE[16], EEO[16];
    int EEEE[8], EEEO[8];
    int EEEEE[4], EEEEO[4];
    int EEEEEE[2], EEEEEO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        for (k = 0; k < 64; k++)
        {
            O[k] = tm[  1*128+k]*src[  1*line] + tm[  3*128+k]*src[  3*line] + tm[  5*128+k]*src[  5*line] + tm[  7*128+k]*src[  7*line] + 
                   tm[  9*128+k]*src[  9*line] + tm[ 11*128+k]*src[ 11*line] + tm[ 13*128+k]*src[ 13*line] + tm[ 15*128+k]*src[ 15*line] + 
                   tm[ 17*128+k]*src[ 17*line] + tm[ 19*128+k]*src[ 19*line] + tm[ 21*128+k]*src[ 21*line] + tm[ 23*128+k]*src[ 23*line] + 
                   tm[ 25*128+k]*src[ 25*line] + tm[ 27*128+k]*src[ 27*line] + tm[ 29*128+k]*src[ 29*line] + tm[ 31*128+k]*src[ 31*line] +
                   tm[ 33*128+k]*src[ 33*line] + tm[ 35*128+k]*src[ 35*line] + tm[ 37*128+k]*src[ 37*line] + tm[ 39*128+k]*src[ 39*line] + 
                   tm[ 41*128+k]*src[ 41*line] + tm[ 43*128+k]*src[ 43*line] + tm[ 45*128+k]*src[ 45*line] + tm[ 47*128+k]*src[ 47*line] + 
                   tm[ 49*128+k]*src[ 49*line] + tm[ 51*128+k]*src[ 51*line] + tm[ 53*128+k]*src[ 53*line] + tm[ 55*128+k]*src[ 55*line] + 
                   tm[ 57*128+k]*src[ 57*line] + tm[ 59*128+k]*src[ 59*line] + tm[ 61*128+k]*src[ 61*line] + tm[ 63*128+k]*src[ 63*line] +
                   tm[ 65*128+k]*src[ 65*line] + tm[ 67*128+k]*src[ 67*line] + tm[ 69*128+k]*src[ 69*line] + tm[ 71*128+k]*src[ 71*line] +
                   tm[ 73*128+k]*src[ 73*line] + tm[ 75*128+k]*src[ 75*line] + tm[ 77*128+k]*src[ 77*line] + tm[ 79*128+k]*src[ 79*line] + 
                   tm[ 81*128+k]*src[ 81*line] + tm[ 83*128+k]*src[ 83*line] + tm[ 85*128+k]*src[ 85*line] + tm[ 87*128+k]*src[ 87*line] + 
                   tm[ 89*128+k]*src[ 89*line] + tm[ 91*128+k]*src[ 91*line] + tm[ 93*128+k]*src[ 93*line] + tm[ 95*128+k]*src[ 95*line] +
                   tm[ 97*128+k]*src[ 97*line] + tm[ 99*128+k]*src[ 99*line] + tm[101*128+k]*src[101*line] + tm[103*128+k]*src[103*line] +
                   tm[105*128+k]*src[105*line] + tm[107*128+k]*src[107*line] + tm[109*128+k]*src[109*line] + tm[111*128+k]*src[111*line] +
                   tm[113*128+k]*src[113*line] + tm[115*128+k]*src[115*line] + tm[117*128+k]*src[117*line] + tm[119*128+k]*src[119*line] +
                   tm[121*128+k]*src[121*line] + tm[123*128+k]*src[123*line] + tm[125*128+k]*src[125*line] + tm[127*128+k]*src[127*line];
        }
        for (k = 0; k < 32; k++)
        {
            EO[k] = tm[  2*128+k]*src[  2*line] + tm[  6*128+k]*src[  6*line] + tm[ 10*128+k]*src[ 10*line] + tm[ 14*128+k]*src[ 14*line] + 
                    tm[ 18*128+k]*src[ 18*line] + tm[ 22*128+k]*src[ 22*line] + tm[ 26*128+k]*src[ 26*line] + tm[ 30*128+k]*src[ 30*line] + 
                    tm[ 34*128+k]*src[ 34*line] + tm[ 38*128+k]*src[ 38*line] + tm[ 42*128+k]*src[ 42*line] + tm[ 46*128+k]*src[ 46*line] + 
                    tm[ 50*128+k]*src[ 50*line] + tm[ 54*128+k]*src[ 54*line] + tm[ 58*128+k]*src[ 58*line] + tm[ 62*128+k]*src[ 62*line] + 
                    tm[ 66*128+k]*src[ 66*line] + tm[ 70*128+k]*src[ 70*line] + tm[ 74*128+k]*src[ 74*line] + tm[ 78*128+k]*src[ 78*line] + 
                    tm[ 82*128+k]*src[ 82*line] + tm[ 86*128+k]*src[ 86*line] + tm[ 90*128+k]*src[ 90*line] + tm[ 94*128+k]*src[ 94*line] + 
                    tm[ 98*128+k]*src[ 98*line] + tm[102*128+k]*src[102*line] + tm[106*128+k]*src[106*line] + tm[110*128+k]*src[110*line] +
                    tm[114*128+k]*src[114*line] + tm[118*128+k]*src[118*line] + tm[122*128+k]*src[122*line] + tm[126*128+k]*src[126*line];
        }
        for (k = 0; k < 16; k++)
        {
            EEO[k] = tm[  4*128+k]*src[  4*line] + tm[ 12*128+k]*src[ 12*line] + tm[ 20*128+k]*src[ 20*line] + tm[ 28*128+k]*src[ 28*line] + 
                     tm[ 36*128+k]*src[ 36*line] + tm[ 44*128+k]*src[ 44*line] + tm[ 52*128+k]*src[ 52*line] + tm[ 60*128+k]*src[ 60*line] + 
                     tm[ 68*128+k]*src[ 68*line] + tm[ 76*128+k]*src[ 76*line] + tm[ 84*128+k]*src[ 84*line] + tm[ 92*128+k]*src[ 92*line] + 
                     tm[100*128+k]*src[100*line] + tm[108*128+k]*src[108*line] + tm[116*128+k]*src[116*line] + tm[124*128+k]*src[124*line];
        }
        for (k = 0; k < 8; k++)
        {
            EEEO[k] =  tm[  8*128+k]*src[  8*line] + tm[ 24*128+k]*src[ 24*line] + tm[ 40*128+k]*src[ 40*line] + tm[ 56*128+k]*src[ 56*line] + 
                       tm[ 72*128+k]*src[ 72*line] + tm[ 88*128+k]*src[ 88*line] + tm[104*128+k]*src[104*line] + tm[120*128+k]*src[120*line];
        }
        for (k = 0; k < 4; k++)
        {
            EEEEO[k] = tm[ 16*128+k]*src[ 16*line] + tm[ 48*128+k]*src[ 48*line] + tm[80*128+k]*src[80*line] + tm[112*128+k]*src[112*line];
        }
        EEEEEO[0] = tm[32*128+0]*src[32*line] + tm[96*128+0]*src[96*line];
        EEEEEO[1] = tm[32*128+1]*src[32*line] + tm[96*128+1]*src[96*line];
        EEEEEE[0] = tm[ 0*128+0]*src[ 0*line] + tm[64*128+0]*src[64*line];
        EEEEEE[1] = tm[ 0*128+1]*src[ 0*line] + tm[64*128+1]*src[64*line];
        for (k = 0; k < 2; k++)
        {
            EEEEE[k] = EEEEEE[k] + EEEEEO[k];
            EEEEE[k + 2] = EEEEEE[1 - k] - EEEEEO[1-k];
        }
        for (k = 0; k < 4; k++)
        {
            EEEE[k] = EEEEE[k] + EEEEO[k];
            EEEE[k + 4] = EEEEE[3 - k] - EEEEO[3-k];
        }
        for (k = 0; k < 8; k++)
        {
            EEE[k] = EEEE[k] + EEEO[k];
            EEE[k + 8] = EEEE[7 - k] - EEEO[7-k];
        }
        for (k = 0; k < 16; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 16] = EEE[15 - k] - EEO[15-k];
        }
        for (k = 0; k < 32; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 32] = EE[31 - k] - EO[31-k];
        }
        for (k = 0; k < 64; k++)
        {
            dst[k] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[k] + O[k] + add) >> shift);
            dst[k + 64] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (E[63 - k] - O[63 - k] + add) >> shift);
        }
        src++;
        dst += tx_size;
    }
}
#endif

/******************   DCT-8   ******************************************/

void itx_dct8_pb4(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input tmp, output block
{
    int i;
    int rnd_factor = 1 << (shift - 1);

    s8 *iT = com_tbl_tm4[DCT8][0];

    int c[4];
    const int  reducedLine = line;
    for (i = 0; i<reducedLine; i++)
    {
        // Intermediate Variables
        c[0] = coeff[0 * line] + coeff[3 * line];
        c[1] = coeff[2 * line] + coeff[0 * line];
        c[2] = coeff[3 * line] - coeff[2 * line];
        c[3] = iT[1] * coeff[1 * line];

        block[0] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift);
        block[1] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[1] * (coeff[0 * line] - coeff[2 * line] - coeff[3 * line]) + rnd_factor) >> shift);
        block[2] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift);
        block[3] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift);

        block += 4;
        coeff++;
    }
}

void itx_dct8_pb8(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 8;
    s8 *iT = com_tbl_tm8[DCT8][0];
    const int  reducedLine = line;
    const int  cutoff = 8;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dct8_pb16(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 16;
    s8 *iT = com_tbl_tm16[DCT8][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dct8_pb32(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 32;
    s8 *iT = com_tbl_tm32[DCT8][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dct8_pb64(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 64;
    s8 *iT = com_tbl_tm64[DCT8][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

/******************   DST-7   ******************************************/
void itx_dst7_pb4(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input tmp, output block
{
    int i, c[4];
    int rnd_factor = 1 << (shift - 1);
    s8 *iT = com_tbl_tm4[DST7][0];
    const int  reducedLine = line;

    for (i = 0; i<reducedLine; i++)
    {
        // Intermediate Variables

        c[0] = coeff[0 * line] + coeff[2 * line];
        c[1] = coeff[2 * line] + coeff[3 * line];
        c[2] = coeff[0 * line] - coeff[3 * line];
        c[3] = iT[2] * coeff[1 * line];

        block[0] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
        block[1] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);
        block[2] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[2] * (coeff[0 * line] - coeff[2 * line] + coeff[3 * line]) + rnd_factor) >> shift);
        block[3] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (iT[1] * c[0] + iT[0] * c[2] - c[3] + rnd_factor) >> shift);

        block += 4;
        coeff++;
    }
}

void itx_dst7_pb8(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 8;
    s8 *iT = com_tbl_tm8[DST7][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dst7_pb16(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 16;
    s8 *iT = com_tbl_tm16[DST7][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dst7_pb32(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 32;
    s8 *iT = com_tbl_tm32[DST7][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;

    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

void itx_dst7_pb64(s16 *coeff, s16 *block, int shift, int line, int max_tr_val, int min_tr_val)  // input block, output coeff
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);
    const int uiTrSize = 64;
    s8 *iT = com_tbl_tm64[DST7][0];
    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        for (j = 0; j<uiTrSize; j++)
        {
            iSum = 0;
            for (k = 0; k<cutoff; k++)
            {
                iSum += coeff[k*line] * iT[k*uiTrSize + j];
            }
            block[j] = (s16)COM_CLIP3(min_tr_val, max_tr_val, (int)(iSum + rnd_factor) >> shift);
        }
        block += uiTrSize;
        coeff++;
    }
}

typedef void(*COM_ITX)(s16 *coef, s16 *t, int shift, int line, int max_tr_val, int min_tr_val);
static COM_ITX tbl_itx[NUM_TRANS_TYPE][MAX_TR_LOG2] =
{
    {
        itx_dct2_pb2,
        itx_dct2_pb4,
        itx_dct2_pb8,
        itx_dct2_pb16,
        itx_dct2_pb32,
        itx_dct2_pb64
#if PARTITIONING_OPT
        , itx_dct2_pb128
#endif
    },
    {
        NULL,
        itx_dct8_pb4,
        itx_dct8_pb8,
        itx_dct8_pb16,
        itx_dct8_pb32,
        itx_dct8_pb64
    },
    {
        NULL,
        itx_dst7_pb4,
        itx_dst7_pb8,
        itx_dst7_pb16,
        itx_dst7_pb32,
        itx_dst7_pb64
    }
};

static void xCTr_4_1d_Inv_Vert(s16 *src, int i_src, s16 *dst, int i_dst, int shift)
{
    int i, j, k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c4_trans[k][i] * tmpSrc[k][j];
            }
            dst[i* i_dst + j] = (s16)COM_CLIP3(-32768, 32767, sum >> shift);
        }
    }
}

static void xCTr_4_1d_Inv_Hor(s16 *src, int i_src, s16 *dst, int i_dst, int shift, int bit_depth)
{
    int i, j, k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    int min_pixel = -(1 << bit_depth);
    int max_pixel = (1 << bit_depth) - 1;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c4_trans[k][i] * tmpSrc[j][k];
            }
            dst[j* i_dst + i] = (s16)COM_CLIP3(min_pixel, max_pixel, sum >> shift);
        }
    }
}

static void xTr2nd_8_1d_Inv_Vert(s16 *src, int i_src)
{
    int i, j, k, sum;
    int tmpSrc[4][4];
    int rnd_factor = 1 << (7 - 1);

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c8_trans[k][i] * tmpSrc[k][j];
            }
            src[i * i_src + j] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
        }
    }
}

static void xTr2nd_8_1d_Inv_Hor(s16 *src, int i_src)
{
    int i, j, k, sum;
    int tmpSrc[4][4];
    int rnd_factor = 1 << (7 - 1);

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c8_trans[k][i] * tmpSrc[j][k];
            }
            src[j* i_src + i] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
        }
    }
}

#if DEST
static void xTr2nd_8x_1d_Inv_Vert(s16* src, int i_src, int tuw)
{
	int i, j, k, sum;
	int tmpSrc[8][8];
	int rnd_factor = 1 << (7 - 1);

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < tuw; j++)
		{
			tmpSrc[i][j] = src[i * i_src + j];
		}
	}
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < tuw; j++)
		{
			sum = rnd_factor;
			for (k = 0; k < 8; k++)
			{
				sum += tab_d8_trans[k][i] * tmpSrc[k][j];
			}
			src[i * i_src + j] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
		}
	}
}

static void xTr2nd_8x_1d_Inv_Hor(s16* src, int i_src, int tuh)
{
	int i, j, k, sum;
	int tmpSrc[8][8];
	int rnd_factor = 1 << (7 - 1);

	for (i = 0; i < tuh; i++)
	{
		for (j = 0; j < 8; j++)
		{
			tmpSrc[i][j] = src[i * i_src + j];
		}
	}
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < tuh; j++)
		{
			sum = rnd_factor;
			for (k = 0; k < 8; k++)
			{
				sum += tab_d8_trans[k][i] * tmpSrc[j][k];
			}
			src[j * i_src + i] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
		}
	}
}
#endif

void com_itrans(COM_MODE *mode, int plane, int blk_idx, s16 *coef_dq, s16 *resi, int cu_width_log2, int cu_height_log2, int bit_depth, int secT_Ver_Hor, int use_alt4x4Trans
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
)
{
    s16 coef_temp[MAX_TR_DIM];

    if ((cu_width_log2 > MAX_TR_LOG2) || (cu_height_log2 > MAX_TR_LOG2))
    {
        assert(0);
    }
    else
    {
        int shift1 = com_get_inverse_trans_shift(cu_width_log2, 0, bit_depth);
        int shift2 = com_get_inverse_trans_shift(cu_height_log2, 1, bit_depth);
#if IST
#if IST_CHROMA
        if ((plane == Y_C || ((plane == U_C || plane == V_C) && mode->cu_mode == MODE_INTRA && !mode->sawp_flag && mode->ipm[0][1] == IPD_DM_C && channel == CHANNEL_LC)) && mode->ist_tu_flag &&
#else
        if (plane == Y_C && mode->ist_tu_flag && 
#endif
            (mode->cu_mode == MODE_INTRA || mode->cu_mode == MODE_INTER || mode->cu_mode == MODE_DIR) &&
#if ISTS
            mode->ph_ists_enable_flag == 0 &&
#endif
            mode->tb_part == SIZE_2Nx2N && cu_width_log2 < 6 && cu_height_log2 < 6)
        {
            int nTrIdxHor = DST7;
            int nTrIdxVer = DST7;
            int max_tr_val = (1 << MAX_TX_DYNAMIC_RANGE) - 1;
            int min_tr_val = -(1 << MAX_TX_DYNAMIC_RANGE);
            tbl_itx[nTrIdxVer][cu_height_log2 - 1](coef_dq, coef_temp, shift1, 1 << cu_width_log2, max_tr_val, min_tr_val);

            max_tr_val = (1 << bit_depth) - 1;
            min_tr_val = -(1 << bit_depth);
            tbl_itx[nTrIdxHor][cu_width_log2 - 1](coef_temp, resi, shift2, 1 << cu_height_log2, max_tr_val, min_tr_val);
            return;
        }
#endif
#if ISTS
#if TS_INTER
        if (plane == Y_C && mode->ph_ists_enable_flag && mode->tb_part == SIZE_2Nx2N && cu_width_log2 < 6 && cu_height_log2 < 6 &&
            ((mode->ist_tu_flag && (mode->cu_mode == MODE_INTRA || mode->cu_mode == MODE_IBC || (mode->cu_mode == MODE_INTER && mode->ph_ts_inter_enable_flag)))
#if SBT
            || (mode->sbt_info && mode->ph_ts_inter_enable_flag)
#endif
            ))
#else
        if (plane == Y_C && mode->ist_tu_flag &&
            mode->ph_ists_enable_flag && (mode->cu_mode == MODE_INTRA || mode->cu_mode == MODE_IBC) &&
            mode->tb_part == SIZE_2Nx2N && cu_width_log2 < 6 && cu_height_log2 < 6)
#endif
        {
            int shift = 15 - bit_depth - ((cu_width_log2 + cu_height_log2) >> 1);
            int stride_tu = (1 << cu_width_log2);
            int height_tu = (1 << cu_height_log2);
#if ETS
            if (mode->cu_mode == MODE_INTRA && mode->ist_tu_flag == 2)
            {
                int rnd_factor = 1 << (shift - 1);
                int half_h = height_tu >> 1;
                int half_w = stride_tu >> 1;
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        if ((i == half_h && j == (half_w - 1)) || (i == (half_h - 1) && j == half_w))
                        {
                            resi[i * stride_tu + j] = COM_CLIP3(-(1 << bit_depth), (1 << bit_depth) - 1,(s16)((coef_dq[i * stride_tu + j] + rnd_factor) >> shift));
                        }
                        else
                        {
                            resi[i * stride_tu + j] = COM_CLIP3(-(1 << bit_depth), (1 << bit_depth) - 1, (s16)((coef_dq[(height_tu - 1 - i) * stride_tu + (stride_tu - 1 - j)] + rnd_factor) >> shift));
                        }
                    }
                }
            }
            else
            {
                int rnd_factor = 1 << (shift - 1);
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        resi[i * stride_tu + j] = COM_CLIP3(-(1 << bit_depth), (1 << bit_depth) - 1, (s16)((coef_dq[i * stride_tu + j] + rnd_factor) >> shift));                    
                    }
                }
            }
#else
            if (shift >= 0)
            {
                int rnd_factor = 1 << (shift - 1);
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        resi[i * stride_tu + j] = (s16)((coef_dq[i * stride_tu + j] + rnd_factor) >> shift);
                    }
                }
            }
            else
            {
                shift = -shift;
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        resi[i * stride_tu + j] = (s16)((coef_dq[i * stride_tu + j]) << shift);
                    }
                }
            }
#endif
            return;
        }
#endif
        if (use_alt4x4Trans && cu_width_log2 == 2 && cu_height_log2 == 2)
        {
            s16 coef_temp2[16];
            xCTr_4_1d_Inv_Vert(coef_dq, 4, coef_temp2, 4, shift1);
            xCTr_4_1d_Inv_Hor(coef_temp2, 4, resi, 4, shift2 + 2, bit_depth);
        }
        else
        {
            int nTrIdxHor = DCT2, nTrIdxVer = DCT2;
            int stride_tu = (1 << cu_width_log2);
            if (secT_Ver_Hor & 1)
            {
#if DEST
#if DEST_PH
                if (cu_width_log2 > 2 && mode->ph_dest_enable_flag)
#else
                if (cu_width_log2 > 2)
#endif
                {
                    xTr2nd_8x_1d_Inv_Hor(coef_dq, stride_tu, (cu_height_log2 > 2) ? 8 : 4);
                }
                else
#endif
                xTr2nd_8_1d_Inv_Hor(coef_dq, stride_tu);
            }
            if (secT_Ver_Hor >> 1)
            {
#if DEST
#if DEST_PH
                if (cu_height_log2 > 2 && mode->ph_dest_enable_flag)
#else
                if (cu_height_log2 > 2)
#endif
                {
                    xTr2nd_8x_1d_Inv_Vert(coef_dq, stride_tu, (cu_width_log2 > 2) ? 8 : 4);
                }
                else
#endif
                xTr2nd_8_1d_Inv_Vert(coef_dq, stride_tu);
            }

            if (plane == Y_C && mode->tb_part == SIZE_NxN)
            {
                nTrIdxHor = com_tbl_subset_inter[blk_idx & 1];
                nTrIdxVer = com_tbl_subset_inter[blk_idx >> 1];
            }
#if SBT
            if( plane == Y_C && mode->sbt_info )
            {
                nTrIdxHor = mode->sbt_hor_trans;
                nTrIdxVer = mode->sbt_ver_trans;
            }
#endif

            int max_tr_val = (1 << MAX_TX_DYNAMIC_RANGE) - 1;
            int min_tr_val = -(1 << MAX_TX_DYNAMIC_RANGE);
            tbl_itx[nTrIdxVer][cu_height_log2 - 1](coef_dq, coef_temp, shift1, 1 << cu_width_log2, max_tr_val, min_tr_val);
            
            max_tr_val = (1 << bit_depth) - 1;
            min_tr_val = -(1 << bit_depth);
            tbl_itx[nTrIdxHor][cu_width_log2 - 1](coef_temp, resi, shift2, 1 << cu_height_log2, max_tr_val, min_tr_val);
        }
    }
}

static int get_transform_shift(const int bit_depth, const int tr_size_log2)
{
    return MAX_TX_DYNAMIC_RANGE - bit_depth - tr_size_log2;
}

static void com_dquant(int qp, s16 *coef, s16* coef_out, u8* wq_matrix[2], int log2_w, int log2_h, int bit_depth)
{
    int i, j;
    int w = 1 << log2_w;
    int h = 1 << log2_h;
    int wq_width;
    int idx_shift;
    int idx_step;
    int log2_size = (log2_w + log2_h) >> 1;
    int refix = (log2_w + log2_h) & 1;
    int scale = com_tbl_dq_scale[qp];
    int shift = com_tbl_dq_shift[qp] - get_transform_shift(bit_depth, log2_size) + 1; // +1 is used to compensate for the mismatching of shifts in quantization and inverse quantization
    int offset = (shift == 0) ? 0 : (1 << (shift - 1));
    u8* wq;

    if ((log2_w > MAX_TR_LOG2) || (log2_h > MAX_TR_LOG2))
    {
        assert(0);
    }

    if (log2_w == 2 && log2_h == 2)
    {
        wq = wq_matrix[0];
        idx_shift = 0;
        idx_step = 1;
        wq_width = 4;
    }
    else
    {
        wq = wq_matrix[1];
        idx_shift = max(log2_w, log2_h) - 3;
        idx_step = 1 << idx_shift;
        wq_width = 8;
    }

    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            int weight = ((i | j) & 0xE0) ? 0 : wq[j >> idx_shift];
            int lev = (((((coef[j] * weight) >> 2) * (s64)scale) >> 4) + offset) >> shift;
            lev = COM_CLIP( lev, -32768, 32767 );
            if( refix )
            {
                lev = (lev * 181 + 128) >> 8;
            }
            coef_out[j] = (s16)lev;
        }
        coef_out += w;
        coef += w;

        if ((i + 1) % idx_step == 0)
        {
            wq += wq_width;
        }
    }
}

void com_itdq(COM_MODE *mode, int plane, int blk_idx, s16 *coef, s16 *resi, u8* wq[2], int log2_w, int log2_h, int qp, int bit_depth, int secT_Ver_Hor, int use_alt4x4Trans
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
)
{
#if CHROMA_NOT_SPLIT
    assert(log2_w >= 2 && log2_h >= 2);
#endif
    static s16 coef_dq[MAX_CU_DIM];
    com_dquant(qp, coef, coef_dq, wq, log2_w, log2_h, bit_depth);
    com_itrans(mode, plane, blk_idx, coef_dq, resi, log2_w, log2_h, bit_depth, secT_Ver_Hor, use_alt4x4Trans
#if IST_CHROMA
        , channel
#endif    
    );
}

void com_itdq_plane(COM_MODE *mode, int plane, s16 coef[MAX_CU_DIM], s16 resi[MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp, int bit_depth, int secT_Ver_Hor[MAX_NUM_TB], int use_alt4x4Trans
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
)
{
    int log2_tb_w, log2_tb_h, tb_size;
    int part_num = get_part_num(plane == Y_C ? mode->tb_part : SIZE_2Nx2N);
#if DT_TRANSFORM
    tb_size = 0;
#else
    get_tb_width_height_log2(cu_width_log2, cu_height_log2, plane == Y_C ? mode->tb_part : SIZE_2Nx2N, &log2_tb_w, &log2_tb_h);
    tb_size = 1 << (log2_tb_w + log2_tb_h);
#endif

    for (int i = 0; i < part_num; i++)
    {
#if DT_TRANSFORM
        get_tb_width_height_log2(cu_width_log2, cu_height_log2, plane == Y_C ? mode->tb_part : SIZE_2Nx2N, &log2_tb_w, &log2_tb_h, i);
#endif	
#if ST_CHROMA
        int secTrans = secT_Ver_Hor[i];
#else
        int secTrans = (plane == Y_C) ? secT_Ver_Hor[i] : 0;
#endif
        if (mode->num_nz[i][plane])
        {
            com_itdq(mode, plane, i, 
#if DT_TRANSFORM
                coef + tb_size, resi + tb_size
#else
                coef + i * tb_size, resi + i* tb_size
#endif
                , wq, log2_tb_w, log2_tb_h, qp, bit_depth, secTrans, use_alt4x4Trans
#if IST_CHROMA
                , channel
#endif            
            );
        }
#if DT_TRANSFORM
        tb_size += 1 << (log2_tb_w + log2_tb_h);
#endif
    }
}

#if ST_CHROMA
void com_itdq_yuv(COM_MODE *mode, s16 coef[N_C][MAX_CU_DIM], s16 resi[N_C][MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp_y, int qp_u, int qp_v, int bit_depth, int secT_Ver_Hor[MAX_NUM_CHANNEL][MAX_NUM_TB], int use_alt4x4Trans[MAX_NUM_CHANNEL]
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
)
#else
void com_itdq_yuv(COM_MODE *mode, s16 coef[N_C][MAX_CU_DIM], s16 resi[N_C][MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp_y, int qp_u, int qp_v, int bit_depth, int secT_Ver_Hor[MAX_NUM_TB], int use_alt4x4Trans)
#endif
{
    for (int i = 0; i < N_C; i++)
    {
        int plane_width_log2 = cu_width_log2 - (i != Y_C);
        int plane_height_log2 = cu_height_log2 - (i != Y_C);
        int qp = (i == Y_C ? qp_y : (i == U_C ? qp_u : qp_v));
#if ST_CHROMA
        int alt4x4 = i == Y_C ? use_alt4x4Trans[CHANNEL_LUMA] : use_alt4x4Trans[CHANNEL_CHROMA];
#else
        int alt4x4 = i == Y_C ? use_alt4x4Trans : 0;
#endif
#if SBT
        get_sbt_tb_size( mode->sbt_info, i, plane_width_log2, plane_height_log2, &plane_width_log2, &plane_height_log2 );
#endif
        get_sbt_tr(mode->sbt_info, plane_width_log2, plane_height_log2, &mode->sbt_hor_trans, &mode->sbt_ver_trans);
#if IPCM
        if (!(mode->cu_mode == MODE_INTRA && ((i == Y_C && mode->ipm[PB0][0] == IPD_IPCM) || (i > Y_C && mode->ipm[PB0][0] == IPD_IPCM && mode->ipm[PB0][1] == IPD_DM_C))))
        {
#endif
#if ST_CHROMA
            com_itdq_plane(mode, i, coef[i], resi[i], wq, plane_width_log2, plane_height_log2, (u8)qp, bit_depth, i == Y_C ? secT_Ver_Hor[CHANNEL_LUMA] : secT_Ver_Hor[CHANNEL_CHROMA], alt4x4
#if IST_CHROMA
                , channel
#endif            
            );
#else
            com_itdq_plane(mode, i, coef[i], resi[i], wq, plane_width_log2, plane_height_log2, (u8)qp, bit_depth, secT_Ver_Hor, alt4x4);
#endif
#if IPCM
        }
#endif
    }
}
