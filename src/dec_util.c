﻿/* ====================================================================================================================

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

#include "dec_def.h"

void dec_picbuf_expand(DEC_CTX * ctx, COM_PIC * pic)
{
    com_picbuf_expand(pic, pic->padsize_luma, pic->padsize_chroma);
}

int dec_picbuf_check_signature(COM_PIC * pic, u8 signature[16])
{
    u8 pic_sign[16];
    int ret;
    /* execute MD5 digest here */
    ret = com_picbuf_signature(pic, pic_sign);
    com_assert_rv(COM_SUCCEEDED(ret), ret);
    com_assert_rv(com_mcmp(signature, pic_sign, 16)==0, COM_ERR_BAD_CRC);
    return COM_OK;
}


void dec_set_dec_info(DEC_CTX * ctx, DEC_CORE * core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s8  (*map_refi)[REFP_NUM];
    s16 (*map_mv)[REFP_NUM][MV_D];
    u32  *map_scu;
    s8   *map_ipm;
    u32  *map_cu_mode;
#if TB_SPLIT_EXT
    u32  *map_pb_tb_part;
#endif
    s8   *map_depth;
    int   w_cu;
    int   h_cu;
    int   scup;
    int   pic_width_in_scu;
    int   i, j;
    int   flag;
    int   cu_cbf_flag;

    scup = mod_info_curr->scup;
    w_cu = (1 << mod_info_curr->cu_width_log2) >> MIN_CU_LOG2;
    h_cu = (1 << mod_info_curr->cu_height_log2) >> MIN_CU_LOG2;
    pic_width_in_scu = ctx->info.pic_width_in_scu;
    map_refi = ctx->map.map_refi + scup;
    map_scu = ctx->map.map_scu + scup;
    map_mv = ctx->map.map_mv + scup;
    map_ipm = ctx->map.map_ipm + scup;
    map_cu_mode = ctx->map.map_cu_mode + scup;
#if TB_SPLIT_EXT
    map_pb_tb_part = ctx->map.map_pb_tb_part + scup;
#endif
    map_depth = ctx->map.map_depth + scup;
#if USE_SP
    u8 * map_usp;
    map_usp = ctx->map.map_usp + scup;
#endif

    flag = (mod_info_curr->cu_mode == MODE_INTRA) ? 1 : 0;

#if CHROMA_NOT_SPLIT // decoder cu_cbf derivation for deblocking
    if (ctx->tree_status == TREE_C)
    {
        return;
    }
    if (ctx->tree_status == TREE_L)
    {
        assert( core->mod_info_curr.num_nz[TBUV0][U_C] == 0 && core->mod_info_curr.num_nz[TBUV0][V_C] == 0 );
    }
#endif

    cu_cbf_flag = is_cu_nz(core->mod_info_curr.num_nz);

    for (i = 0; i < h_cu; i++)
    {
        for (j = 0; j < w_cu; j++)
        {
            int pb_idx_y = get_part_idx(mod_info_curr->pb_part, j << 2, i << 2, w_cu << 2, h_cu << 2);
            if (mod_info_curr->cu_mode == MODE_SKIP)
            {
                MCU_SET_SF(map_scu[j]);
            }
            else
            {
                MCU_CLR_SF(map_scu[j]);
            }
#if USE_IBC
#if USE_SP
            if (ctx->info.pic_header.ibc_flag || ctx->info.pic_header.sp_pic_flag || ctx->info.pic_header.evs_ubvs_pic_flag)
#else
            if (ctx->info.pic_header.ibc_flag)
#endif
            {
                if (mod_info_curr->cu_mode == MODE_IBC)
                {
                    MCU_SET_IBC(map_scu[j]);
#if USE_SP
                    if (mod_info_curr->ibc_flag)
                    {
                        MSP_CLR_SP_INFO(map_usp[j]);
                        MSP_CLR_CS2_INFO(map_usp[j]);
                    }
                    else if (mod_info_curr->sp_flag && !mod_info_curr->cs2_flag)
                    {
                        MSP_SET_SP_INFO(map_usp[j]);
                        MSP_CLR_CS2_INFO(map_usp[j]);
                    }
                    else if (mod_info_curr->sp_flag && mod_info_curr->cs2_flag)
                    {
                        MSP_SET_CS2_INFO(map_usp[j]);
                        MSP_CLR_SP_INFO(map_usp[j]);
                    }
#endif
                }
                else
                {
                    MCU_CLR_IBC(map_scu[j]);
#if USE_SP
                    MSP_CLR_SP_INFO(map_usp[j]);
                    MSP_CLR_CS2_INFO(map_usp[j]); 
#endif
                }
            }
#if IBC_ENH
            else
            {
                MCU_CLR_IBC(map_scu[j]);
#if USE_SP
                MSP_CLR_SP_INFO(map_usp[j]);
                MSP_CLR_CS2_INFO(map_usp[j]);
#endif
            }
#endif
#endif

            if (cu_cbf_flag)
            {
                MCU_SET_CBF(map_scu[j]);
            }
            else
            {
                MCU_CLR_CBF(map_scu[j]);
            }
#if BGC
            if (mod_info_curr->bgc_flag)
            {
                assert(mod_info_curr->cu_mode == MODE_INTER || mod_info_curr->cu_mode == MODE_SKIP || mod_info_curr->cu_mode == MODE_DIR);
                assert(REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && REFI_IS_VALID(mod_info_curr->refi[REFP_1]));
                MCU_SET_BGC_FLAG(map_scu[j]);
                if (mod_info_curr->bgc_idx)
                {
                    MCU_SET_BGC_IDX(map_scu[j]);
                }
                else
                {
                    MCU_CLR_BGC_IDX(map_scu[j]);
                }
            }
            else
            {
                MCU_CLR_BGC_FLAG(map_scu[j]);
                MCU_CLR_BGC_IDX(map_scu[j]);
            }
#endif
            if (mod_info_curr->affine_flag)
            {
                MCU_SET_AFF(map_scu[j], mod_info_curr->affine_flag);
            }
            else
            {
                MCU_CLR_AFF(map_scu[j]);
            }

            MCU_SET_X_SCU_OFF(map_cu_mode[j], j);
            MCU_SET_Y_SCU_OFF(map_cu_mode[j], i);
            MCU_SET_LOGW(map_cu_mode[j], mod_info_curr->cu_width_log2);
            MCU_SET_LOGH(map_cu_mode[j], mod_info_curr->cu_height_log2);
            MCU_SET_IF_COD_SN_QP(map_scu[j], flag, ctx->slice_num, core->qp_y);
#if TB_SPLIT_EXT
            MCU_SET_TB_PART_LUMA(map_pb_tb_part[j], mod_info_curr->tb_part);
#endif
#if SBT
            MCU_SET_SBT_INFO(map_pb_tb_part[j], mod_info_curr->sbt_info);
#endif
#if SUB_TMVP
            if (core->sbTmvp_flag)
            {
                int blk = ((j >= w_cu / SBTMVP_NUM_1D) ? 1 : 0) + ((i >= h_cu / SBTMVP_NUM_1D) ? SBTMVP_NUM_1D : 0);
                map_refi[j][REFP_0] = core->sbTmvp[blk].ref_idx[REFP_0];
                map_refi[j][REFP_1] = core->sbTmvp[blk].ref_idx[REFP_1];
                map_mv[j][REFP_0][MV_X] = core->sbTmvp[blk].mv[REFP_0][MV_X];
                map_mv[j][REFP_0][MV_Y] = core->sbTmvp[blk].mv[REFP_0][MV_Y];
                map_mv[j][REFP_1][MV_X] = core->sbTmvp[blk].mv[REFP_1][MV_X];
                map_mv[j][REFP_1][MV_Y] = core->sbTmvp[blk].mv[REFP_1][MV_Y];
            }
            else
            {
#endif
#if MVAP
                if (core->mvap_flag)
                {
                    map_refi[j][REFP_0] = core->best_cu_mvfield[j + i * w_cu].ref_idx[REFP_0];
                    map_refi[j][REFP_1] = core->best_cu_mvfield[j + i * w_cu].ref_idx[REFP_1];
                    map_mv[j][REFP_0][MV_X] = core->best_cu_mvfield[j + i * w_cu].mv[REFP_0][MV_X];
                    map_mv[j][REFP_0][MV_Y] = core->best_cu_mvfield[j + i * w_cu].mv[REFP_0][MV_Y];
                    map_mv[j][REFP_1][MV_X] = core->best_cu_mvfield[j + i * w_cu].mv[REFP_1][MV_X];
                    map_mv[j][REFP_1][MV_Y] = core->best_cu_mvfield[j + i * w_cu].mv[REFP_1][MV_Y];
                }
                else
                {
#endif
                    map_refi[j][REFP_0] = mod_info_curr->refi[REFP_0];
                    map_refi[j][REFP_1] = mod_info_curr->refi[REFP_1];
                    map_mv[j][REFP_0][MV_X] = mod_info_curr->mv[REFP_0][MV_X];
                    map_mv[j][REFP_0][MV_Y] = mod_info_curr->mv[REFP_0][MV_Y];
                    map_mv[j][REFP_1][MV_X] = mod_info_curr->mv[REFP_1][MV_X];
                    map_mv[j][REFP_1][MV_Y] = mod_info_curr->mv[REFP_1][MV_Y];
#if MVAP
                }
#endif
#if SUB_TMVP
            }
#endif
#if ETMVP
            if (mod_info_curr->etmvp_flag)
            {
                map_refi[j][REFP_0] = core->best_etmvp_mvfield[j + i * w_cu].ref_idx[REFP_0];
                map_refi[j][REFP_1] = core->best_etmvp_mvfield[j + i * w_cu].ref_idx[REFP_1];
                map_mv[j][REFP_0][MV_X] = core->best_etmvp_mvfield[j + i * w_cu].mv[REFP_0][MV_X];
                map_mv[j][REFP_0][MV_Y] = core->best_etmvp_mvfield[j + i * w_cu].mv[REFP_0][MV_Y];
                map_mv[j][REFP_1][MV_X] = core->best_etmvp_mvfield[j + i * w_cu].mv[REFP_1][MV_X];
                map_mv[j][REFP_1][MV_Y] = core->best_etmvp_mvfield[j + i * w_cu].mv[REFP_1][MV_Y];
            }
#endif
            map_ipm[j] = mod_info_curr->ipm[pb_idx_y][0];
#if USE_IBC
            if (ctx->info.pic_header.ibc_flag)
            {
                if (mod_info_curr->cu_mode == MODE_IBC)
                {
                    map_ipm[j] = 0;
                }
            }
#endif
#if USE_SP
            if (mod_info_curr->sp_flag == TRUE)
            {
                map_ipm[j] = IPD_DC;
            }
#endif
            map_depth[j] = (s8)mod_info_curr->cud;
        }
        map_refi += pic_width_in_scu;
        map_mv += pic_width_in_scu;
        map_scu += pic_width_in_scu;
        map_ipm += pic_width_in_scu;
        map_depth += pic_width_in_scu;
        map_cu_mode += pic_width_in_scu;
#if TB_SPLIT_EXT
        map_pb_tb_part += pic_width_in_scu;
#endif
#if USE_SP
        map_usp += pic_width_in_scu;
#endif
    }

    if (mod_info_curr->affine_flag)
    {
        com_set_affine_mvf(&ctx->info, mod_info_curr, ctx->refp, &ctx->map);
    }

#if AWP
    if (mod_info_curr->awp_flag)
    {
        com_set_awp_mvf(&ctx->info, mod_info_curr, ctx->refp, &ctx->map);
    }
#endif

#if SAWP && !DSAWP
    if (mod_info_curr->sawp_flag)
    {
        update_sawp_info_map_scu(mod_info_curr, ctx->map.map_scu, ctx->map.map_ipm, pic_width_in_scu);
    }
#endif // SAWP
#if USE_IBC && USE_SP
    if (mod_info_curr->sp_flag && (!mod_info_curr->cs2_flag))
    {
        int w = (1 << mod_info_curr->cu_width_log2);
        int h = (1 << mod_info_curr->cu_height_log2);
        int w_log2 = mod_info_curr->cu_width_log2;
        int h_log2 = mod_info_curr->cu_height_log2;
        int* p_raster_scan_order = com_tbl_trav2raster[mod_info_curr->sp_copy_direction][w_log2 - MIN_CU_LOG2][h_log2 - MIN_CU_LOG2];
        map_mv = ctx->map.map_mv + scup;

        if (mod_info_curr->sub_string_no > 0)
        {
            int tmp_len = 0;
            int tmp_strlen_before = 0;
            for (int i = 0; i < mod_info_curr->sub_string_no; i++)
            {
                s16 tmp_sv[2] = { 0 };
                if (mod_info_curr->string_copy_info[i].is_matched)
                {
                    tmp_sv[0] = mod_info_curr->string_copy_info[i].offset_x << 2;
                    tmp_sv[1] = mod_info_curr->string_copy_info[i].offset_y << 2;
                    tmp_len = mod_info_curr->string_copy_info[i].length;
                }
                else
                {
                    tmp_len = 4;
                }

                int idx_substr1 = tmp_strlen_before + 0;//first pixel of the substring
                int idx_substr2 = tmp_strlen_before + tmp_len - 1;//last pixel of the substring

                int idx_substr1_raster = p_raster_scan_order[idx_substr1];
                int idx_substr1_raster_x = idx_substr1_raster % w;//x1
                int idx_substr1_raster_y = idx_substr1_raster / w;//y1

                int idx_substr2_raster = p_raster_scan_order[idx_substr2];
                int idx_substr2_raster_x = idx_substr2_raster % w;//x2
                int idx_substr2_raster_y = idx_substr2_raster / w;//y2

                if (ctx->info.pic_header.slice_type != SLICE_I)
                {
                    //y1==y2
                    if (idx_substr1_raster_y == idx_substr2_raster_y)
                    {
                        tmp_strlen_before += tmp_len;
                        continue;
                    }
                    //x1==x2
                    if (idx_substr1_raster_x == idx_substr2_raster_x)
                    {
                        tmp_strlen_before += tmp_len;
                        continue;
                    }
                    //L==(y2-y1+1)*w
                    if (tmp_len == w * (idx_substr2_raster_y - idx_substr1_raster_y + 1))
                    {
                        tmp_strlen_before += tmp_len;
                        continue;
                    }
                }

                for (int j = 0; j < tmp_len; j++)
                {
                    int idx_cur_pixel = (tmp_strlen_before + j);
                    int idx_cur_pixel_raster = p_raster_scan_order[idx_cur_pixel];

                    int cur_pixel_x_cu = (idx_cur_pixel_raster % w);
                    int cur_pixel_y_cu = (idx_cur_pixel_raster / w);
                    int cur_pixel_x_scu = (cur_pixel_x_cu % 4);
                    int cur_pixel_y_scu = (cur_pixel_y_cu % 4);

                    int tmp_scu_idx = (cur_pixel_y_cu >> 2)*(pic_width_in_scu)+(cur_pixel_x_cu >> 2);

                    if (cur_pixel_x_scu == 0 && cur_pixel_y_scu == 3)
                    {
                        map_mv[tmp_scu_idx][REFP_0][MV_X] = tmp_sv[0];
                        map_mv[tmp_scu_idx][REFP_0][MV_Y] = tmp_sv[1];
                        map_mv[tmp_scu_idx][REFP_1][MV_X] = 0;
                        map_mv[tmp_scu_idx][REFP_1][MV_Y] = 0;
                    }
                }
                tmp_strlen_before += tmp_len;
            }
        }
    }
#endif

#if MVF_TRACE
    // Trace MVF in decoder
    {
        map_refi = ctx->map.map_refi + scup;
        map_scu = ctx->map.map_scu + scup;
        map_mv = ctx->map.map_mv + scup;
        map_cu_mode = ctx->map.map_cu_mode + scup;
        for (i = 0; i < h_cu; i++)
        {
            for (j = 0; j < w_cu; j++)
            {
                COM_TRACE_COUNTER;
                COM_TRACE_STR(" x: ");
                COM_TRACE_INT(j);
                COM_TRACE_STR(" y: ");
                COM_TRACE_INT(i);
                COM_TRACE_STR(" ref0: ");
                COM_TRACE_INT(map_refi[j][REFP_0]);
                COM_TRACE_STR(" mv: ");
                COM_TRACE_MV(map_mv[j][REFP_0][MV_X], map_mv[j][REFP_0][MV_Y]);
                COM_TRACE_STR(" ref1: ");
                COM_TRACE_INT(map_refi[j][REFP_1]);
                COM_TRACE_STR(" mv: ");
                COM_TRACE_MV(map_mv[j][REFP_1][MV_X], map_mv[j][REFP_1][MV_Y]);

                COM_TRACE_STR(" affine: ");
                COM_TRACE_INT(MCU_GET_AFF(map_scu[j]));
                if (MCU_GET_AFF(map_scu[j]))
                {
                    COM_TRACE_STR(" logw: ");
                    COM_TRACE_INT(MCU_GET_LOGW(map_cu_mode[j]));
                    COM_TRACE_STR(" logh: ");
                    COM_TRACE_INT(MCU_GET_LOGH(map_cu_mode[j]));
                    COM_TRACE_STR(" xoff: ");
                    COM_TRACE_INT(MCU_GET_X_SCU_OFF(map_cu_mode[j]));
                    COM_TRACE_STR(" yoff: ");
                    COM_TRACE_INT(MCU_GET_Y_SCU_OFF(map_cu_mode[j]));
                }

                COM_TRACE_STR("\n");
            }
            map_refi += pic_width_in_scu;
            map_mv += pic_width_in_scu;
            map_scu += pic_width_in_scu;
            map_cu_mode += pic_width_in_scu;
        }
    }
#endif
}

void dec_derive_skip_direct_info(DEC_CTX * ctx, DEC_CORE * core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int cu_width = (1 << mod_info_curr->cu_width_log2);
    int cu_height = (1 << mod_info_curr->cu_height_log2);

    if (mod_info_curr->affine_flag) // affine merge motion vector
    {
        s8  mrg_list_refi[AFF_MAX_NUM_MRG][REFP_NUM];
        int mrg_list_cp_num[AFF_MAX_NUM_MRG];
        CPMV mrg_list_cp_mv[AFF_MAX_NUM_MRG][REFP_NUM][VER_NUM][MV_D];
#if BGC
        s8 mrg_list_bgc_flag[AFF_MAX_NUM_MRG];
        s8 mrg_list_bgc_idx[AFF_MAX_NUM_MRG];
#endif
        int cp_idx, lidx;
        int mrg_idx = mod_info_curr->skip_idx;

#if HACD
#if BGC
        com_get_affine_merge_candidate(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, mrg_list_refi, mrg_list_cp_mv, mrg_list_cp_num, mrg_list_bgc_flag, mrg_list_bgc_idx, ctx->ptr, core->history_affine_mv);
#else
        com_get_affine_merge_candidate(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, mrg_list_refi, mrg_list_cp_mv, mrg_list_cp_num, ctx->ptr, core->history_affine_mv);
#endif
#else
#if BGC
        com_get_affine_merge_candidate(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, mrg_list_refi, mrg_list_cp_mv, mrg_list_cp_num, mrg_list_bgc_flag, mrg_list_bgc_idx, ctx->ptr);
#else
        com_get_affine_merge_candidate(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, mrg_list_refi, mrg_list_cp_mv, mrg_list_cp_num, ctx->ptr);
#endif
#endif
        mod_info_curr->affine_flag = (u8)mrg_list_cp_num[mrg_idx] - 1;

        COM_TRACE_COUNTER;
        COM_TRACE_STR("merge affine flag after constructed candidate ");
        COM_TRACE_INT(mod_info_curr->affine_flag);
        COM_TRACE_STR("\n");

        for (lidx = 0; lidx < REFP_NUM; lidx++)
        {
            mod_info_curr->mv[lidx][MV_X] = 0;
            mod_info_curr->mv[lidx][MV_Y] = 0;
            if (REFI_IS_VALID(mrg_list_refi[mrg_idx][lidx]))
            {
                mod_info_curr->refi[lidx] = mrg_list_refi[mrg_idx][lidx];
                for (cp_idx = 0; cp_idx < mrg_list_cp_num[mrg_idx]; cp_idx++)
                {
#if AFFINE_UMVE
                    if (mod_info_curr->affine_umve_flag && cp_idx < 2)
                    {
                        s32 affine_umve_mvd[REFP_NUM][MV_D];
                        derive_affine_umve_final_motion(mod_info_curr->refi, mod_info_curr->affine_umve_idx[cp_idx], affine_umve_mvd);
                        mod_info_curr->affine_mv[lidx][cp_idx][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_X] + affine_umve_mvd[lidx][MV_X]);
                        mod_info_curr->affine_mv[lidx][cp_idx][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_Y] + affine_umve_mvd[lidx][MV_Y]);
                    }
                    else
                    {
                        mod_info_curr->affine_mv[lidx][cp_idx][MV_X] = mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_X];
                        mod_info_curr->affine_mv[lidx][cp_idx][MV_Y] = mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_Y];
                    }
#else
                    mod_info_curr->affine_mv[lidx][cp_idx][MV_X] = mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_X];
                    mod_info_curr->affine_mv[lidx][cp_idx][MV_Y] = mrg_list_cp_mv[mrg_idx][lidx][cp_idx][MV_Y];
#endif
                }
            }
            else
            {
                mod_info_curr->refi[lidx] = REFI_INVALID;
                for (cp_idx = 0; cp_idx < mrg_list_cp_num[mrg_idx]; cp_idx++)
                {
                    mod_info_curr->affine_mv[lidx][cp_idx][MV_X] = 0;
                    mod_info_curr->affine_mv[lidx][cp_idx][MV_Y] = 0;
                }
            }
        }
#if BGC
        if (REFI_IS_VALID(mrg_list_refi[mrg_idx][REFP_0]) && REFI_IS_VALID(mrg_list_refi[mrg_idx][REFP_1]))
        {
            mod_info_curr->bgc_flag = mrg_list_bgc_flag[mrg_idx];
            mod_info_curr->bgc_idx = mrg_list_bgc_idx[mrg_idx];
        }
        else
        {
            mod_info_curr->bgc_flag = 0;
            mod_info_curr->bgc_idx = 0;
        }
#endif
    }
#if AWP
    else if (mod_info_curr->awp_flag) // awp candidates temporal + spacial
    {
        s16 pmv_temp[REFP_NUM][MV_D];
        s8 refi_temp[REFP_NUM];
        refi_temp[REFP_0] = 0;
        refi_temp[REFP_1] = 0;

        int scup_co = get_rb_scup(mod_info_curr->scup, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, cu_width, cu_height, ctx->info.max_cuwh);
        if (ctx->info.pic_header.slice_type == SLICE_P)
        {
            refi_temp[REFP_0] = 0;
            refi_temp[REFP_1] = -1;
            if (REFI_IS_VALID(ctx->refp[0][REFP_0].map_refi[scup_co][REFP_0]))
            {
                get_col_mv_from_list0(ctx->refp[0], ctx->ptr, scup_co, pmv_temp);
            }
            else
            {
                pmv_temp[REFP_0][MV_X] = 0;
                pmv_temp[REFP_0][MV_Y] = 0;
            }
            pmv_temp[REFP_1][MV_X] = 0;
            pmv_temp[REFP_1][MV_Y] = 0;
        }
        else
        {
            if (!REFI_IS_VALID(ctx->refp[0][REFP_1].map_refi[scup_co][REFP_0]))
            {
                if (REFI_IS_VALID(ctx->refp[0][REFP_1].map_refi[scup_co][REFP_1]))
                {
                    get_col_mv_ext(ctx->refp[0], ctx->ptr, scup_co, pmv_temp, refi_temp);
                }
                else
                {
                    com_get_mvp_default(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_0, 0, 0, pmv_temp[REFP_0]);
                    com_get_mvp_default(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_1, 0, 0, pmv_temp[REFP_1]);
                }
            }
            else
            {
                get_col_mv_ext(ctx->refp[0], ctx->ptr, scup_co, pmv_temp, refi_temp);
            }
        }

        s16 awp_uni_cands[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D];
        s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM];
        u8 valid_cand_num = com_derive_awp_base_motions(&ctx->info, mod_info_curr, &ctx->map, pmv_temp, refi_temp, awp_uni_cands, awp_uni_refi, ctx->dpm.num_refp, ctx->refp);
#if AWP_MVR
#if BAWP
        if (ctx->info.sqh.awp_mvr_enable_flag && ctx->info.pic_header.slice_type == SLICE_B)
#else
        if (ctx->info.sqh.awp_mvr_enable_flag)
#endif
        {
            s32 awp_mvr_offset[REFP_NUM][MV_D];
            s16 awp_final_mv0[REFP_NUM][MV_D], awp_final_mv1[REFP_NUM][MV_D];
            s8  awp_final_refi0[REFP_NUM], awp_final_refi1[REFP_NUM];

            if (!mod_info_curr->awp_mvr_flag0)
            {
                awp_final_mv0[REFP_0][MV_X] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_X];
                awp_final_mv0[REFP_0][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_Y];
                awp_final_mv0[REFP_1][MV_X] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_X];
                awp_final_mv0[REFP_1][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_Y];
                awp_final_refi0[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_0];
                awp_final_refi0[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_1];
            }
            else
            {
                derive_awp_mvr_final_motion(mod_info_curr->awp_mvr_idx0, ctx->refp, awp_uni_refi[mod_info_curr->awp_idx0], awp_mvr_offset);

                awp_final_mv0[REFP_0][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_X] + awp_mvr_offset[REFP_0][MV_X]);
                awp_final_mv0[REFP_0][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_Y] + awp_mvr_offset[REFP_0][MV_Y]);
                awp_final_mv0[REFP_1][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_X] + awp_mvr_offset[REFP_1][MV_X]);
                awp_final_mv0[REFP_1][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_Y] + awp_mvr_offset[REFP_1][MV_Y]);
                awp_final_refi0[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_0];
                awp_final_refi0[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_1];
            }

            if (!mod_info_curr->awp_mvr_flag1)
            {
                awp_final_mv1[REFP_0][MV_X] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_X];
                awp_final_mv1[REFP_0][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_Y];
                awp_final_mv1[REFP_1][MV_X] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_X];
                awp_final_mv1[REFP_1][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_Y];
                awp_final_refi1[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_0];
                awp_final_refi1[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_1];
            }
            else
            {
                derive_awp_mvr_final_motion(mod_info_curr->awp_mvr_idx1, ctx->refp, awp_uni_refi[mod_info_curr->awp_idx1], awp_mvr_offset);

                awp_final_mv1[REFP_0][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_X] + awp_mvr_offset[REFP_0][MV_X]);
                awp_final_mv1[REFP_0][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_Y] + awp_mvr_offset[REFP_0][MV_Y]);
                awp_final_mv1[REFP_1][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_X] + awp_mvr_offset[REFP_1][MV_X]);
                awp_final_mv1[REFP_1][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_Y] + awp_mvr_offset[REFP_1][MV_Y]);
                awp_final_refi1[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_0];
                awp_final_refi1[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_1];
            }
            com_set_awp_mvr_mv_para(mod_info_curr, awp_final_mv0, awp_final_refi0, awp_final_mv1, awp_final_refi1);
        }
        else
#endif
        com_set_awp_mv_para(mod_info_curr, awp_uni_cands, awp_uni_refi);
    }
#endif
    else
    {
#if ETMVP
        if (mod_info_curr->etmvp_flag)
        {
            COM_MOTION first_stage_motion;
            s32 ref_block_x = 0;
            s32 ref_block_y = 0;
            s32 offset_x[MAX_ETMVP_NUM] = { 0, 8, -8, 0, 0 };
            s32 offset_y[MAX_ETMVP_NUM] = { 0, 0, 0, 8, -8 };
            s32 is_valid_etmvp = 0;
            s32 tmp_index = 0;
            s32 valid_etmvp_offset[MAX_ETMVP_NUM] = { 0 };
            COM_MOTION base_motion;
            s32 tmp_ref_block_x = 0;
            s32 tmp_ref_block_y = 0;

#if USE_SP
            derive_first_stage_motion(ctx->info.pic_header.ibc_flag || ctx->info.pic_header.sp_pic_flag|| ctx->info.pic_header.evs_ubvs_pic_flag, mod_info_curr->scup, cu_width, cu_height, ctx->info.pic_width_in_scu, ctx->map.map_scu, ctx->map.map_mv, ctx->map.map_refi, &first_stage_motion);
#else
            derive_first_stage_motion(mod_info_curr->ibc_flag, mod_info_curr->scup, cu_width, cu_height, ctx->info.pic_width_in_scu, ctx->map.map_scu, ctx->map.map_mv, ctx->map.map_refi, &first_stage_motion);
#endif

            base_motion.mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
            base_motion.mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
            base_motion.ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];
            base_motion.mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
            base_motion.mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
            base_motion.ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            derive_scaled_base_motion(ctx->info.pic_header.slice_type, ctx->ptr, ctx->refp, &base_motion, &first_stage_motion);
            derive_ref_block_position(ctx->info.pic_header.slice_type, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->info.pic_width, ctx->info.pic_height, cu_width, cu_height, core->x_pel, core->y_pel, mod_info_curr->x_pos, mod_info_curr->y_pos, ctx->ptr, ctx->refp, &ref_block_x, &ref_block_y, ctx->info.max_cuwh);
            get_valid_etmvp_motion(ctx->info.pic_header.slice_type, ctx->ptr, cu_width, cu_height, core->x_pel, core->y_pel, ctx->info.pic_width, ctx->info.pic_height, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ref_block_x, ref_block_y, ctx->refp, valid_etmvp_offset, first_stage_motion, ctx->info.max_cuwh);

            ref_block_x = ref_block_x + offset_x[valid_etmvp_offset[mod_info_curr->skip_idx]];
            ref_block_y = ref_block_y + offset_y[valid_etmvp_offset[mod_info_curr->skip_idx]];

            set_etmvp_mvfield(ctx->info.pic_header.slice_type, ctx->ptr, ref_block_x, ref_block_y, cu_width, cu_height, ctx->info.pic_width, ctx->info.pic_height, core->x_pel, core->y_pel, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->refp, core->best_etmvp_mvfield, first_stage_motion, ctx->info.max_cuwh);
        }
        else
        {
#endif

            s16 pmv_temp[REFP_NUM][MV_D];
            s8 refi_temp[REFP_NUM];
            refi_temp[REFP_0] = 0;
            refi_temp[REFP_1] = 0;
            int scup_co = get_colocal_scup(mod_info_curr->scup, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu);
#if SUB_TMVP 
            int sb_scup[SBTMVP_NUM];
            int sb_scup_co[SBTMVP_NUM];
            if (ctx->info.sqh.sbtmvp_enable_flag)
            {
                for (int i = 0; i < SBTMVP_NUM; i++)
                {
                    sb_scup[i] = mod_info_curr->scup + ctx->info.pic_width_in_scu* ((cu_height >> 2) - 1)*(i / 2) + ((cu_width >> 2) - 1)*(i % 2);
                    sb_scup_co[i] = get_colocal_scup(sb_scup[i], ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu);
                }
            }
#endif
            if (ctx->info.pic_header.slice_type == SLICE_P)
            {
                refi_temp[REFP_0] = 0;
                refi_temp[REFP_1] = -1;
                if (REFI_IS_VALID(ctx->refp[0][REFP_0].map_refi[scup_co][REFP_0]))
                {
                    get_col_mv_from_list0(ctx->refp[0], ctx->ptr, scup_co, pmv_temp);
                }
                else
                {
                    pmv_temp[REFP_0][MV_X] = 0;
                    pmv_temp[REFP_0][MV_Y] = 0;
                }
                pmv_temp[REFP_1][MV_X] = 0;
                pmv_temp[REFP_1][MV_Y] = 0;
#if SUB_TMVP 
                if (ctx->info.sqh.sbtmvp_enable_flag)
                {
                    for (int i = 0; i < SBTMVP_NUM; i++)
                    {
                        if (REFI_IS_VALID(ctx->refp[0][REFP_0].map_refi[sb_scup_co[i]][REFP_0]) || REFI_IS_VALID(ctx->refp[0][REFP_0].map_refi[sb_scup_co[i]][REFP_1]))
                        {
                            get_col_mv_from_list0_ext(ctx->refp[0], ctx->ptr, sb_scup_co[i], core->sbTmvp[i].mv, core->sbTmvp[i].ref_idx);
                        }
                        else
                        {
                            copy_mv(core->sbTmvp[i].mv[REFP_0], pmv_temp[REFP_0]);
                        }
                        core->sbTmvp[i].mv[REFP_1][MV_X] = 0;
                        core->sbTmvp[i].mv[REFP_1][MV_Y] = 0;
                        core->sbTmvp[i].ref_idx[REFP_0] = 0;
                        core->sbTmvp[i].ref_idx[REFP_1] = -1;
                    }
                }
#endif
            }
            else
            {
                if (!REFI_IS_VALID(ctx->refp[0][REFP_1].map_refi[scup_co][REFP_0]))
                {
                    com_get_mvp_default(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_0, 0, 0, pmv_temp[REFP_0]);

                    com_get_mvp_default(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_1, 0, 0, pmv_temp[REFP_1]);
                }
                else
                {
                    get_col_mv(ctx->refp[0], ctx->ptr, scup_co, pmv_temp);
                }
#if SUB_TMVP
                if (ctx->info.sqh.sbtmvp_enable_flag)
                {
                    for (int i = 0; i < SBTMVP_NUM; i++)
                    {
                        if (!REFI_IS_VALID(ctx->refp[0][REFP_1].map_refi[sb_scup_co[i]][REFP_0]) && !REFI_IS_VALID(ctx->refp[0][REFP_1].map_refi[sb_scup_co[i]][REFP_1]))
                        {
                            copy_mv(core->sbTmvp[i].mv[REFP_0], pmv_temp[REFP_0]);
                            copy_mv(core->sbTmvp[i].mv[REFP_1], pmv_temp[REFP_1]);
                            SET_REFI(core->sbTmvp[i].ref_idx, 0, 0);
                        }
                        else
                        {
                            get_col_mv_ext(ctx->refp[0], ctx->ptr, sb_scup_co[i], core->sbTmvp[i].mv, core->sbTmvp[i].ref_idx);
                        }
                    }
                }
#endif
            }

            if (!mod_info_curr->umve_flag)
            {
#if BGC
                s8 bgc_flag_cands[MAX_SKIP_NUM];
                s8 bgc_idx_cands[MAX_SKIP_NUM];
                memset(bgc_flag_cands, 0, MAX_SKIP_NUM * sizeof(s8));
                memset(bgc_idx_cands, 0, MAX_SKIP_NUM * sizeof(s8));
#endif
                int num_cands = 0;
                u8 spatial_skip_idx = mod_info_curr->skip_idx;
                s16 skip_pmv_cands[MAX_SKIP_NUM][REFP_NUM][MV_D];
                s8 skip_refi[MAX_SKIP_NUM][REFP_NUM];
                copy_mv(skip_pmv_cands[0][REFP_0], pmv_temp[REFP_0]);
                copy_mv(skip_pmv_cands[0][REFP_1], pmv_temp[REFP_1]);
                skip_refi[0][REFP_0] = refi_temp[REFP_0];
                skip_refi[0][REFP_1] = refi_temp[REFP_1];
                num_cands++;
#if SUB_TMVP
                if (ctx->info.sqh.sbtmvp_enable_flag && mod_info_curr->skip_idx == 0 && cu_width >= SBTMVP_MIN_SIZE && cu_height >= SBTMVP_MIN_SIZE)
                {
                    core->sbTmvp_flag = 1;
#if UNIFIED_HMVP_1
                    mod_info_curr->sub_tmvp_flag = 1;
#endif
                }
#endif
#if UNIFIED_HMVP_2
                int skipCheckTmvpRedundancy = 0;
                if (ctx->info.sqh.profile_id == 0x30 || ctx->info.sqh.profile_id == 0x32)
                {
                    skipCheckTmvpRedundancy = 1;
                }
#endif
                if (mod_info_curr->skip_idx != 0)
                {
                    int skip_idx;
                    COM_MOTION motion_cands_curr[MAX_SKIP_NUM];
                    s8 cnt_hmvp_cands_curr = 0;
#if BGC
                    s8 bgc_flag_cands_curr[MAX_SKIP_NUM];
                    s8 bgc_idx_cands_curr[MAX_SKIP_NUM];
                    derive_MHBskip_spatial_motions(&ctx->info, mod_info_curr, &ctx->map, &skip_pmv_cands[num_cands], &skip_refi[num_cands], &(bgc_flag_cands[num_cands]), &(bgc_idx_cands[num_cands]));
#else
                    derive_MHBskip_spatial_motions(&ctx->info, mod_info_curr, &ctx->map, &skip_pmv_cands[num_cands], &skip_refi[num_cands]);
#endif
                    num_cands += PRED_DIR_NUM;
#if MVAP
                    if (ctx->info.sqh.mvap_enable_flag)
                    {
#if USE_SP
                        derive_mvap_motions(ctx->info.pic_header.ibc_flag || ctx->info.pic_header.sp_pic_flag || ctx->info.pic_header.evs_ubvs_pic_flag, num_cands, mod_info_curr->scup, cu_width, cu_height, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu, ctx->map.map_mv, ctx->map.map_refi, core->neighbor_motions, &core->valid_mvap_num, core->valid_mvap_index, skip_refi);
#else
                        derive_mvap_motions(ctx->info.pic_header.ibc_flag, num_cands, mod_info_curr->scup, cu_width, cu_height, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu, ctx->map.map_mv, ctx->map.map_refi, core->neighbor_motions, &core->valid_mvap_num, core->valid_mvap_index, skip_refi);
#endif
                        num_cands += core->valid_mvap_num;
                    }
#endif
                    if ((ctx->info.sqh.num_of_hmvp_cand || ctx->info.sqh.num_of_mvap_cand) && mod_info_curr->skip_idx >= TRADITIONAL_SKIP_NUM)
                    {
                        for (skip_idx = 0; skip_idx < num_cands; skip_idx++) // fill the traditional skip candidates
                        {
                            fill_skip_candidates(motion_cands_curr, &cnt_hmvp_cands_curr, ctx->info.sqh.num_of_hmvp_cand,
#if MVAP
                                ctx->info.sqh.num_of_mvap_cand,
#endif
#if BGC
                                bgc_flag_cands_curr, bgc_idx_cands_curr, bgc_flag_cands[skip_idx], bgc_idx_cands[skip_idx],
#endif
                                skip_pmv_cands[skip_idx], skip_refi[skip_idx], 0
#if UNIFIED_HMVP_2
                                , skipCheckTmvpRedundancy
#endif
                            );
                        }
#if MVAP
                        assert(cnt_hmvp_cands_curr == TRADITIONAL_SKIP_NUM + core->valid_mvap_num);
#else
                        assert(cnt_hmvp_cands_curr == TRADITIONAL_SKIP_NUM);
#endif
                        for (skip_idx = core->cnt_hmvp_cands; skip_idx > 0; skip_idx--) // fill the HMVP skip candidates
                        {
                            COM_MOTION motion = core->motion_cands[skip_idx - 1];
#if BGC
                            s8 bgc_flag = core->bgc_flag_cands[skip_idx - 1];
                            s8 bgc_idx = core->bgc_idx_cands[skip_idx - 1];
#endif
                            fill_skip_candidates(motion_cands_curr, &cnt_hmvp_cands_curr, ctx->info.sqh.num_of_hmvp_cand,
#if MVAP
                                ctx->info.sqh.num_of_mvap_cand,
#endif
#if BGC
                                bgc_flag_cands_curr, bgc_idx_cands_curr, bgc_flag, bgc_idx,
#endif
                                motion.mv, motion.ref_idx, 1
#if UNIFIED_HMVP_2
                                , skipCheckTmvpRedundancy
#endif
                            );
                        }

                        COM_MOTION motion = core->cnt_hmvp_cands ? core->motion_cands[core->cnt_hmvp_cands - 1] : motion_cands_curr[TRADITIONAL_SKIP_NUM - 1]; // use last HMVP candidate or last spatial candidate to fill the rest
#if BGC
                        s8 bgc_flag_ext = core->cnt_hmvp_cands ? core->bgc_flag_cands[core->cnt_hmvp_cands - 1] : bgc_flag_cands_curr[TRADITIONAL_SKIP_NUM - 1];
                        s8 bgc_idx_ext = core->cnt_hmvp_cands ? core->bgc_idx_cands[core->cnt_hmvp_cands - 1] : bgc_idx_cands_curr[TRADITIONAL_SKIP_NUM - 1];
#endif
                        for (skip_idx = cnt_hmvp_cands_curr; skip_idx < (TRADITIONAL_SKIP_NUM +
#if MVAP
                            max(ctx->info.sqh.num_of_hmvp_cand, ctx->info.sqh.num_of_mvap_cand))
#else
                            ctx->info.sqh.num_of_hmvp_cand)
#endif
                            ; skip_idx++) // fill skip candidates when hmvp not enough
                        {
                            fill_skip_candidates(motion_cands_curr, &cnt_hmvp_cands_curr, ctx->info.sqh.num_of_hmvp_cand,
#if MVAP
                                ctx->info.sqh.num_of_mvap_cand,
#endif
#if BGC
                                bgc_flag_cands_curr, bgc_idx_cands_curr, bgc_flag_ext, bgc_idx_ext,
#endif
                                motion.mv, motion.ref_idx, 0
#if UNIFIED_HMVP_2
                                , skipCheckTmvpRedundancy
#endif
                            );
                        }
#if MVAP
                        assert(cnt_hmvp_cands_curr == (TRADITIONAL_SKIP_NUM + max(ctx->info.sqh.num_of_hmvp_cand, ctx->info.sqh.num_of_mvap_cand)));
#else
                        assert(cnt_hmvp_cands_curr == (TRADITIONAL_SKIP_NUM + ctx->info.sqh.num_of_hmvp_cand));
#endif

                        get_hmvp_skip_cands(motion_cands_curr, cnt_hmvp_cands_curr, 
#if BGC
                            bgc_flag_cands_curr, bgc_idx_cands_curr, bgc_flag_cands, bgc_idx_cands,
#endif
                            skip_pmv_cands, skip_refi);
                    }
#if MVAP
                    assert(cnt_hmvp_cands_curr <= (TRADITIONAL_SKIP_NUM + max(ctx->info.sqh.num_of_hmvp_cand, ctx->info.sqh.num_of_mvap_cand)));
#else
                    assert(cnt_hmvp_cands_curr <= (TRADITIONAL_SKIP_NUM + ctx->info.sqh.num_of_hmvp_cand));
#endif
                }
#if MVAP
                if (ctx->info.sqh.mvap_enable_flag)
                {
                    if ((spatial_skip_idx < (core->valid_mvap_num + TRADITIONAL_SKIP_NUM)) && (spatial_skip_idx >= TRADITIONAL_SKIP_NUM))
                    {
                        core->mvap_flag = 1;
#if UNIFIED_HMVP_1
                        mod_info_curr->mvap_flag = 1;
#endif
                        core->mvap_mode = core->valid_mvap_index[spatial_skip_idx - TRADITIONAL_SKIP_NUM];
                        set_mvap_mvfield(cu_width >> 2, cu_height >> 2, core->valid_mvap_index[spatial_skip_idx - TRADITIONAL_SKIP_NUM], core->neighbor_motions, core->best_cu_mvfield);
                    }
                }
#endif
                mod_info_curr->mv[REFP_0][MV_X] = skip_pmv_cands[spatial_skip_idx][REFP_0][MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = skip_pmv_cands[spatial_skip_idx][REFP_0][MV_Y];
                mod_info_curr->mv[REFP_1][MV_X] = skip_pmv_cands[spatial_skip_idx][REFP_1][MV_X];
                mod_info_curr->mv[REFP_1][MV_Y] = skip_pmv_cands[spatial_skip_idx][REFP_1][MV_Y];
                mod_info_curr->refi[REFP_0] = skip_refi[spatial_skip_idx][REFP_0];
                mod_info_curr->refi[REFP_1] = skip_refi[spatial_skip_idx][REFP_1];
#if BGC
                mod_info_curr->bgc_flag = bgc_flag_cands[spatial_skip_idx];
                mod_info_curr->bgc_idx  = bgc_idx_cands[spatial_skip_idx];
#endif
            }
            else
            {
                int umve_idx = mod_info_curr->umve_idx;
                s16 pmv_base_cands[UMVE_BASE_NUM][REFP_NUM][MV_D];
                s8 refi_base_cands[UMVE_BASE_NUM][REFP_NUM];
#if UMVE_ENH
                s16 pmv_umve_cands[UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM][REFP_NUM][MV_D];
                s8 refi_umve_cands[UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM][REFP_NUM];
#else
                s16 pmv_umve_cands[UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM][REFP_NUM][MV_D];
                s8 refi_umve_cands[UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM][REFP_NUM];
#endif
#if BGC
                s8 base_bgc_flag[UMVE_BASE_NUM];
                s8 base_bgc_idx[UMVE_BASE_NUM];
#if UMVE_ENH
                s8 final_bgc_flag[UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM];
                s8 final_bgc_idx[UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM];
                memset(final_bgc_flag, 0, (UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM) * sizeof(s8));
                memset(final_bgc_idx, 0, (UMVE_MAX_REFINE_NUM_SEC_SET*UMVE_BASE_NUM) * sizeof(s8));
#else
                s8 final_bgc_flag[UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM];
                s8 final_bgc_idx[UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM];
                memset(final_bgc_flag, 0, (UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM) * sizeof(s8));
                memset(final_bgc_idx, 0, (UMVE_MAX_REFINE_NUM*UMVE_BASE_NUM) * sizeof(s8));
#endif
#endif
#if BGC
                derive_umve_base_motions(&ctx->info, mod_info_curr, &ctx->map, pmv_temp, refi_temp, pmv_base_cands, refi_base_cands, base_bgc_flag, base_bgc_idx);
#else
                derive_umve_base_motions(&ctx->info, mod_info_curr, &ctx->map, pmv_temp, refi_temp, pmv_base_cands, refi_base_cands);
#endif
#if UMVE_ENH
#if BGC
                derive_umve_final_motions(umve_idx, ctx->refp, ctx->ptr, pmv_base_cands, refi_base_cands, pmv_umve_cands, refi_umve_cands, (BOOL)ctx->info.pic_header.umve_set_flag, base_bgc_flag, base_bgc_idx, final_bgc_flag, final_bgc_idx);
#else
                derive_umve_final_motions(umve_idx, ctx->refp, ctx->ptr, pmv_base_cands, refi_base_cands, pmv_umve_cands, refi_umve_cands, (BOOL)ctx->info.pic_header.umve_set_flag);
#endif
#else
#if BGC
                derive_umve_final_motions(umve_idx, ctx->refp, ctx->ptr, pmv_base_cands, refi_base_cands, pmv_umve_cands, refi_umve_cands, base_bgc_flag, base_bgc_idx, final_bgc_flag, final_bgc_idx);
#else
                derive_umve_final_motions(umve_idx, ctx->refp, ctx->ptr, pmv_base_cands, refi_base_cands, pmv_umve_cands, refi_umve_cands);
#endif
#endif
                mod_info_curr->mv[REFP_0][MV_X] = pmv_umve_cands[umve_idx][REFP_0][MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = pmv_umve_cands[umve_idx][REFP_0][MV_Y];
                mod_info_curr->mv[REFP_1][MV_X] = pmv_umve_cands[umve_idx][REFP_1][MV_X];
                mod_info_curr->mv[REFP_1][MV_Y] = pmv_umve_cands[umve_idx][REFP_1][MV_Y];
                mod_info_curr->refi[REFP_0] = refi_umve_cands[umve_idx][REFP_0];
                mod_info_curr->refi[REFP_1] = refi_umve_cands[umve_idx][REFP_1];
#if BGC
                mod_info_curr->bgc_flag = final_bgc_flag[umve_idx];
                mod_info_curr->bgc_idx  = final_bgc_idx[umve_idx];
#endif
            }
#if ETMVP
        }
#endif
    }
}
#if IBC_BVP
void dec_derive_bvp_list(u8 slice_type, u8 allowed_hbvp_num, COM_MODE *mod_info_curr, COM_BLOCK_MOTION* block_motion_cands, s8 cnt_hbvp_cands, s16(*bvp_cands)[MV_D], int pic_width, int pic_height, u32 *map_scu
#if USE_SP
    , u8 *map_usp
#endif
    , int log2_max_cuwh, s16(*map_mv)[REFP_NUM][MV_D])
{
    mod_info_curr->cnt_hbvp_cands = cnt_hbvp_cands;
    if (allowed_hbvp_num && (cnt_hbvp_cands >= 2))
    {
        int idx;
        int cntCheck = 0;
        int cbvp_valid[MAX_NUM_BVP];
        s16 cbvp_cands[MAX_NUM_BVP][MV_D];

        for (idx = 0; idx < MAX_NUM_BVP; idx++)
        {
            cbvp_valid[idx] = 0;
            cbvp_cands[idx][MV_X] = 0;
            cbvp_cands[idx][MV_Y] = 0;
        }

        s16 motion_spatial_candi[5][3] = { {0,}, };
        if (slice_type == SLICE_I)
        {
            get_spatial_motion_nonadj_bsv(mod_info_curr, pic_width, pic_height, map_scu
#if USE_SP
                , map_usp
#endif
                , log2_max_cuwh, motion_spatial_candi, map_mv);
        }
        else
        {
            get_spatial_motion_adj_sv(mod_info_curr, pic_width, pic_height, map_scu
#if USE_SP
                , map_usp
#endif
                , motion_spatial_candi, map_mv);
        }

        int x_pos = mod_info_curr->x_pos;
        int y_pos = mod_info_curr->y_pos;
        int w = mod_info_curr->cu_width;
        int h = mod_info_curr->cu_height;


        if (mod_info_curr->cbvp_idx == 0)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 1)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 2)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                }
                else if (motion.x < x_pos && motion.y >= y_pos && motion.y < y_pos + h && cbvp_valid[2] == 0)                  // Left
                {
                    copy_mv(cbvp_cands[2], motion.mv);
                    cbvp_valid[2] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 3)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                }
                else if (motion.y < y_pos && motion.x >= x_pos && motion.x < x_pos + w && cbvp_valid[3] == 0)                  // Top
                {
                    copy_mv(cbvp_cands[3], motion.mv);
                    cbvp_valid[3] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 4)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                }
                else if (motion.x < x_pos && motion.y < y_pos && cbvp_valid[4] == 0)   // Left-Top
                {
                    copy_mv(cbvp_cands[4], motion.mv);
                    cbvp_valid[4] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 5)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                }
                else if (motion.x >= x_pos + w && motion.y < y_pos && cbvp_valid[5] == 0)             // Right-Top
                {
                    copy_mv(cbvp_cands[5], motion.mv);
                    cbvp_valid[5] = 1;
                    break;
                }
            }
        }
        else if (mod_info_curr->cbvp_idx == 6)
        {
            for (idx = cnt_hbvp_cands - 1; idx >= 0; idx--)
            {
                COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if USE_SP
                if (motion.len > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#else
                if (motion.w * motion.h > CBVP_TH_SIZE && cbvp_valid[0] == 0)
#endif
                {
                    copy_mv(cbvp_cands[0], motion.mv);
                    cbvp_valid[0] = 1;
                }
                else if (motion.cnt > CBVP_TH_CNT && cbvp_valid[1] == 0)
                {
                    copy_mv(cbvp_cands[1], motion.mv);
                    cbvp_valid[1] = 1;
                }
                else if (motion.y >= y_pos + h && motion.x < x_pos && cbvp_valid[6] == 0)             // Left-Bottom
                {
                    copy_mv(cbvp_cands[6], motion.mv);
                    cbvp_valid[6] = 1;
                    break;
                }
            }
        }

        if (cbvp_valid[mod_info_curr->cbvp_idx])
        {
            copy_mv(bvp_cands[mod_info_curr->cbvp_idx], cbvp_cands[mod_info_curr->cbvp_idx]);
        }
        else
        {
            if (mod_info_curr->cbvp_idx == 0 || mod_info_curr->cbvp_idx == 2 || mod_info_curr->cbvp_idx == 4 || mod_info_curr->cbvp_idx == 6)
            {
                if (motion_spatial_candi[0][2] == 1)
                {
                    s16 tmp_mv[2] = { motion_spatial_candi[0][0],motion_spatial_candi[0][1] };
                    copy_mv(bvp_cands[mod_info_curr->cbvp_idx], tmp_mv);
                }
                else
                {
                    if (cnt_hbvp_cands >= 2)
                    {
                        s16 tmp_mv[2] = { 0 };
                        if (mod_info_curr->cbvp_idx < cnt_hbvp_cands)
                        {
                            int tmp_idx = cnt_hbvp_cands % (mod_info_curr->cbvp_idx + 1);
                            tmp_mv[0] = block_motion_cands[tmp_idx].mv[0];
                            tmp_mv[1] = block_motion_cands[tmp_idx].mv[1];
                        }
                        else
                        {
                            tmp_mv[0] = block_motion_cands[0].mv[0];
                            tmp_mv[1] = block_motion_cands[0].mv[1];
                        }
                        copy_mv(bvp_cands[mod_info_curr->cbvp_idx], tmp_mv);
                    }
                }
            }
            else
            {
                if (motion_spatial_candi[1][2] == 1)
                {
                    s16 tmp_mv[2] = { motion_spatial_candi[1][0],motion_spatial_candi[1][1] };
                    copy_mv(bvp_cands[mod_info_curr->cbvp_idx], tmp_mv);
                }
                else
                {
                    if (cnt_hbvp_cands >= 2)
                    {
                        s16 tmp_mv[2] = { 0 };
                        if (mod_info_curr->cbvp_idx < cnt_hbvp_cands)
                        {
                            int tmp_idx = cnt_hbvp_cands % (mod_info_curr->cbvp_idx + 1);
                            tmp_mv[0] = block_motion_cands[tmp_idx].mv[0];
                            tmp_mv[1] = block_motion_cands[tmp_idx].mv[1];
                        }
                        else
                        {
                            tmp_mv[0] = block_motion_cands[0].mv[0];
                            tmp_mv[1] = block_motion_cands[0].mv[1];
                    }
                        copy_mv(bvp_cands[mod_info_curr->cbvp_idx], tmp_mv);
                    }
                }
            }
        }

    }
}
void dec_derive_ibc_bvp_info(DEC_CTX * ctx, DEC_CORE * core, s16 bvp[MV_D])
{
    int i, cnt_class_cands = 0;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16 bvp_cands[MAX_NUM_BVP][MV_D];

    for (i = 0;i < MAX_NUM_BVP;i++)
    {
        bvp_cands[i][MV_X] = 0;
        bvp_cands[i][MV_Y] = 0;
    }

    dec_derive_bvp_list(ctx->info.pic_header.slice_type, ctx->info.sqh.num_of_hbvp_cand, &core->mod_info_curr, core->block_motion_cands, core->cnt_hbvp_cands, bvp_cands, ctx->info.pic_width, ctx->info.pic_height, ctx->map.map_scu
#if USE_SP
        , ctx->map.map_usp
#endif
        , ctx->info.log2_max_cuwh, ctx->map.map_mv);

    if (core->cnt_hbvp_cands >= 2)
    {
        bvp[MV_X] = bvp_cands[mod_info_curr->cbvp_idx][MV_X] << 2;
        bvp[MV_Y] = bvp_cands[mod_info_curr->cbvp_idx][MV_Y] << 2;
    }
}
#endif
