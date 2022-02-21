/*! ------------------------------------------------------------------------------------------------------------------
 * @file    ext_api_fulltest.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <string.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
#include "../test_util/test_util.h"

int frst(void)
{
   int rv;
   int err_cnt = 0; 
   int delay_ms = 2000;
   
   HAL_Log("Factory reset.\n");
   rv = Test_CheckTxRx(dwm_factory_reset());
   HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
   HAL_Delay(delay_ms);
   err_cnt += rv; 
   
   if (rv == RV_OK)
   {
      dwm_deinit();
      dwm_init();
   }
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_pos(void)
{
   // ========== dwm_pos_set/get ==========
   int rv, err_cnt = 0; 
   int delay_ms = 1500;
   
   dwm_cfg_anchor_t cfg_an;    
   cfg_an.initiator = 1;
   cfg_an.bridge = 0;
   //cfg_an.uwb_bh_routing = DWM_UWB_BH_ROUTING_AUTO; 
   cfg_an.common.enc_en = 0;
   cfg_an.common.led_en = 0;
   cfg_an.common.ble_en = 0;
   cfg_an.common.uwb_mode = DWM_UWB_MODE_ACTIVE;//DWM_UWB_MODE_ACTIVE;
   cfg_an.common.fw_update_en = 1;
   HAL_Log("dwm_cfg_anchor_set(&cfg_an)\n");
   rv = Test_CheckTxRx(dwm_cfg_anchor_set(&cfg_an));
   Test_Report("dwm_cfg_anchor_set(&cfg_an):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;    
   
   rv = Test_CheckTxRx(dwm_reset());
   Test_Report("dwm_reset():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
   HAL_Delay(delay_ms);
   
   dwm_pos_t pos_set;
   HAL_Log("dwm_pos_set(pos_set)\n");
   pos_set.qf = 100;
   pos_set.x = 121;
   pos_set.y = 50;
   pos_set.z = 251;
   rv = Test_CheckTxRx(dwm_pos_set(&pos_set)); 
   Test_Report("dwm_pos_set(&pos_set):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;      
   
   dwm_pos_t pos_get;
   HAL_Log("dwm_pos_get(&pos_get)\n");
   rv = Test_CheckTxRx(dwm_pos_get(&pos_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tpos_get.x = %d\n", pos_get.x);
      HAL_Log("\t\tpos_get.y = %d\n", pos_get.y);
      HAL_Log("\t\tpos_get.z = %d\n", pos_get.z);
      HAL_Log("\t\tpos_get.qf = %d\n", pos_get.qf);
   }
   Test_Report("dwm_pos_get(&pos_get):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   rv = Test_CheckValue((pos_set.x != pos_get.x) || 
                      (pos_set.y != pos_get.y) || 
                      (pos_set.z != pos_get.z) || 
                      (pos_set.qf != pos_get.qf));
   err_cnt += rv;   
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}
   
int test_loc()
{
   // ========== dwm_loc_get ==========
   int rv, err_cnt = 0, i; 
   
   dwm_loc_data_t loc;
   dwm_pos_t pos;
   loc.p_pos = &pos;
   HAL_Log("dwm_loc_get(&loc)\n");
   rv = Test_CheckTxRx(dwm_loc_get(&loc));
   if(rv == RV_OK)
   {
      HAL_Log("[%d,%d,%d,%u]\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
            loc.p_pos->qf);

      for (i = 0; i < loc.anchors.dist.cnt; ++i) 
      {
         HAL_Log("%u)", i);
         HAL_Log("0x%llx", loc.anchors.dist.addr[i]);
         if (i < loc.anchors.an_pos.cnt) 
         {
            HAL_Log("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
                  loc.anchors.an_pos.pos[i].y,
                  loc.anchors.an_pos.pos[i].z,
                  loc.anchors.an_pos.pos[i].qf);
         }
         HAL_Log("=%u,%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
      }
   }
   Test_Report("dwm_loc_get(&loc):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

   
int test_upd_rate()
{
   // ========== dwm_upd_rate_set/get ==========
   int rv, err_cnt = 0; 
   
   uint16_t ur_set = 22;
   uint16_t ur_s_set = 50;
   HAL_Log("dwm_upd_rate_set(%d, %d)\n", ur_set, ur_s_set);
   rv = Test_CheckTxRx(dwm_upd_rate_set(ur_set, ur_s_set));
   Test_Report("dwm_upd_rate_set(ur, ur_s):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   uint16_t ur_get, ur_s_get;
   HAL_Log("dwm_upd_rate_get(&ur, &ur_s)\n");
   rv = Test_CheckTxRx(dwm_upd_rate_get(&ur_get, &ur_s_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tur  =%d \n", ur_get);
      HAL_Log("\t\tur_s=%d \n", ur_s_get);
   }
   Test_Report("dwm_upd_rate_get(&ur, &ur_s):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   
   rv = Test_CheckValue((ur_set != ur_get) || (ur_s_set != ur_s_get)); 
   err_cnt += rv;
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

   
int test_cfg()
{
   // ========== dwm_cfg_tag_set/dwm_cfg_anchor_set/dwm_cfg_get ==========
   int rv, err_cnt = 0;
   int delay_ms = 1500; 
   
   err_cnt += frst();

   dwm_cfg_anchor_t cfg_an;    
   cfg_an.initiator = 1;
   cfg_an.bridge = 0;
   //cfg_an.uwb_bh_routing = DWM_UWB_BH_ROUTING_AUTO; 
   cfg_an.common.enc_en = 0;
   cfg_an.common.led_en = 0;
   cfg_an.common.ble_en = 0;
   cfg_an.common.uwb_mode = DWM_UWB_MODE_OFF;//DWM_UWB_MODE_ACTIVE;
   cfg_an.common.fw_update_en = 1;
   HAL_Log("dwm_cfg_anchor_set(&cfg_an)\n");
   rv = Test_CheckTxRx(dwm_cfg_anchor_set(&cfg_an));
   Test_Report("dwm_cfg_anchor_set(&cfg_an):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;    
   
   rv = Test_CheckTxRx(dwm_reset());
   Test_Report("dwm_reset():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
   HAL_Delay(delay_ms);
   
   dwm_cfg_tag_t cfg_tag;
   cfg_tag.stnry_en = 1;
   cfg_tag.low_power_en = 0; 
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.common.enc_en = 0;
   cfg_tag.common.led_en = 0;
   cfg_tag.common.ble_en = 0;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
   cfg_tag.common.fw_update_en = 0;
   HAL_Log("dwm_cfg_tag_set(&cfg_tag)\n");
   rv = Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
   Test_Report("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = Test_CheckTxRx(dwm_reset());
   Test_Report("dwm_reset():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
   HAL_Delay(delay_ms);
    
   dwm_cfg_t cfg_node;    
   HAL_Log("dwm_cfg_get(&cfg_node):\n");
   rv = Test_CheckTxRx(dwm_cfg_get(&cfg_node));
   Test_Report("dwm_cfg_get(&cfg_node):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   err_cnt += Test_CheckTxRx(dwm_cfg_get(&cfg_node));
      
   HAL_Log("Comparing set vs. get.\n");
   rv = Test_CheckValue((cfg_tag.low_power_en   != cfg_node.low_power_en) 
                     || (cfg_tag.meas_mode      != cfg_node.meas_mode) 
                     || (cfg_tag.loc_engine_en  != cfg_node.loc_engine_en) 
                     || (cfg_tag.stnry_en       != cfg_node.stnry_en) 
                     || (cfg_tag.common.enc_en  != cfg_node.common.enc_en) 
                     || (cfg_tag.common.led_en  != cfg_node.common.led_en) 
                     || (cfg_tag.common.ble_en  != cfg_node.common.ble_en) 
                     || (cfg_tag.common.uwb_mode != cfg_node.common.uwb_mode) 
                     || (cfg_tag.common.fw_update_en != cfg_node.common.fw_update_en));
   if(rv != 0)
   {
      HAL_Log("\t\tcfg_node.common.fw_update_en  = %d\n", cfg_node.common.fw_update_en);
      HAL_Log("\t\tcfg_node.common.uwb_mode      = %d\n", cfg_node.common.uwb_mode);
      HAL_Log("\t\tcfg_node.common.ble_en        = %d\n", cfg_node.common.ble_en);
      HAL_Log("\t\tcfg_node.common.led_en        = %d\n", cfg_node.common.led_en);
      HAL_Log("\t\tcfg_node.common.enc_en        = %d\n", cfg_node.common.enc_en);
      HAL_Log("\t\tcfg_node.loc_engine_en        = %d\n", cfg_node.loc_engine_en);
      HAL_Log("\t\tcfg_node.low_power_en         = %d\n", cfg_node.low_power_en);
      HAL_Log("\t\tcfg_node.stnry_en             = %d\n", cfg_node.stnry_en);
      HAL_Log("\t\tcfg_node.meas_mode            = %d\n", cfg_node.meas_mode);
      HAL_Log("\t\tcfg_node.bridge               = %d\n", cfg_node.bridge);
      HAL_Log("\t\tcfg_node.initiator            = %d\n", cfg_node.initiator);
      HAL_Log("\t\tcfg_node.mode                 = %d\n", cfg_node.mode);
      //HAL_Log("\t\tcfg_node.uwb_bh_routing       = %d\n", cfg_node.uwb_bh_routing);
      err_cnt += rv;
   }    
   Test_Report("Comparing set vs. get:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}
   
int test_ver()
{
   // ========== dwm_ver_get ==========
   int rv, err_cnt = 0; 
 
   dwm_ver_t ver; 
   HAL_Log("dwm_ver_get(&ver)\n");
   rv = Test_CheckTxRx(dwm_ver_get(&ver));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tver.fw.maj  = %d\n", ver.fw.maj);
      HAL_Log("\t\tver.fw.min  = %d\n", ver.fw.min);
      HAL_Log("\t\tver.fw.patch= %d\n", ver.fw.patch);
      HAL_Log("\t\tver.fw.res  = %d\n", ver.fw.res);
      HAL_Log("\t\tver.fw.var  = %d\n", ver.fw.var);
      HAL_Log("\t\tver.cfg     = %08x\n", ver.cfg);
      HAL_Log("\t\tver.hw      = %08x\n", ver.hw);
   }
   Test_Report("dwm_ver_get(&ver):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_accel(void)
{
   // ========== stationary sensitivity get/set/get/set ==========
   int rv, err_cnt = 0;  
   dwm_stnry_sensitivity_t stnry_get, stnry_get2, stnry_set;
   
   HAL_Log("dwm_stnry_cfg_get:\n");
   rv = Test_CheckTxRx(dwm_stnry_cfg_get(&stnry_get));
   if(rv == RV_OK)
   {
      HAL_Log("stnry_get  = %d\n", stnry_get);
   }
   Test_Report("dwm_stnry_cfg_get(&stnry_get):\t\t\t%s\n", rv==RV_OK ? "pass":"fail");
   err_cnt += rv; 
   
   stnry_set = (stnry_get+1) % (DWM_STNRY_SENSITIVITY_HIGH+1);   
   
   HAL_Log("dwm_stnry_cfg_set: %d\n", stnry_set);
   rv = Test_CheckTxRx(dwm_stnry_cfg_set(stnry_set));
   Test_Report("dwm_stnry_cfg_set(stnry_set):\t\t\t%s\n", rv==RV_OK ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_stnry_cfg_get:\n");
   rv = Test_CheckTxRx(dwm_stnry_cfg_get(&stnry_get2));
   if(rv == RV_OK)
   {
      HAL_Log("stnry_get2 = %d\n", stnry_get2);
   }
   Test_Report("dwm_stnry_cfg_get(&stnry_get2):\t\t\t%s\n", rv==RV_OK ? "pass":"fail");
   err_cnt += rv; 
   
   rv = Test_CheckValue(stnry_get2!=stnry_set);
   Test_Report("dwm_stnry_cfg_get/set:\t\t\t%s\n", rv==RV_OK ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_stnry_cfg_set: %d\n", stnry_get);
   rv = Test_CheckTxRx(dwm_stnry_cfg_set(stnry_get));
   Test_Report("dwm_stnry_cfg_set(stnry_get):\t\t\t%s\n", rv==RV_OK ? "pass":"fail");
   err_cnt += rv; 

   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);   
   return err_cnt;
}

int test_gpio()
{
   // ========== dwm_gpio_: cfg_input/cfg_output/value_set/value_toggle ==========
   int rv, err_cnt = 0; 
   int i, j;
   uint8_t gpio_idx_list[]={
      DWM_GPIO_IDX_2,DWM_GPIO_IDX_8,DWM_GPIO_IDX_9,
      DWM_GPIO_IDX_10,DWM_GPIO_IDX_12,DWM_GPIO_IDX_13,
      DWM_GPIO_IDX_14,DWM_GPIO_IDX_15,DWM_GPIO_IDX_22,
      DWM_GPIO_IDX_23,DWM_GPIO_IDX_27,
      DWM_GPIO_IDX_30,DWM_GPIO_IDX_31 };
   bool gpio_val_list[]={false, true};
   uint8_t gpio_pull_list[]={DWM_GPIO_PIN_NOPULL,DWM_GPIO_PIN_PULLDOWN,DWM_GPIO_PIN_PULLUP};
 
   dwm_cfg_tag_t cfg_tag;      
   HAL_Log("Setup tag - No LED.\n");
   cfg_tag.low_power_en = 0; 
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.stnry_en = 0;
   cfg_tag.common.enc_en = 0;
   cfg_tag.common.led_en = 0;
   cfg_tag.common.ble_en = 0;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_OFF;
   cfg_tag.common.fw_update_en = 0;
   HAL_Log("dwm_cfg_tag_set(&cfg_tag).\n");
   err_cnt += Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
   HAL_Log("dwm_reset()\n");
   err_cnt += Test_CheckTxRx(dwm_reset());
   HAL_Log("Wait 1s for node to reset\n");
   HAL_Delay(1000);
   
   rv = 0;
   HAL_Log("dwm_gpio_cfg_input:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_pull_list); j++)
      {     
         HAL_Log("dwm_gpio_cfg_input(%d, %d)\n", gpio_idx_list[i], gpio_pull_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_idx_list[i], gpio_pull_list[j]));
      }
   }   
   Test_Report("dwm_gpio_cfg_input:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   HAL_Log("dwm_gpio_cfg_output:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {      
         HAL_Log("dwm_gpio_cfg_output(%d, %d)\n", gpio_idx_list[i], gpio_val_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_idx_list[i], gpio_val_list[j]));
      }
   }
   Test_Report("dwm_gpio_cfg_output:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
      
   rv = 0;
   HAL_Log("dwm_gpio_value_set:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {     
         HAL_Log("dwm_gpio_value_set(%d, %d)\n", gpio_idx_list[i], gpio_pull_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_value_set(gpio_idx_list[i], gpio_pull_list[j]));   
      }
   }
   Test_Report("dwm_gpio_value_set:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   static bool value_gpio_value_get;
   HAL_Log("dwm_gpio_value_get:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      HAL_Log("dwm_gpio_value_get(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_CheckTxRx(dwm_gpio_value_get(gpio_idx_list[i], &value_gpio_value_get));
   }
   Test_Report("dwm_gpio_value_get:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   HAL_Log("dwm_gpio_value_toggle\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      HAL_Log("dwm_gpio_value_toggle(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_CheckTxRx(dwm_gpio_value_toggle(gpio_idx_list[i])); 
   }
   Test_Report("dwm_gpio_value_toggle:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}


/**
 * @brief test gpio pins real toggle
 *
 */
int test_gpio1(void)
{
   int i, err_cnt = 0;  
   uint8_t gpio_conn_a[]={ DWM_GPIO_IDX_10, DWM_GPIO_IDX_9,  DWM_GPIO_IDX_12, DWM_GPIO_IDX_27, DWM_GPIO_IDX_23, DWM_GPIO_IDX_15};
   uint8_t gpio_conn_b[]={ DWM_GPIO_IDX_31, DWM_GPIO_IDX_22, DWM_GPIO_IDX_14, DWM_GPIO_IDX_30, DWM_GPIO_IDX_13, DWM_GPIO_IDX_8}; 
   bool out_val = true, in_val = true;
   
   dwm_cfg_tag_t cfg_tag;      
   HAL_Log("Setup tag - No LED.\n");
   cfg_tag.low_power_en = 0; 
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.stnry_en = 0;
   cfg_tag.common.led_en = 0;
   cfg_tag.common.ble_en = 1;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_OFF;
   cfg_tag.common.fw_update_en = 0;
   HAL_Log("dwm_cfg_tag_set(&cfg_tag).\n");
   err_cnt += Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
   HAL_Log("dwm_reset()\n");
   err_cnt += Test_CheckTxRx(dwm_reset());
   HAL_Log("Wait 1s for node to reset\n");
   HAL_Delay(1000);
   
   HAL_Log("Start pin test:\n");   
   HAL_Log("GPIO A-out, B-in\n");      
   for(i = 0; i < sizeof(gpio_conn_a); i++)
   {
      HAL_Log("GPIO out = gpio%d, in = gpio%d\n", gpio_conn_a[i], gpio_conn_b[i]);
      out_val = false;
      HAL_Log("dwm_gpio_cfg_input(gpio_%d, %d)\n", gpio_conn_b[i], DWM_GPIO_PIN_PULLUP);      
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_conn_b[i], DWM_GPIO_PIN_PULLUP));
      HAL_Log("dwm_gpio_cfg_output(gpio_%d, %d)\n", gpio_conn_a[i], out_val);     
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_conn_a[i], out_val));  
      HAL_Log("dwm_gpio_value_get(gpio_%d, &in_val)\n", gpio_conn_b[i]);     
      err_cnt += Test_CheckTxRx(dwm_gpio_value_get(gpio_conn_b[i], &in_val));
      err_cnt += Test_CheckValue(in_val != out_val);
      
      out_val = true;
      HAL_Log("dwm_gpio_cfg_input(gpio_%d, %d)\n", gpio_conn_b[i], DWM_GPIO_PIN_PULLDOWN);    
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_conn_b[i], DWM_GPIO_PIN_PULLDOWN));
      HAL_Log("dwm_gpio_cfg_output(gpio_%d, %d)\n", gpio_conn_a[i], out_val);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_conn_a[i], out_val));  
      HAL_Log("dwm_gpio_value_get(gpio_%d, &in_val)\n", gpio_conn_b[i]);
      err_cnt += Test_CheckTxRx(dwm_gpio_value_get(gpio_conn_b[i], &in_val));
      err_cnt += Test_CheckValue(in_val != out_val);
   }  
   
   HAL_Log("GPIO B-out, A-in\n");
   for(i = 0; i < sizeof(gpio_conn_a); i++)
   {
      HAL_Log("GPIO out = gpio%d, in = gpio%d\n", gpio_conn_b[i], gpio_conn_a[i]);      
      out_val = true;
      HAL_Log("dwm_gpio_cfg_output(gpio_%d, %d)\n", gpio_conn_b[i], out_val);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_conn_b[i], out_val));  
      HAL_Log("dwm_gpio_cfg_input(gpio_%d, %d)\n", gpio_conn_a[i], DWM_GPIO_PIN_PULLDOWN);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_conn_a[i], DWM_GPIO_PIN_PULLDOWN));
      HAL_Log("dwm_gpio_value_get(gpio_%d, &in_val)\n", gpio_conn_a[i]);
      err_cnt += Test_CheckTxRx(dwm_gpio_value_get(gpio_conn_a[i], &in_val));
      err_cnt += Test_CheckValue(in_val != out_val);
      
      out_val = false;
      HAL_Log("dwm_gpio_cfg_output(gpio_%d, %d)\n", gpio_conn_b[i], out_val);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_conn_b[i], out_val));  
      HAL_Log("dwm_gpio_cfg_input(gpio_%d, %d)\n", gpio_conn_a[i], DWM_GPIO_PIN_PULLUP);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_conn_a[i], DWM_GPIO_PIN_PULLUP));
      HAL_Log("dwm_gpio_value_get(gpio_%d, &in_val)\n", gpio_conn_a[i]);
      err_cnt += Test_CheckTxRx(dwm_gpio_value_get(gpio_conn_a[i], &in_val));
      err_cnt += Test_CheckValue(in_val != out_val);
      
      HAL_Log("dwm_gpio_cfg_input(gpio_%d, %d)\n", gpio_conn_b[i], DWM_GPIO_PIN_PULLUP);
      err_cnt += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_conn_b[i], DWM_GPIO_PIN_PULLUP));
   }  
   Test_Report("GPIO part test:\t\t\t%s\n", err_cnt==0 ? "pass":"fail");
   
   printf("%s %s: err_cnt = %d\n", err_cnt? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

/**
 * @brief Test BLE address get/set
 *
 */
int test_baddr(void)
{
   int rv, err_cnt = 0;  
   dwm_baddr_t baddr;
   uint8_t wait_reset_second = 1;
   int i = 0;
   
   //dwm_baddr_get(&baddr);
   HAL_Log("dwm_baddr_get(&baddr):\n");
   err_cnt += Test_CheckTxRx(dwm_baddr_get(&baddr)); 
   
   HAL_Log("sizeof(dwm_baddr_t) = %d\n", sizeof(dwm_baddr_t));   
   
   dwm_baddr_t temp_baddr;
   memcpy((uint8_t*)&temp_baddr, (uint8_t*)&baddr, sizeof(dwm_baddr_t));
   baddr.byte[0] += 1;
   HAL_Log("dwm_baddr_set(&baddr + 1):\n");
   err_cnt += Test_CheckTxRx(dwm_baddr_set(&baddr));
   HAL_Log("dwm_reset()\n");
   err_cnt += Test_CheckTxRx(dwm_reset());   
   HAL_Log("Wait %ds for device to reset...\n", wait_reset_second);
   HAL_Delay(wait_reset_second*1000);

   HAL_Log("dwm_baddr_get(&baddr + 1):\n");
   err_cnt += Test_CheckTxRx(dwm_baddr_get(&baddr)); 
   
   HAL_Log("Comparing read and write BLE addresses.\n");
   rv = Test_CheckValue(! ((baddr.byte[0] == temp_baddr.byte[0]+1)
                        && (baddr.byte[1] == temp_baddr.byte[1])
                        && (baddr.byte[2] == temp_baddr.byte[2])
                        && (baddr.byte[3] == temp_baddr.byte[3])
                        && (baddr.byte[4] == temp_baddr.byte[4])
                        && (baddr.byte[5] == temp_baddr.byte[5])));
   if(rv != 0)
   {
      HAL_Log("Comparison failed:\n");
      i = 0;
      HAL_Log("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]+1=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]+1);
      for (i = 1; i < 6; i++)
      {
         HAL_Log("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]);
      }
      err_cnt += rv; 
   }
                     
   baddr.byte[0] -= 1;
   HAL_Log("dwm_baddr_set(&baddr):\n");
   err_cnt += Test_CheckTxRx(dwm_baddr_set(&baddr));
   HAL_Log("dwm_reset().\n");
   err_cnt += Test_CheckTxRx(dwm_reset());   
   HAL_Log("Wait %ds for device to reset...\n", wait_reset_second);
   HAL_Delay(wait_reset_second*1000);

   HAL_Log("dwm_baddr_get(&baddr):\n");
   err_cnt += Test_CheckTxRx(dwm_baddr_get(&baddr)); 
   
   HAL_Log("Comparing read and write BLE addresses.\n");
   rv = Test_CheckValue(! ((baddr.byte[0] == temp_baddr.byte[0])
                        && (baddr.byte[1] == temp_baddr.byte[1])
                        && (baddr.byte[2] == temp_baddr.byte[2])
                        && (baddr.byte[3] == temp_baddr.byte[3])
                        && (baddr.byte[4] == temp_baddr.byte[4])
                        && (baddr.byte[5] == temp_baddr.byte[5])));
   if(rv != 0)
   {
      HAL_Log("Comparison failed:\n");
      for (i = 0; i < 6; i++)
      {
         HAL_Log("\t baddr.byte[%d]=0x%02x : temp_baddr.byte[%d]=0x%02x\n", i, baddr.byte[i], i, temp_baddr.byte[i]);
      }      
      err_cnt += rv; 
   }
   
   Test_Report("BLE address get/set part test:\t\t\t%s\n", err_cnt==0 ? "pass":"fail");
   
   printf("%s %s: err_cnt = %d\n", err_cnt? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_status()
{
   // ========== dwm_status_get ==========
   int rv, err_cnt = 0; 
   
   dwm_status_t status;
   HAL_Log("dwm_status_get(&status)\n");
   rv = Test_CheckTxRx(dwm_status_get(&status));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tstatus.loc_data            = %d\n", status.loc_data);
      HAL_Log("\t\tstatus.uwbmac_joined       = %d\n", status.uwbmac_joined);
      HAL_Log("\t\tstatus.bh_data_ready       = %d\n", status.bh_data_ready);
      HAL_Log("\t\tstatus.bh_status_changed   = %d\n", status.bh_status_changed);
      HAL_Log("\t\tstatus.bh_initialized      = %d\n", status.bh_initialized);
      HAL_Log("\t\tstatus.uwb_scan_ready      = %d\n", status.uwb_scan_ready);
      HAL_Log("\t\tstatus.usr_data_ready      = %d\n", status.usr_data_ready);
      HAL_Log("\t\tstatus.usr_data_sent       = %d\n", status.usr_data_sent);
      HAL_Log("\t\tstatus.fwup_in_progress    = %d\n", status.fwup_in_progress);
   }
   Test_Report("dwm_status_get(&status):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_int_cfg()
{
   // ========= dwm_int_cfg_get/set/get/set ===========
   int rv, err_cnt = 0; 

   uint16_t value_int_cfg_origin, value_int_cfg_set, value_int_cfg_get;
   
   HAL_Log("dwm_int_cfg_get(&origin)\n");
   rv = Test_CheckTxRx(dwm_int_cfg_get(&value_int_cfg_origin));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tvalue_int_cfg_origin = %d\n", value_int_cfg_origin);
   }
   Test_Report("dwm_int_cfg_get(&origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   value_int_cfg_set = value_int_cfg_origin | DWM1001_INTR_SPI_DATA_READY | DWM1001_INTR_LOC_READY;
   HAL_Log("dwm_int_cfg_set(%d)\n", value_int_cfg_set);
   rv = Test_CheckTxRx(dwm_int_cfg_set(value_int_cfg_set));
   Test_Report("dwm_int_cfg_set(value):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_int_cfg_get(&get)\n");
   rv = Test_CheckTxRx(dwm_int_cfg_get(&value_int_cfg_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tvalue_int_cfg_get    = %d\n", value_int_cfg_get);
   }
   Test_Report("dwm_int_cfg_get(&get):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   value_int_cfg_set = value_int_cfg_origin;
   HAL_Log("dwm_int_cfg_set(origin)\n");
   rv = Test_CheckTxRx(dwm_int_cfg_set(value_int_cfg_set));
   Test_Report("dwm_int_cfg_set(origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_enc()
{
   // ========= dwm_sec_enable/dwm_sec_disable =========== todo
   int rv, err_cnt = 0, i; 
   
   dwm_enc_key_t key;
   for(i = 0; i < DWM_ENC_KEY_LEN; i++)
   {
      key.byte[i] = i;
   }
   
   err_cnt += frst();
   
   HAL_Log("dwm_enc_key_set(&key)\n");
   rv = Test_CheckTxRx(dwm_enc_key_set(&key));
   Test_Report("dwm_enc_key_set(&key):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_enc_key_clear(void)\n");
   rv = Test_CheckTxRx(dwm_enc_key_clear());
   Test_Report("dwm_enc_key_clear(void):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_panid()
{
   // ========= dwm_panid_get/set =========== 
   int rv, err_cnt = 0; 
   
   uint16_t panid_origin, panid_get, panid_set;   
   
   HAL_Log("dwm_panid_get(&panid_origin);\n");
   rv = Test_CheckTxRx(dwm_panid_get(&panid_origin));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tpanid_origin   = 0x%04x\n", panid_origin);
   }
   Test_Report("dwm_panid_get(&panid_origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   panid_set = panid_origin + 1;   
   HAL_Log("dwm_panid_set(panid_set);\n");
   HAL_Log("\t\tpanid_set      = 0x%04x\n", panid_set);
   rv = Test_CheckTxRx(dwm_panid_set(panid_set));
   Test_Report("dwm_panid_set(panid_set):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   HAL_Log("dwm_reset()\n");
   Test_CheckTxRx(dwm_reset());
   HAL_Log("Wait 1s for node to reset.\n");
   HAL_Delay(1000);
   
   HAL_Log("dwm_panid_get(&panid_get);\n");
   rv = Test_CheckTxRx(dwm_panid_get(&panid_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tpanid_get      = 0x%04x\n", panid_get);
   }
   Test_Report("dwm_panid_get(&panid_get):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   rv = panid_get - panid_set;
   HAL_Log("\t\tpanid_get == panid_set?   %s\n", rv == 0?"pass":"fail");
   Test_Report("dwm_panid_set/get():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_panid_set(panid_origin);\n");
   rv = Test_CheckTxRx(dwm_panid_set(panid_origin));
   Test_Report("dwm_panid_set(panid_origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   HAL_Log("dwm_reset()\n");
   Test_CheckTxRx(dwm_reset());
   HAL_Log("Wait 1s for node to reset.\n");
   HAL_Delay(1000);  
   
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_label()
{
   // ========= dwm_label_read/write/read/write =========
   int rv, err_cnt = 0; 
   
   uint8_t label_origin[DWM_LABEL_LEN_MAX+1], len_origin;
   uint8_t label_get[DWM_LABEL_LEN_MAX+1], len_get;
   const char label_set[] = "label_test";
   label_origin[DWM_LABEL_LEN_MAX] = '\0';
   label_get[DWM_LABEL_LEN_MAX] = '\0';

   HAL_Log("dwm_label_read(label_origin, &len_origin);\n");
   rv = Test_CheckTxRx(dwm_label_read(label_origin, &len_origin));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tlabel_origin      = %s\n", label_origin);
      HAL_Log("\t\tlabel_origin_len  = %d\n", len_origin);
   }
   Test_Report("dwm_label_read(label_origin, &len_origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   HAL_Log("dwm_label_write(label_set, strlen(label_set));\n");
      HAL_Log("\t\tlabel_set         = %s\n", label_set);
      HAL_Log("\t\tlabel_set_len     = %d\n", strlen(label_set));
   rv = Test_CheckTxRx(dwm_label_write((uint8_t *)label_set, strlen(label_set)));
   Test_Report("dwm_label_write(label_set, strlen(label_set)):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   
   HAL_Log("dwm_label_read(label_get, &len_get);\n");
   rv = Test_CheckTxRx(dwm_label_read(label_get, &len_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tlabel_get         = %s\n", label_get);
      HAL_Log("\t\tlabel_get_len     = %d\n", len_get);
   }
   Test_Report("dwm_label_read(label_get, &len_get):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   rv = strcmp((const char *)label_get, label_set);
   HAL_Log("\t\tlabel_get == label_set?   %s\n", rv == 0?"pass":"fail");
   Test_Report("dwm_label_read/write():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_label_write(label_origin, strlen(label_origin));\n");
      HAL_Log("\t\tlabel_origin      = %s\n", label_origin);
      HAL_Log("\t\tlabel_origin_len  = %d\n", strlen((const char *)label_origin));
   rv = Test_CheckTxRx(dwm_label_write(label_origin, strlen((const char *)label_origin)));
   Test_Report("dwm_label_write(label_origin, strlen(label_origin)):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}


int test_usr_data()
{
   // ========= dwm_usr_data_read/write =========   
   int i, rv, err_cnt = 0; 
   uint8_t user_data[DWM_API_USR_DATA_LEN_MAX];
   uint8_t len, overwrite;
   
   HAL_Log("dwm_usr_data_read(user_data, &len);\n");
   rv = Test_CheckTxRx(dwm_usr_data_read(user_data, &len));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tuser_data_len  = %d\n", len);
      HAL_Log("\t\tuser_data_get  = %s", len>0?"0x":"");
      for (i = 0; i < len; i++)
      {
         HAL_Log(" %02x", user_data[i]);
      }      
      HAL_Log("\n");
   }
   Test_Report("dwm_usr_data_read(user_data, &len):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   HAL_Log("dwm_usr_data_write(user_data, len, overwrite);\n");
   len = DWM_API_USR_DATA_LEN_MAX;
   overwrite = 0;
   for(i=0; i< DWM_API_USR_DATA_LEN_MAX; i++)
   {
      user_data[i] = i;
   }
   rv = Test_CheckTxRx(dwm_usr_data_write(user_data, len, overwrite));
   Test_Report("dwm_usr_data_write(user_data, len, overwrite):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_bh_status()
{
   // ========== dwm_bh_status_get: node must be configured as bridge ========== 
   // This function is being used by KM all the time. Thus no need to test.
   int i, rv, err_cnt = 0; 
   
   // do frst and wait 10s until the KM re-configure the node to BN.
   err_cnt += frst();
   HAL_Delay(10000);
   
   HAL_Log("dwm_bh_status_get(p_bh_status);\n");
   bh_status_t bh_status;
   rv = Test_CheckTxRx(dwm_bh_status_get(&bh_status));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tbh_status.sf_number  = %d\n", bh_status.sf_number);
      HAL_Log("\t\tbh_status.bh_seat_map  = %d\n", bh_status.bh_seat_map);
      HAL_Log("\t\tbh_status.origin_cnt  = %d\n", bh_status.origin_cnt);
      for (i = 0; i < bh_status.origin_cnt; i++)
      {
         HAL_Log("\t\t\tbh_status.origin_info[%d].node_id  = %d \n", i, bh_status.origin_info[i].node_id);
         HAL_Log("\t\t\tbh_status.origin_info[%d].bh_seat  = %d \n", i, bh_status.origin_info[i].bh_seat);
         HAL_Log("\t\t\tbh_status.origin_info[%d].hop_lvl  = %d \n", i, bh_status.origin_info[i].hop_lvl);
      }      
      HAL_Log("\n");
   }
   Test_Report("dwm_bh_status_get(&p_bh_status):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_node_id()
{
   // ========== dwm_node_id_get ========== 
   int rv, err_cnt = 0; 
   
   HAL_Log("dwm_node_id_get(&node_id);\n");
   uint64_t node_id;
   rv = Test_CheckTxRx(dwm_node_id_get(&node_id));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tnode id:0x%llx \n", node_id);
   }
   Test_Report("dwm_node_id_get(&node_id):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;      
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

int test_uwb_cfg()
{
   // ========= dwm_uwb_cfg_get/set/get/set/ =========== todo
   int rv, err_cnt = 0; 
   
   dwm_uwb_cfg_t cfg_origin, cfg_set, cfg_get;

   HAL_Log("dwm_uwb_cfg_get(&origin)\n");
   rv = Test_CheckTxRx(dwm_uwb_cfg_get(&cfg_origin));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tcfg_origin.pg_delay = %d\n", cfg_origin.pg_delay);
      HAL_Log("\t\tcfg_origin.tx_power = %d\n", cfg_origin.tx_power);
   }
   Test_Report("dwm_uwb_cfg_get(&origin):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   cfg_set.tx_power = cfg_origin.tx_power + 1;
   HAL_Log("dwm_uwb_cfg_set(&cfg_set)\n");
   rv = Test_CheckTxRx(dwm_uwb_cfg_set(&cfg_set));
   Test_Report("dwm_uwb_cfg_set(&cfg_set):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Log("dwm_uwb_cfg_get(&cfg_get)\n");
   rv = Test_CheckTxRx(dwm_uwb_cfg_get(&cfg_get));
   if(rv == RV_OK)
   {
      HAL_Log("\t\tcfg_get.pg_delay             = %d\n", cfg_get.pg_delay);
      HAL_Log("\t\tcfg_get.tx_power             = %d\n", cfg_get.tx_power);
      HAL_Log("\t\tcfg_get.compensated.pg_delay = %d\n", cfg_get.compensated.pg_delay);
      HAL_Log("\t\tcfg_get.compensated.tx_power = %d\n", cfg_get.compensated.tx_power);
      HAL_Log("\t\t%s\n", cfg_get.tx_power == cfg_set.tx_power? "pass":"fail");      
   }
   Test_Report("\t\tdwm_int_cfg_get(&get)%s\n", cfg_get.tx_power == cfg_set.tx_power? "pass":"fail");
   err_cnt += rv; 
   
   cfg_set.tx_power = cfg_origin.tx_power;
   HAL_Log("dwm_uwb_cfg_set(&cfg_set)\n");
   rv = Test_CheckTxRx(dwm_uwb_cfg_set(&cfg_set));
   Test_Report("dwm_uwb_cfg_set(&cfg_set):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}


int test_preamble()
{
   // ========= dwm_uwb_preamble_code_get/dwm_uwb_preamble_code_set =========== todo
   int rv, err_cnt = 0; 
   dwm_uwb_preamble_code_t code = 0;
       
   HAL_Log("dwm_uwb_preamble_code_get(&code);\n");
   rv = Test_CheckTxRx(dwm_uwb_preamble_code_get(&code));
   Test_Report("dwm_uwb_preamble_code_get(&code)\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;
   
   if(code != 0)
   {
      HAL_Log("dwm_uwb_preamble_code_set(code);\n");
      rv = Test_CheckTxRx(dwm_uwb_preamble_code_set(code));
      Test_Report("dwm_uwb_preamble_code_set(code)\t\t\t%s\n", rv==0 ? "pass":"fail");
      err_cnt += rv;
   }
   else
   {
      HAL_Log("preamble code error: code = %d\n", code);
      err_cnt += 1;
   }
   
   printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
   return err_cnt;
}

void example_external_api_fulltest(void)
{   
   int err_cnt = 0;  
   
   printf("Initializing...\n");
   HAL_Log("Initializing...\n");
   dwm_init();
   err_cnt += frst();
   HAL_Log("Done\n");
   Test_Report("external_api_fulltest over interface: %s\n", HAL_IF_STR);
   
   /* ========= published APIs =========*/
   err_cnt += test_pos();   
   err_cnt += test_loc();   
   err_cnt += test_upd_rate();   
   err_cnt += test_cfg();   
   err_cnt += test_ver();       
   err_cnt += test_accel();
   err_cnt += test_gpio();
   err_cnt += test_gpio1();   
   err_cnt += test_baddr();  
   err_cnt += test_status();
   err_cnt += test_int_cfg();
   err_cnt += test_enc();
   err_cnt += test_panid();
   err_cnt += test_label();
   err_cnt += test_usr_data();
   err_cnt += test_node_id();
   err_cnt += test_uwb_cfg();
   // /*========= unpublished APIs =========*/
   err_cnt += test_preamble();
   
   
   // /*========= all done, reset module ========*/
   err_cnt += frst();
   
   HAL_Log("err_cnt = %d \n", err_cnt);
      
   Test_End();
}

int main(int argc, char*argv[])
{   
   int k=1;
   while(k-->0)
   {
      example_external_api_fulltest();
   }
   return 0;
}

