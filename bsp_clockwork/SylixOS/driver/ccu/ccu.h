/*
 * ccu.h
 *
 *  Created on: Sep 20, 2019
 *      Author: databus
 */

#ifndef SYLIXOS_DRIVER_CCU_CCU_H_
#define SYLIXOS_DRIVER_CCU_CCU_H_

int ccu_mmc_output_clk_set_phase(int channel, int degrees);
int ccu_mmc_sample_clk_set_phase(int channel, int degrees);
int ccu_bus_mmc_soft_reset(int channel);
int ccu_mmc_clk_enable(int channel, BOOL enable);
int ccu_mmc_clk_rate_set(int channel, int rate);

#endif /* SYLIXOS_DRIVER_CCU_CCU_H_ */
