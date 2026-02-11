#include "icm20948.h"
#include "icm20948_registers.h"
#include "ak09916_registers.h"
#include "freertos/FreeRTOS.h"

//#include "freertos/FreeRTOS.h"

const icm20948_serif_t NullSerif = {
    NULL, // write
    NULL, // read
    NULL, // user
};

// Private function prototypes

// Function definitions
icm20948_status_e icm20948_init_struct(icm20948_device_t *pdev)
{
  // Initialize all elements by 0 except for _last_bank
  // Initialize _last_bank to 4 (invalid bank number)
  // so icm20948_set_bank function does not skip issuing bank change operation
  static const icm20948_device_t init_device = { ._last_bank = 4 };
  *pdev = init_device;
  return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_link_serif(icm20948_device_t *pdev, const icm20948_serif_t *s)
{
  if (s == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  if (pdev == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  pdev->_serif = s;
  return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_execute_w(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->write == NULL)
  {
    return ICM_20948_STAT_NOT_IMPL;
  }
  return (*pdev->_serif->write)(regaddr, pdata, len, pdev->_serif->user);
}

icm20948_status_e icm20948_execute_r(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->read == NULL)
  {
    return ICM_20948_STAT_NOT_IMPL;
  }
  return (*pdev->_serif->read)(regaddr, pdata, len, pdev->_serif->user);
}

//Transact directly with an I2C device, one byte at a time
//Used to configure a device before it is setup into a normal 0-3 peripheral slot
icm20948_status_e icm20948_i2c_controller_periph4_txn(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  // Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
  icm20948_status_e retval = ICM_20948_STAT_OK;

  addr = (((Rw) ? 0x80 : 0x00) | addr);

  retval = icm20948_set_bank(pdev, 3);
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_ADDR, (uint8_t *)&addr, 1);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_set_bank(pdev, 3);
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_REG, (uint8_t *)&reg, 1);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_i2c_periph4_ctrl_t ctrl;
  ctrl.EN = 1;
  ctrl.INT_EN = false;
  ctrl.DLY = 0;
  ctrl.REG_DIS = !send_reg_addr;

  icm20948_i2c_mst_status_t i2c_mst_status;
  bool txn_failed = false;
  uint16_t nByte = 0;

  while (nByte < len)
  {
    if (!Rw)
    {
      retval = icm20948_set_bank(pdev, 3);
      retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_DO, (uint8_t *)&(data[nByte]), 1);
      if (retval != ICM_20948_STAT_OK)
      {
        return retval;
      }
    }

    // Kick off txn
    retval = icm20948_set_bank(pdev, 3);
    retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_periph4_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    // long tsTimeout = millis() + 3000;  // Emergency timeout for txn (hard coded to 3 secs)
    uint32_t max_cycles = 1000;
    uint32_t count = 0;
    bool peripheral4Done = false;
    while (!peripheral4Done)
    {
      retval = icm20948_set_bank(pdev, 0);
      retval = icm20948_execute_r(pdev, AGB0_REG_I2C_MST_STATUS, (uint8_t *)&i2c_mst_status, 1);

      peripheral4Done = (i2c_mst_status.I2C_PERIPH4_DONE /*| (millis() > tsTimeout) */); //Avoid forever-loops
      peripheral4Done |= (count >= max_cycles);
      count++;
    }
    txn_failed = (i2c_mst_status.I2C_PERIPH4_NACK /*| (millis() > tsTimeout) */);
    txn_failed |= (count >= max_cycles);
    if (txn_failed)
      break;

    if (Rw)
    {
      retval = icm20948_set_bank(pdev, 3);
      retval = icm20948_execute_r(pdev, AGB3_REG_I2C_PERIPH4_DI, &data[nByte], 1);
    }

    nByte++;
  }

  if (txn_failed)
  {
    //We often fail here if mag is stuck
    return ICM_20948_STAT_ERR;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_single_w(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data)
{
  return icm20948_i2c_controller_periph4_txn(pdev, addr, reg, data, 1, false, true);
}

icm20948_status_e icm20948_i2c_master_single_r(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data)
{
  return icm20948_i2c_controller_periph4_txn(pdev, addr, reg, data, 1, true, true);
}

icm20948_status_e icm20948_set_bank(icm20948_device_t *pdev, uint8_t bank)
{
  if (bank > 3)
  {
    return ICM_20948_STAT_PARAM_ERR;
  } // Only 4 possible banks

  if (bank == pdev->_last_bank) // Do we need to change bank?
    return ICM_20948_STAT_OK;   // Bail if we don't need to change bank to avoid unnecessary bus traffic

  pdev->_last_bank = bank;   // Store the requested bank (before we bit-shift)
  bank = (bank << 4) & 0x30; // bits 5:4 of REG_BANK_SEL
  return icm20948_execute_w(pdev, REG_BANK_SEL, &bank, 1);
}

icm20948_status_e icm20948_sw_reset(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  reg.DEVICE_RESET = 1;

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_sleep(icm20948_device_t *pdev, bool on)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (on)
  {
    reg.SLEEP = 1;
  }
  else
  {
    reg.SLEEP = 0;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_low_power(icm20948_device_t *pdev, bool on)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (on)
  {
    reg.LP_EN = 1;
  }
  else
  {
    reg.LP_EN = 0;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_set_clock_source(icm20948_device_t *pdev, icm20948_pwr_mgmt_1_clksel_e source)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  reg.CLKSEL = source;

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_get_who_am_i(icm20948_device_t *pdev, uint8_t *whoami)
{
  if (whoami == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  icm20948_set_bank(pdev, 0); // Must be in the right bank
  return icm20948_execute_r(pdev, AGB0_REG_WHO_AM_I, whoami, 1);
}

icm20948_status_e icm20948_check_id(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  uint8_t whoami = 0x00;
  retval = icm20948_get_who_am_i(pdev, &whoami);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (whoami != ICM_20948_WHOAMI)
  {
    return ICM_20948_STAT_WRONG_ID;
  }
  return retval;
}

icm20948_status_e icm20948_data_ready(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_int_status_1_t reg;
  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_INT_STATUS_1, (uint8_t *)&reg, sizeof(icm20948_int_status_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (!reg.RAW_DATA_0_RDY_INT)
  {
    retval = ICM_20948_STAT_NO_DATA;
  }
  return retval;
}

// Interrupt Configuration
icm20948_status_e icm20948_int_pin_cfg(icm20948_device_t *pdev, icm20948_int_pin_cfg_t *write, icm20948_int_pin_cfg_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (write != NULL)
  { // write first, if available
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)write, sizeof(icm20948_int_pin_cfg_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }
  if (read != NULL)
  { // then read, to allow for verification
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)read, sizeof(icm20948_int_pin_cfg_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }
  return retval;
}

icm20948_status_e icm20948_int_enable(icm20948_device_t *pdev, icm20948_int_enable_t *write, icm20948_int_enable_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_int_enable_0_t en_0;
  icm20948_int_enable_1_t en_1;
  icm20948_int_enable_2_t en_2;
  icm20948_int_enable_3_t en_3;

  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    en_0.I2C_MST_INT_EN = write->I2C_MST_INT_EN;
    en_0.DMP_INT1_EN = write->DMP_INT1_EN;
    en_0.PLL_READY_EN = write->PLL_RDY_EN;
    en_0.WOM_INT_EN = write->WOM_INT_EN;
    en_0.reserved_0 = 0; // Clear RAM garbage
    en_0.REG_WOF_EN = write->REG_WOF_EN;
    en_1.RAW_DATA_0_RDY_EN = write->RAW_DATA_0_RDY_EN;
    en_1.reserved_0 = 0; // Clear RAM garbage
    en_2.individual.FIFO_OVERFLOW_EN_4 = write->FIFO_OVERFLOW_EN_4;
    en_2.individual.FIFO_OVERFLOW_EN_3 = write->FIFO_OVERFLOW_EN_3;
    en_2.individual.FIFO_OVERFLOW_EN_2 = write->FIFO_OVERFLOW_EN_2;
    en_2.individual.FIFO_OVERFLOW_EN_1 = write->FIFO_OVERFLOW_EN_1;
    en_2.individual.FIFO_OVERFLOW_EN_0 = write->FIFO_OVERFLOW_EN_0;
    en_2.individual.reserved_0 = 0; // Clear RAM garbage
    en_3.individual.FIFO_WM_EN_4 = write->FIFO_WM_EN_4;
    en_3.individual.FIFO_WM_EN_3 = write->FIFO_WM_EN_3;
    en_3.individual.FIFO_WM_EN_2 = write->FIFO_WM_EN_2;
    en_3.individual.FIFO_WM_EN_1 = write->FIFO_WM_EN_1;
    en_3.individual.FIFO_WM_EN_0 = write->FIFO_WM_EN_0;
    en_3.individual.reserved_0 = 0; // Clear RAM garbage

    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(icm20948_int_enable_0_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(icm20948_int_enable_1_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(icm20948_int_enable_2_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(icm20948_int_enable_3_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(icm20948_int_enable_0_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(icm20948_int_enable_1_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(icm20948_int_enable_2_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(icm20948_int_enable_3_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->I2C_MST_INT_EN = en_0.I2C_MST_INT_EN;
    read->DMP_INT1_EN = en_0.DMP_INT1_EN;
    read->PLL_RDY_EN = en_0.PLL_READY_EN;
    read->WOM_INT_EN = en_0.WOM_INT_EN;
    read->REG_WOF_EN = en_0.REG_WOF_EN;
    read->RAW_DATA_0_RDY_EN = en_1.RAW_DATA_0_RDY_EN;
    read->FIFO_OVERFLOW_EN_4 = en_2.individual.FIFO_OVERFLOW_EN_4;
    read->FIFO_OVERFLOW_EN_3 = en_2.individual.FIFO_OVERFLOW_EN_3;
    read->FIFO_OVERFLOW_EN_2 = en_2.individual.FIFO_OVERFLOW_EN_2;
    read->FIFO_OVERFLOW_EN_1 = en_2.individual.FIFO_OVERFLOW_EN_1;
    read->FIFO_OVERFLOW_EN_0 = en_2.individual.FIFO_OVERFLOW_EN_0;
    read->FIFO_WM_EN_4 = en_3.individual.FIFO_WM_EN_4;
    read->FIFO_WM_EN_3 = en_3.individual.FIFO_WM_EN_3;
    read->FIFO_WM_EN_2 = en_3.individual.FIFO_WM_EN_2;
    read->FIFO_WM_EN_1 = en_3.individual.FIFO_WM_EN_1;
    read->FIFO_WM_EN_0 = en_3.individual.FIFO_WM_EN_0;
  }

  return retval;
}

icm20948_status_e icm20948_wom_logic(icm20948_device_t *pdev, icm20948_accel_intel_ctrl_t *write, icm20948_accel_intel_ctrl_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_accel_intel_ctrl_t ctrl;

  retval = icm20948_set_bank(pdev, 2); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    ctrl.ACCEL_INTEL_EN = write->ACCEL_INTEL_EN;
    ctrl.ACCEL_INTEL_MODE_INT = write->ACCEL_INTEL_MODE_INT;

    retval = icm20948_execute_w(pdev, AGB2_REG_ACCEL_INTEL_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_accel_intel_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB2_REG_ACCEL_INTEL_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_accel_intel_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->ACCEL_INTEL_EN = ctrl.ACCEL_INTEL_EN;
    read->ACCEL_INTEL_MODE_INT = ctrl.ACCEL_INTEL_MODE_INT;
  }

  return retval;
}

icm20948_status_e icm20948_wom_threshold(icm20948_device_t *pdev, icm20948_accel_wom_thr_t *write, icm20948_accel_wom_thr_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_accel_wom_thr_t thr;

  retval = icm20948_set_bank(pdev, 2); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    thr.WOM_THRESHOLD = write->WOM_THRESHOLD;

    retval = icm20948_execute_w(pdev, AGB2_REG_ACCEL_WOM_THR, (uint8_t *)&thr, sizeof(icm20948_accel_wom_thr_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB2_REG_ACCEL_WOM_THR, (uint8_t *)&thr, sizeof(icm20948_accel_wom_thr_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->WOM_THRESHOLD = thr.WOM_THRESHOLD;
  }

  return retval;
}

icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_lp_config_t reg;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR | ICM_20948_INTERNAL_MST)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    reg.ACCEL_CYCLE = mode;
  } // Set all desired sensors to this setting
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    reg.GYRO_CYCLE = mode;
  }
  if (sensors & ICM_20948_INTERNAL_MST)
  {
    reg.I2C_MST_CYCLE = mode;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Check the data was written correctly
  retval = icm20948_execute_r(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    if (reg.ACCEL_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    if (reg.GYRO_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_MST)
  {
    if (reg.I2C_MST_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }

  return retval;
}

icm20948_status_e icm20948_set_full_scale(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_fss_t fss)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    reg.ACCEL_FS_SEL = fss.a;
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (reg.ACCEL_FS_SEL != fss.a) retval |= ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    reg.GYRO_FS_SEL = fss.g;
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (reg.GYRO_FS_SEL != fss.g) retval |= ICM_20948_STAT_ERR;
  }
  return retval;
}

icm20948_status_e icm20948_set_dlpf_cfg(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_dlpcfg_t cfg)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    reg.ACCEL_DLPFCFG = cfg.a;
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (reg.ACCEL_DLPFCFG != cfg.a) retval |= ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    reg.GYRO_DLPFCFG = cfg.g;
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (reg.GYRO_DLPFCFG != cfg.g) retval |= ICM_20948_STAT_ERR;
  }
  return retval;
}

icm20948_status_e icm20948_enable_dlpf(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (enable)
    {
      reg.ACCEL_FCHOICE = 1;
    }
    else
    {
      reg.ACCEL_FCHOICE = 0;
    }
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (enable)
    {
      if (reg.ACCEL_FCHOICE != 1) retval |= ICM_20948_STAT_ERR;
    }
    else
    {
      if (reg.ACCEL_FCHOICE != 0) retval |= ICM_20948_STAT_ERR;
    }
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (enable)
    {
      reg.GYRO_FCHOICE = 1;
    }
    else
    {
      reg.GYRO_FCHOICE = 0;
    }
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (enable)
    {
      if (reg.GYRO_FCHOICE != 1) retval |= ICM_20948_STAT_ERR;
    }
    else
    {
      if (reg.GYRO_FCHOICE != 0) retval |= ICM_20948_STAT_ERR;
    }
  }
  return retval;
}

icm20948_status_e icm20948_set_sample_rate(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_smplrt_t smplrt)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    uint8_t div1 = (smplrt.a >> 8); // Thank you @yanivamichy #109
    uint8_t div2 = (smplrt.a & 0xFF);
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_SMPLRT_DIV_1, &div1, 1);
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_SMPLRT_DIV_2, &div2, 1);
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    uint8_t div = (smplrt.g);
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_SMPLRT_DIV, &div, 1);
  }
  return retval;
}

// Interface Things
icm20948_status_e icm20948_i2c_master_passthrough(icm20948_device_t *pdev, bool passthrough)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_int_pin_cfg_t reg;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(icm20948_int_pin_cfg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  reg.BYPASS_EN = passthrough;
  retval = icm20948_execute_w(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(icm20948_int_pin_cfg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_enable(icm20948_device_t *pdev, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  // Disable BYPASS_EN
  retval = icm20948_i2c_master_passthrough(pdev, false);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_i2c_mst_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 3);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_mst_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  ctrl.I2C_MST_CLK = 0x07; // corresponds to 345.6 kHz, good for up to 400 kHz
  ctrl.I2C_MST_P_NSR = 1;
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_mst_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_user_ctrl_t reg;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (enable)
  {
    reg.I2C_MST_EN = 1;
  }
  else
  {
    reg.I2C_MST_EN = 0;
  }
  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_reset(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrl.I2C_MST_RST = 1; //Reset!

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_i2c_controller_configure_peripheral(icm20948_device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  uint8_t periph_addr_reg;
  uint8_t periph_reg_reg;
  uint8_t periph_ctrl_reg;
  uint8_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = AGB3_REG_I2C_PERIPH0_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH0_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH0_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH0_DO;
    break;
  case 1:
    periph_addr_reg = AGB3_REG_I2C_PERIPH1_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH1_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH1_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH1_DO;
    break;
  case 2:
    periph_addr_reg = AGB3_REG_I2C_PERIPH2_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH2_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH2_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH2_DO;
    break;
  case 3:
    periph_addr_reg = AGB3_REG_I2C_PERIPH3_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH3_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH3_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH3_DO;
    break;
  default:
    return ICM_20948_STAT_PARAM_ERR;
  }

  retval = icm20948_set_bank(pdev, 3);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Set the peripheral address and the Rw flag
  icm20948_i2c_periphx_addr_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  retval = icm20948_execute_w(pdev, periph_addr_reg, (uint8_t *)&address, sizeof(icm20948_i2c_periphx_addr_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    icm20948_i2c_periphx_do_t dataOutByte;
    dataOutByte.DO = dataOut;
    retval = icm20948_execute_w(pdev, periph_do_reg, (uint8_t *)&dataOutByte, sizeof(icm20948_i2c_periphx_do_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  // Set the peripheral sub-address (register address)
  icm20948_i2c_periphx_reg_t subaddress;
  subaddress.REG = reg;
  retval = icm20948_execute_w(pdev, periph_reg_reg, (uint8_t *)&subaddress, sizeof(icm20948_i2c_periphx_reg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Set up the control info
  icm20948_i2c_periphx_ctrl_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  retval = icm20948_execute_w(pdev, periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(icm20948_i2c_periphx_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

// Higher Level
icm20948_status_e icm20948_get_agmt(icm20948_device_t *pdev, icm20948_agmt_t *pagmt)
{
  if (pagmt == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }

  icm20948_status_e retval = ICM_20948_STAT_OK;
  const uint8_t numbytes = 14 + 9; //Read Accel, gyro, temp, and 9 bytes of mag
  uint8_t buff[numbytes];

  // Get readings
  retval |= icm20948_set_bank(pdev, 0);
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB0_REG_ACCEL_XOUT_H, buff, numbytes);

  pagmt->acc.axes.x = ((buff[0] << 8) | (buff[1] & 0xFF));
  pagmt->acc.axes.y = ((buff[2] << 8) | (buff[3] & 0xFF));
  pagmt->acc.axes.z = ((buff[4] << 8) | (buff[5] & 0xFF));

  pagmt->gyr.axes.x = ((buff[6] << 8) | (buff[7] & 0xFF));
  pagmt->gyr.axes.y = ((buff[8] << 8) | (buff[9] & 0xFF));
  pagmt->gyr.axes.z = ((buff[10] << 8) | (buff[11] & 0xFF));

  pagmt->tmp.val = ((buff[12] << 8) | (buff[13] & 0xFF));

  pagmt->magStat1 = buff[14];
  pagmt->mag.axes.x = ((buff[16] << 8) | (buff[15] & 0xFF)); //Mag data is read little endian
  pagmt->mag.axes.y = ((buff[18] << 8) | (buff[17] & 0xFF));
  pagmt->mag.axes.z = ((buff[20] << 8) | (buff[19] & 0xFF));
  pagmt->magStat2 = buff[21];

  // Get settings to be able to compute scaled values
  retval |= icm20948_set_bank(pdev, 2);
  icm20948_accel_config_t acfg;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t *)&acfg, 1 * sizeof(acfg));
  pagmt->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range
                                    // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
  retval |= icm20948_set_bank(pdev, 2);
  icm20948_gyro_config_1_t gcfg1;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&gcfg1, 1 * sizeof(gcfg1));
  pagmt->fss.g = gcfg1.GYRO_FS_SEL;
  icm20948_accel_config_2_t acfg2;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t *)&acfg2, 1 * sizeof(acfg2));

  return retval;
}

// FIFO

icm20948_status_e icm20948_enable_fifo(icm20948_device_t *pdev, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (enable)
    ctrl.FIFO_EN = 1;
  else
    ctrl.FIFO_EN = 0;

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_reset_fifo(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_rst_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrl.FIFO_RESET = 0x1F; // Datasheet says "FIFO_RESET[4:0]"

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  //delay ???

  ctrl.FIFO_RESET = 0x1E; // The InvenSense Nucleo examples write 0x1F followed by 0x1E

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_set_fifo_mode(icm20948_device_t *pdev, bool snapshot)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_mode_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(icm20948_fifo_mode_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (snapshot)
    ctrl.FIFO_MODE = 0x1F; // Datasheet says "FIFO_MODE[4:0]"
  else
    ctrl.FIFO_MODE = 0;

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(icm20948_fifo_mode_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_get_fifo_count(icm20948_device_t *pdev, uint16_t *count)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_counth_t ctrlh;
  icm20948_fifo_countl_t ctrll;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_COUNT_H, (uint8_t *)&ctrlh, sizeof(icm20948_fifo_counth_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrlh.FIFO_COUNTH &= 0x1F; // Datasheet says "FIFO_CNT[12:8]"

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_COUNT_L, (uint8_t *)&ctrll, sizeof(icm20948_fifo_countl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  *count = (((uint16_t)ctrlh.FIFO_COUNTH) << 8) | (uint16_t)ctrll.FIFO_COUNTL;

  return retval;
}

icm20948_status_e icm20948_read_fifo(icm20948_device_t *pdev, uint8_t *data, uint8_t len)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_R_W, data, len);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

// Magnetometer (AK09916)
icm20948_status_e icm20948_init_magnetometer(icm20948_device_t *pdev) {
    icm20948_status_e result;

    result = icm20948_i2c_master_enable(pdev, true);
    if (result != ICM_20948_STAT_OK) return result;

    result = icm20948_i2c_controller_configure_peripheral(pdev, 0, 
        MAG_AK09916_I2C_ADDR, 
        AK09916_REG_ST1,
        8,                // ST1 + 6 data bytes + ST2
        true,             // Read
        true,             // Enable
        false,            // data_only = false
        false,            // grp = false
        false,            // swap = false
        0);
    if (result != ICM_20948_STAT_OK) return result;

    result = icm20948_i2c_controller_configure_peripheral(pdev, 1,
        MAG_AK09916_I2C_ADDR,
        AK09916_REG_CNTL2,
        1,                // 1 byte
        false,            // Write
        true,             // Enable
        false,            // data_only = false
        false,            // grp = false  
        false,            // swap = false
        AK09916_MODE_CONT_50_HZ);
    if (result != ICM_20948_STAT_OK) return result;

    result = icm20948_set_bank(pdev, 3);
    uint8_t mstODRconfig  = 0x05; // Set the ODR configuration to 1100/2^5 = 34.375Hz
    result = icm20948_execute_w(pdev, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);

    return result;
}