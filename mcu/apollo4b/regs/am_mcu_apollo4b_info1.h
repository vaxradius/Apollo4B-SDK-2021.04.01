//*****************************************************************************
//
//  am_mcu_apollo4b_info1.h
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20210111-833-gc25608de46 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_REG_INFO1_H
#define AM_REG_INFO1_H

#define AM_REG_INFO1_BASEADDR 0x42002000
#define AM_REG_INFO1n(n) 0x42002000

#define AM_REG_INFO1_SBL_VERSION_0_O 0x00001200
#define AM_REG_INFO1_SBL_VERSION_0_ADDR 0x42003200
#define AM_REG_INFO1_SBL_VERSION_1_O 0x00001204
#define AM_REG_INFO1_SBL_VERSION_1_ADDR 0x42003204
#define AM_REG_INFO1_SBR_VERSION_0_O 0x00001208
#define AM_REG_INFO1_SBR_VERSION_0_ADDR 0x42003208
#define AM_REG_INFO1_MAINPTR_O 0x00001210
#define AM_REG_INFO1_MAINPTR_ADDR 0x42003210
#define AM_REG_INFO1_RESETSTATUS_O 0x00001214
#define AM_REG_INFO1_RESETSTATUS_ADDR 0x42003214
#define AM_REG_INFO1_SBLOTA_O 0x00001218
#define AM_REG_INFO1_SBLOTA_ADDR 0x42003218
#define AM_REG_INFO1_SOCID0_O 0x00001220
#define AM_REG_INFO1_SOCID0_ADDR 0x42003220
#define AM_REG_INFO1_SOCID1_O 0x00001224
#define AM_REG_INFO1_SOCID1_ADDR 0x42003224
#define AM_REG_INFO1_SOCID2_O 0x00001228
#define AM_REG_INFO1_SOCID2_ADDR 0x42003228
#define AM_REG_INFO1_SOCID3_O 0x0000122c
#define AM_REG_INFO1_SOCID3_ADDR 0x4200322c
#define AM_REG_INFO1_SOCID4_O 0x00001230
#define AM_REG_INFO1_SOCID4_ADDR 0x42003230
#define AM_REG_INFO1_SOCID5_O 0x00001234
#define AM_REG_INFO1_SOCID5_ADDR 0x42003234
#define AM_REG_INFO1_SOCID6_O 0x00001238
#define AM_REG_INFO1_SOCID6_ADDR 0x42003238
#define AM_REG_INFO1_SOCID7_O 0x0000123c
#define AM_REG_INFO1_SOCID7_ADDR 0x4200323c
#define AM_REG_INFO1_SBR_SDCERT_ADDR_O 0x00001250
#define AM_REG_INFO1_SBR_SDCERT_ADDR_ADDR 0x42003250
#define AM_REG_INFO1_SBR_IPT_ADDR_O 0x00001258
#define AM_REG_INFO1_SBR_IPT_ADDR_ADDR 0x42003258
#define AM_REG_INFO1_SBR_OPT_ADDR_O 0x0000125c
#define AM_REG_INFO1_SBR_OPT_ADDR_ADDR 0x4200325c
#define AM_REG_INFO1_TEMP_CAL_ATE_O 0x00001300
#define AM_REG_INFO1_TEMP_CAL_ATE_ADDR 0x42003300
#define AM_REG_INFO1_TEMP_CAL_MEASURED_O 0x00001304
#define AM_REG_INFO1_TEMP_CAL_MEASURED_ADDR 0x42003304
#define AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_O 0x00001308
#define AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_ADDR 0x42003308
#define AM_REG_INFO1_TRIM_REV_O 0x00001310
#define AM_REG_INFO1_TRIM_REV_ADDR 0x42003310
#define AM_REG_INFO1_ADC_GAIN_ERR_O 0x00001328
#define AM_REG_INFO1_ADC_GAIN_ERR_ADDR 0x42003328
#define AM_REG_INFO1_ADC_OFFSET_ERR_O 0x0000132c
#define AM_REG_INFO1_ADC_OFFSET_ERR_ADDR 0x4200332c

// SBL_VERSION_0 - Contains the SBL Version number encoding.
#define AM_REG_INFO1_SBL_VERSION_0_VERSION_S 0
#define AM_REG_INFO1_SBL_VERSION_0_VERSION_M 0xFFFFFFFF
#define AM_REG_INFO1_SBL_VERSION_0_VERSION(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBL_VERSION_1 - Contains the SBL date code encoding.
#define AM_REG_INFO1_SBL_VERSION_1_DATECODE_S 0
#define AM_REG_INFO1_SBL_VERSION_1_DATECODE_M 0xFFFFFFFF
#define AM_REG_INFO1_SBL_VERSION_1_DATECODE(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBR_VERSION_0 - Contains the SBR version number and date code.
#define AM_REG_INFO1_SBR_VERSION_0_VERSION_S 0
#define AM_REG_INFO1_SBR_VERSION_0_VERSION_M 0xFFFFFFFF
#define AM_REG_INFO1_SBR_VERSION_0_VERSION(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// MAINPTR - The location of the vector table of the main program. This value is read-only.
#define AM_REG_INFO1_MAINPTR_ADDRESS_S 0
#define AM_REG_INFO1_MAINPTR_ADDRESS_M 0xFFFFFFFF
#define AM_REG_INFO1_MAINPTR_ADDRESS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// RESETSTATUS - The actual hardware reset status is maintained at this location by the Secure Boot Loader. This value is read only.
#define AM_REG_INFO1_RESETSTATUS_STATUS_S 0
#define AM_REG_INFO1_RESETSTATUS_STATUS_M 0xFFFFFFFF
#define AM_REG_INFO1_RESETSTATUS_STATUS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBLOTA - This is the address of the slot to be used by the Secure Boot Loader for staging an upgrade.
#define AM_REG_INFO1_SBLOTA_ADDRESS_S 0
#define AM_REG_INFO1_SBLOTA_ADDRESS_M 0xFFFFFFFF
#define AM_REG_INFO1_SBLOTA_ADDRESS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID0 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID0_ID_S 0
#define AM_REG_INFO1_SOCID0_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID0_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID1 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID1_ID_S 0
#define AM_REG_INFO1_SOCID1_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID1_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID2 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID2_ID_S 0
#define AM_REG_INFO1_SOCID2_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID2_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID3 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID3_ID_S 0
#define AM_REG_INFO1_SOCID3_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID3_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID4 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID4_ID_S 0
#define AM_REG_INFO1_SOCID4_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID4_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID5 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID5_ID_S 0
#define AM_REG_INFO1_SOCID5_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID5_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID6 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID6_ID_S 0
#define AM_REG_INFO1_SOCID6_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID6_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SOCID7 - SoC_ID is a statistically unique identification for the device required for the creation of debug certificates.
#define AM_REG_INFO1_SOCID7_ID_S 0
#define AM_REG_INFO1_SOCID7_ID_M 0xFFFFFFFF
#define AM_REG_INFO1_SOCID7_ID(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBR_SDCERT_ADDR - A pointer to the SD certificate.
#define AM_REG_INFO1_SBR_SDCERT_ADDR_ICV_S 0
#define AM_REG_INFO1_SBR_SDCERT_ADDR_ICV_M 0xFFFFFFFF
#define AM_REG_INFO1_SBR_SDCERT_ADDR_ICV(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBR_IPT_ADDR - A pointer to the SBR IPT.
#define AM_REG_INFO1_SBR_IPT_ADDR_ADDRESS_S 0
#define AM_REG_INFO1_SBR_IPT_ADDR_ADDRESS_M 0xFFFFFFFF
#define AM_REG_INFO1_SBR_IPT_ADDR_ADDRESS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// SBR_OPT_ADDR - A pointer to the SBR OPT.
#define AM_REG_INFO1_SBR_OPT_ADDR_ADDRESS_S 0
#define AM_REG_INFO1_SBR_OPT_ADDR_ADDRESS_M 0xFFFFFFFF
#define AM_REG_INFO1_SBR_OPT_ADDR_ADDRESS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// TEMP_CAL_ATE - The temperature measured on the ATE test head when the part's temperature sensor was calibrated.
#define AM_REG_INFO1_TEMP_CAL_ATE_TEMPERATURE_S 0
#define AM_REG_INFO1_TEMP_CAL_ATE_TEMPERATURE_M 0xFFFFFFFF
#define AM_REG_INFO1_TEMP_CAL_ATE_TEMPERATURE(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// TEMP_CAL_MEASURED - The voltage measured on the analog test mux output when the part's temperature sensor was calibrated.
#define AM_REG_INFO1_TEMP_CAL_MEASURED_VOLTS_S 0
#define AM_REG_INFO1_TEMP_CAL_MEASURED_VOLTS_M 0xFFFFFFFF
#define AM_REG_INFO1_TEMP_CAL_MEASURED_VOLTS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// TEMP_CAL_ADC_OFFSET - The offset voltage measured on for the ADC when the part's temperature sensor was calibrated.
#define AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_VOLTS_S 0
#define AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_VOLTS_M 0xFFFFFFFF
#define AM_REG_INFO1_TEMP_CAL_ADC_OFFSET_VOLTS(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// TRIM_REV - Contains the trim revision number.
#define AM_REG_INFO1_TRIM_REV_REVNUM_S 0
#define AM_REG_INFO1_TRIM_REV_REVNUM_M 0xFFFFFFFF
#define AM_REG_INFO1_TRIM_REV_REVNUM(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// ADC_GAIN_ERR - This float value is the ADC gain error used for correcting the error from ADC measurement.
#define AM_REG_INFO1_ADC_GAIN_ERR_ERROR_S 0
#define AM_REG_INFO1_ADC_GAIN_ERR_ERROR_M 0xFFFFFFFF
#define AM_REG_INFO1_ADC_GAIN_ERR_ERROR(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

// ADC_OFFSET_ERR - This float value is the ADC offset (volts).
#define AM_REG_INFO1_ADC_OFFSET_ERR_OFFSET_S 0
#define AM_REG_INFO1_ADC_OFFSET_ERR_OFFSET_M 0xFFFFFFFF
#define AM_REG_INFO1_ADC_OFFSET_ERR_OFFSET(n) (((uint32_t)(n) << 0) & 0xFFFFFFFF)

#endif
