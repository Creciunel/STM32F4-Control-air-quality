/**
  ******************************************************************************
  * @file    state_model_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Aug 18 15:08:41 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "state_model_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_state_model_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_state_model_weights_array_u64[22] = {
  0x3e0f587c3eb98ec4U, 0xbec427e13e2cbc27U, 0x3e1bd8a5bf37246bU, 0x3ec2c18cbf01a9d5U,
  0x3ca3492bbe3fa58fU, 0x3e37dcc1be2e7365U, 0x3e5151793df45e80U, 0x3e631e783e461115U,
  0xbeff695fbeece253U, 0x3eacc326beaaf89cU, 0x3f735e16bf2e120eU, 0x3e120a04be1f8f83U,
  0xbd338a58bf200e5bU, 0xbd8d4a69bf157dccU, 0x3ed335b1bea20e14U, 0x3e6e952fbda075e3U,
  0xbf350f103efd32abU, 0x3ec5133f3e20cfcdU, 0xbec86faa3f438b59U, 0xbf1c0abb3e3ef162U,
  0xbcd4c3b73e14ac2bU, 0xbe44fbb1U,
};


ai_handle g_state_model_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_state_model_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

