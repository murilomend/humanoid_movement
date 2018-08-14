/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include "dynamixel_workbench_toolbox/dynamixel_multi_driver.h"

using namespace dynamixel_multi_driver;

DynamixelMultiDriver::DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version)
  :dynamixel_driver::DynamixelDriver(device_name, baud_rate, protocol_version)
{
  portHandler_   = dynamixel_driver::DynamixelDriver::portHandler_;
  packetHandler_ = dynamixel_driver::DynamixelDriver::packetHandler_;
}

DynamixelMultiDriver::~DynamixelMultiDriver()
{

}

bool DynamixelMultiDriver::loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info)
{
  uint8_t error = 0;

  for (std::vector<dynamixel_driver::DynamixelInfo>::size_type num = 0; num < dynamixel_info.size(); ++num)
  {
    if (packetHandler_->ping(portHandler_, dynamixel_info[num]->model_id, &dynamixel_info[num]->model_number, &error) == COMM_SUCCESS)
    {
      dynamixel_tool::DynamixelTool *dynamixel = new dynamixel_tool::DynamixelTool(dynamixel_info[num]->model_id, dynamixel_info[num]->model_number);
      multi_dynamixel_.push_back(dynamixel);

      dynamixel_info[num]->model_name = dynamixel->model_name_;
    }
    else
    {
      return false;
    }
  }

  return true;
}

dynamixel::GroupSyncWrite* DynamixelMultiDriver::setSyncWrite(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncWrite * groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncWrite;
}

dynamixel::GroupSyncRead* DynamixelMultiDriver::setSyncRead(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncRead* groupSyncRead = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncRead;
}

bool DynamixelMultiDriver::initSyncWrite()
{
	float protocol_version = getProtocolVersion();
    if(protocol_version == 1.0)
	{
        groupSyncWritePosition_ = setSyncWrite("pos_vel");
	}
	if(protocol_version == 2.0)
	{
		groupSyncWriteProfileAcceleration_ = setSyncWrite("pos_vel_acc");
		groupSyncWriteProfileVelocity_     = setSyncWrite("pos_vel");
		groupSyncWriteOperatingMode_       = setSyncWrite("operating_mode");
        groupSyncWriteKpPositionGain_      = setSyncWrite("position_p_gain");
    }

    groupSyncWriteTorque_   = setSyncWrite("torque_enable");
	groupSyncWriteReturnLevel_         = setSyncWrite("status_return_level");
	groupSyncWriteShutdown_            = setSyncWrite("shutdown");
	return true;
}

bool DynamixelMultiDriver::initSyncRead()
{
  groupSyncReadPosition_      = setSyncRead("present_position");
  groupSyncReadCurrent_       = setSyncRead("present_current");
  groupSyncReadKdPositionGain_= setSyncRead("position_d_gain");
  

  return true;
}

bool DynamixelMultiDriver::readMultiRegister(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;
  int32_t value;

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dynamixel_= multi_dynamixel_[num];

    if (readRegister(addr_name, &value))
      read_data->push_back(value);
    else
      return false;
  }

  read_value_[addr_name] = read_data;

  return true;
}

bool DynamixelMultiDriver::syncWritePosition(std::vector<uint32_t> pos)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos[num]));

    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePosition_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteVelocity(std::vector<int32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteMovingSpeed(std::vector<uint16_t> spd)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_speed[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_speed[0] = DXL_LOBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[1] = DXL_HIBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[2] = DXL_LOBYTE(DXL_HIWORD(spd[num]));
    param_goal_speed[3] = DXL_HIBYTE(DXL_HIWORD(spd[num]));

    dynamixel_addparam_result_ = groupSyncWriteMovingSpeed_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_speed);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMovingSpeed_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMovingSpeed_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteCurrent(std::vector<int16_t> cur)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_current[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD(cur[num]));
    param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD(cur[num]));

    dynamixel_addparam_result_ = groupSyncWriteCurrent_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_current);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCurrent_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCurrent_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteTorque(std::vector<uint8_t> &onoff)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_torque[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff[num]));
    param_goal_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff[num]));

    dynamixel_addparam_result_ = groupSyncWriteTorque_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_torque);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteTorque_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteTorque_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteProfileVelocity(std::vector<uint32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileVelocity_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncWriteMaxMinPosition(std::vector<uint32_t> maxPos,std::vector<uint32_t> minPos)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_maxmin_position[8];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_maxmin_position[0] = DXL_LOBYTE(DXL_LOWORD(maxPos[num]));
    param_maxmin_position[1] = DXL_HIBYTE(DXL_LOWORD(maxPos[num]));
    param_maxmin_position[2] = DXL_LOBYTE(DXL_HIWORD(maxPos[num]));
    param_maxmin_position[3] = DXL_HIBYTE(DXL_HIWORD(maxPos[num]));

    param_maxmin_position[4] = DXL_LOBYTE(DXL_LOWORD(minPos[num]));
    param_maxmin_position[5] = DXL_HIBYTE(DXL_LOWORD(minPos[num]));
    param_maxmin_position[6] = DXL_LOBYTE(DXL_HIWORD(minPos[num]));
    param_maxmin_position[7] = DXL_HIBYTE(DXL_HIWORD(minPos[num]));

    dynamixel_addparam_result_ = groupSyncWriteMaxMinPosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_maxmin_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMaxMinPosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMaxMinPosition_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncWriteKpPositionGain(std::vector<uint16_t> kp)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_kp[2];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_kp[0] = DXL_LOBYTE(kp[num]);
    param_goal_kp[1] = DXL_HIBYTE(kp[num]);

    dynamixel_addparam_result_ = groupSyncWriteKpPositionGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_kp);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteKpPositionGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteKpPositionGain_->clearParam();
  return true;
}

/*bool DynamixelMultiDriver::syncWritePosVelAcc(std::vector<uint32_t> pos, std::vector<uint32_t> vel,std::vector<uint32_t> acc)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_acceleration[4];
  uint8_t param_goal_profile_velocity[4];
  uint8_t param_goal_profile_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
	param_goal_profile_acceleration[0] = DXL_LOBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[1] = DXL_HIBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[2] = DXL_LOBYTE(DXL_HIWORD(acc[num]));
    param_goal_profile_acceleration[3] = DXL_HIBYTE(DXL_HIWORD(acc[num]));  
	  
    param_goal_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));
    
    param_goal_profile_position[0] = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal_profile_position[1] = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal_profile_position[2] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal_profile_position[3] = DXL_HIBYTE(DXL_HIWORD(pos[num])); 

    dynamixel_addparam_result_ = groupSyncWriteProfileAcceleration_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_acceleration);
    dynamixel_addparam_result_ = groupSyncWriteProfileVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_velocity);
    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileAcceleration_->txPacket();
  dynamixel_comm_result_ = groupSyncWriteProfileVelocity_->txPacket();
  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePosition_->clearParam();
  groupSyncWriteProfileVelocity_->clearParam();
  groupSyncWriteProfileAcceleration_->clearParam();
  return true;
}*/

bool DynamixelMultiDriver::syncWritePosVelAcc(std::vector<uint32_t> pos, std::vector<uint32_t> vel,std::vector<uint32_t> acc)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal[12];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
	param_goal[0]  = DXL_LOBYTE(DXL_LOWORD(acc[num]));
    param_goal[1]  = DXL_HIBYTE(DXL_LOWORD(acc[num]));
    param_goal[2]  = DXL_LOBYTE(DXL_HIWORD(acc[num]));
    param_goal[3]  = DXL_HIBYTE(DXL_HIWORD(acc[num]));  
    
    param_goal[4]  = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal[5]  = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal[6]  = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal[7]  = DXL_HIBYTE(DXL_HIWORD(vel[num]));  
	  
	param_goal[8]  = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal[9]  = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal[10] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal[11] = DXL_HIBYTE(DXL_HIWORD(pos[num]));  


    dynamixel_addparam_result_ = groupSyncWriteProfileAcceleration_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal);
    std::cout << "ID: " << int(multi_dynamixel_[num]->id_) << "   ADD_PARAM_RESULT: " << dynamixel_addparam_result_ << std::endl;
    std::cout << "POS:" << pos[num] << " VEL: " << vel[num] << " ACC: " << acc[num] << std::endl;
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  //std::cout << "TX_PACKET " << std::endl;
  dynamixel_comm_result_ = groupSyncWriteProfileAcceleration_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  //std::cout << "TX_PACKET SENT " << std::endl;
  groupSyncWriteProfileAcceleration_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncWritePosVel1(std::vector<uint32_t> pos, std::vector<uint32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal[8];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal[0]  = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal[1]  = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal[2]  = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal[3]  = DXL_HIBYTE(DXL_LOWORD(vel[num]));  
    
    
	
    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal);
    //std::cout << "ID: " << int(multi_dynamixel_[num]->id_) << "   ADD_PARAM_RESULT: " << dynamixel_addparam_result_ << std::endl;
    //std::cout << "POS:" << pos[num] << " VEL: " << vel[num]  << std::endl;
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  //std::cout << "TX_PACKET " << std::endl;
  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  //std::cout << "TX_PACKET SENT " << std::endl;
  groupSyncWritePosition_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncWritePosVel2(std::vector<uint32_t> pos, std::vector<uint32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal[8];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal[0]  = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal[1]  = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal[2]  = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal[3]  = DXL_HIBYTE(DXL_HIWORD(vel[num]));  
	  
	param_goal[4]  = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal[5]  = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal[6] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal[7] = DXL_HIBYTE(DXL_HIWORD(pos[num]));  


    dynamixel_addparam_result_ = groupSyncWriteProfileVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal);
    //std::cout << "ID: " << int(multi_dynamixel_[num]->id_) << "   ADD_PARAM_RESULT: " << dynamixel_addparam_result_ << std::endl;
    //std::cout << "POS:" << pos[num] << " VEL: " << vel[num]  << std::endl;
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  //std::cout << "TX_PACKET " << std::endl;
  dynamixel_comm_result_ = groupSyncWriteProfileVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  //std::cout << "TX_PACKET SENT " << std::endl;
  groupSyncWriteProfileVelocity_->clearParam();
  return true;
}

 bool DynamixelMultiDriver::syncWriteProfileAcceleration(std::vector<uint32_t> acc)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_acceleration[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_acceleration[0] = DXL_LOBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[1] = DXL_HIBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[2] = DXL_LOBYTE(DXL_HIWORD(acc[num]));
    param_goal_profile_acceleration[3] = DXL_HIBYTE(DXL_HIWORD(acc[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileAcceleration_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_acceleration);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileAcceleration_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileAcceleration_->clearParam();
  return true;
}



bool DynamixelMultiDriver::syncWriteReturnLevel(std::vector<uint8_t> level)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_status_level[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_status_level[0] = DXL_LOBYTE(DXL_LOWORD(level[num]));
    param_status_level[1] = DXL_HIBYTE(DXL_LOWORD(level[num]));
    param_status_level[2] = DXL_LOBYTE(DXL_HIWORD(level[num]));
    param_status_level[3] = DXL_HIBYTE(DXL_HIWORD(level[num]));

    dynamixel_addparam_result_ = groupSyncWriteReturnLevel_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_status_level);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteReturnLevel_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteReturnLevel_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteOperatingMode(std::vector<uint8_t> mode)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_oper_mode[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_oper_mode[0] = DXL_LOBYTE(DXL_LOWORD(mode[num]));
    param_oper_mode[1] = DXL_HIBYTE(DXL_LOWORD(mode[num]));
    param_oper_mode[2] = DXL_LOBYTE(DXL_HIWORD(mode[num]));
    param_oper_mode[3] = DXL_HIBYTE(DXL_HIWORD(mode[num]));

    dynamixel_addparam_result_ = groupSyncWriteOperatingMode_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_oper_mode);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteOperatingMode_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteOperatingMode_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncWriteShutdown(std::vector<uint8_t> mode)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_shutdown[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_shutdown[0] = DXL_LOBYTE(DXL_LOWORD(mode[num]));
    param_shutdown[1] = DXL_HIBYTE(DXL_LOWORD(mode[num]));
    param_shutdown[2] = DXL_LOBYTE(DXL_HIWORD(mode[num]));
    param_shutdown[3] = DXL_HIBYTE(DXL_HIWORD(mode[num]));

    dynamixel_addparam_result_ = groupSyncWriteShutdown_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_shutdown);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteShutdown_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteShutdown_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncReadPosition(std::vector<uint32_t> &pos)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t position;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_position"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  pos.clear();

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadPosition_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      position  = groupSyncReadPosition_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      pos.push_back(position);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadPosition_->clearParam();

  return true;
}



bool DynamixelMultiDriver::syncReadCurrent(std::vector<uint32_t> &curr)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t current;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_current"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  curr.clear();

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadCurrent_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadCurrent_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadCurrent_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
    if (dxl_getdata_result)
    {
      current  = groupSyncReadCurrent_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      curr.push_back(current);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadCurrent_->clearParam();

  return true;
}
