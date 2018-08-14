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

#include "dynamixel_workbench_single_manager/single_dynamixel_controller.h"

using namespace single_dynamixel_controller;

SingleDynamixelController::SingleDynamixelController()
{
  // init Service Client
  dynamixel_info_client_    = node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("dynamixel/info");
  dynamixel_command_client_ = node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel/command");
}

SingleDynamixelController::~SingleDynamixelController()
{

}

bool SingleDynamixelController::initSingleDynamixelController()
{

}

bool SingleDynamixelController::shutdownSingleDynamixelController(void)
{
  ros::shutdown();
  return true;
}

int SingleDynamixelController::getch()
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int SingleDynamixelController::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

void SingleDynamixelController::viewManagerMenu()
{
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Single Manager supports GUI (dynamixel_workbench_single_manager_gui)  ");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Command list :");
  ROS_INFO("[help|h|?]...........: help");
  ROS_INFO("[info]...............: information of a Dynamixel");
  ROS_INFO("[table]..............: check a control table of a Dynamixel");
  ROS_INFO("[torque_enable]......: torque on Dynamixel");
  ROS_INFO("[torque_disable].....: torque off Dynamixel");
  ROS_INFO("[reboot].............: reboot a Dynamixel(only protocol version 2.0)");
  ROS_INFO("[factory_reset]......: command for all data back to factory settings values");
  ROS_INFO("[[table_item] [value]: change address value of a Dynamixel");
  ROS_INFO("[exit]...............: shutdown");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Press Enter Key To Command A Dynamixel");
}

bool SingleDynamixelController::sendCommandMsg(std::string cmd, std::string addr, int64_t value)
{
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  set_dynamixel_command.request.command   = cmd;
  set_dynamixel_command.request.addr_name = addr;
  set_dynamixel_command.request.value     = value;

  if (dynamixel_command_client_.call(set_dynamixel_command))
  {
    if (!set_dynamixel_command.response.comm_result)
      return false;
    else
      return true;
  }
}

bool SingleDynamixelController::controlLoop()
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  if (kbhit())
  {
    if (getchar() == ENTER_ASCII_VALUE)
    {
      viewManagerMenu();
      printf("[ CMD ]");
      fgets(input, sizeof(input), stdin);

      char *p;
      if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
      fflush(stdin);

      if (strlen(input) == 0) return false;

      token = strtok(input, " ");

      if (token == 0) return false;

      strcpy(cmd, token);
      token = strtok(0, " ");
      num_param = 0;

      while (token != 0)
      {
        strcpy(param[num_param++], token);
        token = strtok(0, " ");
      }

      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
      {
        viewManagerMenu();
      }
      else if (strcmp(cmd, "info") == 0)
      {
        dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;

        if (dynamixel_info_client_.call(get_dynamixel_info))
        {
          ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %ld", get_dynamixel_info.response.dynamixel_info.model_id,
                                                                get_dynamixel_info.response.dynamixel_info.model_name.c_str(),
                                                                get_dynamixel_info.response.dynamixel_info.load_info.baud_rate);
        }
      }
      else if (strcmp(cmd, "exit") == 0)
      {
        if (sendCommandMsg("exit"))
          shutdownSingleDynamixelController();

        return true;
      }
      else if (strcmp(cmd, "table") == 0)
      {
        if (!sendCommandMsg("table"))
          ROS_ERROR("It didn't load DYNAMIXEL Control Table");
      }
      else if (strcmp(cmd, "reboot") == 0)
      {
        if (!sendCommandMsg("reboot"))
          ROS_ERROR("It didn't reboot to DYNAMIXEL");
      }
      else if (strcmp(cmd, "factory_reset") == 0)
      {
        if (!sendCommandMsg("factory_reset"))
          ROS_ERROR("It didn't factory reset to DYNAMIXEL");
      }
      else if (strcmp(cmd, "torque_enable") == 0)
      {
        if (!sendCommandMsg("addr", "torque_enable", 1))
          ROS_ERROR("It didn't works");
      }
      else if (strcmp(cmd, "torque_disable") == 0)
      {
        if (!sendCommandMsg("addr", "torque_enable", 0))
          ROS_ERROR("It didn't works");
      }
      else if (num_param == 1)
      {
        if (sendCommandMsg("addr", cmd, atoi(param[0])))
          ROS_INFO("It works!!");
        else
          ROS_ERROR("It didn't works!!");
      }
      else
      {
        ROS_ERROR("Invalid command. Please check menu[help, h, ?]");
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_controller");
  SingleDynamixelController single_dynamixel_controller;
  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    single_dynamixel_controller.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
