/*******************************************************************************************/
/*                                                                                         */
/* void ucsubCallback(const kinoped_msgs::user_command::ConstPtr& msg)                          */
/*                                                                                         */
/* Callback for handling ROS user_command messages.                                        */
/*                                                                                         */
/*******************************************************************************************/
void ucsubCallback(const kinoped_msgs::user_command::ConstPtr& msg)
{
  if (!strcmp(ROS_COMMAND_DEBUG_MODE, msg->command.c_str()))
  {
     if (!strcmp(ROS_SUB_COMMAND_DEBUG_MODE_ON, msg->sub_command.c_str()))
     {
         debug_mode = KINOPED_DEBUG_MODE_ON;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_DEBUG_ON_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_DEBUG_MODE_OFF, msg->sub_command.c_str()))
     {
         debug_mode = KINOPED_DEBUG_MODE_OFF;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_DEBUG_OFF_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else
     {
         ROS_INFO("Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_ERROR, messageDesc);
     }
  }
  else if (!strcmp(ROS_COMMAND_SESSION, msg->command.c_str()))
  {
     if (!strcmp(ROS_SUB_COMMAND_PAUSE, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_PAUSE;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_PAUSE_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_RUN, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_RUN;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_RUN_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_START, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_START;
         strncpy(dbName, msg->option.c_str(), sizeof(dbName) - 1);
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         startReceived = 1;
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_START_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_STOP, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_STOP;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         keepRunning = 0;
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_STOP_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_END, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_END;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         keepRunning = 0;
         done = 1;
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_RESTART, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_RESTART;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_RESTART_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_REBOOT, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_REBOOT;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_REBOOT_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else if (!strcmp(ROS_SUB_COMMAND_SHUTDOWN, msg->sub_command.c_str()))
     {
         run_mode = KINOPED_RUN_MODE_SHUTDOWN;
         ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_SHUTDOWN_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
     }
     else
     {
         ROS_INFO("Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         sprintf(messageDesc, "Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
         printf("%s\n", messageDesc);
         publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_ERROR, messageDesc);
     }
  }
  else if (!strcmp(ROS_COMMAND_GOTO, msg->command.c_str()))
  {
      if (!strcmp(ROS_SUB_COMMAND_HOME_POSITION, msg->sub_command.c_str()))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GOTO_HOME_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
          gotoHomeReceived = 1;
      }
      else if (!strcmp(ROS_SUB_COMMAND_LOAD_POSITION, msg->sub_command.c_str()))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GOTO_LOAD_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
          gotoLoadReceived = 1;
          gotoLoadOneShot = 1;
      }
      else if (!strcmp(ROS_SUB_COMMAND_SHUTDOWN, msg->sub_command.c_str()))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_SHUTDOWN_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
          shutdownReceived = 1;
      }
      else
      {
          ROS_INFO("Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          sprintf(messageDesc, "Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_ERROR, messageDesc);
      }
  }
  else if (!strcmp(ROS_COMMAND_ECHO, msg->command.c_str()))
  {
      if (!strcmp(ROS_SUB_COMMAND_ECHO_PLC, msg->sub_command.c_str()) && (!strcmp(thisNodeType, ROS_NODE_TYPE_PLC)))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]  option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]  option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_ECHO_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, msg->option.c_str());
      }
      else if (!strcmp(ROS_SUB_COMMAND_ECHO_XSENS, msg->sub_command.c_str()) && (!strcmp(thisNodeType, ROS_NODE_TYPE_XSENS)))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]  option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s]  option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_ECHO_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, msg->option.c_str());
      }
      else if (!strcmp(ROS_SUB_COMMAND_ECHO_WEBUI, msg->sub_command.c_str()) && (!strcmp(thisNodeType, ROS_NODE_TYPE_WEBUI)))
      {
          ROS_INFO("Parsed command[%s]  sub_command[%s]  option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          sprintf(messageDesc, "Parsed command[%s]  sub_command[%s] option[%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
          printf("%s\n", messageDesc);
          publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_ECHO_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, msg->option.c_str());
      }
      else
      {
          // do nothing, since this message wasn't directed to us
          // don't respond to an echo that wasn't directed to us
      }
  }
  else
  {
      ROS_INFO("Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
      sprintf(messageDesc, "Error Parsing command[%s]  sub_command[%s]", msg->command.c_str(), msg->sub_command.c_str());
      printf("%s\n", messageDesc);
      publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_ERROR, messageDesc);
  }
}
