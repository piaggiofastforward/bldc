
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#include "vesc_driver/control_msgs.h"
#include "vesc_driver/vesc_config.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/console.h"
#include "vesc_driver/rapidxml.hpp"
#include "vesc_driver/rapidxml_print.hpp"

namespace rx = rapidxml;

namespace vesc_config
{

/**
 *  These don't really belong in enum mc_config_param, since these values are supposed to be sent in a uint8_t[8] array in
 *  a single transaction. Instead, all of the following keys are mapped to HALL_TABLE or FOC_HALL_TABLE enum values.
 *
 *  Place the value from the XML document into this array depending on the index at the end of the string key.
 */
static std::map<std::string, uint8_t> hall_param_to_index = {
  {"hall_table_0", 0},
  {"hall_table_1", 1},
  {"hall_table_2", 2},
  {"hall_table_3", 3},
  {"hall_table_4", 4},
  {"hall_table_5", 5},
  {"hall_table_6", 6},
  {"hall_table_7", 7},
  {"foc_hall_table_0", 0},
  {"foc_hall_table_1", 1},
  {"foc_hall_table_2", 2},
  {"foc_hall_table_3", 3},
  {"foc_hall_table_4", 4},
  {"foc_hall_table_5", 5},
  {"foc_hall_table_6", 6},
  {"foc_hall_table_7", 7},
};

static bool isHallTableParam(std::string key)
{
  return key.find("hall_table") != std::string::npos;
}

static bool isFocHallTableParam(std::string key)
{
  return key.find("foc_hall_table") != std::string::npos;
}

static int sendConfig(mc_config_union config, xmlSendFunc sendFunc)
{
  uint8_t buf[sizeof(mc_config) + 1];
  buf[0] = CONFIG_WRITE;
  memcpy(buf + 1, config.config_bytes, sizeof(mc_config));
  sendFunc(buf, sizeof(buf));
  return 0;
}

static bool handleHallParam(std::string key, std::string value, mc_config_hall& hall, mc_config_hall& hall_foc)
{
  const auto hall_item = hall_param_to_index.find(key);
  if (hall_item == hall_param_to_index.end())
  {
    ROS_ERROR_STREAM("Couldnt find an index for param " << key);
    return false;
  }

  uint8_t index = hall_item->second;
  if (isFocHallTableParam(key))
    hall_foc.hall_foc_values[index] = atoi(value.c_str());
  else
    hall.hall_values[index] = atoi(value.c_str());

  return true;
}

static void sendHallValues(mc_config_hall_union hall, xmlSendFunc sendFunc)
{
  uint8_t buf[sizeof(mc_config_hall) + 1];
  buf[0] = CONFIG_WRITE_HALL;
  memcpy(buf + 1, hall.config_bytes, sizeof(mc_config_hall));
  sendFunc(buf, sizeof(buf));
}

/**
 *  Depending on the param type, set different fields of the union within mc_config.
 *
 */
static void handleParam(enum mc_config_param param, std::string value, xmlSendFunc sendFunc)
{
  mc_config_union config;
  config.config.param = param;
  char msg[150];
  switch(param)
  {
    // this case includes bools and uint8_t - anything that's a single byte
    case PWM_MODE:
    case COMM_MODE:
    case MOTOR_TYPE:
    case SENSOR_MODE:
    case L_SLOW_ABS_CURRENT:
    case ENCODER_INVERTED:
    case FOC_SAMPLE_V0_V7:
    case FOC_SAMPLE_HIGH_CURRENT:
    case FOC_TEMP_COMP:
    case FOC_SENSOR_MODE:
    case PID_ALLOW_BRAKING:
    case M_SENSOR_PORT_MODE:
    case M_DRV8301_OC_MODE:
    case M_INVERT_DIRECTION:
      config.config.value_byte = atoi(value.c_str());
      sprintf(msg, "------>value: %d", config.config.value_byte);
      break;

    // 32-bit signed fields
    case M_FAULT_STOP_TIME_MS:
    case M_DRV8301_OC_ADJ:
      config.config.value_i = atoi(value.c_str());
      sprintf(msg, "------>value: %d", config.config.value_i);
      break;

    // 32-bit unsigned fields
    case M_ENCODER_COUNTS:
      config.config.value_ui = atoi(value.c_str());
      sprintf(msg, "------>value: %u", config.config.value_ui);
      break;

    // default - the rest are floats
    default:
      config.config.value_f = atof(value.c_str());
      sprintf(msg, "------>value: %f", config.config.value_f);
      break;
  }
  ROS_WARN(msg);
  sendConfig(config, sendFunc);
}

static int getXmlRoot(const char* filename, rx::xml_document<> &doc, rx::xml_node<> *&root_node)
{
  std::ifstream xml_file(filename);
  if (!xml_file.good())
  {
    return -1;
  }

  // // read xml file into a vector
  std::vector<char> buf((std::istreambuf_iterator<char>(xml_file)), std::istreambuf_iterator<char>());
  buf.push_back('\0');

  // // Parse the buffer using the xml file parsing library into doc 
  doc.parse<rx::parse_no_data_nodes>(&buf[0]);

  // // find the root node - this tag contains all key/value configuration pairs within
  root_node = doc.first_node("MCConfiguration");

  if(!root_node)
  {
    return -1;
  }
  return 0;
}


int readAndSendXml(const char* filename, xmlSendFunc sendFunc)
{
  rx::xml_document<> doc;
  rx::xml_node<> *root_node;
  std::ifstream xml_file(filename);
  if (getXmlRoot(filename, doc, root_node) != 0)
  {
    ROS_ERROR_STREAM("Problem parsing VESC XML doc [" << filename << "]. Make sure it exists and contains an "
      "MCConfiguration tag.");
    return -1;
  }

  /**
   *  When iterating, if we hit a hall_table or hall_table_foc key, index the hall_param_to_index
   *  map, and place the corresponding value at the correct index. Send HALL_TABLE and FOC_HALL_TABLE
   *  values each in their own transaction, sending the entire table at once.
   */
  mc_config_hall_union hall, hall_foc;
  hall.config.param = HALL_TABLE;
  hall_foc.config.param = HALL_TABLE_FOC;
  bool hall_table_success = true;

  std::string name, value;

  // iterate over all tags within
  for (rx::xml_node<> * config_node = root_node->first_node(); config_node; config_node = config_node->next_sibling())
  {
    name = config_node->name();
    value = config_node->value();
    const auto item = xml_name_to_enum.find(name);
    if (item == xml_name_to_enum.end())
    {
      // if we couldnt find the tag, and it shouldnt be ignored, log an error
      if (ignored_xml_tags.count(name) == 0)
        ROS_ERROR_STREAM("Not prepared to handle config param " << name << "!!!");
      continue;
    }
    ROS_WARN_STREAM(name << " -> " << value);

    switch (item->second)
    {
      case HALL_TABLE:
      case HALL_TABLE_FOC:
        if (!handleHallParam(name, value, hall.config, hall_foc.config))
          hall_table_success = false;
        break;
      default:

        // send key/value pairs to VESC, sleeping for 2 milliseconds between each
        handleParam(item->second, value, sendFunc);
        break;
    }
  }

  if (hall_table_success)
  {
    sendHallValues(hall, sendFunc);
    sendHallValues(hall_foc, sendFunc);
  }
  else
  {
    ROS_ERROR("Error parsing hall table values!!!!");
  }

  ROS_WARN("FOC hall table values: ");
  for (int i = 0; i < 8; ++i)
  {
    ROS_WARN_STREAM("--> " << (int)hall_foc.config.hall_foc_values[i]);
  }
  ROS_WARN("Hall table values: ");
  for (int i = 0; i < 8; ++i)
  {
    ROS_WARN_STREAM("--> " << (int)hall.config.hall_values[i]);
  }

  return 0;
}


int runHallFocDetection(const char* filename, focDetectFunc detectFunc)
{
  // make sure destination config file exists before running routine
  rx::xml_document<> doc;
  rx::xml_node<> *root_node;
  std::ifstream xml_file(filename);
  if (getXmlRoot(filename, doc, root_node) != 0)
  {
    printf("Problem parsing VESC XML doc [ %s ]. Make sure it exists and contains an "
      "MCConfiguration tag.\n", filename);
    return -1;
  }
  xml_file.close();

  // if the document exists, run the detection routine and update the document
  // wait for the response - max 30 seconds.
  uint8_t hall_detect_result[RESPONSE_DETECT_HALL_FOC_SIZE];
  if (detectFunc(FOC_DETECT_RESPONSE_TIMEOUT, hall_detect_result) != 0)
  {
    printf("Did not receive a response from VESC hall FOC detection routine in "
      "under %d seconds!\n", FOC_DETECT_RESPONSE_TIMEOUT);
    return -1;
  }

  // last byte indicates success/failure
  if (hall_detect_result[RESPONSE_DETECT_HALL_FOC_SIZE - 1] == 0)
  {
    printf("VESC indicated that the FOC detection routine failed.\n");
    return -1;
  }

  printf("Hall detection results...\n");
  for (int i = 0; i < RESPONSE_DETECT_HALL_FOC_SIZE - 1; ++i)
    printf("foc_hall_table_%d -> %d\n", i, hall_detect_result[i]);


  /**
   *  Update the XML document with the obtained values for the FOC hall detect routine.
   *  The memory pool is used here since rapidxml only stores pointers to strings that we 
   *  pass in when updating values - if we allocate a string on the stack in the loop below, that 
   *  memory will be freed before the XML document is saved and the data becomes invalid garbage.
   *
   *  The memory pool's destructor will ensure that all of the allocated strings are freed. This
   *  is a lil bit nicer than using malloc()/free() ourselves on an array of strings.
   */
  rx::memory_pool<char> memPool;  // allocated memory is freed in destructor
  char* allocated_val;
  std::string name, value;
  int hall_value;
  printf("Updating [ %s ] with FOC hall table values...\n", filename);
  for (rx::xml_node<> * config_node = root_node->first_node(); config_node; config_node = config_node->next_sibling())
  {
    name = config_node->name();
    value = config_node->value();

    if (!isFocHallTableParam(name)) continue;

    const auto item = hall_param_to_index.find(name);
    hall_value = hall_detect_result[item->second];
    allocated_val = memPool.allocate_string("", 7);
    sprintf(allocated_val, "%d", hall_value);
    config_node->value(allocated_val);
  }

  // save it
  std::ofstream xml_file_out(filename, std::ios::out | std::ios::trunc);
  xml_file_out << doc;
  xml_file_out.close();
  return 0;
}

} //vesc_config

