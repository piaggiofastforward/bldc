/**
 *  Utilities intended for executables called from the command line.
 */

#ifndef VESC_COMMAND_LINE_H_
#define VESC_COMMAND_LINE_H_


namespace vesc_command_line
{

/**
 *  Validate command line arguments, connect to the VESC.
 */
int init(int sys_argc, const char** sys_argv);

/**
 *  Send the configuration file at "filename" to the VESC. The max response wait time is defined in
 *  vesc_config.h
 */
int configureWithXml(const char* filename);

/**
 *  Ask the VESC to run hall FOC detection routine. The max response wait time is defined in
 *  vesc_config.h
 */
int runFocHallDetection(const char* filename);


} // vesc_command_line


#endif
