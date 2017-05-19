// -*- mode:c++; fill-column: 100; -*-

#include "sparkfun_9dof_razor_imu_m0/driver.h"

#include <boost/regex.hpp>

#include <cassert>
#include <sstream>

#include <ros/console.h>

namespace sparkfun_9dof_razor_imu_m0
{

Driver::Driver(ros::NodeHandle nh,
	       ros::NodeHandle private_nh) :
  serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
	  serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none),
  rx_thread_run_(true)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::cout << "Initializing" << std::endl;
  ROS_DEBUG("Initializing sparkfun_9dof_razor_imu_m0::Driver");
  
  // get razor imu serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    serial_.setPort(port);
    serial_.open();
  }
  catch (std::exception const& e) {
    ROS_FATAL("Failed to connect to the SparkFun 9DoF Razor IMU M0, %s.", e.what());
    ros::shutdown();
    return;
  }

  // start up a serial receive monitoring thread
  int result = pthread_create(&rx_thread_, NULL, &Driver::rxThreadHelper, this);
  assert(0 == result);
}

Driver::~Driver()
{
  std::cout << "Destructing" << std::endl;
  ROS_DEBUG("Destructing sparkfun_9dof_razor_imu_m0::Driver");

  if (serial_.isOpen()) {
    // bring down read thread
    rx_thread_run_ = false;
    int result = pthread_join(rx_thread_, NULL);
    assert(0 == result);
    std::cout << "Joined" << std::endl;
    
    serial_.close();
  }
}

  
void* Driver::rxThread(void)
{
  ROS_DEBUG("sparkfun_9dof_razor_imu_m0::Driver::rxThread starting.");

  // pause briefly to give serial port time to stabilize
  // not sure that this is necessary
  ros::Duration(0.1).sleep();

  configureImu();
  
  std::string buffer;
  while (rx_thread_run_) {
    buffer = serial_.readline();
    if (buffer.length() > 0) {
      ROS_DEBUG("%s", buffer.c_str());
    }
  }

  std::cout << "Thread terminating" << std::endl;
  ROS_DEBUG("sparkfun_9dof_razor_imu_m0::Driver::rxThread terminating.");
}

bool Driver::serialWriteVerify(std::string const& data)
{
  if (!rx_thread_run_)
    return false;

  size_t num_written = serial_.write(data);
  if (num_written != data.length()) {
    ROS_ERROR("Failed to write to SparkFun 9DoF Razor IMU M0 serial port. "
	      "Attempted to write %zd bytes, actually wrote %zd bytes.",
	      data.length(), num_written);
    return false;
  }

  return true;
}

bool Driver::sequentialCommand(std::string const& command, std::string const& response_format,
			       std::string const& response_desired_value, double response_timeout,
			       int max_sequential_commands)
{
  // @todo a full regex seems a little over the top, replace with a simple function
  boost::regex regex(response_format);
  assert(regex.mark_count() == 1);

  int command_count(0);
  while (command_count < max_sequential_commands) {

    // send a command to step to the next setting
    if (!serialWriteVerify(command))
      return false;
    command_count++;

    // check the response to see if we've reached the desired setting
    ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(response_timeout));
    while(ros::WallTime::now() < expire) {

      if (!rx_thread_run_)
	return false;
      std::string received = serial_.readline();
      
      boost::cmatch matches;
      if (boost::regex_match(received.c_str(), matches, regex)) {
	if (response_desired_value.compare(matches[1].first) == 0) {
	  // successfully set command to desired value
	  return true;
	}
      }
      // else: received line could have been a sensor reading, continue
      
    }
  }

  ROS_ERROR("Sent %d '%s' commands to the IMU but did not receive an expected response matching "
	    "format '%s' with value '%s'.", command_count, command.c_str(),
	    response_format.c_str(), response_desired_value.c_str());
  return false;
}

std::string itos(int i) {
  return static_cast< std::ostringstream & >(std::ostringstream() << std::dec << i).str();
}
  
bool Driver::configureImu()
{
  // Configure the SparkFun 9DoF Razor IMU M0 default firmware 

  // set output rate
  sequentialCommand("r", "IMU rate set to (\\d+) Hz", itos(50), 0.2, 12);
  
  // set output rate
  //   repeat:
  //     send 'r'
  //     readline until we see the confirmation (waiting no more than TBD seconds)
  //     until confirmation is requested rate
  // set acceleration FSR
  //   repeat
  //     send 'A'
  //     readline until we see the confirmation (waiting no more than TBD seconds)
  //     until confirmation is requested FSR
  // set gyro FSR
  //   similar as above
  // unpause
  //   readline for sensor msg (waiting 3 * 1/r)
  //   if no sensor message, send ' '
  //   readline for sensor msg (waiting 3 * 1/r)
  //   if no sensor message, error
  // turn on gyro
  //   get sensor message
  //   count number of values
  //   send 'g'
  //   readline until we see change in number of values, maximum of 3 messages?
  //   if values decreased, send 'g' again
  //   otherwise continue
  
}
  
} // namespace sparkfun_9dof_razor_imu_m0
