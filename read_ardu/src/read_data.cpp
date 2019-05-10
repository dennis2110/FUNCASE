#include <ros/ros.h>
#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>


bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;

  double time_offset_in_seconds;

  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;


  ros::init(argc, argv, "read_data");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");

  ros::NodeHandle nh("imu");

  ros::Rate r(200); // 200 hz


  std::string input;
  std::string read;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if(ser.available())
        {
          read = ser.read(ser.available());
          ////////////////////////////////
          std::cout<< "read: " << ser.available()<<std::endl;
          ////////////////////////////////
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());

          input += read;
          ////////////////////////////////
          std::cout <<"input: " <<input.length() <<std::endl;
          ////////////////////////////////
          while (input.length() >= 5) // while there might be a complete package in input
          {
            //parse for data packets
            data_packet_start = input.find("$\x03");
            if (data_packet_start != std::string::npos)
            {
              ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
              ////////////////////////////////
              std::cout <<"data_packet_start: " <<data_packet_start <<std::endl;
              ////////////////////////////////
              if ((input.length() >= data_packet_start + 5) && (input.compare(data_packet_start + 4, 1, "\n") == 0))  //check if positions 26,27 exist, then test values
              {
                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                //read input data here




                uint8_t received_message_number = input[data_packet_start + 2];
                ROS_DEBUG("received message number: %i", received_message_number);
                ////////////////////////////////
                std::cout <<"message_number: " <<received_message_number <<std::endl;
                ////////////////////////////////





                if (received_message) // can only check for continuous numbers if already received at least one packet
                {
                  uint8_t message_distance = received_message_number - last_received_message_number;
                  if ( message_distance > 1 )
                  {
                    ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU6050 data packets from arduino.");
                  }
                }
                else
                {
                  received_message = true;
                }
                last_received_message_number = received_message_number;

                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);


                // publish imu message



                input.erase(0, data_packet_start + 5); // delete everything up to and including the processed packet
              }//end if ((input.length()
              else
              {
                if (input.length() >= data_packet_start + 5)
                {
                  input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                }
                else
                {
                  // do not delete start character, maybe complete package has not arrived yet
                  input.erase(0, data_packet_start);
                }
              }
            }
            else
            {
              // no start character found in input, so delete everything
              input.clear();
            }
          }
        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
