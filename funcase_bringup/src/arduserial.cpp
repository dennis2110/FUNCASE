#include "arduserial.h"

ArduSerial::ArduSerial(std::string port, size_t length){
  _port = port;
  _length = length;
}

ArduSerial::~ArduSerial(){

}

void ArduSerial::init(){
  _ser.setPort(_port);
  _ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  _ser.setTimeout(to);
  _ser.open();
  ROS_INFO("arduserial init port: %s",_port.c_str());
}

/*void ArduSerial::read(){
  if (_ser.isOpen())
  {
    // read string from serial device
    if(_ser.available())
    {
      _read = _ser.read(_ser.available());
      ////////////////////////////////
      //std::cout<< "read: " << ser.available()<<std::endl;
      ////////////////////////////////
      ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)_read.size(), (int)_input.size());

      _input += _read;
      ////////////////////////////////
      //std::cout <<"input: " <<input.length() <<std::endl;
      ////////////////////////////////
      while (_input.length() >= 5) // while there might be a complete package in input
      {
        //parse for data packets
        data_packet_start = _input.find("$\x03");
        if (data_packet_start != std::string::npos)
        {
          ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
          ////////////////////////////////
          //std::cout <<"data_packet_start: " <<data_packet_start <<std::endl;
          ////////////////////////////////
          if ((_input.length() >= data_packet_start + 5) && (_input.compare(data_packet_start + 4, 1, "\n") == 0))  //check if positions 26,27 exist, then test values
          {
            ROS_DEBUG("seems to be a real data package: long enough and found end characters");
            //
            //
            //
            //
            //read input data here
            int16_t pwmA = 0xff &(char)_input[data_packet_start + 2];
            int16_t pwmB = 0xff &(char)_input[data_packet_start + 3];
            PWM_A = _input[data_packet_start + 2];
            PWM_B = _input[data_packet_start + 3];
            ROS_INFO("read %d\t%d",pwmA,pwmB);
            //
            //
            //
            //
            //
            _input.erase(0, data_packet_start + 5); // delete everything up to and including the processed packet
          }//end if ((input.length()
          else
          {
            if (_input.length() >= data_packet_start + 5)
            {
              _input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
            }
            else
            {
              // do not delete start character, maybe complete package has not arrived yet
              _input.erase(0, data_packet_start);
            }
          }
        }
        else
        {
          // no start character found in input, so delete everything
          _input.clear();
        }
      }
    }
  }
}*/

void ArduSerial::read(uint8_t *data, size_t size){
  try
      {
        if (_ser.isOpen())
        {
          // read string from serial device
          if(_ser.available())
          {
            _read = _ser.read(_ser.available());
            ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)_read.size(), (int)_input.size());
            _input += _read;
            //ROS_INFO("in _ser.available");
            while (_input.length() >= size) // while there might be a complete package in input
            {
              //ROS_INFO("in while");
              //parse for data packets
              data_packet_start = _input.find("$\x03");
              if (data_packet_start != std::string::npos)
              {
                //ROS_INFO("found possible start of data packet at position %d", data_packet_start);
                if ((_input.length() >= data_packet_start + size) && (_input.compare(data_packet_start + size-2, 2, "\r\n") == 0))  //check if positions 26,27 exist, then test values
                {
                  ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                  /////////////////////////////////////////////////////////
                  for (int i = 0; i < size-4; i++) {
                    data[i] = _input[data_packet_start+i+2];
                  }
                  /////////////////////////////////////////////////////////

//                  // get quaternion values
//                  int16_t w = (((0xff &(char)_input[data_packet_start + 2]) << 8) | 0xff &(char)_input[data_packet_start + 3]);
//                  int16_t x = (((0xff &(char)_input[data_packet_start + 4]) << 8) | 0xff &(char)_input[data_packet_start + 5]);
//                  int16_t y = (((0xff &(char)_input[data_packet_start + 6]) << 8) | 0xff &(char)_input[data_packet_start + 7]);
//                  int16_t z = (((0xff &(char)_input[data_packet_start + 8]) << 8) | 0xff &(char)_input[data_packet_start + 9]);

//                  double wf = w/16384.0;
//                  double xf = x/16384.0;
//                  double yf = y/16384.0;
//                  double zf = z/16384.0;

//                  /*tf::Quaternion orientation(xf, yf, zf, wf);

//                  if (!zero_orientation_set)
//                  {
//                    zero_orientation = orientation;
//                    zero_orientation_set = true;
//                  }

//                  //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
//                  tf::Quaternion differential_rotation;
//                  differential_rotation = zero_orientation.inverse() * orientation;*/

//                  // get gyro values
//                  int16_t gx = (((0xff &(char)_input[data_packet_start + 10]) << 8) | 0xff &(char)_input[data_packet_start + 11]);
//                  int16_t gy = (((0xff &(char)_input[data_packet_start + 12]) << 8) | 0xff &(char)_input[data_packet_start + 13]);
//                  int16_t gz = (((0xff &(char)_input[data_packet_start + 14]) << 8) | 0xff &(char)_input[data_packet_start + 15]);
//                  // calculate rotational velocities in rad/s
//                  // without the last factor the velocities were too small
//                  // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
//                  // FIFO frequency 100 Hz -> factor 10 ?
//                  // seems 25 is the right factor
//                  //TODO: check / test if rotational velocities are correct
//                  double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
//                  double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
//                  double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

//                  // get acelerometer values
//                  int16_t ax = (((0xff &(char)_input[data_packet_start + 16]) << 8) | 0xff &(char)_input[data_packet_start + 17]);
//                  int16_t ay = (((0xff &(char)_input[data_packet_start + 18]) << 8) | 0xff &(char)_input[data_packet_start + 19]);
//                  int16_t az = (((0xff &(char)_input[data_packet_start + 20]) << 8) | 0xff &(char)_input[data_packet_start + 21]);
//                  // calculate accelerations in m/s²
//                  double axf = ax * (8.0 / 65536.0) * 9.81;
//                  double ayf = ay * (8.0 / 65536.0) * 9.81;
//                  double azf = az * (8.0 / 65536.0) * 9.81;

//                  // get temperature
//                  int16_t temperature = (((0xff &(char)_input[data_packet_start + 22]) << 8) | 0xff &(char)_input[data_packet_start + 23]);
//                  double temperature_in_C = (temperature / 340.0 ) + 36.53;
//                  ROS_DEBUG_STREAM("Temperature [in C] " << temperature_in_C);

//                  uint8_t received_message_number = _input[data_packet_start + 25];
//                  ROS_DEBUG("received message number: %i", received_message_number);

                  /*if (received_message) // can only check for continuous numbers if already received at least one packet
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
                  imu.header.stamp = measurement_time;
                  imu.header.frame_id = frame_id;

                  quaternionTFToMsg(differential_rotation, imu.orientation);

                  imu.angular_velocity.x = gxf;
                  imu.angular_velocity.y = gyf;
                  imu.angular_velocity.z = gzf;

                  imu.linear_acceleration.x = axf;
                  imu.linear_acceleration.y = ayf;
                  imu.linear_acceleration.z = azf;

                  imu_pub.publish(imu);

                  // publish temperature message
                  temperature_msg.header.stamp = measurement_time;
                  temperature_msg.header.frame_id = frame_id;
                  temperature_msg.temperature = temperature_in_C;

                  imu_temperature_pub.publish(temperature_msg);*/

                  // publish tf transform
                  /*if (broadcast_tf)
                  {
                    transform.setRotation(differential_rotation);
                    tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                  }*/
                  _input.erase(0, data_packet_start + size); // delete everything up to and including the processed packet
                }
                else
                {
                  if (_input.length() >= data_packet_start + size)
                  {
                    _input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                  }
                  else
                  {
                    // do not delete start character, maybe complete package has not arrived yet
                    _input.erase(0, data_packet_start);
                  }
                }
              }
              else
              {
                // no start character found in input, so delete everything
                _input.clear();
              }
            }
          }
        }
        else
        {
          // try and open the serial port
          try
          {
            _ser.setPort(_port);
            _ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            _ser.setTimeout(to);
            _ser.open();
          }
          catch (serial::IOException& e)
          {
            ROS_ERROR_STREAM("Unable to open serial port " << _ser.getPort() << ". Trying again in 5 seconds.");
            ros::Duration(5).sleep();
          }

          if(_ser.isOpen())
          {
            ROS_DEBUG_STREAM("Serial port " << _ser.getPort() << " initialized and opened.");
          }
        }
      }
      catch (serial::IOException& e)
      {
        ROS_ERROR_STREAM("Error reading from the serial port " << _ser.getPort() << ". Closing connection.");
        _ser.close();
      }
}

/*void ArduSerial::write(){
  //write serial
  uint8_t TwoPWM[7] = {'0','2','3','2','0','5','\n'};
  int size_write = _ser.write(TwoPWM, 7);
  std::cout << "write: " << size_write <<std::endl;

}*/

void ArduSerial::write(uint8_t *data, size_t size){
  if(_ser.isOpen()){
    size_t size_write = _ser.write(data, size);
    ///ROS_INFO("write data to arduino : %s",_ser.getPort().c_str());
    //std::cout << "write: " << size_write <<std::endl;
  }
}
