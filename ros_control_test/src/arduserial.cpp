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
}

void ArduSerial::read(){
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
}

void ArduSerial::write(){
  //write serial
  uint8_t TwoPWM[7] = {'0','2','3','2','0','5','\n'};
  int size_write = _ser.write(TwoPWM, 7);
  std::cout << "write: " << size_write <<std::endl;

}

void ArduSerial::write(uint8_t *data, size_t size){
  int size_write = _ser.write(data, size);
  std::cout << "write: " << size_write <<std::endl;
}
