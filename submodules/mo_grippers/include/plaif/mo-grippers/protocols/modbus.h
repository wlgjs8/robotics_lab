#ifndef __mo_grippers_plaif_protocols_modbus_h__
#define __mo_grippers_plaif_protocols_modbus_h__

#include <plaif/mo-grippers/protocols/base-protocol.h>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include <iostream>
#include <stdexcept>

namespace plaif
{
  class ModbusProtocol : public BaseProtocol
  {
  public:
    ModbusProtocol(const std::string &port = "/dev/ttyUSB0", int baudrate = 115200,
                   char parity = 'N', int bytesize = 8, int stopbits = 1, int slave_id = 1, int timeout = 1)
        : slave_id_(slave_id)
    {
      client_ = modbus_new_rtu(port.c_str(), baudrate, parity, bytesize, stopbits);

      if (!client_)
      {
        throw std::runtime_error("Failed to create a Modbus context!");
      }

      modbus_set_slave(client_, slave_id_);
      modbus_set_response_timeout(client_, 0, 50000); // Set timeout

      if (modbus_connect(client_) == -1)
      {
        modbus_free(client_);
        throw std::runtime_error("Unable to connect to the gripper!");
      }
    }

    void send(const int address, const int value) override
    {
      modbus_write_register(client_, address, value);
    }

    int receive(const int address) override
    {
      uint16_t reg;
      modbus_read_registers(client_, address, 1, &reg);
      return reg;
    }

    void close() override
    {
      if (client_)
      {
        modbus_close(client_);
        modbus_free(client_);
      }
    }

  private:
    modbus_t *client_;
    int slave_id_;
  };
}

#endif
