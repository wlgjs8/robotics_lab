#include <iostream>
#include <modbus/modbus.h>

int main()
{
  // Create Modbus RTU context
  modbus_t *ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
  if (!ctx)
  {
    std::cerr << "Failed to create Modbus context\n";
    return 1;
  }

  // Set slave ID
  modbus_set_slave(ctx, 1);

  // Connect to the device
  if (modbus_connect(ctx) == -1)
  {
    std::cerr << "Modbus connection failed: " << modbus_strerror(errno) << "\n";
    modbus_free(ctx);
    return 1;
  }

  // Example: Write a single register (address: 0x0100, value: 165)
  int rc = modbus_write_register(ctx, 0x0100, 165);
  if (rc == -1)
  {
    std::cerr << "Failed to write register: " << modbus_strerror(errno) << "\n";
  }
  else
  {
    std::cout << "Register written successfully\n";
  }

  // Close connection and free resources
  modbus_close(ctx);
  modbus_free(ctx);

  return 0;
}
