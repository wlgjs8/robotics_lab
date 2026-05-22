#ifndef __mo_grippers_plaif_protocols_base_protocol_h__
#define __mo_grippers_plaif_protocols_base_protocol_h__

namespace plaif
{
  class BaseProtocol
  {
  public:
    virtual ~BaseProtocol() = default;

    virtual void send(int address, int value) = 0;
    virtual int receive(int address) = 0;
    virtual void close() = 0;
  };
}

#endif
