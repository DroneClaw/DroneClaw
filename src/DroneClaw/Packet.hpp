/*
    DroneClaw copyright 2016
*/

#ifndef _PACKET
#define _PACKET

/** The packet class that can encode and decode data*/
class Packet {
  private:
    byte _id;
    void (*_function)(Stream&);
  public:
    inline Packet(byte id, void (*function)(Stream&)) {
      _id = id;
      _function = function;
    }
    /** This will decode the data */
    inline void decode(Stream &data) {
      _function(data);
    }
};

#endif
