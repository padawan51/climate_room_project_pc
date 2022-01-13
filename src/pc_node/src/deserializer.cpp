#include "pc_node/deserializer.h"

bool Deserializer::read(uint8 &data)
{
    return this->readBytes(1, &data);
}

bool Deserializer::read(uint16 &data)
{
    uint8 bytesRead[2];
    uint16 bytes[2];

    if(!readBytes(2, bytesRead)) return false;

    for(int i = 0; i < 2; i++){
        bytes[i] = static_cast<uint16>(bytesRead[i]);
    }

    data = (bytes[0] << 8u) | (bytes[1] << 0u);

    return true;
}

bool Deserializer::read(uint32 &data)
{
    uint8 bytesRead[4];
    uint32 bytes[4];

    if(!readBytes(4, bytesRead)) return false;

    for(int i = 0; i < 4; i++){
        bytes[i] = static_cast<uint32>(bytesRead[i]);
    }

    data = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3] << 0);

    return true;
}

bool Deserializer::read(uint64 &data)
{
    uint8 bytesRead[8];
    uint64 bytes[8];

    if(!readBytes(8, bytesRead)) return false;

    for(int i = 0; i < 8; i++){
        bytes[i] = static_cast<uint32>(bytesRead[i]);
    }

    data = (bytes[0] << 56) |
           (bytes[1] << 48) |
           (bytes[2] << 40) |
           (bytes[3] << 32) |
           (bytes[4] << 24) |
           (bytes[5] << 16) |
           (bytes[6] <<  8) |
           (bytes[7] <<  0);

    return true;
}

bool Deserializer::read(int8 &data)
{
    return this->read(reinterpret_cast<uint8&>(data));
}

bool Deserializer::read(int16 &data)
{
    return this->read(reinterpret_cast<uint16&>(data));
}

bool Deserializer::read(int32 &data)
{
    return this->read(reinterpret_cast<uint32&>(data));
}

bool Deserializer::read(float32 &data)
{
    uint8 conv[4];
    uint32 temp;
    Havry::Float32ConversionHelper helper = {};

    if(readBytes(4, reinterpret_cast<uint8*>(&conv))){
        temp = (static_cast<uint32>(conv[0]) << 24) | //Octet de poid fort
               (static_cast<uint32>(conv[1]) << 16) |
               (static_cast<uint32>(conv[2]) <<  8) |
               (static_cast<uint32>(conv[3]) <<  0);  //Octet de poid faible

        helper.u = temp;
        data = helper.f;

        return true;
    }
    return false;
}

bool Deserializer::read(float64 &data)
{
    uint8 conv[8];
    uint64 temp;
    Havry::Float64ConversionHelper helper = {};

    if(readBytes(8, reinterpret_cast<uint8*>(&conv))){
        temp = (static_cast<uint64>(conv[0]) << 56) | //Octet de poid fort
               (static_cast<uint64>(conv[1]) << 48) |
               (static_cast<uint64>(conv[2]) << 40) |
               (static_cast<uint64>(conv[3]) << 32) |
               (static_cast<uint64>(conv[4]) << 24) |
               (static_cast<uint64>(conv[5]) << 16) |
               (static_cast<uint64>(conv[6]) <<  8) |
               (static_cast<uint64>(conv[7]) <<  0);  //Octet de poid faible

        helper.u = temp;
        data = helper.f;

        return true;
    }
    return false;
}

void Deserializer::setBuffer(const uint8 *buffer)
{
    mBuffer = buffer;
}

void Deserializer::setBufferSize(const size_t bufferSize)
{
    this->mBufferSize = bufferSize;
}

bool Deserializer::readBytes(size_t nbBytes, uint8 *buffer)
{
    if(remainingBytes() < nbBytes) return false;

    for(size_t i = 0; i < nbBytes; ++i){
        buffer[i] = mBuffer[mBytesRead + i];
    }

    mBytesRead += nbBytes;
    return true;
}