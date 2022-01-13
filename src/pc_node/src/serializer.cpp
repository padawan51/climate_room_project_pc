#include "pc_node/serializer.h"

bool Serializer::write(uint8 data)
{
    return this->writeBytes(&data, 1);
}

bool Serializer::write(uint16 data)
{
    uint8 byte;
    int nbBit = 16;

    for(int i = 0; i < 2; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(uint32 data)
{
    uint8 byte;
    int nbBit = 32;

    for(int i = 0; i < 4; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(uint64 data)
{
    uint8 byte;
    int nbBit = 64;

    for(int i = 0; i < 8; i++){
        byte = static_cast<uint8>((data >> (nbBit - 8*(i+1))) & 0xFF);
        if(write(byte)) continue;
        else return false;
    }

    return true;
}

bool Serializer::write(int8 data)
{
    return this->write(*reinterpret_cast<uint8*>(&data));
}

bool Serializer::write(int16 data)
{
    return this->write(*reinterpret_cast<uint16*>(&data));
}

bool Serializer::write(int32 data)
{
    return this->write(*reinterpret_cast<uint32*>(&data));
}

bool Serializer::write(float32 data)
{
    uint32 conv;
    Havry::Float32ConversionHelper helper = {};
    helper.f = data;
    conv = helper.u;
    return write(conv);
}

bool Serializer::write(float64 data)
{
    uint64 conv;
    Havry::Float64ConversionHelper helper = {};
    helper.f = data;
    conv = helper.u;

    return write(conv);
}

bool Serializer::writeBytes(const uint8 *buffer, size_t nbBytes)
{
    this->mBuffer.insert(mBuffer.cend(), buffer, buffer + nbBytes);
    return true;
}