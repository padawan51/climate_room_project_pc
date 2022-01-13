#ifndef SERIALIZER_H
#define SERIALIZER_H

#include <vector>
#include "types.h"

class Serializer
{
public:
    Serializer() = default;

    bool write(uint8 data);
    bool write(uint16 data);
    bool write(uint32 data);
    bool write(uint64 data);
    bool write(int8 data);
    bool write(int16 data);
    bool write(int32 data);
    bool write(float32 data);
    bool write(float64 data);

    template<class T>
    bool write(const std::vector<T> &data)
    {
        this->mBuffer.reserve(this->mBuffer.size() + data.size() + 1);

        if(!write(static_cast<uint8>(data.size())))
            return false;

        for(T entry : data){
            if(!write(entry))
                return false;
        }
        return true;
    }

    inline const uint8* buffer() const {return mBuffer.data();}
    inline size_t bufferSize() const {return mBuffer.size();}
    inline void clearBuffer() {mBuffer.clear();}
private:
    std::vector<uint8> mBuffer;
private:
    bool writeBytes(const uint8* buffer, size_t nbBytes);
};

#endif //SERIALIZER_H