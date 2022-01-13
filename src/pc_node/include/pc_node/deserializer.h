#ifndef DESERIALIZER_H
#define DESERIALIZER_H

#include "types.h"
#include <vector>

class Deserializer
{
public:
    Deserializer() = default;
    Deserializer(const uint8* buffer, const size_t bufferSize):
        mBuffer(buffer), mBufferSize(bufferSize)
	{}

    bool read(uint8& data);
    bool read(uint16& data);
    bool read(uint32& data);
    bool read(uint64& data);
    bool read(int8& data);
    bool read(int16& data);
    bool read(int32& data);
    bool read(float32& data);
    bool read(float64& data);

    template<typename T>
    bool read(std::vector<T>& data){
        uint8 nbElts;

        if(!read(nbElts))
            return false;

        data.reserve(nbElts);

        for(uint8 i = 0; i < nbElts; ++i){
            std::vector<T>::value_type element;

            if(!read(element))
                return false;
            data.push_back(element);
        }
        return true;
    }

    inline size_t remainingBytes() const {return mBufferSize - mBytesRead;}
    void setBuffer(const uint8 *buffer);
    void setBufferSize(const size_t bufferSize);

private:
    const uint8* mBuffer;
    size_t mBufferSize;
    size_t mBytesRead{0};
private:
    bool readBytes(size_t nbBytes, uint8* buffer);
};

#endif //DESERIALIZER_H