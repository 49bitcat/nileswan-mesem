#include "WS/Carts/Nileswan/NileSpiBuffer.h"
#include "Serializer.h"
#include "pch.h"

NileSpiBuffer::NileSpiBuffer(size_t _size)
	: pos(0), size(_size)
{
	data = new uint8_t[size];
}

NileSpiBuffer::~NileSpiBuffer()
{
	delete[] data;
}

void NileSpiBuffer::Reset()
{
    pos = 0;
}

void NileSpiBuffer::Push(const uint8_t* _data, size_t _length)
{
	if(pos + _length >= size) {
		printf("nileswan/spi: !!! BUFFER OVERRUN !!! (%d + %d >= %d)\n", pos, _length, size);
		exit(1);
	}
	if(_data != NULL) {
		memcpy(data + pos, _data, _length);
	} else {
		memset(data + pos, 0xFF, _length);
	}
	pos += _length;
}

bool NileSpiBuffer::Pop(uint8_t* _data, size_t _length)
{
	uint32_t copy_length = _length;
	if(_length > pos) {
		copy_length = pos;
	}

	pos -= copy_length;
	_length -= copy_length;
	if(_data != NULL) {
		memcpy(_data, data, copy_length);
		_data += copy_length;
		while(_length > 0) {
			*(_data++) = 0xFF;
			_length--;
		}
	}
	if(pos > 0) {
		memmove(data, data + copy_length, pos);
	}
	return copy_length > 0;
}

void NileSpiBuffer::Serialize(Serializer& s)
{
    SV(pos);
    SV(size);
	SVArray(data, size);
}
