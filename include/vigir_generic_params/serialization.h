//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_GENERIC_PARAMS_SERIALIZATION_H__
#define VIGIR_GENERIC_PARAMS_SERIALIZATION_H__

#include <ros/ros.h>

#define IS_LITTLE_ENDIAN (*(uint16_t*)"\0\1">>8 != 0)
#define IS_BIG_ENDIAN (*(uint16_t*)"\1\0">>8 != 0)



namespace vigir_generic_params
{
class ByteStream
{
public:
  ByteStream(unsigned long size = 8);
  ByteStream(char* buffer, unsigned long size, bool allocate = true);
  virtual ~ByteStream();

  bool write(const void* p, unsigned long size);
  bool read(void* p, unsigned long size);

  void getData(void *other) const;
  bool empty() const;
  unsigned long getBufferSize() const;
  unsigned long getDataSize() const;

protected:
  void* _memcpy(void* dst, const void* src, size_t len) const;

  bool is_little_endian_;
  bool has_allocated_;

  char *buffer_;
  unsigned long size_; // size of buffer
  unsigned long rpos_; // position in buffer for next read
  unsigned long wpos_; // position in buffer for next write
};



template<typename T>
ByteStream& operator<<(ByteStream& data, const T& in)
{
  data.write(&in, sizeof(in));
  return data;
}

template<typename T>
ByteStream& operator<<(ByteStream& data, const std::vector<T>& in)
{
  data << in.size();
  for (typename std::vector<T>::const_iterator itr = in.begin(); itr != in.end(); itr++)
    data << *itr;
  return data;
}

ByteStream& operator<<(ByteStream& data, const std::string& in);
ByteStream& operator<<(ByteStream& data, const XmlRpc::XmlRpcValue& in);

template<typename T>
ByteStream& operator>>(ByteStream& data, T& out)
{
  data.read(&out, sizeof(out));
  return data;
}

template<typename T>
ByteStream& operator>>(ByteStream& data, std::vector<T>& out)
{
  size_t size;
  data >> size;
  out.resize(size);
  for (size_t i = 0; i < size; i++)
  {
    T element;
    data >> element;
    out[i] = element;
  }
  return data;
}

ByteStream& operator>>(ByteStream& data, std::string& out);
ByteStream& operator>>(ByteStream& data, XmlRpc::XmlRpcValue& out);



// needed for conversion to ros msgs
template<typename T>
bool operator<<(std::vector<uint8_t>& data, const T& in)
{
  ByteStream data_buf;
  data_buf << in;
  data.resize(data_buf.getDataSize());
  data_buf.getData(data.data());
  return true;
}

template<typename T>
bool operator>>(const T& in, std::vector<uint8_t>& data)
{
  return operator<<(data, in);
}

template<typename T>
bool operator<<(T& out, const std::vector<uint8_t>& data)
{
  unsigned int length = sizeof(T);
  ByteStream data_buf((char*)data.data(), data.size(), false);
  data_buf >> out;
  return true;
}

template<typename T>
bool operator>>(const std::vector<uint8_t>& data, T& out)
{
  return operator<<(out, data);
}
}

#endif
