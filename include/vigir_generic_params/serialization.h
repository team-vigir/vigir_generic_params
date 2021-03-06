//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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

#include <cstring>
#include <vector>
#include <map>
#include <unordered_map>

#define IS_LITTLE_ENDIAN (*(uint16_t*)"\0\1" >> 8 != 0)
#define IS_BIG_ENDIAN (*(uint16_t*)"\1\0" >> 8 != 0)

namespace vigir_generic_params
{
/**
 * @brief The ByteStream enables to serialize any data type into a stream of bytes. Hereby,
 * it is endian-safe, thus can be used between different machines for safe data exchange.
 * However, the ByteStream is designed to reduce the cpu overhead in cost of memory allocation.
 * ByteStream objects are supposed to have a short-lifespan and should be only used for a single
 * serialization as otherwise the object will allocate continuously more memory.
 */
class ByteStream
{
public:
  ByteStream(size_t size = 8);
  ByteStream(char* buffer, size_t size, bool allocate = true);
  virtual ~ByteStream();

  bool write(const void* p, size_t size);
  bool read(void* p, size_t size);

  inline void getData(void* other) const { memcpy(other, buffer_, getDataSize()); }

  inline bool empty() const { return wpos_ == 0; }

  inline size_t getBufferSize() const { return size_; }
  inline size_t getDataSize() const { return wpos_; }

  inline size_t remainingUnwrittenBytes() const { return size_ - wpos_; }
  inline size_t remainingUnreadBytes() const { return wpos_ - rpos_; }

protected:
  void* _memcpy(void* dst, const void* src, size_t len) const;

  bool is_little_endian_;
  bool has_allocated_;

  char* buffer_;
  size_t size_;  // size of buffer
  size_t rpos_;  // position in buffer for next read
  size_t wpos_;  // position in buffer for next write
};

/**
 * In-Stream operators
 */
// template for integral types
template <typename T>
ByteStream& operator<<(ByteStream& stream, const T& in)
{
  stream.write(&in, sizeof(in));
  return stream;
}

template <template <typename...> class Container, typename T>
ByteStream& operator<<(ByteStream& stream, const Container<T>& in)
{
  stream << in.size();
  for (typename Container<T>::const_iterator itr = in.begin(); itr != in.end(); itr++)
    stream << *itr;
  return stream;
}

template <typename K, typename T>
ByteStream& operator<<(ByteStream& stream, const std::pair<K, T>& in)
{
  stream << in.first;
  stream << in.second;
  return stream;
}

template <typename K, typename T>
ByteStream& operator<<(ByteStream& stream, const std::map<K, T>& in)
{
  stream << in.size();
  for (typename std::map<K, T>::const_iterator itr = in.begin(); itr != in.end(); itr++)
    stream << *itr;
  return stream;
}

template <typename K, typename T>
ByteStream& operator<<(ByteStream& stream, const std::unordered_map<K, T>& in)
{
  stream << in.size();
  for (typename std::unordered_map<K, T>::const_iterator itr = in.begin(); itr != in.end(); itr++)
    stream << *itr;
  return stream;
}

ByteStream& operator<<(ByteStream& stream, const std::string& in);
ByteStream& operator<<(ByteStream& stream, const XmlRpc::XmlRpcValue& in);

/**
 * Out-Stream operators
 */
// template for integral types
template <typename T>
ByteStream& operator>>(ByteStream& stream, T& out)
{
  stream.read(&out, sizeof(out));
  return stream;
}

template <template <typename...> class Container, typename T>
ByteStream& operator>>(ByteStream& stream, Container<T>& out)
{
  size_t size;
  stream >> size;

  if (std::is_member_function_pointer<decltype(&Container<T>::reserve)>::value)
    out.reserve(size);
  for (size_t i = 0; i < size; i++)
  {
    T element;
    stream >> element;
    out.push_back(std::move(element));
  }
  return stream;
}

template <typename K, typename T>
ByteStream& operator>>(ByteStream& stream, std::pair<K, T>& out)
{
  stream >> out.first;
  stream >> out.second;
  return stream;
}

template <typename K, typename T>
ByteStream& operator>>(ByteStream& stream, std::map<K, T>& out)
{
  size_t size;
  stream >> size;
  for (size_t i = 0; i < size; i++)
  {
    std::pair<K, T> element;
    stream >> element;
    out.insert(std::move(element));
  }
  return stream;
}

template <typename K, typename T>
ByteStream& operator>>(ByteStream& stream, std::unordered_map<K, T>& out)
{
  size_t size;
  stream >> size;
  for (size_t i = 0; i < size; i++)
  {
    std::pair<K, T> element;
    stream >> element;
    out.insert(std::move(element));
  }
  return stream;
}

ByteStream& operator>>(ByteStream& stream, std::string& out);
ByteStream& operator>>(ByteStream& stream, XmlRpc::XmlRpcValue& out);

/**
 * Templates required for conversion to ros msgs using std::vector<uint8_t> as member
 */
template <typename T>
bool operator<<(std::vector<uint8_t>& stream, const T& in)
{
  ByteStream data_buf;
  data_buf << in;
  stream.resize(data_buf.getDataSize());
  data_buf.getData(stream.data());
  return true;
}

// variant for swapped argument
template <typename T>
bool operator>>(const T& in, std::vector<uint8_t>& stream)
{
  return operator<<(stream, in);
}

template <typename T>
bool operator<<(T& out, const std::vector<uint8_t>& stream)
{
  ByteStream data_buf((char*)stream.data(), stream.size(), false);
  data_buf >> out;
  return true;
}

// variant for swapped argument
template <typename T>
bool operator>>(const std::vector<uint8_t>& stream, T& out)
{
  return operator<<(out, stream);
}
}  // namespace vigir_generic_params

#endif
