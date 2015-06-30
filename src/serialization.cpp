#include <vigir_generic_params/serialization.h>

namespace vigir_generic_params
{
ByteStream::ByteStream(unsigned long size)
  : is_little_endian(IS_LITTLE_ENDIAN)
  , has_allocated(true)
  , size(size)
  , rpos(0)
  , wpos(0)
{
  buffer = (char*)malloc(size * sizeof(char));
}

ByteStream::ByteStream(char* buffer, unsigned long size, bool allocate)
  : is_little_endian(IS_LITTLE_ENDIAN)
  , has_allocated(allocate)
  , size(size)
  , rpos(0)
  , wpos(size)
{
  if (allocate)
  {
    this->buffer = (char*)malloc(size * sizeof(char));
    memcpy(this->buffer, buffer, size);
  }
  else
  {
    this->buffer = buffer;
  }
}

ByteStream::~ByteStream()
{
  if (has_allocated)
    free(buffer);
}

bool ByteStream::write(const void* p, unsigned long size)
{
  if (this->size-wpos < size)
  {
    if (!has_allocated)
      return false;

    unsigned long old_size = this->size;

    while (this->size-wpos < size)
      this->size*=2;

    buffer = (char*)realloc(buffer, this->size);

    ROS_DEBUG("[ByteStream] Increased buffer size from %lu to %lu", old_size, this->size);
  }

  _memcpy(buffer + wpos, p, size);
  wpos += size;

  return true;
}

bool ByteStream::read(void* p, unsigned long size)
{
  if (this->size-rpos < size)
    return false;

  _memcpy(p, buffer + rpos, size);
  rpos += size;

  return true;
}

void ByteStream::getData(void* other) const
{
  memcpy(other, this->buffer, getDataSize());
}

bool ByteStream::empty() const
{
  return wpos == 0;
}

unsigned long ByteStream::getBufferSize() const
{
  return size;
}

unsigned long ByteStream::getDataSize() const
{
  return wpos;
}

void* ByteStream::_memcpy(void* dst, const void* src, size_t len) const
{
  if (is_little_endian)
  {
    return memcpy(dst, src, len);
  }
  else // big endian machine: swap byte order
  {
    for (unsigned int i = 0; i < len; i++)
      memcpy((char*)dst+i, (char*)src+(len-i-1), 1);
    return dst;
  }
}



ByteStream& operator<<(ByteStream& data, const std::string& in)
{
  data << (u_int64_t) in.size();
  data.write(in.c_str(), in.size());
  return data;
}

ByteStream& operator<<(ByteStream& data, const XmlRpc::XmlRpcValue& in)
{
  using namespace XmlRpc;

  XmlRpc::XmlRpcValue& _in = const_cast<XmlRpc::XmlRpcValue&>(in); // needed because XmlRpc doesn't implement const getters

  data << (u_int8_t) in.getType();

  switch (in.getType())
  {
    case XmlRpcValue::TypeBoolean:  data << static_cast<bool&>(_in); break;
    case XmlRpcValue::TypeInt:      data << static_cast<int32_t&>(_in); break;
    case XmlRpcValue::TypeDouble:   data << static_cast<double&>(_in); break;
    case XmlRpcValue::TypeString:   data << static_cast<std::string&>(_in); break;
//    case XmlRpcValue::TypeDateTime:
//    {
//        struct tm* t = _value.asTime;
//        char buf[20];
//        snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d",
//          t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
//        buf[sizeof(buf)-1] = 0;
//        data << buf;
//        break;
//    }
//    case XmlRpcValue::TypeBase64:
//    {
//        int iostatus = 0;
//        std::ostreambuf_iterator<char> out(os);
//        base64<char> encoder;
//        encoder.put(_value.asBinary->begin(), _value.asBinary->end(), out, iostatus, base64<>::crlf());
//        break;
//    }
  case XmlRpcValue::TypeArray:
    {
      data << (u_int32_t) in.size();
      for (size_t i = 0; i < in.size(); i++)
        data << in[i];
      break;
    }
    case XmlRpcValue::TypeStruct:
    {
      data << (u_int32_t) in.size();
      for (XmlRpc::XmlRpcValue::iterator itr = _in.begin(); itr != _in.end(); itr++)
        data << itr->first << itr->second;
//      std::string xml = in.toXml();
//      data << xml;
      break;
    }
    default:
      ROS_ERROR("operator<<(ByteStream& data, XmlRpcValue& in) not implemented for type '%u'!", in.getType());
      break;
  }

  return data;
}

ByteStream& operator>>(ByteStream& data, std::string& out)
{
  u_int64_t size;
  data >> size;
  out.resize(size);
  data.read((void*)out.data(), size);
  return data;
}

ByteStream& operator>>(ByteStream& data, XmlRpc::XmlRpcValue& out)
{
  using namespace XmlRpc;

  u_int8_t type;
  data >> type;

  switch (type)
  {
    case XmlRpcValue::TypeBoolean:
    {
      bool val;
      data >> val;
      out = XmlRpcValue(val);
      break;
    }
    case XmlRpcValue::TypeInt:
    {
      int32_t val;
      data >> val;
      out = XmlRpcValue((int) val);
      break;
    }
    case XmlRpcValue::TypeDouble:
    {
      double val;
      data >> val;
      out = XmlRpcValue(val);
      break;
    }
    case XmlRpcValue::TypeString:
    {
      std::string val;
      data >> val;
      out = XmlRpcValue(val);
      break;
    }
//    case XmlRpcValue::TypeDateTime:
//    {
//        struct tm* t = _value.asTime;
//        char buf[20];
//        snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d",
//          t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
//        buf[sizeof(buf)-1] = 0;
//        data << buf;
//        break;
//    }
//    case XmlRpcValue::TypeBase64:
//    {
//        int iostatus = 0;
//        std::ostreambuf_iterator<char> out(os);
//        base64<char> encoder;
//        encoder.put(_value.asBinary->begin(), _value.asBinary->end(), out, iostatus, base64<>::crlf());
//        break;
//    }
    case XmlRpcValue::TypeArray:
    {
      u_int32_t size;
      data >> size;

      out.clear();
      out.setSize(size);

      for (u_int32_t i = 0; i < size; i++)
        data >> out[i];
      break;
    }
    case XmlRpcValue::TypeStruct:
    {
      // locally implemented wrapper to get access to struct field
      struct XmlRpcValueStruct : public XmlRpc::XmlRpcValue
      {
        XmlRpcValueStruct()
          : XmlRpc::XmlRpcValue()
        {
          assertStruct();
        }

        void insert(const std::string& name, XmlRpcValue& val)
        {
          const std::pair<const std::string, XmlRpcValue> p(name, val);
          _value.asStruct->insert(p);
        }
      } strct;

      u_int32_t size;
      data >> size;
      for (u_int32_t i = 0; i < size; i++)
      {
        std::string name;
        XmlRpcValue val;
        data >> name;
        data >> val;
        strct.insert(name, val);
      }
      out = strct;
      break;
    }
    default:
      ROS_ERROR("operator<<(XmlRpcValue& out, const ByteStream& data) not implemented for type '%u'!", type);
      break;
  }

  if (!out.valid())
    ROS_ERROR("Couldn't deserialize msg of type '%u'!", type);

  return data;
}
}
