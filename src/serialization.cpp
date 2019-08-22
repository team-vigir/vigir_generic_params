#include <vigir_generic_params/serialization.h>

namespace vigir_generic_params
{
ByteStream::ByteStream(size_t size)
  : is_little_endian_(IS_LITTLE_ENDIAN)
  , has_allocated_(true)
  , size_(size)
  , rpos_(0)
  , wpos_(0)
{
  buffer_ = (char*)malloc(size * sizeof(char));
}

ByteStream::ByteStream(char* buffer, size_t size, bool allocate)
  : is_little_endian_(IS_LITTLE_ENDIAN)
  , has_allocated_(allocate)
  , size_(size)
  , rpos_(0)
  , wpos_(size)
{
  if (allocate)
  {
    buffer_ = (char*)malloc(size * sizeof(char));
    memcpy(buffer_, buffer, size);
  }
  else
  {
    buffer_ = buffer;
  }
}

ByteStream::~ByteStream()
{
  if (has_allocated_)
    free(buffer_);
}

bool ByteStream::write(const void* p, size_t size)
{
  if (remainingUnwrittenBytes() < size)
  {
    if (!has_allocated_)
      return false;

    size_t old_size = size_;

    while (remainingUnwrittenBytes() < size)
      size_ *= 2;

    buffer_ = (char*)realloc(buffer_, size_);

    ROS_DEBUG("[ByteStream] Increased buffer size from %lu to %lu", old_size, size_);
  }

  _memcpy(buffer_ + wpos_, p, size);
  wpos_ += size;

  return true;
}

bool ByteStream::read(void* p, size_t size)
{
  if (remainingUnreadBytes() < size)
    return false;

  _memcpy(p, buffer_ + rpos_, size);
  rpos_ += size;

  return true;
}

void* ByteStream::_memcpy(void* dst, const void* src, size_t len) const
{
  if (is_little_endian_)
  {
    return memcpy(dst, src, len);
  }
  else  // big endian machine: swap byte order
  {
    for (unsigned int i = 0; i < len; i++)
      memcpy((char*)dst + i, (char*)src + (len - i - 1), 1);
    return dst;
  }
}

ByteStream& operator<<(ByteStream& stream, const std::string& in)
{
  stream << (u_int64_t)in.size();
  stream.write(in.c_str(), in.size());
  return stream;
}

ByteStream& operator<<(ByteStream& stream, const XmlRpc::XmlRpcValue& in)
{
  using namespace XmlRpc;

  XmlRpc::XmlRpcValue& _in = const_cast<XmlRpc::XmlRpcValue&>(in);  // needed because XmlRpc doesn't implement const getters

  stream << (u_int8_t)in.getType();

  switch (in.getType())
  {
    case XmlRpcValue::TypeBoolean:
      stream << static_cast<bool&>(_in);
      break;
    case XmlRpcValue::TypeInt:
      stream << static_cast<int32_t&>(_in);
      break;
    case XmlRpcValue::TypeDouble:
      stream << static_cast<double&>(_in);
      break;
    case XmlRpcValue::TypeString:
      stream << static_cast<std::string&>(_in);
      break;
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
      stream << (u_int32_t)in.size();
      for (size_t i = 0; i < in.size(); i++)
        stream << in[i];
      break;
    }
    case XmlRpcValue::TypeStruct:
    {
      stream << (u_int32_t)in.size();
      for (XmlRpc::XmlRpcValue::iterator itr = _in.begin(); itr != _in.end(); itr++)
        stream << itr->first << itr->second;
      //      std::string xml = in.toXml();
      //      data << xml;
      break;
    }
    default:
      ROS_ERROR("operator<<(ByteStream& data, XmlRpcValue& in) not implemented for type '%u'!", in.getType());
      break;
  }

  return stream;
}

ByteStream& operator>>(ByteStream& stream, std::string& out)
{
  u_int64_t size;
  stream >> size;
  out.resize(size);
  stream.read((void*)out.data(), size);
  return stream;
}

ByteStream& operator>>(ByteStream& stream, XmlRpc::XmlRpcValue& out)
{
  using namespace XmlRpc;

  u_int8_t type;
  stream >> type;

  switch (type)
  {
    case XmlRpcValue::TypeBoolean:
    {
      bool val;
      stream >> val;
      out = XmlRpcValue(val);
      break;
    }
    case XmlRpcValue::TypeInt:
    {
      int32_t val;
      stream >> val;
      out = XmlRpcValue((int)val);
      break;
    }
    case XmlRpcValue::TypeDouble:
    {
      double val;
      stream >> val;
      out = XmlRpcValue(val);
      break;
    }
    case XmlRpcValue::TypeString:
    {
      std::string val;
      stream >> val;
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
      stream >> size;

      out.clear();
      out.setSize(size);

      for (u_int32_t i = 0; i < size; i++)
        stream >> out[i];
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
      stream >> size;
      for (u_int32_t i = 0; i < size; i++)
      {
        std::string name;
        XmlRpcValue val;
        stream >> name;
        stream >> val;
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

  return stream;
}
}  // namespace vigir_generic_params
