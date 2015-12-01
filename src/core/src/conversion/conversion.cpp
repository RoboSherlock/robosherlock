#include <rs/conversion/conversion.h>
#include <rs/utils/output.h>

using namespace rs::conversion;

ConversionException::ConversionException(const std::string &msg) : std::exception(), msg(msg)
{
}

ConversionException::~ConversionException() throw()
{
}

const char *ConversionException::what() const throw()
{
  return msg.c_str();
}

template<typename T>
void from(const uima::FeatureStructure &fs, T &output)
{
  std::string msg = std::string("no conversion for type '") + typeid(T).name() + "' defined";
  outError(msg);
  throw ConversionException(msg);
}

template<typename T>
uima::FeatureStructure to(uima::CAS &cas, const T &input)
{
  std::string msg = std::string("no conversion for type '") + typeid(T).name() + "' defined";
  outError(msg);
  throw ConversionException(msg);
}
