// STL
#include <map>

// MONGO
#include <mongo/bson/bson.h>

// RS
#include <rs/conversion/bson.h>
#include <rs/conversion/conversion.h>

namespace rs
{
namespace conversion
{

/******************************************************************************
 * Conversion:: FeatureStructure
 *****************************************************************************/

template<>
void from(const uima::FeatureStructure &fs, mongo::BSONObj &output)
{
  output = fromFeatureStructure(fs, mongo::OID());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const mongo::BSONObj &input)
{
  return toFeatureStructure(cas, input);
}

} // namespace conversion

} // namespace rs
