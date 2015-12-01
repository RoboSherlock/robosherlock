
// UIMA
#include <uima/api.hpp>

// MONGO
#include <mongo/bson/bson.h>

namespace rs
{
namespace conversion
{

mongo::BSONObj fromFeatureStructure(const uima::FeatureStructure &fs, const mongo::OID &parent);
uima::FeatureStructure toFeatureStructure(uima::CAS &cas, const mongo::BSONObj &object);

}
}
