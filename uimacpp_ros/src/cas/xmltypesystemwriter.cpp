/** \file xmltypesystemwriter.cpp .
-----------------------------------------------------------------------------


 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

-----------------------------------------------------------------------------

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>
#include <uima/xmltypesystemwriter.hpp>
#include <uima/internal_xmlconstants.hpp>
#include <uima/internal_casimpl.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

using namespace std;

namespace uima {

  XMLTypeSystemWriter::XMLTypeSystemWriter(CAS const & crCAS)
      : iv_crCAS( crCAS) {}

  void XMLTypeSystemWriter::writeType(ostream & os, uima::Type const & crType) const {
    UnicodeStringRef typeName( crType.getName() );

    os << "<" << uima::internal::XMLConstants::TAGNAME_TYPE
    << " " << uima::internal::XMLConstants::ATTRIBUTENAME_NAME << "=\"" << typeName << "\">" << endl;
    vector<uima::Feature> features;
    crType.getAppropriateFeatures(features);
    size_t i;
    for (i=0; i<features.size(); ++i) {
      assert( features[i].isValid());
      // print only features which are introduced at this type
      Type introType;
      features[i].getIntroType(introType);
      assert( introType.isValid() );
      if (introType == crType) {
        Type range;
        features[i].getRangeType(range);
        assert( range.isValid());
        os <<"<" << uima::internal::XMLConstants::TAGNAME_FEATURE
        << " " <<  uima::internal::XMLConstants::ATTRIBUTENAME_NAME << "=\"" << features[i].getName()
        << "\" " << uima::internal::XMLConstants::ATTRIBUTENAME_RANGE << "=\"" << range.getName() << "\"/>" << endl;
// TODO ? add multiRefs ?
      }
    }
    vector<uima::Type> subTypes;
    crType.getDirectSubTypes(subTypes);
    for (i=0; i<subTypes.size(); ++i) {
      assert( subTypes[i].isValid() );
      writeType(os, subTypes[i]);
    }
    os << "</" << uima::internal::XMLConstants::TAGNAME_TYPE << ">" << endl;
  }

  void XMLTypeSystemWriter::write(std::ostream & os) const {
    os << "<?xml version=\"1.0\"?>" << endl;
    os << "<!DOCTYPE " << uima::internal::XMLConstants::TAGNAME_TYPEHIERARCHY << " [\n"
    "<!ELEMENT " << uima::internal::XMLConstants::TAGNAME_TYPEHIERARCHY << " (type*)>\n"
    "<!ELEMENT " << uima::internal::XMLConstants::TAGNAME_TYPE << " (" << uima::internal::XMLConstants::TAGNAME_FEATURE << "|" << uima::internal::XMLConstants::TAGNAME_TYPE << ")*>\n"
    "<!ATTLIST " << uima::internal::XMLConstants::TAGNAME_TYPE << " " << uima::internal::XMLConstants::ATTRIBUTENAME_NAME << " CDATA #REQUIRED>\n"
    "<!ELEMENT " << uima::internal::XMLConstants::TAGNAME_FEATURE << " EMPTY>\n"
    "<!ATTLIST " << uima::internal::XMLConstants::TAGNAME_FEATURE << " " << uima::internal::XMLConstants::ATTRIBUTENAME_NAME << " CDATA #REQUIRED " << uima::internal::XMLConstants::ATTRIBUTENAME_RANGE << " CDATA #REQUIRED >\n"
    "]>" << endl;

    os << "<" << uima::internal::XMLConstants::TAGNAME_TYPEHIERARCHY << ">" << endl;
    Type top = iv_crCAS.getTypeSystem().getTopType();
    writeType(os, top);
    os << "</" << uima::internal::XMLConstants::TAGNAME_TYPEHIERARCHY << ">" << endl;

  }

}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */



