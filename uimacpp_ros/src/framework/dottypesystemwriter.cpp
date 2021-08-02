/** \file dottypesystemwriter.cpp .
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


   09/20/2002  Initial creation

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>
#include <uima/dottypesystemwriter.hpp>
#include <uima/timedatetools.hpp>
#include "unicode/ustring.h"
#include <uima/unistrref.hpp>
#include <uima/typesystem.hpp>
#include <uima/cas.hpp>
#include <time.h>
#ifdef _MSC_VER
#include <minmax.h> // for min
#endif

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

  const size_t cuiINDENT = 3;

  std::string
  currDateString() {
    time_t ltime;
    (void)time(&ltime);   //get time, ignore return value
    const size_t uiMAXSTR = 64;
    char pszDate[uiMAXSTR];
    strftime(pszDate, uiMAXSTR, "%x", localtime(&ltime));
    string t(pszDate);
    return t; //time string
  }

  UnicodeStringRef adjustTypeName(UnicodeStringRef ulstrName) {
    icu::UnicodeString ustrTypePfx( "uima" );                         // Our namespace
    ustrTypePfx.append( (UChar) uima::TypeSystem::NAMESPACE_SEPARATOR );
    if (u_strncmp(ulstrName.getBuffer(), ustrTypePfx.getBuffer(), min(ulstrName.length(), ustrTypePfx.length())) == 0) {
      ulstrName = UnicodeStringRef(ulstrName.getBuffer() + ustrTypePfx.length(),
                                   ulstrName.length()- ustrTypePfx.length());
    }
    return ulstrName;
  }

  UnicodeStringRef adjustFeatName(UnicodeStringRef ulstrName) {
    return ulstrName;
  }

  DotTypeSystemWriter::DotTypeSystemWriter(CAS const & crCAS, bool bNoCASTypes, bool bOnlyCASTypes ) :
      iv_crCAS(crCAS),
      iv_bNoCASTypes(bNoCASTypes),
      iv_bOnlyCASTypes(bOnlyCASTypes) {}

  static inline bool isCasType(uima::Type const & crType) {
    return    crType.getName().startsWith(icu::UnicodeString("uima.cas." ))
              || crType.getName().startsWith(icu::UnicodeString("uima.tcas."));
  }

  void DotTypeSystemWriter::writeType(ostream & os, uima::Type const & crType, size_t uiIndent) const {
    if (iv_bOnlyCASTypes &&
        (crType != crType.getTypeSystem().getTopType()) && (!isCasType(crType))) {
      return;
    }
    if (iv_bNoCASTypes && isCasType(crType)) {
      return;
    }
    string strIndentString(uiIndent, ' ');
    os << strIndentString;
    os << adjustTypeName(crType.getName())
    << "[label=\"" << adjustTypeName(crType.getName());
    vector<Feature> features;
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
        os << "|{" << adjustFeatName(features[i].getName()) << ": " << adjustTypeName(range.getName()) << "}";
      }
    }
    os << "\"];" << endl;
    vector<Type> subTypes;
    crType.getDirectSubTypes(subTypes);
    if (subTypes.size() > 0) {
      // print <typename> -> {<sub-type1>, <sub-type2>, ...};
      os << strIndentString;
      os << adjustTypeName(crType.getName()) << " -> {";
      size_t uiTypesWritten = 0;
      for (i=0; i<subTypes.size(); ++i) {
        if (iv_bOnlyCASTypes && !isCasType(subTypes[i]) ) {
          continue;
        }
        if (iv_bNoCASTypes && isCasType(subTypes[i]) ) {
          continue;
        }
        if (uiTypesWritten > 0 ) {
          os << "; ";
        }
        os << adjustTypeName(subTypes[i].getName());
        ++uiTypesWritten;
      }
      os << "};" << endl;
    }
    for (i=0; i<subTypes.size(); ++i) {
      writeType(os, subTypes[i], uiIndent+cuiINDENT);
    }
  }

  void DotTypeSystemWriter::write(std::ostream & os) const {
    os << "digraph G {\n"
    "   graph [rankdir=LR];\n"
    "   label=\"     Type\\n     Hierarchy\\n     " <<
    currDateString() << "\";\n";
    os << "   fontsize=24;\n"
    "   fontname=Helvetica;\n"
    "   color=white;\n"
    "   node [shape=Mrecord, /*style=bold, */ fontsize=10, width=2.4, height=.28];\n";
    Type top = iv_crCAS.getTypeSystem().getTopType();
    writeType(os, top, cuiINDENT);
    os << "}" << endl;
  }

} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */



