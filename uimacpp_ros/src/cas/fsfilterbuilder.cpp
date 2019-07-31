/** \file filterbuilder.cpp .
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
#include <uima/fsfilterbuilder.hpp>
#include <uima/fsindex.hpp>
#include <uima/internal_fspromoter.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/*
Here are straightforward implementations of FSFilters.
Nothing exciting.
*/
using namespace std;
namespace uima {
  namespace internal {

    //////////////////////////////////////////////////////
    class NOTFilter : public FSFilter {
    private:
      FSFilter * iv_pFilter;
    public:
      NOTFilter(FSFilter* pFilter)
          : iv_pFilter(pFilter) {
        assert(EXISTS(iv_pFilter));
      }

      bool isFiltered(FeatureStructure const & fs) const {
        assert(EXISTS(iv_pFilter));
        return ! iv_pFilter->isFiltered(fs);
      }

      void deInit() {
        assert(EXISTS(iv_pFilter));
        iv_pFilter->deInit();
        delete iv_pFilter;
      }
    };

    //////////////////////////////////////////////////////
    class BoolBinaryOperationFilter : public FSFilter {
    protected:
      FSFilter * iv_pFilter1;
      FSFilter * iv_pFilter2;

      BoolBinaryOperationFilter(FSFilter * pFilter1, FSFilter * pFilter2)
          : iv_pFilter1(pFilter1),
          iv_pFilter2(pFilter2) {
        assert(EXISTS(iv_pFilter1));
        assert(EXISTS(iv_pFilter2));
      }
    public:
      void deInit() {
        assert(EXISTS(iv_pFilter1));
        iv_pFilter1->deInit();
        delete iv_pFilter1;
        assert(EXISTS(iv_pFilter2));
        iv_pFilter2->deInit();
        delete iv_pFilter2;
      }
    };

    //////////////////////////////////////////////////////
    class ANDFilter : public BoolBinaryOperationFilter {
    public:
      ANDFilter(FSFilter * pFilter1, FSFilter * pFilter2)
          : BoolBinaryOperationFilter(pFilter1, pFilter2) {}

      bool isFiltered(FeatureStructure const & fs) const {
        assert(EXISTS(iv_pFilter1));
        assert(EXISTS(iv_pFilter2));
        // ensure that filter 2 is not used when filter 1 already returns false
        bool bResult1 = iv_pFilter1->isFiltered(fs);
        if (!bResult1) {
          return false;
        }
        //            cerr << __FILE__ << ", " << __LINE__ << ": value of filter1: " << iv_pFilter1->isFiltered(fs) << ", value of filter2: " << iv_pFilter2->isFiltered(fs) << endl;
        return iv_pFilter2->isFiltered(fs);
      }
    };

    //////////////////////////////////////////////////////
    class ORFilter : public BoolBinaryOperationFilter {
    public:
      ORFilter(FSFilter * pFilter1, FSFilter * pFilter2)
          : BoolBinaryOperationFilter(pFilter1, pFilter2) {}

      bool isFiltered(FeatureStructure const & fs) const {
        assert(EXISTS(iv_pFilter1));
        assert(EXISTS(iv_pFilter2));
        bool bResult1 = iv_pFilter1->isFiltered(fs);
        if (bResult1) {
          return true;
        }
        return iv_pFilter2->isFiltered(fs);
      }
    };

    //////////////////////////////////////////////////////
    class TypeFilter : public FSFilter {
    private:
      Type iv_type;
      bool iv_bUseSusumption;
    public:
      TypeFilter(Type const & crType, bool bUseSubsumption)
          : iv_type(crType),
          iv_bUseSusumption(bUseSubsumption) {}

      bool isFiltered(FeatureStructure const & fs) const {
        if (iv_bUseSusumption) {
          return iv_type.subsumes(fs.getType());
        }
        return iv_type == fs.getType();
      }

    };

    //////////////////////////////////////////////////////
    FeatureStructure getPath(FeatureStructure const & crFS, vector<Feature> const & crFeatures, size_t uiStep) {
      FeatureStructure result = crFS;
      size_t i;
      for (i=0; i<uiStep; ++i) {
        result = result.getFSValue(crFeatures[i]);
      }
      return result;
    }

    //////////////////////////////////////////////////////
    template<class T>
    class BuiltinFeatureValueFilter : public FSFilter {
    private:
      vector<Feature> iv_features;
      FSFilterBuilder::EnComparisonOperator iv_enOp;
      T iv_val;

      bool compare(FSFilterBuilder::EnComparisonOperator enOp, T val1, T val2) const {
        switch (enOp) {
        case FSFilterBuilder::EQUALS:
          return val1 == val2;
        case FSFilterBuilder::GREATER:
          return val1 > val2;
        case FSFilterBuilder::LESS:
          return val1 < val2;
        };
        assert(false);
        return false;
      }

    protected:
      virtual T getFeature(FeatureStructure const &, Feature const &) const = 0;
    public:
      BuiltinFeatureValueFilter(vector<Feature> const & crFeatures, FSFilterBuilder::EnComparisonOperator enOp, T val)
          : iv_features(crFeatures),
          iv_enOp(enOp),
        iv_val(val) {}

      bool isFiltered(FeatureStructure const & crFS) const {
        size_t uiLast = iv_features.size() - 1;
        FeatureStructure fs = getPath(crFS, iv_features, uiLast);
        T val = getFeature(fs, iv_features[uiLast]);
        return compare(iv_enOp, val, iv_val);
      }

    };

    //////////////////////////////////////////////////////
    class IntFeatureFilter : public BuiltinFeatureValueFilter<int> {
    protected:
      int getFeature(FeatureStructure const & crFS, Feature const & crFeat) const {
        return crFS.getIntValue(crFeat);
      }
    public:
      IntFeatureFilter(vector<Feature> const & crFeatures, FSFilterBuilder::EnComparisonOperator enOp, int val)
          : BuiltinFeatureValueFilter<int>(crFeatures, enOp, val) {}
    };

    //////////////////////////////////////////////////////
    class FloatFeatureFilter : public BuiltinFeatureValueFilter<float> {
    protected:
      float getFeature(FeatureStructure const & crFS, Feature const & crFeat) const {
        return crFS.getFloatValue(crFeat);
      }
    public:
      FloatFeatureFilter(vector<Feature> const & crFeatures, FSFilterBuilder::EnComparisonOperator enOp, float val)
          : BuiltinFeatureValueFilter<float>(crFeatures, enOp, val) {}
    };
    /////////////////////////////////////////////////////////////
    class UnicodeStringRefFeatureFilter : public BuiltinFeatureValueFilter<UnicodeStringRef> {
    protected:
      UnicodeStringRef getFeature(FeatureStructure const & crFS, Feature const & crFeat) const {
        return crFS.getStringValue(crFeat);
      }
    public:
      UnicodeStringRefFeatureFilter(vector<Feature> const & crFeaturePath, UnicodeStringRef const & crUStr)
          : BuiltinFeatureValueFilter<UnicodeStringRef>(crFeaturePath, FSFilterBuilder::EQUALS, crUStr ) {}
    };

    class StringFeatureFilter : public FSFilter {
    private:
      UnicodeStringRefFeatureFilter * iv_pUnicodeStringRefFeatureFilter;
      icu::UnicodeString iv_string;
    public:
      StringFeatureFilter(vector<Feature> const & crFeaturePath, UChar const * cpUTFBuffer, size_t uiLength)
          : iv_pUnicodeStringRefFeatureFilter(NULL) {
        iv_string = icu::UnicodeString( cpUTFBuffer, uiLength );
        UnicodeStringRef up(iv_string.getBuffer(), iv_string.length() );
        iv_pUnicodeStringRefFeatureFilter = new UnicodeStringRefFeatureFilter(crFeaturePath, up);
        assert( EXISTS( iv_pUnicodeStringRefFeatureFilter ) );
      }

      ~StringFeatureFilter() {
        if (iv_pUnicodeStringRefFeatureFilter != NULL) {
          delete iv_pUnicodeStringRefFeatureFilter;
          iv_pUnicodeStringRefFeatureFilter = NULL;
        }
      }

      virtual bool isFiltered(FeatureStructure const &crFS) const {
        assert( EXISTS( iv_pUnicodeStringRefFeatureFilter ) );
        return iv_pUnicodeStringRefFeatureFilter->isFiltered(crFS);
      }

      virtual void deInit() {
        assert( EXISTS( iv_pUnicodeStringRefFeatureFilter ) );
        iv_pUnicodeStringRefFeatureFilter->deInit();
      }

    };


    //////////////////////////////////////////////////////
    class MatchFilter : public FSFilter {
    private:
      vector<Feature> iv_features;
      FSFilter * iv_pFilter;
    public:
      MatchFilter(vector<Feature> const & crFeaturePath, FSFilter * pFilter)
          : iv_features(crFeaturePath),
          iv_pFilter(pFilter) {
        assert(EXISTS(iv_pFilter));
      }

      bool isFiltered(FeatureStructure const & crFS) const {
        FeatureStructure fs = getPath(crFS, iv_features, iv_features.size());
        assert(EXISTS(iv_pFilter));
        return iv_pFilter->isFiltered(fs);
      }

      void deInit() {
        assert(EXISTS(iv_pFilter));
        iv_pFilter->deInit();
        delete iv_pFilter;
      }
    };


  }
} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  FSFilterBuilder::FSFilterBuilder() {}

  FSFilterBuilder::~FSFilterBuilder() {}

  FSFilter * FSFilterBuilder::createNOTFilter(FSFilter * pFilter) const {
    return new internal::NOTFilter(pFilter);
  }

  FSFilter * FSFilterBuilder::createANDFilter(FSFilter * pFilter1, FSFilter * pFilter2) const {
    return new internal::ANDFilter(pFilter1, pFilter2);
  }

  FSFilter * FSFilterBuilder::createORFilter(FSFilter * pFilter1, FSFilter * pFilter2) const {
    return new internal::ORFilter(pFilter1, pFilter2);
  }

  FSFilter * FSFilterBuilder::createTypeFilter(Type const & crType, bool bUseSubsumption) const {
    return new internal::TypeFilter(crType, bUseSubsumption);
  }

  FSFilter * FSFilterBuilder::createIntFeatureFilter(vector<Feature> const & crFeatures, EnComparisonOperator enOp, int iVal) const {
    return new internal::IntFeatureFilter(crFeatures, enOp, iVal);
  }

  FSFilter * FSFilterBuilder::createFloatFeatureFilter(vector<Feature> const & crFeatures, EnComparisonOperator enOp, float fVal) const {
    return new internal::FloatFeatureFilter(crFeatures, enOp, fVal);
  }

  FSFilter * FSFilterBuilder::createMatchFilter(vector<Feature> const & crFeaturePath, FSFilter * pFilter) const {
    return new internal::MatchFilter(crFeaturePath, pFilter);
  }

  FSFilter * FSFilterBuilder::createStringFeatureFilter(vector<Feature> const & crFeaturePath, UChar const * cpUTFBuffer, size_t uiLength) const {
    return new internal::StringFeatureFilter(crFeaturePath, cpUTFBuffer, uiLength);
  }


} // namespace uima

/* ----------------------------------------------------------------------- */



