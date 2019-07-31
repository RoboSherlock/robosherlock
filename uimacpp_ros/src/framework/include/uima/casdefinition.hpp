#ifndef UIMA_CASDEFINITION_HPP
#define UIMA_CASDEFINITION_HPP

/** \file casdefinition.hpp .
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

   \brief Class holding type system and index definitions

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <uima/err_ids.h>
#include <uima/taespecifier.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class AnalysisEngineDescription;
  class AnnotatorContext;
  class AnalysisEngineMetaData;
  class TypeSystem;
  class TypeSystemDescription;
  class TyVecpFSIndexDescriptions;
  class TyVecpTypePriorities;

  namespace lowlevel {
    class IndexDefinition;
    class TypeSystem;
  }
}



/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {
    /**
     * This class contains all information about CASs which live longer than a document,
     * in particular the type system and the index definition.
     * An AnalysisEngine holds a reference to a CASDefinition object which is
     * used when newCAS() is called.
     */
    class UIMA_LINK_IMPORTSPEC CASDefinition {
    protected:
      uima::lowlevel::TypeSystem * iv_typeSystem;
      uima::lowlevel::IndexDefinition * iv_indexDefinition;

      AnnotatorContext const * iv_annotatorContext;

      void mergeTypeSystem(AnalysisEngineDescription const &);
      void createIndexesFromANC(AnnotatorContext const &);
      void addTypePriorities(AnalysisEngineDescription const &);

      virtual void createTypes();
      virtual void createIndexes();

      void createPredefinedCASTypes();
      void createPredefinedCASIndexes();

      void commitTypeSystem();
      void commitIndexDefinition();

      bool bOwnsTypeSystem;
      void createIndexesFromSpecifier(AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & ) ;
      void addTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities const & );
      void mergeTypeSystem(TypeSystemDescription const &,
                           icu::UnicodeString const & );
      void commitTypeSystemOnly();


      CASDefinition(AnnotatorContext const * );
      CASDefinition(AnalysisEngineDescription const &);
      CASDefinition(uima::TypeSystem  &);
      CASDefinition();

    public:
      static CASDefinition * createCASDefinition(AnnotatorContext const &);
      /**
       * Construct a CASDefinition from the TextAnalysisSpecifier
       */
      static CASDefinition * createCASDefinition(AnalysisEngineDescription const &);
      /**
       * Construct a CASDefinition from AnalysisEngineMetaData
       */
      static CASDefinition * createCASDefinition(AnalysisEngineMetaData const &);
      /**
       * Construct a CASDefinition with specified TypeSystem and built in indices.
       */
      static CASDefinition * createCASDefinition(uima::TypeSystem &);
      /**
       * Construct a CASDefinition from the TypeSystem with indices define in the AnalysisEngineMetaData
       */
      static CASDefinition * createCASDefinition(uima::TypeSystem  &,
          AnalysisEngineMetaData const &);

      /**
       * Construct a CASDefinition from the TypeSystem, index and type priority descriptions
       */
      static CASDefinition * createCASDefinition(TypeSystem   &,
          AnalysisEngineMetaData::TyVecpFSIndexDescriptions const   &,
          AnalysisEngineMetaData::TyVecpTypePriorities  const  &);

      /**
       * Construct a CASDefinition from the TypeSystem, index and type priority descriptions
       */
      static CASDefinition * createCASDefinition(TypeSystem   &,
          AnalysisEngineMetaData::TyVecpFSIndexDescriptions const   &);


      /**
       * Construct a TypeSystem object from the type system definition and type priorities
       * defined in the AnalysisEngineMetaData object 
       * Returns a committed typesystem
       */
      static uima::lowlevel::TypeSystem * createTypeSystem(AnalysisEngineMetaData const &);
      /**
       * Construct a TypeSystem from the TypeSystemDescription
       * Return a committed TypeSystem object
       */
      static uima::lowlevel::TypeSystem * createTypeSystem(TypeSystemDescription const &,
          icu::UnicodeString const & );
      /**
       * Construct a TypeSystem object given the typesystem description and type priorities.
       * Returns a committed TypeSystem object.
       */
      static uima::lowlevel::TypeSystem * createTypeSystem(TypeSystemDescription const &,
          AnalysisEngineMetaData::TyVecpTypePriorities const &,
          icu::UnicodeString const & );



      virtual ~CASDefinition();

      void init();
      void commit();

      uima::lowlevel::TypeSystem & getTypeSystem();
      uima::lowlevel::TypeSystem const & getTypeSystem() const;

      uima::lowlevel::IndexDefinition & getIndexDefinition();
      uima::lowlevel::IndexDefinition const & getIndexDefinition() const;
    };


  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

