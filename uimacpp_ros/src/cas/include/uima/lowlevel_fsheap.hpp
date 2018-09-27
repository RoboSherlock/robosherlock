#ifndef UIMA_LOWLEVEL_FSHEAP_HPP
#define UIMA_LOWLEVEL_FSHEAP_HPP

/** \file lowlevel_fsheap.hpp .
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

#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_internal_heap.hpp>

#include <uima/internal_typeshortcuts.hpp>
#include <uima/cas.hpp>
#include <uima/arrayfs.hpp>

#ifndef NDEBUG
#include <iostream>
#endif

#include <vector>

#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class CAS;
  class XMLDumpWriter;
  class LocalSofaDataStream;
  namespace internal {
    class CASSerializer;
    class CASDeserializer;
    class FSPromoter;
  }
  namespace lowlevel {
    class DefaultFSIterator;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


namespace uima {
  namespace lowlevel {

    /**
     * The Feature structure heap;
     * Note: this implementation used to contain temporary and permanent heaps
     *       but the permanent feature has been removed.
     */
    class UIMA_LINK_IMPORTSPEC FSHeap {
      friend class uima::CAS;
      friend class uima::internal::CASSerializer;
      friend class uima::internal::CASDeserializer;
      friend class uima::internal::FSPromoter;
      friend class uima::lowlevel::DefaultFSIterator;
      friend class uima::XCASDeserializerHandler;
	  friend class uima::XmiDeserializerHandler;
      friend class uima::XMLDumpWriter;
      friend class uima::LocalSofaDataStream;
/*  VC++ 8 rejects all these forms of friend declaration
     so method getCArrayFromFS has been made public
#ifdef _MSC_VER
      friend class uima::BasicArrayFS;
#else
      template<class,const uima::lowlevel::TyFSType> friend class uima::BasicArrayFS;
#endif
      // VC++ 7.1 rejects the "correct" declaration with:
      //   C2888: 'uima::BasicArrayFS' : symbol cannot be defined within namespace 'lowlevel'
*/
    public:
      typedef enum {
        PERMANENT,
        TEMPORARY
      }
      EnHeap;

    private:
      typedef internal::Heap<TyHeapCell> TyFSHeap;
      typedef internal::Heap<UChar> TyStringHeap;
      typedef uima::lowlevel::internal::Heap<TyHeapCell> TyStringRefHeap;

      typedef uima::lowlevel::internal::Heap<char> Ty8BitHeap;
      typedef uima::lowlevel::internal::Heap<short> Ty16BitHeap;
      typedef uima::lowlevel::internal::Heap<INT64> Ty64BitHeap;


      TyFSHeap iv_clTemporaryHeap;
      TyStringHeap iv_clTemporaryStringHeap;
      TyStringRefHeap iv_clTemporaryStringRefHeap;
      //support for 8, 32, 64 bit heap
      Ty8BitHeap iv_clTemporary8BitHeap;
      Ty16BitHeap iv_clTemporary16BitHeap;
      Ty64BitHeap iv_clTemporary64BitHeap;


      TypeSystem const & iv_rclTypeSystem;

      // internal helpers to set feature structures on the heap
      //   without type checking
      void setFeatureInternal(TyFS, TyFSFeature, TyFS);
      TyFS getFeatureInternal(TyFS, TyFSFeature) const;

      /**
       * helper function to set the string ref heap entry located
       * at pointerIntoStringRefHeap to point to s.
       */
      void setStringRef(TyFS offsetIntoStringRefHeap,
                        TyFS offsetIntoStringHeap);

      TyFSHeap& getHeap() {
        return iv_clTemporaryHeap;
      }

      TyStringHeap& getStringHeap() {
        return iv_clTemporaryStringHeap;
      }

      TyStringRefHeap & getStringRefHeap() {
        return iv_clTemporaryStringRefHeap;
      }

      //support for 8, 32, 64 bit heap
      Ty8BitHeap & get8BitHeap() {
        return iv_clTemporary8BitHeap;
      }
      Ty16BitHeap & get16BitHeap() {
        return iv_clTemporary16BitHeap;
      }
      Ty64BitHeap & get64BitHeap() {
        return iv_clTemporary64BitHeap;
      }



      FSHeap();
      FSHeap(FSHeap const &);
      FSHeap & operator=(FSHeap const &);

    protected:
      
      char const * get8BitArray(TyFS tyFs) const;

      short const * get16BitArray(TyFS tyFs) const;

      INT64 const * get64BitArray(TyFS tyFs) const;

    public:
      /**
       * Construct a heap with respect to a type system.
       * @param numberOfHeapCells initial heap size
       */
      FSHeap(TypeSystem const & rclTypeSystem,
             size_t uiFSHeapPageSize,
             size_t uiStringHeapPageSize,
             size_t uiStringRefHeapPageSize);

      FSHeap(TypeSystem const & rclTypeSystem,
             size_t uiFSHeapPageSize,
             size_t uiStringHeapPageSize,
             size_t uiStringRefHeapPageSize,
             size_t uiMinHeapPageSize);


      ~FSHeap();

      static TyFS const INVALID_FS;

      /**
       * get the C-style array from an array FS (const version).
       */
      TyHeapCell const * getCArrayFromFS(TyFS) const; //was public


	  /** 
        * get the C-style array from an array FS (non-const version).
        */
      TyHeapCell * getCArrayFromFS(TyFS);

      /**
       * Helper function to provide a unique ID for a feature structure
       *
       */
      TyFS getUniqueID(uima::lowlevel::TyHeapCell const tyFs) const {
        return tyFs;
      }

      /**
       * check if a feature structure (of a non built-in type) actually lives within this heap.
       */
      bool resides(TyFS tyFS) const {
        return iv_clTemporaryHeap.resides(tyFS);
      }

      /**
       * check if a feature structure is valid w.r.t. this heap.
       */
      bool isValid(TyFS tyFS) const {
        // second condition is false if heap was resetted
        return resides(tyFS) && ( tyFS != (TyFS) TypeSystem::INVALID_TYPE );
      }

      TypeSystem const & getTypeSystem() const {
        return iv_rclTypeSystem;
      }

      /**
       * erases all data on the specified heap.
       */
      void reset();

      /**
       * create a feature structure of some type.
       * ("high-end" variant).
       * @param tyFeatureNumber the number of features for this type
       */
      TyFS createFS(TyFSType tyType, TyFeatureOffset tyFeatureNumber) {
        assert( iv_rclTypeSystem.isValidType( tyType ) );
        assert( tyType != uima::internal::gs_tyIntegerType );
        assert( tyType != uima::internal::gs_tyFloatType );
        assert( ! iv_rclTypeSystem.subsumes(uima::internal::gs_tyStringType, tyType) );
        assert( iv_rclTypeSystem.getFeatureNumber(tyType) == tyFeatureNumber );
        TyFSHeap & rtyHeap = getHeap();
        TyFS tyResult = rtyHeap.increaseHeap(tyFeatureNumber + 1);
        rtyHeap.setHeapValue(tyResult,(TyHeapCell) tyType);
        return tyResult;
      }

      /**
       * create a feature structure on the temporary heap where the size of the type is
       * already known.
       */
      TyFS createFSWithSize(TyFSType tyType, TyFeatureOffset tyFeatureNumber) {
        return createFS(tyType, tyFeatureNumber);
      }

      /**
       * create a feature structure of some type.
       */
      TyFS createFS(TyFSType tyType) {
        TyFeatureOffset tyNum = iv_rclTypeSystem.getFeatureNumber(tyType);
        return createFS(tyType, tyNum);
      }

      /**
       * create an array fs with size <code>uiSize</code>
       * on the specified heap.
       */
      TyFS createArrayFS(TyFSType tyType, size_t uiSize) {
        assert( iv_rclTypeSystem.isValidType( tyType ) );
        assert( iv_rclTypeSystem.getFeatureNumber(tyType) == 0 );
        assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, tyType ) );
        TyFSHeap & rtyHeap = getHeap();
        Ty8BitHeap & rty8BitHeap = get8BitHeap();
        Ty16BitHeap & rty16BitHeap = get16BitHeap();
        Ty64BitHeap & rty64BitHeap = get64BitHeap();

        TyFS tyResult;
        if (tyType == uima::internal::gs_tyLongArrayType ||
            tyType == uima::internal::gs_tyDoubleArrayType) {

          tyResult = rtyHeap.increaseHeap(3);
          TyFS ty64BitHeapResult = rty64BitHeap.increaseHeap(uiSize);
          rtyHeap.setHeapValue(tyResult, (TyHeapCell) tyType);
          rtyHeap.setHeapValue(tyResult+1, (TyHeapCell) uiSize);
          rtyHeap.setHeapValue(tyResult+2, (TyHeapCell) ty64BitHeapResult);
        }
        else if (tyType == uima::internal::gs_tyBooleanArrayType ||
                 tyType == uima::internal::gs_tyByteArrayType) {

          tyResult = rtyHeap.increaseHeap(3);
          TyFS ty8BitHeapResult = rty8BitHeap.increaseHeap(uiSize);
          rtyHeap.setHeapValue(tyResult, (TyHeapCell) tyType);
          rtyHeap.setHeapValue(tyResult+1, (TyHeapCell) uiSize);
          rtyHeap.setHeapValue(tyResult+2, (TyHeapCell) ty8BitHeapResult);
        } else if (tyType == uima::internal::gs_tyShortArrayType) {

          tyResult = rtyHeap.increaseHeap(3);
          TyFS ty16BitHeapResult = rty16BitHeap.increaseHeap(uiSize);
          rtyHeap.setHeapValue(tyResult, (TyHeapCell) tyType);
          rtyHeap.setHeapValue(tyResult+1, (TyHeapCell) uiSize);
          rtyHeap.setHeapValue(tyResult+2, (TyHeapCell) ty16BitHeapResult);
        }
        else {
          // a 32bit type. 1 cell for the type, 1 for length, uiSize many for the array data

          tyResult = rtyHeap.increaseHeap(2 + uiSize);
          rtyHeap.setHeapValue(tyResult,(TyHeapCell) tyType);
          assert( sizeof(INT32) <= sizeof(TyHeapCell) );
          rtyHeap.setHeapValue(tyResult+1,(TyHeapCell) uiSize);
        }
        return tyResult;
      }

      /**
       * get the <code>Lstring</code> from a feature structure.
       * Precondition: <code>tyFS</code> must be of type string.
       */
      UnicodeStringRef getFSAsString(TyFS tyFS) const;

      /**
       * returns the type of a feature structure.
       */
      TyFSType getType(TyFS tyFs) const;

      /**
       * method for fast access of feature values.
       * Use this in combination with InternalTypeSystem::getFeatureOffset.
       */
      TyFS getFeatureWithOffset(TyFS tyFs, TyFeatureOffset tyOffset) const;

      /**
       * method for fast setting of feature values.
       * Use this in combination with InternalTypeSystem::getFeatureOffset.
       */
      void setFeatureWithOffset(TyFS tyFs, TyFeatureOffset tyOffset, TyFS tyValue);

      /**
       * store copy of string on the specified heap.
       * @return the reference to the copied string.
       */
      int addString(UnicodeStringRef const & uls) {
        size_t l = uls.length();
        TyStringHeap & rtyStringHeap = getStringHeap();
        int p = rtyStringHeap.increaseHeap(l + 1);
        assert( (int)(2*l) == uls.getSizeInBytes() );
        memcpy(rtyStringHeap.getHeapStart()+p, uls.getBuffer(), uls.getSizeInBytes() );
        return p;
      }

      //store long / double value 64 bit heap

      TyFS addLong(INT64 value) {
        Ty64BitHeap & rty64BitHeap = get64BitHeap();
        TyFS p = rty64BitHeap.increaseHeap(sizeof(INT64) );
        rty64BitHeap.setHeapValue(p, value);
        return p;
      }

      TyFS addDouble(double value) {
        Ty64BitHeap & rty64BitHeap = get64BitHeap();
        TyFS p = rty64BitHeap.increaseHeap(sizeof(INT64) );
        INT64 int64Val;
        memcpy(&int64Val, &value, sizeof(INT64));
        rty64BitHeap.setHeapValue(p, int64Val);
        return p;
      }

      /**
       * return copy of string on the stringHeap.
       * @return the reference to the string.
       */
      UnicodeStringRef getString(int strRef) {
        // convert from logical string offset to absolute offset into refHeap
        if (strRef == 0) {
          return UnicodeStringRef();
        }
        strRef = 1 + 2*(strRef-1);
        return UnicodeStringRef( iv_clTemporaryStringHeap.getHeapStart()+
                                 iv_clTemporaryStringRefHeap.getHeapValue(strRef),
                                 (size_t) iv_clTemporaryStringRefHeap.getHeapValue(strRef+1));
      }

      /**
       * store a copy of a string on the specified heap.
       * @return the reference to the copied string.
       */
      int addString(icu::UnicodeString const & uls) {
        return addString(UnicodeStringRef(uls.getBuffer(), uls.length()));
      }

      /**
       * returns if the feature value tyFeat of feature structure tyFS
       * was already touched, i.e., used in a setFeature() or getFeature() call.
       */
      bool isUntouchedFSValue(TyFS tyFS, TyFSFeature tyFeat) const;

      /**
       * get the value of feature <code>tyFeat</code> on feature structure <code>tyFS</code>.
       */
      TyFS getFSValue(TyFS tyFS, TyFSFeature tyFeat) const;

      /**
       * set the feature <code>tyFeat</code> on feature structure <code>tyFS</code>
       * to value <code>tyValue</code>,
       */
      void setFSValue(TyFS tyFS, TyFSFeature tyFeat, TyFS tyValue);

      /**
       * get the size of an array FS.
       */
      size_t getArraySize(TyFS) const;

      /**
       * gets the start pos of this array in the appropriate heap
      * where values of this array type is stored.
       */
      TyHeapCell getArrayOffset(TyFS) const;


      /*@{*/
      /**
       * @name Conversion Functions. Use only with the set/getFeatureWithOffset methods.
       */

      /**
       * convert an fs into an integer.
       */
      static int getFSAsInt(TyFS tyFs) {
        return(int) tyFs;
      }

      /**
       * convert an fs into a float.
       */
      static float getFSAsFloat(TyFS tyFs) {
        assertWithMsg( sizeof(float) <= sizeof(TyHeapCell), "Port required");
        float f;
        memcpy(&f, &tyFs, 4);
        return f;
      }

      /**
       * convert an fs into a bool.
       */
      static bool getFSAsBoolean(TyFS tyFs) {
        if (tyFs==1)
          return true;
        else return false;
      }

      /**
       * convert an fs into a byte.
       */
      static char getFSAsByte(TyFS tyFs) {
        //assertWithMsg( sizeof(float) <= sizeof(TyHeapCell), "Port required");
        return (char) tyFs;
      }

      /**
       * convert an fs into a short.
       */
      static short getFSAsShort(TyFS tyFs) {
        //assertWithMsg( sizeof(float) <= sizeof(TyHeapCell), "Port required");
        return (short)tyFs;
      }


      inline INT64 getFSAsLong(TyFS tyFS) const {
        if (tyFS == 0) {
          return '\n';
        }
        assert( iv_clTemporary64BitHeap.debugIsValidHeapCell(tyFS) );
        return (INT64) iv_clTemporary64BitHeap.getHeapValue(tyFS);
      }

      inline double getFSAsDouble(TyFS tyFS) const {
        if (tyFS == 0) {
          return '\n';
        }
        assert( iv_clTemporary64BitHeap.debugIsValidHeapCell(tyFS) );
        INT64 int64Val = iv_clTemporary64BitHeap.getHeapValue(tyFS);
        double d;
        memcpy(&d, &int64Val, sizeof(double));
        return d;
      }


      /**
       * convert an integer into an fs.
       */
      static TyFS getAsFS(int i) {
        return(TyFS) i;
      }

      /**
       * convert a float into an fs.
       */
      static TyFS getAsFS(float f) {
        assertWithMsg( sizeof(float) <= sizeof(TyHeapCell), "Port required");
        TyFS tyFs;
        memcpy(&tyFs, &f, 4);
        return tyFs;
      }

      /**
       * convert a byte  into an fs.
       */
      static TyFS getAsFS(char f) {
        assertWithMsg( sizeof(char) <= sizeof(TyHeapCell), "Port required");
        return (TyFS)f;
      }

      /**
       * convert a short  into an fs.
       */
      static TyFS getAsFS(short f) {
        assertWithMsg( sizeof(short) <= sizeof(TyHeapCell), "Port required");
        return (TyFS)f;
      }

      /**
       * convert a byte  into an fs.
       */
      static TyFS getAsFS(bool f) {
        assertWithMsg( sizeof(WORD8) <= sizeof(TyHeapCell), "Port required");
        TyFS tyFs = 0;
        if (f) {
          tyFs=1; //true
        } else {
          tyFs=2; //false
        }
        return tyFs;
      }

      TyFS getLongAsFS(INT64);
      TyFS getDoubleAsFS(double);




      /**
       * convert a stringRef into an offset into the StringRefHeap.
       * This method is non-const since an entry on the string ref heap
       * is created.
       */
      TyFS getStringAsFS(int crStringRef);

      /*@}*/

      /*@{*/
      /**
       * @name Special methods for features with built-in types values.
       */
      void setIntValue(TyFS, TyFSFeature, int);
      void setFloatValue(TyFS, TyFSFeature, float);
      void setStringValue(TyFS, TyFSFeature, int strRef);

      void setByteValue(TyFS, TyFSFeature, char );
      void setShortValue(TyFS, TyFSFeature, short );
      void setBooleanValue(TyFS, TyFSFeature, bool );
      void setLongValue(TyFS, TyFSFeature, INT64 ref);
      void setDoubleValue(TyFS, TyFSFeature, double);

      int getIntValue(TyFS, TyFSFeature) const;
      float getFloatValue(TyFS, TyFSFeature) const;
      UnicodeStringRef getStringValue(TyFS, TyFSFeature) const;

      bool getBooleanValue(TyFS,TyFSFeature) const;
      char getByteValue(TyFS,TyFSFeature) const;
      short getShortValue(TyFS,TyFSFeature) const;
      INT64 getLongValue(TyFS,TyFSFeature) const;
      double getDoubleValue(TyFS,TyFSFeature) const;

      //methods for setting ArrayFS values in the appropriate heap
      void setArrayElement(int val, TyHeapCell offset );
      void setArrayElement(float val, TyHeapCell offset );

      void setArrayElement(char val, TyHeapCell offset );
      void setArrayElement(short val, TyHeapCell offset );
      void setArrayElement(bool val, TyHeapCell offset );
      void setArrayElement(INT64 val, TyHeapCell offset);
      void setArrayElement(double, TyHeapCell offset);

      char getByte( TyHeapCell offset );
      short getShort(TyHeapCell offset );
      bool getBoolean(TyHeapCell offset );
      INT64 getLong(TyHeapCell offset);
      double getDouble(TyHeapCell offset);
	 

	  void copyFromArray(TyHeapCell sourceArray[], size_t srcOffset, TyHeapCell tyCell, size_t destOffset, size_t numelements) {
	    TyHeapCell * ptr = getCArrayFromFS(tyCell);
        if(ptr!=NULL) {
          memcpy(ptr + destOffset, sourceArray + srcOffset, numelements*sizeof(TyHeapCell));
        }
	  }

	  void copyFromArray(char sourceArray[], size_t srcOffset, TyHeapCell tyCell, size_t destOffset, size_t numelements) {
	    char * ptr = const_cast<char*>(get8BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(ptr + destOffset, sourceArray + srcOffset, numelements);
        }
	  }

	  void copyFromArray(short sourceArray[], size_t srcOffset, TyHeapCell tyCell, size_t destOffset, size_t numelements) {
	    short * ptr = const_cast<short*>(get16BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(ptr + destOffset, sourceArray + srcOffset, numelements * sizeof(short));
        }
	  }

	  void copyFromArray(INT64 sourceArray[], size_t srcOffset, TyHeapCell tyCell, size_t destOffset, size_t numelements) {
	    INT64 * ptr = const_cast<INT64*>(get64BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(ptr + destOffset, sourceArray + srcOffset, numelements * sizeof(INT64));
        }
	  }

	  void copyToArray(size_t srcOffset, TyHeapCell tyCell, char destArray[], size_t destOffset, size_t numelements) {
	    char * ptr = const_cast<char*>(get8BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(destArray + destOffset, ptr + srcOffset, numelements);
        }
	  }

	  void copyToArray(size_t srcOffset, TyHeapCell tyCell, TyHeapCell destArray[], size_t destOffset, size_t numelements) {
	    TyHeapCell * ptr = const_cast<TyHeapCell*>(getCArrayFromFS(tyCell));
        if(ptr!=NULL) {
          memcpy(destArray + destOffset, ptr + srcOffset, numelements*sizeof(TyHeapCell) );
        }
	  }

	  void copyToArray(size_t srcOffset, TyHeapCell tyCell, short destArray[], size_t destOffset, size_t numelements) {
	    short * ptr = const_cast<short*>(get16BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(destArray + destOffset, ptr + srcOffset, numelements*sizeof(short) );
        }
	  }

	  void copyToArray(size_t srcOffset, TyHeapCell tyCell, INT64 destArray[], size_t destOffset, size_t numelements) {
	    INT64 * ptr = const_cast<INT64*>(get64BitArray(tyCell));
        if(ptr!=NULL) {
          memcpy(destArray + destOffset, ptr + srcOffset, numelements*sizeof(INT64) );
        }
	  }


      /*@}*/


      /**
       * shallow copy of feature structures.
       * ints, floats, and strings are copied "by-value".
       * If featnum == 0 the types of <code>tyTarget</code> and <code>tySource</code> should be equal.
       * Otherwise, just the first featNum features of source are copied to target
       * (no questions asked).
       */
      void copyFeatures(TyFS tyTarget, TyFS tySource, size_t featNum = 0);

      /**
       * print an FS.
       */
      void printFS(std::ostream&, TyFS) const;

//#ifndef NDEBUG
      // debug methods
      TyHeapCell* getHeapStart() const;
      bool debugIsValidHeapCell(TyHeapCell) const;
      bool debugIsConsistent() const;
      void print(std::ostream&) const;
//#endif

    };

  }
}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {

    inline TyFSType FSHeap::getType(TyFS tyFs) const {
      assert( debugIsValidHeapCell(tyFs) );
      TyFSType tyType = (TyFSType) iv_clTemporaryHeap.getHeapValue(tyFs);
      assert( iv_rclTypeSystem.isValidType(tyType) );
      return tyType;
    }


    inline TyFS FSHeap::getFeatureWithOffset(TyFS tyFs, TyFeatureOffset tyOffset) const {
      assert( debugIsValidHeapCell(tyFs) );
      assert( debugIsValidHeapCell(tyFs + tyOffset) );
      assert( tyOffset > 0 );
      assert( tyOffset <= iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ));
      return(TyFS) iv_clTemporaryHeap.getHeapValue(tyFs + tyOffset);
    }

    inline void FSHeap::setFeatureWithOffset(TyFS tyFs, TyFeatureOffset tyOffset, TyFS tyValue) {
      assert( debugIsValidHeapCell(tyFs) );
      assert( debugIsValidHeapCell(tyFs + tyOffset) );
      assert( tyOffset > 0 );
      assert( tyOffset <= iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ));
      iv_clTemporaryHeap.setHeapValue(tyFs + tyOffset, tyValue);
    }

    inline void FSHeap::setStringRef(TyFS strHeapRef, TyFS strRef) {
      // convert from logical string offset to absolute offset into refHeap
      strHeapRef = 1 + 2*(strHeapRef-1);

      // set offset into StringHeap
      iv_clTemporaryStringRefHeap.setHeapValue(strHeapRef, strRef);

      // set length
      iv_clTemporaryStringRefHeap.setHeapValue(strHeapRef+1,
          u_strlen(iv_clTemporaryStringHeap.getHeapStart()+strRef) );
    }

    inline UnicodeStringRef FSHeap::getFSAsString(TyFS tyFS) const {
      if (tyFS == 0) {
        return UnicodeStringRef();
      }
      assert( iv_clTemporaryStringRefHeap.debugIsValidHeapCell(tyFS) );
      // convert from logical string offset to absolute offset into refHeap
      tyFS = 1 + 2*(tyFS-1);
      return UnicodeStringRef( iv_clTemporaryStringHeap.getHeapStart()+
                               iv_clTemporaryStringRefHeap.getHeapValue(tyFS),
                               (size_t) iv_clTemporaryStringRefHeap.getHeapValue(tyFS+1));
    }

    inline TyFS FSHeap::getStringAsFS(int crStringRef) {
      // increase string ref heap by 2
      TyFS stringRef = iv_clTemporaryStringRefHeap.increaseHeap(2);
      // convert to local string offset
      stringRef = 1 + ((stringRef-1)/2);
      setStringRef(stringRef, crStringRef);
      return stringRef;
    }

    inline TyFS FSHeap::getLongAsFS(INT64 l) {
      return addLong(l);
    }

    inline TyFS FSHeap::getDoubleAsFS(double l) {
      return addDouble(l);
    }

    inline TyFS FSHeap::getFeatureInternal(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.isAppropriateFeature( getType(tyFs), tyFeature ) );
      TyFeatureOffset tyFeatureOffset = iv_rclTypeSystem.getFeatureOffset(tyFeature);
      assert( debugIsValidHeapCell(tyFs + tyFeatureOffset) );
      TyFS tyFsCell = iv_clTemporaryHeap.getHeapValue(tyFs + tyFeatureOffset);
      return tyFsCell;
    }

    inline bool FSHeap::isUntouchedFSValue(TyFS tyFS, TyFSFeature tyFeature) const {
      TyFS tyFsCell = getFeatureInternal(tyFS, tyFeature);
      return tyFsCell == INVALID_FS;
    }


    inline int FSHeap::getIntValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_INTEGER) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsInt(tyResult);
    }

    inline float FSHeap::getFloatValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_FLOAT) );
      assertWithMsg( sizeof(float) <= sizeof(TyHeapCell*), "Port required");
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsFloat(tyResult);
    }

    inline UnicodeStringRef FSHeap::getStringValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyStringType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsString(tyResult);
    }

    inline bool FSHeap::getBooleanValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyBooleanType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsBoolean(tyResult);
    }


    inline char FSHeap::getByteValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyByteType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsByte(tyResult);
    }

    inline short FSHeap::getShortValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyShortType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsShort(tyResult);
    }

    inline INT64 FSHeap::getLongValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyLongType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsLong(tyResult);
    }

    inline double FSHeap::getDoubleValue(TyFS tyFs, TyFSFeature tyFeature) const {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyDoubleType, iv_rclTypeSystem.getRangeType(tyFeature) ) );
      TyFS tyResult = getFeatureInternal(tyFs, tyFeature);
      return getFSAsDouble(tyResult);
    }


    inline TyHeapCell const * FSHeap::getCArrayFromFS(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ) == 0 );
      if ( 0 == iv_clTemporaryHeap.getHeapValue(tyFs + 1)) {
        return NULL;
      }
      return(TyHeapCell const *) iv_clTemporaryHeap.getHeapStart() + tyFs + 2;
    }

    inline TyHeapCell * FSHeap::getCArrayFromFS(TyFS tyFs) {
      FSHeap const * cpConstThis = this;
      return CONST_CAST(TyHeapCell *, cpConstThis->getCArrayFromFS(tyFs) );
    }

    inline char const * FSHeap::get8BitArray(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ) == 0 );
      if ( 0 == iv_clTemporaryHeap.getHeapValue(tyFs + 1)) {
        return NULL;
      }
      return(char const *) this->iv_clTemporary8BitHeap.getHeapStart() + iv_clTemporaryHeap.getHeapValue(tyFs + 2);
    }

    inline short const * FSHeap::get16BitArray(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ) == 0 );
      if ( 0 == iv_clTemporaryHeap.getHeapValue(tyFs + 1)) {
        return NULL;
      }
      return(short const *) this->iv_clTemporary16BitHeap.getHeapStart() + iv_clTemporaryHeap.getHeapValue(tyFs + 2);
    }

    inline INT64 const * FSHeap::get64BitArray(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( iv_rclTypeSystem.getFeatureNumber( getType(tyFs) ) == 0 );
      if ( 0 == iv_clTemporaryHeap.getHeapValue(tyFs + 1)) {
        return NULL;
      }

      return(INT64 const *) this->iv_clTemporary64BitHeap.getHeapStart() + iv_clTemporaryHeap.getHeapValue(tyFs + 2);
    }

    inline size_t FSHeap::getArraySize(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( sizeof(WORD32) <= sizeof(uima::lowlevel::TyFS) );
      return(size_t) iv_clTemporaryHeap.getHeapValue(tyFs + 1);
    }

    inline TyHeapCell FSHeap::getArrayOffset(TyFS tyFs) const {
      assert( isValid(tyFs) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, getType(tyFs) ) );
      assert( sizeof(WORD32) <= sizeof(uima::lowlevel::TyFS) );
      TyFSType typecode = getType(tyFs);
      if (typecode == uima::internal::gs_tyBooleanArrayType ||
          typecode == uima::internal::gs_tyByteArrayType ||
          typecode == uima::internal::gs_tyShortArrayType ||
          typecode == uima::internal::gs_tyLongArrayType ||
          typecode == uima::internal::gs_tyDoubleArrayType)  {
        return(size_t) iv_clTemporaryHeap.getHeapValue(tyFs + 2);
      } else  {
        return tyFs+2;
      }
    }


//////////////////////////////////////////////////////////////////////////


    inline void FSHeap::setFeatureInternal(TyFS tyFs, TyFSFeature tyFeature, TyFS tyValue) {
      assert( debugIsValidHeapCell(tyFs) );
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.isAppropriateFeature( getType(tyFs), tyFeature ) );
      TyFeatureOffset tyFeatureOffset = iv_rclTypeSystem.getFeatureOffset(tyFeature);
      TyHeapCell tyFsCell = tyFs + tyFeatureOffset;
      assert( debugIsValidHeapCell(tyFsCell) );
      iv_clTemporaryHeap.setHeapValue(tyFsCell, tyValue);
    }

    inline void FSHeap::setFSValue(TyFS tyFs, TyFSFeature tyFeature, TyFS tyValue) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( debugIsValidHeapCell(tyFs) );
      assert( resides(tyFs) );
      assert( iv_rclTypeSystem.isAppropriateFeature( getType(tyFs), tyFeature ) );
#ifndef NDEBUG
      if (tyValue != INVALID_FS) {
        assert( debugIsValidHeapCell(tyValue) );
        if (iv_rclTypeSystem.subsumes( uima::internal::gs_tyFSArrayType, iv_rclTypeSystem.getRangeType( tyFeature ) )) {
          assert(resides(tyValue));
        } else if ( !iv_rclTypeSystem.subsumes( uima::internal::gs_tyStringType, iv_rclTypeSystem.getRangeType( tyFeature ) ) ) {
          assert( resides(tyValue) );
          assert( iv_rclTypeSystem.subsumes( iv_rclTypeSystem.getRangeType( tyFeature ), getType(tyValue) ) );
        } else {
          assert( iv_clTemporaryStringRefHeap.resides(tyValue) );
        }
      }
#endif
      setFeatureInternal(tyFs, tyFeature, tyValue);
    }

    inline TyFS FSHeap::getFSValue(TyFS tyFs, TyFSFeature tyFeature) const {
      TyFS tyFsCell = getFeatureInternal(tyFs, tyFeature);
#ifndef NDEBUG
      if (tyFsCell != INVALID_FS) {
        if ( ! iv_rclTypeSystem.subsumes( uima::internal::gs_tyStringType, iv_rclTypeSystem.getRangeType( tyFeature ) ) ) {
          assert( iv_rclTypeSystem.subsumes( iv_rclTypeSystem.getRangeType(tyFeature), getType(tyFsCell) ) );
        }
      }
#endif
      return tyFsCell;
    }

    inline void FSHeap::setIntValue(TyFS tyFs, TyFSFeature tyFeature, int iValue) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_INTEGER) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, (TyFS)iValue );
    }

    inline void FSHeap::setFloatValue(TyFS tyFs, TyFSFeature tyFeature, float fValue) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_FLOAT) );
      assertWithMsg( sizeof(float) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, getAsFS(fValue) );
    }


    inline void FSHeap::setBooleanValue(TyFS tyFs, TyFSFeature tyFeature, bool ref) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_BOOLEAN) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, getAsFS(ref) );
    }


    inline void FSHeap::setByteValue(TyFS tyFs, TyFSFeature tyFeature, char ref) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_BYTE) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, getAsFS(ref) );
    }

    inline void FSHeap::setShortValue(TyFS tyFs, TyFSFeature tyFeature, short ref) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_SHORT) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, getAsFS(ref) );
    }

    inline void FSHeap::setLongValue(TyFS tyFs, TyFSFeature tyFeature, INT64 ref) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_LONG) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, addLong(ref) );
    }

    inline void FSHeap::setDoubleValue(TyFS tyFs, TyFSFeature tyFeature, double ref) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.getTypeName( iv_rclTypeSystem.getRangeType(tyFeature) )  == icu::UnicodeString(CAS::TYPE_NAME_DOUBLE) );
      assertWithMsg( sizeof(int) <= sizeof(TyHeapCell*), "Port required");
      setFeatureInternal(tyFs, tyFeature, addDouble(ref) );
    }


    inline void FSHeap::setArrayElement(int tyValue, TyHeapCell tyFsCell) {
      this->iv_clTemporaryHeap.setHeapValue(tyFsCell, tyValue);
    }
    inline void FSHeap::setArrayElement(float tyValue, TyHeapCell tyFsCell) {
      this->iv_clTemporaryHeap.setHeapValue(tyFsCell, getAsFS(tyValue));
    }

    inline void FSHeap::setArrayElement(bool tyValue, TyHeapCell tyFsCell) {
      iv_clTemporary8BitHeap.setHeapValue(tyFsCell, tyValue);
    }
    inline void FSHeap::setArrayElement(char tyValue, TyHeapCell tyFsCell) {
      iv_clTemporary8BitHeap.setHeapValue(tyFsCell, tyValue);
    }
    inline void FSHeap::setArrayElement(short tyValue, TyHeapCell tyFsCell) {
      iv_clTemporary16BitHeap.setHeapValue(tyFsCell, tyValue);
    }
    inline void FSHeap::setArrayElement(INT64 tyValue, TyHeapCell tyFsCell) {
      iv_clTemporary64BitHeap.setHeapValue(tyFsCell, tyValue);
    }
    inline void FSHeap::setArrayElement(double tyValue, TyHeapCell tyFsCell) {
      INT64 int64Val;
      memcpy(&int64Val, &tyValue, sizeof(INT64));
      iv_clTemporary64BitHeap.setHeapValue(tyFsCell, int64Val);
    }

    inline bool FSHeap::getBoolean(TyHeapCell tyFsCell) {
      char val = iv_clTemporary8BitHeap.getHeapValue(tyFsCell);
      if
      (val==1) return true;
      else
        return false;
    }
    inline char FSHeap::getByte(TyHeapCell tyFsCell) {
      return iv_clTemporary8BitHeap.getHeapValue(tyFsCell);
    }
    inline short FSHeap::getShort(TyHeapCell tyFsCell) {
      return iv_clTemporary16BitHeap.getHeapValue(tyFsCell);
    }
    inline INT64 FSHeap::getLong(TyHeapCell tyFsCell) {
      return iv_clTemporary64BitHeap.getHeapValue(tyFsCell);
    }
    inline double FSHeap::getDouble(TyHeapCell tyFsCell) {
      INT64 int64Val = iv_clTemporary64BitHeap.getHeapValue(tyFsCell);
      double d;
      memcpy(&d, &int64Val, sizeof(double));
      return d;
    }

  }
}


/* ----------------------------------------------------------------------- */

#if defined( _MSC_VER )
#pragma warning( pop )
#endif

#endif


