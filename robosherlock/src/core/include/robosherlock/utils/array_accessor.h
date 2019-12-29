/*
 * Copyright (c) 2012, Christian Kerl <christian.kerl@in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARRAY_ACCESSOR_H_
#define ARRAY_ACCESSOR_H_

#include <uima/api.hpp>

#include <robosherlock/utils/accessor.h>

namespace rs
{
class FeatureStructureProxy;
}

namespace rs
{
namespace accessor
{

/**
 * Implementation of ArrayFSTrait. Template specializations of ArrayFSTrait should inherit
 * from this structure and define appropriate template arguments.
 */
template<typename ArrayFST, typename ArrayFSElementType, size_t FactorT, bool OptimizeT>
struct ArrayFSTraitImpl
{
  /**
   * subclass of uima::BasicArrayFS
   */
  typedef ArrayFST ArrayFSType;

  /**
   * type of the array elements
   */
  typedef ArrayFSElementType ArrayElementType;

  /**
   * specifies the size in bytes of elements in the array, if it has to be mapped to 'char'
   * and stored as binary blob
   */
  static const size_t Factor = FactorT;

  /**
   * true, if optimized uima::BasicArrayFS::copyFromArray and copyToArray can be used with
   * the array element type, false otherwise
   */
  static const bool Optimize = OptimizeT;
};

/**
 * ArrayFSTrait provides information about the uima::BasicArrayFS subclass, which can store
 * elements of type ArrayT.
 */
template<typename ArrayT, class Enable = void>
struct ArrayFSTrait
{
};

/**
 * ArrayFSTrait definitions for bool, char, int, short, long, float, double, std::string, FeatureStructure
 *
 * For these types special uima::BasicArrayFS subclasses exist
 *
 * Whether optimization can be turned on has to be checked in the UIMA source code.
 * For some primitive types copyToArray and copyFromArray are not implemented!
 */

template<>
struct ArrayFSTrait<bool, void> : public ArrayFSTraitImpl<uima::BooleanArrayFS, bool, 1, false>
{ };

template<>
struct ArrayFSTrait<char, void> : public ArrayFSTraitImpl<uima::ByteArrayFS, char, 1, true>
{ };

template<>
struct ArrayFSTrait<int, void> : public ArrayFSTraitImpl<uima::IntArrayFS, int, 1, true>
{ };

template<>
struct ArrayFSTrait<short, void> : public ArrayFSTraitImpl<uima::ShortArrayFS, short, 1, true>
{ };

template<>
struct ArrayFSTrait<INT64, void> : public ArrayFSTraitImpl<uima::LongArrayFS, INT64, 1, true>
{ };

template<>
struct ArrayFSTrait<float, void> : public ArrayFSTraitImpl<uima::FloatArrayFS, float, 1, false>
{ };

template<>
struct ArrayFSTrait<double, void> : public ArrayFSTraitImpl<uima::DoubleArrayFS, double, 1, false>
{ };

template<>
struct ArrayFSTrait<std::string, void> : public ArrayFSTraitImpl<uima::StringArrayFS, uima::UnicodeStringRef, 1, false>
{ };

template<>
struct ArrayFSTrait<uima::FeatureStructure, void> : public ArrayFSTraitImpl<uima::ArrayFS, uima::FeatureStructure, 1, false>
{ };

/**
 * ArrayFSTrait for types subclassing rs::FeatureStructureProxy, which are internally stored
 * as uima::FeatureStructure in an uima::ArrayFS
 */
template<class ArrayT>
struct ArrayFSTrait<ArrayT, typename std::enable_if<std::is_base_of<rs::FeatureStructureProxy, ArrayT>::value >::type > : public ArrayFSTraitImpl<uima::ArrayFS, uima::FeatureStructure, 1, false>
{
};

/**
 * ArrayFSTrait for all none primitive types not subclassing rs::FeatureStructureProxy, e.g. pcl::PointXYZ,
 * which have to be stored in a uima::ByteArrayFS as binary blob
 */
template<typename ArrayT>
struct ArrayFSTrait<ArrayT, typename std::enable_if<!std::is_base_of<rs::FeatureStructureProxy, ArrayT>::value >::type > : public ArrayFSTraitImpl < uima::ByteArrayFS, char, sizeof(ArrayT) / sizeof(char), true >
{
};

/**
 * ArrayFSHelper1 provides methods to get correctly typed uima::*ArrayFS objects
 * from a uima::FeatureStructure and to create new objects.
 */
template <typename ArrayT>
struct ArrayFSHelper1
{
  typedef ArrayFSTrait<ArrayT> trait;

  typename trait::ArrayFSType get(const uima::FeatureStructure &fs, const uima::Feature &f)
  {
    return typename trait::ArrayFSType(fs.getFSValue(f));
  }

  typename trait::ArrayFSType create(uima::FeatureStructure &fs, size_t size) const
  {
    return trait::ArrayFSType::createArrayFS(fs.getCAS(), size, true);
  }
};

/**
 * ArrayFSHelper2 provides methods to work with uima::*ArrayFS objects. There are
 * two specializations for arrays supporting optimized access and ones without this support.
 * The selection should be made depending on the value of ArrayFSTrait<ArrayT>::Optimize.
 *
 * Checking of the array bounds has to be done by the caller. Otherwise it is done by UIMA,
 * causing UIMA specific exceptions in case of failure.
 */
template<typename ArrayT, typename ArrayTAllocator, bool optimize>
struct ArrayFSHelper2
{
};

/**
 * Optimized ArrayFSHelper2 specialization.
 */
template<typename ArrayT, typename ArrayTAllocator>
struct ArrayFSHelper2<ArrayT, ArrayTAllocator, true>
{
  typedef ArrayFSTrait<ArrayT> trait;

  /**
   * copy the array contents to a std::vector
   */
  std::vector<ArrayT, ArrayTAllocator> get(typename trait::ArrayFSType fs)
  {
    std::vector<ArrayT, ArrayTAllocator> result(fs.size() / trait::Factor);

    fs.copyToArray(0, fs.size(), (typename trait::ArrayElementType *)result.data(), 0);

    return result;
  }

  /**
   * get an element by index
   */
  ArrayT get(typename trait::ArrayFSType fs, size_t idx)
  {
    ArrayT result;
    fs.copyToArray(idx * trait::Factor, (idx + 1) * trait::Factor, (typename trait::ArrayElementType *)(&result), 0);

    return result;
  }

  /**
   * copy the contents of the std::vector to the uima::*ArrayFS
   */
  void set(typename trait::ArrayFSType fs, const std::vector<ArrayT, ArrayTAllocator> &value)
  {
    fs.copyFromArray((typename trait::ArrayElementType *)value.data(), 0, fs.size(), 0);
  }

  /**
   * set an element by index
   */
  void set(typename trait::ArrayFSType fs, size_t idx, const ArrayT &value)
  {
    fs.copyFromArray((typename trait::ArrayElementType *)(&value), 0, trait::Factor, idx * trait::Factor);
  }
};
/**
 * Unoptimized ArrayFSHelper2 specialization.
 */
template<typename ArrayT, typename ArrayTAllocator>
struct ArrayFSHelper2<ArrayT, ArrayTAllocator, false>
{
  typedef ArrayFSTrait<ArrayT> trait;

  /**
   * copy the array contents to a std::vector
   */
  std::vector<ArrayT, ArrayTAllocator> get(typename trait::ArrayFSType fs)
  {
    std::vector<ArrayT, ArrayTAllocator> result;

    for(size_t idx = 0; idx < fs.size(); idx++)
    {
      typename trait::ArrayElementType tmp = fs.get(idx);

      result.push_back(convert<typename trait::ArrayElementType, ArrayT>(tmp));
    }

    return result;
  }

  /**
   * get an element by index
   */
  ArrayT get(typename trait::ArrayFSType fs, size_t idx)
  {
    typename trait::ArrayElementType tmp = fs.get(idx);

    return convert<typename trait::ArrayElementType, ArrayT>(tmp);
  }

  /**
   * copy the contents of the std::vector to the uima::*ArrayFS
   */
  void set(typename trait::ArrayFSType fs, const std::vector<ArrayT, ArrayTAllocator> &value)
  {
    for(size_t idx = 0; idx < fs.size(); idx++)
    {
      fs.set(idx, convert<ArrayT, typename trait::ArrayElementType>(value[idx]));
    }
  }

  /**
   * set an element by index
   */
  void set(typename trait::ArrayFSType fs, size_t idx, const ArrayT &value)
  {
    fs.set(idx, convert<ArrayT, typename trait::ArrayElementType>(value));
  }
};
/**
 * Unoptimized ArrayFSHelper2 specialization.
 */
template<>
struct ArrayFSHelper2<std::string, std::allocator<std::string>, false>
{
  std::vector<std::string> get(uima::StringArrayFS fs)
  {
    std::vector<std::string> result;

    for(size_t idx = 0; idx < fs.size(); idx++)
    {
      uima::UnicodeStringRef tmp = fs.get(idx);

      result.push_back(convert<uima::UnicodeStringRef, std::string>(tmp));
    }

    return result;
  }

  /**
   * get an element by index
   */
  std::string get(uima::StringArrayFS fs, size_t idx)
  {
    uima::UnicodeStringRef tmp = fs.get(idx);

    return convert<uima::UnicodeStringRef, std::string>(tmp);
  }

  /**
  * copy the contents of the std::vector to the uima::*ArrayFS
  */
  void set(uima::StringArrayFS fs, const std::vector<std::string> &value)
  {
    for(size_t idx = 0; idx < fs.size(); idx++)
    {
      UnicodeString ustr = UnicodeString::fromUTF8(value[idx]);
      fs.set(idx, ustr);
    }
  }

  /**
  * set an element by index
  */
  void set(uima::StringArrayFS fs, size_t idx, const std::string &value)
  {
    UnicodeString ustr = UnicodeString::fromUTF8(value);
    fs.set(idx, ustr);
  }
};

} // namespace
} // namespace

#endif /* ARRAY_ACCESSOR_H_ */
