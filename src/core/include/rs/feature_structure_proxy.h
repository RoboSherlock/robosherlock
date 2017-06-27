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

#ifndef FEATURE_STRUCTURE_PROXY_H_
#define FEATURE_STRUCTURE_PROXY_H_

#include <uima/api.hpp>

#include <rs/utils/accessor.h>
#include <rs/utils/output.h>
#include <rs/utils/array_accessor.h>

// MONGO
#include <mongo/bson/bson.h>

/**
 * TypeTrait allows to access the name of a type as string at runtime.
 */
template<class T>
struct TypeTrait
{
  /* this will cause compile errors for types without TypeTrait specialization

   static const char* name()
   {
   static const char* value = "undefined";

   return value;
   }
   */
};

/**
 * Macro to define a TypeTrait specialization for a type.
 */
#define TYPE_TRAIT(TYPE,NAME) \
  template<> \
  struct TypeTrait<TYPE > \
  { \
    static const char* name() \
    { \
      static const char* value = NAME; \
      \
      return value; \
    } \
  };

namespace rs
{

/**
 * Gets the uima::Type for the type T from the typesystem of the given uima::CAS.
 */
template<typename T>
uima::Type type(uima::CAS &cas)
{
  return cas.getTypeSystem().getType(TypeTrait<T>::name());
}

/**
 * Creates a new instance of T passing a newly created uima::FeatureStructure to the ctor.
 * The type of the uima::FeatureStructure is determined by a call to rs::type<T>(cas).
 */
template<typename T>
T create(uima::CAS &cas)
{
  return T(cas.createFS(type<T>(cas)));
}

/**
 * FeatureStructureProxy provides a thin proxy for a uima::FeatureStructure to more easily
 * work with UIMA types and objects. There should be a subclass of FeatureStructureProxy for
 * every custom UIMA type, which has a field of type *FeatureStructureEntry for every feature
 * defined in the UIMA type.
 */
class FeatureStructureProxy
{
  template<typename T>
  friend class FeatureStructureEntry;
  template<typename T>
  friend class ComplexFeatureStructureEntry;
  template<typename T, typename TAllocator>
  friend class ArrayFeatureStructureEntry;
  template<typename T, typename TAllocator>
  friend class ListFeatureStructureEntry;

private:
  uima::Type type_;
  uima::FeatureStructure fs_;

protected:

  FeatureStructureProxy()
  {

  }

  template<typename T>
  T create()
  {
    return rs::create<T>(this->fs_.getCAS());
  }

  template<typename T>
  uima::Type gettype()
  {
    return rs::type<T>(fs_.getCAS());
  }

public:
  FeatureStructureProxy(uima::FeatureStructure fs)
  {
    type_ = fs.getType();
    fs_ = fs;
  }

  FeatureStructureProxy(const FeatureStructureProxy &other)
  {
    type_ = other.type_;
    fs_ = other.fs_;
  }

  uima::Type type() const
  {
    return type_;
  }

  operator uima::FeatureStructure() const
  {
    return fs_;
  }

  operator uima::FeatureStructure &()
  {
    return fs_;
  }

  operator const uima::FeatureStructure &() const
  {
    return fs_;
  }
};

/**
 * FeatureStructureEntry allows easy access to the value of a UIMA feature having a
 * primitive type. It uses rs::util::Accessor.
 */
template<typename T>
class FeatureStructureEntry
{
protected:
  uima::Feature feature_;
  FeatureStructureProxy *parent_;

  uima::FeatureStructure &fs() const
  {
    return parent_->fs_;
  }

public:
  virtual ~FeatureStructureEntry()
  {
  }

  void init(FeatureStructureProxy *parent, const char *name)
  {
    parent_ = parent;
    feature_ = parent->type_.getFeatureByBaseName(name);
    if(!feature_.isValid())
    {
      outError("Error: invalid feature when initializing! check your typesystem.xml file!" << std::endl
               << "       feature name: " << name);
      if(parent_->fs_.isValid())
      {
        outError("       parent  type name: " << parent_->type_.getName());
      }
      else
      {
        outError("       invalid parent feature structure!");
      }
    }
  }

  virtual T get()
  {
    return accessor::Accessor<T>::get(this->fs(), feature_);
  }

  virtual void set(const T &value)
  {
    accessor::Accessor<T>::set(this->fs(), feature_, value);
  }

  T operator()()
  {
    return get();
  }

  void operator()(const T &value)
  {
    set(value);
  }
};

/**
 * ComplexFeatureStructureEntry allows easy access to the value of a UIMA feature having an
 * other UIMA type. T has to be a subclass of FeatureStructureProxy.
 */
template<typename T>
class ComplexFeatureStructureEntry : public FeatureStructureEntry<T>
{
private:
  uima::FeatureStructure _get() const
  {
    return this->fs().getFSValue(this->feature_);
  }
public:
  virtual ~ComplexFeatureStructureEntry()
  {
  }

  /**
   * populates the feature with a new instance
   */
  virtual void allocate()
  {
    set(create<T>(this->fs().getCAS()));
  }

  /**
   * checks whether the feature has a value or is 'null'
   */
  virtual bool has() const
  {
    return _get().isValid();
  }

  operator bool() const
  {
    return has();
  }

  /*
   virtual T get()
   {
   return get<T>();
   }
   */

  /**
   * gets the value, if it is 'null' it is allocated
   */
  //template <class TT>
  T get()
  {
    uima::FeatureStructure fs = _get();

    // invalid child
    if(!fs.isValid())
    {
      T result = create<T>(fs.getCAS());
      set(result);

      return result;
    }
    else
    {
      return T(fs);
    }
  }

  virtual void set(const T &value)
  {
    this->fs().setFSValue(this->feature_, (uima::FeatureStructure)value);
  }
};

/**
 * ArrayFeatureStructureEntry allows easy access to the value of a UIMA feature of an
 * array type. The element type of the array can be any type (primitive, subclass of FeatureStructureProxy, ...).
 * It uses the rs::util::ArrayFSHelper* classes.
 */
template<typename T, typename TAllocator = std::allocator<T> >
class ArrayFeatureStructureEntry : public FeatureStructureEntry<std::vector<T, TAllocator> >
{
private:
  typedef accessor::ArrayFSTrait<T> trait;

  static accessor::ArrayFSHelper1<T> h1;
  static accessor::ArrayFSHelper2<T, TAllocator, trait::Optimize> h2;

  typename trait::ArrayFSType _get(void) const
  {
    return h1.get(this->fs(), this->feature_);
  }

  typename trait::ArrayFSType _create(size_t size)
  {
    return h1.create(this->fs(), size);
  }
public:
  ArrayFeatureStructureEntry()
  {
  }

  virtual ~ArrayFeatureStructureEntry()
  {
  }

  virtual ArrayFeatureStructureEntry<T, TAllocator> &allocate(size_t size)
  {
    this->fs().setFSValue(this->feature_, _create(size * trait::Factor));

    return *this;
  }

  virtual bool has() const
  {
    return _get().isValid();
  }

  operator bool() const
  {
    return has();
  }

  virtual size_t size()
  {
    return _get().size() / trait::Factor;
  }

  virtual std::vector<T, TAllocator> get()
  {
    return h2.get(_get());
  }

  virtual T get(size_t idx)
  {
    return h2.get(_get(), idx);
  }

  virtual void set(const std::vector<T, TAllocator> &value)
  {

    if(!has() || size() < value.size())
    {
      allocate(value.size());
    }

    h2.set(_get(), value);
  }

  virtual void set(size_t idx, const T &value)
  {
    h2.set(_get(), idx, value);
  }

  template<typename TargetT>
  bool filter(std::vector<TargetT> &result)
  {
    bool success = false;

    if(has())
    {
      typename trait::ArrayFSType arrayfs = _get();
      result.reserve(arrayfs.size());

      uima::Type type = rs::type<TargetT>(this->fs().getCAS());

      for(size_t idx = 0; idx < arrayfs.size(); ++idx)
      {
        uima::FeatureStructure fs = arrayfs.get(idx);

        if(fs.getType() == type)
        {
          success = true;
          TargetT elem(fs);
          result.push_back(elem);
        }
      }
    }

    return success;
  }

  /*
   *    Example code:
   *    std::vector<rs::Cluster> class_clust;
   *    std::vector<std::vector<rs::Classification>> class_anno;
   *
   *    scene.identifiables.filter(class_clust, class_anno);
   *
   *    Explanation:
   *
   *    1.  std::vectorrs::Cluster class_clust;
   *        This is the vector containing all the clusters that are found in the containing list that have the specified kind of annotation. Generally the type used for the result vector can be of any kind, but in order to use the functionality of the function it has to be a type that can contain annotations - otherwise nothing will be returned.
   *
   *    2.  std::vector<std::vectorpercepteros::ToolObject> class_anno;
   *        This is the vector of vectors for all the annotations of the clusters. It has to be a vector of vectors since every cluster can have multiple annotations of the same type. The vector of annotations at index 0 corresponds to the cluster at index 0 and so on. As with the clusters above, the type of the vectors can be anything, but in order to get a result they have to be some type of annotations.
   *
   *    3.  scene.identifiables.filter(class_clust, class_anno);
   *        This is the call, filtering out all of the clusters containing annotations of type Classification and the annotations they have from the list scene.identifiables
   */
  template<typename TargetT, typename AnnotT>
  bool filter(std::vector<TargetT> &result, std::vector<std::vector<AnnotT>> &annots)
  {
    bool success = false;

    if(has())
    {
      typename trait::ArrayFSType arrayfs = _get();
      result.reserve(arrayfs.size());

      uima::Type type = rs::type<TargetT>(this->fs().getCAS());

      for(size_t idx = 0; idx < arrayfs.size(); ++idx)
      {
        uima::FeatureStructure fs = arrayfs.get(idx);

        if(fs.getType() == type)
        {
          std::vector<AnnotT> clustannots;
          TargetT cluster(fs);
          cluster.annotations.filter(clustannots);

          if(clustannots.size() > 0)
          {
            success = true;
            result.push_back(cluster);
            annots.push_back(clustannots);
          }
        }
      }
    }

    return success;
  }
};

template<typename T, typename TAllocator>
accessor::ArrayFSHelper1<T> ArrayFeatureStructureEntry<T, TAllocator>::h1;

template<typename T, typename TAllocator>
accessor::ArrayFSHelper2<T, TAllocator, ArrayFeatureStructureEntry<T, TAllocator>::trait::Optimize> ArrayFeatureStructureEntry <
T, TAllocator >::h2;

/**
 * Implementation for ListFSTrait specializations
 */
template<typename ListT, typename ListElementT>
struct ListFSTraitImpl
{
  /**
   * the uima::*ListFS type
   */
  typedef ListT ListType;

  /**
   * the element type
   */
  typedef ListElementT ListElementType;
};

/**
 * ListFSTrait provides information about the uima::BasicListFS subclass, which can store
 * elements of type T.
 */
template<typename T>
struct ListFSTrait : public ListFSTraitImpl<uima::ListFS, uima::FeatureStructure>
{
};

/**
 * ListFSTrait specializations for int, float and std::string
 */

template<>
struct ListFSTrait<int> : public ListFSTraitImpl<uima::IntListFS, int>
{
};

template<>
struct ListFSTrait<float> : public ListFSTraitImpl<uima::FloatListFS, float>
{
};

template<>
struct ListFSTrait<std::string> : public ListFSTraitImpl<uima::StringListFS, uima::UnicodeStringRef>
{
};

/**
 * std::iterator implementation for uima::*ListFS.
 */
template<typename T>
class ListFSIterator : std::iterator<std::input_iterator_tag, T>
{
private:
  typedef ListFSTrait<T> list_trait;

  typename list_trait::ListType list_;
public:
  ListFSIterator(typename list_trait::ListType list) :
    list_(list)
  {
  }

  ListFSIterator(const ListFSIterator<T> &other) :
    list_(other.list_)
  {
  }

  // postfix, e.g. it++
  ListFSIterator<T> operator++(int)
  {
    ListFSIterator<T> tmp(*this);
    operator ++();
    return tmp;
  }

  // prefix, e.g. ++it
  ListFSIterator<T> &operator++()
  {
    list_ = list_.getTail();
    return *this;
  }

  bool operator==(const ListFSIterator<T> &other)
  {
    return
      // ensures both lists are at the same position (this is the standard case)
      (this->list_ == other.list_) ||
      // if we are invalid or empty or other is invalid or empty, we consider us equal
      (
        // we are invalid or empty
        (!this->list_.isValid() || this->list_.isEmpty()) &&
        // other is invalid or empty
        (!other.list_.isValid() || other.list_.isEmpty()));
  }

  bool operator!=(const ListFSIterator<T> &other)
  {
    return !(this->operator ==(other));
  }

  T operator*()
  {
    return accessor::convert<typename list_trait::ListElementType, T>(list_.getHead());
  }
};

/**
 * ListFeatureStructureEntry allows easy access to the value of a UIMA feature of a
 * list type. The list element type has to be a int, float, string or a subclass of
 * FeatureStructureProxy or uima::FeatureStructure.
 */
template<typename T, typename TAllocator = std::allocator<T> >
class ListFeatureStructureEntry : public FeatureStructureEntry<std::vector<T, TAllocator> >
{
private:
  typedef ListFSTrait<T> trait;

  typename trait::ListType _get(void) const
  {
    return trait::ListType::getListFSValue(this->fs(), this->feature_);
  }

  void _set(typename trait::ListType list) const
  {
    this->fs().setFSValue(this->feature_, list);
  }
public:
  typedef ListFSIterator<T> iterator;

  virtual ~ListFeatureStructureEntry()
  {
  }

  // TODO: is this required?! append()/prepend() seem to work without calling this
  void allocate()
  {
    typename trait::ListType list = trait::ListType::createListFS(this->fs().getCAS(), true);
    _set(list);
  }

  virtual std::vector<T> get()
  {
    std::vector<T> l;

    for(iterator it = begin(); it != end(); it++)
    {
      l.push_back(*it);
    }

    return l;
  }

  virtual void set(const std::vector<T> &value)
  {
    // create new empty list
    allocate();

    for(typename std::vector<T>::const_iterator it = value.begin(); it != value.end(); ++it)
    {
      append(*it);
    }
  }

  iterator begin()
  {
    return iterator(_get());
  }

  iterator end()
  {
    return iterator(typename trait::ListType());
  }

  bool empty() const
  {
    return _get().isEmpty();
  }

  size_t size() const
  {
    return _get().getLength();
  }

  void append(const T &value)
  {
    // workaround weird UIMA behavior :(
    if(empty())
    {
      _set(_get().addLast(accessor::convert<T, typename trait::ListElementType>(value)));
    }
    else
    {
      _get().addLast(accessor::convert<T, typename trait::ListElementType>(value));
    }
  }

  void append(std::vector<T> &values)
  {
    for(int i = 0; i < values.size(); ++i)
    {
      append(values[i]);
    }
  }

  void prepend(const T &value)
  {
    // workaround weird UIMA behavior :(
    _set(_get().addFirst(accessor::convert<T, typename trait::ListElementType>(value)));
  }

  void remove(const T &value)
  {
    // TODO: check if this behaves as stupid as append(...)
    _get().removeElement(accessor::convert<T, typename trait::ListElementType>(value));
  }

  template<typename TargetT>
  bool filter(std::vector<TargetT> &result)
  {
    bool success = false;

    if(!empty())
    {
      typename trait::ListType listfs = _get();

      uima::Type type = rs::type<TargetT>(this->fs().getCAS());

      while(!listfs.isEmpty())
      {
        uima::FeatureStructure fs = listfs.getHead();

        if(fs.getType() == type)
        {
          success = true;
          TargetT elem(fs);
          result.push_back(elem);
        }

        listfs = listfs.getTail();
      }
    }

    return success;
  }

  /*
   *    Example code:
   *    std::vector<rs::Cluster> class_clust;
   *    std::vector<std::vector<rs::Classification>> class_anno;
   *
   *    scene.identifiables.filter(class_clust, class_anno);
   *
   *    Explanation:
   *
   *    1.  std::vectorrs::Cluster class_clust;
   *        This is the vector containing all the clusters that are found in the containing list that have the specified kind of annotation. Generally the type used for the result vector can be of any kind, but in order to use the functionality of the function it has to be a type that can contain annotations - otherwise nothing will be returned.
   *
   *    2.  std::vector<std::vectorpercepteros::ToolObject> class_anno;
   *        This is the vector of vectors for all the annotations of the clusters. It has to be a vector of vectors since every cluster can have multiple annotations of the same type. The vector of annotations at index 0 corresponds to the cluster at index 0 and so on. As with the clusters above, the type of the vectors can be anything, but in order to get a result they have to be some type of annotations.
   *
   *    3.  scene.identifiables.filter(class_clust, class_anno);
   *        This is the call, filtering out all of the clusters containing annotations of type Classification and the annotations they have from the list scene.identifiables
   */
  template<typename TargetT, typename AnnotT>
  bool filter(std::vector<TargetT> &result, std::vector<std::vector<AnnotT>> &annots)
  {
    bool success = false;

    if(!empty())
    {
      typename trait::ListType listfs = _get();

      uima::Type type = rs::type<TargetT>(this->fs().getCAS());

      while(!listfs.isEmpty())
      {
        uima::FeatureStructure fs = listfs.getHead();

        if(fs.getType() == type)
        {
          std::vector<AnnotT> clustannots;
          TargetT cluster(fs);
          cluster.annotations.filter(clustannots);

          if(clustannots.size() > 0)
          {
            success = true;
            result.push_back(cluster);
            annots.push_back(clustannots);
          }
        }

        listfs = listfs.getTail();
      }
    }

    return success;
  }

  template<typename TargetT>
  bool getFirst(TargetT &result)
  {
    bool success = false;

    if(!empty())
    {
      typename trait::ListType listfs = _get();

      uima::Type type = rs::type<TargetT>(this->fs().getCAS());

      while(!listfs.isEmpty())
      {
        uima::FeatureStructure fs = listfs.getHead();

        if(fs.getType() == type)
        {
          success = true;
          result = fs;
          break;
        }
        listfs = listfs.getTail();
      }
    }

    return success;
  }

};

/**
 * ListFeatureStructureEntry allows easy access to the value of a UIMA feature of a
 * list type. The list element type has to be a int, float, string or a subclass of
 * FeatureStructureProxy or uima::FeatureStructure.
 */
template<>
class ListFeatureStructureEntry<std::string, std::allocator<std::string> > : public FeatureStructureEntry<std::vector<std::string> >
{
private:

  uima::StringListFS _get(void) const
  {
    uima::StringListFS list = this->fs().getStringListFSValue(this->feature_);
    if(!list.isValid())
    {
      list = this->fs().getCAS().createStringListFS();
    }
    return list;
  }

  void _set(const uima::StringListFS &list) const
  {
    this->fs().setFSValue(this->feature_, list);
  }

public:
  typedef ListFSIterator<std::string> iterator;

  virtual ~ListFeatureStructureEntry()
  {
  }

  // TODO: is this required?! append()/prepend() seem to work without calling this
  void allocate()
  {
    _get();
  }

  virtual std::vector<std::string> get()
  {
    std::vector<std::string> l;

    for(iterator it = begin(); it != end(); it++)
    {
      l.push_back(*it);
    }

    return l;
  }

  virtual void set(const std::vector<std::string> &value)
  {
    _set(this->fs().getCAS().createStringListFS());
    append(value);
  }

  iterator begin()
  {
    return iterator(_get());
  }

  iterator end()
  {
    return iterator(uima::StringListFS());
  }

  bool empty() const
  {
    return _get().isEmpty();
  }

  size_t size() const
  {
    return _get().getLength();
  }

  void append(const std::string &value)
  {
    uima::StringListFS list = _get();

    UnicodeString ustr = UnicodeString::fromUTF8(value);
    list.addLast(ustr);
    _set(list);
  }

  void append(const std::vector<std::string> &value)
  {
    uima::StringListFS list = _get();
    for(int i = 0; i < value.size(); ++i)
    {
      UnicodeString ustr = UnicodeString::fromUTF8(value[i]);
      list.addLast(ustr);
    }
    _set(list);
  }

  void prepend(const std::string &value)
  {
    uima::StringListFS list = _get();
    UnicodeString ustr = UnicodeString::fromUTF8(value);
    list.addFirst(ustr);
    _set(list);
  }

  void remove(const std::string &value)
  {
    uima::StringListFS list = _get();
    UnicodeString ustr = UnicodeString::fromUTF8(value);
    list.removeElement(ustr);
    _set(list);
  }

  template<typename TargetT>
  bool filter(std::vector<TargetT> &result)
  {
    outError("filter is not implemented for StringListFS! Filtering different types seems to be useless.");
    return false;
  }
};
}

TYPE_TRAIT(rs::FeatureStructureProxy, "uima.cas.TOP")

#endif /* FEATURE_STRUCTURE_PROXY_H_ */
