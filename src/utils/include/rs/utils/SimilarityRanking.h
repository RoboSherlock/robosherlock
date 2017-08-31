/**
 * Copyright 2017 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SIMILARITY_RANKING_H__
#define __SIMILARITY_RANKING_H__

#include <algorithm>
#include <iterator>
#include <set>
#include <type_traits>
#include <map>
#include <mutex>
#include <vector>


/// \class RankingItem<ClassId, SampleId> SimilarityRanking.h
template <typename ClassId, typename SampleId>
class RankingItem {
  static_assert(std::is_scalar<SampleId>::value,
                "`SampleId` is not a scalar type");

  /// \brief Type of item's class identifier
  public: using class_id_t = ClassId;

  /// \brief Type of item's sample number identifier
  public: using sample_id_t = SampleId;

  /// \brief Constructor
  /// \param[in] cId  Item's class identifier
  /// \param[in] sId  Item's sample identifier
  /// \param[in] score Score of this item
  public: RankingItem(const ClassId cId, const SampleId sId,
                      const double score):
      classId(cId), sampleId(sId), score(score)
  {
  }

  /// \brief Accessor for class id
  /// \return Class identifier
  public: ClassId getClass() const noexcept
  {
    return this->classId;
  }

  /// \brief Accessor for sample id
  /// \return Sample identifier
  public: SampleId getSampleId() const noexcept
  {
    return this->sampleId;
  }

  /// \brief Accessor for item's score
  /// \return Item's score
  public: double getScore() const noexcept
  {
    return this->score;
  }

  /// \brief Mutator for item's score
  /// \param[in] value New score
  public: void setScore(double value) noexcept
  {
    this->score = value;
  }

  /// \brief Item's class identifier
  private: ClassId classId;

  /// \brief Item's sample identifier
  private: SampleId sampleId;

  /// \brief Score of this item
  private: double score;
};

template <typename RankingItem_t>
class SimilarityRanking {
  // TODO: check if it is correct type
  // static_assert(std::is_base_of<::RankingItem, RankingItem_t>::value,
  //     "`RankingItem_t` is not an instance of RankingItem<Cid, Sid>");

  /// \brief Type of item's class identifier
  public: using ClassId = typename RankingItem_t::class_id_t;

  /// \brief Type of item's sample number identifier
  public: using SampleId = typename RankingItem_t::sample_id_t;

  /// \brief Type of container to store items
  public: using Container_t = std::map<ClassId, std::vector<RankingItem_t>>;

  /// \brief Get number of stored items
  /// \return Number of stored items
  public: size_t size() noexcept
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);
    size_t count = 0;

    for (auto &class_i : this->items)
      count += class_i.second.size();

    return count;
  }

  /// \brief Add new elemnt to the ranking
  /// \param[in] newItem A new ranking item to add
  ///   Items of different class are stored in buckets, ordered by sample id
  public: void addElement(const RankingItem_t &newItem)
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    auto it = this->items.find(newItem.getClass());
    if (it != this->items.end())
      it->second.push_back(newItem);
    else {
      this->items.emplace(newItem.getClass(),
                          std::vector<RankingItem_t>{newItem});
    }
  }

  /// \brief Get n items of the highest score
  /// \param[in] n=1 Number of items to get
  /// \return        Vector of ranking items
  ///   Will return all the items it has if `n` is larger than
  ///   the number of items contained
  public: std::vector<RankingItem_t> getTop(const size_t n = 1)
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    // put all classes in a heap and get a top (hacky, but will work for now)
    std::set<RankingItem_t, LessScoreCmp> heap;

    for (auto &class_i : this->items)
      for (auto &ri : class_i.second)
        heap.insert(ri);

    auto num = std::min(n, heap.size());
    std::vector<RankingItem_t> result(heap.rbegin(),
                                      std::next(heap.rbegin(), num));

    return result;
  }

  /// \brief Reject all items which are not local maxima in a neighborhood of given radius
  /// \param[in] radius Half of the neighborhood size to expect
  ///   For each item, neighboring sample id's are expected, if for current
  ///   item there exist other item with higher score, the item is rejected
  public: void supressNonMaximum(const SampleId radius)
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    for (auto &class_i : this->items)
    {
      auto &rankItems = class_i.second;

      auto isNotLocalMaximum = [&radius, &rankItems] (const RankingItem_t &item)
      {
        bool isMaximum = true;
        auto offset = &item - &*rankItems.cbegin();
        auto it = std::next(rankItems.begin(), offset);

        auto begin = std::max(rankItems.begin(), std::prev(it, radius));
        auto end = std::min(rankItems.end(), std::next(it, radius+1));
        for (auto jt = begin; jt != end; jt = std::next(jt))
          isMaximum &= (jt->getScore() <= it->getScore());

        return !isMaximum;
      };

      rankItems.erase(
        std::remove_if(rankItems.begin(), rankItems.end(), isNotLocalMaximum),
        rankItems.end());
      // non-maximum supression should't remove all items from some class,
      // so no empty classes cleanup needed
    }
  }

  /// \brief Finds the maximal score among stored items
  /// \return Maximal score
  public: double getMaxScore()
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    double max_score = std::accumulate(
        this->begin(), this->end(),
        std::numeric_limits<double>::lowest(),
        [](const double acc, const RankingItem_t &ri) {
          return std::max(ri.getScore(), acc);
        });

    return max_score;
  }

  /// \brief Bring all items to common scale - [0, 1] range
  /// \return Maximal score
  ///   It's assumed that scores are non-negative
  public: double normalize()
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    auto max_score = this->getMaxScore();

    for (auto &class_i : this->items)
      for (auto &ri : class_i.second)
        ri.setScore(ri.getScore() / max_score);

    return max_score;
  }

  /// \brief Reject all items which score is lower than given threshold
  /// \param[in] minLevel Threshold value
  public: void filter(const double minLevel)
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    for (auto &class_i : this->items)
    {
      auto &rankItems = class_i.second;
      
      auto isItemBad = [&minLevel] (const RankingItem_t &item)
      {
        return (item.getScore() < minLevel);
      };

      rankItems.erase(std::remove_if(rankItems.begin(), rankItems.end(),
                                     isItemBad), rankItems.end());
    }

    this->removeEmptyClasses();
  }

  /// \brief Create a table of all stored items
  /// \return Vector of items' scores
  public: std::vector<double> getHistogram()
  {
    std::lock_guard<std::mutex> lock(this->itemsLock);

    std::vector<double> result;
    for (auto sampleIt = this->begin(); sampleIt != this->end(); ++sampleIt)
      result.push_back(sampleIt->getScore());

    return result;
  }

  /// \class SimilarityRanking::Iterator SimilarityRanking.h
  /// \brief Forward Iterator for SimilarityRanking items
  public: class Iterator : public std::iterator<std::forward_iterator_tag,
                                                RankingItem_t>
  {
    /// \brief An iterator on class buckests
    public: typename Container_t::iterator classIt;

    /// \brief An iterator pointing to the end of the class buckets container
    public: typename Container_t::iterator classItEnd;

    /// \brief An iterator on the samples inside class bucket
    public: typename Container_t::mapped_type::iterator sampleIt;

    /// \brief Constructor
    public: Iterator() = default;

    /// \brief Constructor
    /// \param[in] cIt    An iterator on class buckests
    /// \param[in] cItEnd An iterator pointing to the end of the class buckets container
    /// \param[in] sIt    An iterator on the samples inside class bucket
    public: Iterator(decltype(classIt) cIt,
                     decltype(classIt) cItEnd,
                     decltype(sampleIt) sIt):
        classIt(cIt), classItEnd(cItEnd), sampleIt(sIt)
    {
    }

    /// \brief Iterator post-increment
    /// \return Reference to iterator pointing to current element
    public: Iterator &operator ++ ()
    {
      if (this->sampleIt != std::prev(this->classIt->second.end()))
        ++(this->sampleIt);
      else {
        if (this->classIt != std::prev(this->classItEnd))
          this->sampleIt = (++this->classIt)->second.begin();
        else
          ++(this->sampleIt); // go for end()
      }

      return *this;
    }

    /// \brief Iterator pre-increment
    /// \return A new iterator pointing to the next element in collection
    public: Iterator operator ++ (int)
    {
      Iterator retval = *this;
      this->operator ++ ();

      return retval;
    }

    /// \brief Equality operator
    /// \param[in] it Another iterator to compare to
    /// \return       True if iterators point to the same element in collection
    public: bool operator == (Iterator it) const
    {
      return (this->classIt == it.classIt && this->sampleIt == it.sampleIt);
    }

    /// \brief In equality operator
    /// \param[in] it Another iterator to compare to
    /// \return       True if iterators do not point to the same element in collection
    public: bool operator != (Iterator it) const
    {
      return !(*this == it);
    }

    /// \brief Dereference operator
    /// \return A reference to item an iterator points to
    RankingItem_t &operator * () const
    {
      return *sampleIt;
    }

    /// \brief Arrow(Member access) operator
    /// \return A C pointer to an item iterator points to
    RankingItem_t *operator -> () const
    {
      return &*sampleIt;
    }
  };

  /// \brief Get an iterator to the first element in collection
  /// \return An iterator if the collection is nor empty
  public: Iterator begin()
  {
    assert(!this->items.empty());

    return Iterator{this->items.begin(),
                    this->items.end(),
                    this->items.begin()->second.begin()};
  }

  /// \brief Get an iterator to the element after the last one in collection
  /// \return An iterator if the collection is nor empty
  public: Iterator end()
  {
    assert(!this->items.empty());

    Iterator it;
    it.classIt = std::prev(this->items.end());
    it.sampleIt = it.classIt->second.end();

    return it;
  }

  /// \brief Removes buckets of the classes which no longer have samples
  protected: void removeEmptyClasses()
  {
    for(auto it = this->items.begin(); it != this->items.end();)
    {
      if (it->second.size() == 0)
        it = this->items.erase(it);
      else
        it = std::next(it);
    }
  }

  /// \class SimilarityRanking::LessScoreCmp SimilarityRanking.h
  /// \brief Comparator used to ordder items by score
  private: class LessScoreCmp {
    /// \brief Compare scores of the items a and b
    /// \param[in] a LHS item
    /// \param[in] b RHS item
    /// \return      True if a's score is less than b's score
    public: bool operator()(const RankingItem_t &a,
                            const RankingItem_t &b) const
    {
      return (a.getScore() < b.getScore());
    }
  };

  /// \brief Mutex to keep container consistent during multithreaded access
  private: std::mutex itemsLock;

  /// \brief Stored ranking items
  private: Container_t items;
};

#endif /*__SIMILARITY_RANKING_H__*/
