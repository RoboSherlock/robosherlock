#include <string>
#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/CASConsumerContext.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>

#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>
#include "../main.h"

class CASConsumerContextTest : public testing::Test
{
protected:
    CASConsumerContextTest()
  {
  }
  virtual void SetUp()
  {
    // Do something
  }

  virtual void TearDown()
  {
      rs::CASConsumerContext::getInstance().clearCASes();
  }
};

TEST_F(CASConsumerContextTest, AddingAndRemoving)
{
    uima::CAS* test_cas = nullptr;
    uima::CAS* second_cas = nullptr;
    uima::CAS* chird_cas = nullptr;

    rs::CASConsumerContext::getInstance().addCAS("testcas", test_cas);
    rs::CASConsumerContext::getInstance().addCAS("chirdcas", chird_cas);
    rs::CASConsumerContext::getInstance().addCAS("secondcas", second_cas);

    std::vector<std::string> list_of_cas_identifiers = rs::CASConsumerContext::getInstance().getCASIdentifiers();

    EXPECT_EQ(3, list_of_cas_identifiers.size());
    // getCASIdentifiers returns an alphabetically ordered list
    EXPECT_STREQ("chirdcas", list_of_cas_identifiers[0].c_str() );
    EXPECT_STREQ("secondcas", list_of_cas_identifiers[1].c_str() );
    EXPECT_STREQ("testcas", list_of_cas_identifiers[2].c_str() );

    rs::CASConsumerContext::getInstance().removeCAS("secondcas");

    std::vector<std::string> second_list_of_cas_identifiers = rs::CASConsumerContext::getInstance().getCASIdentifiers();
    EXPECT_EQ(2, second_list_of_cas_identifiers.size());
    EXPECT_STREQ("chirdcas", second_list_of_cas_identifiers[0].c_str() );
    EXPECT_STREQ("testcas", second_list_of_cas_identifiers[1].c_str() );
}

TEST_F(CASConsumerContextTest, clearCASes)
{
    uima::CAS* test_cas = nullptr;
    uima::CAS* second_cas = nullptr;
    uima::CAS* chird_cas = nullptr;

    rs::CASConsumerContext::getInstance().addCAS("testcas", test_cas);
    rs::CASConsumerContext::getInstance().addCAS("chirdcas", chird_cas);
    rs::CASConsumerContext::getInstance().addCAS("secondcas", second_cas);

    std::vector<std::string> list_of_cas_identifiers = rs::CASConsumerContext::getInstance().getCASIdentifiers();

    EXPECT_EQ(3, list_of_cas_identifiers.size());

    rs::CASConsumerContext::getInstance().clearCASes();

    std::vector<std::string> second_list_of_cas_identifiers = rs::CASConsumerContext::getInstance().getCASIdentifiers();
    EXPECT_EQ(0, second_list_of_cas_identifiers.size());
}

TEST_F(CASConsumerContextTest, getCASes)
{
    uima::CAS* test_cas = cas;
    uima::CAS* second_cas = cas;

    rs::CASConsumerContext::getInstance().addCAS("testcas", test_cas);
    rs::CASConsumerContext::getInstance().addCAS("secondcas", second_cas);

    EXPECT_EQ(cas, rs::CASConsumerContext::getInstance().getCAS("testcas"));
    EXPECT_EQ(cas, rs::CASConsumerContext::getInstance().getCAS("secondcas"));
    EXPECT_EQ(nullptr, rs::CASConsumerContext::getInstance().getCAS("notexisting"));
}
