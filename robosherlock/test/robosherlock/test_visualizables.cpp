#include <string>
#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/io/Visualizable.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>

class TestVisualizable : public Visualizable
{
public:
  TestVisualizable(const std::string& name) : Visualizable(name)
  {
    outInfo("Instantiation of TestVisualizable");
  }
};

class VisualizablesTest : public testing::Test
{
protected:


  VisualizablesTest()
  {

  }
  virtual void SetUp()
  {
    Visualizable::clearVisualizableList();
  }

  virtual void TearDown()
  {

  }
};

TEST_F(VisualizablesTest, Instantiation)
{
  TestVisualizable tv1("Visualizable1");
  TestVisualizable tv2("Visualizable2");

  std::map<std::string, Visualizable*> mapOfVisualizables;
  Visualizable::copyVisualizableList(mapOfVisualizables);
  outInfo("Size of map: " << mapOfVisualizables.size());
  EXPECT_TRUE(mapOfVisualizables.size() == 2);

  Visualizable::clearVisualizableList();
  Visualizable::copyVisualizableList(mapOfVisualizables);
  EXPECT_TRUE(mapOfVisualizables.size() == 0);
}

