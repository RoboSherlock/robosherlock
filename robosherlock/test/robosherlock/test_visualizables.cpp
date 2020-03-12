#include <string>
#include <gtest/gtest.h>

#include <robosherlock/utils/common.h>
#include <robosherlock/types/all_types.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/io/Visualizable.h>
#include <robosherlock/io/VisualizableGroupManager.h>

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

TEST_F(VisualizablesTest, SelectingVisualizablesinVGMs)
{
  TestVisualizable tv1("Visualizable1");
  TestVisualizable tv2("Visualizable2");

  rs::VisualizableGroupManager vgm("testvgm");
  vgm.start();
  // Test going back and forth
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable1");
  vgm.nextVisualizable();
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable2");
  vgm.nextVisualizable();
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable1");
  vgm.prevVisualizable();
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable2");
  vgm.prevVisualizable();
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable1");

  // Test to select individual Visualizables
  vgm.selectVisualizable("Visualizable2");
  ASSERT_EQ(vgm.getCurrentVisualizableName(), "Visualizable2");
}
