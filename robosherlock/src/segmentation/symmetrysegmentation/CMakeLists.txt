project(rs_symmetrysegmentation)

add_library(rs_symmetrysegmentation SHARED
  src/OverSegmenter.cpp
  src/RotationalSymmetryExtractor.cpp
  src/RotationalSymmetrySegmenter.cpp
  src/BilateralSymmetryExtractor.cpp
  src/BilateralSymmetrySegmenter.cpp
  src/SymmetrySegmentationAnnotator.cpp
)
target_link_libraries(rs_symmetrysegmentation rs_core rs_io)

rs_add_library(rs_symmetrySegmentationAnnotator src/SymmetrySegmentationAnnotator.cpp)
target_link_libraries(rs_symmetrySegmentationAnnotator rs_core rs_symmetrysegmentation rs_io)
