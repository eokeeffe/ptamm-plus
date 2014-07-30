// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not
// invalidated!

#ifndef PATAM_CONSTRUCT_MAP_H_
#define PATAM_CONSTRUCT_MAP_H_
#include <vector>
#include <TooN/se3.h>
#include <cvd/image.h>

namespace ptam {
struct MapPoint;
struct KeyFrame;

struct Map {
  Map();
  inline bool IsGood() {return good;}
  void Reset();

  void MoveBadPointsToTrash();
  void EmptyTrash();

  std::vector<MapPoint*> points;
  std::vector<MapPoint*> trashed_points;
  std::vector<KeyFrame*> keyframes;

  bool good;
};
}  // namespace ptam
#endif

