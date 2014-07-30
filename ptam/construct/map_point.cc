// Copyright 2008 Isis Innovation Limited
#include "construct/map_point.h"
#include "construct/keyframe.h"
namespace ptam {
void MapPoint::RefreshPixelVectors() {
  KeyFrame &k = *pPatchSourceKF;

  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  TooN::Vector<3> v3PlanePoint_C = k.se3CfromW * v3WorldPos;

  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dCamHeight = std::fabs(v3PlanePoint_C * v3Normal_NC);

  double dPixelRate = std::fabs(v3Center_NC * v3Normal_NC);
  double dOneRightRate = std::fabs(v3OneRightFromCenter_NC * v3Normal_NC);
  double dOneDownRate = std::fabs(v3OneDownFromCenter_NC * v3Normal_NC);

  // Find projections onto plane
  TooN::Vector<3> v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  TooN::Vector<3> v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  TooN::Vector<3> v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;

  // Find differences of these projections in the world frame
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() *
      (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() *
      (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}
}  // namespace ptam
