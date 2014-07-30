// Copyright 2008 Isis Innovation Limited
#include "track/relocaliser.h"
#include "construct/small_blurry_image.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;

namespace ptam {
Relocaliser::Relocaliser(Map &map, ATANCamera &camera)
  : map_(map), camera_(camera) {}

TooN::SE3<> Relocaliser::BestPose() {
  return se3_best_pose_;
}

bool Relocaliser::AttemptRecovery(KeyFrame &kCurrent) {
  // Ensure the incoming frame has a SmallBlurryImage attached
  if (!kCurrent.pSBI)
    kCurrent.pSBI = new SmallBlurryImage(kCurrent);
  else
    kCurrent.pSBI->MakeFromKF(kCurrent);

  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kCurrent);

  // And estimate a camera rotation from a 3DOF image alignment
  pair<TooN::SE2<>, double> result_pair =
      kCurrent.pSBI->IteratePosRelToTarget(*map_.keyframes[nbest_]->pSBI, 6);
  se2_pose_ = result_pair.first;
  double dScore =result_pair.second;

  TooN::SE3<> se3KeyFramePos = map_.keyframes[nbest_]->se3CfromW;
  se3_best_pose_ = SmallBlurryImage::SE3fromSE2(se2_pose_, camera_) * se3KeyFramePos;

  if (dScore < GVars3::GV2.GetDouble("Reloc2.MaxScore", 9e6, GVars3::SILENT))
    return true;
  else
    return false;
}

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame &kCurrent) {
  best_score_ = 99999999999999.9;
  nbest_ = -1;

  for (unsigned int i = 0; i < map_.keyframes.size(); i++) {
    double dSSD = kCurrent.pSBI->ZMSSD(*map_.keyframes[i]->pSBI);
    if (dSSD < best_score_) {
      best_score_ = dSSD;
      nbest_ = i;
    }
  }
}
}  // namespace ptam

