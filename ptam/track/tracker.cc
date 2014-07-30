// Copyright 2008 Isis Innovation Limited
#include "ui/open_gl.h"
#include "track/tracker.h"

#include <fstream>
#include <fcntl.h>

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <TooN/se2.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include "track/m_estimator.h"
#include "construct/shi_tomasi.h"
#include "track/small_matrix_opts.h"
#include "track/patch_finder.h"
#include "track/tracker_data.h"

using namespace CVD;
using namespace std;
using namespace TooN;

namespace ptam {
// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm) :
  mMap(m),
  mMapMaker(mm),
  mCamera(c),
  mRelocaliser(mMap, mCamera),
  mirSize(irVideoSize) {
  mCurrentKF.bFixed = false;
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;

  // Most of the initialisation is done in Reset()
  Reset();
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset() {
  mbDidCoarse = false;
  mbUserAskInitialTrack = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = TooN::Zeros;
  mbJustRecoveredSoUseCoarse = false;

  // Tell the MapMaker to reset itself..
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  mMapMaker.RequestReset();
  while(!mMapMaker.ResetDone())
#ifndef WIN32
    usleep(10);
#else
    Sleep(1);
#endif
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(Image<byte> &imFrame, bool bDraw) {
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clean

  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  mCurrentKF.mMeasurements.clear();
  mCurrentKF.MakeKeyFrame_Lite(imFrame);

  // Update the small images for the rotation estimator
  static GVars3::gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, GVars3::SILENT);
  static GVars3::gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, GVars3::SILENT);
  mbUseSBIInit = *gvnUseSBI;
  if (!mpSBIThisFrame) {
    mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
  } else {
    delete  mpSBILastFrame;
    mpSBILastFrame = mpSBIThisFrame;
    mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
  }

  // From now on we only use the keyframe struct!
  mnFrame++;

//  if (mbDraw) {
//    glDrawPixels(mCurrentKF.aLevels[0].im);
//    if (GVars3::GV2.GetInt("Tracker.DrawFASTCorners",0, GVars3::SILENT)) {
//      glColor3f(1, 0, 1);  glPointSize(1); glBegin(GL_POINTS);
//      for (unsigned int i = 0; i < mCurrentKF.aLevels[0].vCorners.size(); i++)
//        glVertex(mCurrentKF.aLevels[0].vCorners[i]);
//      glEnd();
//    }
//  }

  // Decide what to do - if there is a map, try to track the map ...
  if (mMap.IsGood()) {
    if (mnLostFrames < 3) {  // .. but only if we're not lost!
      if(mbUseSBIInit)
        CalcSBIRotation();
      ApplyMotionModel();       //
      TrackMap();               //  These three lines do the main tracking work.
      UpdateMotionModel();      //

      AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.

      { // Provide some feedback for the user:
        mMessageForUser << "Tracking Map, quality ";
        if (mTrackingQuality == GOOD)  mMessageForUser << "good.";
        if (mTrackingQuality == DODGY) mMessageForUser << "poor.";
        if (mTrackingQuality == BAD)   mMessageForUser << "bad.";
        mMessageForUser << " Found:";
        for (int i = 0; i < LEVELS; i++)
          mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
        //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
        mMessageForUser << " Map: " << mMap.points.size() << "P, " << mMap.keyframes.size() << "KF";
      }

      // Heuristics to check if a key-frame should be added to the map:
      if (mTrackingQuality == GOOD &&
          mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
          mnFrame - mnLastKeyFrameDropped > 20  &&
          mMapMaker.QueueSize() < 3) {
        mMessageForUser << " Adding key-frame.";
        AddNewKeyFrame();
      }
    } else {  // what if there is a map, but tracking has been lost?
      mMessageForUser << "** Attempting recovery **.";
      if (AttemptRecovery()) {
        TrackMap();
        AssessTrackingQuality();
      }
    }
//    if (mbDraw)
//      RenderGrid();
  }
  else // If there is no map, try to make one.
    TrackForInitialMap();
}

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery() {
  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if (!bRelocGood)
    return false;

  TooN::SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = TooN::Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap() {
  // MiniPatch tracking threshhold.
  static GVars3::gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD",
                                      100000, GVars3::SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;

  // What stage of initial tracking are we at?
  if (mnInitialStage == TRAIL_TRACKING_NOT_STARTED) {
    if (mbUserAskInitialTrack) {  // First spacebar = this is the first keyframe
      mbUserAskInitialTrack = false;
      TrailTracking_Start();
      mnInitialStage = TRAIL_TRACKING_STARTED;
    } else {
      mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
    }
    return;
  };

  if (mnInitialStage == TRAIL_TRACKING_STARTED) {
    int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
    if (nGoodTrails < 10) { // if most trails have been wiped out, no point continuing.
      Reset();
      return;
    }

    // If the user pressed spacebar here, use trails to run stereo and make the intial map..
    if (mbUserAskInitialTrack) {
      mbUserAskInitialTrack = false;
      vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
      for (list<Trail>::iterator i = mlTrails.begin(); i != mlTrails.end(); i++)
        vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,
                                                    i->irCurrentPos));
      mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
      mnInitialStage = TRAIL_TRACKING_COMPLETE;
    } else {
      mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;
    }
  }
}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start() {
  mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
  mFirstKF = mCurrentKF;
  std::vector<std::pair<double, ImageRef> > vCornersAndSTScores;
  for (size_t i = 0; i < mCurrentKF.aLevels[0].vCandidates.size(); i++)  {
    // Copy candidates into a trivially sortable vector
    // so that we can choose the image corners with max ST score
    Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
    if (!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
      continue;
    vCornersAndSTScores.push_back(pair<double, ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
  };
  sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
  int nToAdd = GVars3::GV2.GetInt("MaxInitialTrails", 1000, GVars3::SILENT);
  for (unsigned int i = 0; i < vCornersAndSTScores.size() && nToAdd > 0; i++) {
    if (!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
      continue;
    Trail t;
    t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
    t.irInitialPos = vCornersAndSTScores[i].second;
    t.irCurrentPos = t.irInitialPos;
    mlTrails.push_back(t);
    nToAdd--;
  }
  mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance() {
  int ngoodtrails = 0;

  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

  for (list<Trail>::iterator i = mlTrails.begin(); i != mlTrails.end();) {
    list<Trail>::iterator next = i; next++;

    Trail &trail = *i;
    ImageRef irStart = trail.irCurrentPos;
    ImageRef irEnd = irStart;
    bool found = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im,
                                         10, lCurrentFrame.vCorners);
    if (found) {
      // Also find backwards in a married-matches check
      BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
      ImageRef irBackWardsFound = irEnd;
      found = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
      if((irBackWardsFound - irStart).mag_squared() > 2)
        found = false;

      trail.irCurrentPos = irEnd;
      ngoodtrails++;
      trail.found = true;
    } else {
      trail.found = false;
    }
    if (!found) {  // Erase from list of trails if not found this frame.
      mlTrails.erase(i);
    }
    i = next;
  }
  mPreviousFrameKF = mCurrentKF;
  return ngoodtrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap() {
  // Some accounting which will be used for tracking quality assessment:
  for (int i = 0; i < LEVELS; i++)
    manMeasAttempted[i] = manMeasFound[i] = 0;

  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  std::vector<TrackerData*> v_pvs[LEVELS];
  for (int i = 0; i < LEVELS; i++)
    v_pvs[i].reserve(500);

  // For all points in the map..
  for (size_t i = 0; i < mMap.points.size(); i++) {
    MapPoint &p= *(mMap.points[i]);
    // Ensure that this map point has an associated TrackerData struct.
    if(!p.pTData) p.pTData = new TrackerData(&p);
    TrackerData &TData = *p.pTData;

    // Project according to current view, and if it's not in the image, skip.
    TData.Project(mse3CamFromWorld, mCamera);
    if (!TData.bInImage)
      continue;

    // Calculate camera projection derivatives of this point.
    TData.GetDerivsUnsafe(mCamera);

    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
    if (TData.nSearchLevel == -1)
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    TData.bSearched = false;
    TData.bFound = false;
    v_pvs[TData.nSearchLevel].push_back(&TData);
  };

  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for (int i = 0; i < LEVELS; i++)
    random_shuffle(v_pvs[i].begin(), v_pvs[i].end());

  // The next two data structs contain the list of points which will next
  // be searched for in the image, and then used in pose update.
  std::vector<TrackerData*> v_next2search;
  std::vector<TrackerData*> v_iteration_set;

  // Tunable parameters to do with the coarse tracking stage:
  static GVars3::gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, GVars3::SILENT);   // Min number of large-scale features for coarse stage
  static GVars3::gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, GVars3::SILENT);   // Max number of large-scale features for coarse stage
  static GVars3::gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, GVars3::SILENT);       // Pixel search radius for coarse features
  static GVars3::gvar3<int> gvn_coarse_subpixiter("Tracker.CoarseSubPixIts", 8, GVars3::SILENT); // Max sub-pixel iterations for coarse features
  static GVars3::gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, GVars3::SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static GVars3::gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, GVars3::SILENT);  // Speed above which coarse stage is used.

  unsigned int ncoarse_max = *gvnCoarseMax;
  unsigned int ncoarse_range = *gvnCoarseRange;

  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool try_coarse = true;
  if (*gvnCoarseDisabled ||
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     ncoarse_max == 0)
    try_coarse = false;
  if (mbJustRecoveredSoUseCoarse) {
    try_coarse = true;
    ncoarse_max *=2;
    ncoarse_range *=2;
    mbJustRecoveredSoUseCoarse = false;
  };

  // If we do want to do a coarse stage, also check that there's enough high-level
  // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
  // with preference to LEVELS-1.
  if (try_coarse &&
      v_pvs[LEVELS-1].size() + v_pvs[LEVELS-2].size() > *gvnCoarseMin ) {
    // Now, fill the vNextToSearch struct with an appropriate number of
    // TrackerDatas corresponding to coarse map points! This depends on how many
    // there are in different pyramid levels compared to CoarseMin and CoarseMax.

    if (v_pvs[LEVELS-1].size() <= ncoarse_max) {
      // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
      v_next2search = v_pvs[LEVELS-1];
      v_pvs[LEVELS-1].clear();
    } else { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
      for(unsigned int i=0; i<ncoarse_max; i++)
        v_next2search.push_back(v_pvs[LEVELS-1][i]);
      v_pvs[LEVELS-1].erase(v_pvs[LEVELS-1].begin(), v_pvs[LEVELS-1].begin() + ncoarse_max);
    }

    // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
    if (v_next2search.size() < ncoarse_max) {
      unsigned int nMoreCoarseNeeded = ncoarse_max - v_next2search.size();
      if (v_pvs[LEVELS-2].size() <= nMoreCoarseNeeded) {
        v_next2search = v_pvs[LEVELS-2];
        v_pvs[LEVELS-2].clear();
      } else {
        for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
          v_next2search.push_back(v_pvs[LEVELS-2][i]);
        v_pvs[LEVELS-2].erase(v_pvs[LEVELS-2].begin(), v_pvs[LEVELS-2].begin() + nMoreCoarseNeeded);
      }
    }
    // Now go and attempt to find these points in the image!
    unsigned int nfounds = SearchForPoints(v_next2search, ncoarse_range,
                                           *gvn_coarse_subpixiter);
    v_iteration_set = v_next2search;  // Copy over into the to-be-optimised list.
    if (nfounds >= *gvnCoarseMin) {  // Were enough found to do any meaningful optimisation?
      mbDidCoarse = true;
      for (int iter = 0; iter < 10; iter++) {
        // If so: do ten Gauss-Newton pose updates iterations.
        if (iter != 0) {
          // Re-project the points on all but the first iteration.
          for (size_t i = 0; i < v_iteration_set.size(); i++)
            if (v_iteration_set[i]->bFound)
              v_iteration_set[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        }
        for(size_t i=0; i<v_iteration_set.size(); i++)
          if(v_iteration_set[i]->bFound)
            v_iteration_set[i]->CalcJacobian();
        double d_override_sigma = 0.0;
        // Hack: force the MEstimator to be pretty brutal
        // with outliers beyond the fifth iteration.
        if (iter > 5)
          d_override_sigma = 1.0;

        // Calculate and apply the pose update...
        TooN::Vector<6> v6_update =
            CalcPoseUpdate(v_iteration_set, d_override_sigma);
        mse3CamFromWorld = TooN::SE3<>::exp(v6_update) * mse3CamFromWorld;
      };
    }
  };

  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!

  int nfine_range = 10;  // Pixel search range for the fine stage.
  if (mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
    nfine_range = 5;

  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
    int l = LEVELS - 1;
    for (size_t i = 0; i < v_pvs[l].size(); i++)
      v_pvs[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    SearchForPoints(v_pvs[l], nfine_range, 8);
    for (unsigned int i = 0; i < v_pvs[l].size(); i++)
      v_iteration_set.push_back(v_pvs[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  };

  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  v_next2search.clear();
  for (int l = LEVELS - 2; l >= 0; l--)
    for (size_t i = 0; i < v_pvs[l].size(); i++)
      v_next2search.push_back(v_pvs[l][i]);

  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
  // ourselves to 1000, and choose these randomly.
  static GVars3::gvar3<int> gvn_max_patches_per_frame("Tracker.MaxPatchesPerFrame",
                                                  1000, GVars3::SILENT);
  int nfine_patches2use = *gvn_max_patches_per_frame - v_iteration_set.size();
  if ((int) v_next2search.size() > nfine_patches2use) {
    random_shuffle(v_next2search.begin(), v_next2search.end());
    v_next2search.resize(nfine_patches2use); // Chop!
  };

  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if (mbDidCoarse)
    for (unsigned int i = 0; i < v_next2search.size(); i++)
      v_next2search[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);

  // Find fine points in image:
  SearchForPoints(v_next2search, nfine_range, 0);
  // And attach them all to the end of the optimisation-set.
  for (size_t i = 0; i < v_next2search.size(); i++)
    v_iteration_set.push_back(v_next2search[i]);

  // Again, ten gauss-newton pose update iterations.
  TooN::Vector<6> v6_last_update;
  v6_last_update = TooN::Zeros;
  for (int iter = 0; iter < 10; iter++) {
    bool b_nonlinear_iteration; // For a bit of time-saving: don't do full nonlinear
    // reprojection at every iteration - it really isn't necessary!
    if (iter == 0 || iter == 4 || iter == 9)
      b_nonlinear_iteration = true;   // Even this is probably overkill, the reason we do many
    else                            // iterations is for M-Estimator convergence rather than
      b_nonlinear_iteration = false;  // linearisation effects.

    if (iter != 0) {  // Either way: first iteration doesn't need projection update.
      if (b_nonlinear_iteration) {
        for (unsigned int i = 0; i < v_iteration_set.size(); i++)
          if (v_iteration_set[i]->bFound)
            v_iteration_set[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
      } else {
        for (unsigned int i = 0; i < v_iteration_set.size(); i++)
          if (v_iteration_set[i]->bFound)
            v_iteration_set[i]->LinearUpdate(v6_last_update);
      };
    }

    if (b_nonlinear_iteration)
      for (unsigned int i = 0; i < v_iteration_set.size(); i++)
        if (v_iteration_set[i]->bFound)
          v_iteration_set[i]->CalcJacobian();

    // Again, an M-Estimator hack beyond the fifth iteration.
    double d_override_sigma = 0.0;
    if (iter > 5)
      d_override_sigma = 16.0;

    // Calculate and update pose; also store update vector for linear iteration updates.
    TooN::Vector<6> v6_update =
        CalcPoseUpdate(v_iteration_set, d_override_sigma, iter == 9);
    mse3CamFromWorld = TooN::SE3<>::exp(v6_update) * mse3CamFromWorld;
    v6_last_update = v6_update;
  };

  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mCurrentKF.se3CfromW = mse3CamFromWorld;

  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mCurrentKF.mMeasurements.clear();
  for (vector<TrackerData*>::iterator it = v_iteration_set.begin();
      it != v_iteration_set.end();
      it++) {
    if (!(*it)->bFound)
      continue;
    Measurement m;
    m.v2RootPos = (*it)->v2Found;
    m.nLevel = (*it)->nSearchLevel;
    m.bSubPix = (*it)->bDidSubPix;
    mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
  }

  // Finally, find the mean scene depth from tracked features
  {
    double sum = 0.0;
    double sumsq = 0.0;
    int count = 0;
    for (vector<TrackerData*>::iterator it = v_iteration_set.begin();
        it!= v_iteration_set.end();
        it++)
      if ((*it)->bFound) {
        double z = (*it)->v3Cam[2];
        sum += z;
        sumsq += z*z;
        count++;
      };
    if (count > 20) {
      mCurrentKF.dSceneDepthMean = sum/count;
      mCurrentKF.dSceneDepthSigma = std::sqrt((sumsq / count) -
                                         (mCurrentKF.dSceneDepthMean) *
                                         (mCurrentKF.dSceneDepthMean));
    }
  }
}

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData*> &v_trackdatas,
                             int nrange, int n_subpixiters) {
  int nfounds = 0;
  for (size_t i = 0; i < v_trackdatas.size(); i++) {  // for each point..
    // First, attempt a search at pixel locations which are FAST corners.
    // (PatchFinder::FindPatchCoarse)
    TrackerData &TD = *v_trackdatas[i];
    PatchFinder &Finder = TD.Finder;
    Finder.MakeTemplateCoarseCont(TD.Point);
    if (Finder.TemplateBad()) {
      TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
      continue;
    }
    manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta

    bool bFound =
        Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nrange);
    TD.bSearched = true;
    if (!bFound) {
      TD.bFound = false;
      continue;
    }

    TD.bFound = true;
    TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());

    nfounds++;
    manMeasFound[Finder.GetLevel()]++;

    // Found the patch in coarse search - are Sub-pixel iterations wanted too?
    if (n_subpixiters > 0) {
      TD.bDidSubPix = true;
      Finder.MakeSubPixTemplate();
      bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, n_subpixiters);
      if (!bSubPixConverges) {
        // If subpix doesn't converge, the patch location is probably very dubious!
        TD.bFound = false;
        nfounds--;
        manMeasFound[Finder.GetLevel()]--;
        continue;
      }
      TD.v2Found = Finder.GetSubPixPos();
    } else {
      TD.v2Found = Finder.GetCoarsePosAsVector();
      TD.bDidSubPix = false;
    }
  }
  return nfounds;
}

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
TooN::Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> v_trackdatas,
                                        double d_override_sigma,
                                        bool b_mark_outliers) {
  // Which M-estimator are we using?
  int n_estimator = 0;
  static GVars3::gvar3<string> gvs_estimator("TrackerMEstimator", "Tukey", GVars3::SILENT);
  if (*gvs_estimator == "Tukey") {
    n_estimator = 0;
  } else if (*gvs_estimator == "Cauchy") {
    n_estimator = 1;
  } else if (*gvs_estimator == "Huber") {
    n_estimator = 2;
  } else {
    cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
    n_estimator = 0;
    *gvs_estimator = "Tukey";
  };

  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  std::vector<double> sq_errors;
  for (size_t f = 0; f < v_trackdatas.size(); f++) {
    TrackerData &TD = *v_trackdatas[f];
    if(!TD.bFound)
      continue;
    TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
    sq_errors.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
  }

  // No valid measurements? Return null update.
  if (sq_errors.size() == 0)
    return TooN::makeVector(0, 0, 0, 0, 0, 0);

  // What is the distribution of errors?
  double sigma_squared;
  if (d_override_sigma > 0) {
    sigma_squared = d_override_sigma; // Bit of a waste having stored the vector of square errors in this case!
  } else {
    if (n_estimator == 0)
      sigma_squared = Tukey::FindSigmaSquared(sq_errors);
    else if(n_estimator == 1)
      sigma_squared = Cauchy::FindSigmaSquared(sq_errors);
    else
      sigma_squared = Huber::FindSigmaSquared(sq_errors);
  }

  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  TooN::WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for (size_t f = 0; f < v_trackdatas.size(); f++) {
    TrackerData& track_data = *v_trackdatas[f];
    if(!track_data.bFound)
      continue;
    TooN::Vector<2> &v2 = track_data.v2Error_CovScaled;
    double error_sq = v2 * v2;
    double weight;

    if (n_estimator == 0)
      weight = Tukey::Weight(error_sq, sigma_squared);
    else if (n_estimator == 1)
      weight = Cauchy::Weight(error_sq, sigma_squared);
    else
      weight = Huber::Weight(error_sq, sigma_squared);

    // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
    if (weight == 0.0) {
      if (b_mark_outliers)
        track_data.Point.nMEstimatorOutlierCount++;
      continue;
    } else {
      if(b_mark_outliers)
        track_data.Point.nMEstimatorInlierCount++;
    }
    TooN::Matrix<2,6> &m26Jac = track_data.m26Jacobian;
    wls.add_mJ(v2[0], track_data.dSqrtInvNoise * m26Jac[0], weight); // These two lines are currently
    wls.add_mJ(v2[1], track_data.dSqrtInvNoise * m26Jac[1], weight); // the slowest bit of poseits
  }

  wls.compute();
  return wls.get_mu();
}

// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel() {
  mse3StartPos = mse3CamFromWorld;
  TooN::Vector<6> v6Velocity = mv6CameraVelocity;
  if (mbUseSBIInit) {
    v6Velocity.slice<3, 3>() = mv6SBIRot.slice<3, 3>();
    v6Velocity[0] = 0.0;
    v6Velocity[1] = 0.0;
  }
  mse3CamFromWorld = TooN::SE3<>::exp(v6Velocity) * mse3StartPos;
}

// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel() {
  TooN::SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  TooN::Vector<6> v6Motion = TooN::SE3<>::ln(se3NewFromOld);
  TooN::Vector<6> v6OldVel = mv6CameraVelocity;

  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);

  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  TooN::Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame() {
  mMapMaker.AddKeyFrame(mCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality() {
  int total_attempted = 0;
  int total_found = 0;
  int large_attempted = 0;
  int large_found = 0;

  for (int i = 0; i < LEVELS; i++) {
    total_attempted += manMeasAttempted[i];
    total_found += manMeasFound[i];
    if(i>=2) large_attempted += manMeasAttempted[i];
    if(i>=2) large_found += manMeasFound[i];
  }

  if (total_found == 0 || total_attempted == 0) {
    mTrackingQuality = BAD;
  } else {
    double total_frac_found = (double) total_found / total_attempted;
    double large_frac_found;
    if(large_attempted > 10)
      large_frac_found = (double) large_found / large_attempted;
    else
      large_frac_found = total_frac_found;

    static GVars3::gvar3<double> gvd_quality_good("Tracker.TrackingQualityGood",
                                                0.3, GVars3::SILENT);
    static GVars3::gvar3<double> gvd_quality_lost("Tracker.TrackingQualityLost",
                                                0.13, GVars3::SILENT);


    if (total_frac_found > *gvd_quality_good)
      mTrackingQuality = GOOD;
    else if (large_frac_found < *gvd_quality_lost)
      mTrackingQuality = BAD;
    else
      mTrackingQuality = DODGY;
  }

  if (mTrackingQuality == DODGY) {
    // Further heuristics to see if it's actually bad, not just dodgy...
    // If the camera pose estimate has run miles away, it's probably bad.
    if (mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
      mTrackingQuality = BAD;
  }

  if (mTrackingQuality == BAD)
    mnLostFrames++;
  else
    mnLostFrames = 0;
}

string Tracker::GetMessageForUser() {
  return mMessageForUser.str();
}

void Tracker::CalcSBIRotation() {
  mpSBILastFrame->MakeJacs();
  pair<TooN::SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  TooN::SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here
}  // namespace ptam






