//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to
// do simple patch tracking across a stereo pair. This is handled
// by the TrackForInitialMap() method and associated sub-methods.
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef PTAM_TRACK_TRACKER_H_
#define PTAM_TRACK_TRACKER_H_

#include <sstream>
#include <vector>
#include <list>

#include "construct/map_maker.h"
#include "math/atan_camera.h"
#include "track/mini_patch.h"
#include "track/relocaliser.h"

namespace ptam {
class TrackerData;
struct Trail {  // Data for initial correspondences of the first stereo pair.
  Trail() : found(true) {}
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
  bool found;
};

class Tracker {
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);

  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw = false);

  inline TooN::SE3<> GetCurrentPose() { return mse3CamFromWorld;}

  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();

  const std::list<Trail>& GetTrails() { return mlTrails; }

  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.

  void AskInitialTrack() {
    mbUserAskInitialTrack = true;
  }

 protected:
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct

  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED,
  TRAIL_TRACKING_STARTED,
  TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches

  // Methods for tracking the map once it has been made:
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &v_trackdatas,
          int nrange,
          int n_subpixiters);  // Finds points in the image
  TooN::Vector<6> CalcPoseUpdate(std::vector<TrackerData*> v_trackdatas,
         double d_override_sigma = 0.0,
         bool b_mark_outliers = false); // Updates pose from found points.

  TooN::SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  TooN::SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  TooN::Vector<6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

  bool mbDraw;                    // Should the tracker draw anything to OpenGL?

  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;

  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  TooN::Vector<6> mv6SBIRot;
  bool mbUseSBIInit;

  // User interaction for initial tracking:
  bool mbUserAskInitialTrack;
  std::ostringstream mMessageForUser;
};
}  // namespace ptam
#endif






