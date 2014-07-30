#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "slam_system.h"
#include "config.h"

using namespace ptam;
using namespace i3d;

int main(int argc, char **argv) {
  printf("PTAM \n");
  printf("  --------------- \n");
  printf("  Parsing settings.cfg ....\n");
  std::string cam_configfile = (argc > 1) ? argv[1] : CAMERA_CONFIG_FILE;
  std::string configfile = (argc > 2) ? argv[2] : CONFIG_FILE;
  GVars3::GUI.LoadFile(cam_configfile);
  GVars3::GUI.LoadFile(configfile);
  VideoCaptureDispatch<CameraFamily::PGR_FIREFLY_MV> videodispatch;
  std::string vsource = GVars3::GV3::get<string>("Video.source");
  if (vsource.length() < 3) {
    if(!videodispatch.open(atoi(vsource.c_str())))
      return -1;
    videodispatch.setProperty("Brightness", GVars3::GV3::get<int>("Video.brightness", 50, GVars3::SILENT));
    videodispatch.setProperty("Exposure", GVars3::GV3::get<int>("Video.exposure", 40, GVars3::SILENT));
    videodispatch.setProperty("Gain", GVars3::GV3::get<int>("Video.gain", 40, GVars3::SILENT));
    videodispatch.setProperty("Shutter", GVars3::GV3::get<int>("Video.shutter", 530, GVars3::SILENT));
  } else {
    if(!videodispatch.open(vsource))
      return -1;
  }

  // Parsing console input
  GVars3::GUI.StartParserThread();
  atexit(GVars3::GUI.StopParserThread);

  try {
    SlamSystem sys;
    sys.mVideoSource = &videodispatch;
    sys.Run();
  } catch (CVD::Exceptions::All e) {
    printf("Oops! slam.cc main()::%s\n", e.what.c_str());
  }
}










