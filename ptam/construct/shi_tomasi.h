// Copyright 2008 Isis Innovation Limited
#ifndef PTAM_CONSTRUCT_SHI_TOMASI_H_
#define PTAM_CONSTRUCT_SHI_TOMASI_H_

#include <cvd/image.h>
#include <cvd/byte.h>

namespace ptam {
double FindShiTomasiScoreAtPoint(CVD::BasicImage<CVD::byte> &image,
         int nHalfBoxSize,
         CVD::ImageRef irCenter);
}
#endif
