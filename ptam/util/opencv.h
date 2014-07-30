#ifndef PTAM_UTIL_OPENCV_H_
#define PTAM_UTIL_OPENCV_H_
namespace ptam {
// copy image data to vector of pixels
inline void SafeCopyImage(std::vector<uchar>& v_pixels, cv::Mat const& image) {
  if(image.isContinuous()){
    memcpy(&v_pixels.front(), (uchar*)image.data, image.cols*image.rows*3);
  }else{
    for(int y = 0; y < image.rows; ++y)
      for(int x = 0; x < image.cols; ++x) {
        const int id = 3*(x + y*image.cols);
        const uchar* p = image.at<uchar[3]>(y, x);
        v_pixels[id] = p[0];
        v_pixels[id + 1] = p[1];
        v_pixels[id + 2] = p[2];
      }
  }
}
}  // namespace ptam
#endif  // PTAM_UTIL_OPENCV_H_
