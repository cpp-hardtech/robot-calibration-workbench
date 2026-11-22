#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Expected a source image and an output path\n";
    return 1;
  }

  const cv::Mat source = cv::imread(argv[1]);
  if (source.empty())
  {
    std::cerr << "Could not decode the sample image\n";
    return 1;
  }

  cv::Mat padded;
  cv::copyMakeBorder(source, padded, 0, 0, 0, 64, cv::BORDER_REPLICATE);
  return cv::imwrite(argv[2], padded) ? 0 : 1;
}
