/* sys headers */
#include <cstdio>
#include <ctime>
#include <iostream>
#include <stdio.h>

/* boost includes */
#include <boost/lexical_cast.hpp>

/* libfreenect2 includes */
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

/* opencv includes */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct KinectCaptureApp {
  KinectCaptureApp() {}
};

bool capture_opencv(std::string writeDir, bool viewerEnabled) {
  libfreenect2::Freenect2 freenect2;
  libfreenect2::PacketPipeline *pipeline =
      new libfreenect2::OpenGLPacketPipeline();
  libfreenect2::Freenect2Device *dev = freenect2.openDevice(0, pipeline);

  int types = 0;
  types |= libfreenect2::Frame::Color;
  types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  libfreenect2::Registration *registration = new libfreenect2::Registration(
      dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  clock_t begin = clock();
  size_t framecount = 0;

  std::cout << writeDir << std::endl;

  while (double(clock() - begin) / CLOCKS_PER_SEC < 10) {
    try {
      listener.waitForNewFrame(frames);

      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

      registration->apply(rgb, depth, &undistorted, &registered);

      cv::Mat colorMat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
      cv::Mat depthMat =
          cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);

      cv::Mat registeredMat = cv::Mat(registered.height, registered.width,
                                      CV_8UC4, registered.data);
      cv::Mat undistortedMat = cv::Mat(undistorted.height, undistorted.width,
                                       CV_32FC1, undistorted.data);
      undistortedMat.convertTo(undistortedMat, CV_8U, 255. / 65535);

      cv::Mat frame_color, frame_depth_color;
      cv::cvtColor(registeredMat, frame_color, CV_RGB2BGR);
      cv::cvtColor(undistortedMat, frame_depth_color, CV_GRAY2BGR);

      if (viewerEnabled) {
        cv::imshow("color", registeredMat);
        cv::imshow("depth", undistortedMat);

        // react on keyboard input
        char key = cv::waitKey(10);
        switch (key) {
        case 'q':
          break;
        }
      }

      cv::imwrite(writeDir + "color/" + to_string(framecount) + ".png",
                  registeredMat);
      cv::imwrite(writeDir + "depth/" + to_string(framecount) + ".png",
                  undistortedMat);

      listener.release(frames);

      framecount++;
    } catch (int e) {
      framecount++;
    }
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elpased time: " << elapsed_secs
            << "s, no. of frames: " << framecount << std::endl;

  dev->stop();
  dev->close();

  delete registration;

  return true;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stderr, "not enough arguments");
    exit(0);
  }

  KinectCaptureApp *app;
  app = new KinectCaptureApp();

  std::string writeDir(argv[1]);
  std::string viewerEnabledStr(argv[2]);
  bool viewerEnabled = boost::lexical_cast<bool>(viewerEnabledStr);

  try {
    std::cout << "launching capture" << std::endl;

    capture_opencv(writeDir, viewerEnabled);

    std::cout << "capture complete" << std::endl;
  } catch (int e) {
  };

  delete app;

  return 0;
}
