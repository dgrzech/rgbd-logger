/* boost includes */
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

/* realsense includes */
#include <librealsense2/rs.hpp>

/* opencv includes */
#include <opencv2/opencv.hpp>

#include <fstream>  /* file io */
#include <iostream> /* terminal io */
#include <sstream>  /* stringstreams */

namespace po = boost::program_options;
namespace fs = boost::filesystem;

const int HEIGHT = 480;
const int WIDTH = 640;
const int FRAME_RATE = 30;

/* window names */
const char *WINDOW_DEPTH = "depth image";
const char *WINDOW_COLOR = "colur image";

bool run = true;

std::string filePath = "frames/";
std::string fileName = "frame-";

/* stops loop logging .pngs in main on button press */
static void onMouse(int event, int x, int y, int, void *window_name) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    run = false;
  }
}

/* sets up windows to display color and depth images from the camera during
 * logging */
void setup_viewers() {
  cv::namedWindow(WINDOW_DEPTH, 0);
  cv::namedWindow(WINDOW_COLOR, 0);

  cv::setMouseCallback(WINDOW_DEPTH, onMouse, (void *)WINDOW_DEPTH);
  cv::setMouseCallback(WINDOW_COLOR, onMouse, (void *)WINDOW_COLOR);
}

/* helper function for writing metadata to disk as a .csv file */
void metadata_to_csv(const rs2::frame &frm, const std::string &filename);

int main(int argc, char *argv[]) try {
  rs2::context rsContext;

  /* build output dirs */
  fs::path framesDir(filePath);
  fs::create_directory(framesDir);
  fs::path colorDir(filePath + "/color");
  fs::create_directory(colorDir);
  fs::path depthDir(filePath + "/depth");
  fs::create_directory(depthDir);

  /* declare realsense pipeline, encapsulating the actual device and sensors */
  rs2::pipeline pipe;

  /* create a configuration for the pipeline with a custom profile */
  rs2::config cfg;

  /* add colour and depth streams to configuration */
  cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8,
                    FRAME_RATE);
  cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16,
                    FRAME_RATE);

  /* start pipeline */
  rs2::pipeline_profile selection = pipe.start(cfg);

  /* get depth stream and print depth camera intrinsics */
  auto depth_stream =
      selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

  auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
  auto i = depth_stream.get_intrinsics();
  auto principal_point = std::make_pair(i.ppx, i.ppy);
  auto focal_length = std::make_pair(i.fx, i.fy);
  rs2_distortion model = i.model;

  std::cout << "depth camera principal point         : "
            << principal_point.first << ", " << principal_point.second
            << std::endl;
  std::cout << "depth camera focal length            : " << focal_length.first
            << ", " << focal_length.second << std::endl;

  /* camera warmup--drop several first frames to let
   auto-exposure stabilize */
  rs2::frameset frames;
  for (int i = 0; i < 30; i++) {
    frames = pipe.wait_for_frames();
  }

  /* set up viewers */
  setup_viewers();

  int frameCounter = 0;

  while (true) {
    frames = pipe.wait_for_frames();

    /* get each frame */
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();

    /* create opencv mat from the color image */
    cv::Mat color(cv::Size(WIDTH, HEIGHT), CV_8UC3,
                  (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    /* create opencv mat from the depth image */
    cv::Mat depth(cv::Size(WIDTH, HEIGHT), CV_16UC1,
                  (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << frameCounter;
    std::string frameNum = ss.str();

    cv::imwrite("frames/depth/" + fileName + "-" + frameNum + ".depth.png",
                depth);
    cv::imwrite("frames/color/" + fileName + "-" + frameNum + ".color.png",
                color);

    /* increment the frame counter */
    frameCounter++;

    /* display the images in a gui */
    imshow(WINDOW_COLOR, color);
    cvWaitKey(1);

    imshow(WINDOW_DEPTH, depth);
    cvWaitKey(1);
  }

  cv::destroyAllWindows();

  return 0;
} catch (const rs2::error &e) {
  std::cerr << "realsense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame &frm, const std::string &filename) {
  std::ofstream csv;

  csv.open(filename);

  std::cout << "writing metadata to " << filename << std::endl;
  csv << "stream," << rs2_stream_to_string(frm.get_profile().stream_type())
      << "\nMetadata Attribute,Value\n";

  /* record all the available metadata attributes */
  for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++) {
    if (frm.supports_frame_metadata((rs2_frame_metadata_value)i)) {
      csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
          << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
    }
  }

  csv.close();
}
