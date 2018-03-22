/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#include <hash_map>
#include <opencv2\opencv.hpp>
#include <gflags\gflags.h>
#include <glog\logging.h>

#include "..\svaf\svaf.pb.h"
#include "..\svaf\io.hpp"

#include "Circuit.h"

using namespace std;
using namespace cv;

DEFINE_bool(use_gui, false, "wheather to use process communication module");
DEFINE_string(config_file, "./svaf/svaf.pbf", "config file of the framework");

int main(int argc, char *argv[]){
	google::InitGoogleLogging((const char *)argv[0]);
	google::SetLogDestination(google::GLOG_INFO, "./log/LOG");
	google::SetStderrLogging(google::GLOG_INFO);
	LOG(INFO) << "Svaf Copyright(c) 2016-2018, Peng Chao";
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	LOG(INFO) << FLAGS_config_file;
	svaf::SvafTask svafTask;
	svaf::ReadProtoFromTextFileOrDie(FLAGS_config_file, &svafTask);
	LOG(INFO) << svafTask.name();
	svaf::Circuit circuit(svafTask, FLAGS_use_gui);
	LOG(INFO) << "Done.";
	google::ShutdownGoogleLogging();
	return 0;
}
