/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
主函数
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
	google::InitGoogleLogging((const char *)argv[0]);				// 初始化GLOG日志巨鹿
	google::SetLogDestination(google::GLOG_INFO, "./log/LOG");		// 设置日志记录文件
	google::SetStderrLogging(google::GLOG_INFO);					// 设置日志记录级别
	LOG(INFO) << "Svaf Copyright(c) 2016-2018, Peng Chao";			// 版权信息
	gflags::ParseCommandLineFlags(&argc, &argv, true);				// GFLAGS处理命令行
	LOG(INFO) << FLAGS_config_file;									// 输出脚本文件名称
	svaf::SvafTask svafTask;										// 实例化任务
	svaf::ReadProtoFromTextFileOrDie(FLAGS_config_file, &svafTask);	// Protobuf读取脚本
	LOG(INFO) << svafTask.name();									// 显示脚本中的任务名称
	svaf::Circuit circuit(svafTask, FLAGS_use_gui);					// 执行算法路线
	LOG(INFO) << "Done.";											// 结束任务
	google::ShutdownGoogleLogging();								// 关闭日志
	return 0;
}
