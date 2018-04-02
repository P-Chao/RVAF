/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
记录结果的数据表格
*/


#include <glog\logging.h>
#include <iostream>
#include <vector>
#include <map>

using namespace std;

namespace svaf{

// 操作数据模板元
template<typename T>
struct Cell{
	bool	mask = false;
	T		val;
	void operator=(T &value){val = value; mask = true;}
	Cell(T value, bool valid) : val(value), mask(valid){}
	Cell(T value) : val(value), mask(true){}
	Cell() : mask(false){};
};

// 数据表模板
template<typename T = float>
class Figures{
public:
	Figures();
	~Figures();
	size_t rows();
	size_t cols();
	void clear();
	void clearAll();
	void addRow();
	void setRow(size_t size);
	void addCol(string filed);
	void push_back(string filed, size_t id, T value);
	void print2txt(string filename);
	void print2txt_im(string filename, vector<string>& images);
	void print2txt_impair(string filename, vector<pair<string, string>>&);
	void print2scr();

	vector<Cell<T>>& operator[](string& filed);
	vector<Cell<T>>& operator[](const char * filed);

protected:
	void resize(size_t size);

private:
	size_t size_;
	map<string, vector<Cell<T>>> figure_;
	
};

}

namespace svaf{

	// 构造
	template<typename T>
	Figures<T>::Figures(){
	}

	// 析构
	template<typename T>
	Figures<T>::~Figures(){
	}

	// 返回行数
	template<typename T>
	size_t Figures<T>::rows(){
		return size_;
	}

	// 返回列数
	template<typename T>
	size_t Figures<T>::cols(){
		return figure_.size();
	}

	// 添加一行
	template<typename T>
	void Figures<T>::addRow(){
		size_++;
		resize(size_);
	}

	// 设置行数
	template<typename T>
	void Figures<T>::setRow(size_t size){
		if (size < size_){
			LOG(ERROR) << "Figures Data May Loss.";
		}
		resize(size);
	}

	// 添加一列
	template<typename T>
	void Figures<T>::addCol(string filed){
		vector<Cell<T>> tempvec
			figure_[filed] = tempvec;
	}

	// 设置表格大小
	template<typename T>
	void Figures<T>::resize(size_t size){
		size_ = size;
		for (auto &itm : figure_){
			itm.second.resize(size_);
		}

	}

	// 输入数据到表格中
	template<typename T>
	void Figures<T>::push_back(string filed, size_t id, T value){
		if (id >= size_){
			resize(id+1);
		}
		if (id >= (*this)[filed].size()){

		}
		if (id >= figure_.size()){
			resize(id + 1);
		}
		(*this)[filed][id].val = value;
	}

	// 重载操作运算符
	template<typename T>
	vector<Cell<T>>& Figures<T>::operator[](string& filed){
		auto iter = figure_.find(filed);
		if (iter == figure_.end()){
			figure_[filed].resize(size_);
		}
		return figure_[filed];
	}

	template<typename T>
	vector<Cell<T>>& Figures<T>::operator[](const char* filed){
		return figure_[string(filed)];
	}

	// 清空数据表
	template<typename T>
	void Figures<T>::clear(){
		resize(0);
	}

	template<typename T>
	void Figures<T>::clearAll(){
		size_ = 0;
		figure_.clear();
	}

	// 输出数据结果到文件
	template<typename T>
	void Figures<T>::print2txt(string filename){
		FILE* fp = fopen(filename.c_str(), "wb+");

		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");

		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// 输出数据结果到文件（包含图像文件名）
	template<typename T>
	void Figures<T>::print2txt_im(string filename, vector<string>& images){
		FILE* fp = fopen(filename.c_str(), "wb+");

		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");

		fprintf(fp, "name\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", images[i].c_str());
		}
		fprintf(fp, "\r\n");

		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// 输出数据结果到文件（包含一对图像文件名）
	template<typename T>
	void Figures<T>::print2txt_impair(string filename, vector<pair<string, string>>& imagepairs){
		FILE* fp = fopen(filename.c_str(), "wb+");

		fprintf(fp, "id\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%5d\t", i);
		}
		fprintf(fp, "\r\n");

		fprintf(fp, "left\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", imagepairs[i].first.c_str());
		}
		fprintf(fp, "\r\n");

		fprintf(fp, "right\t");
		for (int i = 0; i < size_; ++i){
			fprintf(fp, "%s\t", imagepairs[i].second.c_str());
		}
		fprintf(fp, "\r\n");

		for (auto it : figure_){
			fprintf(fp, "%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					fprintf(fp, "%.4f\t", it.second[j].val);
				}
				else{
					fprintf(fp, "NAN\t");
				}
			}
			fprintf(fp, "\r\n");
		}
		fclose(fp);
	}

	// 输出表格到屏幕
	template<typename T>
	void Figures<T>::print2scr(){
		for (auto it : figure_){
			printf("%s\t", it.first.c_str());
			for (int j = 0; j < size_; ++j){
				if (it.second[j].mask){
					printf("%.4f\t", it.second[j].val);
				}
				else{
					printf("NAN\t");
				}
			}
			printf("\r\n");
		}
	}

}