/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include "feature.h"
#include "classifier.h"

namespace pc{

//extern TrackParam trparam;

void sort_order_des(vector<float>& v, vector<int>& order){
	const uint n = (uint)v.size();
	order.resize(n);
	for (int i = 0; i < n; i++){
		order[i] = i;
	}
	for (int i = 0; i < n; i++){
		for (int j = 0; j < n - i - 1; j++){
			if (v[j] > v[j + 1]){
				swap(v[j], v[j + 1]);
				swap(order[j], order[j + 1]);
			}	
		}
	}
}

vector<float> milclassify(vector<Weak>& weakclf, vector<pc::Rect> sampleset, 
	vector<vector<float>> ftrVal, fimg ii_img, bool uselogR, TrackParam& trparam){
	int numsamples = sampleset.size();
	vector<float> res(numsamples);
	vector<float> tr;

//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
	for (uint w = 0; w < trparam.selectors.size(); w++){
		tr = online_classifysetf(weakclf[trparam.selectors[w]], sampleset, ftrVal, ii_img, false);
		for (int j = 0; j < numsamples; j++)
			res[j] += tr[j];
	}

	if (!uselogR){
		for (int j = 0; j < res.size(); j++){
			res[j] = sigmoid(res[j]);
		}
	}

	return res;
}

vector<float> milclassify_mc(vector<Weak>& weakclf, vector<pc::Rect> sampleset,
	vector<vector<float>> ftrVal, vector<fimg>& ii_img, bool uselogR, TrackParam& trparam){
	int numsamples = sampleset.size();
	vector<float> res(numsamples);
	vector<float> tr;

//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
	for (uint w = 0; w < trparam.selectors.size(); w++){
		tr = online_classifysetf_mc(weakclf[trparam.selectors[w]], sampleset, ftrVal, ii_img, false);
		for (int j = 0; j < numsamples; j++)
			res[j] += tr[j];
	}

	if (!uselogR){
		for (int j = 0; j < res.size(); j++){
			res[j] = sigmoid(res[j]);
		}
	}

	return res;
}

void mil_trainupdate(uimg img, fimg ii_img, vector<Weak>& weakclf, 
	vector<Haar*>& ftrs, vector<pc::Rect>& posx, vector<pc::Rect>& negx, TrackParam& trparam){
	const int numneg = negx.size();
	const int numpos = posx.size();
	const int numftr = ftrs.size();

	//计算特征  得到 特征矩阵 ftrVals [ftr][sample]
	vector<vector<float>> posftrVal, negftrVal;
	posftrVal = computefeature_set(img, ii_img, posx, ftrs);
	negftrVal = computefeature_set(img, ii_img, negx, ftrs);

	static vector<float> Hpos, Hneg;
	Hpos.clear(); Hneg.clear();
	Hpos.resize(posx.size(), 0.0f); Hneg.resize(negx.size(), 0.0f);

	trparam.selectors.clear();
	vector<float> posw(posx.size()), negw(negx.size());
	vector<vector<float>> pospred(weakclf.size()), negpred(weakclf.size());
	
	// 不考虑权重的 训练所有弱分类器
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int m = 0; m < numftr; m++){														
		online_trainupdate(weakclf[m], posx.size(), negx.size(), posftrVal, negftrVal); 	
		pospred[m] = online_classifysetf(weakclf[m], posx, posftrVal, ii_img, true); 
		negpred[m] = online_classifysetf(weakclf[m], negx, negftrVal, ii_img, true); 
	}
	
	// 选择最优特征
	const int numSel = trparam.numSel;	//50;
	for (int s = 0; s < numSel; s++){

		vector<float> poslikl(weakclf.size(), 1.0f), neglikl(weakclf.size()), likl(weakclf.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int w = 0; w < (int)weakclf.size(); w++){
			float lll = 1.0f;
			for (int j = 0; j < numpos; j++){
				lll *= (1 - sigmoid(Hpos[j] + pospred[w][j]));
				//printf("%f,%f, %f\n", lll, Hpos[j], pospred[w][j]);
			}
				
			poslikl[w] = (float)-log(1 - lll + 1e-5);
			

			lll = 0.0f;
			for (int j = 0; j < numneg; j++)
				lll += (float)-log(1e-5f+1-sigmoid(Hneg[j]+negpred[w][j]));
			neglikl[w] = lll;

			likl[w] = poslikl[w] / numpos + neglikl[w] / numneg;	

			//printf("%f,%f,%f\n",poslikl[w],neglikl[w], likl[w]);
		}

		if (false){
			FILE *fp_pos = fopen("tmp/mil_pos.txt","a+");
			FILE *fp_neg = fopen("tmp/mil_neg.txt","a+");
			for (int w = 0; w < weakclf.size(); ++w){
				fprintf(fp_pos, "%f\t", poslikl[w]);
				fprintf(fp_neg, "%f\t", neglikl[w]);
			}
			fprintf(fp_pos, "\r\n");
			fprintf(fp_neg, "\r\n");
			fclose(fp_pos);
			fclose(fp_neg);
		}

		// 选出最优的弱分类器
		vector<int> order;
		sort_order_des(likl, order);
		for (uint k = 0; k < order.size(); k++){
			if (count(trparam.selectors.begin(), trparam.selectors.end(), order[k]) == 0){	
				trparam.selectors.push_back(order[k]);										
				break;
			}
		}

		// 更新 H = H + h_m																	
		for (int k = 0; k < posx.size(); k++)
			Hpos[k] += pospred[trparam.selectors[s]][k];
		for (int k = 0; k < negx.size(); k++)
			Hneg[k] += negpred[trparam.selectors[s]][k];
	}
	
	return;
}

vector<float> adaclassify(vector<Weak>& weakclf, vector<pc::Rect> sampleset, 
	vector<vector<float>> ftrVal, fimg ii_img, bool uselogR, TrackParam& trparam){
	int numsamples = sampleset.size();
	vector<float> res(numsamples);
	vector<bool> tr;

	for (uint w = 0; w < trparam.selectors.size(); w++){
		tr = online_classifyset(weakclf[trparam.selectors[w]], sampleset, ftrVal, ii_img, false);
		for (int j = 0; j < numsamples; j++)
			res[j] += tr[j] ? trparam.alphas[w] : -trparam.alphas[w];
	}

	if (!uselogR){
		for (int j = 0; j < res.size(); j++){
			res[j] = sigmoid(2*res[j]);
		}
	}

	return res;
}

void ada_trainupdate(uimg img, fimg ii_img, vector<Weak>& weakclf, vector<Haar*>& ftrs, 
	vector<pc::Rect>& posx, vector<pc::Rect>& negx, TrackParam& trparam){
	const int numpts = posx.size() + negx.size();
	const int numFeat = ftrs.size();
	const int numSel = trparam.numSel;

	vector<vector<float>> posftrVal, negftrVal;
	posftrVal = computefeature_set(img, ii_img, posx, ftrs);
	negftrVal = computefeature_set(img, ii_img, negx, ftrs);

	vector<float> poslam(posx.size(), .5f / posx.size());
	vector<float> neglam(negx.size(), .5f / negx.size());
	vector<vector<bool>> pospred(numFeat);
	vector<vector<bool>> negpred(numFeat);
	vector<float> errs(numFeat);
	vector<int> order(numFeat);

	float sumAlpha = 0.0f;
	trparam.selectors.clear();

	for (int m = 0; m < numFeat; m++){
		online_trainupdate(weakclf[m], posx.size(), negx.size(), posftrVal, negftrVal);
		pospred[m] = online_classifyset(weakclf[m], posx, posftrVal, ii_img, true); 
		negpred[m] = online_classifyset(weakclf[m], negx, negftrVal, ii_img, true);
	}
	
	vector<int> worstins;
	for (int t = 0; t < numSel; t++){
		for (int k = 0; k < numFeat; k++){
			for (int j = 0; j < (int)poslam.size(); j++){
				(pospred[k][j]) ? (trparam.countTPv[t][k] += poslam[j])
					: (trparam.countFNv[t][k] += poslam[j]);
			}
		}
		for (int k = 0; k < numFeat; k++){
			for (int j = 0; j < (int)neglam.size(); j++){
				(!negpred[k][j]) ? (trparam.countTNv[t][k] += neglam[j])
					: (trparam.countFPv[t][k] += neglam[j]);
			}
		}
		for (int k = 0; k < numFeat; k++){
			errs[k] = (trparam.countFPv[t][k] + trparam.countFNv[t][k])
				/ (trparam.countFPv[t][k] + trparam.countFNv[t][k] 
				+ trparam.countTPv[t][k] + trparam.countTNv[t][k]);
		}

		float minerr;
		uint bestidx;
		sort_order_des(errs, order);

		for (uint k = 0; k < order.size(); k++){
			if (count(trparam.selectors.begin(), trparam.selectors.end(), order[k]) == 0){
				trparam.selectors.push_back(order[k]);
				minerr = errs[k];
				bestidx = order[k];
				break;
			}
		}

		worstins.push_back(order[order.size() - 1]);

		trparam.alphas[t] = max(0.0f, min(0.5f * log((1-minerr)/(minerr + 0.00001f)), 10.0f));
		sumAlpha += trparam.alphas[t];
		
		float corw = 1 / (2 - 2 * minerr);
		float incorw = 1 / (2 * minerr);
		for (int j = 0; j < poslam.size(); j++)
			poslam[j] *= (pospred[bestidx][j] == 1) ? corw : incorw;
		for (int j = 0; j < neglam.size(); j++)
			neglam[j] *= (negpred[bestidx][j] == 0) ? corw : incorw;
	}

	trparam.numsamples += numpts;
	return;
}

void mil_trainupdate_mc(vector<uimg>& img, vector<fimg>& ii_img, vector<Weak>& weakclf,
	vector<Haar*>& ftrs, vector<pc::Rect>& posx, vector<pc::Rect>& negx, TrackParam& trparam){
	const int numneg = negx.size();
	const int numpos = posx.size();
	const int numftr = ftrs.size();

	//计算特征  得到 特征矩阵 ftrVals [ftr][sample]
	vector<vector<float>> posftrVal, negftrVal;
	posftrVal = computefeature_set_mc(img, ii_img, posx, ftrs);
	negftrVal = computefeature_set_mc(img, ii_img, negx, ftrs);

	static vector<float> Hpos, Hneg;
	Hpos.clear(); Hneg.clear();
	Hpos.resize(posx.size(), 0.0f); Hneg.resize(negx.size(), 0.0f);

	trparam.selectors.clear();
	vector<float> posw(posx.size()), negw(negx.size());
	vector<vector<float>> pospred(weakclf.size()), negpred(weakclf.size());

	// 不考虑权重的 训练所有弱分类器
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int m = 0; m < numftr; m++){
		online_trainupdate(weakclf[m], posx.size(), negx.size(), posftrVal, negftrVal);
		pospred[m] = online_classifysetf_mc(weakclf[m], posx, posftrVal, ii_img, true);
		negpred[m] = online_classifysetf_mc(weakclf[m], negx, negftrVal, ii_img, true);
	}

	// 选择最优特征
	int numSel = trparam.numSel;	//50;
	for (int s = 0; s < numSel; s++){

		vector<float> poslikl(numftr, 1.0f), neglikl(numftr), likl(numftr);
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int w = 0; w < numftr; w++){
			float lll = 1.0f;
			for (int j = 0; j < numpos; j++){
				lll *= (1 - sigmoid(Hpos[j] + pospred[w][j]));
				//printf("%f,%f, %f\n", lll, Hpos[j], pospred[w][j]);
			}

			poslikl[w] = (float)-log(1 - lll + 1e-5);


			lll = 0.0f;
			for (int j = 0; j < numneg; j++)
				lll += (float)-log(1e-5f + 1 - sigmoid(Hneg[j] + negpred[w][j]));
			neglikl[w] = lll;

			likl[w] = poslikl[w] / numpos + neglikl[w] / numneg;

			//printf("%f,%f,%f\n",poslikl[w],neglikl[w], likl[w]);
		}

		// 选出最优的弱分类器
		vector<int> order;
		sort_order_des(likl, order);
		const int ordersize = order.size();
		for (uint k = 0; k < ordersize; k++){
			if (count(trparam.selectors.begin(), trparam.selectors.end(), order[k]) == 0){
				trparam.selectors.push_back(order[k]);
				break;
			}
		}

		// 更新 H = H + h_m	
		for (int k = 0; k < numpos; k++)
			Hpos[k] += pospred[trparam.selectors[s]][k];
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int k = 0; k < numneg; k++)
			Hneg[k] += negpred[trparam.selectors[s]][k];
	}

	return;
}



}
