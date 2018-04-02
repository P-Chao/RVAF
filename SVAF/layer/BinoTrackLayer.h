/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"
#include "MilTrackLayer.h"

namespace svaf{

class BinoTrackLayer :
	public Layer
{
public:
	explicit BinoTrackLayer(LayerParameter& layer);
	~BinoTrackLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

	static bool	reinit_;

protected:
	bool InitFirstFrame(vector<Block>&, LayerParameter&);
	bool TrackFrame(vector<Block>&);
	void InitSetParam(BoostTrack&);
	void ComputeScale(int);
	void RecoverScale(vector<Block>&, vector<Block>&);

private:
	AdaboostLayer *adaboost;
	pc::BinoTrackFun binotrack_frame, binotrack_firstframe;


	vector<BoostTrack> trackers_;
	int		trackcount_;
	

	Size	trsize_;
	vector<Size> insize_;
	vector<Size> winsize_;
	vector<float> x_factor_;
	vector<float> y_factor_;
	float	scalefactor_;
	

	MilTrackParameter_InitType init_type_;
	MilTrackParameter_TrackType track_type_;

	cv::Rect	init_rect_;
	vector<cv::Rect> init_rects_;
	int		init_negnum_;
	int		negnum_;
	int		posmax_;
	int		numsel_;
	int		numfeat_;
	int		srchwinsz_;
	int		min_rectnum_;
	int		max_rectnum_;
	int		negsample_strat_;

	float	lrate_;
	float	posrad_;
	float	init_posrad_;

	bool	uselogR_;
};

}

