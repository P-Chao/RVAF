#include <stdio.h>
#include "string.h"

//#include "
#include "math.h"
#include "DataIO.h"
#include "ImageIO.h"
#include "TimeStat.h"

#include "DataTypeConvert.h"
#include "ImageIer.h"
#include "ImageMatchEadp.h"
#include "ImageMatchSgm.h"
#include "ImageRegionFeature.h"
#include "ImageRegionClassify.h"

void TestImageResize(int argc, char** argv)
{
    char *srcdirname = argv[1];
    char *dstdirname = argv[2];
    char *imagename = argv[3];
    int s = atoi(argv[4]);

    IMAGE_S srcImage8;
    IMAGE_S smallImage;
    int height, width, channel;
    int imagesize;
    char filename[FILE_NAME_LENGTH];

    // 读取图像
    sprintf(filename, "%s/%s%s", srcdirname, imagename, ".png");
    ImageRead(filename, &srcImage8);

    height = srcImage8.height;
    width = srcImage8.width;
    channel = srcImage8.channel;
    imagesize = height*width*channel;
    if (height < 0 || width < 0)    return;

    smallImage.data = MallocType(PIXEL, imagesize);
{
    PIXEL *src = srcImage8.data;
    PIXEL *dst = smallImage.data;

    int dsth = height/s;
    int dstw = width/s;
    int i, j;
    int s1, s2;
    PIXEL *ps, *p;
    PIXEL *q;

    q = dst;
    ps = src;
    s1 = s*width*channel;
    s2 = s*channel;
    for (i = 0;i < dsth;i++, ps += s1)
    {
        p = ps;
        for (j = 0;j < dstw;j++, p += s2, q += channel)
        {
            memcpy(q, p, channel*sizeof(PIXEL));
        }
    }
    
    smallImage.height = dsth;
    smallImage.width = dstw;
    smallImage.channel = channel;
}

    DirCreateRecursion(dstdirname);    
    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".png");
    ImageSave(filename, &smallImage, NULL);

    free(srcImage8.data);
    free(smallImage.data);

    return;
}

void TestLabelResize(int argc, char** argv)
{
    char *srcdirname = argv[1];
    char *dstdirname = argv[2];
    char *imagename = argv[3];
    int s = atoi(argv[4]);

    int *labelImage;
    int *smallImage;
    int height, width, channel;
    char filename[FILE_NAME_LENGTH];

    // 读取图像
    sprintf(filename, "%s/%s%s", srcdirname, imagename, ".txt");
    int flag = GetMatrixDataSize(filename, &height, &width);
    if (FALSE == flag)  return;
    channel = 1;
    labelImage = ReadMatrixInt2(filename, height, width);
    smallImage = MallocType(int, height*width*channel);

    int *src = labelImage;
    int *dst = smallImage;
    int dsth = height/s;
    int dstw = width/s;

{ 
    int i, j;
    int s1, s2;
    int *ps, *p;
    int *q;

    q = dst;
    ps = src;
    s1 = s*width*channel;
    s2 = s*channel;
    for (i = 0;i < dsth;i++, ps += s1)
    {
        p = ps;
        for (j = 0;j < dstw;j++, p += s2, q += channel)
        {
            memcpy(q, p, channel*sizeof(int));
        }
    }
}

    DirCreateRecursion(dstdirname);    
    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".txt");
    SaveMatrixInt(filename, smallImage, dsth, dstw);

    free(labelImage);
    free(smallImage);

    return;
}

void TestDataShow(int argc, char** argv)
{
    char *srcfile = argv[1];
    char *dstfile = argv[2];
    int s = atoi(argv[3]);

    int i;
    int *data;
    IMAGE_S image;
    int height, width, channel;
    int imagesize;

    // 读取图像
    int flag = GetMatrixDataSize(srcfile, &height, &width);
    if (FALSE == flag)  return;
    channel = 1;
    imagesize = height*width*channel;
    data = ReadMatrixInt2(srcfile, height, width);
    image.data = MallocType(PIXEL, imagesize);
    image.height = height;
    image.width = width;
    image.channel = channel;
    for (i = 0;i < imagesize;i++)
    {
        image.data[i] = (PIXEL)(data[i]*s);
    }
    ImageSave(dstfile, &image, NULL);

    free(data);
    free(image.data);

    return;
}

void TestImage2Ccs(int argc, char** argv)
{
    char *srcdirname = argv[1];
    char *dstdirname = argv[2];
    char *imagename = argv[3];

    IMAGE_S srcImage8;
    int height, width, channel;
    int imagesize;
    int ccssize;
    CcsData *ccs;
    char filename[FILE_NAME_LENGTH];

    // 读取图像
    sprintf(filename, "%s/%s%s", srcdirname, imagename, ".png");
    ImageRead(filename, &srcImage8);

    height = srcImage8.height;
    width = srcImage8.width;
    channel = srcImage8.channel;
    imagesize = height*width*channel;
    if (height < 0 || width < 0)    return;
    ccssize = height*width*channel/4;
    ccs = MallocType(CcsData, ccssize);
    Pixel2Ccs(srcImage8.data, height*width*channel, ccs);

    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".dat");
    SaveCcsData(filename, ccs, ccssize);

    free(ccs);
    free(srcImage8.data);

    return;
}

void TestImageSupixSeg(int argc, char** argv)
{
#if 0
    char *srcdirname = argv[1];
    char *dstdirname = argv[2];
    char *imagename = argv[3];
#else
    char *srcdirname = ".";
    char *dstdirname = ".";
    char *imagename = "0";
#endif

    int K = 800;
    int M = 10;
    int optint = 1;

    IMAGE_S srcImage8;
    IerInfo ier;
    int height, width, channel;
    char filename[FILE_NAME_LENGTH];

    // 读取图像
    sprintf(filename, "%s/%s%s", srcdirname, imagename, ".bmp");
    ImageRead(filename, &srcImage8);  
    height = srcImage8.height;
    width = srcImage8.width;
    channel = srcImage8.channel;
    if (height < 0 || width < 0)    return;

    // 参数配置
    ImageIerPara(&ier);
    ier.optint = optint;
    ier.height = height;
    ier.width = width;
    ier.channel = channel;
    ier.imagecolori = srcImage8.data;
    ier.K = K;
    ier.normC = M;
    ier.maxK = 2*K;
    ImageIerInit(&ier);

    // IER
    IerPixelInit(&ier);
    IerPixelRefine(&ier);

    // 保存
    DirCreateRecursion(dstdirname);  //递归创建目录，这个函数的作用在于可以根据/来创建文件夹

    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".seg");
    SaveMatrixInt(filename, ier.label, height, width);

    ImageSegShow(&srcImage8, ier.label, NULL, &srcImage8);
    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".png");
    ImageSave(filename, &srcImage8, NULL);

    // 释放内存
    free(srcImage8.data);
    ImageIerDestroy(&ier);
}

void SuperPixelSegment(IplImage* pIamge, int K){
	//int K = 1600;
	int M = 10;
	int optint = 1;

	IMAGE_S srcImage8;
	IerInfo ier;
	int height, width, channel;

	//ImageRead(filename, &srcImage8);
	//IplImage* pIamge = cvLoadImage(filename, CV_LOAD_IMAGE_UNCHANGED);

	srcImage8.width = srcImage8.height = 0;
	srcImage8.channel = IMAGE_SEQ_NORMAL;
	if (NULL == pIamge)
	{
		//ImageReadFree(filename, img, 0);
		return;
	}
	else
	{
		int size = pIamge->height*pIamge->width*pIamge->nChannels;
		srcImage8.data = MallocType(PIXEL, size);

		ImageOpenCv2Hustsong(pIamge, &srcImage8);

		cvReleaseImage(&pIamge);
	}

	height = srcImage8.height;
	width = srcImage8.width;
	channel = srcImage8.channel;
	if (height < 0 || width < 0)    return;

	// 参数配置
	ImageIerPara(&ier);
	ier.optint = optint;
	ier.height = height;
	ier.width = width;
	ier.channel = channel;
	ier.imagecolori = srcImage8.data;
	ier.K = K;
	ier.normC = M;
	ier.maxK = 2 * K;
	ImageIerInit(&ier);

	// IER
	IerPixelInit(&ier);
	IerPixelRefine(&ier);

	// 保存
	
	//SaveMatrixInt("a.seg", ier.label, height, width);

	ImageSegShow(&srcImage8, ier.label, NULL, &srcImage8);
	
	ImageShow("result", &srcImage8, NULL);

	// 释放内存
	free(srcImage8.data);
	ImageIerDestroy(&ier);
}

void TestImageMatchEadp(int argc, char** argv)
{
#if 0
    char *srcdir = argv[1];
    char *dispname = argv[2];
    char *imagename = argv[3];
#else
    //char *srcdir = "H:/实景实验DSP";
    char *dispname = "eadpleft";
    //char *imagename = "07";
	char *databasedir = "F:/data/patch";
	char *databasename = "";
#endif

    int dlength = 24;//24
    int factor = 10*256;

    int guildmr = 1;
    int dispmr = 1;
    floatcost sg = -25.0;
    floatcost sc = 0.1*255;
    floatcost r1 = 10;
    floatcost r2 = 500;

    IMAGE32_S dispimage;
    IMAGE_S imageL, imageR;
    ImEadpInfo eadp;

    char dstdirname[FILE_NAME_LENGTH];
    char filename[FILE_NAME_LENGTH];
    int height, width, channel, area;

	sprintf(filename, "%s/%s/%s%s", databasedir, databasename, "left3", ".bmp");
    ImageRead(filename, &imageL);
	sprintf(filename, "%s/%s/%s%s", databasedir, databasename, "right3", ".bmp");
    ImageRead(filename, &imageR);

    height = imageL.height;
    width = imageL.width;
    channel = imageL.channel;
    area = height*width;
    if (height < 0 || width < 0)    return;

    // 初始化
    ImageMatchEadpPara(&eadp);
    eadp.height = height;
    eadp.width = width;
    eadp.channel = channel;
    eadp.dlength = dlength;
    eadp.src[0] = &imageL;
    eadp.src[1] = &imageR;
    eadp.guildmr = guildmr;
    eadp.dispmr = dispmr;
    eadp.sg = sg;
    eadp.sc = sc;
    eadp.r1 = r1;
    eadp.r2 = r2;
    ImageMatchEadpInit(&eadp);

    // 视差求解
    ImageMatchEadpProc(&eadp);

    // 保存
	sprintf(dstdirname, "%s/%s/%s", ".", dispname, databasename);
    DirCreateRecursion(dstdirname);

    dispimage.data = MallocType(int, height*width);
    dispimage.height = height;
    dispimage.width = width;
    dispimage.channel = 1;

    ImppDisp2ImagePixel(dispimage.data, eadp.left, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/left.png");
    Image16Save(filename, &dispimage, NULL);

    ImppDisp2ImagePixel(dispimage.data, eadp.right, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/right.png");
    Image16Save(filename, &dispimage, NULL); 

	ImppDisp2ImagePixel(dispimage.data, eadp.sparse, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/check.png");
    Image16Save(filename, &dispimage, NULL); 

	ImppDisp2ImagePixel(dispimage.data, eadp.dense, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/fill.png");
    Image16Save(filename, &dispimage, NULL);

    sprintf(filename, "%s%s", dstdirname, "/left.txt");
    SaveMatrixPixel(filename, eadp.left, height, width);
    
    sprintf(filename, "%s%s", dstdirname, "/right.txt");
    SaveMatrixPixel(filename, eadp.right, height, width);

    sprintf(filename, "%s%s", dstdirname, "/check.txt");
    SaveMatrixPixel(filename, eadp.sparse, height, width);

    sprintf(filename, "%s%s", dstdirname, "/fill.txt");
    SaveMatrixPixel(filename, eadp.dense, height, width);

    free(dispimage.data);

    // 释放内存
    free(imageL.data);
    free(imageR.data);
    ImageMatchEadpDestroy(&eadp);

    return;
}

void TestImageMatchSgm(int argc, char** argv)
{
#if 0
    char *srcdir = argv[1];
    char *dispname = argv[2];
    char *imagename = argv[3];
#else
    char *srcdir = "H:/实景实验DSP";
    char *dispname = "sgmleft";
	//char *imagename = "07";
	char *databasedir = "F:/data/patch";
	char *databasename = "";
#endif

    int dlength = 24;//24
    int factor = 10*256;

    int dispmr = 1;
    floatcost r1 = 10;
    floatcost r2 = 500;

    IMAGE32_S dispimage;
    IMAGE_S imageL, imageR;
    ImSgmInfo sgm;

    char dstdirname[FILE_NAME_LENGTH];
    char filename[FILE_NAME_LENGTH];
    int height, width, channel, area;
    double t = 0;

	sprintf(filename, "%s/%s/%s%s", databasedir, databasename, "left3", ".bmp");
    ImageRead(filename, &imageL);
	sprintf(filename, "%s/%s/%s%s", databasedir, databasename, "right3", ".bmp");
    ImageRead(filename, &imageR);

    height = imageL.height;
    width = imageL.width;
    channel = imageL.channel;
    area = height*width;
    if (height < 0 || width < 0)    return;

    // 初始化
    ImageMatchSgmPara(&sgm);
    sgm.height = height;
    sgm.width = width;
    sgm.channel = channel;
    sgm.dlength = dlength;
    sgm.src[0] = &imageL;
    sgm.src[1] = &imageR;
    sgm.dispmr = dispmr;
    sgm.r1 = r1;
    sgm.r2 = r2;
    ImageMatchSgmInit(&sgm);

    // 视差求解
TsTimeInit();
TsTimeStart(t);
    ImageMatchSgmProc(&sgm);
TsTimeEnd(t);
TsGetTime(t);
    printf("sgm time: %.1f ms\n", t*1000);

    // 保存
	sprintf(dstdirname, "%s/%s/%s", ".", dispname, databasename);
    DirCreateRecursion(dstdirname);

    dispimage.data = MallocType(int, height*width);
    dispimage.height = height;
    dispimage.width = width;
    dispimage.channel = 1;

    ImppDisp2ImagePixel(dispimage.data, sgm.left, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/left.png");
    Image16Save(filename, &dispimage, NULL);

    ImppDisp2ImagePixel(dispimage.data, sgm.right, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/right.png");
    Image16Save(filename, &dispimage, NULL); 

	ImppDisp2ImagePixel(dispimage.data, sgm.sparse, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/check.png");
    Image16Save(filename, &dispimage, NULL); 

	ImppDisp2ImagePixel(dispimage.data, sgm.dense, area, (float)factor);
    sprintf(filename, "%s%s", dstdirname, "/fill.png");
    Image16Save(filename, &dispimage, NULL);

    sprintf(filename, "%s%s", dstdirname, "/left.txt");
    SaveMatrixPixel(filename, sgm.left, height, width);

    sprintf(filename, "%s%s", dstdirname, "/right.txt");
    SaveMatrixPixel(filename, sgm.right, height, width);

    sprintf(filename, "%s%s", dstdirname, "/check.txt");
    SaveMatrixPixel(filename, sgm.sparse, height, width);

    sprintf(filename, "%s%s", dstdirname, "/fill.txt");
    SaveMatrixPixel(filename, sgm.dense, height, width);

    free(dispimage.data);

    // 释放内存
    free(imageL.data);
    free(imageR.data);
    ImageMatchSgmDestroy(&sgm);

    return;
}

void TestImageSupixFeature(int argc, char** argv)
{
    char *srcdir = "H:/实景实验DSP";
    char *srcname = "imgleft";
    char *segname = "segleft";
    char *feaname = "featureleft";

    int channel = 3;
	IrfFeaInfo feainfo;
	IMAGE32_S markImage;
	IMAGE_S srcImage8;
    Matrix_S colorImage;
	double *feature;
    float *feafloat;
    int *featureint;
	int height, width, size;
	int regionnum;
    int feanum;

	FileList *imagefile;
    FileList *p;
    char imgdir[FILE_NAME_LENGTH];
    char feadir[FILE_NAME_LENGTH];
	char filename[FILE_NAME_LENGTH];

	// 初始化
	IrfFeaturePara(&feainfo);
	IrfFeatureInit(&feainfo);
	feanum = IrfFeatureNum(&feainfo, channel);
    feature = MallocType(double, feainfo.maxRegionnum*feanum);
    feafloat = MallocType(float, feainfo.maxRegionnum*feanum);
    featureint = MallocType(int, feainfo.maxRegionnum*feanum);

    size = feainfo.maxHeight*feainfo.maxWidth;
    srcImage8.data = MallocType(PIXEL, size*channel);
	colorImage.data = MallocType(double, size*channel);
	markImage.data = MallocType(int, size);

	// 遍历处理
	sprintf(imgdir, "%s/%s", srcdir, srcname);
    sprintf(feadir, "%s/%s", srcdir, feaname);
	DirCreateRecursion(feadir);
	p = imagefile = FileListSearch(imgdir, "*.png");
	for (p = imagefile;p != NULL;p = p->next)
    {
    	// 载入原始图像
    	ImageRead(p->filename, &srcImage8, size);
        colorImage.height = srcImage8.height;
	    colorImage.width = srcImage8.width;
        colorImage.channel = srcImage8.channel;

		height = colorImage.height;
		width = colorImage.width;
		if (colorImage.channel != channel)
        {
            channel = colorImage.channel;
            feanum = IrfFeatureNum(&feainfo, channel);
        }

        // 转换为浮点图像
        if (height*width > size)
		{
			size = height*width;
            free(colorImage.data);
			free(markImage.data);
			markImage.data = MallocType(int, size);
            colorImage.data = MallocType(double, size*channel);
		}
        DataPixel2Double(srcImage8.data, colorImage.data, width*height*channel);

		// 载入分割掩码图像
        sprintf(filename, "%s/%s/%s%s", srcdir, segname, p->name, ".seg");
		ReadMatrixInt1(filename, markImage.data, height, width);
        markImage.height = height;
        markImage.width = width;
		regionnum = StatMaxInt(markImage.data, width*height, 1) + 1;

		// 提取特征
		if (regionnum > feainfo.maxRegionnum)
		{
			free(feature);
            free(feafloat);
            free(featureint);
			feature = MallocType(double, regionnum*feanum);
            feafloat = MallocType(float, regionnum*feanum);
            featureint = MallocType(int, regionnum*feanum);
		}
		IrfFeatureMemoryAdapt(&feainfo, height, width, regionnum);
		IrfFeatureProc(&feainfo, &markImage, &colorImage, regionnum, feature, feanum);
        IrfFeatureProcInt(&feainfo, &markImage, &srcImage8, regionnum, featureint, feanum);

		// 保存特征
		sprintf(filename, "%s/%s%s", feadir, p->name, ".features.txt");
        DataDouble2Float(feature, feafloat, regionnum*feanum);
		SaveMatrixFloat(filename, feafloat, regionnum, feanum);

        sprintf(filename, "%s/%s%s", feadir, p->name, ".features.int.txt");
		SaveMatrixInt(filename, featureint, regionnum, feanum);
	}

    free(srcImage8.data);
	free(colorImage.data);
	free(markImage.data);
    free(feature);
    free(feafloat);
    free(featureint);
    IrfFeatureDestroy(&feainfo);
    FileListDestroy(&imagefile);

	return;
}

void TestImageSupixClassify(int argc, char** argv)
{
#if 1
    char *srcdir = argv[1];
    char *srcname = argv[2];
    char *segname = argv[3];
    char *dstname = argv[4];
    char *imagename = argv[5];
#else
    char *srcdir = "H:/实景实验DSP";
    char *srcname = "imgleft";
    char *segname = "segleft";
    char *dstname = "classifyleft";
    char *imagename = "01";
#endif

    int channel = 3;
	IrfFeaInfo feainfo;
	IMAGE32_S markImage;
	IMAGE_S srcImage8;
    IMAGE_S lblImage;
    int *featureint;
    int *label;
	int height, width;
	int regionnum;
    int feanum;

    char dstdirname[FILE_NAME_LENGTH];
	char filename[FILE_NAME_LENGTH];

    // 载入图像
    sprintf(filename, "%s/%s/%s%s", srcdir, srcname, imagename, ".png");
	ImageRead(filename, &srcImage8);
    height = srcImage8.height;
    width = srcImage8.width;
    channel = srcImage8.channel;
    if (height < 0 || width < 0)    return;

    // 载入分割掩码图像
    markImage.height = height;
    markImage.width = width;
    markImage.data = MallocType(int, height*width);
    sprintf(filename, "%s/%s/%s%s", srcdir, segname, imagename, ".seg");
	ReadMatrixInt1(filename, markImage.data, height, width);
	regionnum = StatMaxInt(markImage.data, height*width, 1) + 1;

	// 初始化
	IrfFeaturePara(&feainfo);
    feainfo.maxHeight = height;
    feainfo.maxWidth = width;
    feainfo.maxChannel = channel;
    feainfo.maxRegionnum = regionnum;  
	IrfFeatureInit(&feainfo);
	feanum = IrfFeatureNum(&feainfo, channel);
    featureint = MallocType(int, feainfo.maxRegionnum*feanum);  //总特征数量 = 超像素数目 * 特征数目
    lblImage.data = MallocType(PIXEL, height*width);
    label = MallocType(int, regionnum);

    // 求特征
    IrfFeatureProcInt(&feainfo, &markImage, &srcImage8, regionnum, featureint, feanum);

    // 分类
    IrcClassifyImage(&markImage, &lblImage, label, featureint, regionnum);

    // 保存
    sprintf(dstdirname, "%s/%s", srcdir, dstname);
    DirCreateRecursion(dstdirname);
    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".txt");
    SaveMatrixInt(filename, label, regionnum, 1);
    sprintf(filename, "%s/%s%s", dstdirname, imagename, ".png");
    ImageSave(filename, &lblImage, NULL);

    free(label);
    free(srcImage8.data);
	free(markImage.data);
    free(lblImage.data);
    free(featureint);
    IrfFeatureDestroy(&feainfo);

	return;
}

void Test0()
{
    char *srcfile = "H:/实景实验DSP/imgleft/00.png";
    char *dstfile = "H:/实景实验DSP/imgleft/00.gray.png";
    IMAGE_S srcImage;
    IMAGE_S grayImage;
    int height, width;

    ImageRead(srcfile, &srcImage);  
    height = srcImage.height;
    width = srcImage.width;
    if (height < 0 || width < 0)    return;

    grayImage.height = height;
    grayImage.width = width;
    grayImage.data = MallocType(PIXEL, height*width);
    ImageRgb2Gray(&srcImage, &grayImage, NULL);

    ImageSave(dstfile, &grayImage, NULL);

    free(srcImage.data);
	free(grayImage.data);
}


#include "ctmfopt.h"
void Test1()
{
    // char *srcfile = "H:/实景实验DSP/imgleft/00.gray.png";
    char *srcfile = "H:/实景实验DSP/imgleft/00.png";
    char *dstfile = "H:/实景实验DSP/imgleft/00.ctmf.png";
    int r = 1;

    IMAGE_S srcImage;
    IMAGE_S dstImage;
    int height, width, channel;

    ImageRead(srcfile, &srcImage);  
    height = srcImage.height;
    width = srcImage.width;
    channel = srcImage.channel;
    if (height < 0 || width < 0)    return;

    dstImage.height = height;
    dstImage.width = width;
    dstImage.channel = channel;
    dstImage.data = MallocType(PIXEL, height*width*channel);
    
#if 0
    ctmf(srcImage.data, dstImage.data, width, height, width*channel, width*channel, r, channel, height*width*channel);         
    // EadpMedian3x3(&srcImage, &dstImage);
#else
    for (int c = 0;c < channel;c++)
    {
        ctmfopt(srcImage.data+c, dstImage.data+c, height, width, channel, r);
    }
#endif

    ImageSave(dstfile, &dstImage, NULL);

    free(srcImage.data);
	free(dstImage.data);
}


void maint(int argc, char** argv)
{
    // TestImageResize(argc, argv);
    // TestLabelResize(argc, argv);
    // TestDataShow(argc, argv);
    // TestImage2Ccs(argc, argv);
    // TestImageSupixSeg(argc, argv);
    // TestImageMatchEadp(argc, argv);
    TestImageMatchSgm(argc, argv);
    // TestImageSupixFeature(argc, argv);
    // TestImageSupixClassify(argc, argv);
    // Test1();

	system("pause");
}
