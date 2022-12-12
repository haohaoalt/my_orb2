/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{
//NOTE  八叉树筛选实现 定义了一个 ExtractorNode 类用于进行八叉树分配
class ExtractorNode
{
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
};
// FAST 特征点和 ORB 描述子本身不具有尺度信息, ORBextractor 通过构建图像金字塔来得到特征点尺度信息.将输入图片逐级缩放得到图像金字塔, 金字塔层级越高, 图片分辨率越低, ORB特征点越大.
class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };
    //构造函数
    //NOTE 构造函数的流程：初始化图像金字塔相关变量->初始化用于计算描述子的pattern->计算近似圆形的边界坐标umax
    //NOTE 参数从配置文件*.yaml文件中读入

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);
    //析构函数

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

//访问控制public 图像金字塔每层的图像
    std::vector<cv::Mat> mvImagePyramid;

protected:
    //NOTE 构建图像金字塔ComputePyramid()
    void ComputePyramid(cv::Mat image);
    // NOTE 提取特征点并进行筛选
//简单描述特征点响应值/描述子的区别
//响应值描述的是该特征点的区分度大小:响应值越大的点越应该被留用做特征点，响应值类似于分数，分数越高的学生越好，越应该被保留
//描述子是特征点的哈希运算：大小无意义，仅用来在数据库中快速找到某特征点，描述子相当于学号，系统随机运算出一串数字，用于查找
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    // NOTE 八叉树筛选特征点 == 非极大值抑制 每个省取100个上清华北大
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    //金字塔每层级中提取的特征点数
    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;
    //各层级的缩放系数
    std::vector<float> mvScaleFactor;
    //各层级缩放系数的倒数
    std::vector<float> mvInvScaleFactor;    
    //各层级缩放系数的平方
    std::vector<float> mvLevelSigma2;
    //各层级缩放系数的平方倒数
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

