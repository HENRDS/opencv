/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "precomp.hpp"
#include "../../imgproc/include/opencv2/imgproc.hpp"
/* VALUES */
#define BLOCK_THRESH 8
#define BLOCK_SIZE 8
#define KERNEL_SIZE 8
#define CLOSE_ITERS 6
#define DIFF_THRESHOLD 30;

/* MACROS */
#define __RECT_ARGS(x, y) (x*BLOCK_SIZE, y*BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE)

namespace cv {
/*------------Block_Processor------------*/     
class Block_Processor {
private:
    static void process_ROI(Mat* roi);
public:
    static void BinaryBlocks(Mat& frame);
};
  
 void Block_Processor::process_ROI(Mat* roi){
    int x = countNonZero(*roi);
    uchar color = x > BLOCK_THRESH ? (uchar)255 : (uchar)0;
    uchar * p;
    for (int i = 0; i < roi->rows; ++i) {
        p = roi->ptr<uchar>(i);
        for (int j = 0; j < roi->cols; ++j) {
            p[j] = color;
        }
    }
    delete roi;
}


void Block_Processor::BinaryBlocks(Mat& frame) {
    //Since the BLOCK_SIZE must be power of 2, it is possible to divide only by executing shifts
    int log2_block_size = (int) log2((double)BLOCK_SIZE);
    int vblocks = (frame.rows >> log2_block_size);
    int hblocks = (frame.cols >> log2_block_size);

    for (int i = 0; i < vblocks; i++) {
        for (int j = 0; j < hblocks; j++) {
            Rect r __RECT_ARGS(j, i);
            process_ROI(new Mat(frame, r));
        }
    }
}
void morph(const _InputOutputArray &frame, int op, int iters) {
    morphologyEx(frame, frame, op,getStructuringElement(MORPH_RECT, Size(KERNEL_SIZE, KERNEL_SIZE)), Point(-1, -1), iters);
}

/*------------Background------------*/      
class IndoorPosBackground {
private:
    Mat* last_frames[3];
    double processTminus(int minus, OutputArray output);
    void calculateMask(InputOutputArray mask);
    inline void push_frame_back(const Mat &frame);
    Mat background_frame;
    
public:
    // Constructor
    IndoorPosBackground();
    // Update background estimation
    void updateBackground(InputArray input, OutputArray background);
};

IndoorPosBackground::IndoorPosBackground() {
    for (int i = 0; i < 3; i++) {
        last_frames[i] = NULL;
    }
}
double IndoorPosBackground::processTminus(int minus, const _OutputArray &output) {
    Mat *current = last_frames[0], *previous= last_frames[minus];
    Mat oMat(current->size(), CV_8UC1);
    absdiff(*current, *previous, oMat);
    Scalar _mean, _stdDev;
    meanStdDev(oMat, _mean, _stdDev);
    output.create(oMat.size(), CV_8UC1);
    oMat.copyTo(output);
    return _mean[0] + _stdDev[0];
}

void IndoorPosBackground::calculateMask(InputOutputArray mask) {
    Mat iMat = mask.getMat();
    Mat tMinus1, tMinus2;
    double max1 = processTminus(1, tMinus1);
    double max2 = processTminus(2, tMinus2);
    uchar *p1, *p2;
    for (int i = 0; i < last_frames[0]->rows; i++) {
        p1= tMinus1.ptr<uchar>(i);
        p2= tMinus2.ptr<uchar>(i);
        for (int j = 0; j < last_frames[0]->cols; j++) {
            if ((p1[j] >= max1 ) && (p2[j] >= max2 ))
                iMat.at<uchar>(i,j) = 255;
            else
                iMat.at<uchar>(i,j) = 0;
        }
    }


    morph(iMat, MORPH_DILATE, 8);
    Block_Processor::BinaryBlocks(iMat);
}

inline void IndoorPosBackground::push_frame_back(const Mat &frame) {
    if (last_frames[0] == NULL) {
        frame.copyTo(this->background_frame);
        for (int i = 0; i < 3; i++) {
            last_frames[i] = new Mat(frame.size(), CV_8UC1);
            frame.copyTo(*last_frames[i]);
        }
    } else {
        delete last_frames[2];
        last_frames[2] = last_frames[1];
        last_frames[1] = last_frames[0];
        last_frames[0] = new Mat(frame.size(), CV_8UC1);
        frame.copyTo(*last_frames[0]);
    }
}
uchar avg(uchar input1, uchar input2, double alpha) {
    return (uchar)(input1*alpha + input2*(1-alpha));
}
void IndoorPosBackground::updateBackground(InputArray input, OutputArray background) {
    //Update the pointers with the three last frames location
    Mat iMat = input.getMat();
    push_frame_back(iMat);

    Mat mask(last_frames[0]->size(), CV_8UC1);
    calculateMask(mask);
    //Only update the background with pixels from the current frame, if the pixels are not considered part of a movement.
    uchar *p, *b;
    for (int i = 0; i < last_frames[0]->rows; i++) {
        p= mask.ptr<uchar>(i);
        b = background_frame.ptr<uchar>(i);
        for (int j = 0; j < last_frames[0]->cols; j++) {
            if (!p[j])
                b[j]= avg(b[j], last_frames[0]->at<uchar>(i,j), 0.2);
        }
    }
    background_frame.copyTo(background);
}
/*------------InPosProcess------------*/  
class InPosProcess {
private:

    Ptr<IVideoCapture> video;
    IndoorPosBackground* background;
    double people_cnt;
    
    void _retrieve(int flags, OutputArray frame);
    int _detectBlobs(InputArray input, InputOutputArray output);

public:
    int getPeopleCount() const { return (int)ceil(people_cnt); }
    void process(int flags, OutputArray output);
    InPosProcess(Ptr<IVideoCapture> video);
};

InPosProcess::InPosProcess(Ptr<IVideoCapture> video) {
    this->video = video;
    this->background = new IndoorPosBackground();
}

void InPosProcess::_retrieve(int flags, const _OutputArray &frame) {
    Mat m;
    video->retrieveFrame(flags,m);
    m.copyTo(frame);
}
void InPosProcess::process(int flags,const _OutputArray &output)
{
    Mat frame;
    Mat back(frame.size(), CV_8UC1);
    Mat mShow(frame.size(), CV_8UC3);

    this->_retrieve(flags,mShow);
    cvtColor(mShow, frame, CV_RGB2GRAY);

    GaussianBlur(frame, frame, Size(21, 21), 0);
    this->background->updateBackground(frame, back);
    absdiff(frame, back, frame);

    threshold(frame, frame, DIFF_THRESHOLD, 255, THRESH_BINARY);
    morph(frame, MORPH_CLOSE, CLOSE_ITERS);
    subtract(Scalar::all(255), frame, frame);
    int x;
    x = _detectBlobs(frame, mShow);
    people_cnt = (people_cnt * 0.8) + (x * 0.2);
    frame.copyTo(output);
}

int InPosProcess::_detectBlobs(const _InputArray &input, InputOutputArray output) {
    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 0;
    params.filterByArea = true;
    params.minArea = 1000;
    params.maxArea = 500000;
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = true;
    params.minInertiaRatio= 0.01;
    params.maxInertiaRatio= 0.99;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    std::vector<KeyPoint> keypoints;
    detector->detect(input, keypoints);
    drawKeypoints(input, keypoints, output, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    return keypoints.size();
       
            
}
    
 /*------------IndoorPosCaptureWrapper------------*/     
class IndoorPosCaptureWrapper : public IVideoCapture
{
public:
    virtual double getProperty(int id) const { return this->_capture->getProperty(id); }
    virtual bool setProperty(int id, double value) { return this->_capture->setProperty(id, value); }
    virtual bool grabFrame() { return this->_capture->grabFrame(); }
    virtual bool retrieveFrame(int flags, OutputArray image) { this->_processor->process(flags, image); return true;}
    virtual bool isOpened() const { return this->_capture->isOpened(); }
    virtual int getCaptureDomain() { return CAP_INPOS_OFFLINE; }
    IndoorPosCaptureWrapper(Ptr<IVideoCapture>);
    ~IndoorPosCaptureWrapper();
private:
    Ptr<IVideoCapture> _capture;
    InPosProcess * _processor;
 };
 IndoorPosCaptureWrapper::IndoorPosCaptureWrapper(Ptr<IVideoCapture> capture)
 {
     this->_capture = capture;
     this->_processor = new InPosProcess(capture);
 }
  
 IndoorPosCaptureWrapper::~IndoorPosCaptureWrapper()
 {
     delete this->_processor;
 }
    
 virtual double IndoorPosCaptureWrapper::getProperty(int id) const { 
    if (id == CAP_PROP_PEOPLE_CNT)
        return 0;
    else
        return this->_capture->getProperty(id); 
 }
 Ptr<IVideoCapture> createIndoorPosOfflineCapture(Ptr<IVideoCapture> cap)
 {
     Ptr<IVideoCapture> capture(new IndoorPosCaptureWrapper(cap));
     return capture;
 }
}