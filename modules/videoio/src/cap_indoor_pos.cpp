/* 
 * File:   cap_indoor_pos.cpp
 * Author: henry
 *
 * Created on 30 de Maio de 2016, 19:02
 */
#include "precomp.hpp"
#include <sys/mman.h>
#include <fcntl.h>

#include <InPosStructures.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <system_error>


namespace cv {

class InPosPollingThread
{
public:
    InPosPollingThread(InPosImage *);
    ~InPosPollingThread();
    void stop();
    bool get(OutputArray, unsigned int *);
    
private:
    std::mutex* frame_lock;
    std::thread* worker_thread;
    Mat* current_frame;
    InPosImage* mapped_buff;
    
    unsigned int index;
    volatile bool should_stop;
    void poll();
    inline void copy_from_mmap();
};

InPosPollingThread::InPosPollingThread(InPosImage* buffAddr)
{
    this->mapped_buff = buffAddr;
    this->frame_lock = new Mutex();
    this->worker_thread = new std::thread(poll);
    this->index = 0
    this->should_stop = false;
    this->current_frame = new Mat(Size(INPOS_MAT_WIDTH, INPOS_MAT_HEIGHT), CV_8UC1);
}

InPosPollingThread::~InPosPollingThread()
{
    this->stop();
    delete worker_thread;
    if (current_frame)
        delete current_frame;
    delete frame_lock;
}
inline void InPosPollingThread::copy_from_mmap() 
{
    for (int i = 0; i < INPOS_MAT_HEIGHT; i++) 
        for (int j = 0; i < INPOS_MAT_WIDTH; j++)
            Mat.at<uchar>(i, j) = this->mapped_buff->img[i][j];
    
}
void InPosPollingThread::poll() 
{
    while(!should_stop)
    {
        while(!(mapped_buff->frameProduced || should_stop))
            std::this_thread::yield();
        if (should_stop) break;
        this->mapped_buff->canTransfer = true;
        
        while(!(this->mapped_buff->canConsume || should_stop)) 
            std::this_thread::yield();
        if (should_stop) break;
        
        this->frame_lock->lock();            
        copy_from_mmap();
        this->mapped_buff->canConsume = false;
        index++;
        this->frame_lock->unlock();
    }
}

void InPosPollingThread::stop()
{
    this->should_stop = true;
    try 
    {
        this->worker_thread->join();
    } catch (const std::system_error& e) {  }
}

bool InPosPollingThread::get(OutputArray image, unsigned int *frame_index) 
{
    if (should_stop) 
    {
        image= noArray();
        return false;
    }
    while ((*frame_index) == index)
        std::this_thread::yield();
    this->frame_lock->lock();
    current_frame->copyTo(image);
    *frame_index = index;
    this->frame_lock->unlock();   
    return true;
}


class IndoorPosCapture: public IVideoCapture
{
public:
    virtual ~IndoorPosCapture();
    virtual double getProperty(int) const;
    virtual bool setProperty(int, double);
    virtual bool grabFrame();
    virtual bool retrieveFrame(int, OutputArray);
    virtual bool isOpened() const;
    virtual int getCaptureDomain() { return CAP_INPOS; }
    IndoorPosCapture(const String&);
    
    bool openFile(const String&);
    void closeFile();
private: /* static */
    static InPosImage* buffer;
    static InPosPollingThread * poll_thread;
    static int file_desc;
    static volatile unsigned int usage_count;
    
private: /* instance */
    Mat grab_buff;
    unsigned int frame_index;
    double people_cnt;
    int getPeopleCnt();
};

IndoorPosCapture::IndoorPosCapture(const String& filename)
{
    this->openFile(filename);
    this->frame_index = 0;
}

IndoorPosCapture::~IndoorPosCapture() 
{
    this->closeFile();
}

bool IndoorPosCapture::isOpened() const 
{
    return file_desc > 0;
}

bool IndoorPosCapture::openFile(const String& filename) 
{
    if (!isOpened()) {
        file_desc = open(filename, O_WRONLY);
        if (file_desc <= 0)
            return false;
        
        buffer = (InPosImage*)mmap(0, sizeof(InPosImage), PROT_READ | PROT_WRITE, MAP_SHARED, file_desc, 0);
        if (buffer == MAP_FAILED)
            return false;
            
        poll_thread = new InPosPollingThread(buffer);
    }
    CV_XADD(usage_count, 1);
    return isOpened();
}

void IndoorPosCapture::closeFile() 
{
    CV_XADD(usage_count, -1);
    
    if (isOpened() && (usage_count == 0)) {
        delete poll_thread;
        close(file_desc);
    }
}

bool IndoorPosCapture::grabFrame()
{
    if (isOpened()) {
        return poll_thread->get(grab_buff, &frame_index);
    }
    return false;
}

int IndoorPosCapture::getPeopleCnt() {
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
    detector->detect(this->grab_buff, keypoints);
    return keypoints.size();
}

double IndoorPosCapture::getProperty(int propID) const 
{
    switch (propID)
    {
        case CAP_PROP_FRAME_WIDTH:
            return INPOS_MAT_WIDTH;
        case CAP_PROP_FRAME_HEIGHT:
            return INPOS_MAT_HEIGHT;
        case CAP_PROP_FPS:
            return INPOS_FRAME_RATE;
        case CAP_PROP_PEOPLE_CNT:
            return ceil(this->people_cnt);
        default:
            return 0;
    }
}

bool IndoorPosCapture::retrieveFrame(int flags, OutputArray image) 
{
    if (isOpened() && (!grab_buff.empty())) {
        grab_buff.copyTo(image);
        this->people_cnt = (this->people_cnt * 0.8) + (this->getPeopleCnt() * 0.2);
        return true;
    }
    image = NULL;
    return false;
}

bool IndoorPosCapture::setProperty(int propID, double value) 
{
    //There are no properties that can be set for this device. At least not yet.
    return false; 
}

Ptr<IVideoCapture> createIndoorPosCapture(const String& filename)
{
    Ptr<IndoorPosCapture> inpos(new IndoorPosCapture(filename));
    if (inpos->isOpened())
        return inpos;
    return Ptr<IndoorPosCapture>();
}
class CV_EXPORTS InPosVideoCapture {
public:
  CV_WRAP  InPosVideoCapture(const String& filename);
  CV_WRAP  InPosVideoCapture(const String& filename, int apireference);
  CV_WRAP  InPosVideoCapture(int index);
  CV_WRAP ~InPosVideoCapture();
  
  CV_WRAP int getPeopleCount();
  CV_WRAP bool read(OutputArray image);
  CV_WRAP bool grab();
  CV_WRAP bool retrieve(OutputArray image, int flag=0);
  CV_WRAP double get(int propId) const;
private:
  Ptr<VideoCapture> videocap;  
};
}