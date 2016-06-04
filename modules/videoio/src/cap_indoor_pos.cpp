/* 
 * File:   cap_indoor_pos.cpp
 * Author: henry
 *
 * Created on 30 de Maio de 2016, 19:02
 */
#include "precomp.hpp"
#include <sys/mman.h>
#include <fcntl.h>

#include <math.h>
#include <mutex>
#include <system_error>

namespace cv 
{
struct IndoorPosImg
{
    unsigned char img[45][80];
    volatile bool canRead;
};    

class InPosPollingThread
{
public:
    InPosPollingThread(IndoorPosImg *);
    ~InPosPollingThread();
    void stop();
    void get(OutputArray);
    
private:
    std::mutex* frame_lock;
    std::thread* worker_thread;
    Mat* current_frame;
    IndoorPosImg* mapped_buff;
    
    int index;
    volatile bool polling;
    volatile bool should_stop;
    void poll();
    inline void copy_from_mmap();
};

InPosPollingThread::InPosPollingThread(IndoorPosImg* buffAddr)
{
    this->mapped_buff = buffAddr;
    this->frame_lock = new Mutex();
    this->worker_thread = new std::thread(poll);
    this->current_frame = new Mat(Size(80, 45), CV_8UC1);
}

InPosPollingThread::~InPosPollingThread()
{
    this->stop();
    if (current_frame)
        delete current_frame;
    delete frame_lock;
}
inline void InPosPollingThread::copy_from_mmap() 
{
    this->frame_lock->lock();
    for (int i = 0; i < 45; i++) 
        for (int j = 0; i < 80; j++)
            Mat.at<uchar>(i, j) = this->mapped_buff[index].img[i][j];
    this->frame_lock->unlock();
}
void InPosPollingThread::poll() 
{
    while(!should_stop)
    {
        while(!(this->mapped_buff[index].canRead || should_stop)) 
            std::this_thread::yield();
        if (should_stop)
            break;
        copy_from_mmap();
        this->mapped_buff[index].canRead = false;
        index++;
    }
}

void InPosPollingThread::stop()
{
    this->should_stop = true;
    try 
    {
        this->worker_thread->join();
    } catch (const std::system_error& e)
    {}
}

void InPosPollingThread::get(OutputArray image) 
{
    if (!polling) {
        image= noArray();
        return;
    }
    while (!current_frame) 
        std::this_thread::yield();
    this->frame_lock->lock();
    current_frame->copyTo(image);
    this->frame_lock->unlock();    
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
    static IndoorPosImg* buffer;
    static InPosPollingThread * poll_thread;
    static int file_desc;
    static unsigned int use_count;
    
private: /*instance*/
    Mat grab_buff;
};

IndoorPosCapture::IndoorPosCapture(const String& filename)
{
    this->openFile(filename);
}

IndoorPosCapture::~IndoorPosCapture() 
{
    this->closeFile();
}

bool IndoorPosCapture::isOpened() const {
    return file_desc != -1;
}

bool IndoorPosCapture::openFile(const String& filename) 
{
    if (!isOpened()) {
        this->file_desc = open(filename, O_WRONLY);
        if (this->file_desc == -1)
            return false;
        
        buffer = (IndoorPosImg*)mmap(0, sizeof(IndoorPosImg) * 3, PROT_READ | PROT_WRITE, MAP_SHARED, file_desc,0);
        if (buffer == MAP_FAILED)
            return false;
            
        this->poll_thread = new InPosPollingThread(buffer);
    }

    CV_XADD(use_count, 1);
    return file_desc != 0;
}

void IndoorPosCapture::closeFile() {
    if (isOpened()) {
        delete this->poll_thread;
        munmap(this->buffer, sizeof(IndoorPosImg) * 3);
        if (close(file_desc) == -1)
            CV_ERROR(-1, "Cannot close IndoorPos file!");  // IDK... dangerous
    }
    CV_XADD(use_count, -1);
}

bool IndoorPosCapture::grabFrame()
{
    if (isOpened()) {
        this->poll_thread->get(grab_buff);
        return true;
    }
    return false;
}

double IndoorPosCapture::getProperty(int propID) const 
{
    switch (propID)
    {
        case CAP_PROP_FRAME_WIDTH:
            return 80.0;
        case CAP_PROP_FRAME_HEIGHT:
            return 45.0;
        case CAP_PROP_FPS:
            return 0.5;
        default:
            return 0;
    }
}

bool IndoorPosCapture::retrieveFrame(int flags, OutputArray image) 
{
    if (isOpened() && (!grab_buff.empty())) {
        grab_buff.copyTo(image);
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

}