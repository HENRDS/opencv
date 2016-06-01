/* 
 * File:   cap_indoor_pos.cpp
 * Author: henry
 *
 * Created on 30 de Maio de 2016, 19:02
 */

#include "precomp.hpp"


namespace cv 
{
class IndoorPosCapture: public IVideoCapture
{
public:
    virtual ~IndoorPosCapture();
    virtual double getProperty(int) const;
    virtual bool setProperty(int, double);
    virtual bool grabFrame();
    virtual bool retrieveFrame(int, OutputArray);
    virtual bool isOpened() const;
    virtual int getCaptureDomain();
    IndoorPosCapture(const String&);
    
    bool open(const String&);
    void close();
    
private:
    std::vector<String>* opened;
    static FILE* dev_file;
    static unsigned int use_count;
};

IndoorPosCapture::IndoorPosCapture(const String& filename)
{
    this->open(filename);
}

IndoorPosCapture::~IndoorPosCapture() 
{
    this->close();
}

bool IndoorPosCapture::isOpened() const {
    return dev_file != 0;
}

bool IndoorPosCapture::open(const String& filename) 
{
    if (!isOpened()) 
        dev_file = fopen(filename.c_str(), "rb");
    if (dev_file != 0)
        use_count++;
    return dev_file != 0;
}

void IndoorPosCapture::close() {
    if (isOpened()) {
        fclose(dev_file);
        use_count--;
    }
}


}