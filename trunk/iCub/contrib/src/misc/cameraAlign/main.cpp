// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_contrib_modules
 *
 * \defgroup cameraAlign cameraAlign
 *
 * This module has been developed as a tool for cameras alignment along the vertical axis
 * It uses the feedback coming from the user whom is asked to click on an object present on both images 
 * In both the images a horizontal line is drawn and that help in the alignment process

 * \author ReaFrancesco
 */ 


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <cstring>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char **argv) {
    Network yarp;
    std::string fname,tmp;
    if(argc<2){
        //there are no parameters
        // litte help on how to use
        printf("______ HELP ________ \n");
        printf(" \n");
        printf("USER COMMANDS: \n");
        printf("--name (XXXX): defines the name of module \n");
        printf(" \n");
        printf(" \n");
        return 0;
    }
    else{
        //estracts the command from command line
        for (int i=1;i<argc;i++) {
            if ((strcmp(argv[i],"--name")==0)||(strcmp(argv[i],"-n")==0)) {
                fname = argv[++i];
                printf("file name:%s \n",fname.c_str());
            }
        }
    }

    BufferedPort<Bottle> coordPort;

    tmp=fname;
    tmp.append("/coord:i");
    coordPort.open(tmp.c_str());
    BufferedPort<ImageOf<PixelRgb> > imageInPort;
    
    tmp=fname;
    tmp.append("/image:i");
    imageInPort.open(tmp.c_str());

    BufferedPort<ImageOf<PixelRgb> > imageOutPort;
    tmp=fname;
    tmp.append("/image:o");
    imageOutPort.open(tmp.c_str());

    int x=0,y=0;
    while (true) {
        cout << "waiting for input" << endl;
        Bottle *input = coordPort.read(false);
        if(input!=NULL) {
            x=input->get(0).asInt();
            printf("x: %d ", x);
            y=input->get(1).asInt();
            printf("y: %d \n",y);
        }

        ImageOf<PixelRgb>* inImage=imageInPort.read();
        unsigned char* pin=inImage->getRawImage();
        int padding=inImage->getPadding();
        if (inImage!=NULL) {
            ImageOf<PixelRgb>& outImage = imageOutPort.prepare();
            outImage.resize(320,240);
            unsigned char* pout=outImage.getRawImage();
            for(int r=0;r<240;r++) {
                for(int c=0;c<320;c++) {
                    if(r==y){
                        *pout++=255;
                        *pout++=0;
                        *pout++=0;
                    }
                    else {
                        *pout++=*pin++;
                        *pout++=*pin++;
                        *pout++=*pin++;
                    }
                }
                pin+=padding;
                pout+=padding;
            }
            imageOutPort.write();
        }
    }
    return 0;
}
