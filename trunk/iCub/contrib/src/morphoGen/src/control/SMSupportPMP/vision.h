#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

#include "segment.h"

// OpenCV library:
// #include <cv.h> 

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

class VisionSystem
{
	private:
		double a[3][3], det,l1[11],l2[11],bias1[84],weight1[77][2],bias2[3],weight2[2][77],Y1,Y2,Y3,Y4;
		double rw,rx,ImagX,ImagY,ImagZ,FinXpos[2];
		double Sconf[4][3],SconfT[3][4],C[3][3],point[5][4];
		float s[5000];
		int iLocX,iLocY,iLocXL,iLocYL;
		
public:
	VisionSystem();
	~VisionSystem();
	double* Vision(int ObjectType);
	bool ModelLoad();
	int colSegMainR();
	void colSegMainL();
	void threadRelease();
	void init_onsize(int width_, int height_);
	double Determinant(double **a,int n);
	void CoFactor(double **a,int n,double **b);
	void Transpose(double **a,int n);
	void ConvImg(double UUC1,double UUV1,double UUC2,double UUV2);
	void RBF(double UUC1,double UUV1,double UUC2,double UUV2);
	void GCamSceneR(int ActCamsR);
	void GCamSceneL(int ActCamsL);
	void GCamSceneRred(int ActCamsRred);
	void GCamSceneLred(int ActCamsLred);
	void initHead(double headzero);

	BufferedPort<ImageOf<PixelRgb> > cam_in;
	BufferedPort<ImageOf<PixelRgb> > cam_out;
	Port output;

	ResourceFinder rf;
	string threadName;

	int width, height; // image dimensions
	int nK; // number of labels
	string colormapFile, svmFile;
	unsigned char *colormap; // maps labels to RGB-triplets
	float *W; // segmentation model (SVM weights)
	float smoothness; // smoothness prior
	int niter; // number of iterations of MRF optimizer (TRW-S)
	unsigned minsize; // minimmum size of connected components
	int nE; // number of edges in the image graph
	unsigned *E; // edges of the image graph
	real *q; // MRF unary potentials
	real *f; // TRW-S messages
	unsigned char *K; // image with labels (output of MRF optimizer)
	unsigned *J; // image with labels (output of connected components algorithm)
	//====================================
	/*int nKL; //
	int nEL; // number of edges in the image graph
	unsigned *EL; // edges of the image graph
	real *qL; // MRF unary potentials
	real *fL; // TRW-S messages
	unsigned char *KL; // image with labels (output of MRF optimizer)
	unsigned *JL; // image with labels (output of connected components algorithm) */ 
	//=====================================
	double imageDetails[10][8];
	double imageDetailsL[10][8];
	double ObjectIdentityR[10][8];
	double ObjectIdentityL[10][8];
	int bbvals[100];	
    int countee; 
	int bbvalsL[100];	
    int counteeL; 
	int numobjectsR;
	int numobjectsL;
	
};
