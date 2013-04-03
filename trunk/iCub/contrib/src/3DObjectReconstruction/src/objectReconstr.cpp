#include "objectReconstr.h"
#include <iostream>
#include <fstream>
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;

ObjectReconstr::ObjectReconstr()
{
    current_state=STATE_WAIT;
    gazeCtrl=NULL;
    write=true;
    visualizationOn=false;
    useSegmentation=true;
    closing=false;
    number=0;
    leftTopVertex.resize(2,0.0);
    rightDownVertex.resize(2,0.0);
    dim.resize(3,0.0);
}

bool ObjectReconstr::configure(ResourceFinder &rf)
{
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string name=rf.check("name",Value("objectReconstr")).asString().c_str();
    setName(name.c_str());
    string imL=rf.check("imL",Value("/leftPort")).asString().c_str();
    string imR=rf.check("imR",Value("/rightPort")).asString().c_str();
    tableHeight=rf.check("tableHeight",Value(-0.11)).asDouble();
    useChris=false;//rf.check("useChris");
    outputDir=rf.check("outputDir",Value("C:\\Lib\\iCub\\app\\3DObjectReconstruction\\conf")).asString().c_str();
    computeBB=false;

    middlex=-1;
    middley=-1;

    Property commOptions;
    commOptions.put("milPortName",rf.check("MilPortName",Value("/objectReconstr/mil")).asString().c_str());
    commOptions.put("opcPortName",rf.check("OpcPortName",Value("/objectReconstr/opc")).asString().c_str());

    if (useChris)
    {
        if (!communicator.open(commOptions))
        {
            fprintf(stdout, "Chris ports seem to be closed, run Chris first\n");
            return false;
        }
    }

    Property optGaze;
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/client/gaze");

    gazeCtrl=new PolyDriver(optGaze);
    if (gazeCtrl->isValid()) 
        gazeCtrl->view(igaze);
    else 
    {
        fprintf(stdout, "Gaze controller failed to open\n");
        delete gazeCtrl;
        return false;
    }
    string slash="/";

    Property optTorso;
    optTorso.put("device","remote_controlboard");   
    optTorso.put("remote",(slash+robot+"/torso").c_str());
    optTorso.put("local",(slash+name+"/torso/position").c_str());
    if (polyTor.open(optTorso))
    {
        polyTor.view(posTor);
    }
    else
    {
        delete gazeCtrl;
        fprintf(stdout, "IPositionControl failed to open\n");
        return false;
    }

    Property optHead;
    optHead.put("device","remote_controlboard");
    optHead.put("remote",(slash+robot+"/head").c_str());
    optHead.put("local",(slash+name+"/head/position").c_str());
    if (polyHead.open(optHead))
        polyHead.view(posHead);
    else
    {
        delete gazeCtrl;
        if (polyTor.isValid())
            polyTor.close();
        fprintf(stdout, "IPositionControl failed to open\n");
        return false;
    }

    imagePortInLeft.open(imL.c_str());
    imagePortInRight.open(imR.c_str());

    Network::connect((slash+robot+"/camcalib/left/out").c_str(),imL.c_str());
    Network::connect((slash+robot+"/camcalib/right/out").c_str(),imR.c_str());

    string pcOut=slash + getName().c_str() + "/mesh:o";
    pointCloudPort.open(pcOut.c_str());

    string segmIn=slash+getName().c_str()+"/segmentation:i";
    segmentationPort.open(segmIn.c_str());
    segmModInterface.yarp().attachAsClient(segmentationPort);

    Network::connect(segmIn.c_str(),"/GBSeg/conf");

    leftTopVertex[0]=0;
    leftTopVertex[1]=0;
    rightDownVertex[0]=320-1;
    rightDownVertex[1]=240-1;

    Bottle p;
    igaze->getInfo(p);
    minVergence=ceil(p.check(("min_allowed_vergence"),Value(1)).asDouble());

    initProc();

    string rpcName=rf.check("rpcName",Value("/objectReconstr/rpc")).asString().c_str();
    useTable=rf.check("useTable");

    rpc.open(rpcName.c_str());
    attach(rpc);

    igaze->storeContext(&context_without_eyes_blocked);
    igaze->blockEyes(5.0);
    igaze->storeContext(&context_with_eyes_blocked);
    igaze->restoreContext(context_without_eyes_blocked);

    Property recRoutOptions;
    recRoutOptions.put("ConfigDisparity",rf.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str());
    recRoutOptions.put("CameraContext",rf.check("CameraContext",Value("cameraCalibration/conf")).asString().c_str());
    recRoutOptions.put("outputDir",outputDir.c_str());

    if (!recRoutine.open(recRoutOptions))
    {
        fprintf(stdout, "Problem with thread, the module will be closed\n");
        close();
        return false;
    }

    return true;
}

bool ObjectReconstr::initProc()
{
    if (useChris)
    {
        double height;
        if (communicator.retrieveTableHeight(height))
            setTableHeight(height);

        if (!communicator.disableAttention())
            return false;

        steerEyesToHome();
    }

    ImageOf< PixelRgb >* toCheckdim = imagePortInLeft.read(false);
    if (toCheckdim!=NULL)
    {
        rightDownVertex[0]=toCheckdim->width()-1;
        rightDownVertex[1]=toCheckdim->height()-1;
    }
    return true;
}

void ObjectReconstr::setTableHeight(const double tableHeight)
{
    this->tableHeight=tableHeight;
    recRoutine.setTableHeight(tableHeight);
}

bool ObjectReconstr::close()
{
    moveTorso(0.0);

    if (useChris)
        communicator.close();

    delete gazeCtrl;
    if (polyTor.isValid())
        posTor->stop();

    if (polyTor.isValid())
        polyTor.close();

    if (polyHead.isValid())
        polyHead.close();

    imagePortInLeft.close();
    imagePortInRight.close();

    rpc.close();
    pointCloudPort.interrupt();
    segmentationPort.interrupt();

    recRoutine.close();

    return true;
}

void ObjectReconstr::moveTorso(const double degrees)
{
    yarp::sig::Vector possT(3,0.0); 
    possT[1]=degrees;
    yarp::sig::Vector accsT(3,1e9);
    yarp::sig::Vector spdsT(3,10.0);

    posTor->setRefAccelerations(accsT.data());
    posTor->setRefSpeeds(spdsT.data());
    posTor->positionMove(possT.data());

    bool ok=false;

    while(!ok)
        posTor->checkMotionDone(&ok);
}

void ObjectReconstr::steerEyesToHome()
{
    yarp::sig::Vector poss(6,0.0);
    poss[0]=-40.0;
    poss[5]=5.0;
    yarp::sig::Vector accs(6,1e9);
    yarp::sig::Vector spds(6,3.0);

    posHead->setRefAccelerations(accs.data());
    posHead->setRefSpeeds(spds.data());
    posHead->positionMove(poss.data());

    bool ok=false;

    while (!ok) 
    {
        yarp::os::Time::delay(0.1);
        posHead->checkMotionDone(&ok);
    }


    fprintf(stdout, "Eyes home\n");
}

std::vector<yarp::sig::Pixel> ObjectReconstr::getPixelList()
{
    if (useChris)
    {
        communicator.retrieveBoundingBox(object,leftTopVertex,rightDownVertex);
        middlex=(leftTopVertex[0]+rightDownVertex[0])/2;
        middley=(leftTopVertex[1]+rightDownVertex[1])/2;
    }

    if (useSegmentation)
        return segmModInterface.get_component_around(yarp::sig::Pixel(middlex,middley));
    else
        return computePixelListFromBB();
}

std::vector<yarp::sig::Pixel> ObjectReconstr::computePixelListFromBB()
{
    std::vector<yarp::sig::Pixel> pixelList;
    for (int i=leftTopVertex[0]; i<rightDownVertex[0]; i++)
        for (int j=leftTopVertex[1]; j<rightDownVertex[1]; j++)
            pixelList.push_back(yarp::sig::Pixel(i,j));

    return pixelList;
}

void ObjectReconstr::savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    stringstream s;
    s.str("");
    s<<outputDir + "/3Dobject" <<number;
    string filename=s.str();
    string filenameNumb=filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++)
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

    plyfile.close();

    number++;
    fprintf(stdout, "Writing finished\n");
}

bool ObjectReconstr::updateCloud()
{
    printf("Prima porte\n");
    ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(true);
    ImageOf<PixelRgb> *tmpR = imagePortInRight.read(true);

    printf("Dopo porte\n");
    IplImage* imgL;
    IplImage* imgR;
    if(tmpL!=NULL && tmpR!=NULL)
    {
        imgL= (IplImage*) tmpL->getIplImage();
        imgR= (IplImage*) tmpR->getIplImage();
    }
    else
    {
        fprintf(stdout, "Problem with image ports occurred\n");
        return false;
    }

    std::vector<yarp::sig::Pixel> pixelList=getPixelList();

    return recRoutine.reconstruct(imgL,imgR,pixelList);
}

bool ObjectReconstr::updateModule()
{
    switch(current_state)
    {

    case STATE_WAIT:
        return true;

    case STATE_RECONSTRUCT:
        {
            if (useChris)
            {
                if (!communicator.disableAttention())
                    return false;

                steerEyesToHome();
            }

            igaze->restoreContext(context_with_eyes_blocked);
            Time::delay(1.0);

            recRoutine.resetClouds();

            if (!updateCloud())
                return false;

            /*if (!closing)
                moveTorso(20.0);
            Time::delay(1);

            if (!updateCloud())
                return false;

            if (!closing)
                moveTorso(-20.0);
            Time::delay(1);

            if (!updateCloud())
                return false;*/

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=recRoutine.getPointCloudComplete();

            SurfaceMesh &pointCloudOnPort=pointCloudPort.prepare();
            pointCloudOnPort.points.clear();
            for (unsigned int i=0; i<cloud->width; i++)
            {
                pointCloudOnPort.points.push_back(PointXYZ(cloud->at(i).x,cloud->at(i).y, cloud->at(i).z));
                pointCloudOnPort.rgbColour.push_back(RGBA(cloud->at(i).rgb));
            }

            pointCloudPort.write();

            //if (write)
                //savePointsPly(cloud);

            moveTorso(0.0);
            igaze->restoreContext(context_without_eyes_blocked);

            if (computeBB)
            {
                MinBoundBox minboundbox(cloud);
                minboundbox.retrieveMinimumBox(cornerpoints,rotmat);
                retrieveDimension(cornerpoints,rotmat,dim,centerroot,vectxroot,vectyroot,vectzroot);
            }

            if (visualizationOn)
                current_state=STATE_VISUALIZE;
            else
                current_state=STATE_WAIT;

            middlex=-1;
            middley=-1;

            return true;
        }

    case STATE_VISUALIZE:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=recRoutine.getPointCloud();

            if(useTable)
                addPlanePoints(cloud);

            if (visualizationOn)
            {
                boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
                tmpViewer->setBackgroundColor (0, 0, 0);
                if (computeBB)
                {
                    drawBoundingBox(tmpViewer,cornerpoints);
                    drawDimension(tmpViewer,centerroot,vectxroot,vectyroot,vectzroot);
                }
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudComplete;
                cloudComplete=recRoutine.getPointCloudComplete();
                visualize(tmpViewer, cloudComplete);
            }
            current_state=STATE_WAIT;
        }
    }

    return true;
}

void ObjectReconstr::addPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Eigen::Vector4f minim;
    Eigen::Vector4f maxim;
    pcl::getMinMax3D(*cloud,minim,maxim);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int i=0; i<cloud->size(); i++)
    {
        pcl::PointXYZRGB point=cloud->at(i);
        if (useTable)
            point.z=tableHeight;
        else
            point.z=minim[2];
        cloud_tmp->push_back(point);
    }

    for (unsigned int i=0; i<cloud_tmp->size(); i++)
        cloud->push_back(cloud_tmp->at(i));
}

void ObjectReconstr::drawBoundingBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const Matrix &cornerpoints)
{
    pcl::PointXYZRGB point1;
    point1.x=cornerpoints(0,0);
    point1.y=cornerpoints(0,1);
    point1.z=cornerpoints(0,2);
    point1.r=255;
    point1.g=0;
    point1.b=0;
    pcl::PointXYZRGB point2;
    point2.x=cornerpoints(1,0);
    point2.y=cornerpoints(1,1);
    point2.z=cornerpoints(1,2);
    point2.r=255;
    point2.g=0;
    point2.b=0;
    pcl::PointXYZRGB point3;
    point3.x=cornerpoints(2,0);
    point3.y=cornerpoints(2,1);
    point3.z=cornerpoints(2,2);
    point3.r=255;
    point3.g=0;
    point3.b=0;
    pcl::PointXYZRGB point4;
    point4.x=cornerpoints(3,0);
    point4.y=cornerpoints(3,1);
    point4.z=cornerpoints(3,2);
    point4.r=255;
    point4.g=0;
    point4.b=0;
    pcl::PointXYZRGB point5;
    point5.x=cornerpoints(4,0);
    point5.y=cornerpoints(4,1);
    point5.z=cornerpoints(4,2);
    point5.r=255;
    point5.g=0;
    point5.b=0;
    pcl::PointXYZRGB point6;
    point6.x=cornerpoints(5,0);
    point6.y=cornerpoints(5,1);
    point6.z=cornerpoints(5,2);
    point6.r=255;
    point6.g=0;
    point6.b=0;
    pcl::PointXYZRGB point7;
    point7.x=cornerpoints(6,0);
    point7.y=cornerpoints(6,1);
    point7.z=cornerpoints(6,2);
    point7.r=255;
    point7.g=0;
    point7.b=0;
    pcl::PointXYZRGB point8;
    point8.x=cornerpoints(7,0);
    point8.y=cornerpoints(7,1);
    point8.z=cornerpoints(7,2);
    point8.r=255;
    point8.g=0;
    point8.b=0;
    viewer->addLine<pcl::PointXYZRGB>(point1,point2,"line1");
    viewer->addLine<pcl::PointXYZRGB>(point2,point3,"line2");
    viewer->addLine<pcl::PointXYZRGB>(point3,point4,"line3");
    viewer->addLine<pcl::PointXYZRGB>(point4,point1,"line4");
    viewer->addLine<pcl::PointXYZRGB>(point5,point6,"line5");
    viewer->addLine<pcl::PointXYZRGB>(point6,point7,"line6");
    viewer->addLine<pcl::PointXYZRGB>(point7,point8,"line7");
    viewer->addLine<pcl::PointXYZRGB>(point8,point5,"line8");
    viewer->addLine<pcl::PointXYZRGB>(point1,point5,"line9");
    viewer->addLine<pcl::PointXYZRGB>(point2,point6,"line10");
    viewer->addLine<pcl::PointXYZRGB>(point3,point7,"line11");
    viewer->addLine<pcl::PointXYZRGB>(point4,point8,"line12");
}

void ObjectReconstr::retrieveDimension(const Matrix &cornerpoints, const Matrix &rotmat, yarp::sig::Vector &dim, pcl::PointXYZ &centerroot, yarp::sig::Vector &vectxroot, yarp::sig::Vector &vectyroot, yarp::sig::Vector &vectzroot)
{
    Matrix corner_i=cornerpoints*rotmat;
    yarp::sig::Vector indexes(3);
    yarp::sig::Vector point1(3); point1[0]=corner_i(0,0); point1[1]=corner_i(0,1); point1[2]=corner_i(0,2);
    int m=0;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            if (j<=i)
                continue;
            int index=-1;
            double minimum=10000;
            for (int t=1; t<corner_i.rows(); t++)
            {
                yarp::sig::Vector tmp(3); tmp[0]=corner_i(t,0); tmp[1]=corner_i(t,1); tmp[2]=corner_i(t,2);
                double value=norm(Helpers::extractSubVector(point1,i,j)-Helpers::extractSubVector(tmp,i,j));
                if (value<minimum)
                {
                    index=t;
                    minimum=value;
                }
            }
            indexes[m]=index;
            m+=1;
        }
    }

    dim[2]=max(point1[2],corner_i(indexes[0],2))-min(point1[2],corner_i(indexes[0],2));
    dim[1]=max(point1[1],corner_i(indexes[1],1))-min(point1[1],corner_i(indexes[1],1));
    dim[0]=max(point1[0],corner_i(indexes[2],0))-min(point1[0],corner_i(indexes[2],0));

    yarp::sig::Vector index0(3); index0[0]=corner_i(indexes[0],0); index0[1]=corner_i(indexes[0],1); index0[2]=corner_i(indexes[0],2);
    yarp::sig::Vector index1(3); index1[0]=corner_i(indexes[1],0); index1[1]=corner_i(indexes[1],1); index1[2]=corner_i(indexes[1],2);
    yarp::sig::Vector index2(3); index2[0]=corner_i(indexes[2],0); index2[1]=corner_i(indexes[2],1); index2[2]=corner_i(indexes[2],2);

    yarp::sig::Vector vectz=point1-index0;
    vectzroot=vectz*rotmat.transposed();
    yarp::sig::Vector vecty=point1-index1;
    vectyroot=vecty*rotmat.transposed();
    yarp::sig::Vector vectx=point1-index2;
    vectxroot=vectx*rotmat.transposed();

    yarp::sig::Vector center(3);
    center[2]=(point1[2]+index0[2])/2;
    center[1]=(point1[1]+index1[1])/2;
    center[0]=(point1[0]+index2[0])/2;

    yarp::sig::Vector center2=center*rotmat.transposed();

    centerroot.x=center2[0];
    centerroot.y=center2[1];
    centerroot.z=center2[2];
}

void ObjectReconstr::drawDimension(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointXYZ &centerroot, yarp::sig::Vector &vectxroot, yarp::sig::Vector &vectyroot, yarp::sig::Vector &vectzroot)
{
    pcl::PointXYZ z1;
    z1.x=centerroot.x+(vectzroot[0]/2);
    z1.y=centerroot.y+(vectzroot[1]/2);
    z1.z=centerroot.z+(vectzroot[2]/2);

    pcl::PointXYZ z2;
    z2.x=centerroot.x+(-vectzroot[0]/2);
    z2.y=centerroot.y+(-vectzroot[1]/2);
    z2.z=centerroot.z+(-vectzroot[2]/2);

    pcl::PointXYZ y1;
    y1.x=centerroot.x+(vectyroot[0]/2);
    y1.y=centerroot.y+(vectyroot[1]/2);
    y1.z=centerroot.z+(vectyroot[2]/2);

    pcl::PointXYZ y2;
    y2.x=centerroot.x+(-vectyroot[0]/2);
    y2.y=centerroot.y+(-vectyroot[1]/2);
    y2.z=centerroot.z+(-vectyroot[2]/2);

    pcl::PointXYZ x1;
    x1.x=centerroot.x+(vectxroot[0]/2);
    x1.y=centerroot.y+(vectxroot[1]/2);
    x1.z=centerroot.z+(vectxroot[2]/2);

    pcl::PointXYZ x2;
    x2.x=centerroot.x+(-vectxroot[0]/2);
    x2.y=centerroot.y+(-vectxroot[1]/2);
    x2.z=centerroot.z+(-vectxroot[2]/2);
   
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,z1,1,1,1,"z1");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,z2,1,1,1,"z2");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,y1,1,1,1,"y1");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,y2,1,1,1,"y2");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,x1,1,1,1,"x1");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(centerroot,x2,1,1,1,"x2");
}

bool ObjectReconstr::interruptModule()
{
    closing=true;
    rpc.interrupt();

    imagePortInLeft.interrupt();
    imagePortInRight.interrupt();

    pointCloudPort.interrupt();
    segmentationPort.interrupt();

    fprintf(stdout, "Close the window!\n");

    return true;
}

double ObjectReconstr::getPeriod()
{
    return 0.1;
}

bool ObjectReconstr::respond(const Bottle& command, Bottle& reply) 
{
    if (command.get(0).asString()=="set")
    {
        if (command.get(1).asString()=="tableHeight")
        {
            setTableHeight(command.get(2).asDouble());
            reply.addVocab(ACK);
            reply.addString("tableHeight ");
            reply.addDouble(command.get(2).asDouble());
            return true;
        }
        else if (command.get(1).asString()=="write")
        {
            if (command.size()>2)
            {
                if (command.get(2).asString()=="true")
                {
                    reply.addVocab(ACK);
                    write=true;
                }
                else
                {
                    write=false;
                    reply.addVocab(NACK);
                }
                return true;
            }
        }
        else if (command.get(1).asString()=="box")
        {
            if (command.size()<6)
            {
                reply.addVocab(NACK);
                reply.addString("not enough argument");
                return true;
            }
            setBoundingBox(command.get(2).asDouble(), command.get(3).asDouble(), command.get(4).asDouble(), command.get(5).asDouble());
            reply.addVocab(ACK);
            reply.addString("bounding box set");
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            reply.addString("command not recognized");
            return true;
        }
    }
    if (command.get(0).asString()=="3Drec")
    {
        if (middlex==-1 || middley==-1)
        {
            reply.addVocab(NACK);
            reply.addString("Click on the segmentation image first");
            return true;
        }
        if (command.size()<2)
        {
            reply.addVocab(NACK);
            reply.addString("Command not recognized");
            return true;
        }
        object=command.get(1).asString();

        current_state=STATE_RECONSTRUCT;

        if (command.size()==3)
            if (command.get(2).asString()=="on")
                visualizationOn=true;

        reply.addVocab(ACK);
        return true;
    }

    if (command.get(0).asString()=="get")
    {
        if (current_state!=STATE_RECONSTRUCT)
        {
            reply.addVocab(ACK);
            Bottle& center=reply.addList();
            center.addString("center");
            Bottle& centerpoint=center.addList();
            centerpoint.addDouble(centerroot.x);
            centerpoint.addDouble(centerroot.y);
            centerpoint.addDouble(centerroot.z);
            Bottle& cornerpointsB=reply.addList();
            cornerpointsB.addString("cornerpoints");
            Bottle& cornerpointsB2=cornerpointsB.addList();
            for (int i=0; i<cornerpoints.rows(); i++)
            {
                Bottle& corner=cornerpointsB2.addList();
                corner.addDouble(cornerpoints(i,0));
                corner.addDouble(cornerpoints(i,1));
                corner.addDouble(cornerpoints(i,2));
            }
            Bottle& dimension=reply.addList();
            dimension.addString("dimension");
            Bottle& dimensionPoint=dimension.addList();
            dimensionPoint.addDouble(dim[0]);
            dimensionPoint.addDouble(dim[1]);
            dimensionPoint.addDouble(dim[2]);
            Bottle& rotmatrix=reply.addList();
            rotmatrix.addString("frame");
            Bottle& rotmatrixPoint=rotmatrix.addList();
            for (int i=0; i<rotmat.rows(); i++)
                for (int j=0; j<rotmat.cols(); j++)
                    rotmatrixPoint.addDouble(rotmat(i,j));
        }
        else
        {
            reply.addVocab(NACK);
            reply.addString("I'm still processing");
        }
        return true;
    }

    if (command.size()==2)
    {
        if (command.get(0).asInt()!=0 && command.get(1).asInt()!=0)
        {
            middlex=(double)command.get(0).asInt();
            middley=(double)command.get(1).asInt();
            reply.addVocab(ACK);
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            reply.addString("command not recognized");
            return true;
        }
    }

    reply.addVocab(NACK);
    reply.addString("command not recognized");
    return true;
}

void ObjectReconstr::setBoundingBox(double lx, double ly, double rx, double ry)
{
    leftTopVertex[0]=lx;
    leftTopVertex[1]=ly;
    rightDownVertex[0]=rx;
    rightDownVertex[1]=ry;
}

void ObjectReconstr::visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    string id=object+" Cloud";
    tmpViewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id);
    tmpViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    tmpViewer->initCameraParameters();
    tmpViewer->spin();
    tmpViewer->removePointCloud(id);
}

