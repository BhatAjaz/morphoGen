// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) Robotics Brain & Cognitive Sciences Department, IIT
 * Authors: Rea Francesco, Ajaz Ahmad Bhat 
 * email:   francesco.rea@iit.it, ajaz.bhat@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "visualizationGui.h"

#include <yarp/os/Property.h> 
#include <yarp/os/Network.h> 
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>

#include <string.h>
#include <sstream>

#define COMMAND_VOCAB_VIS VOCAB3('V','I','S')
#define COMMAND_VOCAB_OFF VOCAB3('O','F','F')
#define COMMAND_VOCAB_IIT VOCAB3('I','I','T')
#define COMMAND_VOCAB_NEU VOCAB3('N','E','U')
#define COMMAND_VOCAB_REM VOCAB3('R','E','M')
#define COMMAND_VOCAB_CSH VOCAB3('C','S','H')
#define COMMAND_VOCAB_GRA VOCAB3('G','R','A')
#define COMMAND_VOCAB_VIS VOCAB3('V','I','S')
#define COMMAND_VOCAB_ON  VOCAB2('O','N')

#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_KBU VOCAB3('k','b','u') //weight of the bottom-up algorithm
#define COMMAND_VOCAB_KTD VOCAB3('k','t','d') //weight of top-down algorithm
#define COMMAND_VOCAB_RIN VOCAB3('r','i','n') //red intensity value
#define COMMAND_VOCAB_GIN VOCAB3('g','i','n') //green intensity value
#define COMMAND_VOCAB_BIN VOCAB3('b','i','n') //blue intensity value
#define COMMAND_VOCAB_K1 VOCAB2('k','1')
#define COMMAND_VOCAB_K2 VOCAB2('k','2')
#define COMMAND_VOCAB_K3 VOCAB2('k','3')
#define COMMAND_VOCAB_K4 VOCAB2('k','4')
#define COMMAND_VOCAB_K5 VOCAB2('k','5')
#define COMMAND_VOCAB_K6 VOCAB2('k','6')
#define COMMAND_VOCAB_KC1 VOCAB3('k','c','1')
#define COMMAND_VOCAB_KC2 VOCAB3('k','c','2')
#define COMMAND_VOCAB_KC3 VOCAB3('k','c','3')
#define COMMAND_VOCAB_KC4 VOCAB3('k','c','4')
#define COMMAND_VOCAB_KC5 VOCAB3('k','c','5')
#define COMMAND_VOCAB_KC6 VOCAB3('k','c','6')
#define COMMAND_VOCAB_KMOT VOCAB4('k','m','o','t')

using namespace yarp::os;
using namespace std;

GtkWidget *mainWindow=0;
static GdkPixbuf *frame = NULL;

BufferedPort<yarp::sig::FlexImage> *ptr_inputPort=0;


std::string* command;   //reference to the string refering to the last command to send

int _frameN;            //Frame Number
bool _savingSet;        //Save Set of Images mode
// 
//yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort=0; //Output Point Port
Port* _nocaPort         = 0;
Port* _profactorPort    = 0;
Port* _visControlPort   = 0;
Port* _pOutPort         = 0;

Port* _iitPort1, *_iitPort2, *_iitPort3;
Port* _kclPort1,*_kclPort2,*_kclPort3;
Port* _forthPort1,*_forthPort2,*_forthPort3;
Port* _cvutPort1,*_cvutPort2,*_cvutPort3;

bool _iitBool1   = false; 
bool _iitBool2   = false; 
bool _iitBool3   = false;
bool _kclBool1   = false;
bool _kclBool2   = false;
bool _kclBool3   = false;
bool _forthBool1 = false;
bool _forthBool2 = false;
bool _forthBool3 = false;
bool _cvutBool1  = false;
bool _cvutBool2  = false;
bool _cvutBool3  = false;
bool _nocaBool   = false;
bool _profactorBool = false;

yarp::os::Bottle _outBottle;//Output Bottle Container
pgmOptions _options; //option set by the config file

GtkWidget *saveSingleDialog;
GtkWidget *saveSetDialog;
GtkWidget *menubar;
GtkWidget *fileMenu, *imageMenu, *helpMenu;
GtkWidget *fileItem, *helpItem;
GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
GtkWidget *imageSizeItem, *imageRatioItem, *imageFreezeItem, *imageFramerateItem;
GtkWidget *synchroDisplayItem;
GtkWidget *helpAboutItem;
// StatusBar
GtkWidget *statusbar;
GtkWidget *fpsStatusBar;
GtkWidget *fpsStatusBar2;

guint timeout_ID=0;
guint timeout_update_ID=0;

ViewerResources _resources;


static void createObjects() {
    command=new string("");
}

static void deleteObjects() {
    /*if (ptr_inputPort!=0)
        delete ptr_inputPort;
    if (ptr_portCallback!=0)
        delete ptr_portCallback;*/
    delete command;
}




//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------
/**
* usual callback function 
*/
/*
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);
    
    if(!strcmp((char *)data,"OpencvSobel")){
        printf("OpencvSobel");
        command->assign("set ocv");
    }
    if(!strcmp((char *)data,"ConvolveMax")){
        printf("ConvolveMax");
        command->assign("set max");
    }
    if(!strcmp((char *)data,"ConvolveSeq")){
        printf("ConvolveSeq");
        command->assign("set seq");
    }
    if(!strcmp((char *)data,"ConvolveFil")){
        printf("ConvolvFil");
        command->assign("set fil");
    }
    if(!strcmp((char *)data,"IppSobel")){
        printf("IppSobel");
        command->assign("set ipp");
    }

}
*/


//-------------------------------------------------
// Call Backs
//-------------------------------------------------



//iit call backs
static void cb_iit_mod1( GtkWidget *widget, gpointer data ) {
    
    if (_iitPort1!=NULL) {
        if(_iitPort1->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();

            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }
            
            _iitPort1->write(bot,in);
        }    
    }

    if (_visControlPort!=NULL) {
        printf("iit_mod1 \n");
        if(_visControlPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_CSH);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}

static void cb_iit_mod2( GtkWidget *widget, gpointer data ) {
    if (_iitPort2!=NULL) {
        if(_iitPort2->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_iitBool2){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool2 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool2 = true;
            }
            _iitPort2->write(bot,in);
           
        }   
    }
    if (_visControlPort!=NULL) {        
        printf("iit_mod2 \n");
        if(_visControlPort->getOutputCount()){
            
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_REM);
            if(_iitBool2){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool2 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool2 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}


static void cb_iit_mod3( GtkWidget *widget, gpointer data ) {
    if (_iitPort3!=NULL) {
        if(_iitPort3->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }
            
            _iitPort1->write(bot,in);
        }
    }
    if (_visControlPort!=NULL) {
        printf("iit_mod3 \n");
        if(_visControlPort->getOutputCount()){
            
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_NEU);
            if(_iitBool3){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool3 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool3 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}




// kcl call backs
static void cb_kcl_mod1( GtkWidget *widget, gpointer data ) {
    if (_kclPort1!=NULL) {
        if(_kclPort1->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_kclBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _kclBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _kclBool1 = true;
            }
            _kclPort1->write(bot,in);
           
        }      
    }
    if (_visControlPort!=NULL) {
        printf("kcl_mod1 \n");
        if(_visControlPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_GRA);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}

static void cb_kcl_mod2( GtkWidget *widget, gpointer data ) {
    if (_kclPort2!=NULL) {
        if(_kclPort2->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_kclBool2){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _kclBool2 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _kclBool2 = true;
            }
            _kclPort2->write(bot,in);           
        }        
    }
    if (_visControlPort!=NULL) {
        if(_visControlPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_GRA);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}

static void cb_kcl_mod3( GtkWidget *widget, gpointer data ) {
    if (_kclPort3!=NULL) {
        if(_kclPort3->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_kclBool3){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _kclBool3 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _kclBool3 = true;
            }
            _kclPort3->write(bot,in);
           
        }
        
    }
}


// forth call backs
static void cb_forth_mod1( GtkWidget *widget, gpointer data ) {
    if (_forthPort1!=NULL) {
        if(_forthPort1->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_forthBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _forthBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _forthBool1 = true;
            }
            _forthPort1->write(bot,in);
        }
    }
    if (_visControlPort!=NULL) {
        printf("forth_mod1 \n");
        if(_visControlPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}

static void cb_forth_mod2( GtkWidget *widget, gpointer data ) {
    if (_forthPort2!=NULL) {
        if(_forthPort2->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_forthBool2){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _forthBool2 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _forthBool2 = true;
            }
            _forthPort2->write(bot,in);           
        }
    }
    if (_visControlPort!=NULL) {
        printf("forth_mod2 \n");
        if(_visControlPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot;
            bot.clear();
            //bot.addVocab(COMMAND_VOCAB_IIT);
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_iitBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _iitBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _iitBool1 = true;
            }            
            _visControlPort->write(bot,in);
        }    
    }
}

static void cb_forth_mod3( GtkWidget *widget, gpointer data ) {
    if (_forthPort3!=NULL) {
        if(_forthPort3->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_forthBool3){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _forthBool3 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _forthBool3 = true;
            }
            _forthPort3->write(bot,in);
           
        }
        
    }
}

static void cb_cvut_mod1( GtkWidget *widget, gpointer data ) {
      if (_cvutPort1!=NULL) {
        if(_cvutPort1->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_cvutBool1){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _cvutBool1 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _cvutBool1 = true;
            }
            _cvutPort1->write(bot,in);
           
        }
        
    }
}

static void cb_cvut_mod2( GtkWidget *widget, gpointer data ) {
      if (_cvutPort2!=NULL) {
        if(_cvutPort2->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_cvutBool2){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _cvutBool2 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _cvutBool2 = true;
            }
            _cvutPort2->write(bot,in);
           
        }
        
    }
}

static void cb_cvut_mod3( GtkWidget *widget, gpointer data ) {
      if (_cvutPort3!=NULL) {
        if(_cvutPort3->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_cvutBool3){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _cvutBool3 = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _cvutBool3 = true;
            }
            _cvutPort3->write(bot,in);
           
        }
        
    }
}


static void cb_noca_mod1( GtkWidget *widget, gpointer data ) {
    if (_nocaPort!=NULL) {
        if(_nocaPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_nocaBool){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _nocaBool = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _nocaBool = true;
            }
            _nocaPort->write(bot,in);
           
        }
        
    }
}

static void cb_profactor_mod1( GtkWidget *widget, gpointer data ) {
     if (_profactorPort!=NULL) {
        if(_profactorPort->getOutputCount()){
            Bottle in;
            yarp::os::Bottle bot; //= _pOutPort->prepare();
            bot.clear();
            bot.addVocab(COMMAND_VOCAB_VIS);
            if(_profactorBool){
                bot.addVocab(COMMAND_VOCAB_OFF);
                _profactorBool = false;
            }
            else {
                bot.addVocab(COMMAND_VOCAB_ON);
                _profactorBool = true;
            }
            _profactorPort->write(bot,in);
           
        }
        
    }
}


/*
static void cb_digits_scale2( GtkAdjustment *adj ) {
   
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K2);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scale3( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K3);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scale4( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K4);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scale5( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K5);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scale6( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K6);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scale11( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_KC1);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}

static void cb_digits_scaleMotion( GtkAdjustment *adj ) {
    
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_KMOT);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        Bottle in;
        _pOutPort->write(bot,in);
    }
}
*/

gint timeout_update_CB(gpointer data) {
    //portFpsData.getStats(av, min, max);
    //portFpsData.reset();
    gchar *msg;
    //gdk_threads_enter();
    msg=g_strdup_printf("visualizationGui");
    updateStatusbar(fpsStatusBar, msg);
    g_free(msg);
    //displayFpsData.getStats(av, min, max);
    //displayFpsData.reset();
    
    //periodToFreq(av, min, max, avHz, minHz, maxHz);

    //msg=g_strdup_printf("Display: %.1f (min:%.1f max:%.1f) fps", avHz, minHz, maxHz);
    //updateStatusbar(fpsStatusBar2, msg);
    //g_free(msg);
    //gdk_threads_leave();
    return TRUE;
}

gint timeout_CB (gpointer data) {
    c++;
    switch(c) {
        case 0: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KMOT);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    //mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adjMotion),value);
                    //mutex.post();
            }
        }
        break;
        case 1: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K1);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj1),value);
                    mutex.post();
            }
        }
        break;
        case 2: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K2);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj2),value);
                    mutex.post();
            }
        }
        break;
        case 3: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K3);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj3),value);
                    mutex.post();
            }
        }
        break;
        case 4: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K4);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj4),value);
                    mutex.post();
            }
        }
        break;
        case 5: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K5);
                    //_pOutPort->Content() = _out5Bottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj5),value);
                    mutex.post();
            }
        }
        break;
        case 6: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K6);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj6),value);
                    mutex.post();
           }
        }
        break;
        case 7: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC1);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj11),value);
                    mutex.post();
            }
        }
        break;
        case 8: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC2);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj12),value);
                    mutex.post();
            }
        }
        break;
        case 9: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC3);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj13),value);
                    mutex.post();
            }
        }
        break;
        case 10: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC4);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj14),value);
                    mutex.post();
            }
        }
        break;
        case 11: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC5);
                    //_pOutPort->Content() = _out5Bottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj15),value);
                    mutex.post();
            }
        }
        break;
        case 12: {
            if(_pOutPort->getOutputCount()) {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_KC6);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj16),value);
                    mutex.post();
            }
        }
        break;
        default: {
                    c=-1;
                 }
    }
    return TRUE;
}


gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data ) {
    // If you return FALSE in the "delete_event" signal handler,
    // GTK will emit the "destroy" signal. Returning TRUE means
    // you don't want the window to be destroyed.
    // This is useful for popping up 'are you sure you want to quit?'
    // type dialogs. 
    cleanExit();
    return TRUE;
}

gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    return TRUE;
}

static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event ) {
   _resources.configure(widget, 
        widget->allocation.width,
        widget->allocation.height);
  return TRUE;
}
gint menuFileQuit_CB(GtkWidget *widget, gpointer data) {
    cleanExit();
    return TRUE;
}

gint menuHelpAbout_CB(GtkWidget *widget, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)
    const gchar *authors[] = 
        {
            "Yarp developers",
            NULL
        };
    const gchar *license =
        "Released under the terms of the LGPLv2.1 or later, see LGPL.TXT\n"
        "The complete license description is contained in the\n"
        "COPYING file included in this distribution.\n"
        "Please refer to this file for complete\n"
        "information about the licensing of YARP.\n"
        "\n"
        "DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE\n"
        "SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS\n"
        "DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS\n"
        "EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE\n"
        "SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT\n"
        "OWNERS AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED\n"
        "INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
        "FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO\n"
        "EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE\n"
        "LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN\n"
        "ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "ONNECTION WITH THE SOFTWARE.\n";

    gtk_show_about_dialog(GTK_WINDOW(mainWindow),
                          "name", "visualizationGui",
                          "version", "1.0",
                          "license", license,
                          "website", "http://sourceforge.net/projects/yarp0",
                          "comments", "Interface for DARWIN demonstrator",
                          "authors", authors,
                          NULL);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif

    return TRUE;
}

gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            gtk_widget_show_all (saveSingleDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSingleDialog);
        }

    return TRUE;
}
void setTimedMode(guint dT) {
   // ptr_portCallback->mustDraw(false);
    if (timeout_ID!=0)
        gtk_timeout_remove(timeout_ID);
    timeout_ID = gtk_timeout_add (dT, timeout_CB, NULL);
}

void setSynchroMode() {
    gtk_timeout_remove(timeout_ID);
    timeout_ID=0;
    //ptr_portCallback->mustDraw(true);
}

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
/*
GtkWidget* createSaveSingleDialog(void) {

    GtkWidget *dialog = NULL;
    GtkWidget *hbox;
    GtkWidget *button;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Snapshot");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    //gtk_window_resize(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    button = gtk_button_new_from_stock(GTK_STOCK_SAVE);
    gtk_widget_set_size_request (GTK_WIDGET(button), 150,50);
    gtk_box_pack_start (GTK_BOX (hbox), button, TRUE, TRUE, 16); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    
    //gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    
    return dialog;
}

GtkWidget* createSaveSetDialog(void) {
    GtkWidget *dialog = NULL;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Image Set");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 190, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    return dialog;
}
*/
//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
/*
GtkWidget* createMenubar(void) {
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    fileSingleItem = gtk_check_menu_item_new_with_label ("Save single image..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSingleItem);
    gtk_signal_connect( GTK_OBJECT(fileSingleItem), "toggled", GTK_SIGNAL_FUNC(menuFileSingle_CB), mainWindow);
    fileSetItem = gtk_check_menu_item_new_with_label ("Save a set of images..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSetItem);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    // Submenu: Image  
    imageMenu = gtk_menu_new();
    
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    
    // Submenu: Help
    helpMenu = gtk_menu_new();
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);
    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}
*/
/*
static GtkWidget *xpm_label_box( gchar     *xpm_filename,gchar *label_text ) {
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    // Create box for image and label 
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    // Now on to the image stuff 
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    // Create a label for the button 
    label = gtk_label_new (label_text);

    // Pack the image and label into the box 
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

static void scale_set_default_values( GtkScale *scale ) {
    gtk_range_set_update_policy (GTK_RANGE (scale),GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 2);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}
*/
//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg) {
    GtkStatusbar *sb=GTK_STATUSBAR (statusbar);

    gtk_statusbar_pop (sb, 0); // clear any previous message, underflow is allowed 
    gtk_statusbar_push (sb, 0, msg);
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* createMainWindow(void) {
    
    
    GtkRequisition actualSize;
    GtkWidget* window;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "visualizationGui");
    gtk_window_set_default_size(GTK_WINDOW (window), 205, 300); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);

    // When the window is given the "delete_event" signal (this is given
    // by the window manager, usually by the "close" option, or on the
    // titlebar), we ask it to call the delete_event () function
    // as defined above. The data passed to the callback
    // function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);
    // Box for main window
    GtkWidget *box;
    GtkWidget *box1;
    GtkWidget *box2, *box2a, *box2b;
    GtkWidget *box3, *box3a, *box3b, *box3c, *box3d;
    //box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    //gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    //menubar = createMenubar();
    //gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);
    // Drawing Area : here the image will be drawed
    //da = gtk_drawing_area_new ();
    //g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
    /*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }*/
    //gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
    
    
    //Toolbox area
    //creates the area as collection of port processes sequence
    box1 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box1);
    //GtkWidget *boxButtons;
    //GtkWidget *boxSliders;
    //boxButtons = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    //gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    //boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    //gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
    
    //-----SCALE section
    GtkWidget *scrollbar;
    GtkWidget *separator;
    GtkWidget *label;
    GtkWidget *button2a1,*button2a2;
    GtkWidget *button2b1,*button2b2;

    GtkWidget *button3a1,*button3a2,*button3a3;
    GtkWidget *button3b1,*button3b2,*button3b3;
    GtkWidget *button3c1,*button3c2,*button3c3;
    GtkWidget *button3d1,*button3d2,*button3d3;
    //GtkWidget *scale;

   //Add some space here

    box2=  gtk_vbox_new (FALSE, 0);   
    gtk_box_pack_start (GTK_BOX (box1), box2, FALSE, TRUE, 0);
    gtk_widget_show (box2);

    label = gtk_label_new ("General Control");
    gtk_box_pack_start (GTK_BOX (box2), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    // Add some space here

    //subsection a
    box2a=  gtk_hbox_new (TRUE, 0);   
    gtk_box_pack_start (GTK_BOX (box2), box2a, FALSE, TRUE, 0);
    gtk_widget_show (box2a);
    
    button2a1 = gtk_check_button_new_with_label("Reasoning(IIT)");
    gtk_box_pack_start (GTK_BOX (box2a), button2a1, TRUE, FALSE, 0);
    gtk_widget_show (button2a1);
    
    button2a2 = gtk_check_button_new_with_label("Grasping(KCL)");
    gtk_box_pack_start (GTK_BOX (box2a), button2a2, TRUE, FALSE, 0);
    gtk_widget_show (button2a2);
    
    //subsection b
    box2b=  gtk_hbox_new (TRUE, 0);   
    gtk_box_pack_start (GTK_BOX (box2), box2b, FALSE, TRUE, 0);
    gtk_widget_show (box2b);

    button2b1 = gtk_check_button_new_with_label("vision(CVUT-FORTH)");
    gtk_box_pack_start (GTK_BOX (box2b), button2b1, TRUE, FALSE, 0);
    gtk_widget_show (button2b1);

    //button2b2 = gtk_check_button_new_with_label("CVUT_all");
    //gtk_box_pack_start (GTK_BOX (box2b), button2b2, TRUE, FALSE, 0);
    // gtk_widget_show (button2b2);

    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box1), separator, FALSE, FALSE, 10);
    gtk_widget_show (separator);

    //separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    //gtk_widget_show (separator);

    //----------BOX3 SECTION:1
    //box3 is the area that controls the processing towards the output ports individually
    //every processes sequence has a sequence of checkboxes a label and a button
    box3 = gtk_hbox_new (TRUE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box1), 0);
    gtk_box_pack_start (GTK_BOX (box1), box3, FALSE, FALSE, 0);
    gtk_widget_show (box3);

    //----------BOX3 SUBSECTION:a
    box3a = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (box3), box3a, TRUE, FALSE, 0);
    gtk_widget_show (box3a);

    label = gtk_label_new ("Reasoning(IIT)");
    gtk_box_pack_start (GTK_BOX (box3a), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    button3a1 = gtk_check_button_new_with_label("colourShape");
    gtk_box_pack_start (GTK_BOX (box3a), button3a1, TRUE, FALSE, 0);
    gtk_widget_show (button3a1);
     
    button3a2 = gtk_check_button_new_with_label("remembering");
    gtk_box_pack_start (GTK_BOX (box3a), button3a2, TRUE, FALSE, 0);
    gtk_widget_show (button3a2);
    
    button3a3 = gtk_check_button_new_with_label("neuronVisualization");
    gtk_box_pack_start (GTK_BOX (box3a), button3a3, TRUE, FALSE, 0);
    gtk_widget_show (button3a3);

    //signal connects
    g_signal_connect (GTK_OBJECT (button3a1), "clicked", GTK_SIGNAL_FUNC (cb_iit_mod1), NULL); 
    g_signal_connect (GTK_OBJECT (button3a2), "clicked", GTK_SIGNAL_FUNC (cb_iit_mod2), NULL); 
    g_signal_connect (GTK_OBJECT (button3a3), "clicked", GTK_SIGNAL_FUNC (cb_iit_mod3), NULL); 


    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);


    //----------BOX3 SUBSECTION:b
    box3b = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (box3), box3b, FALSE, FALSE, 0);
    gtk_widget_show (box3b);

    label = gtk_label_new ("Grasping(KCL)");
    gtk_box_pack_start (GTK_BOX (box3b), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    button3b1 = gtk_check_button_new_with_label("grasp Left/Right");
    gtk_box_pack_start (GTK_BOX (box3b), button3b1, TRUE, FALSE, 0);
    gtk_widget_show (button3b1);

    //button3b2 = gtk_check_button_new_with_label("grasp Right");
    //gtk_box_pack_start (GTK_BOX (box3b), button3b2, TRUE, FALSE, 0);
    //gtk_widget_show (button3b2);
    
    //button3b3 = gtk_check_button_new_with_label("module 3");
    //gtk_box_pack_start (GTK_BOX (box3b), button3b3, TRUE, FALSE, 0);
    //gtk_widget_show (button3b3);

    //signal connects
    g_signal_connect (GTK_OBJECT (button3b1), "clicked", GTK_SIGNAL_FUNC (cb_kcl_mod1), NULL); 
    //g_signal_connect (GTK_OBJECT (button3b2), "clicked", GTK_SIGNAL_FUNC (cb_kcl_mod1), NULL); 

    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);


    //----------BOX3 SUBSECTION:c
    box3c = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (box3), box3c, FALSE, FALSE, 0);
    gtk_widget_show (box3c);

    label = gtk_label_new ("Vision(CVUT-FORTH)");
    gtk_box_pack_start (GTK_BOX (box3c), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    button3c1 = gtk_check_button_new_with_label("2D3D recognition");
    gtk_box_pack_start (GTK_BOX (box3c), button3c1, TRUE, FALSE, 0);
    gtk_widget_show (button3c1);

    //button3c2 = gtk_check_button_new_with_label("3D localization");
    //gtk_box_pack_start (GTK_BOX (box3c), button3c2, TRUE, FALSE, 0);
    //gtk_widget_show (button3c2);
    
    //signal connects
    g_signal_connect (GTK_OBJECT (button3c1), "clicked", GTK_SIGNAL_FUNC (cb_forth_mod1), NULL); 
    //g_signal_connect (GTK_OBJECT (button3c2), "clicked", GTK_SIGNAL_FUNC (cb_forth_mod1), NULL);

    
    //button3c3 = gtk_check_button_new_with_label("module 3");
    //gtk_box_pack_start (GTK_BOX (box3c), button3c3, TRUE, FALSE, 0);
    //gtk_widget_show (button3c3);


    //separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    //gtk_widget_show (separator);


    //----------BOX3 SUBSECTION:d
    //box3d = gtk_vbox_new (FALSE, 0);
    //gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    //gtk_box_pack_start (GTK_BOX (box3), box3d, FALSE, FALSE, 0);
    //gtk_widget_show (box3d);

    //label = gtk_label_new ("CVUT");
    //gtk_box_pack_start (GTK_BOX (box3d), label, FALSE, FALSE, 0);
    //gtk_widget_show (label);

    //button3d1 = gtk_check_button_new_with_label("grasping");
    //gtk_box_pack_start (GTK_BOX (box3d), button3d1, TRUE, FALSE, 0);
    //gtk_widget_show (button3d1);

    //button3d2 = gtk_check_button_new_with_label("module 2");
    //gtk_box_pack_start (GTK_BOX (box3d), button3d2, TRUE, FALSE, 0);
    //gtk_widget_show (button3d2);
    
    //button3d3 = gtk_check_button_new_with_label("module 3");
    //gtk_box_pack_start (GTK_BOX (box3d), button3d3, TRUE, FALSE, 0);
    //gtk_widget_show (button3d3);




/*
    label = gtk_label_new ("map1 k1:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj1 = gtk_adjustment_new (0.5, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale), NULL);
    


    label = gtk_label_new ("map2 k2:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    adj2 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);

    label = gtk_label_new ("map3: k3");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj3 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj3));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj3), "value_changed",
                      G_CALLBACK (cb_digits_scale3), NULL);


    label = gtk_label_new ("map4: k4");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj4 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj4));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj4), "value_changed",
                      G_CALLBACK (cb_digits_scale4), NULL);

    label = gtk_label_new ("map5: k5");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj5 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj5));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj5), "value_changed",
                      G_CALLBACK (cb_digits_scale5), NULL);
*/
//    label = gtk_label_new ("map6: k6");
//    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
//    gtk_widget_show (label);
    
/*    adj6 = gtk_adjustment_new (1.0, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj6));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj6), "value_changed",
                      G_CALLBACK (cb_digits_scale6), NULL);
*/
/*    gtk_box_pack_start (GTK_BOX (box3), box5, FALSE, FALSE, 0);
    gtk_widget_show (box5);

    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);
*/
    //----------BOX6 SUBSECTION:2
/*    box6 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box6), 0);

    label = gtk_label_new ("CARTESIAN MAP COEFFICIENT:");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("map1 k1:");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
*/
/*    
   adj11 = gtk_adjustment_new (1.0, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj11));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj11), "value_changed",
                      G_CALLBACK (cb_digits_scale11), NULL);

    
    label = gtk_label_new ("map2 k2:");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    adj12 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj12));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj12), "value_changed",
                      G_CALLBACK (cb_digits_scale12), NULL);

    label = gtk_label_new ("map3: k3");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj13 = gtk_adjustment_new (0.5, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj13));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj13), "value_changed",
                      G_CALLBACK (cb_digits_scale13), NULL);


    label = gtk_label_new ("map4: k4");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj14 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj14));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj14), "value_changed",
                      G_CALLBACK (cb_digits_scale14), NULL);

    label = gtk_label_new ("map5: k5");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj15 = gtk_adjustment_new (0.1, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj15));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj15), "value_changed",
                      G_CALLBACK (cb_digits_scale15), NULL);

    label = gtk_label_new ("map6: k6");
    gtk_box_pack_start (GTK_BOX (box6), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj16 = gtk_adjustment_new (0.5, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj16));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box6), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj16), "value_changed",
                      G_CALLBACK (cb_digits_scale16), NULL);
    */  
/*
    gtk_box_pack_start (GTK_BOX (box3), box6, FALSE, FALSE, 0);
    gtk_widget_show (box6);
*/
    /*
    label = gtk_label_new ("Processing Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    */


   //scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    // Notice how this causes the scales to always be updated
    // continuously when the scrollbar is moved 
    //gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
    //                             GTK_UPDATE_CONTINUOUS);
    //gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    //gtk_widget_show (scrollbar;)

    //-----Check Buttons
    

    /*-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4
    */


/*    
    adjMotion = gtk_adjustment_new (0.3, 0.0,1.0,0.01, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjMotion));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box2), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjMotion), "value_changed",
                      G_CALLBACK (cb_digits_scaleMotion), NULL);
*/    
/*    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 10);
    gtk_widget_show (separator);
*/

    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
//    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
//    statusbar = gtk_statusbar_new ();
    //updateStatusbar(GTK_STATUSBAR (statusbar));
//    gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
//    gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);*/

//    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
 //   timeout_ID = gtk_timeout_add (100, timeout_CB, NULL);

    mainWindow=window;

    return window;
}

void configure(yarp::os::ResourceFinder rf){
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    _options.portName      = rf.check("name", 
                           Value("/visualizationGui"), 
                           "module name (string)").asString();
    _options.posX      = rf.check("x", 
                           Value(200), 
                           "module pos x (int)").asInt();
    _options.posY      = rf.check("y", 
                           Value(150), 
                           "module pos y (int)").asInt();

}

void setOptionsToDefault() {
    // Options defaults
    _options.refreshTime = 100;
    _options.outputEnabled = 0;
    _options.windWidth = 300;
    _options.windHeight = 300;
    _options.posX = 200;
    _options.posY = 150;
    _options.saveOnExit = 0;
}

bool openPorts() {


    string portStr = _options.portName.c_str();
    //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    _pOutPort=new Port;
    portStr+="default/command:o";
    bool ok = _pOutPort->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _visControlPort=new Port;
    portStr+="/visControl/command:o";
    ok = _visControlPort->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
        g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
        return false;
    }
    
    portStr = _options.portName.c_str();
    _iitPort1=new Port;
    portStr+="/reasoning/command:o";
    ok = _iitPort1->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _iitPort2=new Port;
   portStr+="/opc/command:o";
    ok = _iitPort2->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _iitPort3=new Port;
    portStr+="/episodic/command:o";
    ok = _iitPort3->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }



    portStr = _options.portName.c_str();
    _kclPort1=new Port;
    portStr+="/grasping/command:o";
    ok = _kclPort1->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }


    portStr = _options.portName.c_str();
    _forthPort1=new Port;
    portStr+="/localization/command:o";
    ok = _forthPort1->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _cvutPort1=new Port;
    portStr+="/identification/command:o";
    ok = _cvutPort1->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _nocaPort=new Port;
    portStr+="/support/command:o";
    ok = _nocaPort->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    portStr = _options.portName.c_str();
    _profactorPort=new Port;
    portStr+="/industrial/command:o";
    ok = _profactorPort->open(portStr.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }

    return true;
}

void closePorts() {


    _pOutPort->close();
    bool ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _pOutPort;
    _pOutPort = NULL;

    _visControlPort->close();

    _iitPort1->close();
    _iitPort2->close();
    _iitPort3->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _iitPort1;
    delete _iitPort2;
    delete _iitPort3;
    _iitPort1 = NULL;
    _iitPort2 = NULL;
    _iitPort3 = NULL;

    _kclPort1->close();
//    _kclPort2->close();
//    _kclPort3->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _kclPort1;
//    delete _kclPort2;
//    delete _kclPort3;
    _kclPort1 = NULL;
//    _kclPort2 = NULL;
//    _kclPort3 = NULL;


    _forthPort1->close();
//    _forthPort2->close();
//    _forthPort3->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _forthPort1;
//    delete _forthPort2;
//    delete _forthPort3;
    _forthPort1 = NULL;
//    _forthPort2 = NULL;
//    _forthPort3 = NULL;


    _cvutPort1->close();
//    _cvutPort2->close();
//    _cvutPort3->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _cvutPort1;
//    delete _cvutPort2;
//    delete _cvutPort3;
    _cvutPort1 = NULL;
//    _cvutPort2 = NULL;
//    _cvutPort3 = NULL;


    _nocaPort->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _nocaPort;
    _nocaPort = NULL;


    _profactorPort->close();
    ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _profactorPort;
    _profactorPort = NULL;



    





}

void cleanExit() {
    if (timeout_ID!=0)
        g_source_remove (timeout_ID);
    timeout_ID = 0;
    
    g_source_remove(timeout_update_ID);
    timeout_update_ID=0;

    gtk_main_quit ();
}

//-------------------------------------------------
// Main
//-------------------------------------------------
#undef main //ace leaves a "main" macro defined

int myMain(int argc, char* argv[]) {
    yarp::os::Network yarp;

    
    //initialize threads in gtk, copied almost verbatim from
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //g_thread_init (NULL);
    //gdk_threads_init ();
    //gdk_threads_enter ();
    createObjects();
    _frameN = 0;
    timeout_ID = 0;
    setOptionsToDefault();

    yarp::os::ResourceFinder* rf;
    rf=new ResourceFinder();
    rf->setVerbose(true);
    rf->setDefaultConfigFile("visualizationGui.ini"); //overridden by --from parameter
    rf->setDefaultContext("visualizationGui/conf");   //overridden by --context parameter
    rf->configure("ICUB_ROOT", argc, argv);
    configure(*rf);
    
    // Parse command line parameters, do this before
    // calling gtk_init(argc, argv) otherwise weird things 
    // happens
    if (!openPorts())
        goto exitRoutine;

    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    // create a new window
    mainWindow = createMainWindow();
    
    // Non Modal Dialogs
/*#if GTK_CHECK_VERSION(2,6,0)
    saveSingleDialog = createSaveSingleDialog();
   saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif
 */   // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), _options.posX, _options.posY);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    //ptr_portCallback->attach(&_resources);
    //ptr_portCallback->attach(&portFpsData);
    //ptr_inputPort->useCallback(*ptr_portCallback);

    if (_options.synch)
    {
        setSynchroMode();
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), true);
    }
    else
    {
        setTimedMode(_options.refreshTime);
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), false);
    }
    gtk_main ();

exitRoutine:
    // leave critical section here. From example
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //gdk_threads_leave ();

    closePorts();
    deleteObjects();
    return 0;
}

#ifdef YARP_WIN32_NOCONSOLE
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);
}
#else
int main(int argc, char* argv[]) {
    return myMain(argc, argv);
}
#endif

void printHelp() {
    g_print("visualizationGui usage:\n");
    g_print("--name: input port name (default: /visualizationGui)\n");
}

