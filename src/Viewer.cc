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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <limits>
#include <iostream>
#include <mutex>

namespace ORB_SLAM2
{
//W added pframedrawer2 and mpframedrawer2
Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, FrameDrawer *pFrameDrawer2, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpFrameDrawer2(pFrameDrawer2), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 410;
        mImageHeight = 308;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(180));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuLocalizationMode2("menu.Localization Mode 2",false,true); //W
    pangolin::Var<bool> menuCameraview("menu.Cameraview",false,false);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);


    //W
    pangolin::View& d_image = pangolin::Display("image")
           // .SetBounds(1.0/3.0-20.0/(mImageHeight+20)/2.0,2.0/3.0, pangolin::Attach::Pix(180),1.0, mImageWidth/(mImageHeight+20))
            .SetBounds(0.00,0.95, 0.44,0.76, mImageWidth/(mImageHeight+20)*1.1)
            .SetLock(pangolin::LockCenter, pangolin::LockCenter);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    //W image height +20 want er wordt nog iets bijgezet in frame drawer
    unsigned char* imageArray;
    pangolin::GlTexture imageTexture(mImageWidth,mImageHeight+20,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //W Matrix for pose of second camera
    pangolin::OpenGlMatrix Twc2;
    Twc2.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    //W
    cv::namedWindow("ORB-SLAM2: Current Frame2");

    bool bFollow = true;
    bool bLocalizationMode = false;

    bool bLocalizationMode2 = false;

    bool bCameraview = false;

    while(1)
    {
        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);

        //W
        cv::Mat im2 = mpFrameDrawer2->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame2",im2);
        cv::waitKey(mT);

        if(menuCameraview && !bCameraview)
        {
            // TODO ook weer ongedaan kunnen maken
            float mCameraviewpointX = 0;
            float mCameraviewpointY = 0;
            float mCameraviewpointZ = -0.001;
            float mCameraviewpointF = 170;
            menuFollowCamera = true;
            bFollow = true;
            menuShowGraph = false;
            menuShowKeyFrames = false;
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mCameraviewpointX,mCameraviewpointY,mCameraviewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mCameraviewpointF,mCameraviewpointF,512,389,0.1,1000));
            bCameraview = true;
        }
        else if(!menuCameraview && bCameraview)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
            bCameraview = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Twc2);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        //W
        if(menuLocalizationMode2 && !bLocalizationMode2)
        {
            mpSystem->ActivateLocalizationMode2();
            bLocalizationMode2 = true;
        }
        else if(!menuLocalizationMode2 && bLocalizationMode2)
        {
            mpSystem->DeactivateLocalizationMode2();
            bLocalizationMode2 = false;
        }


        if(menuCameraview)
        {
            cv::Mat imflip;
            cv::flip(im,imflip,0);
            imageArray = imflip.data;
            //use fast 4-byte alignment (default anyway) if possible

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            //glPixelStorei(GL_UNPACK_ALIGNMENT, (imflip.step & 4) ? 1 : 4);
            //set length of one complete row in data (doesn't need to equal image.cols)
            glPixelStorei(GL_UNPACK_ROW_LENGTH, imflip.step/imflip.elemSize());
            imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);
            d_image.Activate();
            glColor3f(1.0,1.0,1.0);
            imageTexture.RenderToViewport();
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc, Twc2);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();


        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
