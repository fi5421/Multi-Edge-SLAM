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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>

using namespace std;

namespace ORB_SLAM2
{
    // Edge-SLAM: map updates
    // Edge-SLAM: vector to hold local-map update
    vector<std::string> Tracking::mapVec;
    // Edge-SLAM: debug counters
    int Tracking::mapCallbackCount = 0;
    // Edge-SLAM: map update checkers
    const unsigned int Tracking::TIME_KF = 300;
    unsigned int Tracking::mnMapUpdateLastKFId = 0;
    bool Tracking::mapUpToDate = false;
    const unsigned int Tracking::LOCAL_MAP_SIZE = 6;
    bool Tracking::refKFSet = false;

    // Edge-SLAM: measure
    std::chrono::high_resolution_clock::time_point Tracking::msRelocLastMapUpdateStart = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point Tracking::msRelocLastMapUpdateStop;
    std::chrono::high_resolution_clock::time_point Tracking::msLastKeyFrameStart = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point Tracking::msLastKeyFrameStop;

    // Edge-SLAM: relocalization
    bool Tracking::msRelocStatus = false;
    const int Tracking::RELOC_FREQ = 500;

    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor) : mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
                                                                                                                                                                                                  mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
                                                                                                                                                                                                  mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
    {
        // Load camera parameters from settings file

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD)
        {
            mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD)
        {
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

        // Edge-SLAM: setting up connections
        string ip, server_ip, dummy;
        string port, server_port;
        cout << "Enter the device IP address: ";
        // getline(cin, ip);
        cout << "Enter the server IP address: ";
        // getline(cin, server_ip);
        ip = "127.0.0.1";
        server_ip = "127.0.0.1";

        cout << "Enter Port: ";
        getline(cin, port);
        cout << "Select Edge 1 or 2: ";
        // int edgeNumber;
        cin >> edgeNumber;
        cout<<"Edge Selected: "<<edgeNumber<<endl;
        cin.clear();
        cin.ignore();
        int *edgeNumberPointer=&edgeNumber;
        // cout<<std::format("KeyFrame Connection port Edge1 {} Edge2 {}\n",port_number,port_number+1);
        int port_number = std::stoi(port);
        cout << "KeyFrame Connection port Edge1 " << port_number << " Edge2 " << port_number + 1 << endl;
        // Edge-SLAM: keyframe connection
        // cout << "Enter the port number used for keyframe connection: ";
        // cout << "Enter the server port number used for keyframe connection: ";
        // getline(cin, server_port);
        cout << "here0" << ip << " " << port_number << " " << server_ip << " " << endl;
        keyframe_socket = new TcpSocket(ip, port_number, server_ip, port_number);
        keyframe_socket->sendConnectionRequest();
        port_number++;

        cout << "here\n";
        keyframe_socket2 = new TcpSocket(ip, port_number, server_ip, port_number);
        keyframe_socket2->sendConnectionRequest();

        keyframe_thread = new thread(&ORB_SLAM2::Tracking::tcp_send, &keyframe_queue, keyframe_socket, "keyframe", keyframe_socket2, edgeNumberPointer);
        // Edge-SLAM: frame connection

        port_number++;
        cout << "Keyframe thread created " << port_number << endl;
        getline(cin, dummy);
        // cout<<std::format("Frame Connection port Edge1 {} Edge2 {}\n",port_number,port_number+1);
        cout << "Frame Connection port Edge1 " << port_number << " Edge2 " << port_number + 1 << endl;

        // cout << "Enter the port number used for frame connection: ";
        // getline(cin, port_number);
        // cout << "Enter the server port number used for frame connection: ";
        // getline(cin, server_port);
        frame_socket = new TcpSocket(ip, port_number, server_ip, port_number);
        frame_socket->sendConnectionRequest();
        port_number++;
        frame_socket2 = new TcpSocket(ip, port_number, server_ip, port_number);
        frame_socket2->sendConnectionRequest();
        frame_thread = new thread(&ORB_SLAM2::Tracking::tcp_send, &frame_queue, frame_socket, "frame", frame_socket2, edgeNumberPointer);
        // Edge-SLAM: map connection

        port_number++;
        cout << "Frame thread created " << port_number << endl;
        getline(cin, dummy);
        // cout<<std::format("Map Connection port Edge1 {} Edge2 {}\n",port_number,port_number+1);
        cout << "Map Connection port Edge1 " << port_number << " Edge2 " << port_number + 1 << endl;

        // cout << "Enter the port number used for map connection: ";
        // getline(cin, port_number);
        // cout << "Enter the server port number used for map connection: ";
        // getline(cin, server_port);
        map_socket = new TcpSocket(ip, port_number, server_ip, port_number);
        map_socket->sendConnectionRequest();
        port_number += 1;
        map_socket2 = new TcpSocket(ip, port_number, server_ip, port_number);
        map_socket2->sendConnectionRequest();

        map_thread = new thread(&ORB_SLAM2::Tracking::tcp_receive, &map_queue, map_socket, 1, "map", map_socket2, edgeNumberPointer);

        // Edge-SLAM: debug
        cout << "log,Tracking::Tracking,done" << endl;
    }

    // Edge-SLAM
    void Tracking::mapCallback(const std::string &msg)
    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock2(mpMap->mMutexCallBackUpdate);

        if (!msRelocStatus)
        {
            if (mpMap->KeyFramesInMap() < LOCAL_MAP_SIZE)
            {
                cout << "log,Tracking::mapCallback,map is still initializing. skip update" << endl;
                return;
            }

            if (mapUpToDate)
            {
                cout << "log,Tracking::mapCallback,map is up to date. skip update" << endl;
                return;
            }

            // Check if high rate of keyframes are being created
            {
                // Edge-SLAM: measure
                msLastKeyFrameStop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msLastKeyFrameStop - msLastKeyFrameStart);
                auto dCount = duration.count();

                // As the map adds more than LOCAL_MAP_SIZE keyframes, decrease TIME_KF to accept the update sooner
                unsigned int divRate = (unsigned)(TIME_KF / ((mpMap->KeyFramesInMap() / LOCAL_MAP_SIZE) + 1));
                if (divRate < 50)
                    divRate = 0;

                if (dCount < divRate)
                {
                    cout << "log,Tracking::mapCallback,map update rejected due to high keyframe rate " << dCount << "ms; KFs in map " << mpMap->KeyFramesInMap() << "; divRate " << divRate << "ms" << endl;
                    return;
                }
            }
        }

        // Receive local-map update from server
        mapVec.clear();
        try
        {
            std::stringstream is(msg);
            boost::archive::text_iarchive ia(is);
            ia >> mapVec;
            is.clear();
        }
        catch (boost::archive::archive_exception e)
        {
            cout << "log,Tracking::mapCallback,map error: " << e.what() << endl;
            return;
        }

        // If not in relocalization mode, then discard relocalization map update
        if ((!msRelocStatus) && (mpMap->KeyFramesInMap() >= LOCAL_MAP_SIZE) && (mnMapUpdateLastKFId > 10) && (mapVec.size() != LOCAL_MAP_SIZE))
        {
            cout << "log,Tracking::mapCallback,relocalization map received after successfully relocalizing. map rejected. keyframes in map " << mpMap->KeyFramesInMap() << ". last MU KF id " << mnMapUpdateLastKFId << ". map size " << mapVec.size() << endl;
            return;
        }

        // Edge-SLAM: debug
        cout << "log,Tracking::mapCallback,accept and process map update " << ++mapCallbackCount << endl;

        // Keep current reference keyframe Id, and reset refKFSet
        long unsigned int refId = mpReferenceKF->mnId;
        refKFSet = false;

        // Before clearing the map, we should retrieve the ids of map points within last frame
        vector<unsigned long int> lastFrame_points_ids;
        vector<bool> lastFrame_points_availability;
        vector<unsigned long int> mvpLocalMapPoints_ids;
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                lastFrame_points_ids.push_back(pMP->mnId);
                lastFrame_points_availability.push_back(true);
            }
            else
            {
                lastFrame_points_availability.push_back(false);
            }
        }
        for (unsigned int i = 0; i < mvpLocalMapPoints.size(); i++)
        {
            mvpLocalMapPoints_ids.push_back(mvpLocalMapPoints[i]->mnId);
        }

        // Reset tracking thread to update it using a new local-map
        MUReset();

        // Reconstruct keyframes loop
        // For every keyframe, check its mappoints, then add them to tracking local-map
        // Add keyframe to tracking local-map
        for (int i = 0; i < (int)mapVec.size(); i++)
        {
            // Reconstruct keyframe
            KeyFrame *tKF = new KeyFrame();
            {
                try
                {
                    std::stringstream iis(mapVec[i]);
                    boost::archive::text_iarchive iia(iis);
                    iia >> tKF;
                    iis.clear();
                }
                catch (boost::archive::archive_exception e)
                {
                    cout << "log,Tracking::mapCallback,keyframe error: " << e.what() << endl;

                    // Clear
                    tKF = static_cast<KeyFrame *>(NULL);
                    continue;
                }
            }

            // Set keyframe fields
            tKF->setORBVocab(mpORBVocabulary);
            tKF->setMapPointer(mpMap);
            tKF->setKeyFrameDatabase(mpKeyFrameDB);
            tKF->ComputeBoW();

            // Get keyframe's mappoints
            vector<MapPoint *> vpMapPointMatches = tKF->GetMapPointMatches();

            // Iterate through current keyframe's mappoints and double check them
            for (size_t i = 0; i < vpMapPointMatches.size(); i++)
            {
                MapPoint *pMP = vpMapPointMatches[i];
                if (pMP)
                {
                    if (!pMP->isBad())
                    {
                        // If tracking id is set
                        if (pMP->trSet)
                        {
                            MapPoint *pMPMap = mpMap->RetrieveMapPoint(pMP->mnId, true);

                            if (pMPMap != NULL)
                            {
                                // Replace keyframe's mappoint pointer to the existing one in tracking local-map
                                tKF->AddMapPoint(pMPMap, i);

                                // Add keyframe observation to the mappoint
                                pMPMap->AddObservation(tKF, i);

                                // Delete duplicate mappoint
                                delete pMP;
                            }
                            else
                            {
                                // Add keyframe's mappoint to tracking local-map
                                mpMap->AddMapPoint(pMP);

                                // Add keyframe observation to the mappoint
                                pMP->AddObservation(tKF, i);
                                pMP->setMapPointer(mpMap); // We are not sending the map pointer in marshalling
                                pMP->SetReferenceKeyFrame(tKF);
                            }
                        }
                        else if (pMP->lmSet) // If tracking id is not set, but local-mapping id is set
                        {
                            MapPoint *pMPMap = mpMap->RetrieveMapPoint(pMP->lmMnId, false);

                            if (pMPMap != NULL)
                            {
                                // Replace keyframe's mappoint pointer to the existing one in tracking local-map
                                tKF->AddMapPoint(pMPMap, i);

                                // Add keyframe observation to the mappoint
                                pMPMap->AddObservation(tKF, i);

                                // Delete duplicate mappoint
                                delete pMP;
                            }
                            else
                            {
                                // Assign tracking id
                                pMP->AssignId(true);

                                // Add keyframe's mappoint to tracking local-map
                                mpMap->AddMapPoint(pMP);

                                // Add keyframe observation to the mappoint
                                pMP->AddObservation(tKF, i);
                                pMP->setMapPointer(mpMap); // We are not sending the map pointer in marshalling
                                pMP->SetReferenceKeyFrame(tKF);
                            }
                        }
                    }
                }
            }

            // Add keyframe to tracking local-map
            mpMap->AddKeyFrame(tKF);

            // Add Keyframe to database
            mpKeyFrameDB->add(tKF);

            // Set RefKF to previous RefKF if it is part of the map update
            if (tKF->mnId == refId)
            {
                mpReferenceKF = tKF;
                mLastFrame.mpReferenceKF = tKF;
                refKFSet = true;
            }

            // Clear
            tKF = static_cast<KeyFrame *>(NULL);
            vpMapPointMatches.clear();
        }

        // Get all keyframes in tracking local-map
        vector<KeyFrame *> vpKeyFrames = mpMap->GetAllKeyFrames();

        // Initialize Reference KeyFrame and other KF variables
        if (vpKeyFrames.size() > 0)
        {
            if (!refKFSet)
                mpReferenceKF = vpKeyFrames[0];
            else
            {
                if (mpReferenceKF->mnId < vpKeyFrames[0]->mnId)
                {
                    mpReferenceKF = vpKeyFrames[0];
                    refKFSet = false;
                }
            }

            mnLastKeyFrameId = vpKeyFrames[0]->mnFrameId;
            mpLastKeyFrame = vpKeyFrames[0];
            mnMapUpdateLastKFId = vpKeyFrames[0]->mnId;
        }

        // Edge-SLAM: debug
        cout << "log,Tracking::mapCallback,keyframes in update: ";

        // Iterate through keyframes and reconstruct connections
        for (std::vector<KeyFrame *>::iterator it = vpKeyFrames.begin(); it != vpKeyFrames.end(); ++it)
        {
            KeyFrame *pKFCon = *it;

            pKFCon->ReconstructConnections();

            // Edge-SLAM: debug
            cout << pKFCon->mnId << " ";

            // If RefKF has lower id than current KF, then set it to that KF
            if (mpReferenceKF->mnId < pKFCon->mnId)
            {
                mpReferenceKF = pKFCon;
                refKFSet = false;
            }

            // Update other KF variables
            if (mnMapUpdateLastKFId < pKFCon->mnId)
            {
                mnLastKeyFrameId = pKFCon->mnFrameId;
                mpLastKeyFrame = pKFCon;
                mnMapUpdateLastKFId = pKFCon->mnId;
            }
        }

        // Edge-SLAM: debug
        cout << endl;

        // Set current-frame RefKF
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // Get all map points in tracking local-map
        vector<MapPoint *> vpMapPoints = mpMap->GetAllMapPoints();

        // Iterate through all mappoints and SetReferenceKeyFrame
        for (std::vector<MapPoint *>::iterator it = vpMapPoints.begin(); it != vpMapPoints.end(); ++it)
        {
            MapPoint *rMP = *it;

            if ((unsigned)rMP->mnFirstKFid == rMP->GetReferenceKeyFrame()->mnId)
                continue;

            KeyFrame *rKF = mpMap->RetrieveKeyFrame(rMP->mnFirstKFid);

            if (rKF)
                rMP->SetReferenceKeyFrame(rKF);
        }

        // Updating mLastFrame
        for (int i = 0; i < mLastFrame.N; i++)
        {
            if (lastFrame_points_availability[i])
            {
                MapPoint *newpMP = mpMap->RetrieveMapPoint(lastFrame_points_ids[i], true);

                if (newpMP)
                {
                    mLastFrame.mvpMapPoints[i] = newpMP;
                }
                else
                {
                    mLastFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }
        }

        // We should update mvpLocalMapPoints for viewer
        mvpLocalMapPoints.clear();
        for (unsigned int i = 0; i < mvpLocalMapPoints_ids.size(); i++)
        {
            MapPoint *pMP = mpMap->RetrieveMapPoint(mvpLocalMapPoints_ids[i], true);
            if (pMP)
            {
                mvpLocalMapPoints.push_back(pMP);
            }
        }

        if (mpViewer)
        {
            cout << "mpViewer Called" << endl;
            mpViewer->Release();
        }
        // changed

        // Edge-SLAM: measure
        msRelocLastMapUpdateStart = std::chrono::high_resolution_clock::now();

        mapUpToDate = true;
        msRelocStatus = false;

        // Edge-SLAM: debug
        cout << "log,Tracking::mapCallback,local map update is done" << endl;
    }

    // Edge-SLAM: disabled
    /*
    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing=pLoopClosing;
    }
    */

    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
    }

    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
    {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
    {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
    {
        mImGray = im;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::Track()
    {
        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        // Edge-SLAM: check if there is a new map update received
        // MapUpdate regardless of initialization or not
        {
            string msg;
            if (map_queue.try_dequeue(msg))
                mapCallback(msg);
        }

        // Edge-SLAM: scope the locks
        {
            // Edge-SLAM: we also use this lock when a map update is received from the server. Check mapCallback() function
            // Get Map Mutex -> Map cannot be changed
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            unique_lock<mutex> lock2(mpMap->mMutexCallBackUpdate);

            if (mState == NOT_INITIALIZED)
            {
                if (mSensor == System::STEREO || mSensor == System::RGBD)
                    StereoInitialization();
                else
                    MonocularInitialization();

                mpFrameDrawer->Update(this);

                if (mState != OK)
                    return;
            }
            else
            {
                // System is initialized. Track Frame.
                bool bOK;

                // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
                if (!mbOnlyTracking)
                {
                    // Local Mapping is activated. This is the normal behaviour, unless
                    // you explicitly activate the "only tracking" mode.

                    if (mState == OK)
                    {
                        // Local Mapping might have changed some MapPoints tracked in last frame
                        CheckReplacedInLastFrame();

                        if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                        {
                            bOK = TrackReferenceKeyFrame();
                        }
                        else
                        {
                            bOK = TrackWithMotionModel();
                            if (!bOK)
                                bOK = TrackReferenceKeyFrame();
                        }
                    }
                    else
                    {
                        bOK = Relocalization();
                    }
                }
                else
                {
                    // Localization Mode: Local Mapping is deactivated

                    if (mState == LOST)
                    {
                        bOK = Relocalization();
                    }
                    else
                    {
                        if (!mbVO)
                        {
                            // In last frame we tracked enough MapPoints in the map

                            if (!mVelocity.empty())
                            {
                                bOK = TrackWithMotionModel();
                            }
                            else
                            {
                                bOK = TrackReferenceKeyFrame();
                            }
                        }
                        else
                        {
                            // In last frame we tracked mainly "visual odometry" points.

                            // We compute two camera poses, one from motion model and one doing relocalization.
                            // If relocalization is sucessfull we choose that solution, otherwise we retain
                            // the "visual odometry" solution.

                            bool bOKMM = false;
                            bool bOKReloc = false;
                            vector<MapPoint *> vpMPsMM;
                            vector<bool> vbOutMM;
                            cv::Mat TcwMM;
                            if (!mVelocity.empty())
                            {
                                bOKMM = TrackWithMotionModel();
                                vpMPsMM = mCurrentFrame.mvpMapPoints;
                                vbOutMM = mCurrentFrame.mvbOutlier;
                                TcwMM = mCurrentFrame.mTcw.clone();
                            }

                            bOKReloc = Relocalization();

                            if (bOKMM && !bOKReloc)
                            {
                                mCurrentFrame.SetPose(TcwMM);
                                mCurrentFrame.mvpMapPoints = vpMPsMM;
                                mCurrentFrame.mvbOutlier = vbOutMM;

                                if (mbVO)
                                {
                                    for (int i = 0; i < mCurrentFrame.N; i++)
                                    {
                                        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                        {
                                            mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                        }
                                    }
                                }
                            }
                            else if (bOKReloc)
                            {
                                mbVO = false;
                            }

                            bOK = bOKReloc || bOKMM;
                        }
                    }
                }

                mCurrentFrame.mpReferenceKF = mpReferenceKF;

                // If we have an initial estimation of the camera pose and matching. Track the local map.
                cout << "bok here" << bOK << endl;
                if (!mbOnlyTracking)
                {
                    if (bOK)
                        bOK = TrackLocalMap();
                    cout << "here1" << bOK << endl;
                }
                else
                {
                    // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                    // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                    // the camera we will use the local map again.
                    if (bOK && !mbVO)
                        bOK = TrackLocalMap();
                    cout << "here2" << bOK << endl;
                }

                if (bOK)
                    mState = OK;
                else
                    mState = LOST;

                // Update drawer
                mpFrameDrawer->Update(this);

                // If tracking were good, check if we insert a keyframe
                if (bOK)
                {
                    // Update motion model
                    if (!mLastFrame.mTcw.empty())
                    {
                        cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                        mVelocity = mCurrentFrame.mTcw * LastTwc;
                    }
                    else
                        mVelocity = cv::Mat();

                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                    // Clean VO matches
                    for (int i = 0; i < mCurrentFrame.N; i++)
                    {
                        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                        if (pMP)
                            if (pMP->Observations() < 1)
                            {
                                mCurrentFrame.mvbOutlier[i] = false;
                                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                            }
                    }

                    // Delete temporal MapPoints
                    for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                    {
                        MapPoint *pMP = *lit;
                        delete pMP;
                    }
                    mlpTemporalPoints.clear();

                    // Edge-SLAM: added needNKF
                    // Check if we need to insert a new keyframe
                    // if(NeedNewKeyFrame())
                    //    CreateNewKeyFrame();
                    int needNKF = NeedNewKeyFrame();
                    if (needNKF > 0)
                    {
                        CreateNewKeyFrame(needNKF);
                    }

                    // We allow points with high innovation (considererd outliers by the Huber Function)
                    // pass to the new keyframe, so that bundle adjustment will finally decide
                    // if they are outliers or not. We don't want next frame to estimate its position
                    // with those points so we discard them in the frame.
                    for (int i = 0; i < mCurrentFrame.N; i++)
                    {
                        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }
                }

                // Reset if the camera get lost soon after initialization
                if (mState == LOST)
                {
                    // Edge-SLAM: added mnMapUpdateLastKFId condition to prevent reseting if relocalization map received has <= 5 keyframes in it
                    if ((mpMap->KeyFramesInMap() <= 5) && (mnMapUpdateLastKFId <= 10))
                    {
                        cout << "Track lost soon after initialisation, reseting..." << endl;
                        mpSystem->Reset();
                        return;
                    }
                }

                if (!mCurrentFrame.mpReferenceKF)
                {
                    mCurrentFrame.mpReferenceKF = mpReferenceKF;
                }

                mLastFrame = Frame(mCurrentFrame);
            }

            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            if (!mCurrentFrame.mTcw.empty())
            {
                cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                mlRelativeFramePoses.push_back(Tcr);
                mlpReferences.push_back(mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState == LOST);
            }
            else
            {
                // This can happen if tracking is lost
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState == LOST);
            }

            // Edge-SLAM: debug
            cout << "log,Tracking::Track,end process frame " << mCurrentFrame.mnId << endl;
            if ((mCurrentFrame.mnId > 525) && (edgeNumber!=2)) {
                ofstream f;
                // f.open("SwitchTime.txt");
                // f << "-------------SWITCHING EDGES at time " << std::fixed << setprecision(6) <<  mCurrentFrame.mTimeStamp << "-------------" << endl;
                // f.close();
                edgeNumber = 2; 
            }
            cout << "Number of frames in MAP:" << mpMap->KeyFramesInMap() << endl;
        }
    }

    void Tracking::StereoInitialization()
    {
        if (mCurrentFrame.N > 500)
        {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Edge-SLAM: Add Keyframe to database
            mpKeyFrameDB->add(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);

                    // Edge-SLAM: added wchThread parameter
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap, 1);

                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }

            // Edge-SLAM
            pKFini->ComputeBoW();

            cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

            // Edge-SLAM
            // Edge-SLAM: moved to KeyframeCallback() function in local-mapping thread on server
            // mpLocalMapper->InsertKeyFrame(pKFini);
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << pKFini;
            std::string msg;
            msg = os.str();
            keyframe_queue.enqueue(msg);
            cout << "log,Tracking::StereoInitialization,create keyframe " << pKFini->mnId << endl;
            mapUpToDate = false;

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }

    void Tracking::MonocularInitialization()
    {
        if (!mpInitializer)
        {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100)
            {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        }
        else
        {
            // Try to initialize
            if ((int)mCurrentFrame.mvKeys.size() <= 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

            // Check if there are enough correspondences
            if (nmatches < 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            cv::Mat Rcw;                 // Current Camera Rotation
            cv::Mat tcw;                 // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
            {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
                {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Edge-SLAM: Add Keyframe to database
        mpKeyFrameDB->add(pKFini);
        mpKeyFrameDB->add(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++)
        {
            if (mvIniMatches[i] < 0)
                continue;

            // Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            // Edge-SLAM: added wchThread parameter
            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap, 1);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            // Add to Map
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
        {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            }
        }

        // Edge-SLAM
        // Edge-SLAM: moved to KeyframeCallback() function in local-mapping thread on server
        // mpLocalMapper->InsertKeyFrame(pKFini);
        // mpLocalMapper->InsertKeyFrame(pKFcur);
        // pKFini
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << pKFini;
            std::string msg;
            msg = os.str();
            keyframe_queue.enqueue(msg);
            cout << "log,Tracking::CreateInitialMapMonocular,create keyframe " << pKFini->mnId << endl;
        }
        // pKFcur
        {
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << pKFcur;
            std::string msg;
            msg = os.str();
            keyframe_queue.enqueue(msg);
            cout << "log,Tracking::CreateInitialMapMonocular,create keyframe " << pKFcur->mnId << endl;
        }
        mapUpToDate = false;

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15)
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
            {
                bCreateNew = true;
            }

            if (bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);

                // Edge-SLAM: added wchThread parameter
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i, 1);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    bool Tracking::TrackWithMotionModel()
    {
        ORBmatcher matcher(0.9, true);

        // Edge-SLAM: mLastFrame has a reference keyframe which may
        // not necessarily be in current map, especially if there is lag
        // so we are not updating its pose
        // Edge-SLAM: to fix this, we check if last-frame RefKF is set or not after
        // a map update
        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        if (refKFSet)
            UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;
        if (mSensor != System::STEREO)
            th = 15;
        else
            th = 7;
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

        // If few matches, uses a wider window search
        if (nmatches < 20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        if (nmatches < 20)
            return false;

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking)
        {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                    {
                        mnMatchesInliers++;
                    }
                }
                else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }

    // Edge-SLAM: NeedNewKeyFrame() function has been divided into two functions, one on client and the other in local-mapping thread on server
    // Edge-SLAM: Return type has been changed from bool to int(0==false, 1==more processing required, 2==true)
    int Tracking::NeedNewKeyFrame()
    {
        if (mbOnlyTracking)
        {
            return 0;
        }

        // Edge-SLAM
        const int nKFs = mpMap->KeyFramesInMap();

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR)
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        // Edge-SLAM: moved to NeedNewKeyFrame() function in local-mapping on server
        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);

        // Condition 1c: tracking is weak
        const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        // Edge-SLAM: split conditions between tracking on client and local-mapping on server
        /*
        if((c1a||c1b||c1c)&&c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if(bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if(mSensor!=System::MONOCULAR)
                {
                    if(mpLocalMapper->KeyframesInQueue()<3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
        */
        if (c2)
            if (c1c)
                return 2;
            else
                return 1;
        else
            return 0;
    }

    // Edge-SLAM: CreateNewKeyFrame() function has been customized for client side where some parts of it has been moved to NeedNewKeyFrame() in local-mapping on the server
    // Edge-SLAM: the function has been changed to receive an int parameter to tell the server whether additional processing on the keyframe is required or not
    void Tracking::CreateNewKeyFrame(int needNKF)
    {
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        // Edge-SLAM: set keyframe needNKF
        pKF->SetNeedNKF(needNKF);

        // Edge-SLAM: moved from NeedNewKeyFrame() above
        // Edge-SLAM: set keyframe passedF
        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames)
            pKF->SetPassedF(false);

        // Edge-SLAM: disabled. No need in Edge-SLAM since UpdateLocalKeyFrames() will take care of that. Also, it might cause
        // Edge-SLAM to lose track due to setting an unprocessed keyframe as reference kf
        // mpReferenceKF = pKF;
        // mCurrentFrame.mpReferenceKF = pKF;

        if (mSensor != System::MONOCULAR)
        {
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int>> vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);

                        // Edge-SLAM: added wchThread parameter
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap, 1);

                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }

        // Edge-SLAM
        // Edge-SLAM: moved to KeyframeCallback() function in local-mapping thread on server
        // mpLocalMapper->InsertKeyFrame(pKF);
        // mpLocalMapper->SetNotStop(false);
        pKF->ComputeBoW();
        mpMap->AddKeyFrame(pKF);
        // Edge-SLAM: Add Keyframe to database
        mpKeyFrameDB->add(pKF);
        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << pKF;
        std::string msg;
        msg = os.str();
        if (keyframe_queue.size_approx() >= (LOCAL_MAP_SIZE / 3))
        {
            string data;
            if (keyframe_queue.try_dequeue(data))
            {
                data.clear();
            }
        }
        keyframe_queue.enqueue(msg);
        // Edge-SLAM: debug
        cout << "log,Tracking::CreateNewKeyFrame,create keyframe " << pKF->mnId << endl;
        mapUpToDate = false;

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;

        // Edge-SLAM: debug
        cout << "log,Tracking::CreateNewKeyFrame,map has " << mpMap->MapPointsInMap() << " mappoints and " << mpMap->KeyFramesInMap() << " keyframes" << endl;

        // Edge-SLAM: measure
        msLastKeyFrameStart = std::chrono::high_resolution_clock::now();
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // Edge-SLAM: if pKFmax is an unprocessed kf, then check if it is fully accepted
        if (pKFmax && ((pKFmax->mnId <= mnMapUpdateLastKFId) || ((pKFmax->mnId > mnMapUpdateLastKFId) && (pKFmax->GetNeedNKF() == 2))))
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization()
    {
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Edge-SLAM
        // Edge-SLAM: send frame to server for relocalization
        // Edge-SLAM: measure
        // Edge-SLAM: check how long it has been since last received reloc map update
        msRelocLastMapUpdateStop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(msRelocLastMapUpdateStop - msRelocLastMapUpdateStart);
        auto dCount = duration.count();
        if ((dCount > RELOC_FREQ) || (!msRelocStatus))
        {
            // Edge-SLAM: debug
            cout << "log,Tracking::Relocalization,send frame " << mCurrentFrame.mnId << " to server for relocalization" << endl;

            // Pointer to current frame
            Frame *tF = &mCurrentFrame;

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << tF;
            std::string msg;
            msg = os.str();
            frame_queue.enqueue(msg);

            msRelocStatus = true;
            msRelocLastMapUpdateStart = std::chrono::high_resolution_clock::now();
        }

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if (vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50)
                    {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        // Edge-SLAM: fix memory leak from: https://github.com/raulmur/ORB_SLAM2/issues/429
        for (auto pSolver : vpPnPsolvers)
            delete pSolver;

        if (!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
    }

    // multiedge
    void Tracking::Reset()
    {
        // Edge-SLAM
        cout << "Starting Edge-SLAM client reset..." << endl;

        // Edge-SLAM: reset frame/keyframe/map queues
        string data;
        // Keyframe
        while (keyframe_queue.try_dequeue(data))
        {
            data.clear();
        }
        // Frame
        while (frame_queue.try_dequeue(data))
        {
            data.clear();
        }
        // Map
        while (map_queue.try_dequeue(data))
        {
            data.clear();
        }

        // Edge-SLAM
        // Edge-SLAM: send an existing keyframe to server as reset signal
        if (mpLastKeyFrame)
        {
            // Edge-SLAM: debug
            cout << "log,Tracking::Reset,send reset signal to server through keyframe " << mpLastKeyFrame->mnId << "\n";

            // Set reset flag
            mpLastKeyFrame->SetResetKF(true);

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << mpLastKeyFrame;
            std::string msg;
            msg = os.str();
            keyframe_queue.enqueue(msg);
        }
        else
        {
            // Edge-SLAM: debug
            cout << "log,Tracking::Reset,map is in initial state so can't send reset signal to server" << endl;
        }

        cout << "System Reseting" << endl;
        if (mpViewer)
        {
            cout << "in if";
            mpViewer->RequestStop();
            cout << "after requ";
            // while(!mpViewer->isStopped())
            // {cout<<"stuck here";
            //     usleep(3000);}
        }

        /*
        // Edge-SLAM: disabled
        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;
        */

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        // Edge-SLAM: reset Edge-SLAM variables
        // Edge-SLAM: debug counters
        mapCallbackCount = 0;
        // Edge-SLAM: map update checkers
        mnMapUpdateLastKFId = 0;
        mapUpToDate = false;
        refKFSet = false;
        // Edge-SLAM: measure
        msRelocLastMapUpdateStart = std::chrono::high_resolution_clock::now();
        msLastKeyFrameStart = std::chrono::high_resolution_clock::now();
        // Edge-SLAM: relocalization
        msRelocStatus = false;

        // Edge-SLAM: ORB-SLAM2 variables
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mnLastKeyFrameId = 0;
        mnLastRelocFrameId = 0;

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if (mpViewer)
        {
            cout << "mpViewer called here reset" << endl;
            mpViewer->Release();
        }
        // changed

        // Edge-SLAM
        cout << "Edge-SLAM client reset is complete" << endl;
    }

    void Tracking::MUReset()
    {
        // Edge-SLAM
        cout << "Starting map update reset..." << endl;

        cout << "System Reseting" << endl;
        // if(mpViewer)
        // {
        //     mpViewer->RequestStop();
        //     while(!mpViewer->isStopped())
        //         usleep(3000);
        // }

        // Edge-SLAM: disabled
        /*
        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;
        */

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        // Edge-SLAM
        cout << "Map update reset is complete" << endl;
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

    // Edge-SLAM: send function to be called on a separate thread
    void Tracking::tcp_send(moodycamel::BlockingConcurrentQueue<std::string> *messageQueue, TcpSocket *socketObject, std::string name, TcpSocket *nextEdgeSocket, int *edgeNumber)
    {
        std::string msg;
        bool success = true;

        // This is not a busy wait because wait_dequeue function is blocking
        do
        {
            if (*edgeNumber == 1)
            {
                if (!socketObject->checkAlive())
                {
                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_send,terminating thread" << endl;

                    break;
                }

                if (success)
                    messageQueue->wait_dequeue(msg);

                if ((!msg.empty()) && (msg.compare("exit") != 0))
                {
                    if (socketObject->sendMessage(msg) == 1)
                    {
                        success = true;
                        msg.clear();

                        // Edge-SLAM: debug
                        cout << "log,Tracking::tcp_send,sent " << name << endl;
                    }
                    else
                    {
                        success = false;
                    }
                }
            }
            else
            {
                if (!nextEdgeSocket->checkAlive())
                {
                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_send,terminating thread" << endl;

                    break;
                }

                if (success)
                    messageQueue->wait_dequeue(msg);

                if ((!msg.empty()) && (msg.compare("exit") != 0))
                {
                    if (nextEdgeSocket->sendMessage(msg) == 1)
                    {
                        success = true;
                        msg.clear();

                        // Edge-SLAM: debug
                        cout << "log,Tracking::tcp_send,sent " << name << endl;
                    }
                    else
                    {
                        success = false;
                    }
                }
            }
        } while (1);
    }

    // Edge-SLAM: receive function to be called on a separate thread
    void Tracking::tcp_receive(moodycamel::ConcurrentQueue<std::string> *messageQueue, TcpSocket *socketObject, unsigned int maxQueueSize, std::string name, TcpSocket *nextEdgeSocket, int *edgeNumber)
    {
        // Here the while(1) won't cause busy waiting as the implementation of receive function is blocking.
        while (1)
        {
            if (*edgeNumber == 1)
            {
                if (!socketObject->checkAlive())
                {
                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_receive,terminating thread" << endl;

                    break;
                }
                std::string msg = socketObject->recieveMessage();

                if (!msg.empty())
                {
                    if (messageQueue->size_approx() >= maxQueueSize)
                    {
                        string data;
                        if (messageQueue->try_dequeue(data))
                        {
                            data.clear();

                            // Edge-SLAM: debug
                            cout << "log,Tracking::tcp_receive,dropped " << name << endl;
                        }
                    }
                    messageQueue->enqueue(msg);

                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_receive,received " << name << endl;
                }
            }
            else
            {
                if (!nextEdgeSocket->checkAlive())
                {
                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_receive,terminating thread" << endl;

                    break;
                }
                std::string msg = nextEdgeSocket->recieveMessage();

                if (!msg.empty())
                {
                    if (messageQueue->size_approx() >= maxQueueSize)
                    {
                        string data;
                        if (messageQueue->try_dequeue(data))
                        {
                            data.clear();

                            // Edge-SLAM: debug
                            cout << "log,Tracking::tcp_receive,dropped " << name << endl;
                        }
                    }
                    messageQueue->enqueue(msg);

                    // Edge-SLAM: debug
                    cout << "log,Tracking::tcp_receive,received " << name << endl;
                }
            }
        }
    }

    // Edge-SLAM: added destructor to destroy all connections on object termination
    Tracking::~Tracking()
    {
        keyframe_socket->~TcpSocket();
        frame_socket->~TcpSocket();
        map_socket->~TcpSocket();
        // This is just a dummy enqueue to unblock the wait_dequeue function in tcp_send()
        keyframe_queue.enqueue("exit");
        frame_queue.enqueue("exit");
    }

} // namespace ORB_SLAM
