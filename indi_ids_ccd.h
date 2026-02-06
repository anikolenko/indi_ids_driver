/*
  IDS CCD simple driver
 Copyright (C) 2026 Andy Nikolenko (andy.nikolenko@gmail.com)
 Copyright (C) 2021 Jasem Mutlaq (mutlaqja@ikarustech.com)

 Multiple device support Copyright (C) 2013 Peter Polakovic (peter.polakovic@cloudmakers.eu)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 */

#pragma once


#include <indiccd.h>

//#ifdef LONGLONG
//#undef LONGLONG
//#endif
#define LONGLONG ueye_longlong
#include "ueye.h"
#undef LONGLONG
// Restore LONGLONG for INDI/FITS
#define LONGLONG long long

using namespace std;

#define DEVICE struct usb_device *

struct ImageBuffer
{
    char *memory  = nullptr;
    int id;
};

class IDSCCD : public INDI::CCD
{
    public:
        IDSCCD(DEVICE device, const char *name);
        IDSCCD();
        virtual ~IDSCCD();

        const char *getDefaultName() override;

        bool initProperties() override;
        bool updateProperties() override;

        bool Connect() override;
        bool Disconnect() override;
        bool StartExposure(float duration) override;
        bool AbortExposure() override;

    protected:
        void TimerHit() override;
      //  virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n);
        virtual bool UpdateCCDFrame(int x, int y, int w, int h) override;
        virtual bool UpdateCCDBin(int binx, int biny) override;
     //   virtual bool UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType) override;

        bool setupCameraDefault();
        void setBayerEnabled(bool onOff);
        bool captureImage();

        // Guide Port
      /*  virtual IPState GuideNorth(uint32_t ms) override;
        virtual IPState GuideSouth(uint32_t ms) override;
        virtual IPState GuideEast(uint32_t ms) override;
        virtual IPState GuideWest(uint32_t ms) override;
*/
        struct timeval ExpStart
        {
            0, 0
        };

        bool AbortPrimaryFrame { false };

    private:

        SENSORINFO si;
        CAMINFO ci;
        DEVICE device;        

        ImageBuffer buffer;

        HCAM hCam = 0;

        int bitsPerPixel = 16;
        bool InProcess { false };

        int sensorWidth = 0;
        int sensorHeight = 0;
        float sensorPixelSize  = 0;
        char sensorName[32];

        float gainMin { 0 };
        float gainMax { 100 };
        
        bool m_MonoCamera { true };

        char m_SensorBayer1stPixel { 'R' };
        
        char m_Name[MAXINDINAME];
   
        INDI::PropertyNumber GainNP {1};
        INDI::PropertyNumber CCDTempNP {1};

        INDI::ElapsedTimer m_ElapsedTimer;
        double ExposureRequest;
        double TemperatureRequest;

     //   int SetTemperature(double temperature) override;
        IS_RECT getAOI();
        bool setAOI(IS_RECT aoi);
        float CalcTimeLeft(timeval start, float req);
        double getCameraTemp(UINT nDeviceId = 1);
        double decodeTemperature(WORD wTemperature);
        bool getCameraImageFormats();
        bool setupParams();
     //   bool allocateImageMemory(int w, int h, int bpp);
        void freeImageMemory();

    

};
