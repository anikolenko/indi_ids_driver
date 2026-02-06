/*
 IDS CCD simple driver
 
 Copyright (C) 2026 Andy Nikolenko (andy.nikolenko@gmail.com)

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

#include <memory>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <memory>
#include <deque>

#include "config.h"
#include "indi_ids_ccd.h"

std::unique_ptr<IDSCCD> ids_ccd(new IDSCCD());

IDSCCD::IDSCCD()
{
    snprintf(this->m_Name, MAXINDINAME, "IDS CCD");
    setDeviceName(this->m_Name);
    setVersion(GENERIC_VERSION_MAJOR, GENERIC_VERSION_MINOR);
}

IDSCCD::~IDSCCD()
{
}

const char *IDSCCD::getDefaultName()
{
    return "IDS CCD";
}

void IDSCCD::setBayerEnabled(bool onOff)
{
    if (onOff)
    {
        SetCCDCapability(GetCCDCapability() | CCD_HAS_BAYER);
        BayerTP[CFA_OFFSET_X].setText("0");
        BayerTP[CFA_OFFSET_Y].setText("0");
        BayerTP[CFA_TYPE].setText("RGGB");
    }
    else
    {
        SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
    }
}



bool IDSCCD::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{

    if (isDeviceNameMatch(dev))
    {

        if (GainNP.isNameMatch(name))
        {

            GainNP.update(values, names, n);

            INT nMaster = GainNP[0].getValue(),iRet;
            iRet = is_SetHardwareGain (hCam, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
            LOGF_INFO("IDS: old master gain factor (R) : %d , new : %d", iRet, nMaster);
            iRet = is_SetHardwareGain (hCam, nMaster, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

            GainNP.setState(IPS_OK);
            GainNP.apply();
        }    
    }
    // If we didn't process anything above, let the parent handle it.
    return INDI::CCD::ISNewNumber(dev,name,values,names,n);
}




bool IDSCCD::initProperties()
{
    // Always call parent initProperties first.
    INDI::CCD::initProperties();

    // Next, let's setup which capabilities are offered by our camera?
  //  uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_COOLER | CCD_HAS_BAYER;
  //  SetCCDCapability(cap);

      // Add Debug, Simulator, and Configuration controls
    addAuxControls();
  
    setDefaultPollingPeriod(500);

    // Gain property
    GainNP[0].fill("GAIN", "Gain", "%.0f", gainMin, gainMax, 1, 0); // Max gain can vary, set a reasonable default
    GainNP.fill(getDeviceName(), "CCD_GAIN", "Gain", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // Sensor temp reading
    CCDTempNP[0].fill("CCDTEMP", "CCD Temperature [°C]", "%.1f", -128.5, 128.5, 0.1, 0);
    CCDTempNP.fill(getDeviceName(), "CCDTEMP", "CCD Temp", IMAGE_INFO_TAB, IP_RO, 60, IPS_IDLE);

    // Add configuration for Debug
    addDebugControl();

    // We're done!
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
/// updateProperties is called whenever Connect/Disconnect event is triggered.
/// If we are now connected to the camera, we might want to query some parameters.
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD

        defineProperty(GainNP);
        defineProperty(CCDTempNP);
        
        setupParams();
        SetTimer(getCurrentPollingPeriod());
    }else{
        //deleteProperty(GainNP.name);
        deleteProperty(GainNP);
        deleteProperty(CCDTempNP);
    }

    return true;
}


double IDSCCD::decodeTemperature(WORD wTemperature)
{
    // Special value: no temperature sensor
    if (wTemperature == 0x87F9)   // -127.9 °C
        return NAN;
    int sign = (wTemperature & 0x8000) ? -1 : 1;
    int integer_part = (wTemperature >> 4) & 0x7F;
    int fractional_part = wTemperature & 0x0F;
    return sign * ( 10 * integer_part + fractional_part) / 10;
}


double IDSCCD::getCameraTemp(UINT nDeviceId )
{

    IS_DEVICE_INFO deviceInfo;
    memset(&deviceInfo, 0, sizeof(IS_DEVICE_INFO));
 
    INT nRet = is_DeviceInfo((HIDS)(nDeviceId | IS_USE_DEVICE_ID),
                        IS_DEVICE_INFO_CMD_GET_DEVICE_INFO,
                        (void*)&deviceInfo, sizeof(deviceInfo));
    if (nRet == IS_SUCCESS)
    {        
        return decodeTemperature(deviceInfo.infoDevHeartbeat.wTemperature);
    }else{
        return 0;
    }

}

bool IDSCCD::setupCameraDefault()
{
    if (!hCam)
        return false;

    // Get sensor and camera info 
    
    if (is_GetSensorInfo(hCam, &si) != IS_SUCCESS)
    {
        LOG_INFO("IDS: is_GetSensorInfo failed");
        return false;
    }

    if (is_GetCameraInfo(hCam, &ci) != IS_SUCCESS)
    {
        LOG_INFO("IDS: is_GetCameraInfo failed");
        return false;
    }


    sensorWidth = si.nMaxWidth;
    sensorHeight = si.nMaxHeight;
    sensorPixelSize = (float)si.wPixelSize / 100;
    //sensorName = &si.strSensorName;

    LOGF_INFO("IDS: sensor name: %s %s (#%s)", ci.ID, si.strSensorName, ci.SerNo);
    LOGF_INFO("IDS: sensor max image size: %dx%d, pix: %4.2f μm", sensorWidth, sensorHeight, sensorPixelSize);
    LOGF_INFO("IDS: sensor analog master gain is %s", si.bMasterGain ? "Available": "Not Available");
    LOGF_INFO("IDS: sensor shutter type: %s", si.bGlobShutter ? "Global": "Rolling");

    LOGF_INFO("IDS: sensor Color mode: %s", si.nColorMode == IS_COLORMODE_MONOCHROME ? "Monochrome" : "Bayer/Other");   

    m_MonoCamera = si.nColorMode == IS_COLORMODE_MONOCHROME ? true : false;
   
    // determine first left top pixel. Not used for now
    if(m_MonoCamera == false)
    {
        switch(si.nUpperLeftBayerPixel) {
            case 0:
                m_SensorBayer1stPixel = 'R';
                break;
            case 1:
                m_SensorBayer1stPixel = 'G';
                break;
            case 2:            
                m_SensorBayer1stPixel = 'B';
                break;
        }        
    }
 
    IS_RECT rectAOI;
    rectAOI.s32X     = 0;
    rectAOI.s32Y     = 0;
    rectAOI.s32Width = sensorWidth;
    rectAOI.s32Height = sensorHeight;

    setAOI(rectAOI);
    

    if (is_SetGainBoost(hCam, IS_SET_GAINBOOST_ON) != IS_SUCCESS)
    {
        LOG_INFO("IDS: is_SetGainBoost failed");
        return false;
    }else{
        LOG_INFO("IDS: enabling Gain boost...");
    }

    // set min pixel clock

    UINT nRange[3];
    ZeroMemory(nRange, sizeof(nRange));
 
    // Get pixel clock range
    if(IS_SUCCESS == is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange)))
    {
        UINT nMin = nRange[0];
        //UINT nMax = nRange[1];
        //UINT nInc = nRange[2];
        int nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET, (void*)&nMin, sizeof(nMin));
        if(nRet == IS_SUCCESS) 
        {
            LOGF_INFO("IDS: setting min pixel clock(%d)",nMin);    
        }    
    }

    int colormode = IS_CM_SENSOR_RAW12; 
    if (is_SetColorMode(hCam, colormode) != IS_SUCCESS)
    {
        LOG_INFO("IDS: is_SetColorMode(IS_CM_SENSOR_RAW16) failed, trying IS_COLORMODE_BAYER");
        // Fallback to generic Bayer colormode ( 8-bit))
        if (is_SetColorMode(hCam, IS_COLORMODE_BAYER) != IS_SUCCESS)
        {
            LOG_INFO("IDS: is_SetColorMode(IS_COLORMODE_BAYER) also failed");
        }
    }
    else
    {

    }

    //  is_SetBayerConversion(hCam, IS_SET_BAYER_CV_NORMAL); - obsolete function

 /*   if(IS_SUCCESS == is_SetColorConverter (hCam, colormode, IS_CONV_MODE_NONE))
    {
        LOG_INFO("IDS: Color Conversion set to SENSOR_RAW16");
    }
    else
    {
        LOG_INFO("IDS: Color Conversion cannot be set !");
    }
*/

    // Configure Bayer conversion mode to 'normal' (no post-debayer by SDK) so we get raw data
    // Note: is_SetBayerConversion controls debayering behavior of the driver layer.

    // Enable long exposure capability (important - many uEye Peak/legacy modes differ)
    uint enable = 1;
    if (is_Exposure(hCam, IS_EXPOSURE_CMD_SET_LONG_EXPOSURE_ENABLE, &enable, sizeof(enable)) != IS_SUCCESS)
    {
        LOG_INFO("IDS: enabling long exposure failed (not all models support it)");
        // Not fatal; continue but log
    }else{
        LOG_INFO("IDS: long exposure mode enabled..");
    }

    double dblRange[3];
    double dblMin, dblMax, dblInc;
     
    INT nRet = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_LONG_EXPOSURE_RANGE,
                          (void*)dblRange, sizeof(dblRange));
     
    if (nRet == IS_SUCCESS)
    {
      dblMin = dblRange[0];
      dblMax = dblRange[1];
      dblInc = dblRange[2];

      PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", dblMin / 1000.0, dblMax / 1000.0, 1, false);      
      LOGF_INFO("IDS: Exposure ranges avilable: %f - %f (ms)",dblMin,dblMax,dblInc);
    }

    return true;
}



bool IDSCCD::getCameraImageFormats()
{

     
    // Get number of available formats and size of list
    UINT count;
    UINT bytesNeeded = sizeof(IMAGE_FORMAT_LIST);
    is_ImageFormat(hCam, IMGFRMT_CMD_GET_NUM_ENTRIES, &count, sizeof(count));
    bytesNeeded += (count - 1) * sizeof(IMAGE_FORMAT_INFO);
    void* ptr = malloc(bytesNeeded);

    LOGF_INFO("IDS: %d image formats available..", count);
     
    // Create and fill list
    IMAGE_FORMAT_LIST* pformatList = (IMAGE_FORMAT_LIST*) ptr;
    pformatList->nSizeOfListEntry = sizeof(IMAGE_FORMAT_INFO);
    pformatList->nNumListElements = count;
    is_ImageFormat(hCam, IMGFRMT_CMD_GET_LIST, pformatList, bytesNeeded);
     
    // Prepare for creating image buffers
   // char* pMem = NULL;
  //  int memID = 0;
     
    IMAGE_FORMAT_INFO formatInfo;
    for (UINT i = 0; i < count; i++)
    {
        formatInfo = pformatList->FormatInfo[i];
        int width = formatInfo.nWidth;
        int height = formatInfo.nHeight;
        LOGF_INFO("ID# %d: %dx%d (%s)",formatInfo.nFormatID,width,height,formatInfo.strFormatName);
        // Allocate image mem for current format, set format
        //nRet = is_AllocImageMem(hCam, width, height, 24, &pMem, &memID);
        //nRet = is_SetImageMem(hCam, pMem, memID);  
        //nRet = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatInfo.nFormatID, sizeof(formatInfo.nFormatID));
     
        // Capture image
        //nRet = is_FreezeVideo(hCam, IS_WAIT);
    }
    return true;
}

bool IDSCCD::Connect()
{
    LOG_INFO("Attempting to find the IDS CCD...");

    /**********************************************************
    *
    *
    *
    *  IMPORRANT: Put here your CCD Connect function
    *  If you encameraCounter an error, send the client a message
    *  e.g.
    *  LOG_INFO( "Error, unable to connect due to ...");
    *  return false;
    *
    *
    **********************************************************/


    INT pnNumCams;
    INT camStatus = is_GetNumberOfCameras(&pnNumCams);
    if(IS_SUCCESS == camStatus)
    {
        if(pnNumCams > 1) 
        {
            LOGF_INFO("IDS: found %d IDS uEye cameras connected! using only the first one !", pnNumCams);
        }else{
            if(pnNumCams == 0)
            {
                LOG_INFO("IDS: no camera found");
                return false;
            }
        }        
    }else{
        LOGF_INFO("IDS uEye Error:  %d", camStatus);
        return false;
    }

    // Initialize uEye camera
    int ret = is_InitCamera(&hCam, nullptr);
    if (ret != IS_SUCCESS || hCam == 0)
    {
        LOGF_INFO("IDS: is_InitCamera failed: %d", ret);
        return false;
    }

    // Setup default camera parameters: full sensor AOI, Bayer raw12, long exposure enabled
    if (!setupCameraDefault())
    {
        LOG_INFO("IDS: setupCameraDefault failed");
        Disconnect();
        return false;
    }

    LOGF_INFO("IDS: camera connected (handle=%d)", (int)hCam);

    /// set Sofrware Trigger Mode

    ret = is_SetExternalTrigger (hCam, IS_SET_TRIGGER_SOFTWARE);
    if(ret != IS_SUCCESS)
    {
        LOG_ERROR("IDS: FAILED is_SetExternalTrigger");
    }

    uint32_t cap = CCD_CAN_BIN | CCD_CAN_ABORT | CCD_CAN_SUBFRAME; // | CCD_HAS_STREAMING
 
    SetCCDCapability(cap);
    setBayerEnabled(m_MonoCamera == true ? false : true);
    SetTimer(getCurrentPollingPeriod());

    return true;
}


bool IDSCCD::Disconnect()
{
    // Free image memory
    freeImageMemory();

    if (hCam)
    {
        is_ExitCamera(hCam);
        hCam = 0;
    }
    LOG_INFO("Camera is offline.");

    return INDI::CCD::Disconnect();
}


///////////////////////////////////////////////////////////////////////////////////////
/// This is called after connecting to the camera to setup some parameters.
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::setupParams()
{
    float x_pixel_size, y_pixel_size;
    int bit_depth = bitsPerPixel;
    int x_1, y_1, x_2, y_2;

    /**********************************************************
    *
    *
    *
    *  IMPORRANT: Get basic CCD parameters here such as
    *  + Pixel Size X
    *  + Pixel Size Y
    *  + Bit Depth?
    *  + X, Y, W, H of frame
    *  + Temperature
    *  + ...etc
    *
    *
    *
    **********************************************************/

    ///////////////////////////
    // 1. Get Pixel size
    ///////////////////////////
    // Actucal CALL to CCD to get pixel size here
    x_pixel_size = y_pixel_size = sensorPixelSize; // 3.45;
    
    ///////////////////////////
    // 2. Get Frame
    ///////////////////////////

    // Actucal CALL to CCD to get frame information here
    x_1 = y_1 = 0;
    x_2       = sensorWidth;
    y_2       = sensorHeight;

    ///////////////////////////
    // 3. Get temperature
    ///////////////////////////
    // Setting sample temperature -- MAKE CALL TO API FUNCTION TO GET TEMPERATURE IN REAL DRIVER

    double temp = getCameraTemp();
    if(temp !=  NAN) 
    {
        CCDTempNP[0].setValue(temp);
        CCDTempNP.setState(IPS_OK);
        CCDTempNP.apply();
        LOGF_INFO("The CCD Temperature is %f", temp);

    }
    else
    {
        LOG_INFO("IDS: camera Temp data: Not Available ");
    }    

    SetCCDParams(x_2 - x_1, y_2 - y_1, bit_depth, x_pixel_size, y_pixel_size);
    PrimaryCCD.setBin(1, 1);   
    PrimaryCCD.setNAxis(2); // for RAW

    CaptureFormat raw = {"INDI_RAW", "RAW 16", (uint8_t)bitsPerPixel, true};  // default
    // CaptureFormat mono16 = {"INDI_MONO_16", "Mono 16", 16, false};

    // addCaptureFormat(mono16);
    addCaptureFormat(raw);

    // Now we usually do the following in the hardware
    // Set Frame to LIGHT or NORMAL
    // Set Binning to 1x1
    /* Default frame type is NORMAL */

    // Let's calculate required buffer
    //int nbuf;
    //nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8; //  this is pixel cameraCount
    //nbuf += 512;                                                                  //  leave a little extra at the end
    //PrimaryCCD.setFrameBufferSize(nbuf);

    return true;
}




IS_RECT IDSCCD::getAOI()
{
    IS_RECT aoi;
    if (is_AOI(hCam, IS_AOI_IMAGE_GET_AOI, (void*)&aoi, sizeof(aoi)) != IS_SUCCESS)
    {
        LOG_ERROR("Error getting AOI");
    }
    return aoi;
}

bool IDSCCD::setAOI(IS_RECT aoi)
{
    int ret;
    if ((ret = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&aoi, sizeof(aoi))) != IS_SUCCESS)
    {
        LOG_ERROR("IDS: Setting AOI failed");
        switch(ret) 
        {
            case IS_BAD_STRUCTURE_SIZE:
                LOG_ERROR("IDS: AOI:IS_BAD_STRUCTURE_SIZE");
                break;
            case IS_TRIGGER_ACTIVATED:
                LOG_ERROR("IDS: AOI:IS_TRIGGER_ACTIVATED");
                break;
            case IS_INVALID_PARAMETER:
                LOGF_ERROR("IDS: AOI:parameters is outside the valid range or is not supported for this sensor or is not available in this mode: %d,%d,%d,%d",aoi.s32X,aoi.s32Y,aoi.s32Width,aoi.s32Height);
                break;               
            default:
                LOGF_ERROR("IDS: AOI:ERROR: %d",ret);
        }
        return false;
    }
    return true;
}


void IDSCCD::freeImageMemory()
{

    if (hCam && buffer.memory && is_FreeImageMem(hCam, buffer.memory, buffer.id) != IS_SUCCESS)
    {
        LOG_ERROR("Error freeing image memory.");
    }

    buffer.memory = nullptr;
}



float IDSCCD::CalcTimeLeft(timeval start, float req)
{
    double timesince;
    double timeleft;
    struct timeval now
    {
        0, 0
    };
    gettimeofday(&now, nullptr);

    timesince =
        (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) - (double)(start.tv_sec * 1000.0 + start.tv_usec / 1000);
    timesince = timesince / 1000;
    timeleft  = req - timesince;
    return timeleft;
}


///////////////////////////////////////////////////////////////////////////////////////
/// Start Exposure
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::StartExposure(float duration)
{

    PrimaryCCD.setExposureDuration(duration);
    ExposureRequest = duration;
    AbortPrimaryFrame = false;
 
    LOGF_INFO("Taking a %g seconds frame...", ExposureRequest);

    // 2. Set the camera hardware exposure time, values are in ms 
    double actualExp = duration  * 1000;
    if (is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, &actualExp, sizeof(actualExp)) != IS_SUCCESS)
    {
        LOG_ERROR( "IDS: Error, unable to start exposure.");
        return false;
    }else{
        LOGF_INFO( "IDS: Started exposure =  %f (s)", actualExp / 1000);
    }

    IS_RECT aoi = getAOI();
    uint32_t nbuf = aoi.s32Height * aoi.s32Width * bitsPerPixel / 8  + 512;   

    // Allocate temporary buffer, only if camera params were changed or first time

    if( (uint32_t)PrimaryCCD.getFrameBufferSize() != nbuf || !buffer.memory)
    {
        PrimaryCCD.setFrameBufferSize(nbuf);
        PrimaryCCD.setResolution(aoi.s32Width, aoi.s32Height);
        PrimaryCCD.setBPP(bitsPerPixel);
        PrimaryCCD.setNAxis(2);
        PrimaryCCD.setFrameBuffer(new uint8_t[nbuf]);
        buffer.memory = (char*)PrimaryCCD.getFrameBuffer();
        LOGF_INFO("IDS: allocated image mem id=%d, ptr=%p, size: %d Kb", buffer.id, (void*)buffer.memory,nbuf/1024);
    }

    buffer.memory = (char*)PrimaryCCD.getFrameBuffer();
    if (!buffer.memory)
    {
        LOG_ERROR("INDI: Couldn't get PrimaryCCD frame buffer");
        return false;
    }

    // Set INDI framebuffer for direct uEye usage
    int nRet = is_SetAllocatedImageMem(hCam, aoi.s32Width, aoi.s32Height, bitsPerPixel, buffer.memory, &buffer.id);
    if(nRet != IS_SUCCESS)
    {
        LOG_ERROR("IDS: FAILED is_SetAllocatedImageMem");
        return false;
    }

    // Make buffer memory Active to use by camera
    if (is_SetImageMem(hCam, buffer.memory, buffer.id) != IS_SUCCESS)
    {            
        LOG_INFO("IDS: is_SetImageMem failed");
        return false;
    }
    

    InProcess = true;
    gettimeofday(&ExpStart, nullptr);

    // enable LED blink during exposure

    int nCurrentState = IO_LED_BLINK_ENABLE;
    nRet = is_IO(hCam, IS_IO_CMD_LED_SET_STATE, (void*)&nCurrentState, sizeof(nCurrentState));
     
    // Start the exposure on the hardware
    // IS_DONT_WAIT allows the function to return immediately while the sensor integrates
  
    nRet = is_FreezeVideo(hCam, IS_DONT_WAIT);
    if (nRet != IS_SUCCESS)
    {
        LOGF_ERROR("IDS: is_FreezeVideo failed: %d", nRet);
        if(nRet == IS_TIMED_OUT)
        {
            LOGF_ERROR("IDS: ..camera timeout: %d", nRet); 
        }
        return false;
    }

    ExposureRequest = duration;
    InExposure      = true;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
/// Abort Exposure
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::AbortExposure()
{
    InExposure = false;
    is_ForceTrigger(hCam);
    AbortPrimaryFrame = true;
    return true;
}



bool IDSCCD::captureImage()
{
 
    // disable LED blink - end exposure

    int nCurrentState = IO_LED_BLINK_DISABLE;
    is_IO(hCam, IS_IO_CMD_LED_SET_STATE, (void*)&nCurrentState, sizeof(nCurrentState));
         
  //  is_ForceTrigger(hCam);

    // delay required to grab image - image should appear already in INDI buffer after delay !
    usleep(100000 * 7); // May need to be adjusted !!
    is_FreeImageMem(hCam, buffer.memory, buffer.id);
    return true;

}


void IDSCCD::TimerHit()
{
    uint32_t nextTimer = getCurrentPollingPeriod();

    //  No need to reset timer if we are not connected anymore
    if (!isConnected())
        return;

    if (InExposure)
    {
        if (AbortPrimaryFrame)
        {
            InExposure        = false;
            AbortPrimaryFrame = false;
        }
        else
        {
            float timeleft;
            timeleft = CalcTimeLeft(ExpStart, ExposureRequest);

            if (timeleft < 0)
                timeleft = 0;

            PrimaryCCD.setExposureLeft(timeleft);

            if (timeleft < 1.0)
            {
                if (timeleft <= 0.001)
                {
                    int slv;
                    slv = 100000 * timeleft;
                    usleep(slv);
                  
                    PrimaryCCD.setExposureLeft(0);

                    InExposure = false;
                    captureImage();

                    CCDTempNP[0].setValue(getCameraTemp());
                    CCDTempNP.setState(IPS_OK);
                    CCDTempNP.apply();

                    ExposureComplete(&PrimaryCCD);
                }
                else
                {
                    nextTimer = timeleft * 1000;
                }
            }
        }
            
    }
    SetTimer(nextTimer);
}



///////////////////////////////////////////////////////////////////////////////////////
/// Update Camera Region of Interest ROI
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    /* Add the X and Y offsets */
    long x_1 = x;
    long y_1 = y;

    long bin_width  = x_1 + (w / PrimaryCCD.getBinX());
    long bin_height = y_1 + (h / PrimaryCCD.getBinY());

    if (bin_width > PrimaryCCD.getXRes() / PrimaryCCD.getBinX())
    {
        LOGF_INFO("Error: invalid width requested %d", w);
        return false;
    }
    else if (bin_height > PrimaryCCD.getYRes() / PrimaryCCD.getBinY())
    {
        LOGF_INFO("Error: invalid height request %d", h);
        return false;
    }

    IS_RECT rectAOI;

    rectAOI.s32X     = x;
    rectAOI.s32Y     = y;
    rectAOI.s32Width = w;
    rectAOI.s32Height = h;

    if(setAOI(rectAOI))
    {
        LOGF_INFO("Setting AOL (%d,%d : %d x %d) successfull", x,y,w,h);
    }

     
    /**********************************************************
    *
    *
    *
    *  IMPORRANT: Put here your CCD Frame dimension call
    *  The values calculated above are BINNED width and height
    *  which is what most CCD APIs require, but in case your
    *  CCD API implementation is different, don't forget to change
    *  the above calculations.
    *  If there is an error, report it back to client
    *  e.g.
    *  LOG_INFO( "Error, unable to set frame to ...");
    *  return false;
    *
    *
    **********************************************************/
    PrimaryCCD.setFrame(x_1, y_1, w, h);

    int nbuf;
    nbuf = (bin_width * bin_height * PrimaryCCD.getBPP() / 8); //  this is pixel count
    nbuf += 512;  
    LOGF_DEBUG("New frame buffer size : %d bytes.", nbuf);

    return true; 
 
}

///////////////////////////////////////////////////////////////////////////////////////
/// Update Camera Binning
///////////////////////////////////////////////////////////////////////////////////////
bool IDSCCD::UpdateCCDBin(int binx, int biny)
{
    /**********************************************************
    *
    *
    *
    *  IMPORRANT: Put here your CCD Binning call
    *  If there is an error, report it back to client
    *  e.g.
    *  LOG_INFO( "Error, unable to set binning to ...");
    *  return false;
    *
    *
    **********************************************************/

    PrimaryCCD.setBin(binx, biny);

    return UpdateCCDFrame(PrimaryCCD.getSubX(), PrimaryCCD.getSubY(), PrimaryCCD.getSubW(), PrimaryCCD.getSubH());
}

/*
IPState IDSCCD::GuideNorth(uint32_t ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

IPState IDSCCD::GuideSouth(uint32_t ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

IPState IDSCCD::GuideEast(uint32_t ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

IPState IDSCCD::GuideWest(uint32_t ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

*/