//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    capture_hikmvcam.cpp
  \brief   C++ Implementation: CaptureHikMvCam
  \author  Mark Huang, (C) 2022
*/
//========================================================================
#include "capture_hikmvcam.h"
#include <sys/time.h>

CaptureHikMvCam::CaptureHikMvCam(VarList * _settings, int default_camera_id, QObject * parent) : QObject(parent), CaptureInterface(_settings),m_pcMyCamera(nullptr){
    cam_id = (unsigned int) default_camera_id;
    is_capturing = false;
    mutex.lock();
    settings->addChild(capture_settings = new VarList("Capture Settings"));
    settings->addChild(camera_params = new VarList("Camera Parameters"));
    //=======================CAPTURE SETTINGS==========================
    v_capture_device = new VarInt("Camera ID", cam_id, 0, 100);
    {
        // ch:枚举子网内所有设备 | en:Enumerate all devices within subnet
        int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
        if (MV_OK != nRet){
            fprintf(stderr, "EnumDevices fail! nRet [%x]\n", nRet);
            return;
        }
    }
    capture_settings->addChild(v_capture_device);
    v_expose_auto = new VarStringEnum("Auto Exposure", toString(AutoEnum::manual_));
    v_expose_auto->addItem(toString(AutoEnum::manual_));
    v_expose_auto->addItem(toString(AutoEnum::once_));
    v_expose_auto->addItem(toString(AutoEnum::continuous_));
    v_exposure = new VarDouble("Expose [us]",8000,10,100000);
    v_gain_auto = new VarStringEnum("Auto Gain", toString(AutoEnum::manual_));
    v_gain_auto->addItem(toString(AutoEnum::manual_));
    v_gain_auto->addItem(toString(AutoEnum::once_));
    v_gain_auto->addItem(toString(AutoEnum::continuous_));
    v_gain = new VarDouble("Gain [dB]",0,0,48);
    v_frame_rate = new VarDouble("Frame Rate [fps]",74,0,100);
    camera_params->addChild(v_expose_auto);
    camera_params->addChild(v_exposure);
    camera_params->addChild(v_gain_auto);
    camera_params->addChild(v_gain);
    camera_params->addChild(v_frame_rate);
    mvcConnect(camera_params);
    mutex.unlock();
}
CaptureHikMvCam::~CaptureHikMvCam(){
    // stopCapture();
    capture_settings->deleteAllChildren();
    camera_params->deleteAllChildren();
}
void CaptureHikMvCam::mvcConnect(VarList * group){
    for (auto &i : group->getChildren()) {
        connect(i,SIGNAL(wasEdited(VarType *)),group,SLOT(mvcEditCompleted()));
    }
    connect(group,SIGNAL(wasEdited(VarType *)),this,SLOT(changed(VarType *)));
}
void CaptureHikMvCam::changed(VarType * group) {
    if (group->getType()==VARTYPE_ID_LIST) {
        writeParameterValues( (VarList *)group );
        readParameterValues( (VarList *)group );
    }
}
void CaptureHikMvCam::readAllParameterValues(){
    readParameterValues(camera_params);
}
void CaptureHikMvCam::writeAllParameterValues(){
    writeParameterValues(camera_params);
}
void CaptureHikMvCam::readParameterValues(VarList * item)
{
    if(item != camera_params || m_pcMyCamera == nullptr)
        return;
    double t_expose,t_gain;
    MVCC_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
    int nRet = m_pcMyCamera->GetFloatValue("ExposureTime", &stFloatValue);
    if (MV_OK != nRet){
        fprintf(stderr, "Get ExposureTime fail! nRet [%x]\n", nRet);
    }else{
        t_expose = stFloatValue.fCurValue;
    }
    nRet = m_pcMyCamera->GetFloatValue("Gain", &stFloatValue);
    if (MV_OK != nRet){
        fprintf(stderr, "Get Gain fail! nRet [%x]\n", nRet);
    }else{
        t_gain = stFloatValue.fCurValue;
    }
    mutex.lock();
    v_exposure->setDouble(t_expose);
    v_gain->setDouble(t_gain);
    mutex.unlock();
}

void CaptureHikMvCam::writeParameterValues(VarList * item)
{
    if(item != camera_params || m_pcMyCamera == nullptr)
        return;

    double t_expose,t_gain,t_frame_rate;
    mutex.lock();
    t_expose = v_exposure->getDouble();
    t_gain = v_gain->getDouble();
    t_frame_rate = v_frame_rate->getDouble();
    mutex.unlock();
    int nRet = m_pcMyCamera->SetFloatValue("ExposureTime", t_expose);
    if (MV_OK != nRet){
        fprintf(stderr, "Set ExposureTime fail! nRet [%x]", nRet);
    }
    nRet = m_pcMyCamera->SetFloatValue("Gain", t_gain);
    if (MV_OK != nRet){
        fprintf(stderr, "Set Gain fail! nRet [%x]", nRet);
    }
    nRet = m_pcMyCamera->SetFloatValue("AcquisitionFrameRate", t_frame_rate);
    if (MV_OK != nRet){
        fprintf(stderr, "Set AcquisitionFrameRate fail! nRet [%x]", nRet);
    }
}
string CaptureHikMvCam::getCaptureMethodName() const {
    return "HikMvCam";
}
bool CaptureHikMvCam::startCapture(){ 
    printf("startCapture\n");
    cam_id = v_capture_device->getInt();
    if ((cam_id < 0) || (cam_id > m_stDevList.nDeviceNum)){
        fprintf(stderr, "Index out of range!\n");
        return false;
    }
    if(nullptr == m_stDevList.pDeviceInfo[cam_id]){
        fprintf(stderr, "Selected device is null!\n");
        return false;
    }
    if(m_pcMyCamera == nullptr){
        m_pcMyCamera = new CMvCamera;
        if (NULL == m_pcMyCamera){
            fprintf(stderr, "create CMvCamera fail!\n");
            return false;
        }
    }
    int nRet = m_pcMyCamera->Open(m_stDevList.pDeviceInfo[cam_id]);
    if (MV_OK != nRet){
        delete m_pcMyCamera;
        m_pcMyCamera = NULL;
        fprintf(stderr, "Open fail! nRet [%x]\n", nRet);
        return false;
    }
    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (m_stDevList.pDeviceInfo[cam_id]->nTLayerType == MV_GIGE_DEVICE){
        unsigned int nPacketSize = 0;
        nRet = m_pcMyCamera->GetOptimalPacketSize(&nPacketSize);
        if (nRet == MV_OK){
            nRet = m_pcMyCamera->SetIntValue("GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK){
                fprintf(stderr,"Warning: Set Packet Size fail!", nRet);
            }
        }else{
            fprintf(stderr,"Warning: Get Packet Size fail!", nRet);
        }
    }
    m_pcMyCamera->SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    m_pcMyCamera->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
    writeAllParameterValues();
    readAllParameterValues();

    nRet = m_pcMyCamera->StartGrabbing();
    if (MV_OK != nRet){
        fprintf(stderr, "Start Grabbing fail! nRet [%x]\n", nRet);
        return false;
    }
    is_capturing = true;
    printf("startCapture success\n");
    return true;
}
bool CaptureHikMvCam::stopCapture(){
    if(m_pcMyCamera == nullptr)
        return false;
    if(isCapturing()){
        int nRet = m_pcMyCamera->StopGrabbing();
        if (MV_OK != nRet){
            fprintf(stderr, "Stop Grabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        is_capturing = false;
        m_pcMyCamera->Close();
        delete m_pcMyCamera;
        m_pcMyCamera = NULL;
    }
    return true;
}
RawImage CaptureHikMvCam::getFrame(){
    RawImage result;
    int nRet = m_pcMyCamera->GetImageBuffer(&m_stFrame, 1000);
    if (MV_OK == nRet){
        result.setColorFormat(COLOR_RAW8);
        result.setWidth(m_stFrame.stFrameInfo.nWidth);
        result.setHeight(m_stFrame.stFrameInfo.nHeight);
        result.setData(m_stFrame.pBufAddr);
        timeval tv{};
        gettimeofday(&tv, nullptr);
        result.setTime((double)tv.tv_sec + (double)tv.tv_usec * (1.0E-6));
    }
    return result;
}
void CaptureHikMvCam::releaseFrame(){
    if(NULL != m_stFrame.pBufAddr){
        m_pcMyCamera->FreeImageBuffer(&m_stFrame);
    }
}
bool CaptureHikMvCam::resetBus(){
    mutex.lock();
    mutex.unlock();
    return true;
}
bool CaptureHikMvCam::copyAndConvertFrame(const RawImage & src, RawImage & target){
    mutex.lock();
    if (src.getTime() == 0) {
        mutex.unlock();
        return false;
    }
    target.setTime(src.getTime());
    target = src;
    mutex.unlock();
    return true;
}


/*!
  \class CMvCamera
  \brief interfaces for HikMvCamera
  \author  Hikrobot MVS - Samples/QtCreator/BasicDemo/MvCamera.h
*/
CMvCamera::CMvCamera()
{
    m_hDevHandle = MV_NULL;
}

CMvCamera::~CMvCamera()
{
    if (m_hDevHandle)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle    = MV_NULL;
    }
}

// ch:获取SDK版本号 | en:Get SDK Version
int CMvCamera::GetSDKVersion()
{
    return MV_CC_GetSDKVersion();
}

// ch:枚举设备 | en:Enumerate Device
int CMvCamera::EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList)
{
    return MV_CC_EnumDevices(nTLayerType, pstDevList);
}

// ch:判断设备是否可达 | en:Is the device accessible
bool CMvCamera::IsDeviceAccessible(MV_CC_DEVICE_INFO* pstDevInfo, unsigned int nAccessMode)
{
    return MV_CC_IsDeviceAccessible(pstDevInfo, nAccessMode);
}

// ch:打开设备 | en:Open Device
int CMvCamera::Open(MV_CC_DEVICE_INFO* pstDeviceInfo)
{
    if (MV_NULL == pstDeviceInfo)
    {
        return MV_E_PARAMETER;
    }

    if (m_hDevHandle)
    {
        return MV_E_CALLORDER;
    }

    int nRet  = MV_CC_CreateHandle(&m_hDevHandle, pstDeviceInfo);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    nRet = MV_CC_OpenDevice(m_hDevHandle);
    if (MV_OK != nRet)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle = MV_NULL;
    }

    return nRet;
}

// ch:关闭设备 | en:Close Device
int CMvCamera::Close()
{
    if (MV_NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    MV_CC_CloseDevice(m_hDevHandle);

    int nRet = MV_CC_DestroyHandle(m_hDevHandle);
    m_hDevHandle = MV_NULL;

    return nRet;
}

// ch:判断相机是否处于连接状态 | en:Is The Device Connected
bool CMvCamera::IsDeviceConnected()
{
    return MV_CC_IsDeviceConnected(m_hDevHandle);
}

// ch:注册图像数据回调 | en:Register Image Data CallBack
int CMvCamera::RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser), void* pUser)
{
    return MV_CC_RegisterImageCallBackEx(m_hDevHandle, cbOutput, pUser);
}

// ch:开启抓图 | en:Start Grabbing
int CMvCamera::StartGrabbing()
{
    return MV_CC_StartGrabbing(m_hDevHandle);
}

// ch:停止抓图 | en:Stop Grabbing
int CMvCamera::StopGrabbing()
{
    return MV_CC_StopGrabbing(m_hDevHandle);
}

// ch:主动获取一帧图像数据 | en:Get one frame initiatively
int CMvCamera::GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec)
{
    return MV_CC_GetImageBuffer(m_hDevHandle, pFrame, nMsec);
}

// ch:释放图像缓存 | en:Free image buffer
int CMvCamera::FreeImageBuffer(MV_FRAME_OUT* pFrame)
{
    return MV_CC_FreeImageBuffer(m_hDevHandle, pFrame);
}

// ch:设置显示窗口句柄 | en:Set Display Window Handle
int CMvCamera::DisplayOneFrame(MV_DISPLAY_FRAME_INFO* pDisplayInfo)
{
    return MV_CC_DisplayOneFrame(m_hDevHandle, pDisplayInfo);
}

// ch:设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
int CMvCamera::SetImageNodeNum(unsigned int nNum)
{
    return MV_CC_SetImageNodeNum(m_hDevHandle, nNum);
}

// ch:获取设备信息 | en:Get device information
int CMvCamera::GetDeviceInfo(MV_CC_DEVICE_INFO* pstDevInfo)
{
    return MV_CC_GetDeviceInfo(m_hDevHandle, pstDevInfo);
}

// ch:获取GEV相机的统计信息 | en:Get detect info of GEV camera
int CMvCamera::GetGevAllMatchInfo(MV_MATCH_INFO_NET_DETECT* pMatchInfoNetDetect)
{
    if (MV_NULL == pMatchInfoNetDetect)
    {
        return MV_E_PARAMETER;
    }

    MV_CC_DEVICE_INFO stDevInfo = {0};
    GetDeviceInfo(&stDevInfo);
    if (stDevInfo.nTLayerType != MV_GIGE_DEVICE)
    {
        return MV_E_SUPPORT;
    }

    MV_ALL_MATCH_INFO struMatchInfo = {0};

    struMatchInfo.nType = MV_MATCH_TYPE_NET_DETECT;
    struMatchInfo.pInfo = pMatchInfoNetDetect;
    struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_NET_DETECT);
    memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_NET_DETECT));

    return MV_CC_GetAllMatchInfo(m_hDevHandle, &struMatchInfo);
}

// ch:获取U3V相机的统计信息 | en:Get detect info of U3V camera
int CMvCamera::GetU3VAllMatchInfo(MV_MATCH_INFO_USB_DETECT* pMatchInfoUSBDetect)
{
    if (MV_NULL == pMatchInfoUSBDetect)
    {
        return MV_E_PARAMETER;
    }

    MV_CC_DEVICE_INFO stDevInfo = {0};
    GetDeviceInfo(&stDevInfo);
    if (stDevInfo.nTLayerType != MV_USB_DEVICE)
    {
        return MV_E_SUPPORT;
    }

    MV_ALL_MATCH_INFO struMatchInfo = {0};

    struMatchInfo.nType = MV_MATCH_TYPE_USB_DETECT;
    struMatchInfo.pInfo = pMatchInfoUSBDetect;
    struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_USB_DETECT);
    memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_USB_DETECT));
    
    return MV_CC_GetAllMatchInfo(m_hDevHandle, &struMatchInfo);
}

// ch:获取和设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Int type parameters, such as Width and Height, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::GetIntValue(IN const char* strKey, OUT MVCC_INTVALUE_EX *pIntValue)
{
    return MV_CC_GetIntValueEx(m_hDevHandle, strKey, pIntValue);
}

int CMvCamera::SetIntValue(IN const char* strKey, IN int64_t nValue)
{
    return MV_CC_SetIntValueEx(m_hDevHandle, strKey, nValue);
}

// ch:获取和设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Enum type parameters, such as PixelFormat, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::GetEnumValue(IN const char* strKey, OUT MVCC_ENUMVALUE *pEnumValue)
{
    return MV_CC_GetEnumValue(m_hDevHandle, strKey, pEnumValue);
}

int CMvCamera::SetEnumValue(IN const char* strKey, IN unsigned int nValue)
{
    return MV_CC_SetEnumValue(m_hDevHandle, strKey, nValue);
}

int CMvCamera::SetEnumValueByString(IN const char* strKey, IN const char* sValue)
{
    return MV_CC_SetEnumValueByString(m_hDevHandle, strKey, sValue);
}

// ch:获取和设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Float type parameters, such as ExposureTime and Gain, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::GetFloatValue(IN const char* strKey, OUT MVCC_FLOATVALUE *pFloatValue)
{
    return MV_CC_GetFloatValue(m_hDevHandle, strKey, pFloatValue);
}

int CMvCamera::SetFloatValue(IN const char* strKey, IN float fValue)
{
    return MV_CC_SetFloatValue(m_hDevHandle, strKey, fValue);
}

// ch:获取和设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Bool type parameters, such as ReverseX, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::GetBoolValue(IN const char* strKey, OUT bool *pbValue)
{
    return MV_CC_GetBoolValue(m_hDevHandle, strKey, pbValue);
}

int CMvCamera::SetBoolValue(IN const char* strKey, IN bool bValue)
{
    return MV_CC_SetBoolValue(m_hDevHandle, strKey, bValue);
}

// ch:获取和设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
// en:Get String type parameters, such as DeviceUserID, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::GetStringValue(IN const char* strKey, MVCC_STRINGVALUE *pStringValue)
{
    return MV_CC_GetStringValue(m_hDevHandle, strKey, pStringValue);
}

int CMvCamera::SetStringValue(IN const char* strKey, IN const char* strValue)
{
    return MV_CC_SetStringValue(m_hDevHandle, strKey, strValue);
}

// ch:执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Execute Command once, such as UserSetSave, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int CMvCamera::CommandExecute(IN const char* strKey)
{
    return MV_CC_SetCommandValue(m_hDevHandle, strKey);
}

// ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
int CMvCamera::GetOptimalPacketSize(unsigned int* pOptimalPacketSize)
{
    if (MV_NULL == pOptimalPacketSize)
    {
        return MV_E_PARAMETER;
    }

    int nRet = MV_CC_GetOptimalPacketSize(m_hDevHandle);
    if (nRet < MV_OK)
    {
        return nRet;
    }

    *pOptimalPacketSize = (unsigned int)nRet;

    return MV_OK;
}

// ch:注册消息异常回调 | en:Register Message Exception CallBack
int CMvCamera::RegisterExceptionCallBack(void(__stdcall* cbException)(unsigned int nMsgType, void* pUser),void* pUser)
{
    return MV_CC_RegisterExceptionCallBack(m_hDevHandle, cbException, pUser);
}

// ch:注册单个事件回调 | en:Register Event CallBack
int CMvCamera::RegisterEventCallBack(const char* pEventName, void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser), void* pUser)
{
    return MV_CC_RegisterEventCallBackEx(m_hDevHandle, pEventName, cbEvent, pUser);
}

// ch:强制IP | en:Force IP
int CMvCamera::ForceIp(unsigned int nIP, unsigned int nSubNetMask, unsigned int nDefaultGateWay)
{
    return MV_GIGE_ForceIpEx(m_hDevHandle, nIP, nSubNetMask, nDefaultGateWay);
}

// ch:配置IP方式 | en:IP configuration method
int CMvCamera::SetIpConfig(unsigned int nType)
{
    return MV_GIGE_SetIpConfig(m_hDevHandle, nType);
}

// ch:设置网络传输模式 | en:Set Net Transfer Mode
int CMvCamera::SetNetTransMode(unsigned int nType)
{
    return MV_GIGE_SetNetTransMode(m_hDevHandle, nType);
}

// ch:像素格式转换 | en:Pixel format conversion
int CMvCamera::ConvertPixelType(MV_CC_PIXEL_CONVERT_PARAM* pstCvtParam)
{
    return MV_CC_ConvertPixelType(m_hDevHandle, pstCvtParam);
}

// ch:保存图片 | en:save image
int CMvCamera::SaveImage(MV_SAVE_IMAGE_PARAM_EX* pstParam)
{
    return MV_CC_SaveImageEx2(m_hDevHandle, pstParam);
}

