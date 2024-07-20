/*========================================================*
* Copyright (c) 2020 Institute of Cyber-Systems & Control *
*                   Zhejiang University                   *
*                   All rights reserved                   *
*                                                         *
*               Author: Dr. Yongsheng Zhao                *
*                Email: zhaoyongsheng@zju.edu.cn          *
*        Creating Date: Thu Mar 12 2020                   *
*         Descriptions: Daheng camera capture             *
*=========================================================*/

#include "capture_daheng.h"

#include <iostream>
#include <vector>
#include <string>
#include <QThread>
#include <opencv2/opencv.hpp>

bool DahengInitManager::is_registered = false;
int DahengInitManager::register_count = 0;
GX_STATUS DahengInitManager::emStatus = GX_STATUS_SUCCESS;

void DahengInitManager::register_capture()
{
	//std::cout << __FUNCTION__ << " starts..." << std::endl;
	if (is_registered == false)
	{
		//Initialize libary
		emStatus = GXInitLib();
		emStatus = GXInitLib();
		if (emStatus != GX_STATUS_SUCCESS)
		{
			printf("Daheng camera context initialization failed.\n");
			return;
		}
		printf("Daheng camera context initialization success.\n");
		is_registered = true;
		register_count++;
	}
	else
	{
		register_count++;
	}
	//std::cout << __FUNCTION__ << " end..." << std::endl;
}

void DahengInitManager::unregister_capture()
{
	if (register_count == 0 && is_registered == true)
	{
		emStatus = GXCloseLib();
		if (emStatus != GX_STATUS_SUCCESS)
		{
			printf("Daheng camera context close failed.\n");
			return;
		}
		printf("Daheng camera context close success.\n");
		is_registered = false;
		register_count = 0;
	}
	else
	{
		if (is_registered == true)
		{
			register_count--;
		}
		else
		{
			register_count = 0;
		}
	}
}

CaptureDaheng::CaptureDaheng(VarList *_settings, unsigned int _camera_id, QObject *_parent) : QObject(_parent), CaptureInterface(_settings)
{
	//std::cout << __FUNCTION__ << " starts..." << std::endl;
	is_capturing = false;
	// is_param_recoved = false;
	camera = NULL;
	camera_id = _camera_id;
	color_filter = GX_COLOR_FILTER_NONE;
	rgb_image_buf = NULL;
	raw8_image_buf = NULL;
	ignore_capture_failure = false;
	//camera.PixelFormat.SetValue(Daheng_GigECamera::PixelFormat_YUV422Packed, true);
	last_buf = NULL;

	settings->addChild(vars = new VarList("Capture Settings"));
	settings->removeFlags(VARTYPE_FLAG_HIDE_CHILDREN);
	vars->removeFlags(VARTYPE_FLAG_HIDE_CHILDREN);
	v_color_mode = new VarStringEnum("color mode", Colors::colorFormatToString(COLOR_RGB8));
	v_color_mode->addItem(Colors::colorFormatToString(COLOR_YUV422_UYVY));
	v_color_mode->addItem(Colors::colorFormatToString(COLOR_RGB8));
	vars->addChild(v_color_mode);

	vars->addChild(v_camera_id = new VarInt("Camera ID", (int)camera_id, 0, 3));

	v_auto_balance = new VarBool("auto balance", false);
	vars->addChild(v_auto_balance);

	v_balance_ratio_red = new VarDouble("Balance Ratio Red", 1.5, 1.0, 7.998);
	vars->addChild(v_balance_ratio_red);

	v_balance_ratio_green = new VarDouble("Balance Ratio Green", 1.5, 1.0, 7.998);
	vars->addChild(v_balance_ratio_green);

	v_balance_ratio_blue = new VarDouble("Balance Ratio Blue", 1.5, 1.0, 7.998);
	vars->addChild(v_balance_ratio_blue);

	v_auto_gain = new VarBool("auto gain", true);
	vars->addChild(v_auto_gain);

	v_gain = new VarDouble("gain", 5, 0, 16);
	vars->addChild(v_gain);

	// v_gamma_enable = new VarBool("enable gamma correction", false);
	// vars->addChild(v_gamma_enable);
	// v_gamma = new VarDouble("gamma", 0.5, 0, 1);
	// vars->addChild(v_gamma);
	// v_auto_black = new VarBool("auto black level", false);
	// vars->addChild(v_auto_black);
	// v_black_level = new VarDouble("black level", 64, 0, 1000);
	// vars->addChild(v_black_level);

	v_auto_exposure = new VarBool("auto exposure", false);
	vars->addChild(v_auto_exposure);

	v_exposure_time = new VarDouble("exposure time (μs)", 20000, 20, 1000000);
	vars->addChild(v_exposure_time);

	mvc_connect(settings);
	mvc_connect(vars);
}

CaptureDaheng::~CaptureDaheng()
{
	vars->deleteAllChildren();
}

bool CaptureDaheng::_buildCamera()
{
	//std::cout << __FUNCTION__ << " Starts.." << std::endl;
	//Initialize libary
	DahengInitManager::register_capture();
	//Get device enumerated number
	unsigned int device_nums;
	DahengInitManager::emStatus = GXUpdateDeviceList(&device_nums, 1000);
	printf("Enumerate device nums: %d\n", device_nums);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		DahengInitManager::unregister_capture();
		printf("Enumerate device nums failed.\n");
		return false;
	}
	printf("Current camera id: %d\n", camera_id);
	if (device_nums > camera_id)
	{
		DahengInitManager::emStatus = GXOpenDeviceByIndex(camera_id + 1, &camera); // Index starts from 1
		if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
		{
			// delete camera;
			DahengInitManager::unregister_capture();
			//std::cout << __FUNCTION__ << " end..." << std::endl;
			return false;
		}
		printf("Camera %d opened.\n", camera_id);
		if (_setCamera())
		{
			_mallocImageBuf();
		}
		printf("Done!\n");
		is_capturing = true;
		//std::cout << __FUNCTION__ << " end..." << std::endl;
		return true;
	}
	//std::cout << __FUNCTION__ << " end..." << std::endl;

	return false;
}

bool CaptureDaheng::_mallocImageBuf()
{
	DahengInitManager::emStatus = GXGetInt(camera, GX_INT_PAYLOAD_SIZE, &payload_size);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}
	// free before malloc
	_freeImageBuf();
	rgb_image_buf = new unsigned char[payload_size * 3];
	raw8_image_buf = new unsigned char[payload_size];
	return true;
}

bool CaptureDaheng::_freeImageBuf()
{
	//Release resources
	if (rgb_image_buf != NULL)
	{
		delete[] rgb_image_buf;
		rgb_image_buf = NULL;
	}
	if (raw8_image_buf != NULL)
	{
		delete[] raw8_image_buf;
		raw8_image_buf = NULL;
	}

	return true;
}

bool CaptureDaheng::_setCamera()
{
	//Set acquisition mode
	DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}

	//Set trigger mode
	DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}
	DahengInitManager::emStatus = GXSetAcqusitionBufferNumber(camera, ACQ_BUFFER_NUM);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}

	bool bStreamTransferSize = false;
	DahengInitManager::emStatus = GXIsImplemented(camera, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}

	if (bStreamTransferSize)
	{
		//Set size of data transfer block
		DahengInitManager::emStatus = GXSetInt(camera, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
		if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
		{
			GXCloseDevice(camera);
			DahengInitManager::unregister_capture();
			return false;
		}
	}

	bool bStreamTransferNumberUrb = false;
	DahengInitManager::emStatus = GXIsImplemented(camera, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);

	if (bStreamTransferNumberUrb)
	{
		//Set qty. of data transfer block
		DahengInitManager::emStatus = GXSetInt(camera, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
		if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
		{
			GXCloseDevice(camera);
			DahengInitManager::unregister_capture();
			return false;
		}
	}

	return true;

	// //Set Balance White Mode : Continuous
	// emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
	// GX_VERIFY_EXIT(emStatus);
}

bool CaptureDaheng::startCapture(){
	{
		std::unique_lock<std::mutex> lock(_mutex);

		try
		{
			if (camera == NULL)
			{
				if (!_buildCamera() || !_getColorFilter())
				{
					// Did not make a camera!
					// delete camera;
					camera = NULL;
					return false;
				}
			}
			//Device start acquisition
			// //std::cout << __FUNCTION__ << " Camera Open.." << std::endl;
			DahengInitManager::emStatus = GXStreamOn(camera);
			// //std::cout << __FUNCTION__ << " Camera Stream On.." << std::endl;
			if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
			{
				//Release the memory allocated
				_freeImageBuf();
				GXCloseDevice(camera);
				DahengInitManager::unregister_capture();
				return false;
			}
		}
		catch (...)
		{
			printf("Uncaught exception at line 241\n");
			return false;
		}
	}
	writeParameterValues(this->settings);
	std::cout << __FUNCTION__ << " end..." << std::endl;
	return true;
}

bool CaptureDaheng::_stopCapture()
{
	if (is_capturing)
	{
		//Device stop acquisition
		DahengInitManager::emStatus = GXStreamOff(camera);
		DahengInitManager::emStatus = GXCloseDevice(camera);
		_freeImageBuf();
		if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
		{
			DahengInitManager::unregister_capture();
			return false;
		}
		camera = NULL;
		is_capturing = false;
		return true;
	}
	return false;
}

bool CaptureDaheng::stopCapture()
{
	std::unique_lock<std::mutex> lock(_mutex);
	bool stopped;
	try
	{
		stopped = _stopCapture();
		if (stopped)
		{
			// delete camera;
			// camera = 0;
			DahengInitManager::unregister_capture();
		}
	}
	catch (...)
	{
		printf("Uncaught exception at line 288\n");
		throw;
	}
	return stopped;
}

void CaptureDaheng::releaseFrame()
{
	std::unique_lock<std::mutex> lock(_mutex);
	try
	{
		// raw8_image_buf, rgb_image_buf are global buffer, do not need to release per frame
		// if (last_buf)
		// {
		// 	free(last_buf);
		// 	last_buf = NULL;
		// }
	}
	catch (...)
	{
		throw;
	}
}

RawImage CaptureDaheng::getFrame()
{
	std::unique_lock<std::mutex> lock(_mutex);
	RawImage img;
	img.setWidth(0);
	img.setHeight(0);
	img.setColorFormat(COLOR_RGB8);
	try
	{
		timeval tv;
		gettimeofday(&tv, 0);
		img.setTime((double)tv.tv_sec + (tv.tv_usec / 1000000.0));
		try
		{
			DahengInitManager::emStatus = GXDQBuf(camera, &frame_buffer, 1000);
			if (DahengInitManager::emStatus != GX_STATUS_SUCCESS){
				printf("<Abnormal Acquisition in get data: Exception code: %d>\n", frame_buffer->nStatus);
				return img;
			}
			if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS){
				printf("<Abnormal Acquisition in frame_buffer: Exception code: %d>\n", frame_buffer->nStatus);
				return img;
			}
			// Acquisition success, then format converter
			if (!_convertFormat(frame_buffer)){
				DahengInitManager::emStatus = GXQBuf(camera, frame_buffer);
				printf("<Abnormal Acquisition in convertFormat: Exception code: %d>\n", frame_buffer->nStatus);
				return img;
			}
			img.setWidth(frame_buffer->nWidth);
			img.setHeight(frame_buffer->nHeight);
			img.setColorFormat(COLOR_RGB8);
			img.setData(rgb_image_buf);

			// release frame_buffer for continuous grab
			DahengInitManager::emStatus = GXQBuf(camera, frame_buffer);
		}
		catch (...)
		{
			fprintf(stderr, "Timeout expired in CaptureDaheng::getFrame\n");
			return img;
		}
	}
	catch (...)
	{
		// Make sure the mutex is unlocked before propagating
		printf("Uncaught exception!\n");
		throw;
	}
	return img;
}

bool CaptureDaheng::_getColorFilter()
{
	//Get the type of Bayer conversion. whether is a color camera.
	bool is_colorful;
	DahengInitManager::emStatus = GXIsImplemented(camera, GX_ENUM_PIXEL_COLOR_FILTER, &is_colorful);
	if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
	{
		GXCloseDevice(camera);
		DahengInitManager::unregister_capture();
		return false;
	}

	//This app only support color cameras
	if (!is_colorful)
	{
		printf("<This app only support color cameras! App Exit!>\n");
		GXCloseDevice(camera);
		camera = NULL;
		DahengInitManager::unregister_capture();
		return false;
	}
	else
	{
		DahengInitManager::emStatus = GXGetEnum(camera, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter);
		if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
		{
			GXCloseDevice(camera);
			DahengInitManager::unregister_capture();
			return false;
		}
	}
	return true;
}

bool CaptureDaheng::_convertFormat(PGX_FRAME_BUFFER frame_buf)
{
	// GX_STATUS emStatus = GX_STATUS_SUCCESS;
	VxInt32 emDXStatus = DX_OK;

	// Convert RAW8 or RAW16 image to RGB24 image
	switch (frame_buf->nPixelFormat)
	{
	case GX_PIXEL_FORMAT_BAYER_GR8:
	case GX_PIXEL_FORMAT_BAYER_RG8:
	case GX_PIXEL_FORMAT_BAYER_GB8:
	case GX_PIXEL_FORMAT_BAYER_BG8:
	{
		// Convert to the RGB image
		emDXStatus = DxRaw8toRGB24((unsigned char *)frame_buf->pImgBuf, rgb_image_buf, frame_buf->nWidth, frame_buf->nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(color_filter), false);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
			return false;
		}
		break;
	}
	case GX_PIXEL_FORMAT_BAYER_GR10:
	case GX_PIXEL_FORMAT_BAYER_RG10:
	case GX_PIXEL_FORMAT_BAYER_GB10:
	case GX_PIXEL_FORMAT_BAYER_BG10:
	case GX_PIXEL_FORMAT_BAYER_GR12:
	case GX_PIXEL_FORMAT_BAYER_RG12:
	case GX_PIXEL_FORMAT_BAYER_GB12:
	case GX_PIXEL_FORMAT_BAYER_BG12:
	{
		// Convert to the Raw8 image
		emDXStatus = DxRaw16toRaw8((unsigned char *)frame_buf->pImgBuf, raw8_image_buf, frame_buf->nWidth, frame_buf->nHeight, DX_BIT_2_9);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
			return false;
		}
		// Convert to the RGB24 image
		emDXStatus = DxRaw8toRGB24((unsigned char *)raw8_image_buf, rgb_image_buf, frame_buf->nWidth, frame_buf->nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(color_filter), false);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
			return false;
		}
		break;
	}
	default:
	{
		printf("Error : PixelFormat of this camera is not supported\n");
		return false;
	}
	}
	return true;
}
string CaptureDaheng::getCaptureMethodName() const
{
	return "Daheng";
}

bool CaptureDaheng::copyAndConvertFrame(const RawImage &src, RawImage &target)
{
	std::unique_lock<std::mutex> lock(_mutex);
	try
	{
		target.ensure_allocation(src.getColorFormat(), src.getWidth(), src.getHeight());
		target.setTime(src.getTime());
		memcpy(target.getData(), src.getData(), src.getNumBytes());
	}
	catch (...)
	{
		throw;
	}
	return true;
}

void CaptureDaheng::readAllParameterValues()
{
	std::unique_lock<std::mutex> lock(_mutex);
	try
	{
		if (camera == NULL)
		{
			return;
		}

		// Check auto balance
		int64_t nValue = 0;
		DahengInitManager::emStatus = GXGetEnum(camera, GX_ENUM_BALANCE_WHITE_AUTO, &nValue);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_auto_balance->setBool(nValue == GX_BALANCE_WHITE_AUTO_CONTINUOUS);
		}
		else
		{
			printf("Check balance mode failed.\n");
			v_auto_balance->setBool(false);
		}

		// Get white balance
		double balance_value;
		DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
		DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_BALANCE_RATIO, &balance_value);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_balance_ratio_red->setDouble(balance_value);
		}
		else
		{
			printf("Get red balance value failed.\n");
		}
		DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
		DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_BALANCE_RATIO, &balance_value);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_balance_ratio_green->setDouble(balance_value);
		}
		else
		{
			printf("Get green balance value failed.\n");
		}
		DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
		DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_BALANCE_RATIO, &balance_value);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_balance_ratio_blue->setDouble(balance_value);
		}
		else
		{
			printf("Get blue balance value failed.\n");
		}

		// check whether auto gain
		DahengInitManager::emStatus = GXGetEnum(camera, GX_ENUM_GAIN_AUTO, &nValue);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_auto_gain->setBool(nValue == GX_GAIN_AUTO_CONTINUOUS);
		}
		else
		{
			printf("Check gain mode failed.\n");
			v_auto_gain->setBool(false);
		}

		// get gain value
		// Selects the gain channel type.
		DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
		// Gets the gain value.
		double gain_value;
		DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_GAIN, &gain_value);
		// // Sets the gain to the minimum.
		// status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMin);
		// // Sets the gain to the maximum.
		// status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMax);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_gain->setDouble(gain_value);
		}
		else
		{
			printf("Get gain value failed.\n");
		}
		// DahengInitManager::emStatus = GXSetBool(camera, GX_BOOL_GAMMA_ENABLE, true);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	printf("Set gamma enable status succeed.\n");
		// }
		// else
		// {
		// 	printf("Set gamma enable status failed.\n");
		// }
		// // Determines whether the current camera supports Gamma capture .
		// bool is_gamma_implemented;
		// DahengInitManager::emStatus = GXIsImplemented(camera, GX_FLOAT_GAMMA_PARAM,
		// &is_gamma_implemented);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	//std::cout << "Camera supports gamma: " << is_gamma_implemented << std::endl;
		// }
		// else
		// {
		// 	//std::cout << "Check whether camera supports gamma failed." << std::endl;
		// }
		// // bool gamma_enable;
		// // DahengInitManager::emStatus = GXGetBool(camera, GX_BOOL_GAMMA_ENABLE, &gamma_enable);
		// // if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// // {
		// // 	v_gamma_enable->setBool(gamma_enable);
		// // }
		// // else
		// // {
		// // 	printf("Get gamma enable status failed.\n");
		// // 	v_gamma_enable->setBool(false);
		// // }
		// //Sets Gamma mode to user-defined mode.
		// DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_USER);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	printf("Set gamma user mode failed.\n");
		// }
		// else
		// {
		// 	printf("Set gamma user mode succeed.\n");
		// }

		// // Gets the Gamma adjustment parameter.
		// double gamma_value;
		// DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_GAMMA_PARAM, &gamma_value);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	v_gamma->setDouble(gamma_value);
		// }
		// else
		// {
		// 	printf("Get gamma value failed.\n");
		// }

		// //Gets to continuous automatic black level mode.
		// DahengInitManager::emStatus = GXGetEnum(camera, GX_ENUM_BLACKLEVEL_AUTO, &nValue);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	v_auto_black->setBool(nValue == GX_BLACKLEVEL_AUTO_CONTINUOUS);
		// }
		// else
		// {
		// 	printf("Check black level mode failed.\n");
		// 	v_auto_black->setBool(false);
		// }

		// //Selects the black level channel type.
		// DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_ALL);
		// // Gets the range of the black level.
		// double black_level;
		// DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_BLACKLEVEL, &black_level);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	v_black_level->setDouble(black_level);
		// }
		// else
		// {
		// 	printf("Get black level failed.\n");
		// }

		// Check exposure mode
		DahengInitManager::emStatus = GXGetEnum(camera, GX_ENUM_EXPOSURE_AUTO, &nValue);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_auto_exposure->setBool(nValue == GX_EXPOSURE_AUTO_CONTINUOUS);
		}
		else
		{
			printf("Check exposure mode failed.\n");
			v_auto_exposure->setBool(false);
		}

		// Get exposure time
		double exposure_time;
		DahengInitManager::emStatus = GXGetFloat(camera, GX_FLOAT_EXPOSURE_TIME, &exposure_time);
		if (DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		{
			v_exposure_time->setDouble(exposure_time);
		}
		else
		{
			printf("Get exposure time failed.\n");
		}
	}
	catch (...)
	{
		fprintf(stderr, "Exception reading parameter values: \n");
		throw;
	}
}

void CaptureDaheng::resetCamera(unsigned int new_id)
{
	// //std::cout << __FUNCTION__ << ", Starts...";
	bool restart = is_capturing;
	if (restart)
	{
		stopCapture();
	}
	camera_id = new_id;
	if (restart)
	{
		startCapture();
	}
	// //std::cout << __FUNCTION__ << ", Ends...";
}

// void CaptureDaheng::writeParameterValues()
// {
// 	MUTEX_LOCK;
// 	try
// 	{
//         if (camera != NULL)
// 		{
//             if(v_auto_balance->getBool())
// 			{
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set balance auto mode failed.\n");
// 				}
//             }
// 			else
// 			{
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
// 				DahengInitManager::emStatus=GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
// 				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, f_balance_ratio_red);
// 				v_balance_ratio_red->setDouble(f_balance_ratio_red);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set red balance value failed.\n");
// 				}
// 				DahengInitManager::emStatus=GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
// 				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, f_balance_ratio_green);
// 				v_balance_ratio_green->setDouble(f_balance_ratio_green);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set green balance value failed.\n");
// 				}
// 				DahengInitManager::emStatus=GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
// 				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, f_balance_ratio_blue);
// 				v_balance_ratio_blue->setDouble(f_balance_ratio_blue);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set blue balance value failed.\n");
// 				}
//             }
//             // camera->BalanceWhiteAuto.SetValue(Daheng_GigECamera::BalanceWhiteAuto_Once);
//
//             if(v_auto_gain->getBool())
// 			{
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set gain auto mode failed.\n");
// 				}
//             }
// 			else
// 			{
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
// 				// Set gain value
// 				// Selects the gain channel type.
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
// 				// Gets the gain value.
// 				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_GAIN, f_gain);
// 				v_gain->setDouble(f_gain);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set gain value failed.\n");
// 				}
//             }
//
//             if(v_auto_exposure->getBool())
// 			{
// 				// Set exposure mode
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set exposure auto mode on failed.\n");
// 				}
//             }
// 			else
// 			{
// 				// Set exposure time
// 				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set exposure auto mode off failed.\n");
// 				}
// 				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_EXPOSURE_TIME, f_exposure_time);
// 				v_exposure_time->setDouble(f_exposure_time);
// 				if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
// 				{
// 					printf("Set exposure time failed.\n");
// 				}
//             }
//         }
// 	}
// 	catch(...)
// 	{
// 		fprintf(stderr, "Error writing parameter values.\n");
// 		MUTEX_UNLOCK;
// 		throw;
// 	}
// 	// is_param_recoved = true;
// 	MUTEX_UNLOCK;
// }

void CaptureDaheng::writeParameterValues(VarList *vars)
{
	std::unique_lock<std::mutex> lock(_mutex);
	if (vars != this->settings)
	{
		//std::cout << __FUNCTION__ << "vars != this->settings" << std::endl;
		return;
	}
	// else
	// {
	// 	//std::cout << __FUNCTION__ << "vars == this->settings" << std::endl;
	// 	if(is_param_recoved == false)
	// 	{
	// 		// //std::cout << __FUNCTION__ << "vars == this->settings" << std::endl;
	// 		f_balance_ratio_red =  v_balance_ratio_red->getDouble();
	// 		f_balance_ratio_green = v_balance_ratio_green->getDouble();
	// 		f_balance_ratio_blue = v_balance_ratio_blue->getDouble();
	// 		f_gain = v_gain->getDouble();
	// 		f_exposure_time = v_exposure_time->getDouble();
	// 		//std::cout << f_balance_ratio_red << std::endl;
	// 		//std::cout << f_balance_ratio_green << std::endl;
	// 		//std::cout << f_balance_ratio_blue << std::endl;
	// 		//std::cout << f_gain << std::endl;
	// 		//std::cout << f_exposure_time << std::endl;
	// 	}
	// 	// return;
	// }
	// bool restart = is_capturing;
	//if (restart) {
	//	_stopCapture();
	//}
	try
	{
		camera_id = v_camera_id->get();

		// MUTEX_UNLOCK;
		// resetCamera(v_camera_id->get()); // locks itself
		// MUTEX_LOCK;
		//Device stop acquisition

		// DahengInitManager::emStatus = GXStreamOff(camera);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	printf("Camera stream off.\n");
		// }

		if (camera != NULL)
		{
			// //使 能 采 集 帧 率 调 节 模 式
			DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
													GX_ACQUISITION_FRAME_RATE_MODE_ON);
			// //设 置 采 集 帧 率,假 设 设 置 为 10.0, 用 户 按 照 实 际 需 求 设 置 此 值
			DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_ACQUISITION_FRAME_RATE, 70.0);

			if (v_auto_balance->getBool())
			{
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set balance auto mode failed.\n");
				}
			}
			else
			{
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, v_balance_ratio_red->getDouble());
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set red balance value failed.\n");
				}
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, v_balance_ratio_green->getDouble());
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set green balance value failed.\n");
				}
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BALANCE_RATIO, v_balance_ratio_blue->getDouble());
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set blue balance value failed.\n");
				}
			}
			// camera->BalanceWhiteAuto.SetValue(Daheng_GigECamera::BalanceWhiteAuto_Once);

			if (v_auto_gain->getBool())
			{
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set gain auto mode failed.\n");
				}
			}
			else
			{
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
				// Set gain value
				// Selects the gain channel type.
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
				// Gets the gain value.
				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_GAIN, v_gain->getDouble());
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set gain value failed.\n");
				}
			}

			// if(v_gamma_enable->getBool())
			// {
			// 	DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_GAMMA_PARAM, v_gamma->getDouble());
			//	if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
			//	{
			//		printf("Set gamma value failed.\n");
			//	}
			// }

			// if(v_auto_black->getBool())
			// {
			// Set black level auto mode
			// 	DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
			//	if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
			//	{
			//		printf("Set black level auto mode failed.\n");
			// 	}
			// }
			// else
			// {
			// 	// Set black level channel
			// 	DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_ALL);
			// 	// Set black level
			// 	DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_BLACKLEVEL, v_black_level->getDouble());
			// 	if(DahengInitManager::emStatus != GX_STATUS_SUCCESS)
			// 	{
			// 		printf("Set black level failed.\n");
			// 	}
			//          }

			if (v_auto_exposure->getBool())
			{
				// Set exposure mode
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set exposure auto mode on failed.\n");
				}
			}
			else
			{
				// Set exposure time
				DahengInitManager::emStatus = GXSetEnum(camera, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set exposure auto mode off failed.\n");
				}
				DahengInitManager::emStatus = GXSetFloat(camera, GX_FLOAT_EXPOSURE_TIME, v_exposure_time->getDouble());
				if (DahengInitManager::emStatus != GX_STATUS_SUCCESS)
				{
					printf("Set exposure time failed.\n");
				}
			}
		}
		// DahengInitManager::emStatus = GXStreamOn(camera);
		// if(DahengInitManager::emStatus == GX_STATUS_SUCCESS)
		// {
		// 	printf("Camera stream on.\n");
		// }
	}
	catch (...)
	{
		fprintf(stderr, "Error writing parameter values.\n");
		throw;
	}
}

void CaptureDaheng::mvc_connect(VarList *group)
{
	vector<VarType *> v = group->getChildren();
	for (unsigned int i = 0; i < v.size(); i++)
	{
		connect(v[i], SIGNAL(wasEdited(VarType *)), group, SLOT(mvcEditCompleted()));
	}
	connect(group, SIGNAL(wasEdited(VarType *)), this, SLOT(changed(VarType *)));
	// v[0]->mvcEditCompleted();
}

void CaptureDaheng::changed(VarType *group)
{
	if (group->getType() == VARTYPE_ID_LIST)
	{
		writeParameterValues(dynamic_cast<VarList *>(group));
	}
}
