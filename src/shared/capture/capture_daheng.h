/*========================================================*
* Copyright (c) 2020 Institute of Cyber-Systems & Control *
*                   Zhejiang University                   *
*                   All rights reserved                   *
*                                                         *
*               Author: Dr. Yongsheng Zhao                *
*                Email: zhaoyongsheng@zju.edu.cn          *
*        Personal Blog: www.zhaoyongsheng.com             *
*        Creating Date: Thu Mar 12 2020                   *
*         Descriptions: Daheng camera capture             *
*=========================================================*/

#ifndef CAPTURE_DAHENG_H
#define CAPTURE_DAHENG_H

#include "captureinterface.h"
#include <sys/time.h>
#include <GxIAPI.h>
#include <DxImageProc.h>
#include <mutex>
#include "VarTypes.h"

class DahengInitManager
{
public:
	static void register_capture();
	static void unregister_capture();
	//private:
	static bool is_registered;
	static GX_STATUS emStatus;
	static int register_count;
};

class CaptureDaheng : public QObject, public CaptureInterface
{
public:
	Q_OBJECT
public slots:
	void changed(VarType *group);

private:
	std::mutex _mutex;

public:
	CaptureDaheng(VarList *_settings = nullptr, unsigned int _camera_id = 0, QObject *parent = nullptr);
	void mvc_connect(VarList *group);
	~CaptureDaheng();

	bool startCapture();

	bool stopCapture();

	bool isCapturing() { return is_capturing; };

	RawImage getFrame();

	void releaseFrame();

	string getCaptureMethodName() const;

	bool copyAndConvertFrame(const RawImage &src, RawImage &target);

	void readAllParameterValues();

	void writeParameterValues(VarList *vars);
	
	bool setAcquisitionBufferNum();
	
	bool prepareForShowImg();

private:
	bool is_capturing;
	bool ignore_capture_failure;
	GX_DEV_HANDLE camera;			   // Device handle, Pointer
	PGX_FRAME_BUFFER* frame_buffers;   ///< Array of PGX_FRAME_BUFFER
	uint64_t cam_acquisition_buffer_num; ///< Acquisition buffer number
	uint32_t cam_frame_count;///< Acquisition frame count
	uint32_t cam_frame_num; /// use in GXDQAllBufs
	int64_t color_filter;			   // Color filter of device
	unsigned char *rgb_image_buf;	   // Memory for RAW8toRGB24
	unsigned char *raw8_image_buf;	   // Memory for RAW16toRAW8
	unsigned int camera_id;			   // Camera ID
	unsigned char *last_buf;

	VarList *vars;
	VarInt *v_camera_id;
	VarBool *v_auto_balance;
	VarDouble *v_balance_ratio_red;
	VarDouble *v_balance_ratio_green;
	VarDouble *v_balance_ratio_blue;
	VarBool *v_auto_gain;
	VarDouble *v_gain;
	// VarBool* v_gamma_enable;
	// VarDouble* v_gamma;
	// VarBool* v_auto_black;
	// VarDouble* v_black_level;
	VarBool *v_auto_exposure;
	VarDouble *v_exposure_time;
	VarStringEnum *v_color_mode;
	VarDouble *v_target_fps;
	VarBool *v_limit_mode;
	VarStringEnum *v_buffer_mode;
	// double f_balance_ratio_red;
	// double f_balance_ratio_green;
	// double f_balance_ratio_blue;
	// double f_gain;
	// double f_exposure_time;
	// bool is_param_recoved;

	void resetCamera(unsigned int new_id);
	bool _stopCapture();
	bool _buildCamera();
	bool _mallocImageBuf(); // Allocate image buffer
	bool _freeImageBuf();	// Release image buffer
	bool _setCamera();
	bool _getColorFilter();
	bool _convertFormat(PGX_FRAME_BUFFER frame_buf);

	static string toString(GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY e) {
		switch (e) {
		case GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST:
			return "OldestFirst";
		case GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST_OVERWRITE:
			return "OldestOverwrite";
		case GX_DS_STREAM_BUFFER_HANDLING_MODE_NEWEST_ONLY:
			return "NewestOnly";
		default:
			return "OldestFirst";
		}
	}
	static GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY stringToBufferMode(const char *s) {
		if (strcmp(s, "OldestFirst") == 0) {
			return GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST;
		} else if (strcmp(s, "OldestOverwrite") == 0) {
			return GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST_OVERWRITE;
		} else if (strcmp(s, "NewestOnly") == 0) {
			return GX_DS_STREAM_BUFFER_HANDLING_MODE_NEWEST_ONLY;
		}
		return GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST;
	}

// A slight blur helps to reduce noise and improve color recognition.
#ifdef OPENCV
	static const double blur_sigma;
	void gaussianBlur(RawImage &img);
	void contrast(RawImage &img, double factor);
	void sharpen(RawImage &img);
#endif
};

#endif /* CAPTURE_DAHENG_H */