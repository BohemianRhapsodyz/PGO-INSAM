#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"
class CSINSOD;

typedef struct {
	CVect3 wm, vm, fog;
	double t;
	CVect3 gpsv, gpsp;
} ImuGpsData;

class CSINSOD :public CSINSTDKF	// sizeof(CSINSGPSOD)~=30k bytes
{
public:
	CVect3 posDR, vnRes, posRes;
	CVect3 lvOD;
	CMat3 Cbo, MpkD;			// Cbo: from body-frame to OD-frame
	double Kod, tODInt;
	BOOL measGPSvnValid, measGPSposValid, measODValid, measMAGyawValid;

	CSINSOD(void);
	virtual void Init(const CSINS &sins0, int grade = -1);
	virtual void SetFt(int nnq);
	virtual void SetMeas(void);
	virtual void Feedback(double fbts);
	void SetMeasGPS(const CVect3 &pgps = O31, const CVect3 &vgps = O31);
	void SetMeasOD(double dSod, double ts);
	void SetMeasYaw(double ymag);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
	int TDUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nStep);
};

//Initial pos and att
double latitude0=0.59393781, longtitude0=1.8983, altitude0=444.84;
double roll0=0.01843, pitch0=0.0171802, yaw0=3.064575;
//double latitude0=0.393280276, longtitude0=1.988601613, altitude0=91.169;
//double roll0=0.034417893, pitch0=0.0312763, yaw0=1.888446251;
#endif


