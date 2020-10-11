/* 
 *  $Id: format.c,v 1.7 2003/06/30 08:24:45 dmitry Exp $
 *
 *  Copyright (c) 2003, Dmitry Yurtaev <dm1try@umail.ru>
 *
 *  This is free software; you can redistribute it and/or modify it under the
 *  terms of the GNU General Public License as published by the Free Software
 *  Foundation; either version 2, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 *  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 */

#include <StringMgr.h>
#include "format.h"
#include "panel.h"

#define DEG "\260"	// degree symbol

extern UInt8 metricUnit;
extern UInt8 mapScaling; // Scaling selection for MAP sensor
extern UInt8 mafHz; // added for DSM MAF Hz logging
extern UInt8 carSel; // added for DSM/3S Car Select

/*

MMC Galant E3x, MMC Eclipse 1G, ...

*/


// just decimal value 0..255
Int16 f_DEC(UInt8 d, Char *s) {
	return StrPrintF(s, "%u", (UInt16)d);
}


// hexadecimal value
Int16 f_HEX(UInt8 d, Char *s) {
	return StrPrintF(s, "%x", d);
}


// Flags 0
Int16 f_FLG0(UInt8 d, Char *s) {
	return StrPrintF(s, "%s%s", 
		d & 0x10 ? "-" : "F",	// Fuel Pump Relay
		d & 0x20 ? "-" : "A"	// AC Clutch
	);
}


// Flags 1
Int16 f_FLG1(UInt8 d, Char *s) {
	return StrPrintF(s, "%s%s", 
		d & 0x10 ? "-" : "C",	// CE Light
		d & 0x20 ? "-" : "P"	// Canister Purge
	);
}


// Flags 2
Int16 f_FLG2(UInt8 d, Char *s) {
	return StrPrintF(s, "%s%s%s%s%s", 
		d & 0x04 ? "-" : "T",	// TDC Sensor
		d & 0x08 ? "S" : "-",	// Power Steering
		d & 0x10 ? "-" : "A",	// AC Switch
		d & 0x20 ? "-" : "N",	// Park / Neutral Switch (A/T)
		d & 0x80 ? "I" : "-"	// Idle Switch
	);
}


// Air Temperature, degrees C
// interp - 60
Int16 f_AIRT(UInt8 d, Char *s) {
	static const UInt8 interp[17] = {
		0xF4, 0xB0, 0x91, 0x80, 0x74, 0x6A, 0x62, 0x5A,
		0x53, 0x4C, 0x45, 0x3E, 0x35, 0x2B, 0x1D, 0x01,
		0x01
	};

	UInt8 v1 = interp[d / 16], v2 = interp[d / 16 + 1];

	if(metricUnit == PREFS_UNIT_METRIC) {
		Int32 v = v1 * 65536L - (d % 16) * (v1 - v2) * 4096L - 60 * 65536L;
		return StrPrintF(s, "%ld.%ld" DEG "C", v >> 16, (v & 0xffff) * 10 >> 16);
	} else if(metricUnit == PREFS_UNIT_ENGLISH) {
		Int32 v = v1 * 65536L * 9/5 - (d % 16) * (v1 - v2) * 4096L  * 9/5 - 60 * 65536L * 9/5 + 32 * 65536L;
		return StrPrintF(s, "%ld.%ld" DEG "F", v >> 16, (v & 0xffff) * 10 >> 16);
	} else {
		return f_DEC(d, s);
	}
}


// Coolant temperature, degrees C
// interp - 80
Int16 f_COOL(UInt8 d, Char *s) {
	static const UInt8 interp[17] = {
		0xEE, 0xBE, 0xA0, 0x90, 0x84, 0x7B, 0x73, 0x6C,
		0x65, 0x5F, 0x58, 0x51, 0x49, 0x40, 0x33, 0x15,
		0x15
	};

	UInt8 v1 = interp[d / 16], v2 = interp[d / 16 + 1];
	
        if(metricUnit == PREFS_UNIT_METRIC) {
		Int32 v = v1 * 65536L - (d % 16) * (v1 - v2) * 4096L - 80 * 65536L;
		return StrPrintF(s, "%ld.%ld" DEG "C", v >> 16, (v & 0xffff) * 10 >> 16);
	} else if(metricUnit == PREFS_UNIT_ENGLISH) {
		Int32 v = v1 * 65536L * 9/5 - (d % 16) * (v1 - v2) * 4096L  * 9/5 - 80 * 65536L * 9/5 + 32 * 65536L;
		return StrPrintF(s, "%ld.%ld" DEG "F", v >> 16, (v & 0xffff) * 10 >> 16);
	} else {
		return f_DEC(d, s);
	}
}


// G/REV
// Value * 1.035 / 128 = g/rev
Int16 f_GREV(UInt8 d, Char *s) {
	Int32 v = d * 67830L / 128;
	return StrPrintF(s, "%ld.%02ld", v >> 16, (v & 0xffff) * 100 >> 16);
}


// Lb/min
// g/Rev * RPM value in hex from mmcd.c * 64.375 / 453.592370 ... ugly ugly ugly
Int16 f_LBMN(UInt8 d, Char *s) {
	Int32 v = d * 4218815L / 453.592370; 
	return StrPrintF(s, "%ld.%02ld", v >> 16, (v & 0xffff) * 100 >> 16);
}


// EGRT Temperature, degrees C
// -1.5x + 314.2(7)
// F: -2.7x + 597.7
Int16 f_EGRT(UInt8 d, Char *s) {
if(metricUnit == PREFS_UNIT_METRIC) {
		Int32 v = -98304L * d + 20596508L;
		return StrPrintF(s, "%ld.%ld" DEG "C", v >> 16, (v & 0xffff) * 10 >> 16);
	} else if(metricUnit == PREFS_UNIT_ENGLISH) {
		Int32 v = -176947L * d + 39170867L;
		return StrPrintF(s, "%ld.%ld" DEG "F", v >> 16, (v & 0xffff) * 10 >> 16);
	} else {
		return f_DEC(d, s);
	}
}


// External MAP conversions for coming in on the EGR-T line
Int16 f_MAP(UInt8 d, Char *s) {

	static const Int32 metricScale[4]= {
	0.0117647 * 65535L,
	0.0098039 * 65535L,
	0.0078431 * 65535L,
	-98304L };

	static const Int32 metricOffset[4]= {
	0.1 * 65535L,
	0.1 * 65535L,
	0.1 * 65535L,
	20596508L};

	static const Int32 englishScale[4]= {
	0.17063 * 65535L,
	0.14219 * 65535L,
	0.11375 * 65535L,
	-176947L};

	static const Int32 englishOffset[4]= {
	-13.0534 * 65535L,
	-13.0534 * 65535L,
	-13.0534 * 65535L,
	391708L};


	if(metricUnit == PREFS_UNIT_METRIC) {
		Int32 v = metricScale[mapScaling]* d +  metricOffset[mapScaling];
		return StrPrintF(s, "%ld.%02ld bar", v >> 16, (v & 0xffff) * 100 >> 16);
	} else if(metricUnit == PREFS_UNIT_ENGLISH) {
		Int32 v = englishScale[mapScaling]* d +  englishOffset[mapScaling];
		if (v > 0 ){
			return StrPrintF(s, "%ld.%01ld psi", v >> 16, (v & 0xffff) * 10 >> 16);}
		else if (v < -65535) {
			return StrPrintF(s, "%ld.%01ld psi", v >> 16, ((v & 0xffff) ^ 0xffff ) * 10 >> 16);}
		else {
			return StrPrintF(s, "-0.%01ld psi", ((v & 0xffff) ^ 0xffff ) * 10 >> 16);}
	} else {
		return f_DEC(d, s);
	}
}


// Battery voltage, 0.0..18.7 volts
// 0.0733 * x
Int16 f_BATT(UInt8 d, Char *s) {
	Int32 v = 4804L * d;
	return StrPrintF(s, "%ld.%ldV", v >> 16, (v & 0xffff) * 10 >> 16);
}


// Engine speed, 0..8000 rpm
// 31.25 * x
Int16 f_ERPM(UInt8 d, Char *s) {
	Int32 v = 2048000L * d;
	return StrPrintF(s, "%ldrpm", v >> 16);
}


// Injector pulse duty period, 0.0..65.0 ms
// 0.256 * x
Int16 f_INJP(UInt8 d, Char *s) {
	Int32 v = 16777L * d;
	return StrPrintF(s, "%ld.%02ldms", v >> 16, (v & 0xffff) * 100 >> 16);
}


// Barometric pressure, 0.00..1.24 bar
// 0.00486 * x
// 1 bar = 14.50326 psi
Int16 f_BARO(UInt8 d, Char *s) {
	if(metricUnit == PREFS_UNIT_METRIC) {
		Int32 v = 319L * d;
		return StrPrintF(s, "%ld.%02ld bar", v >> 16, (v & 0xffff) * 100 >> 16);
	} else if(metricUnit == PREFS_UNIT_ENGLISH) {
		Int32 v = 4619L * d;
		return StrPrintF(s, "%ld.%02ld psi", v >> 16, (v & 0xffff) * 100 >> 16);
	} else {
		return f_DEC(d, s);
	}
}


// Mass air flow sensor, 0-1600Hz 6.29 * x
// or 0-3200hz 12.58x for DSM's who've changed their MAF hz logging
Int16 f_AIRF(UInt8 d, Char *s) {
	if(mafHz == PREFS_UNIT_MAF16) {
		Int32 v = 412221L * d;
		return StrPrintF(s, "%ld.%ldHz", v >> 16, (v & 0xffff) * 10 >> 16);
} else if(mafHz == PREFS_UNIT_MAF32) {
		Int32 v = 824442L * d;
		return StrPrintF(s, "%ld.%ldHz", v >> 16, (v & 0xffff) * 10 >> 16);
	} else {
		return f_DEC(d, s);
	}
}


// Throttle position, 0..100%
Int16 f_THRL(UInt8 d, Char *s) {
	Int32 v = 25600L * d;
	return StrPrintF(s, "%ld.%ld%%", v >> 16, (v & 0xffff) * 10 >> 16);
}


// Fuel trim, 0..200%
// f: 0.7843 * x
Int16 f_FTxx(UInt8 d, Char *s) {
	Int32 v = 51200L * d;
	return StrPrintF(s, "%ld.%ld%%", v >> 16, (v & 0xffff) * 10 >> 16);
}


// Oxygen sensor voltage, 0.00..5.00 volts
// f: 0.0196 * x
Int16 f_OXYG(UInt8 d, Char *s) {
	Int32 v = 1280L * d;
	return StrPrintF(s, "%ld.%02ldV", v >> 16, (v & 0xffff) * 100 >> 16);
}


// Timing advance
// DSM: f: x - 10
// 3S: f: x - 22 (for whatever reason mitsu made them -12 from DSM)
Int16 f_TIMA(UInt8 d, Char *s) {
	if(carSel == PREFS_UNIT_CARDSM) {
		return StrPrintF(s, "%ld" DEG, (Int32)d - 10);
} else if(carSel == PREFS_UNIT_CAR3S) {
		return StrPrintF(s, "%ld" DEG, (Int32)d - 22);
	} else {
		return f_DEC(d, s);
	}
}


// Added this to get a NA on ACLE for 3S cars ... I'm not the smartest programmer :( - Andrew Shinn
Int16 f_ACLE(UInt8 d, Char *s) {
	if(carSel == PREFS_UNIT_CARDSM) {
		Int32 v = 25600L * d;
		return StrPrintF(s, "%ld.%ld%%", v >> 16, (v & 0xffff) * 10 >> 16);
} else if (carSel == PREFS_UNIT_CAR3S) {
		return StrPrintF(s, "N/A");
	} else {
		return f_DEC(d, s);
	}
}


// just decimal value 0..255 ... will eventually be something scaled/useful
Int16 f_ANA1(UInt8 d, Char *s) {
	return StrPrintF(s, "%u", (UInt16)d);
}


// just decimal value 0..255 ... will eventually be something scaled/useful
Int16 f_ANA2(UInt8 d, Char *s) {
	return StrPrintF(s, "%u", (UInt16)d);
}