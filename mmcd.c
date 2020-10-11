/* 
 *  $Id: mmcd.c,v 1.16 2003/07/03 10:29:50 dmitry Exp $
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

#include <PalmOS.h>
#include <HsExt.h>
#include "mmcd.h"
#include "format.h"
#include "graph.h"
#include "panel.h"

//
UInt32 screenWidth, screenHeight, screenDepth;
Int16 screenDensity;
FontPtr bigFont;

// global state vars
Boolean colorMode = false;
UInt32 ticksPerSecond;

UInt16 portId;
UInt16 autoOff;
Boolean talkingECU = false;
Boolean portOpen = false;
UInt8 currentSensor = 0;
UInt8 currentBank = 0;
UInt8 currentMode = 0;
UInt8 scanError = 0; 
Int8 lastSensorReturned = 0;

#define REVIEW_MODE	0
#define MONITOR_MODE	1

UInt32 graphSensors = 0;
UInt32 captureSensors = 0;
UInt8 selectedSensor = 0;
GraphSample currentSample;
FileHand stream;
Char streamName[32];
UInt16 streamAttr;

Char editFormTitle[32];

Int32 alarmFreq[5] = { 0, 4000, 1000, 300, 50 };

// custom widgets
GraphType graph;
PanelType panel;
Coord graphX, graphY, graphW, graphH;

//#define ECU_DEBUG
#ifndef ECU_DEBUG
#define ECU_BAUDRATE 1920
#elif
#define ECU_BAUDRATE 1953
#elif
#define ECU_BAUDRATE 8192
#elif
#define ECU_BAUDRATE 9600
#elif
#define ECU_BAUDRATE 19200
#elif
#define ECU_BAUDRATE 38400
#else
#define ECU_BAUDRATE 56700
#endif

/*
 *
 */

typedef struct Prefs {
	UInt32 exists;
	UInt32 capture;
	UInt32 graph;
	UInt8 currentBank;
	UInt8 currentMode;
	UInt8 addr[SENSOR_COUNT];
	UInt8 metricUnit;
	UInt8 mapScaling;
	UInt8 mafHz; // added for MAF Hz Select
	UInt8 carSel; // added for Car Select
	UInt32 serialSpeed;
	Char slug[SENSOR_COUNT][4];
	UInt16 topVisible;
	UInt8 selectedSensor;

	UInt32 polarity;
	UInt8 threshold[SENSOR_COUNT];
	UInt8 sound[SENSOR_COUNT];

	Boolean hideUnused;
	Boolean numericMode;
	Boolean audibleAlarms;

//	UInt32 dummy;
} Prefs;

UInt16 topVisible;
UInt32 serialSpeed;
UInt8 metricUnit;
UInt8 mapScaling;
UInt8 mafHz; // added for MAF Hz Select
UInt8 carSel; // added for Car Select
UInt8 audibleAlarms;
UInt8 numericMode;	// show sensor numeric value in Monitor mode
			// instead of graph
/* Insert SP 06/01/03 - Hide Unused preference */
Boolean fHideUnused;
/* End Insert */

UInt8 sensorPosition[SENSOR_COUNT] = { 
	17, 14, 19, 9, 18,  3, 10, 22,
	 4, 11, 21, 13, 12,  0,  20, 23,
	 5, 15,  6, 16,  7,  1,  8,  2,
	24, 25, 26, 27, 28, 29, 30, 31,
};

/* Insert SP 06/01/03 - default sensor order */
const UInt8 defSensorPosition[SENSOR_COUNT] = {
	17, 14, 19, 9, 18,  3, 10, 22,
	 4, 11, 21, 13, 12,  0,  20, 23,
	 5, 15,  6, 16,  7,  1,  8,  2,
	24, 25, 26, 27, 28, 29, 30, 31,
};
/* End Insert */

UInt8 GetNextSensorOriginal(UInt8 index)
{
   	return (index+1) % 32; //Original logic
}

UInt8 GetNextSensor(UInt8 currentsensor)
{

	UInt8	index = 0; // Find current sensor and return next
	while( sensorPosition[index++] != currentsensor);
	index = index % 32;
	if (sensorPosition[index] == lastSensorReturned){ // Stuck at the end of the hide list
		index = 0; // Return first sensor in list
	}
	lastSensorReturned = sensorPosition[index];
   	return sensorPosition[index];
}

// calculate injector duty cycle from RPM and injector pulse width, calculate lb/hr flow from RPM and g/rev
void computeDerivatives(GraphSample* sample) {
	if(sample->dataPresent & 1L << IDX_RPM && // calculate inj. pulse width
	   sample->dataPresent & 1L << IDX_INJP
	) {
		Int32 v = sample->data[IDX_INJP] * sample->data[IDX_RPM] / 117L;
		sample->data[IDX_INJD] = MIN(v, 255);
		sample->dataPresent |= 1L << IDX_INJD;
	}
	if(sample->dataPresent & 1L << IDX_RPM && // calculate lb/min ... this is so ugly, but i fried my brain. I'm such a lame programmer :)
	   sample->dataPresent & 1L << IDX_GREV
	) {
		Int32 v = sample->data[IDX_GREV] * sample->data[IDX_RPM] / 117L;
		sample->data[IDX_LBMN] = MIN(v, 255);
		sample->dataPresent |= 1L << IDX_LBMN;
	}
}

/* transpose function - row to column orientation */
#define XPOSE(i) ((i&0x18)|(i<<1&6)|(i>>2&1))

/* Insert SP 06/01/03 - hide/unhide unused sensors */
void HideUnused(void) {
	UInt8 i, j = 0;

	if(currentMode == REVIEW_MODE) {
		for(i = 0; i < SENSOR_COUNT; i++) {
			UInt8 xi = XPOSE(i);
			SensorType *sensor = &_pnlSensor[defSensorPosition[xi]];
			if(sensor->exists && (currentSample.dataPresent & (1L << defSensorPosition[xi]))) {
				sensorPosition[XPOSE(j)] = defSensorPosition[xi];
				j++;
			}
		}
	} else if(currentMode == MONITOR_MODE) {
		for(i = 0; i < SENSOR_COUNT; i++) {
			UInt8 xi = XPOSE(i);
			SensorType *sensor = &_pnlSensor[defSensorPosition[xi]];
			if(sensor->exists && (sensor->capture || sensor->graph)) {
				sensorPosition[XPOSE(j)] = defSensorPosition[xi];
				j++;
			}
		}
	} else {
		return;
	}

	/* fill remaining slots with blanks */
	for(; j < SENSOR_COUNT; j++) {
		sensorPosition[XPOSE(j)] = 0;
	}
}

void UnhideUnused(void) {
	UInt8 i;

	for(i = 0; i < SENSOR_COUNT; i++)
		sensorPosition[i] = defSensorPosition[i];
}

void RefreshPanel(void) {
	if(fHideUnused) HideUnused();
		PnlSetBank(&panel, &sensorPosition[currentBank * 8]);
}
/* End Insert */

/*
 *  return pointer to object in form by its identifier, if found
 *  NULL otherwise
 */
void *FrmGetObjectPtrById(FormType *form, UInt16 id)
{
	UInt16 index = FrmGetObjectIndex(form, id);
	return index == frmInvalidObjectId ? NULL : FrmGetObjectPtr(form, index);
}

/*
 *  graph-callback stream reader
 */

Int32 streamReader(UInt16 id, Int32 streamOffset, GraphSample *pBuffer, Int32 sampleCount)
{
	Int32 samplesRead = 0, i;
	Err err;

	// stream length request
	if(pBuffer == NULL) {
		FileTell(stream, &samplesRead, &err);
		return samplesRead / sizeof(GraphSample);
	}

	if(sampleCount == 0) return 0;

	FileSeek(stream, streamOffset * sizeof(GraphSample), fileOriginBeginning);
	samplesRead = FileRead(stream, pBuffer, sizeof(GraphSample), sampleCount, &err);
	if(samplesRead < sampleCount)
		MemSet(&pBuffer[samplesRead], (sampleCount - samplesRead) * sizeof(GraphSample), 0);

	// for compatibility with old logs
	for(i = 0; i < samplesRead; i++) {
		computeDerivatives(pBuffer + i);
	}
	return samplesRead;
}

Int32 streamWriter(GraphSample *sample)
{
	Int32 samplesWritten = 0;
	Err err;

	FileSeek(stream, 0, fileOriginEnd);
	samplesWritten = FileWrite(stream, sample, sizeof(GraphSample), 1, &err);
	return samplesWritten;
}

/*
 *  open serial port, leaving handle in portId global var
 */

Boolean		waitingAnswer;
UInt32		receiveTimeout = 0;
GraphSample	scratchSample;

Err openPort(UInt32 baudRate, Boolean enableFlowControl)
{
	Err err;

	if(!portOpen) {
		err = SrmOpen(serPortCradleRS232Port, baudRate, &portId);
		if(err != errNone)
			err = SrmOpen(serPortCradlePort, baudRate, &portId);
		if(err != errNone) {
			ErrAlert(err);
			return err;
		}

		if(!enableFlowControl) {
			UInt16 paramSize;
			UInt32 flags =	srmSettingsFlagBitsPerChar8 |
					srmSettingsFlagStopBits1;
			paramSize = sizeof(flags);
			err = SrmControl(portId, srmCtlSetFlags, &flags, &paramSize);
		}
		portOpen = true;

		// disable auto-sleep when talking to ECU
		autoOff = SysSetAutoOffTime(0);

		currentSensor = 0;
		waitingAnswer = false;
		scratchSample.dataPresent = 0;
	}

	return 0;
}

Err closePort()
{
//	Err err;

	if(portOpen) {
		portOpen = false;
		SrmClose(portId);
	}

	if(autoOff) {
		SysSetAutoOffTime(autoOff);
		autoOff = 0;
	}

	return 0;
}

Boolean lastSensor()
{
	return currentSensor == (SENSOR_COUNT - 1) || captureSensors < (1L << (currentSensor + 1));
}

Boolean nextSensor()
{
	if(captureSensors == 0) {
		currentSensor = 0;
		scratchSample.dataPresent = 0;
		return false;
	}

	for(;;) {
		currentSensor = (currentSensor + 1) % SENSOR_COUNT;
		if (currentSensor == IDX_INJD) currentSensor++; // Skip INJD sensor
		if (currentSensor == IDX_LBMN) currentSensor++; // Skip LBMN sensor
		if(captureSensors & (1L << currentSensor)) break;
	}

	return true;
}

Boolean switchMode(UInt8 newMode); //Resolve forward reference
void haltScanning(); // FOrward reference


Boolean queryECU(GraphSample *sample)
{
	Err err;
	Boolean sampleComplete = false;

	if(!portOpen) return false;

	ErrTry {
		if(waitingAnswer) {
			UInt32 bytesAvailable;
			UInt8 buf[2];

			// any data available?
			err = SrmReceiveCheck(portId, &bytesAvailable);
			if(err != errNone) ErrThrow(0);

			// got the answer?
			if(bytesAvailable >= 2) {
				Int32 bytesReceived = SrmReceive(portId, &buf, 2, 1, &err);
				if(err != errNone || bytesReceived != 2) ErrThrow(0);

//				ASSERT(buf[0] == _pnlSensor[currentSensor].addr);
				scratchSample.dataPresent |= 1L << currentSensor;
				scratchSample.data[currentSensor] = buf[1];

				waitingAnswer = false;

				scanError = 0;

			
			} // not yet - half-second timeout expired?
			else if(TimGetTicks() - receiveTimeout > ticksPerSecond / 2) 
				{
				if (scanError++ > 10) //pause to allow powerdown
				{				  
					//switchMode(REVIEW_MODE);
					haltScanning();
					scanError = 0;
				}
				ErrThrow(0);
			// wait some more...
				} 
			else {
#ifdef ECU_DEBUG
				// pretend that we got an anwer
				scratchSample.dataPresent |= 1L << currentSensor;
				scratchSample.data[currentSensor] = SysRandom(0);
				waitingAnswer = false;
#else
				return false;
#endif
			}

			// check  threshold
			// if((audibleAlarms && _pnlSensor[currentSensor].sound)
	 		// && ((currentSensor == 10 || currentSensor == 22)&& (scratchSample.data[14] > 200)))

 		 	if( (audibleAlarms && _pnlSensor[currentSensor].sound)
	 		    && (((currentSensor == 10 || currentSensor == 22)&& (scratchSample.data[14] > 200))
			    || (currentSensor != 10 && currentSensor !=  22)))

			{
				if((_pnlSensor[currentSensor].polarity == 0 && 				scratchSample.data[currentSensor] > _pnlSensor				[currentSensor].threshold) ||
				(_pnlSensor[currentSensor].polarity == 1 && 				scratchSample.data[currentSensor] < _pnlSensor				[currentSensor].threshold)) 
				
				{
					SndCommandType sndCmd = { sndCmdFrqOn, 0, 					alarmFreq[_pnlSensor								[currentSensor].sound], 100, sndMaxAmp };
					SndDoCmd(NULL, &sndCmd, true);
				}
				
			}
		}
	} ErrCatch(exception __attribute__ ((unused))) {
		SrmClearErr(portId);
		SrmReceiveFlush(portId, 0);
		waitingAnswer = false;
	} ErrEndCatch

//	ASSERT(waitingAnswer == false);

	if(lastSensor() && scratchSample.dataPresent) {
		// before starting over return collected sample to the caller
		*sample = scratchSample;
		sample->time = TimGetSeconds();
		sampleComplete = true;
		scratchSample.dataPresent = 0;
	}

	if(nextSensor()) {
		SrmSend(portId, &_pnlSensor[currentSensor].addr, 1, &err);
		receiveTimeout = TimGetTicks();
		waitingAnswer = true;
	}

	return sampleComplete;
}

/*
 *  review mode-specific event handler
 */


Int32 scrollerScale = 1;
void _setScroller(Int32 value) {
	ScrollBarType *bar = FrmGetObjectPtrById(FrmGetActiveForm(), scrollBarId);
	scrollerScale = graph.length / 32000 + 1;
	SclSetScrollBar(bar, value / scrollerScale, 0, 
		graph.length > graph.width
		? (graph.length - graph.width) / scrollerScale
		: 0,
		graph.width / scrollerScale
	);
}

void updateSampleTime()
{
	static Char s[32];
	DateTimeType date;

	TimSecondsToDateTime(currentSample.time, &date);
	StrPrintF(s, "%02d:%02d:%02d", date.hour, date.minute, date.second);
	FrmCopyLabel(FrmGetActiveForm(), timeLabelId, s);
}

Boolean reviewHandleEvent(EventType* e)
{
	/* Insert SP 06/03/03 */
	FormType *form = FrmGetActiveForm();
	/* End Insert */

	if(GrfHandleEvent(&graph, e)) return true;

	switch(e->eType) {

	case ctlSelectEvent:
		switch(e->data.ctlSelect.controlID) {
		case hideUnusedButtonId:
			fHideUnused = !fHideUnused;
			if (fHideUnused) {
				/* highlight "H" button */
				FrmSetControlValue(form, FrmGetObjectIndex(form, hideUnusedButtonId), 1);
			} else {
				/* deselect "H" button */
				FrmSetControlValue(FrmGetActiveForm(), FrmGetObjectIndex(form, hideUnusedButtonId), 0);
				UnhideUnused();
			}
			RefreshPanel();
			return true;
		}
		break;

	case sclRepeatEvent:
		// user moved scroller - update graph
		if(e->data.sclRepeat.newValue != e->data.sclRepeat.value)
			GrfSetGraph(&graph, graph.length, e->data.sclRepeat.newValue * scrollerScale, graph.cursor, graph.dataMask);
		return false;

	case grfChangeEvent:
		// graph has been changed by user interaction
		{
			GraphEventType *event = (GraphEventType *)e;

			// position changed? update scroller, then
			if(event->data.grfChange.positionChanged || event->data.grfChange.lengthChanged) {
				_setScroller(event->data.grfChange.position);
			}

			// cursor moved? update values and currectSample
			if(event->data.grfChange.cursorChanged) {
				currentSample = graph._buffer[graph.cursor - graph.position];
				computeDerivatives(&currentSample);
				/* Insert SP 06/01/03 - update hidden sensors */
				RefreshPanel();
				/* End Insert */
				PnlUpdate(&panel, &currentSample);
				updateSampleTime();
			}
		}
		return true;

	/*
	 * removed, pageUp and pageDown now scroll thru banks
	 */
	
	case keyDownEvent:

		// move cursor 1 sample left if not at the beginning of graph
		if(e->data.keyDown.chr == hard1Chr && graph.cursor > 0) {
			Int32 newPosition = graph.position;
			Int32 newCursor = graph.cursor - 1;

			SndPlaySystemSound(sndClick);

			// update graph and scroller positions
			// if moved out of visible area
			if(newCursor < newPosition) {
				_setScroller(newPosition);
				newPosition = newCursor;
			}

			// update the graph
			GrfSetGraph(&graph, graph.length, newPosition, newCursor, graph.dataMask);

			// update values
			currentSample = graph._buffer[graph.cursor - graph.position];
			computeDerivatives(&currentSample);
			// Insert SP 06/01/03 - update hidden sensors
			RefreshPanel();
			// End Insert
			PnlUpdate(&panel, &currentSample);
			updateSampleTime();
			return true;
		}

		// move cursor 1 sample right if not at the end of graph
		if(e->data.keyDown.chr == hard2Chr && graph.cursor < graph.length - 1) {
			Int32 newPosition = graph.position;
			Int32 newCursor = graph.cursor + 1;

			SndPlaySystemSound(sndClick);

			// update graph and scroller positions
			// if moved out of visible area
			if(newCursor >= newPosition + graph.width) {
				newPosition = newCursor - graph.width + 1;
				_setScroller(newPosition);
			}

			// update the graph
			GrfSetGraph(&graph, graph.length, newPosition, newCursor, graph.dataMask);

			// update values
			currentSample = graph._buffer[graph.cursor - graph.position];
			computeDerivatives(&currentSample);
			// Insert SP 06/01/03 - update hidden sensors
			RefreshPanel();
			// End Insert
			PnlUpdate(&panel, &currentSample);
			updateSampleTime();
			return true;
		}
		break;

	default:
		break;
	}
	return false;
}

/*
 *  monitor mode-specific event handler
 */



void haltScanning()
{
	FormType *form = FrmGetActiveForm();

	SndPlaySystemSound(sndClick);
	{

		UInt16 index = FrmGetObjectIndex(form, pauseCheckId);
		Int16 newValue = FrmGetControlValue(form, index) ? 0 : 1;
		FrmSetControlValue(form, index, newValue);
				
		if(newValue) 
		{
			RectangleType r = { { 1, 147 }, { 50, 13 } };
			WinEraseRectangle(&r, 0);
			FrmShowObject(form, FrmGetObjectIndex(form, reviewButtonId));
			FrmShowObject(form, FrmGetObjectIndex(form, doneButtonId));
			closePort();
			talkingECU = false;
			GrfAppendSample(&graph, NULL);
		}

	}

}


Boolean monitorHandleEvent(EventType* e)
{
	FormType *form = FrmGetActiveForm();
	static UInt32 startTicks;
	static UInt32 cycles;
	static UInt32 lastUpdateTime;
	static Boolean needsUpdate;
	static GraphSample sample;
	static Boolean logging;

	GrfHandleEvent(&graph, e);

	if(e->eType == keyDownEvent && !(e->data.keyDown.modifiers & autoRepeatKeyMask)) {
		switch(e->data.keyDown.chr) {
		case hard1Chr:
			// change sensor for numeric display
			SndPlaySystemSound(sndClick);
			PnlSetSelection(&panel, selectedSensor = GrfSetSensor(&graph, 0xff));
			return true;

		case hard2Chr:
			// toggle Graph checkbox
			SndPlaySystemSound(sndClick);
			{
				UInt16 index = FrmGetObjectIndex(form, graphCheckId);
				Int16 newValue = FrmGetControlValue(form, index) ? 0 : 1;
				FrmSetControlValue(form, index, newValue);
				GrfSetMode(&graph, numericMode = !newValue);
				GrfAppendSample(&graph, NULL);

				PnlSetMode(&panel, numericMode ? PANEL_SINGLESELECT : PANEL_MULTISELECT);
			}
			return true;

		case hard3Chr:
			// toggle Log checkbox
			SndPlaySystemSound(sndClick);
			{
				UInt16 index = FrmGetObjectIndex(form, logCheckId);
				Int16 newValue = FrmGetControlValue(form, index) ? 0 : 1;
				FrmSetControlValue(form, index, newValue);

				logging = newValue;
			}
			return true;

		case hard4Chr:
			// toggle Pause checkbox
			SndPlaySystemSound(sndClick);
			{
				UInt16 index = FrmGetObjectIndex(form, pauseCheckId);
				Int16 newValue = FrmGetControlValue(form, index) ? 0 : 1;
				FrmSetControlValue(form, index, newValue);
				
				if(newValue) {
					RectangleType r = { { 1, 147 }, { 50, 13 } };

					WinEraseRectangle(&r, 0);
					FrmShowObject(form, FrmGetObjectIndex(form, reviewButtonId));
					FrmShowObject(form, FrmGetObjectIndex(form, doneButtonId));
					closePort();
					talkingECU = false;
					GrfAppendSample(&graph, NULL);
				} else {
					FrmHideObject(form, FrmGetObjectIndex(form, reviewButtonId));
					FrmHideObject(form, FrmGetObjectIndex(form, doneButtonId));
					// update current mode pushbutton as its frame
					// gets erased by neighbour hiding buttons
					FrmShowObject(form, FrmGetObjectIndex(form, monitorButtonId));
					openPort(serialSpeed, false);
					talkingECU = true;
					startTicks = TimGetTicks();
					cycles = 0;
				}
			}
			return true;
		}
	}
	
	/* Modified SP 06/01/03 - change conditional structure */
	if (e->eType == ctlSelectEvent) {
		switch (e->data.ctlSelect.controlID) {
		case pauseCheckId:
			// disable mode-switching when serial communication
			// is in progress
			if(e->data.ctlSelect.on) {
				RectangleType r = { { 1, 147 }, { 50, 13 } };

				WinEraseRectangle(&r, 0);
				FrmShowObject(form, FrmGetObjectIndex(form, reviewButtonId));
				FrmShowObject(form, FrmGetObjectIndex(form, doneButtonId));
				closePort();
				talkingECU = false;
				GrfAppendSample(&graph, NULL);
			} else {
				FrmHideObject(form, FrmGetObjectIndex(form, reviewButtonId));
				FrmHideObject(form, FrmGetObjectIndex(form, doneButtonId));
				// update current mode pushbutton as its frame
				// gets erased by neighbour hiding buttons
				FrmShowObject(form, FrmGetObjectIndex(form, monitorButtonId));
				openPort(serialSpeed, false);
				talkingECU = true;
				startTicks = TimGetTicks();
				cycles = 0;
			}
			break;

		case logCheckId:
			logging = e->data.ctlSelect.on;
			break;
              
		case graphCheckId:
			GrfSetMode(&graph, numericMode = !e->data.ctlSelect.on);
			GrfAppendSample(&graph, NULL);
			PnlSetMode(&panel, numericMode ? PANEL_SINGLESELECT : PANEL_MULTISELECT);
			break;
              
		/* Insert SP 06/01/03 - handle Hide Unused button */
		case hideUnusedButtonId:
			if (e->data.ctlSelect.on) {
				/* Hide */
				fHideUnused = true;
			} else {
				/* Do not hide */
				UnhideUnused();
				fHideUnused = false;
			}
			RefreshPanel();
			break;
		/* End Insert - handle Hide Unused button */
		}
	}
        /* End Modified - change conditional structure */
        
	// got complete sample?
	if(queryECU(&sample)) {
		computeDerivatives(&sample);
		// show it
		GrfAppendSample(&graph, &sample);
		needsUpdate = true;
		cycles++;

		// if log checkbox is checked - write the sample to log file
		if(logging) streamWriter(&sample);
	}

	if(TimGetTicks() > lastUpdateTime + ticksPerSecond / 6) {
		lastUpdateTime = TimGetTicks();
		PnlUpdate(&panel, &sample);

		// show sample rate and current sensor
		if(portOpen) {
			static Char str[32];
			Int16 strLen, labelWidth;
			RectangleType r;

			strLen = StrPrintF(str, "%ldHz %s", cycles * ticksPerSecond / (lastUpdateTime - startTicks + 1), _pnlSensor[currentSensor].slug);
			labelWidth = FntCharsWidth(str, strLen);
			r.topLeft.x = 1 + labelWidth;
			r.topLeft.y = 147;
			r.extent.x = 50 - labelWidth;
			r.extent.y = 13;
			WinDrawChars(str, strLen, 1, 147);
			WinEraseRectangle(&r, 0);
		}
	}

	return false;
}

void doAboutDialog()
{
	FormType *form = FrmInitForm(aboutDialogId);
	FrmDoDialog(form);
	FrmDeleteForm(form);
}

/*
 *  p r e f s
 */

void FldInsertText(FieldType *field, const Char *source)
{
	MemHandle textHandle;
	Char *text;

	textHandle = FldGetTextHandle(field);
	if(textHandle != NULL) {
		FldSetTextHandle(field, NULL);
		MemHandleFree(textHandle);
	}

	textHandle = MemHandleNew(5);
	text = MemHandleLock(textHandle);
	StrNCopy(text, source, 5);
	MemHandleUnlock(textHandle);

	FldSetTextHandle (field, textHandle);
}

void doPrefsDialog()
{
	FormType *form = FrmInitForm(prefsDialogId);
	UInt16 i, index;
	FieldType *field;
	Char *s, str[16];
	ControlType* ctl;
	//Custom Sensors
	 for(i = 0; i < 8; i++) {
		SensorType *sensor = &_pnlSensor[sensorPosition[3 * 8 + i]];

		index = FrmGetObjectIndex(form, customSensorId+i*3);	// checkbox
		FrmSetControlValue(form, index, sensor->exists);

		index = FrmGetObjectIndex(form, customSensorId+i*3+1);	// addr
		field = FrmGetObjectPtr(form, index);
		StrIToH(str, sensor->addr);
		FldInsertText(field, str + 6);

		index = FrmGetObjectIndex(form, customSensorId+i*3+2);	// slug
		field = FrmGetObjectPtr(form, index);
		s = sensor->slug;
		FldInsertText(field, s);
	}

	ctl = FrmGetObjectPtrById(form, speedTriggerId); // serial speed
	StrPrintF(str, "%ld bps", serialSpeed);
	CtlSetLabel(ctl, str);

	ctl = FrmGetObjectPtrById(form, unitTriggerId); // unit
	switch(metricUnit) {
	case PREFS_UNIT_ENGLISH: CtlSetLabel(ctl, "English"); break;
	case PREFS_UNIT_NUMERIC: CtlSetLabel(ctl, "Numeric"); break;
	default:		 CtlSetLabel(ctl, "Metric"); break;
	}

	ctl = FrmGetObjectPtrById(form, egtinputTriggerId); // EGT input
	switch(mapScaling) {
	case 0: CtlSetLabel(ctl, "3.0 BAR"); break;
	case 1: CtlSetLabel(ctl, "2.5 BAR"); break;
	case 2: CtlSetLabel(ctl, "2.0 BAR"); break;
	case 3: CtlSetLabel(ctl, "EGT"); break;
	}

	ctl = FrmGetObjectPtrById(form, mafTriggerId); // MAF Hz Select
	switch(mafHz) {
	case PREFS_UNIT_MAF32: CtlSetLabel(ctl, "3200Hz"); break;
	default:		 CtlSetLabel(ctl, "1600Hz"); break;
	}

	ctl = FrmGetObjectPtrById(form, carTriggerId); // Car Select
	switch(carSel) {
	case PREFS_UNIT_CAR3S: CtlSetLabel(ctl, "3S"); break;
	default:		 CtlSetLabel(ctl, "DSM"); break;
	}

	index = FrmGetObjectIndex(form, alarmsCheckId);
	FrmSetControlValue(form, index, audibleAlarms);

	i = FrmDoDialog(form);
	if(i == defaultButtonId) {
		MemMove(_pnlSensor, _pnlSensorDefault, sizeof(_pnlSensor));

		captureSensors = 0;
		graphSensors = 0;
		metricUnit = PREFS_UNIT_METRIC;
		serialSpeed = ECU_BAUDRATE;
		mapScaling = 0;
		mafHz = PREFS_UNIT_MAF16;
		carSel = PREFS_UNIT_CARDSM;

		for(i = 0; i < SENSOR_COUNT; i++) {
			if(_pnlSensor[i].capture) captureSensors |= (1L << i);
			if(_pnlSensor[i].graph) graphSensors |= (1L << i);
		}
	} else if(i == okButtonId) {
		//Custom Sensors
		 for(i = 0; i < 8; i++) {
			SensorType *sensor = &_pnlSensor[sensorPosition[3 * 8 + i]];

			index = FrmGetObjectIndex(form, customSensorId+i*3);	// checkbox
			sensor->exists = FrmGetControlValue(form, index) ? 1 : 0;

			index = FrmGetObjectIndex(form, customSensorId+i*3+1);	// addr
			field = FrmGetObjectPtr(form, index);
			s = FldGetTextPtr(field);
			while(s && *s) {
				index <<= 4;
				if(*s >= '0' && *s <= '9')
					index += *s - '0';
				else if(*s >= 'a' && *s <= 'f')
					index += *s - 'a' + 10;
				else if(*s >= 'A' && *s <= 'F')
					index += *s - 'A' + 10;
				s++;
			}
			sensor->addr = index;

			index = FrmGetObjectIndex(form, customSensorId+i*3+2);	// slug
			field = FrmGetObjectPtr(form, index);
			s = FldGetTextPtr(field);
			StrNCopy(sensor->slug, s ? s : "", 5);
		}
		
		ctl = FrmGetObjectPtrById(form, speedTriggerId); // serial speed
		serialSpeed = StrAToI(CtlGetLabel(ctl));

		ctl = FrmGetObjectPtrById(form, unitTriggerId); // Units ... (metric / english)
		s = (Char*)CtlGetLabel(ctl);
		if(0 == StrCompare(s, "English")) {
			metricUnit = PREFS_UNIT_ENGLISH;
		} else if(0 == StrCompare(s, "Numeric")) {
			metricUnit = PREFS_UNIT_NUMERIC;
		} else {
			metricUnit = PREFS_UNIT_METRIC;
		}

		ctl = FrmGetObjectPtrById(form, egtinputTriggerId); // EGR-T Input
		s = (Char*)CtlGetLabel(ctl);
		if(0 == StrCompare(s, "3.0 BAR")) {
			mapScaling = 0;
		} else if(0 == StrCompare(s, "2.5 BAR")) {
			mapScaling = 1;

		} else if(0 == StrCompare(s, "2.0 BAR")) {
			mapScaling = 2;		
		} else {
			mapScaling = 3;  // Revert back to EGT Scaling
		}

		ctl = FrmGetObjectPtrById(form, mafTriggerId); // MAF Hz Select
		s = (Char*)CtlGetLabel(ctl);
		if(0 == StrCompare(s, "3200Hz")) {
			mafHz = PREFS_UNIT_MAF32;
		} else {
			mafHz = PREFS_UNIT_MAF16;
		}

		ctl = FrmGetObjectPtrById(form, carTriggerId); // Car Type Select
		s = (Char*)CtlGetLabel(ctl);
		if(0 == StrCompare(s, "3S")) {
			carSel = PREFS_UNIT_CAR3S;
		} else {
			carSel = PREFS_UNIT_CARDSM;
		}

		index = FrmGetObjectIndex(form, alarmsCheckId);
		audibleAlarms = FrmGetControlValue(form, index) ? true : false;
	}

	FrmDeleteForm (form);
}

/*
 *  a l a r m s
 */

static Char* alarmListSlugs[32];
static Char alarmListStrings[32*6];
static Char alarmListSensor[32];

void updateAlarmThreshold(FormType* form, UInt16 si)
{
	static Char str[16] = "\0";

	_pnlSensor[si].format(_pnlSensor[si].threshold, str);
	FrmCopyLabel(form, threshFieldId, str);
}

void updateAlarmForm(FormType* form, UInt16 si)
{
	UInt16 index;
	ControlType* control;
	UInt16 v[4] = { 0, 255, 1, _pnlSensor[si].threshold };

	index = FrmGetObjectIndex(form, alarmTriggerId);
	control = FrmGetObjectPtr(form, index);
	CtlSetLabel(control, _pnlSensor[si].slug);

	FrmSetControlGroupSelection(form, soundGroupId, sound0ButtonId + _pnlSensor[si].sound);
	index = FrmGetObjectIndex(form, threshSliderId);
	control = FrmGetObjectPtr(form, index);
	CtlSetSliderValues(control, &v[0], &v[1], &v[2], &v[3]);

	index = FrmGetObjectIndex(form, threshSignButtonId);
	control = FrmGetObjectPtr(form, index);
	CtlSetLabel(control, _pnlSensor[si].polarity ? "<" : ">");

	updateAlarmThreshold(form, si);
}

void updateAlarmList(FormType* form)
{
	Int16 i, last = 0;
	ListType* list;
/*
	for(i = 0; i < SENSOR_COUNT; i++) {
		if(_pnlSensor[sensorPosition[i]].exists) {
			alarmListSensor[last] = sensorPosition[i];
			StrNCopy(alarmListSlugs[last], _pnlSensor[sensorPosition[i]].slug, 5);
			if(_pnlSensor[sensorPosition[i]].sound)
				StrNCat(alarmListSlugs[last], "#", 6);
			last++;
		}
	}
*/
	for(i = 0; i < SENSOR_COUNT; i++) {
		if(_pnlSensor[i].exists) {
			alarmListSensor[last] = i;
			StrNCopy(alarmListSlugs[last], _pnlSensor[i].slug, 5);
			if(_pnlSensor[i].sound)
				StrNCat(alarmListSlugs[last], "#", 6);
			last++;
		}
	}

	list = FrmGetObjectPtr(form, FrmGetObjectIndex(form, alarmListId));
	LstSetListChoices(list, alarmListSlugs, last);
}

Boolean alarmsFormHandleEvent(EventType *e)
{
	FormType *form = FrmGetActiveForm();
	SndCommandType sndCmd = { sndCmdFrqOn, 0, 1000, 200, sndMaxAmp };
	static int si;
	Int32 i;
	Err err;

	if(SysHandleEvent(e) || MenuHandleEvent(NULL, e, &err)) return true;
	
	switch(e->eType) {

	case frmOpenEvent:

		for(i = 0; i < SENSOR_COUNT; i++)
			alarmListSlugs[i] = alarmListStrings + i * 6;

		si = selectedSensor & 0x1f;
		for(i = 0; i < SENSOR_COUNT; i++) {
			if(!_pnlSensor[si].exists)
				si = (si + 1) % SENSOR_COUNT;
			else
				break;
		}

		FrmDrawForm(form);
		updateAlarmList(form);
		updateAlarmForm(form, si);

		return true;

	case popSelectEvent:
		si = alarmListSensor[e->data.popSelect.selection];
		updateAlarmForm(form, si);
		return true;


	case frmCloseEvent:
		// if user switcher application when this form is open
		// post frmCloseEvent to caller form as it will get it
		// a chance to free resources
		FrmReturnToForm(0);
		return true;

	case ctlRepeatEvent:
		switch(e->data.ctlRepeat.controlID) {
		case threshSliderId:
			_pnlSensor[si].threshold = e->data.ctlRepeat.value;
			updateAlarmThreshold(form, si);
			return false;
		}
		break;

	case ctlSelectEvent:
		switch(e->data.ctlSelect.controlID) {
		case okButtonId:
			FrmReturnToForm(0);
			return true;

		case sound0ButtonId:
		case sound1ButtonId:
		case sound2ButtonId:
		case sound3ButtonId:
		case sound4ButtonId:
			_pnlSensor[si].sound = e->data.ctlSelect.controlID - sound0ButtonId;
			updateAlarmList(form);
			if((sndCmd.param1 = alarmFreq[_pnlSensor[si].sound]))
				SndDoCmd(NULL, &sndCmd, true);
			return true;

		case threshSignButtonId:
			{
				UInt16 index = FrmGetObjectIndex(form, threshSignButtonId);
				ControlType* control = FrmGetObjectPtr(form, index);
				const Char *label = CtlGetLabel(control);
				_pnlSensor[si].polarity = *label == '<' ? 0 : 1;
				CtlSetLabel(control, *label == '<' ? ">" : "<");
			}
			return true;
		}
		return false;

	default:
		break;
	}

	return false;
}

/*
 *  e d i t
 */

/*
 *  update form according to current mode
 */

Boolean switchMode(UInt8 newMode)
{
	FormType *form = FrmGetActiveForm();
	
	UInt16 pauseCheckIndex = FrmGetObjectIndex(form, pauseCheckId);
	UInt16 logCheckIndex = FrmGetObjectIndex(form, logCheckId);
	UInt16 graphCheckIndex = FrmGetObjectIndex(form, graphCheckId);
	UInt16 timeLabelIndex = FrmGetObjectIndex(form, timeLabelId);
	UInt16 scrollBarIndex = FrmGetObjectIndex(form, scrollBarId);

	if(newMode == currentMode) return false;

	closePort();
	{	// erase graph
		RectangleType r = { { 0, graphY - 14}, { 160, 1 } };
		WinEraseRectangle(&r, 0);
		r.topLeft.y = graphY - 12;
		r.extent.y = 11;
		WinEraseRectangle(&r, 0);
	}

	RefreshPanel();
	PnlUpdate(&panel, &currentSample);

	switch(newMode) {

	case REVIEW_MODE:
		currentMode = REVIEW_MODE;

		PnlSetMode(&panel, PANEL_MULTISELECT);
		PnlShowPanel(&panel);

		// hide Graph, Log and Pause checkboxen
		FrmHideObject(form, pauseCheckIndex);
		FrmHideObject(form, logCheckIndex);
		FrmHideObject(form, graphCheckIndex);
		FrmShowObject(form, timeLabelIndex);

		// show graph
		GrfSetMode(&graph, 0);
		GrfShowCursor(&graph, 1);
		GrfUpdateGraph(&graph);

		// show graph scroller
		_setScroller(graph.position);
		FrmShowObject(form, scrollBarIndex);

		break;

	case MONITOR_MODE:
		currentMode = MONITOR_MODE;

		PnlSetMode(&panel, numericMode ? PANEL_SINGLESELECT : PANEL_MULTISELECT);
		PnlShowPanel(&panel);

		FrmHideObject(form, timeLabelIndex);
		FrmHideObject(form, scrollBarIndex);

		// set and show Pause checkbox
		FrmSetControlValue(form, pauseCheckIndex, 1);
		FrmSetControlValue(form, graphCheckIndex, !numericMode);
		FrmShowObject(form, pauseCheckIndex);
		FrmShowObject(form, graphCheckIndex);
		FrmShowObject(form, logCheckIndex);


		// hide graph
		GrfSetMode(&graph, !FrmGetControlValue(form, graphCheckIndex));
		GrfEraseGraph(&graph);
		GrfShowCursor(&graph, 0);
		GrfAppendSample(&graph, 0);

		break;
	}

	return true;
}

Boolean editFormHandleEvent(EventType *e)
{
	Err err;
	FormType *form = FrmGetActiveForm();

	// give a chance to mode-specific handlers first
	if(currentMode == REVIEW_MODE && reviewHandleEvent(e)) return true;
	if(currentMode == MONITOR_MODE && monitorHandleEvent(e)) return true;
	// if serial comm in progress - block all system events
	// otherwise just block hardware buttons 1..4
	if(!talkingECU && !(
		e->eType == keyDownEvent && 
			(
			e->data.keyDown.chr == hard1Chr ||
			e->data.keyDown.chr == hard2Chr ||
			e->data.keyDown.chr == hard3Chr ||
			e->data.keyDown.chr == hard4Chr
			)
		) && (SysHandleEvent(e) || MenuHandleEvent(NULL, e, &err))) return true;

	// should be the last as it interfere with MenuHandleEvent
	if(PnlHandleEvent(&panel, e)) return true;

	switch(e->eType) {

	case frmOpenEvent:
		{
			FrmSetTitle(form, editFormTitle);
			FrmDrawForm(form);

			// TODO: move to resource some day
			graphX = 0;
			graphY = 70;
			graphW = 160;
			graphH = 64;

			// streamName was filled by main form
			stream = FileOpen(0, streamName, 0, CRID, fileModeUpdate, NULL);

			/* Insert SP 06/01/03 - default Hide Unused setting if off */
			FrmSetControlValue(form, FrmGetObjectIndex(form, hideUnusedButtonId), fHideUnused);
			/* End Insert */

			// panel
			PnlCreatePanel(&panel, panelId, NULL, &sensorPosition[currentBank * 8], selectedSensor, 0, 16, 160, 10 * 4);

			// create and paint the graph
			GrfCreateGraph(&graph, graphId, graphX, graphY, graphW, graphH, streamReader);
			// TODO: set graph without painting it
			GrfSetGraph(&graph, 0, 0, 0, graphSensors);
			GrfSetNumericMask(&graph, captureSensors);
			GrfSetSensor(&graph, selectedSensor);

			// hilite current bank
			FrmSetControlGroupSelection(form, bankGroupId, bank1ButtonId + currentBank);

			// hilite current mode
			FrmSetControlGroupSelection(form, modeGroupId, reviewButtonId + currentMode);

			WinDrawGrayLine(graphX, graphY - 13, graphX + graphW - 1, graphY - 13);
			WinDrawGrayLine(graphX, graphY - 1, graphX + graphW - 1, graphY - 1);
			WinDrawGrayLine(graphX, graphY + graphH, graphX + graphW - 1, graphY + graphH);

			// paint the form according to current mode
			switchMode(currentMode++);
		}
		return true;

	case frmCloseEvent:
		numericMode = graph._numericMode;
		GrfDestroyGraph(&graph);
		PnlDestroyPanel(&panel);
		{
			Int32 streamSize;
			Err err;

			FileTell(stream, &streamSize, &err);
			FileClose(stream);
			// delete log if empty
			if(streamSize == 0) FileDelete(0, streamName);
		}
		
		return false;

	case pnlSelectEvent:
		{
			PanelEventType *event = (PanelEventType *)e;

			if(graphSensors != event->data.pnlSelect.graph)
				GrfSetMask(&graph, event->data.pnlSelect.graph);

			if(captureSensors != event->data.pnlSelect.capture)
				GrfSetNumericMask(&graph, event->data.pnlSelect.capture);

			if(selectedSensor != event->data.pnlSelect.selection);
				GrfSetSensor(&graph, event->data.pnlSelect.selection);

			graphSensors = event->data.pnlSelect.graph;
			captureSensors = event->data.pnlSelect.capture;
			selectedSensor = event->data.pnlSelect.selection;
		}
		return true;

	case keyDownEvent:
		if(e->data.keyDown.modifiers & autoRepeatKeyMask) break;

		// decrement current bank
		if(e->data.keyDown.chr == pageUpChr) {
//			SndPlaySystemSound(sndClick);
			currentBank = (currentBank - 1) & 3;
			FrmSetControlGroupSelection(form, bankGroupId, bank1ButtonId + currentBank);
			RefreshPanel();
			return true;
		}
		// cincrement current bank
		if(e->data.keyDown.chr == pageDownChr) {
//			SndPlaySystemSound(sndClick);
			currentBank = (currentBank + 1) & 3;
			FrmSetControlGroupSelection(form, bankGroupId, bank1ButtonId + currentBank);
			RefreshPanel();
			return true;
		}
		break;

	case ctlSelectEvent:
		switch(e->data.ctlSelect.controlID) {

		case doneButtonId:
			FrmGotoForm(mainFormId);
			return true;

		case bank1ButtonId:
		case bank2ButtonId:
		case bank3ButtonId:
		case bank4ButtonId:
		case bank5ButtonId:
			currentBank = (FrmGetControlGroupSelection(form, bankGroupId) - 1) & 3;
			/* Insert SP 06/01/03 */
			RefreshPanel();
			/* End Insert */
			return true;

		case reviewButtonId:
			switchMode(REVIEW_MODE);
			return true;

		case monitorButtonId:
			switchMode(MONITOR_MODE);
			return true;

		}
    		break;

	case menuEvent:
		switch (e->data.menu.itemID) {

		case alarmsMenuId:
			FrmPopupForm(alarmsFormId);
			return true;

		case prefsMenuId:
			doPrefsDialog();
			PnlSetBank(&panel, &sensorPosition[currentBank * 8]);
			panel.graph = graphSensors;
			panel.capture = captureSensors;
			GrfSetMask(&graph, panel.graph);
			GrfSetNumericMask(&graph, panel.capture);
			return true;

		case aboutMenuId:
			doAboutDialog();
			return true;

		case renameLogMenuId:
			{
				FormType *dialog = FrmInitForm(renameLogFormId);
				UInt16 fieldIndex = FrmGetObjectIndex(dialog, newNameFieldId);
				Int16 checkIndex = FrmGetObjectIndex(dialog, backupLogCheckId);
				FieldType *field = FrmGetObjectPtr(dialog, fieldIndex);

				// paste current log name into the field
				FldInsert(field, streamName, StrLen(streamName));
				// select field contents, so it would be deleted if user starts typing
				FldSetSelection(field, 0, StrLen(streamName));
				FrmSetFocus(dialog, fieldIndex);
				FrmSetControlValue(dialog, checkIndex, (streamAttr & dmHdrAttrBackup) ? 1 : 0);

				// show dialog
				if(FrmDoDialog(dialog) == okButtonId) {
					const Char *newName = FldGetTextPtr(field);
					Boolean streamBackup = FrmGetControlValue(dialog, checkIndex);

					// file can't have no name - don't even try
					if(StrLen(newName) <= 0)
						newName = streamName;

					{
						DmOpenRef dbRef = NULL;
						LocalID dbId = 0;
						Int32 size = sizeof(DmOpenRef);
						UInt16 newAttr = dmHdrAttrStream | (streamBackup ? dmHdrAttrBackup : 0);

						(void)(0
						|| FileControl(fileOpGetOpenDbRef, stream, &dbRef, &size)
						|| DmOpenDatabaseInfo(dbRef, &dbId, NULL, NULL, NULL, NULL)
						|| DmSetDatabaseInfo(0, dbId, StrCompare(streamName, newName) ? newName : NULL, &newAttr, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL)
						|| StrCopy(streamName, newName)
						|| (streamAttr = newAttr)
						);
					}
				}

				FrmDeleteForm(dialog);
			}
			return true;

		case newLogMenuId:
			{
				if(FrmAlert(newLogAlertId) == 0) {
				}
			}
			return true;

		case deleteLogMenuId:
			if(FrmAlert(deleteLogAlertId) == 0) {
				// just truncate the log and switch to main
				// form. frmCloseEvent handler will delete
				// the file...
				FileSeek(stream, 0, fileOriginBeginning);
				FileTruncate(stream, 0);
				FrmGotoForm(mainFormId);
			}
			return true;

		case truncateLogMenuId:
			if(FrmAlert(truncateLogAlertId) == 0) {
				FileSeek(stream, 0, fileOriginBeginning);
				FileTruncate(stream, 0);
				GrfUpdateGraph(&graph);
				_setScroller(graph.position);
			}
			return true;

		default:
			/* let the system handle it */
			break;
		}
		break;

	default:
		break;
	}

	return false;
}

/*
 *  m a i n
 */

struct DirEntry {
	Char name[32];
	UInt16 attr;
};

struct DirEntry *dir;
Int16 dirSize;

#define ITEM_HEIGHT 11
#define ITEM_COUNT 11

Int16 compare(void *p1, void *p2, Int32 length)
{
	Char* s1 = ((struct DirEntry *)p1)->name;
	Char* s2 = ((struct DirEntry *)p2)->name;
	Int16 r;

	while(*s2) if((r = *s1++ - *s2++)) return r;
	return *s1;

	// couldn't believe it but PalmOS prior to 4.0 have no
	// plain strcmp! StrComapare is doing something obscure...

	/* return StrCompare((Char *)p1, (Char *)p2); */
}

void loadDirectory()
{
	Boolean newSearch = true;
	DmSearchStateType state;
	UInt16 cardNo;
	LocalID dbID;
	Int16 i;

	dirSize = 16;
	dir = MemPtrNew(dirSize * sizeof(struct DirEntry));

	i = 0;
	while(errNone == DmGetNextDatabaseByTypeCreator(newSearch, &state, sysFileTFileStream, CRID, false, &cardNo, &dbID)) {
		if(errNone == DmDatabaseInfo(cardNo, dbID,
			dir[i].name, &dir[i].attr /* attributesP */,
			NULL /* versionP */, NULL /* cdDateP */,
			NULL /* modDateP */, NULL /*bckUpDateP */,
			NULL /* modNumP */, NULL /* appInfoIDP */,
			NULL /* sortInfoIDP */,
			NULL /* *typeP */, NULL /* creatorP */))
		{
			if(++i >= dirSize) {
				void *p = MemPtrNew((dirSize + 16) * sizeof(struct DirEntry));
				MemMove(p, dir, dirSize * sizeof(struct DirEntry));
				MemPtrFree(dir);
				dir = p;
				dirSize += 16;
			}
		}

		newSearch = false;
	}

	dirSize = i;
	SysInsertionSort(dir, dirSize, sizeof(struct DirEntry), compare, 32);
}

void mainDrawItem(UInt16 i, Boolean highlight)
{
	if(i < topVisible + ITEM_HEIGHT) {
		UInt16 strWidth, strLen;
		Char str[32];
		RectangleType clip, rect = { { 0, 18 + i * ITEM_HEIGHT }, { 151, 11 } };

		WinPushDrawState();
		
		if(highlight) {
			WinSetBackColor(UIColorGetTableEntryIndex(UIObjectSelectedFill));
			WinSetTextColor(UIColorGetTableEntryIndex(UIObjectSelectedForeground));
		} else {
			WinSetBackColor(UIColorGetTableEntryIndex(UIObjectFill));
			WinSetTextColor(UIColorGetTableEntryIndex(UIObjectForeground));
		}

		WinEraseRectangle(&rect, 0);
		WinGetClip(&clip);
		WinSetClip(&rect);

		strLen = StrPrintF(str, "%d.", topVisible + i + 1);
		strWidth = FntCharsWidth(str, strLen);
		WinDrawChars(str, strLen, 14 - strWidth, 18 + i * ITEM_HEIGHT);

		if(dir[topVisible + i].attr & dmHdrAttrBackup)
			WinDrawChars("\225", 1, 146, 18 + i * ITEM_HEIGHT);

		rect.extent.x -= 6;
		WinSetClip(&rect);

		WinDrawChars(dir[topVisible + i].name, StrLen(dir[topVisible + i].name), 18, 18 + i * ITEM_HEIGHT);

		WinSetClip(&clip);
		WinPopDrawState();
	}
}

Boolean mainFormHandleEvent(EventType *e)
{
	FormType *form = FrmGetActiveForm();
	static Int16 hilitedItem;
	Int16 i;
	Err err;
	ScrollBarType* bar = FrmGetObjectPtr(form, FrmGetObjectIndex(form, mainScrollbarId));

	if(SysHandleEvent(e) || MenuHandleEvent(NULL, e, &err)) return true;
	
	switch(e->eType) {

	case frmOpenEvent:

		closePort();
		FrmDrawForm(form);
		hilitedItem = -1;

		loadDirectory();
		if(topVisible + ITEM_COUNT > dirSize) topVisible = MAX(0, dirSize - ITEM_COUNT); 

		for(i = 0; i < MIN(dirSize, ITEM_COUNT); i++)
			mainDrawItem(i, false);
		
		SclSetScrollBar(bar, topVisible, 0, MAX(0, dirSize - ITEM_COUNT), ITEM_COUNT);

		return true;

	case frmCloseEvent:
		MemPtrFree(dir);
		return false;

	case sclRepeatEvent:
		topVisible = e->data.sclRepeat.newValue;
		
		for(i = 0; i < MIN(dirSize, ITEM_COUNT); i++)
			mainDrawItem(i, false);

		return false;
	
	case penDownEvent:
		if(e->screenX < 150 && e->screenY >= 18 && e->screenY <= 18 + ITEM_COUNT * ITEM_HEIGHT) {
			i = (e->screenY - 18) / ITEM_HEIGHT;
			if(i >= 0 && i < dirSize) {
				mainDrawItem(i, true);
				hilitedItem = i;
	                	return true;
			}
		}
		return false;

	case penMoveEvent:
		i = (e->screenY - 18) / ITEM_HEIGHT;
		if(hilitedItem >= 0 && i != hilitedItem) {
			mainDrawItem(hilitedItem, false);
			hilitedItem = -1;
                	return true;
		}
		return false;

	case penUpEvent:
		i = (e->screenY - 18) / ITEM_HEIGHT;
		if(hilitedItem >= 0 && i == hilitedItem) {
			StrCopy(streamName, dir[topVisible + i].name);
			streamAttr = dir[topVisible + i].attr;
			StrPrintF(editFormTitle, "MMCd Log %d of %d", i + 1 + topVisible, dirSize);
			FrmGotoForm(editFormId);
			return true;
		} else if(hilitedItem >= 0) {
			mainDrawItem(hilitedItem, false);
			hilitedItem = -1;
		}
		
		return false;

	case ctlSelectEvent:
		if(e->data.ctlSelect.controlID == newButtonId) {
			DateTimeType date;

			TimSecondsToDateTime(TimGetSeconds(), &date);
			StrPrintF(streamName, "%04d-%02d-%02d %02d:%02d:%02d", date.year, date.month, date.day, date.hour, date.minute, date.second);
			streamAttr = dmHdrAttrStream;
			StrPrintF(editFormTitle, "MMCd Log");
			FrmGotoForm(editFormId);
			return true;
		}
		return false;

	case menuEvent:
		switch (e->data.menu.itemID) {

		case alarmsMenuId:
			FrmPopupForm(alarmsFormId);
			return true;

		case aboutMenuId:
			doAboutDialog();
			return true;

		case testMenuId:
			FrmGotoForm(testFormId);
			return true;

		case debugMenuId:
			FrmGotoForm(debugFormId);
			return true;

		case prefsMenuId:
			doPrefsDialog();
			return true;

		default:
			break;

		}
	default:
		break;
	}

	return false;
}

/*
 *
 */

Boolean appHandleEvent(EventType *e)
{
	if(e->eType == frmLoadEvent) {
		// Load the form resource.
		UInt16 formID = e->data.frmLoad.formID;
		FormPtr form = FrmInitForm(formID);
		FrmSetActiveForm(form);

		switch(formID) {

		case mainFormId:
			FrmSetEventHandler(form, mainFormHandleEvent);
			break;

		case editFormId:
			FrmSetEventHandler(form, editFormHandleEvent);
			break;

		case testFormId:
			FrmSetEventHandler(form, testFormHandleEvent);
			break;

		case debugFormId:
			FrmSetEventHandler(form, debugFormHandleEvent);
			break;

		case alarmsFormId:
			FrmSetEventHandler(form, alarmsFormHandleEvent);
			break;

		default:
			ErrFatalDisplayIf(1, "Unknown form!");
			break;
		}
	}
	return false;
}

void loadPrefs()
{
	Prefs prefs = { 0, 0, 0, 0 };
	UInt16 size = sizeof(Prefs);
	/* Insert SP 06/01/03 - used for pref size check */
	UInt16 chksize = 0;
	/* End Insert */
	UInt32 i;
	/* Modified SP 06/01/03 - default error code */
	Err err = !errNone;
	/* End Modified */

	// initialize sensors
	MemMove(_pnlSensor, _pnlSensorDefault, sizeof(_pnlSensor));

	/* Insert SP 06/01/03 - check size of preferences before reading */
	PrefGetAppPreferences(CRID, 0, NULL, &chksize, false);
	if (chksize == size) err = PrefGetAppPreferences(CRID, 0, &prefs, &size, false);
	/* End Insert */
	/* Modified SP 06/01/03 - change error check */
	if (err != errNone) {
	/* End Modified */
		PrefSetAppPreferences(CRID, 0, 0, NULL, 0, false);

		graphSensors = 0;
		captureSensors = 0;
		currentBank = 0;
		currentMode = 0;
		topVisible = 0;
		metricUnit = PREFS_UNIT_METRIC;
		serialSpeed = ECU_BAUDRATE;
		mapScaling = 0;

		audibleAlarms = true;
		numericMode = 0;
		selectedSensor = 0;
		/* Insert SP 06/01/03 */
		fHideUnused = false;
		/* End Insert */
		audibleAlarms = true;
		mafHz = PREFS_UNIT_MAF16;
		carSel = PREFS_UNIT_CARDSM;

		for(i = 0; i < SENSOR_COUNT; i++) {
			if(_pnlSensor[i].capture) captureSensors |= (1L << i);
			if(_pnlSensor[i].graph) graphSensors |= (1L << i);
		}
	} else {
		for(i = 0; i < SENSOR_COUNT; i++) {
			_pnlSensor[i].capture = !!(prefs.capture & (1L << i));
			_pnlSensor[i].graph = !!(prefs.graph & (1L << i));
			_pnlSensor[i].exists = !!(prefs.exists & (1L << i));
			_pnlSensor[i].polarity = !!(prefs.polarity & (1L << i));
			_pnlSensor[i].addr = prefs.addr[i];
			_pnlSensor[i].threshold = prefs.threshold[i];
			_pnlSensor[i].sound = prefs.sound[i];
			StrNCopy(_pnlSensor[i].slug, prefs.slug[i], 4);
		}
		graphSensors = prefs.graph;
		captureSensors = prefs.capture;
		currentBank = prefs.currentBank;
		currentMode = prefs.currentMode;
		topVisible = prefs.topVisible;
		serialSpeed = prefs.serialSpeed;
		metricUnit = prefs.metricUnit;
		mapScaling = prefs.mapScaling;
		mafHz = prefs.mafHz;
		carSel = prefs.carSel;
		audibleAlarms = prefs.audibleAlarms;
		selectedSensor = prefs.selectedSensor;
		fHideUnused = prefs.hideUnused;
		numericMode = prefs.numericMode;
		audibleAlarms = prefs.audibleAlarms;
	}
}

void savePrefs()
{
	UInt16 i;
	Prefs prefs;

	prefs.capture = captureSensors;
	prefs.graph = graphSensors;
	prefs.currentBank = currentBank;
	prefs.currentMode = currentMode;
	prefs.topVisible = topVisible;
	prefs.serialSpeed = serialSpeed;
	prefs.metricUnit = metricUnit;
	prefs.mapScaling = mapScaling;
	prefs.mafHz = mafHz;
	prefs.carSel = carSel;
	prefs.selectedSensor = selectedSensor;
	prefs.hideUnused = fHideUnused;
	prefs.numericMode = numericMode;
	prefs.audibleAlarms = audibleAlarms;

	prefs.exists = 0;
	prefs.polarity = 0;

	for(i = 0; i < SENSOR_COUNT; i++) {
		if(_pnlSensor[i].exists) prefs.exists |= (1L << i);
		prefs.addr[i] = _pnlSensor[i].addr;
		MemMove(prefs.slug[i], _pnlSensor[i].slug, 4);
		prefs.threshold[i] = _pnlSensor[i].threshold;
		if(_pnlSensor[i].polarity) prefs.polarity |= (1L << i);
		prefs.sound[i] = _pnlSensor[i].sound;
	}

	PrefSetAppPreferences(CRID, 0, 0, &prefs, sizeof(prefs), false);
}

UInt32 PilotMain(UInt16 cmd, void *pbp, UInt16 flags __attribute__ ((unused)))
{
	switch(cmd) {
	case sysAppLaunchCmdNormalLaunch:
		{
			EventType event;
			Boolean screenEnableColor;
			UInt32 version;

			// check if PalmOS version is 3.5 or newer
			if(errNone != FtrGet(sysFtrCreator, sysFtrNumROMVersion, &version) || version < 0x03503000) {
				FrmAlert(versionAlertId);
				return 0;
			}

			colorMode = false;
			screenDensity = 1;

			if(errNone == WinScreenMode(winScreenModeGet,
			   &screenWidth, &screenHeight, &screenDepth,
			   &screenEnableColor)) {
				if(screenDepth >= 8) colorMode = true;
			}

			if(errNone == FtrGet(sysFtrCreator, sysFtrNumWinVersion, &version) && version >= 4) {
				UInt32 attr;
				WinScreenGetAttribute(winScreenDensity, &attr);

				switch(attr) {
				case kDensityDouble:
					screenDensity = 2;
					break;
				case kDensityTriple:
					screenDensity = 3;
					break;
				case kDensityQuadruple:
					screenDensity = 4;
					break;
				}
			}

			// If on Handspring device, disable the keyboard
			// thread before opening the serial library.
			if(!FtrGet('hsEx', 0, &version))
				HsExtKeyboardEnable(false);


			ticksPerSecond = SysTicksPerSecond();

			loadPrefs();
			FrmGotoForm(mainFormId);

			bigFont = MemHandleLock(DmGetResource('NFNT', bigFontId));
			FntDefineFont(fntAppFontCustomBase, bigFont);

			talkingECU = false;
			portOpen = false;

			do {
				/* Modified SP 06/03/03 - Wait up to 1 second for events */
				/* reduced timeout to 1/50 sec, because null events are
				   used by graph widget to scroll diplay when pen
				   is dragged to the edge of the screen -/dmitry */
				EvtGetEvent(&event, talkingECU ? 0 : ticksPerSecond / 50);
				/* End Modified */
				(void) (
				        appHandleEvent(&event)
				     || FrmDispatchEvent(&event)
				);
			} while(event.eType != appStopEvent);

			FrmCloseAllForms();
			closePort();	// just in case
			savePrefs();
		}
		break;

	case sysAppLaunchCmdSystemReset: 
		break;

	case sysAppLaunchCmdAlarmTriggered:
		{
			SysAlarmTriggeredParamType *param = pbp;
			param->purgeAlarm = true;
		}
		break;
	}

	return 0;
}

