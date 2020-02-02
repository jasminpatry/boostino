// Boostino -- A Teensy 3.6/Tactrix OP 2.0 logger and gauge.
// Copyright (c) 2019, 2020 Jasmin Patry
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <Adafruit_GFX.h>
#include <font_Exo-BoldItalic.h>
#include <ILI9341_t3.h>
#include <SD.h>
#include <USBHost_t36.h>
#include <XPT2046_Touchscreen.h>

#include "gaugeBg.h"
#include "gaugeFg.h"
#include "labelMph.h"
#include "labelAfr.h"



// Set to 1 to enable verbose logging.

#define DEBUG 0

static const bool s_fTraceComms = false;
static const bool s_fTraceTouch = false;

// Set to 1 to test without Tactrix

#define TEST_NO_TACTRIX 0

// Set to 1 to test Tactrix without being plugged into the vehicle

#define TEST_OFFLINE (0 || TEST_NO_TACTRIX)



// Useful macros, typedefs, etc.

#define CASSERT(f) static_assert(f, #f)

template <typename T, int N>
constexpr int DIM(T (&aT)[N])
{
	(void)aT;
	return N;
}

typedef uint32_t	u32;
typedef int32_t		s32;
typedef uint16_t	u16;
typedef int16_t		s16;
typedef uint8_t		u8;
typedef int8_t		s8;

CASSERT(sizeof(int) == 4);		// Teensy 3.6 is 32-bit
CASSERT(sizeof(void *) == 4);	//	...

static const float s_gPi = 3.14159265f;



// Available ECU parameters

enum PARAM
{
	PARAM_FbkcDeg,
	PARAM_FlkcDeg,
	PARAM_BoostPsi,
	PARAM_Iam,
	PARAM_CoolantTempF,	// BB (jpatry) Currently unused
	PARAM_Rpm,
	PARAM_Maf,
	PARAM_LoadGPerRev,	// NOTE (jpatry) Calculated from PARAM_Rpm and PARAM_Maf
	PARAM_IatF,
	PARAM_ThrottlePct,
	PARAM_SpeedMph,
	PARAM_IpwMs,
	PARAM_IdcPct,		// NOTE (jpatry) calculated from PARAM_Rpm and PARAM_IpwMs
	PARAM_Afr1,
	PARAM_WidebandAfr,	// Read from analog input
	PARAM_TargetAfr,
	PARAM_MafVoltage,
	PARAM_LightSwitch,

	PARAM_Max,
	PARAM_Min = 0,
	PARAM_Nil
};

PARAM & operator++(PARAM & param)
{
	param = PARAM(param + 1);
	return param;
}

static const char * s_mpParamPChz[] =
{
	"Feedback Knock Correction (4-byte)* (degrees)",	// PARAM_FbkcDeg
	"Fine Learning Knock Correction (degrees)",			// PARAM_FlkcDeg
	"Manifold Relative Pressure (psi)",					// PARAM_BoostPsi
	"IAM (multiplier)",									// PARAM_Iam
	"Coolant Temperature (F)",							// PARAM_CoolantTempF
	"Engine Speed (rpm)",								// PARAM_Rpm
	"Mass Airflow (g/s)",								// PARAM_Maf
	"Engine Load (Calculated) (g/rev)",					// PARAM_LoadGPerRev
	"Intake Air Temperature (F)",						// PARAM_IatF
	"Throttle Opening Angle (%)",						// PARAM_ThrottlePct
	"Vehicle Speed (mph)",								// PARAM_SpeedMph
	"Fuel Injector #1 Pulse Width",						// PARAM_IpwMs
	"Injector Duty Cycle",								// PARAM_IdcPct
	"A/F Sensor #1",									// PARAM_Afr1
	"Wideband A/F Sensor",								// PARAM_WidebandAfr
	"Final Fueling Base (2-byte)* (estimated AFR)",		// PARAM_TargetAfr
	"Mass Airflow Sensor Voltage (V)",					// PARAM_MafVoltage
	"Light Switch",										// PARAM_LightSwitch
};
CASSERT(DIM(s_mpParamPChz) == PARAM_Max);

static const u8 s_cBSsmAddr = 3;
static const u32 s_mpParamABAddr[] =
{
	0xff81fc,		// PARAM_FbkcDeg
	0x000199,		// PARAM_FlkcDeg
	0x000024,		// PARAM_BoostPsi
	0x0000f9,		// PARAM_Iam
	0x000008,		// PARAM_CoolantTempF
	0x00000e,		// PARAM_Rpm
	0x000013,		// PARAM_Maf
	0x000000,		// PARAM_LoadGPerRev
	0x000012,		// PARAM_IatF
	0x000015,		// PARAM_ThrottlePct
	0x000010,		// PARAM_SpeedMph
	0x000020,		// PARAM_IpwMs
	0x000000,		// PARAM_IdcPct
	0x000046,		// PARAM_Afr1
	0x000000,		// PARAM_WidebandAfr
	0xff688a,		// PARAM_TargetAfr
	0x00001d,		// PARAM_MafVoltage
	0x000064,		// PARAM_LightSwitch
};
CASSERT(DIM(s_mpParamABAddr) == PARAM_Max);

static const u8 s_mpParamCB[] =
{
	 4,		// PARAM_FbkcDeg
	 1,		// PARAM_FlkcDeg
	 1,		// PARAM_BoostPsi
	 1,		// PARAM_Iam
	 1,		// PARAM_CoolantTempF
	 2,		// PARAM_Rpm
	 2,		// PARAM_Maf
	 0,		// PARAM_LoadGPerRev
	 1,		// PARAM_IatF
	 1,		// PARAM_ThrottlePct
	 1,		// PARAM_SpeedMph
	 1,		// PARAM_IpwMs
	 0,		// PARAM_IdcPct
	 1,		// PARAM_Afr1
	 0,		// PARAM_WidebandAfr
	 2,		// PARAM_TargetAfr
	 1,		// PARAM_MafVoltage
	 1,		// PARAM_LightSwitch
};
CASSERT(DIM(s_mpParamCB) == PARAM_Max);

// BB (jpatry) Learning Table value data, for future:
// { 0xff31d4, 4, "A/F Learning #1 A (Stored)*" }, // float, x*100, %
// { 0xff31dc, 4, "A/F Learning #1 B (Stored)*" }, // float, x*100, %
// { 0xff31e4, 4, "A/F Learning #1 C (Stored)*" }, // float, x*100, %
// { 0xff31ec, 4, "A/F Learning #1 D (Stored)*" }, // float, x*100, %
// { 0x0000d1, 1, "A/F Learning #3" }, // u8, (x-128)*100/128, %
// { 0x000023, 1, "Atmospheric Pressure" }, //u8,  x*37/255, psi
// { 0x00001c, 1, "Battery Voltage" }, // u8, x*8/100, V
// { 0x000008, 1, "Coolant Temperature" }, // u8, 32+9*(x-40)/5, F
// { 0xff8298, 4, "Fine Learning Knock Correction (4-byte)*" }, // float, x, degrees
// { 0xff329c, 4, "IAM (4-byte)*" }, // float, x, multiplier
// { 0x000012, 1, "Intake Air Temperature" }, // u8, 32+9*(x-40)/5, F
// { 0x0cce3c, 4, "A/F Learning #1 Airflow Ranges", 3 }, // float, x, g/s
// { 0x0d3dd8, 4 ,"Fine Correction Columns (Load)", 4 }, // float, x, g/rev
// { 0x0d3dbc, 4 ,"Fine Correction Rows (RPM)", 6 }, // float, x, RPM
// { 0xff32b0, 4 ,"Fine Correction Row 1 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM // NOTE (jpatry) Address is IAM (4-byte) + 4
// { 0xff32d8, 4 ,"Fine Correction Row 2 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM
// { 0xff3300, 4 ,"Fine Correction Row 3 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM
// { 0xff3328, 4 ,"Fine Correction Row 4 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM
// { 0xff3350, 4 ,"Fine Correction Row 5 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM
// { 0xff3378, 4 ,"Fine Correction Row 6 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM
// { 0xff33a0, 4 ,"Fine Correction Row 7 (degrees)", 5, 4 /*stride*/ }, // float, x, RPM



// ECU Parameters to Poll

static const PARAM s_aParamPoll[] =
{
	PARAM_FbkcDeg,		// 4 B
	PARAM_Rpm,			// 2 B
	PARAM_Maf,			// 2 B
	PARAM_FlkcDeg,		// 1 B
	PARAM_Iam,			// 1 B
	PARAM_BoostPsi,		// 1 B
	PARAM_IatF,			// 1 B
	PARAM_ThrottlePct,	// 1 B
	PARAM_SpeedMph,		// 1 B
	PARAM_TargetAfr,	// 2 B
	PARAM_MafVoltage,	// 1 B
	PARAM_LightSwitch,	// 1 B
};
static const u8 s_cParamPoll = DIM(s_aParamPoll);



// ECU Parameters to Log

static const PARAM s_aParamLog[] =
{
	PARAM_LoadGPerRev,
	PARAM_Rpm,
	PARAM_Maf,
	PARAM_FbkcDeg,
	PARAM_FlkcDeg,
	PARAM_BoostPsi,
	PARAM_IatF,
	PARAM_ThrottlePct,
	PARAM_Iam,
	PARAM_WidebandAfr,
	PARAM_TargetAfr,
	PARAM_MafVoltage,
};
static const u8 s_cParamLog = DIM(s_aParamLog);



// Debugging support

inline void Trace(bool f, const char * pChz)
{
#if DEBUG
	if (f)
		Serial.print(pChz);
#endif // DEBUG
}

inline void Trace(bool f, u8 b, int mode = DEC)
{
#if DEBUG
	if (f)
		Serial.print(b, mode);
#endif // DEBUG
}

#if DEBUG
void Trace(bool f, char ch)
{
	if (!f)
		return;

	if (isprint(ch))
	{
		Serial.print(ch);
	}
	else
	{
		switch (ch)
		{
		case '\r':
			Serial.print("\\r");
			break;

		case '\n':
			Serial.print("\\n");
			break;

		case '\0':
			Serial.print("\\0");
			break;

		default:
			Serial.print("\\x");
			Serial.print(u8(ch), HEX);
			break;
		}
	}
}
#else // !DEBUG
inline void Trace(bool f, char ch)
{
}
#endif // !DEBUG

inline void Trace(bool f, u16 uN, int mode = DEC)
{
#if DEBUG
	if (f)
		Serial.print(uN, mode);
#endif // DEBUG
}

inline void Trace(bool f, s16 n, int mode = DEC)
{
#if DEBUG
	if (f)
		Serial.print(n, mode);
#endif // DEBUG
}

inline void Trace(bool f, u32 uN, int mode = DEC)
{
#if DEBUG
	if (f)
		Serial.print(uN, mode);
#endif // DEBUG
}

inline void Trace(bool f, int n, int mode = DEC)
{
#if DEBUG
	if (f)
		Serial.print(n, mode);
#endif // DEBUG
}

inline void Trace(bool f, float g, int cDigit = 2)
{
#if DEBUG
	if (f)
		Serial.print(g, cDigit);
#endif // DEBUG
}

#if DEBUG
void TraceHex(bool f, const u8 * aB, u16 cB)
{
	if (!f || cB == 0)
		return;

	int iB = 0;
	for (;;)
	{
		u8 b = aB[iB];
		if (b < 0x10)
			Trace(true, '0');
		Trace(true, b, HEX);

		++iB;
		if (iB >= cB)
			break;

		Trace(true, ' ');

		if (!(iB & 0xf))
			Trace(true, "\n");
		else if (!(iB & 0x3))
			Trace(true, "| ");
	}
	Trace(true, "\n");
}
#else // !DEBUG
inline void TraceHex(bool f, const u8 * aB, u16 cB)
{
}
#endif // !DEBUG



// BB (jpatry) Assigning __LINE__ to nAssertLine is necessary to get rid of warning about 'large integer implicitly
//	truncated to unsigned type'.

#define ASSERT(f)								\
	do											\
	{											\
		if (!(f))								\
		{										\
			Trace(true, "Assert failed (");		\
			Trace(true, __FILE__);				\
			Trace(true, ":");					\
			u32 nAssertLine = __LINE__;			\
			Trace(true, nAssertLine);			\
			Trace(true, "): ");					\
			Trace(true, #f);					\
			Trace(true, "\n");					\
		}										\
	} while(0)



// NOTE (jpatry) J2534 protocol reverse-engineered using Wireshark, with help from this reference:
//	https://github.com/NikolaKozina/j2534/blob/master/j2534.c . This fork (by one of the RomRaider devs) looks to be
//	more up to date: https://github.com/dschultzca/j2534/blob/master/j2534.c

// J2534 Message kinds

enum MSGK
{
	MSGK_InfoReply,			// "ari" reply to "ati"
	MSGK_LoopbackStart,		// "ar" 0xa0 reply to "att"
	MSGK_Loopback,			// "ar" 0x20 reply to "att"
	MSGK_LoopbackEnd,		// "ar" 0x60 reply to "att"
	MSGK_ReplyStart,		// "ar" 0x80 reply to "att"
	MSGK_Reply,				// "ar" 0x00 reply to "att"
	MSGK_ReplyEnd,			// "ar" 0x40 reply to "att"

	MSGK_Max,
	MSGK_Min = 0,
	MSGK_Nil = -1
};

// BB (jpatry) why doesn't this work? Compiler says "MSGK does not name a type".
// MSGK & operator++(MSGK & msgk)
// {
// 	msgk = MSGK(msgk + 1);
// 	return msgk;
// }

static const char * s_mpMsgkPChz[] =
{
	"InfoReply",				// MSGK_InfoReply
	"LoopbackStart",			// MSGK_LoopbackStart
	"Loopback",					// MSGK_Loopback
	"LoopbackEnd",				// MSGK_LoopbackEnd
	"ReplyStart",				// MSGK_ReplyStart
	"Reply",					// MSGK_Reply
	"ReplyEnd",					// MSGK_ReplyEnd
};
CASSERT(DIM(s_mpMsgkPChz) == MSGK_Max);

static const u8 s_mpMsgkBType[] =
{
	0xff,				// MSGK_InfoReply (no type byte)
	0xa0,				// MSGK_LoopbackStart
	0x20,				// MSGK_Loopback
	0x60,				// MSGK_LoopbackEnd
	0x80,				// MSGK_ReplyStart
	0x00,				// MSGK_Reply
	0x40,				// MSGK_ReplyEnd
};
CASSERT(DIM(s_mpMsgkBType) == MSGK_Max);



// Structure of MSGK_LoopbackStart to MSGK_ReplyEnd inclusive

struct SJ2534Ar	// tag = ar
{
	u8	m_aBHeader[2];			// "ar"
	u8	m_nProtocol;			// E.g. '3' for ISO9141
	u8	m_cBPlus1;				// length including m_bType
	u8	m_bType;				// s_mpMsgkBType[msgk]
	u8	m_aB[0];				// Payload

	u8			CBPayload() const
					{ return m_cBPlus1 - 1; }

	const u8 *	PBPayload() const
					{ return m_aB; }
};



// Subaru Select Monitor (SSM) protocol description: http://www.romraider.com/RomRaider/SsmProtocol
// Max SSM packet size is ~250 bytes

static const u8 s_cBSsmMax = 250;



// SSM Source/Destination IDs (ISO9141)

enum SSMID : u8
{
	SSMID_Ecu	  = 0x10,
	SSMID_TcuDccd = 0x18,
	SSMID_Tool	  = 0xf0,
};



// SSM Command/Response IDs

enum SSMCMD : u8
{
	SSMCMD_BlockReadRequest	   = 0xa0,
	SSMCMD_AddressReadRequest  = 0xa8,
	SSMCMD_AddressReadResponse = 0xe8,
	SSMCMD_WriteBlockRequest   = 0xb0,
	SSMCMD_AddressWriteRequest = 0xb8,
	SSMCMD_EcuInitRequest	   = 0xbf,
	SSMCMD_EcuInitResponse	   = 0xff,
};



u8 BSsmChecksum(const u8 * aB, u16 cB)
{
	u8 bSum = 0;
	for (int iB = 0; iB < cB; ++iB)
		bSum += aB[iB];

	return bSum;
}



// SSM Packet

struct SSsm	// tag = ssm
{
	u8		m_bHdr;				// 0x80
	SSMID	m_bDst;				// Destination
	SSMID	m_bSrc;				// Source
	u8		m_cB;				// Data + checksum size
	SSMCMD	m_bCmd;				// Command/response byte
	u8		m_aB[0];			// Data (including checksum as last byte)

	u8			CBData() const
					{ return m_cB - 1; }

	const u8 *	PBData() const
					{ return m_aB; }

	u8			BChecksum() const
					{ return m_aB[CBData()]; }

	bool		FVerifyChecksum() const;
};

bool SSsm::FVerifyChecksum() const
{
	const u8 * aB = (const u8 *)this;
	int cB = m_cB + 4;
	u8 bChecksum = BSsmChecksum(aB, cB);
	if (bChecksum == BChecksum())
	{
		return true;
	}
	else
	{
		Trace(true, "SSM checksum error\n");
		return false;
	}
}



// Log entry

struct SLogEntry	// tag = logent
{
	u32		m_msTimestamp;
	float	m_mpIParamLogGValue[s_cParamLog];
};

// Number of entries in log history

static const int s_cEntryLogHistory = 32;

// Circular buffer of data to log, with capacity s_cEntryLogHistory

class CLogHistory
{
public:
	u8					CEntry() const
							{ return m_iaGWrite - m_iaGRead; }
	bool				FIsEmpty() const
							{ return m_iaGRead == m_iaGWrite; }
	bool				FIsFull() const
							{ return CEntry() == s_cEntryLogHistory; }
	const SLogEntry &	LogentRead();
	void				WriteLogEntry(const SLogEntry & logent);

protected:
	// Implementation based on https://www.snellman.net/blog/archive/2016-12-13-ring-buffers/

	CASSERT(s_cEntryLogHistory <= u8(~0));
	CASSERT((s_cEntryLogHistory & (s_cEntryLogHistory - 1)) == 0);

	u8				IMask(u8 i)
						{ return i & (s_cEntryLogHistory - 1); }

	SLogEntry		m_aLogent[s_cEntryLogHistory];
	u8				m_iaGRead;						// unmasked read index
	u8				m_iaGWrite;						// unmasked write index
};

const SLogEntry & CLogHistory::LogentRead()
{
	ASSERT(!FIsEmpty());
	return m_aLogent[IMask(m_iaGRead++)];
}

void CLogHistory::WriteLogEntry(const SLogEntry & logent)
{
	// Allow writing to full buffer by discarding oldest entry

	if (FIsFull())
		(void) LogentRead();

	m_aLogent[IMask(m_iaGWrite++)] = logent;
}

CLogHistory g_loghist;



// USB host configuration

USBHost g_uhost;
USBSerial g_userial(g_uhost);
bool g_fUserialActive = false;

static const u16 s_nIdTactrix = 0x0403;
static const u16 s_nIdOpenPort20 = 0xcc4c;
static const u32 s_dMsTimeoutDefault = 2000;



// Wideband analog input

static const int s_cBitAnalog = 13;
static const float s_rAnalog = 1.0f / ((1 << s_cBitAnalog) - 1);



// Tactrix State

enum TACTRIXS
{
	TACTRIXS_Disconnected,
	TACTRIXS_Connected,
	TACTRIXS_Polling,
};

class CTactrix
{
public:
					CTactrix()
					: m_tactrixs(TACTRIXS_Disconnected),
					  m_msPollingStart(0),
					  m_aBRecv(),
					  m_cBRecv(0),
					  m_iBRecv(0),
					  m_iBRecvPrev(0),
					  m_cBParamPoll(0),
					  m_mpParamGValue()
						{ ; }

					~CTactrix()
						{ ; }

	bool			FTryConnect();
	void			Disconnect();
	bool			FTryStartPolling();
	bool			FTryUpdatePolling();
	TACTRIXS		Tactrixs() const
						{ return m_tactrixs; }
	float			GParam(PARAM param) const;

protected:
	void			SendCommand(const u8 * aB, u16 cB);
	void			SendCommand(const char * pChz);

	int				CBReceive(u32 dMsTimeout);
	void			ResetReceive();
	bool			FTryReceiveMessage(const char * pChz, u32 dMsTimeout = s_dMsTimeoutDefault);
	bool			FMustReceiveMessage(const char * pChz, u32 dMsTimeout = s_dMsTimeoutDefault);
	int				CBMessage()
						{ return m_iBRecv - m_iBRecvPrev; }
	u8 *			PBMessage()
						{ return &m_aBRecv[m_iBRecvPrev]; }
	int				CBRemaining()
						{ return m_cBRecv - m_iBRecv; }
	u8 *			PBRemaining()
						{ return &m_aBRecv[m_iBRecv]; }

	bool			FTryReceiveMessage(MSGK msgk, u32 dMsTimeout = s_dMsTimeoutDefault);
	bool			FMustReceiveMessage(MSGK msgk, u32 dMsTimeout = s_dMsTimeoutDefault);
	bool			FTrySkipMessage(u32 dMsTimeout = s_dMsTimeoutDefault);

	bool			FTryIssuePollRequest();

	void			FlushIncoming();

	void			ProcessParamValue(PARAM param, u32 nValue);

	TACTRIXS		m_tactrixs;						// Current state
	u32				m_msPollingStart;				// Timestamp when polling started
	u8				m_aBRecv[4096];					// Receive buffer
	int				m_cBRecv;						// Total bytes in receive buffer
	int				m_iBRecv;						// Index of remaining data in receive buffer
	int				m_iBRecvPrev;					// Index of last message in receive buffer
	int				m_cBParamPoll;					// Total number of bytes (addresses) being polled
	float			m_mpParamGValue[PARAM_Max];		// Latest values obtained from ECU
};

void CTactrix::SendCommand(const u8 * aB, u16 cB)
{
	if (!g_userial)
	{
		Trace(true, "SendCommand: Can't send, not connected\n");
		m_tactrixs = TACTRIXS_Disconnected;
		return;
	}

	Trace(s_fTraceComms, "Sending ");
	Trace(s_fTraceComms, cB);
	Trace(s_fTraceComms, " bytes:\n\"");

	for (int iB = 0; iB < cB; ++iB)
	{
		u8 b = aB[iB];

		Trace(s_fTraceComms, char(b));

		g_userial.write(b);
	}

	Trace(s_fTraceComms, "\"\n");

	TraceHex(s_fTraceComms, aB, cB);
}

void CTactrix::SendCommand(const char * pChz)
{
	SendCommand((const u8 *)pChz, strlen(pChz));
}

int CTactrix::CBReceive(u32 dMsTimeout)
{
	ResetReceive();

	if (!g_userial)
	{
		Trace(true, "CBReceive: Can't receive, no serial USB device connected\n");
		m_tactrixs = TACTRIXS_Disconnected;
		return 0;
	}

	u32 msStart = millis();
	for (;;)
	{
		m_cBRecv = g_userial.readBytes(m_aBRecv, sizeof(m_aBRecv));
		if (m_cBRecv > 0)
		{
#if DEBUG
			Trace(s_fTraceComms, "Received ");
			Trace(s_fTraceComms, m_cBRecv);
			Trace(s_fTraceComms, " bytes:\n\"");
			for (int iB = 0; iB < m_cBRecv; ++iB)
				Trace(s_fTraceComms, char(m_aBRecv[iB]));
			Trace(s_fTraceComms, "\"\n");

			TraceHex(s_fTraceComms, m_aBRecv, m_cBRecv);
#endif // DEBUG

			return m_cBRecv;
		}

		if (dMsTimeout > 0 && millis() - msStart > dMsTimeout)
			return 0;

		delayMicroseconds(100);
	}
}

void CTactrix::ResetReceive()
{
	m_cBRecv = 0;
	m_iBRecv = 0;
	m_iBRecvPrev = 0;
}

bool CTactrix::FTryReceiveMessage(const char * pChz, u32 dMsTimeout)
{
	int cCh = strlen(pChz);

	if (CBRemaining() == 0)
	{
		if (CBReceive(dMsTimeout) == 0)
		{
			Trace(true, "FTryReceiveMessage timed out waiting for reply \"");
			for (int iCh = 0; iCh < cCh; ++iCh)
				Trace(true, pChz[iCh]);
			Trace(true, "\"\n");
			return false;
		}
	}

	if (CBRemaining() < cCh ||
		strncmp((const char *)PBRemaining(), pChz, cCh) != 0)
	{
		return false;
	}

	m_iBRecvPrev = m_iBRecv;
	m_iBRecv += cCh;

	return true;
}

bool CTactrix::FMustReceiveMessage(const char * pChz, u32 dMsTimeout)
{
	if (!FTryReceiveMessage(pChz, dMsTimeout))
	{
		ResetReceive();

#if DEBUG
		if (CBRemaining())
		{
			Trace(true, "FMustReceiveMessage got unexpected reply (expected \"");
			int cCh = strlen(pChz);
			for (int iCh = 0; iCh < cCh; ++iCh)
				Trace(true, pChz[iCh]);
			Trace(true, "\")\n");
		}
#endif // DEBUG

		return false;
	}

	return true;
}

bool CTactrix::FTryReceiveMessage(MSGK msgk, u32 dMsTimeout)
{
	if (CBRemaining() == 0)
	{
		if (CBReceive(dMsTimeout) == 0)
		{
			Trace(true, "Timed out waiting for message type ");
			Trace(true, s_mpMsgkPChz[msgk]);
			Trace(true, "\n");
			return false;
		}
	}

	int cBRem = CBRemaining();
	u8 * pBRem = PBRemaining();

	int cBMsg = 0;
	switch (msgk)
	{
	case MSGK_InfoReply:
		{
			if (cBRem < 5 ||
				pBRem[0] != 'a' ||
				pBRem[1] != 'r' ||
				pBRem[2] != 'i')
			{
				return false;
			}

			bool fFoundCrLf = false;
			for (; cBMsg < cBRem - 1; ++cBMsg)
			{
				if (pBRem[cBMsg] == '\r' &&
					pBRem[cBMsg + 1] == '\n')
				{
					cBMsg += 2;
					fFoundCrLf = true;
					break;
				}
			}

			if (!fFoundCrLf)
				return false;
		}
		break;

	case MSGK_LoopbackStart:
	case MSGK_Loopback:
	case MSGK_LoopbackEnd:
	case MSGK_ReplyStart:
	case MSGK_Reply:
	case MSGK_ReplyEnd:
		{
			if (cBRem < 6 ||
				pBRem[0] != 'a' ||
				pBRem[1] != 'r' ||
				!isdigit(pBRem[2]) ||
				pBRem[4] != s_mpMsgkBType[msgk])
			{
				return false;
			}

			cBMsg = 4 + pBRem[3];
		}
		break;

	default:
		Trace(true, "FTryReceiveMessage called with invalid msgk\n");
		return false;
	}

	if (cBMsg > cBRem)
	{
		Trace(true, "FTryReceiveMessage received malformed message (cBMsg > cBRem)\n");
		return false;
	}

	// Receive succeeded; advance indices

	m_iBRecvPrev = m_iBRecv;
	m_iBRecv += cBMsg;

	return true;
}

bool CTactrix::FMustReceiveMessage(MSGK msgk, u32 dMsTimeout)
{
	if (!FTryReceiveMessage(msgk, dMsTimeout))
	{
		ResetReceive();

		Trace(true, "FMustReceiveMessage failed for message type ");
		Trace(true, s_mpMsgkPChz[msgk]);
		Trace(true, "\n");

		return false;
	}

	return true;
}

bool CTactrix::FTrySkipMessage(u32 dMsTimeout)
{
	if (CBRemaining() == 0)
	{
		if (dMsTimeout == 0)
		{
			return false;
		}
		else if (CBReceive(dMsTimeout) == 0)
		{
			Trace(true, "Timed out trying to skip message\n");
			return false;
		}
	}

	// Try each message type until we find one we recognize.

	for (MSGK msgk = MSGK_Min; msgk < MSGK_Max; msgk = MSGK(msgk + 1))
	{
		if (FTryReceiveMessage(msgk, 0))
			return true;
	}

	return false;
}

void CTactrix::FlushIncoming()
{
	Trace(true, "Flushing incoming data...\n");

	while (g_userial.available())
	{
		u8 b = g_userial.read();

		Trace(true, char(b));
	}

	Trace(true, "FlushIncoming done.\n");
}

bool CTactrix::FTryConnect()
{
	if (m_tactrixs != TACTRIXS_Disconnected)
	{
		Disconnect();
	}

#if !TEST_NO_TACTRIX
	if (!g_userial)
	{
		return false;
	}

	if (g_userial.idVendor() != s_nIdTactrix ||
		g_userial.idProduct() != s_nIdOpenPort20)
	{
		return false;
	}

	// Only receive one bulk transfer at a time

	g_userial.setTimeout(0);

	FlushIncoming();

	SendCommand("\r\n\r\nati\r\n");
	if (!FMustReceiveMessage(MSGK_InfoReply))
		return false;

	SendCommand("ata 2\r\n");
	if (!FMustReceiveMessage("aro 2\r\n"))
		return false;

	// Equivalent to J2534 PassThruConnect:
	//	protocol = ISO9141 (3),
	//	flags = ISO9141_NO_CHECKSUM (0x200 or 512)
	//	baudrate = 4800

	SendCommand("ato3 512 4800 0 3\r\n");
	if (!FMustReceiveMessage("aro 3\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P1_MAX (7) -- maximum inter-byte time for ECU responses in ms (default is 20)
	//	Value = 2

	SendCommand("ats3 7 2 4\r\n");
	if (!FMustReceiveMessage("aro 4\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P3_MIN (0xa or 10) -- minimum time between ECU response and start of new tester request in ms (default is 55)
	//	Value = 0

	SendCommand("ats3 10 0 5\r\n");
	if (!FMustReceiveMessage("aro 5\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P4_MIN (0xc or 12) -- minimum inter-byte time for a tester request in ms (default is 5)
	//	Value = 0

	SendCommand("ats3 12 0 6\r\n");
	if (!FMustReceiveMessage("aro 6\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = LOOPBACK (3) -- echo transmitted messages in the receive queue (default is OFF (0))
	//	Value = ON (1)

	SendCommand("ats3 3 1 7\r\n");
	if (!FMustReceiveMessage("aro 7\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = DATA_BITS (0x20 or 32)
	//	Value = 8

	SendCommand("ats3 32 8 8\r\n");
	if (!FMustReceiveMessage("aro 8\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = PARITY (0x16 or 22) -- default is NO_PARITY (0)
	//	Value = NO_PARITY (0)
	// BB (jpatry) Redundant?

	SendCommand("ats3 22 0 9\r\n");
	if (!FMustReceiveMessage("aro 9\r\n"))
		return false;

	// Equivalent to J2534 PassThruStartMsgFilter:
	//	FilterType = PASS_FILTER (1)
	//	pMaskMsg->TxFlags = 0
	//	pMaskMsg->DataSize = 1
	//	pMaskMsg->Data = "\0"
	//	pPatternMsg->Data = "\0"
	//
	//	This allows all messages through.

	const u8 aBFilter[] = { "atf3 1 0 1 10\r\n\0\0" };
	SendCommand(aBFilter, sizeof(aBFilter) - 1);
	if (!FMustReceiveMessage("arf3 0 10\r\n"))
		return false;
#endif // !TEST_NO_TACTRIX

	m_tactrixs = TACTRIXS_Connected;

	return true;
}

bool CTactrix::FTryStartPolling()
{
	if (m_tactrixs != TACTRIXS_Connected)
		return false;

#if !TEST_NO_TACTRIX
	// Equivalent to J2534 PassThruWriteMsgs; send SSM init sequence

	const u8 aBSsmInit[] = { "att3 6 0 2000000 11\r\n\x80\x10\xf0\x01\xbf\x40" };
	SendCommand(aBSsmInit, sizeof(aBSsmInit) - 1);
#endif // !TEST_NO_TACTRIX

#if !TEST_OFFLINE
	// Start of loopback message

	if (!FMustReceiveMessage(MSGK_LoopbackStart))
		return false;

	// Loopback message

	if (!FMustReceiveMessage("ar3\x07 \x80\x10\xf0\x01\xbf\x40"))
		return false;

	// End of loopback message

	if (!FMustReceiveMessage(MSGK_LoopbackEnd))
		return false;
#endif // !TEST_OFFLINE

#if !TEST_NO_TACTRIX
	// Acknowledgement

	if (!FMustReceiveMessage("aro 11\r\n"))
		return false;
#endif // !TEST_NO_TACTRIX

#if !TEST_OFFLINE
	// Start of normal message

	if (!FMustReceiveMessage(MSGK_ReplyStart))
		return false;

	{
		u8 aBSsmInitReply[255];
		u8 cBSsmInitReply = 0;

		while (!FTryReceiveMessage(MSGK_ReplyEnd))
		{
			if (CBRemaining() == 0 || !FMustReceiveMessage(MSGK_Reply))
				return false;

			const SJ2534Ar * pAr = (const SJ2534Ar *)PBMessage();

			u8 cBPayload = pAr->CBPayload();
			if (cBSsmInitReply + cBPayload > sizeof(aBSsmInitReply))
			{
				Trace(true, "Overflow of SSM init reply buffer\n");
				return false;
			}
			else
			{
				memcpy(&aBSsmInitReply[cBSsmInitReply], pAr->PBPayload(), cBPayload);
				cBSsmInitReply += cBPayload;
			}
		}

		// BB (jpatry) Validate SSM reply?

		Trace(true, "SSM Init Reply: ");
		TraceHex(true, aBSsmInitReply, cBSsmInitReply);
		Trace(true, "\n");
	}
#endif // !TEST_OFFLINE

	return FTryIssuePollRequest();
}

bool CTactrix::FTryIssuePollRequest()
{
#if !TEST_NO_TACTRIX
	// Issue address read request (A8)

	m_cBParamPoll = 0;
	for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
		m_cBParamPoll += s_mpParamCB[s_aParamPoll[iParamPoll]];

	int cBSsmRead = 7; // header + checksum + PP byte (single response/respond until interrupted)
	cBSsmRead += s_cBSsmAddr * m_cBParamPoll;

	if (cBSsmRead > s_cBSsmMax)
	{
		Trace(true, "SSM packet overflow (trying to read too many params)\n");
		return false;
	}

	// Equivalent to J2534 PassThruWriteMsgs; format is:
	//		att<channel> <data size> <TxFlags> <timeout> <id>\r\n<data>
	//	where data in this case is:
	//		0x80, 0x10, 0xf0, <data size - 5>, 0xa8, <PP byte: 0x00 or 0x01>, <address list...>, <checksum>
	//	PP byte is 0 if data should be sent once or 1 if it should be sent until interrupted.
	//	J2534 header is at most 23 bytes for this request.

	u8 aBReadRequest[s_cBSsmMax + 23];
	int cBReadRequest = snprintf(
							(char *)aBReadRequest,
							DIM(aBReadRequest),
							"att3 %d 0 2000000 12\r\n\x80\x10\xf0%c\xa8\x01",
							cBSsmRead,
							cBSsmRead - 5);
	int iSsmStart = cBReadRequest - 6;
	for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
	{
		PARAM param = s_aParamPoll[iParamPoll];
		u32 nAddr = s_mpParamABAddr[param];

		for (int iB = 0; iB < s_mpParamCB[param]; ++iB)
		{
			aBReadRequest[cBReadRequest + 0] = (nAddr & 0xff0000) >> 16;
			aBReadRequest[cBReadRequest + 1] = (nAddr & 0x00ff00) >> 8;
			aBReadRequest[cBReadRequest + 2] = (nAddr & 0x0000ff);
			CASSERT(s_cBSsmAddr == 3);
			cBReadRequest += s_cBSsmAddr;
			nAddr += 1;
		}
	}

	aBReadRequest[cBReadRequest] = BSsmChecksum(&aBReadRequest[iSsmStart], cBReadRequest - iSsmStart);
	++cBReadRequest;

	ASSERT(cBReadRequest - iSsmStart == cBSsmRead);

	SendCommand(aBReadRequest, cBReadRequest);
#endif // !TEST_NO_TACTRIX

#if !TEST_OFFLINE
	int iBLoopback = iSsmStart;

	while (iBLoopback < cBReadRequest)
	{
		if (!FMustReceiveMessage(MSGK_LoopbackStart))
			return false;

		if (!FMustReceiveMessage(MSGK_Loopback))
			return false;

		const SJ2534Ar * pArLoopback = (const SJ2534Ar *)PBMessage();
		int cBLoopbackFrag = pArLoopback->CBPayload();
		if (iBLoopback + cBLoopbackFrag > cBReadRequest ||
			memcmp(&aBReadRequest[iBLoopback], pArLoopback->PBPayload(), cBLoopbackFrag) != 0)
		{
			Trace(true, "Loopback error\n");
			return false;
		}

		iBLoopback += cBLoopbackFrag;

		if (!FMustReceiveMessage(MSGK_LoopbackEnd))
			return false;
	}
#endif // !TEST_OFFLINE

#if !TEST_NO_TACTRIX
	if (!FMustReceiveMessage("aro 12\r\n"))
		return false;
#endif // !TEST_NO_TACTRIX

	m_tactrixs = TACTRIXS_Polling;
	m_msPollingStart = millis();

	return true;
}

bool CTactrix::FTryUpdatePolling()
{
	if (m_tactrixs != TACTRIXS_Polling)
		return false;

#if !TEST_NO_TACTRIX
	if (!g_userial)
		return false;
#endif // !TEST_NO_TACTRIX

	u32 msCur = millis();

#if !TEST_OFFLINE
	bool fReceivedReply = false;
	for (int i = 0; i < 10; ++i)
	{
		static const int s_dMsReplyStartTimeout = 300;
		if (FTryReceiveMessage(MSGK_ReplyStart, s_dMsReplyStartTimeout))
		{
			if (FMustReceiveMessage(MSGK_Reply))
			{
				const SJ2534Ar * pAr = (const SJ2534Ar *)PBMessage();
				const SSsm * pSsm = (const SSsm *)pAr->PBPayload();
				if (pSsm->m_bHdr == 0x80 &&
					pSsm->m_bDst == SSMID_Tool &&
					pSsm->m_bSrc == SSMID_Ecu &&
					pSsm->m_bCmd == SSMCMD_AddressReadResponse &&
					pSsm->FVerifyChecksum())
				{
					fReceivedReply = true;
					break;
				}
			}
		}
		else
		{
			if (m_cBRecv == 0)
			{
				Trace(true, "Timed out while waiting to receive AddressReadResponse message\n");

				// Try to re-issue request

				if (!FTryIssuePollRequest())
					return false;
			}
			else
			{
				// Try to skip message

				if (!FTrySkipMessage(0))
					ResetReceive();	// last resort
			}
		}
	}

	if (!fReceivedReply)
	{
		Trace(true, "Failed to received AddressReadResponse message after 10 tries\n");
		return false;
	}

	const SJ2534Ar * pAr = (const SJ2534Ar *)PBMessage();
	const SSsm * pSsm = (const SSsm *)pAr->PBPayload();
	const u8 * pBData = pSsm->PBData();

	// BB (jpatry) TODO: Handle replies split over multiple messages

	if (pSsm->CBData() != m_cBParamPoll)
	{
		Trace(true, "Received incorrect number of bytes in AddressReadResponse\n");
		return false;
	}

	for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
	{
		PARAM param = s_aParamPoll[iParamPoll];

		int nValue = 0;
		for (int iB = 0; iB < s_mpParamCB[param]; ++iB)
		{
			nValue <<= 8;
			nValue |= *pBData;
			++pBData;
		}

		ProcessParamValue(param, nValue);
	}
#else // TEST_OFFLINE
	// Dummy boost values to test gauge

	float gBoost = -10.0f + 30.0f * (-sinf(msCur / 1000.0f) * 0.5f + 0.5f);
	m_mpParamGValue[PARAM_BoostPsi] = gBoost;

	float uIam = ((msCur & 0x1fff) - 0x1e0 < 0x100) ? 0.93f : 1.0f;
	m_mpParamGValue[PARAM_Iam] = uIam;

	float degFbkc = ((msCur & 0x1fff) < 0x100) ? -1.4f : 0.0f;
	m_mpParamGValue[PARAM_FbkcDeg] = degFbkc;

	float degFlkc = ((msCur & 0x1fff) - 0xf0 < 0x100) ? -1.4f : 0.0f;
	m_mpParamGValue[PARAM_FlkcDeg] = degFlkc;

	m_mpParamGValue[PARAM_SpeedMph] = 113.0f;

	m_mpParamGValue[PARAM_IatF] = 109.0f;

	m_mpParamGValue[PARAM_Maf] = 50.0f;

	m_mpParamGValue[PARAM_Rpm] = 2500.0f;

	m_mpParamGValue[PARAM_LightSwitch] = ((msCur & 0x1fff) < 0x1000);
#endif // TEST_OFFLINE

	// Update calculated params

	m_mpParamGValue[PARAM_LoadGPerRev] = GParam(PARAM_Maf) * 60.0f / max(GParam(PARAM_Rpm), 1.0f);
	m_mpParamGValue[PARAM_IdcPct] = GParam(PARAM_Rpm) * GParam(PARAM_IpwMs) * (1.0f / 1200.0f);

	// Update wideband AFR (serial connection). Innovate LC-2 use LC-1/"new AFR" sub-packets described here:
	//  https://www.innovatemotorsports.com/support/downloads/Seriallog-2.pdf and
	//  https://www.innovatemotorsports.com/support/manual/OT-2%20SDK.pdf

	static const int s_cBAfrPacket = 6;
	static const int s_cBAfrPacketHeader = 2;
	static const int s_cBAfrPayload = s_cBAfrPacket - s_cBAfrPacketHeader;
	while (Serial5.available() >= s_cBAfrPacket)
	{
		// Look for start byte

		int cBPayload = 0;
		bool fDataPacket = false;

		u8 bHdr0 = Serial5.read();
		if ((bHdr0 & 0xA2) == 0xA2)
		{
			u8 bHdr1 = Serial5.peek();
			if ((bHdr1 & 0x80) == 0x80)
			{
				(void) Serial5.read();

				// Header specifies payload size in 16-bit words

				cBPayload = (((bHdr0 & 0x1) << 7) | (bHdr1 & 0x7F)) * 2;
				fDataPacket = (bHdr0 & 0x10);
			}
		}

		if (fDataPacket && cBPayload == s_cBAfrPayload && Serial5.available() >= s_cBAfrPayload)
		{
			CASSERT(s_cBAfrPayload == 4);
			u8 bPayload0 = Serial5.read();
			u8 bPayload1 = Serial5.read();
			u8 bPayload2 = Serial5.read();
			u8 bPayload3 = Serial5.read();

			if ((bPayload0 & 0xE0) == 0x40)
			{
				// AFR payload

				u8 bFunction = (bPayload0 & 0x1C) >> 2;
				bool fReady = (bFunction == 0x0);
				bool fHighO2 = (bFunction == 0x1);
				bool fWarmup = (bFunction == 0x4);

				u32 nLambda = ((bPayload2 & 0x3F) << 7) | (bPayload3 & 0x7F);

				if (fReady)
				{
					u32 nAfr = ((bPayload0 & 0x1) << 7) | (bPayload1 & 0x7F);
					float gAfr = float((nLambda + 500) * nAfr) / 10000.0f;
					m_mpParamGValue[PARAM_WidebandAfr] = gAfr;
				}
				else if (fHighO2)
				{
					// NOTE (jpatry) Max valid AFR I've seen is ~116.

					static const float s_gAfrMax = 1000.0f;
					m_mpParamGValue[PARAM_WidebandAfr] = s_gAfrMax;
				}
				else if (fWarmup)
				{
					// Warmup counts up from 1.0 to 4.x
					// BB (jpatry) Docs say nLambda is temperature in 1/10% of operating temperature. This doesn't
					//	appear to be accurate; I'm seeing values from 0 to ~180.

					float gWarmup = nLambda / 50.0f;
					m_mpParamGValue[PARAM_WidebandAfr] = 1.0f + gWarmup;
				}
				else
				{
					// Display unexpected function code as 0.<function code>

					m_mpParamGValue[PARAM_WidebandAfr] = bFunction / 10.0f;
				}
			}
		}
		else if (cBPayload > 0)
		{
			// Discard payload

			cBPayload = min(cBPayload, Serial5.available());
			for (int iBPacket = 0; iBPacket < cBPayload; ++iBPacket)
				(void) Serial5.read();
		}
	}

#if !TEST_OFFLINE
	ASSERT(pBData - pSsm->PBData() == pSsm->CBData());

	if (!FMustReceiveMessage(MSGK_ReplyEnd))
		return false;
#endif // !TEST_OFFLINE

	// Update log

	SLogEntry logent;
	logent.m_msTimestamp = msCur;
	for (int iParamLog = 0; iParamLog < s_cParamLog; ++iParamLog)
		logent.m_mpIParamLogGValue[iParamLog] = GParam(s_aParamLog[iParamLog]);
	g_loghist.WriteLogEntry(logent);

	return true;
}

float CTactrix::GParam(PARAM param) const
{
	return m_mpParamGValue[param];
}

void CTactrix::ProcessParamValue(PARAM param, u32 nValue)
{
	union
	{
		u32		m_n;
		float	m_g;
	} ng;
	ng.m_n = nValue;

	switch (param)
	{
	case PARAM_FbkcDeg:
		// No conversion necessary

		break;

	case PARAM_FlkcDeg:
		ng.m_g = 0.25f * float(nValue) - 32.0f;
		break;

	case PARAM_Iam:
		ng.m_g = float(nValue) / 16.0f;
		break;

	case PARAM_BoostPsi:
		ng.m_g = 0.145098039216f * float(nValue) - 18.5725490196f; // (x-128)*37/255
		break;

	case PARAM_CoolantTempF:
	case PARAM_IatF:
		ng.m_g = 1.8f * float(nValue) - 40.0f;
		break;

	case PARAM_Rpm:
		ng.m_g = 0.25f * float(nValue);
		break;

	case PARAM_Maf:
		ng.m_g = 0.01f * float(nValue);
		break;

	case PARAM_ThrottlePct:
		ng.m_g = float(nValue) * (100.0f / 255.0f);
		break;

	case PARAM_SpeedMph:
		ng.m_g = float(nValue) * 0.621371192f;
		break;

	case PARAM_IpwMs:
		ng.m_g = float(nValue) * (256.0f / 1000.0f);
		break;

	case PARAM_Afr1:
		ng.m_g = float(nValue) * (14.7f / 128.0f);
		break;

	case PARAM_TargetAfr:
		ng.m_g = (14.7f / 0.0004882812f) / float(nValue);
		break;

	case PARAM_MafVoltage:
		ng.m_g = float(nValue) * (1.0f / 50.0f);
		break;

	case PARAM_LightSwitch:
		ng.m_g = (nValue & 0x8) ? 1.0f : 0.0f;
		break;

	default:
		CASSERT(PARAM_LightSwitch == PARAM_Max - 1); // Compile-time reminder to add new params to switch
		ASSERT(false);
	}

	Trace(s_fTraceComms, s_mpParamPChz[param]);
	Trace(s_fTraceComms, ": ");
	Trace(s_fTraceComms, ng.m_g);
	Trace(s_fTraceComms, "\n");

	m_mpParamGValue[param] = ng.m_g;
}

void CTactrix::Disconnect()
{
	ResetReceive();

	if (m_tactrixs == TACTRIXS_Connected)
	{
		// Equivalent to J2534 PassThruStopMsgFilter

		SendCommand("atk3 0 12\r\n");
		(void) FMustReceiveMessage("aro 12\r\n");

		// Equivalent to J2534 PassThruDisconnect

		SendCommand("atc3 13\r\n");
		(void) FMustReceiveMessage("aro 13\r\n");
	}

	SendCommand("atz 14\r\n");
	(void) FMustReceiveMessage("aro 14\r\n");

	m_tactrixs = TACTRIXS_Disconnected;
}

CTactrix g_tactrix;



// Touchscreen Configuration

static const int s_nPinTouchCs = 8;
static const int s_nPinTouchIrq = 2;

// Calibration data for the raw touch data to the screen coordinates

static const int s_xTsMin = 180;
static const int s_yTsMin = 250;
static const int s_xTsMax = 3700;
static const int s_yTsMax = 3800;

XPT2046_Touchscreen g_ts = XPT2046_Touchscreen(s_nPinTouchCs, s_nPinTouchIrq);
bool g_fTouch = false;
u16 g_xTouch;
u16 g_yTouch;



// TFT Display Configuration

static const int s_dXScreen = 320;
static const int s_dYScreen = 240;
static const int s_nPinTftCs = 10;
static const int s_nPinTftDc = 9;
static const int s_nPinTftLedPwm = 36;
static const u32 s_msLogAfterEvent = 5000;	// how long to log after an event

ILI9341_t3 g_tft = ILI9341_t3(s_nPinTftCs, s_nPinTftDc);
GFXcanvas8 g_aCnvs[2] = { GFXcanvas8(s_dXScreen, s_dYScreen), GFXcanvas8(s_dXScreen, s_dYScreen) };
GFXcanvas8 * g_pCnvs = &g_aCnvs[0];
GFXcanvas8 * g_pCnvsPrev = &g_aCnvs[1];
uint16_t g_aColorPalette[64];

static const uint8_t s_iColorBlack = 0;

static const uint8_t s_iColorWhiteMic = 0;

static const uint8_t s_iColorWhite = 31;
CASSERT(s_iColorWhite == DIM(g_aColorPalette) / 2 - 1);

static const uint8_t s_iColorGrey = 16;

static const uint8_t s_iColorRedMic = 32;
CASSERT(s_iColorRedMic == DIM(g_aColorPalette) / 2);

static const uint8_t s_iColorRed = 63;
CASSERT(s_iColorRed == DIM(g_aColorPalette) - 1);

static const uint8_t s_iColorIamAlert = 55;



void UpdateTft()
{
	// Always fully write one scan line each frame in case of hardware issues that corrupt the display.

	static int s_iYLineUpdate = 0;

	int dBLine = s_iYLineUpdate * s_dXScreen;

	g_tft.writeRect8BPP(
			0,
			s_iYLineUpdate,
			s_dXScreen,
			1,
			g_pCnvs->getBuffer() + dBLine,
			g_aColorPalette);

	// Update previous buffer so we don't write that line again

	memcpy(
		g_pCnvsPrev->getBuffer() + dBLine,
		g_pCnvs->getBuffer() + dBLine,
		s_dXScreen);

	// Wrap around

	s_iYLineUpdate += 1;
	if (s_iYLineUpdate >= s_dYScreen)
		s_iYLineUpdate = 0;

	// Incremental update of the rest

	g_tft.updateRect8BPP(
			0,
			0,
			s_dXScreen,
			s_dYScreen,
			g_pCnvs->getBuffer(),
			g_pCnvsPrev->getBuffer(),
			g_aColorPalette);
}



// Built-in bitmaps. Arrays converted from bitmaps using The Dot Factory:
//	http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator/

static const u8 s_aBSti[] =
{
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001111, 0b11111111, 0b11111111, 0b11100000, 0b00000000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01110000, 0b00000000, 0b00000011, 0b11111111, 0b10000000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b10000000, 0b00000000, 0b00000000, 0b01111111, 0b11111000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b11100000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00011000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11000000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00100000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01100000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111000, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b11000000, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b11111110, 0b00000000,
	0b00000000, 0b00000000, 0b00000000, 0b00000011, 0b11000000, 0b00000000, 0b00000000, 0b00000000, 0b00000011, 0b11111111, 0b10000000,
	0b00000000, 0b00000000, 0b00000000, 0b00001100, 0b11000000, 0b00000000, 0b00000000, 0b00000011, 0b11111111, 0b11111111, 0b11000000,
	0b00000000, 0b00000001, 0b11111111, 0b11110000, 0b00000000, 0b00001111, 0b11111111, 0b11111111, 0b11111111, 0b11111100, 0b01000000,
	0b00000000, 0b00001111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111000, 0b01000000,
	0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111100, 0b01000000,
	0b00000111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11000000,
	0b00001111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111100, 0b01111111, 0b11111111, 0b11111111, 0b11000000,
	0b00010000, 0b01111111, 0b11111111, 0b11111111, 0b11111111, 0b00000000, 0b00000001, 0b01111111, 0b11111111, 0b11111111, 0b11100000,
	0b00100000, 0b11111100, 0b00000111, 0b11111111, 0b11111110, 0b11111111, 0b11111111, 0b11111111, 0b11100000, 0b00111111, 0b11100000,
	0b01000011, 0b11110000, 0b00000001, 0b11111111, 0b11111101, 0b10000000, 0b10000010, 0b11111111, 0b10000000, 0b00001111, 0b11110000,
	0b01111111, 0b11100000, 0b00000000, 0b00001111, 0b11111101, 0b11110110, 0b10110101, 0b11111111, 0b00000000, 0b00000111, 0b11110000,
	0b01111111, 0b11100000, 0b00000000, 0b11011111, 0b11111110, 0b00011010, 0b10110101, 0b11111111, 0b00000000, 0b00000111, 0b11110000,
	0b01111111, 0b11000000, 0b00000000, 0b01011111, 0b11111111, 0b11111010, 0b10101011, 0b11111110, 0b00000000, 0b00000011, 0b11110000,
	0b01111110, 0b01000000, 0b00000000, 0b01011111, 0b11111000, 0b00000110, 0b10101011, 0b11111110, 0b00000000, 0b00000011, 0b11110000,
	0b01111110, 0b01000000, 0b00000001, 0b01101111, 0b11111111, 0b11111111, 0b11111111, 0b11111110, 0b00000000, 0b00001011, 0b11110000,
	0b01111110, 0b01000000, 0b00000001, 0b01101111, 0b11111111, 0b11111111, 0b11111111, 0b11111110, 0b00000000, 0b00001011, 0b11100000,
	0b01111110, 0b01000000, 0b00000001, 0b01110111, 0b11111111, 0b11111111, 0b11111111, 0b11111110, 0b00000000, 0b00001011, 0b11100000,
	0b01111111, 0b11000000, 0b00000010, 0b01111111, 0b11111111, 0b11111111, 0b11111111, 0b11111110, 0b00000000, 0b00010011, 0b10000000,
	0b11111111, 0b11001000, 0b00000010, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01000000, 0b00010000, 0b00000000,
	0b00000000, 0b00000110, 0b00001100, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00110000, 0b01100000, 0b00000000,
	0b00000000, 0b00000001, 0b11110000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001111, 0b10000000, 0b00000000,
};

static const u8 s_dXSti = 88;
static const u8 s_dYSti = 29;

static const u8 s_aBGfxguy[] =
{
	0b00000000, 0b00000000, 0b00000000,
	0b00000010, 0b00000000, 0b00000000,
	0b00100100, 0b10100100, 0b10101010,
	0b01010110, 0b01001010, 0b10101010,
	0b00100100, 0b10100100, 0b01100100,
	0b00010000, 0b00000010, 0b00001000,
	0b00100000, 0b00000100, 0b00010000,
	0b00000000, 0b00000000, 0b00000000,
};

static const u8 s_dXGfxguy = 24;
static const u8 s_dYGfxguy = 8;



void DrawSplashScreen()
{
	static const int s_xSti = 198;
	static const int s_ySti = 201;
	static const int s_xGfxguy = 282;
	static const int s_yGfxguy = 222;

	g_pCnvs->drawBitmap(s_xSti, s_ySti, s_aBSti, s_dXSti, s_dYSti, s_iColorWhite);
	g_pCnvs->drawBitmap(s_xGfxguy, s_yGfxguy, s_aBGfxguy, s_dXGfxguy, s_dYGfxguy, s_iColorGrey);
}



void DisplayStatus(const char * pChz)
{
	g_pCnvs->fillScreen(s_iColorBlack);

	// Move down a line each update so that we can tell it's alive

	static u32 s_diYCursor = 0;
	g_pCnvs->setCursor(10, 38 + (++s_diYCursor % 128));

	g_pCnvs->setT3Font(&Exo_16_Bold_Italic);
	g_pCnvs->setTextColor(s_iColorWhite);
	g_pCnvs->println(pChz);
}

void DrawAbIntoCanvas(const u8 * aB, const u16 (&aNBb)[4])
{
	const u8 * pB = &aB[0];
	for (int iY = aNBb[1]; iY < aNBb[3]; ++iY)
	{
		u8 * aBRow = g_pCnvs->getBuffer() + iY * s_dXScreen;
		for (int iX = aNBb[0]; iX < aNBb[2]; ++iX)
		{
			u8 b = *pB++;
			if (b)
				aBRow[iX] = b >> 3;
		}
	}
}



// SD Card Configuration

static const int s_nPinSdChipSelect = BUILTIN_SDCARD;
static const char * s_pChzLogPrefix = "log";
static const char * s_pChzLogSuffix = ".csv";
static const int s_cLogFileMax = 1000;
static int s_nLogSuffix = 0;
static File s_fileLog;



void UpdateLog()
{
	if (!s_fileLog)
	{
		// Open the log file

		char aChzPath[32];
		snprintf(aChzPath, DIM(aChzPath), "%s%d%s", s_pChzLogPrefix, s_nLogSuffix, s_pChzLogSuffix);

		bool fWriteHeader = !SD.exists(aChzPath);
		s_fileLog = SD.open(aChzPath, FILE_WRITE | O_APPEND);

		if (s_fileLog)
		{
			Trace(true, "Opened file '");
			Trace(true, aChzPath);
			Trace(true, "' for append.\n");

			if (fWriteHeader)
			{
				// Write the header

				s_fileLog.print("Time (msec),");
				for (int iParamLog = 0;;)
				{
					s_fileLog.print(s_mpParamPChz[s_aParamLog[iParamLog]]);

					if (++iParamLog >= s_cParamLog)
						break;

					s_fileLog.print(",");
				}

				s_fileLog.println();
			}
		}
		else
		{
			// Don't try to open again

			s_nLogSuffix = 0;

			Serial.print("Failed to open file '");
			Serial.print(aChzPath);
			Serial.print("' for writing. Is SD card full?\n");

			return;
		}
	}

	// Write the log history

	while (!g_loghist.FIsEmpty())
	{
		const SLogEntry & logent = g_loghist.LogentRead();

		s_fileLog.print(logent.m_msTimestamp);

		s_fileLog.print(",");

		CASSERT(s_cParamLog > 0);
		for (int iParamLog = 0;;)
		{
			s_fileLog.print(logent.m_mpIParamLogGValue[iParamLog], 3);

			if (++iParamLog >= s_cParamLog)
				break;

			s_fileLog.print(",");
		}

		s_fileLog.println();
	}
}



// Arduino setup() func

void setup()
{
	analogWrite(s_nPinTftLedPwm, 255);

#if DEBUG
	while (!Serial && (millis() < 5000))
		(void) 0; // wait for Arduino Serial Monitor
#endif // DEBUG

	Serial.print("\n\nBoostino 0.1  Copyright (C) 2019, 2020  Jasmin Patry\n");
	Serial.print("This program comes with ABSOLUTELY NO WARRANTY.\n");
    Serial.print("This is free software, and you are welcome to redistribute it\n");
    Serial.print("under certain conditions. For details see\n");
	Serial.print("https://github.com/jasminpatry/boostino/blob/master/COPYING\n\n");

	// Initialize Serial5 (for reading AFR). Innovate LC-2 outputs 0-5V RS-232 serial; using a voltage divider (2.2K and
	//	3.3K) to step down to 0-3V, and specifying RXINV since TTL and RS-232 serial are inverted. TX is unused and
	//	unconnected.

	Trace(true, "Initialize Serial5\n");
	Serial5.begin(19200, SERIAL_8N1_RXINV_TXINV);

	while (!Serial5)
	{
		Trace(true, "Waiting for Serial5\n");
		delay(1000);
	}

	Trace(true, "Serial5 initialized\n");

	// Initialize OLED

	g_tft.begin();
	g_tft.setRotation(1);
	g_tft.fillScreen(ILI9341_BLACK);

	// Create palette

	for (int iColor = 0; iColor < DIM(g_aColorPalette); ++iColor)
	{
		uint8_t b = iColor & s_iColorWhite;
		uint16_t color = (b << 11) | (b << 6) | b;

		// Entries 32-63 are red

		if (iColor >= DIM(g_aColorPalette) / 2)
			color &= 0xF800;

		g_aColorPalette[iColor] = color;
	}

	// Setup canvases

	for (int iCnvs = 0; iCnvs < DIM(g_aCnvs); ++iCnvs)
	{
		g_aCnvs[iCnvs].setTextColor(s_iColorWhite);
		g_aCnvs[iCnvs].setRotation(0);
		g_aCnvs[iCnvs].fillScreen(s_iColorBlack);
	}

	DrawSplashScreen();
	UpdateTft();

	if (g_ts.begin())
		g_ts.setRotation(3);
	else
		Serial.println("Couldn't start touchscreen controller");

	// Initialize SD Card

	if (SD.begin(s_nPinSdChipSelect))
	{
		// Determine log file index

		for (int iSuffix = 1; iSuffix < s_cLogFileMax; ++iSuffix)
		{
			char aChzPath[32];
			snprintf(aChzPath, DIM(aChzPath), "%s%d%s", s_pChzLogPrefix, iSuffix, s_pChzLogSuffix);
			if (!SD.exists(aChzPath))
			{
				s_nLogSuffix = iSuffix;
				Trace(true, "Will use log file name '");
				Trace(true, aChzPath);
				Trace(true, "'\n");
				break;
			}
		}
	}
	else
	{
		Serial.println("SD card failed, or not present");
	}

	// Initialize USB host interface

	g_uhost.begin();

	// Teensy has 13-bit ADC (defaults to 10-bit to be Arduino-compatible)

	analogReadResolution(s_cBitAnalog);
}



// Arduino loop() func

void loop()
{
	// Update USB

	g_uhost.Task();

	// Flip buffers

	swap(g_pCnvs, g_pCnvsPrev);

	// Draw splash screen for 5 seconds, or until we connect

	static const u32 s_msSplashMax = 5000;

	u32 msCur = millis();

	if (msCur <= s_msSplashMax)
		DrawSplashScreen();

	// Detect touchscreen events

	bool fTouch = false;
	if (!g_ts.bufferEmpty())
	{
		TS_Point posTouch = g_ts.getPoint();
		static const int s_zTouchMin = 500;
		if (posTouch.z >= s_zTouchMin)
		{
			fTouch = true;

			g_xTouch = map(posTouch.x, s_xTsMin, s_xTsMax, 0, s_dXScreen);
			g_yTouch = map(posTouch.y, s_yTsMin, s_yTsMax, 0, s_dYScreen);

			Trace(s_fTraceTouch, "Touch point: ");
			Trace(s_fTraceTouch, g_xTouch);
			Trace(s_fTraceTouch, " ");
			Trace(s_fTraceTouch, g_yTouch);
			Trace(s_fTraceTouch, " pressure ");
			Trace(s_fTraceTouch, posTouch.z);
			Trace(s_fTraceTouch, "\n");

			g_fTouch = true;
		}
	}

	bool fTouchRelease = false;
	if (!fTouch && g_fTouch)
	{
		g_fTouch = false;
		fTouchRelease = true;

		Trace(true, "Touch release: ");
		Trace(true, g_xTouch);
		Trace(true, " ");
		Trace(true, g_yTouch);
		Trace(true, "\n");
	}

	if (g_userial != g_fUserialActive)
	{
		if (g_fUserialActive)
		{
			Serial.print("*** Device - disconnected ***\n");
			g_fUserialActive = false;
		}
		else
		{
			Serial.printf("*** Device %x:%x - connected ***\n", g_userial.idVendor(), g_userial.idProduct());
			g_fUserialActive = true;

			const u8 * pChz = g_userial.manufacturer();
			if (pChz && *pChz)
				Serial.printf("  manufacturer: %s\n", pChz);

			pChz = g_userial.product();
			if (pChz && *pChz)
				Serial.printf("  product: %s\n", pChz);

			pChz = g_userial.serialNumber();
			if (pChz && *pChz)
				Serial.printf("  Serial: %s\n", pChz);

			static const int s_nUsbBaud = 115200;
			g_userial.begin(s_nUsbBaud);
		}
	}

#if !TEST_NO_TACTRIX
	if (!g_fUserialActive)
	{
		if (msCur > s_msSplashMax)
			DisplayStatus("Connect to Tactrix");
	}
	else
#endif // !TEST_NO_TACTRIX
	if (g_tactrix.Tactrixs() == TACTRIXS_Disconnected)
	{
		if (msCur > s_msSplashMax)
			DisplayStatus("Connecting...");

		if (!g_tactrix.FTryConnect())
		{
			g_tactrix.Disconnect();
			delay(s_dMsTimeoutDefault);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Connected)
	{
		if (msCur > s_msSplashMax)
			DisplayStatus("Polling ECU...");

		if (!g_tactrix.FTryStartPolling())
		{
			g_tactrix.Disconnect();
			delay(s_dMsTimeoutDefault);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Polling)
	{
		if (g_tactrix.FTryUpdatePolling())
		{
			// Update data, handle touch events, draw gauge and log if necessary

			g_pCnvs->fillScreen(s_iColorBlack);

			// Update screen brightness based on headlights on/off

			analogWrite(s_nPinTftLedPwm, (g_tactrix.GParam(PARAM_LightSwitch)) ? 32 : 255);

			static float s_gBoostMax = -1000.0f;

			// Check for max boost reset event

			if (fTouchRelease && g_xTouch >= 245 && g_yTouch >= 195)
				s_gBoostMax = -1000.0f;

			float gBoost = g_tactrix.GParam(PARAM_BoostPsi);
			s_gBoostMax = max(gBoost, s_gBoostMax);

			static float s_degFbkcAnyMin = 0.0f;
			static float s_degFbkcHighLoadMin = 0.0f;
			static float s_degFbkcPrev = 0.0f;
			static u32 s_msFbkcEventLast = 0;
			static int s_cFbkcEvent = 0;
			float degFbkc = g_tactrix.GParam(PARAM_FbkcDeg);
			float gLoad = g_tactrix.GParam(PARAM_LoadGPerRev);

			// Check for min FBKC reset event

			if (fTouchRelease && g_xTouch >= 240 && g_yTouch <= 55)
			{
				s_degFbkcHighLoadMin = 0.0f;
				s_cFbkcEvent = 0;
			}

			static bool s_fDrawMph = false;
			if (fTouchRelease && g_xTouch <= 100 && g_yTouch <= 40)
				s_fDrawMph = !s_fDrawMph;

			// Check if we have a new FBKC event (ignore small, low-load events for display, but still log them)

			static const float s_gLoadHighThreshold = 1.25f;
			static const float s_degFbkcLowLoadThreshold = -3.0f;
			if (degFbkc < s_degFbkcPrev && (gLoad > s_gLoadHighThreshold || degFbkc < s_degFbkcLowLoadThreshold))
			{
				++s_cFbkcEvent;
				s_degFbkcHighLoadMin = min(degFbkc, s_degFbkcHighLoadMin);
			}

			s_degFbkcPrev = degFbkc;
			s_degFbkcAnyMin = min(degFbkc, s_degFbkcAnyMin);

			if (degFbkc < 0.0f)
				s_msFbkcEventLast = msCur;

			static float s_degFlkcMin = 1000.0f;
			static u32 s_msFlkcEventLast = 0;
			float degFlkc = g_tactrix.GParam(PARAM_FlkcDeg);
			s_degFlkcMin = min(degFlkc, s_degFlkcMin);
			if (degFlkc < 0.0f)
				s_msFlkcEventLast = msCur;

			static float s_uIamMin = 1.0f;

			static const float s_sRadiusIamCircle = 32;
			static const int s_xBoostCenter = 160;
			static const int s_yBoostCenter = 131;

			// Check for min IAM reset event

			if (fTouchRelease)
			{
				int dXTouch = g_xTouch - s_xBoostCenter;
				int dYTouch = g_yTouch - s_yBoostCenter;
				if (dXTouch * dXTouch + dYTouch * dYTouch < s_sRadiusIamCircle * s_sRadiusIamCircle)
					s_uIamMin = 1.0f;
			}

			float uIam = g_tactrix.GParam(PARAM_Iam);
			s_uIamMin = min(uIam, s_uIamMin);

			// Check if doing a WOT pull

			static const int s_cWotThreshold = 20;
			CASSERT(s_cWotThreshold < s_cEntryLogHistory);

			static int s_cWot = 0;
			float gThrottlePct = g_tactrix.GParam(PARAM_ThrottlePct);
			if (gThrottlePct > 95.0f)
				++s_cWot;
			else
				s_cWot = 0;

			float gSpeedMph = g_tactrix.GParam(PARAM_SpeedMph);

			// Write to log if WOT for long enough, or if min IAM < 1 or a knock event happened recently

			static const bool s_fLogAlways = false;

			bool fWriteToLog = (s_nLogSuffix &&
								gSpeedMph >= 5.0f &&
								(s_fLogAlways ||
								 s_cWot >= s_cWotThreshold ||
								 s_uIamMin < 1.0f ||
								 (s_degFbkcAnyMin < 0.0f && msCur - s_msFbkcEventLast < s_msLogAfterEvent) ||
								 (s_degFlkcMin < 0.0f && msCur - s_msFlkcEventLast < s_msLogAfterEvent)));

			g_pCnvs->setT3Font(&Exo_28_Bold_Italic);

			// Boost gauge
			// NOTE (jpatry) Angles measured CCW from +X axis (3 o'clock).
			//	Boost of 0 = 12 o'clock; -15 = 9 o'clock; +15 = 3 o'clock.

			float radBoost = (6.0f * gBoost + 90.0f) * s_gPi / 180.0f;
			float radBoostMax = (6.0f * s_gBoostMax + 90.0f) * s_gPi / 180.0f;
			float gSinBoost = sinf(radBoost);
			float gCosBoost = cosf(radBoost);
			float gCotanBoost = gCosBoost / gSinBoost;
			static const int s_sNeedle = 113;

			{
				// Draw boost gauge background, with portion behind needle in red

				const u8 * pB = &g_aBGaugeBg[0];
				if (radBoost < s_gPi)
				{
					for (int iY = g_aNBbGaugeBg[1]; iY < g_aNBbGaugeBg[3]; ++iY)
					{
						int diY = iY - s_yBoostCenter;
						int iXRedMac;
						if (diY >= 0)
							iXRedMac = -1;
						else
							iXRedMac = min(g_aNBbGaugeBg[2], roundf(float(diY) * gCotanBoost + s_xBoostCenter));

						u8 * aBRow = g_pCnvs->getBuffer() + iY * s_dXScreen;
						int iX;
						for (iX = g_aNBbGaugeBg[0]; iX < iXRedMac; ++iX)
						{
							u8 b = *pB++;
							aBRow[iX] = 0x20 | (b >> 2);
						}
						for (; iX < g_aNBbGaugeBg[2]; ++iX)
						{
							u8 b = *pB++;
							aBRow[iX] = b >> 3;
						}
					}
				}
				else
				{
					for (int iY = g_aNBbGaugeBg[1]; iY < g_aNBbGaugeBg[3]; ++iY)
					{
						int diY = iY - s_yBoostCenter;
						int iXRedMic;
						if (diY <= 0)
							iXRedMic = g_aNBbGaugeBg[0];
						else
							iXRedMic = min(g_aNBbGaugeBg[2], roundf(float(diY) * gCotanBoost + s_xBoostCenter));

						u8 * aBRow = g_pCnvs->getBuffer() + iY * s_dXScreen;
						int iX;
						for (iX = g_aNBbGaugeBg[0]; iX < iXRedMic; ++iX)
						{
							u8 b = *pB++;
							aBRow[iX] = b >> 3;
						}
						for (; iX < g_aNBbGaugeBg[2]; ++iX)
						{
							u8 b = *pB++;
							aBRow[iX] = 0x20 | (b >> 2);
						}
					}
				}
			}

			// Draw boost gauge foreground

			DrawAbIntoCanvas(&g_aBGaugeFg[0], g_aNBbGaugeFg);

			// Draw MPH/AFR label

			if (s_fDrawMph)
				DrawAbIntoCanvas(&g_aBLabelMph[0], g_aNBbLabelMph);
			else
				DrawAbIntoCanvas(&g_aBLabelAfr[0], g_aNBbLabelAfr);

			char aChz[16];

			{
				// Draw boost value, aligning at decimal point

				int cCh = snprintf(aChz, DIM(aChz), "%.1f", fabsf(gBoost));
				char chDec = aChz[cCh - 1];
				aChz[cCh - 1] = '\0';

				g_pCnvs->setCursor(138 - g_pCnvs->strPixelLen(aChz), 175);
				aChz[cCh - 1] = chDec;
				g_pCnvs->print(aChz);

				if (gBoost < 0.0f)
				{
					g_pCnvs->setCursor(77, 175);
					g_pCnvs->write('-');
				}
			}

			if (s_fDrawMph)
			{
				// Draw speed

				snprintf(aChz, DIM(aChz), "%d", int(roundf(gSpeedMph)));
				g_pCnvs->setCursor(66 - g_pCnvs->strPixelLen(aChz), 10);
				g_pCnvs->print(aChz);
			}
			else
			{
				// Draw AFR and align at decimal point

				float gAfr = g_tactrix.GParam(PARAM_WidebandAfr);

				// Clamp at 19.9 because we don't have room for 2x.x

				gAfr = min(19.9f, gAfr);

				int cCh = snprintf(aChz, DIM(aChz), "%.1f", gAfr);

				char chDec = aChz[cCh - 1];
				aChz[cCh - 1] = '\0';

				g_pCnvs->setCursor(47 - g_pCnvs->strPixelLen(aChz), 10);
				aChz[cCh - 1] = chDec;

				g_pCnvs->print(aChz);
			}

			g_pCnvs->setT3Font(&Exo_16_Bold_Italic);

			{
				// Draw IAT

				float gIatF = g_tactrix.GParam(PARAM_IatF);
				snprintf(aChz, DIM(aChz), "%d", int(roundf(gIatF)));
				g_pCnvs->setCursor(51 - g_pCnvs->strPixelLen(aChz), 214);
				g_pCnvs->print(aChz);
			}

			// Draw FBKC

			if (degFbkc < 0.0f && (gLoad > s_gLoadHighThreshold || degFbkc < s_degFbkcLowLoadThreshold))
			{
				g_pCnvs->setTextColor(s_iColorRed);
				snprintf(aChz, DIM(aChz), "%.2f", degFbkc);
			}
			else
			{
				snprintf(aChz, DIM(aChz), "%.2f", s_degFbkcHighLoadMin);
			}
			g_pCnvs->setCursor(301 - g_pCnvs->strPixelLen(aChz), 26);
			g_pCnvs->print(aChz);

			// Draw FBKC count

			snprintf(aChz, DIM(aChz), "%d", s_cFbkcEvent);
			g_pCnvs->setCursor(297 - g_pCnvs->strPixelLen(aChz), 48);
			g_pCnvs->print(aChz);
			g_pCnvs->setTextColor(s_iColorWhite);

			// Draw max boost

			snprintf(aChz, DIM(aChz), "%.1f", s_gBoostMax);
			g_pCnvs->setCursor(284 - g_pCnvs->strPixelLen(aChz), 214);
			g_pCnvs->print(aChz);

			// Draw gauge hands

			g_pCnvs->writeLineAntialiased(
						s_xBoostCenter,
						s_yBoostCenter,
						s_xBoostCenter - s_sNeedle * cosf(radBoostMax),
						s_yBoostCenter - s_sNeedle * sinf(radBoostMax),
						s_iColorRedMic,
						s_iColorRed);

			g_pCnvs->writeLineAntialiased(
						s_xBoostCenter,
						s_yBoostCenter,
						s_xBoostCenter - s_sNeedle * gCosBoost,
						s_yBoostCenter - s_sNeedle * gSinBoost,
						s_iColorWhiteMic,
						s_iColorWhite);

			// IAM alarm

			if (s_uIamMin < 1.0f)
			{
				g_pCnvs->fillCircle(
							s_xBoostCenter,
							s_yBoostCenter,
							s_sRadiusIamCircle,
							s_iColorIamAlert);
				g_pCnvs->drawCircle(
							s_xBoostCenter,
							s_yBoostCenter,
							s_sRadiusIamCircle,
							s_iColorBlack);
				g_pCnvs->drawCircle(
							s_xBoostCenter,
							s_yBoostCenter,
							s_sRadiusIamCircle + 1,
							s_iColorBlack);
				g_pCnvs->setTextColor(s_iColorBlack);
				const char * pChz = "IAM";
				g_pCnvs->setCursor(s_xBoostCenter - g_pCnvs->strPixelLen(pChz) / 2, s_yBoostCenter - 18);
				g_pCnvs->print(pChz);
				snprintf(aChz, DIM(aChz), "%0.2f", s_uIamMin);
				g_pCnvs->setCursor(s_xBoostCenter - g_pCnvs->strPixelLen(aChz) / 2, s_yBoostCenter + 2);
				g_pCnvs->print(aChz);
				g_pCnvs->setTextColor(s_iColorWhite);
			}

#if !TEST_OFFLINE
			if (fWriteToLog)
			{
				UpdateLog();
			}
			else if (s_fileLog)
			{
				s_fileLog.close();
			}
#endif // !TEST_OFFLINE
		}
		else
		{
			if (msCur > s_msSplashMax)
				DisplayStatus("Polling Error");

			g_tactrix.Disconnect();
			delay(s_dMsTimeoutDefault);
		}
	}
	else
	{
		if (g_userial.available())
		{
			Serial.print("Unhandled serial data: ");
			while (g_userial.available())
			{
				Serial.write(g_userial.read());
			}
			Serial.println();
		}
	}

	Serial.flush();

	UpdateTft();
}
