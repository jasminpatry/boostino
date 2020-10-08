// Boostino -- A Teensy 3.6 SSM logger and gauge.
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
#include <EEPROM.h>
#include <font_Exo-BoldItalic.h>
#include <ILI9341_t3.h>
#include <SD.h>
#include <XPT2046_Touchscreen.h>

#include "gaugeBg.h"
#include "gaugeFg.h"
#include "labelMph.h"
#include "labelAfr.h"



// Set to 1 to enable verbose logging.

#define DEBUG 1

static const bool s_fTraceComms = true;
static const bool s_fTraceTouch = false;

// Set to 1 to test without being plugged into the vehicle

#define TEST_OFFLINE 0



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
	PARAM_AtmPres,
	PARAM_ManifAbsPres,
	PARAM_BoostPsi,		// NOTE (jpatry) Calculated from PARAM_ManifAbsPres and PARAM_AtmPres
	PARAM_Iam,
	PARAM_CoolantTempF,	// BB (jpatry) Currently unused
	PARAM_Rpm,
	PARAM_Maf,
	PARAM_LoadGPerRev,	// NOTE (jpatry) Calculated from PARAM_Rpm and PARAM_Maf
	PARAM_IatF,
	PARAM_ThrottlePct,
	PARAM_SpeedMph,
	PARAM_IpwMs,
	PARAM_IdcPct,		// NOTE (jpatry) Calculated from PARAM_Rpm and PARAM_IpwMs
	PARAM_Afr1,
	PARAM_WidebandAfr,	// Read from analog input
	PARAM_TargetAfr,
	PARAM_MafVoltage,
	PARAM_LightSwitch,	// BB (jpatry) Always 1 unless parking brake is up, so not useful.

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
	"Atmospheric Pressure (psi)",						// PARAM_AtmPres
	"Manifold Absolute Pressure (psi)",					// PARAM_ManifAbsPres
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
	0x000023,		// PARAM_AtmPres
	0x00000D,		// PARAM_ManifAbsPres
	0x000000,		// PARAM_BoostPsi
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
	 1,		// PARAM_AtmPres
	 1,		// PARAM_ManifAbsPres
	 0,		// PARAM_BoostPsi
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
	PARAM_ManifAbsPres,	// 1 B
	PARAM_AtmPres,		// 1 B
	PARAM_Maf,			// 2 B
	PARAM_FlkcDeg,		// 1 B
	PARAM_Iam,			// 1 B
	PARAM_IatF,			// 1 B
	PARAM_ThrottlePct,	// 1 B
	PARAM_SpeedMph,		// 1 B
	PARAM_TargetAfr,	// 2 B
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
void TraceHex(bool f, u8 b)
{
	if (!f)
		return;

	if (b < 0x10)
		Trace(true, '0');

	Trace(true, b, HEX);
}
#else // !DEBUG
inline void TraceHex(bool f, u8 b)
{
}
#endif // !DEBUG

#if DEBUG
void TraceHex(bool f, const u8 * aB, u16 cB)
{
	if (!f || cB == 0)
		return;

	int iB = 0;
	for (;;)
	{
		u8 b = aB[iB];
		TraceHex(true, b);

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
	SSMCMD_BlockReadRequest		= 0xa0,
	SSMCMD_AddressReadRequest	= 0xa8,
	SSMCMD_AddressReadResponse	= 0xe8,
	SSMCMD_WriteBlockRequest	= 0xb0,
	SSMCMD_AddressWriteRequest	= 0xb8,
	SSMCMD_EcuInitRequest		= 0xbf,
	SSMCMD_EcuInitResponse		= 0xff,
};



u8 BSsmChecksum(const u8 * aB, u16 cB)
{
	u8 bSum = 0;
	for (int iB = 0; iB < cB; ++iB)
		bSum += aB[iB];

	return bSum;
}



static const u8 s_bSsmHdr = 0x80;



// SSM Packet

struct SSsm	// tag = ssm
{
	u8		m_bHdr;				// 0x80
	SSMID	m_bDst;				// Destination
	SSMID	m_bSrc;				// Source
	u8		m_cB;				// Data + checksum size
	SSMCMD	m_bCmd;				// Command/response byte
	u8		m_aB[0];			// Data (including checksum as last byte)

	u8			CBPacket() const
					{ return sizeof(*this) + m_cB; }

	u8			CBData() const
					{ return m_cB - 1; }

	const u8 *	PBData() const
					{ return m_aB; }

	u8 *		PBChecksum()
					{ return &m_aB[CBData()]; }

	u8			BChecksum() const
					{ return m_aB[CBData()]; }

	u8			BComputeChecksum() const;
	bool		FVerifyChecksum() const;
};

u8 SSsm::BComputeChecksum() const
{
	const u8 * aB = (const u8 *)this;
	int cB = CBPacket() - 1;
	return BSsmChecksum(aB, cB);
}

bool SSsm::FVerifyChecksum() const
{
	u8 bChecksum = BComputeChecksum();
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



// Wideband analog input

static const int s_cBitAnalog = 13;
static const float s_rAnalog = 1.0f / ((1 << s_cBitAnalog) - 1);



static const u32 s_dMsTimeoutDefault = 500;



// Connection State

enum CNXNS
{
	CNXNS_Idle,
	CNXNS_Polling,
};

class CConnectionMgr	// tag = cnxnmgr
{
public:
					CConnectionMgr()
					: m_cnxns(CNXNS_Idle),
					  m_rcvs(RCVS_Waiting),
					  m_aBRecv(),
					  m_cBRecv(0),
					  m_msRecv(0),
					  m_msReadResponse(0),
					  m_cBParamPoll(0),
					  m_mpParamGValue()
						{ ; }

					~CConnectionMgr()
						{ ; }

	void			Disconnect();
	bool			FTryStartPolling();
	bool			FTryUpdatePolling();
	CNXNS			Cnxns() const
						{ return m_cnxns; }
	float			GParam(PARAM param) const;

protected:

	// Receive state

	enum RCVS
	{
		RCVS_Waiting,
		RCVS_Header,
		RCVS_Data,
		RCVS_Checksum,
		RCVS_IncompleteMac,

		RCVS_MessageReceived = RCVS_IncompleteMac,
		RCVS_Error,

		RCVS_Max,
		RCVS_Min = 0,
		RCVS_Nil = -1,

	};

	void			SendCommand(const u8 * aB, u16 cB);
	void			SendCommand(const SSsm * pSsm);

	void			ResetReceive();
	void			Receive();

	const SSsm *	PSsmMessage() const;

	bool			FTryReceiveMessage(SSMCMD ssmcmd, u32 dMsTimeout = s_dMsTimeoutDefault);

	void			FlushIncoming();

	void			ProcessParamValue(PARAM param, u32 nValue);

	CNXNS			m_cnxns;						// Connection state
	RCVS			m_rcvs;							// K-Line receive state
	u8				m_aBRecv[4096];					//	... receive buffer
	u32				m_cBRecv;						// Total bytes in K-Line receive buffer
	u32				m_msRecv;						// Timestamp when data last received on K-Line
	u32				m_msReadResponse;				//	... AddressReadResponse
	int				m_cBParamPoll;					// Total number of bytes (addresses) being polled
	float			m_mpParamGValue[PARAM_Max];		// Latest values obtained from ECU
};

void CConnectionMgr::SendCommand(const u8 * aB, u16 cB)
{
	Trace(s_fTraceComms, "Sending ");
	Trace(s_fTraceComms, cB);
	Trace(s_fTraceComms, " bytes:\n\"");

	for (int iB = 0; iB < cB; ++iB)
	{
		u8 b = aB[iB];

		Trace(s_fTraceComms, char(b));

		Serial4.write(b);
	}

	Trace(s_fTraceComms, "\"\n");

	TraceHex(s_fTraceComms, aB, cB);
}

void CConnectionMgr::SendCommand(const SSsm * pSsm)
{
	SendCommand((const u8 *)pSsm, pSsm->CBPacket());
}

void CConnectionMgr::ResetReceive()
{
	m_rcvs = RCVS_Waiting;
	m_cBRecv = 0;
	m_msRecv = millis();
}

void CConnectionMgr::Receive()
{
	SSsm * pSsm = (SSsm *)&m_aBRecv[0];

	while (m_rcvs < RCVS_IncompleteMac)
	{
		if (!Serial4.available())
			return;

		u8 b = Serial4.read();

		Trace(s_fTraceComms, "Recv: ");
		TraceHex(s_fTraceComms, b);
		Trace(s_fTraceComms, "\n");

		m_msRecv = millis();

#if DEBUG
		RCVS rcvsPrev = m_rcvs;
#endif

		switch (m_rcvs)
		{
		case RCVS_Waiting:
			{
				m_cBRecv = 0;
				if (b == s_bSsmHdr)
				{
					m_aBRecv[0] = b;
					++m_cBRecv;
					m_rcvs = RCVS_Header;
				}
			}
			break;

		case RCVS_Header:
			{
				ASSERT(m_cBRecv < sizeof(SSsm));
				m_aBRecv[m_cBRecv] = b;
				++m_cBRecv;
				if (m_cBRecv == sizeof(SSsm))
				{
					if (pSsm->CBPacket() > s_cBSsmMax)
					{
						m_rcvs = RCVS_Error;

						Trace(s_fTraceComms, "ERROR: Received a packet that exceeds maximum size\n");
					}
					else if (pSsm->CBData() == 0)
					{
						m_rcvs = RCVS_Checksum;
					}
					else
					{
						m_rcvs = RCVS_Data;
					}
				}
			}
			break;

		case RCVS_Data:
			{
				ASSERT(m_cBRecv >= sizeof(SSsm));
				ASSERT(m_cBRecv < sizeof(SSsm) + pSsm->CBData());
				m_aBRecv[m_cBRecv] = b;
				++m_cBRecv;
				if (m_cBRecv == sizeof(SSsm) + pSsm->CBData())
					m_rcvs = RCVS_Checksum;
			}
			break;

		case RCVS_Checksum:
			{
				m_aBRecv[m_cBRecv] = b;
				++m_cBRecv;

				Trace(s_fTraceComms, "Received SSM message:\n");
				TraceHex(s_fTraceComms, (u8 *)pSsm, pSsm->CBPacket());

				if (pSsm->FVerifyChecksum())
				{
					m_rcvs = RCVS_MessageReceived;
				}
				else
				{
					m_rcvs = RCVS_Error;

					Trace(s_fTraceComms, "ERROR: Invalid checksum\n");
				}

				Trace(s_fTraceComms, "\n");

				return;
			}
			break;

		default:
			ASSERT(false);
		}

#if DEBUG
		if (m_rcvs != rcvsPrev)
		{
			Trace(s_fTraceComms, "RCVS changed from ");
			Trace(s_fTraceComms, rcvsPrev);
			Trace(s_fTraceComms, " to ");
			Trace(s_fTraceComms, m_rcvs);
			Trace(s_fTraceComms, "\n");
		}
#endif // DEBUG
	}
}

const SSsm * CConnectionMgr::PSsmMessage() const
{
	if (m_rcvs != RCVS_MessageReceived)
		return nullptr;

	return (const SSsm *)&m_aBRecv[0];
}

bool CConnectionMgr::FTryReceiveMessage(SSMCMD ssmcmd, u32 dMsTimeout)
{
	for (;;)
	{
		Receive();
		const SSsm * pSsm = PSsmMessage();
		if (pSsm)
		{
			if (pSsm->m_bSrc == SSMID_Ecu &&
				pSsm->m_bDst == SSMID_Tool &&
				pSsm->m_bCmd == ssmcmd)
			{
				// Matching message found

				Trace(s_fTraceComms, "FTryReceiveMessage() received matching message\n");

				return true;
			}
			else
			{
				// Wrong kind of message; reset and look for another

				Trace(s_fTraceComms, "FTryReceiveMessage() received wrong message, ignoring...\n");

				ResetReceive();
			}
		}
		else
		{
			// Still waiting for a complete message

			if (millis() - m_msRecv >= dMsTimeout)
			{
				// Too long since we've received anything

				Trace(s_fTraceComms, "FTryReceiveMessage() timed out\n");

				return false;
			}
			else
			{
				// Wait, then try again

				delayMicroseconds(100);
			}
		}
	}
}

void CConnectionMgr::FlushIncoming()
{
	ResetReceive();

	Trace(true, "Flushing incoming data...\n");

	while (Serial4.available())
	{
		u8 b = Serial4.read();

		TraceHex(true, b);
	}

	Trace(true, "\nFlushIncoming done.\n");
}

bool CConnectionMgr::FTryStartPolling()
{
	if (m_cnxns != CNXNS_Idle)
		return false;

#if !TEST_OFFLINE
	FlushIncoming();

	// Send SSM init sequence

	const u8 aBSsmInit[] = { s_bSsmHdr, SSMID_Ecu, SSMID_Tool, 0x01, SSMCMD_EcuInitRequest, 0x40 };
	SendCommand(aBSsmInit, sizeof(aBSsmInit));

	if (!FTryReceiveMessage(SSMCMD_EcuInitResponse))
	{
		Trace(true, "Failed to receive EcuInitResponse message\n");

		return false;
	}

	// BB (jpatry) Validate SSM reply?

	Trace(true, "SSM Init Reply Received\n");

	// Issue address read request (A8)

	m_cBParamPoll = 0;
	for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
		m_cBParamPoll += s_mpParamCB[s_aParamPoll[iParamPoll]];

	int cBSsmRead = sizeof(SSsm) + 2; // header + checksum + PP byte (single response/respond until interrupted)
	cBSsmRead += s_cBSsmAddr * m_cBParamPoll;

	if (cBSsmRead > s_cBSsmMax)
	{
		Trace(true, "SSM packet overflow (trying to read too many params)\n");
		return false;
	}

	// Send SSM packet to start polling:
	//		0x80, 0x10, 0xf0, <data size - 5>, 0xa8, <PP byte: 0x00 or 0x01>, <address list...>, <checksum>
	//	PP byte is 0 if data should be sent once or 1 if it should be sent until interrupted.

	u8 aBReadRequest[s_cBSsmMax];
	SSsm * pSsm = (SSsm *)&aBReadRequest[0];
	pSsm->m_bHdr = s_bSsmHdr;
	pSsm->m_bDst = SSMID_Ecu;
	pSsm->m_bSrc = SSMID_Tool;
	pSsm->m_cB = cBSsmRead - sizeof(SSsm);
	pSsm->m_bCmd = SSMCMD_AddressReadRequest;
	pSsm->m_aB[0] = 0x1;	// Continuous polling
	int iB = 1;
	for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
	{
		PARAM param = s_aParamPoll[iParamPoll];
		u32 nAddr = s_mpParamABAddr[param];

		for (int iBRead = 0; iBRead < s_mpParamCB[param]; ++iBRead)
		{
			pSsm->m_aB[iB + 0] = (nAddr & 0xff0000) >> 16;
			pSsm->m_aB[iB + 1] = (nAddr & 0x00ff00) >> 8;
			pSsm->m_aB[iB + 2] = (nAddr & 0x0000ff);
			CASSERT(s_cBSsmAddr == 3);
			iB += s_cBSsmAddr;
			nAddr += 1;
		}
	}

	ASSERT(pSsm->CBPacket() == cBSsmRead);

	*pSsm->PBChecksum() = pSsm->BComputeChecksum();

	FlushIncoming();

	SendCommand(pSsm);
#endif // !TEST_OFFLINE

	m_cnxns = CNXNS_Polling;

	return true;
}

bool CConnectionMgr::FTryUpdatePolling()
{
	if (m_cnxns != CNXNS_Polling)
		return false;

	u32 msCur = millis();

#if !TEST_OFFLINE
	// Local copy of last SSM read response message

	u8 aBReadResponse[s_cBSsmMax];
	SSsm * pSsmReadResponse = (SSsm *)(&aBReadResponse[0]);
	bool fReceivedReadResponse = false;

	// Drain the pipe of all accumulated messages

	static const u32 s_dMsTimeoutReadResponse = 10;
	while (FTryReceiveMessage(SSMCMD_AddressReadResponse, (fReceivedReadResponse) ? 0 : s_dMsTimeoutReadResponse))
	{
		const SSsm * pSsm = PSsmMessage();
		memcpy(pSsmReadResponse, PSsmMessage(), pSsm->CBPacket());
		fReceivedReadResponse = true;
		m_msReadResponse = msCur;

		// We've copied the message, so reset so we can start receiving the next one

		ResetReceive();
	}

	// Check if we've timed out

	static const int s_dMsReplyStartTimeout = 300;
	if (msCur - m_msReadResponse > s_dMsReplyStartTimeout)
	{
		Trace(true, "Timed out waiting to receive AddressReadResponse message\n");
		return false;
	}

	if (fReceivedReadResponse)
	{
		const u8 * pBData = pSsmReadResponse->PBData();

		if (pSsmReadResponse->CBData() != m_cBParamPoll)
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

		// Done with SSM data

		pSsmReadResponse = nullptr;
	}
#else // TEST_OFFLINE
	// Dummy boost values to test gauge
	m_mpParamGValue[PARAM_AtmPres] = 14.7f;

	float gBoost = -10.0f + 30.0f * (-sinf(msCur / 1000.0f) * 0.5f + 0.5f);
	m_mpParamGValue[PARAM_ManifAbsPres] = gBoost + m_mpParamGValue[PARAM_AtmPres];

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
#endif // TEST_OFFLINE

	// Update calculated params

	m_mpParamGValue[PARAM_BoostPsi] = m_mpParamGValue[PARAM_ManifAbsPres] - m_mpParamGValue[PARAM_AtmPres];
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

	// Update log

	SLogEntry logent;
	logent.m_msTimestamp = msCur;
	for (int iParamLog = 0; iParamLog < s_cParamLog; ++iParamLog)
		logent.m_mpIParamLogGValue[iParamLog] = GParam(s_aParamLog[iParamLog]);
	g_loghist.WriteLogEntry(logent);

	return true;
}

float CConnectionMgr::GParam(PARAM param) const
{
	return m_mpParamGValue[param];
}

void CConnectionMgr::ProcessParamValue(PARAM param, u32 nValue)
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

	case PARAM_AtmPres:
	case PARAM_ManifAbsPres:
		ng.m_g = float(nValue) * 37.0f / 255.0f;
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

void CConnectionMgr::Disconnect()
{
	ResetReceive();

	m_cnxns = CNXNS_Idle;
}

CConnectionMgr g_cnxnmgr;



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



class CClock	// tag = clock
{
public:
			CClock()
			: m_eus(0),
			  m_dT(0.0f)
				{ ; }

	void	Reset()
				{
					m_eus = 0;
					m_dT = 0.0f;
				}

	void	Update();

	float	DT() const
				{ return m_dT; }

protected:
	elapsedMicros	m_eus;
	float			m_dT;
};

void CClock::Update()
{
	u32 nMicros = m_eus;
	m_eus = 0;
	m_dT = nMicros * 1.0e-6f;
}

CClock g_clock;



enum BRIGHT : s8
{
	BRIGHT_Off,
	BRIGHT_Low,
	BRIGHT_High,

	BRIGHT_Max,
	BRIGHT_Min = 0,
	BRIGHT_Nil = -1,
};

static const float s_mpBrightGLogBrightness[] =
{
	-6.91f,	// BRIGHT_Off
	-1.66f,	// BRIGHT_Low
	0.0f,	// BRIGHT_High
};
CASSERT(DIM(s_mpBrightGLogBrightness) == BRIGHT_Max);

class CScreenFader	// tag = scrnfdr
{
public:
			CScreenFader()
			: m_gLogTgt(s_mpBrightGLogBrightness[BRIGHT_Off]),
			  m_gLogCur(s_mpBrightGLogBrightness[BRIGHT_Off])
				{ ; }

	void	Update();

	void	SetBrightnessTarget(BRIGHT bright)
				{ m_gLogTgt = s_mpBrightGLogBrightness[bright]; }

	void	SetBrightness(BRIGHT bright)
				{
					SetBrightnessTarget(bright);
					m_gLogCur = m_gLogTgt;
				}

protected:
	float	m_gLogTgt;
	float	m_gLogCur;
};

void CScreenFader::Update()
{
	static const float s_dTEma = 0.2f;

	float uLerp = min(1.0f, g_clock.DT() / s_dTEma);

	m_gLogCur = uLerp * m_gLogTgt + (1.0f - uLerp) * m_gLogCur;

	analogWrite(s_nPinTftLedPwm, roundf(expf(m_gLogCur) * 255.0f));
}

CScreenFader g_scrnfdr;



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



// Persisted settings

static const u32 s_nNvramMagic = 'BSTI';

enum NVRAMVER : u8
{
	NVRAMVER_Initial = 1,

	NVRAMVER_Max,
	NVRAMVER_Current = NVRAMVER_Max - 1,
};

struct SNvram	// tag = nvram
{
				SNvram()
				: m_nvramver(NVRAMVER_Current),
				  m_bright(BRIGHT_Low),
				  m_fDisplayMph(false)
					{ ; }

	void		Read();
	void		Write();

	NVRAMVER	m_nvramver;
	BRIGHT		m_bright;
	bool		m_fDisplayMph;
};

void SNvram::Read()
{
	union
	{
		u32 m_nMagic;
		u8	m_aB[4];
	} u;

	for (int iB = 0; iB < sizeof(u32); ++iB)
	{
		u.m_aB[iB] = EEPROM.read(iB);
	}

	if (u.m_nMagic != s_nNvramMagic)
	{
		*this = SNvram();
		return;
	}

	u8 * aB = (u8 *)this;
	for (int iB = 0; iB < sizeof(*this); ++iB)
	{
		aB[iB] = EEPROM.read(sizeof(u32) + iB);
	}
}

void SNvram::Write()
{
	union
	{
		u32 m_nMagic;
		u8	m_aB[4];
	} u;

	u.m_nMagic = s_nNvramMagic;

	for (int iB = 0; iB < sizeof(u32); ++iB)
	{
		EEPROM.update(iB, u.m_aB[iB]);
	}

	u8 * aB = (u8 *)this;
	for (int iB = 0; iB < sizeof(*this); ++iB)
	{
		EEPROM.update(sizeof(u32) + iB, aB[iB]);
	}
}

SNvram g_nvram;



// Arduino setup() func

void setup()
{
	// Turn screen LED off

	g_scrnfdr.Update();

#if DEBUG
	while (!Serial && (millis() < 5000))
		(void) 0; // wait for Arduino Serial Monitor
#endif // DEBUG

	Serial.print("\n\nBoostino 0.1  Copyright (C) 2019, 2020  Jasmin Patry\n");
	Serial.print("This program comes with ABSOLUTELY NO WARRANTY.\n");
    Serial.print("This is free software, and you are welcome to redistribute it\n");
    Serial.print("under certain conditions. For details see\n");
	Serial.print("https://github.com/jasminpatry/boostino/blob/master/COPYING\n\n");

	Trace(true, "Initialize Serial4\n");
	Serial4.begin(4800, SERIAL_8N1);

	while (!Serial4)
	{
		Trace(true, "Waiting for Serial4\n");
		delay(1000);
	}

	Trace(true, "Serial4 initialized\n");

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

	// Teensy has 13-bit ADC (defaults to 10-bit to be Arduino-compatible)

	analogReadResolution(s_cBitAnalog);

	g_clock.Reset();

	// Read NVRAM

	g_nvram.Read();

	// Start fading in screen LED

	g_scrnfdr.SetBrightnessTarget(g_nvram.m_bright);
}



// Arduino loop() func

void loop()
{
	g_clock.Update();

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

	// Check for screen brightness inputs

	if (fTouchRelease && abs(g_xTouch - 160) <= 30)
	{
		if (abs(g_yTouch - 30) <= 30)
		{
			if (g_nvram.m_bright != BRIGHT_High)
			{
				g_scrnfdr.SetBrightnessTarget(BRIGHT_High);
				g_nvram.m_bright = BRIGHT_High;
				g_nvram.Write();
			}
		}
		else if (abs(g_yTouch - 210) <= 30)
		{
			if (g_nvram.m_bright != BRIGHT_Low)
			{
				g_scrnfdr.SetBrightnessTarget(BRIGHT_Low);
				g_nvram.m_bright = BRIGHT_Low;
				g_nvram.Write();
			}
		}
	}

	if (g_cnxnmgr.Cnxns() == CNXNS_Idle)
	{
		if (msCur > s_msSplashMax)
			DisplayStatus("Polling ECU...");

		if (!g_cnxnmgr.FTryStartPolling())
		{
			g_cnxnmgr.Disconnect();
			delay(s_dMsTimeoutDefault);
		}
	}

	if (g_cnxnmgr.Cnxns() == CNXNS_Polling)
	{
		if (g_cnxnmgr.FTryUpdatePolling())
		{
			// Update data, handle touch events, draw gauge and log if necessary

			g_pCnvs->fillScreen(s_iColorBlack);

			static float s_gBoostMax = -1000.0f;

			// Check for max boost reset event

			if (fTouchRelease && g_xTouch >= 245 && g_yTouch >= 195)
				s_gBoostMax = -1000.0f;

			float gBoost = g_cnxnmgr.GParam(PARAM_BoostPsi);
			s_gBoostMax = max(gBoost, s_gBoostMax);

			static float s_degFbkcAnyMin = 0.0f;
			static float s_degFbkcHighLoadMin = 0.0f;
			static float s_degFbkcPrev = 0.0f;
			static u32 s_msFbkcEventLast = 0;
			static int s_cFbkcEvent = 0;
			float degFbkc = g_cnxnmgr.GParam(PARAM_FbkcDeg);
			float gLoad = g_cnxnmgr.GParam(PARAM_LoadGPerRev);

			// Check for min FBKC reset event

			if (fTouchRelease && g_xTouch >= 240 && g_yTouch <= 55)
			{
				s_degFbkcHighLoadMin = 0.0f;
				s_cFbkcEvent = 0;
			}

			if (fTouchRelease && g_xTouch <= 100 && g_yTouch <= 40)
			{
				g_nvram.m_fDisplayMph = !g_nvram.m_fDisplayMph;
			}

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
			float degFlkc = g_cnxnmgr.GParam(PARAM_FlkcDeg);
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

			float uIam = g_cnxnmgr.GParam(PARAM_Iam);
			s_uIamMin = min(uIam, s_uIamMin);

			// Check if doing a WOT pull

			static const int s_cWotThreshold = 20;
			CASSERT(s_cWotThreshold < s_cEntryLogHistory);

			static int s_cWot = 0;
			float gThrottlePct = g_cnxnmgr.GParam(PARAM_ThrottlePct);
			if (gThrottlePct > 95.0f)
				++s_cWot;
			else
				s_cWot = 0;

			float gSpeedMph = g_cnxnmgr.GParam(PARAM_SpeedMph);

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

			if (g_nvram.m_fDisplayMph)
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

			if (g_nvram.m_fDisplayMph)
			{
				// Draw speed

				snprintf(aChz, DIM(aChz), "%d", int(roundf(gSpeedMph)));
				g_pCnvs->setCursor(66 - g_pCnvs->strPixelLen(aChz), 10);
				g_pCnvs->print(aChz);
			}
			else
			{
				// Draw AFR and align at decimal point

				float gAfr = g_cnxnmgr.GParam(PARAM_WidebandAfr);

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

				float gIatF = g_cnxnmgr.GParam(PARAM_IatF);
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

			g_cnxnmgr.Disconnect();
			delay(s_dMsTimeoutDefault);
		}
	}
	else
	{
#if DEBUG
		if (Serial4.available())
		{
			Serial.print("Unhandled serial data: ");
			while (Serial4.available())
			{
				Serial.print(Serial4.read(), HEX);
				Serial.write(' ');
			}
			Serial.println();
		}
#endif // DEBUG
	}

#if DEBUG
	Serial.flush();
#endif // DEBUG

	g_scrnfdr.Update();

	UpdateTft();
}
