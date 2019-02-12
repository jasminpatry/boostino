// Tactsy -- A Simple Tactrix OP 2.0 <-> Teensy 3.6 Interface. Pronounced "taxi".
// Copyright (c) 2019 Jasmin Patry

#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <USBHost_t36.h>
#include <Wire.h>



// Set to 1 to enable verbose USB serial logging.

#define DEBUG 0

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



// Available ECU parameters

enum PARAM
{
	PARAM_FbkcDeg,
	PARAM_FlkcDeg,
	PARAM_BoostPsi,
	PARAM_Iam,
	PARAM_CoolantTempF,
	PARAM_LoadGPerRev,
	PARAM_Rpm,
	PARAM_IatF,
	PARAM_ThrottlePct,

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
	"FBKC (deg)",				// PARAM_FbkcDeg
	"FLKC (deg)",				// PARAM_FlkcDeg
	"Boost (psi)",				// PARAM_BoostPsi
	"Ignition Advance Mult.",	// PARAM_Iam
	"Coolant Temp (F)",			// PARAM_CoolantTempF
	"Load (g/rev)",				// PARAM_LoadGPerRev
	"RPM",						// PARAM_Rpm
	"Intake Air Temp (F)",		// PARAM_IatF
	"Throttle (%)",				// PARAM_ThrottlePct
};
CASSERT(DIM(s_mpParamPChz) == PARAM_Max);

static const u8 s_cBSsmAddr = 3;
static const u8 s_mpParamABAddr[][s_cBSsmAddr] =
{
	{ 0xff, 0x81, 0xfc },		// PARAM_FbkcDeg
	{ 0x00, 0x01, 0x99 },		// PARAM_FlkcDeg
	{ 0x00, 0x00, 0x24 },		// PARAM_BoostPsi
	{ 0x00, 0x00, 0xf9 },		// PARAM_Iam
	{ 0x00, 0x00, 0x08 },		// PARAM_CoolantTempF
	{ 0xff, 0x64, 0x10 },		// PARAM_LoadGPerRev
	{ 0x00, 0x00, 0x0e },		// PARAM_Rpm
	{ 0x00, 0x00, 0x12 },		// PARAM_IatF
	{ 0x00, 0x00, 0x15 },		// PARAM_ThrottlePct
};
CASSERT(DIM(s_mpParamABAddr) == PARAM_Max);

static const u8 s_mpParamCB[] =
{
	 4,							// PARAM_FbkcDeg
	 1,							// PARAM_FlkcDeg
	 1,							// PARAM_BoostPsi
	 1,							// PARAM_Iam
	 1,							// PARAM_CoolantTempF
	 4,							// PARAM_LoadGPerRev
	 2,							// PARAM_Rpm
	 1,							// PARAM_IatF
	 1,							// PARAM_ThrottlePct
};
CASSERT(DIM(s_mpParamCB) == PARAM_Max);



// ECU Parameters to Poll

static const PARAM s_aParamPoll[] =
{
	PARAM_FbkcDeg,
	PARAM_LoadGPerRev,
	PARAM_Rpm,
	PARAM_FlkcDeg,
	PARAM_Iam,
	PARAM_BoostPsi,
	PARAM_CoolantTempF,
	PARAM_IatF,
	PARAM_ThrottlePct,
};
static const u8 s_cParamPoll = DIM(s_aParamPoll);



// ECU Parameters to Log

static const PARAM s_aParamLog[] =
{
	PARAM_LoadGPerRev,
	PARAM_Rpm,
	PARAM_FbkcDeg,
	PARAM_FlkcDeg,
	PARAM_BoostPsi,
	PARAM_IatF,
	PARAM_ThrottlePct,
	PARAM_Iam,
};
static const u8 s_cParamLog = DIM(s_aParamLog);



// Debugging support

inline void Trace(const char * pChz)
{
#if DEBUG
	Serial.print(pChz);
#endif // DEBUG
}

inline void Trace(u8 b, int mode = DEC)
{
#if DEBUG
	Serial.print(b, mode);
#endif // DEBUG
}

#if DEBUG
void Trace(char ch)
{
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
inline void Trace(char ch)
{
}
#endif // !DEBUG

inline void Trace(u16 uN, int mode = DEC)
{
#if DEBUG
	Serial.print(uN, mode);
#endif // DEBUG
}

inline void Trace(s16 n, int mode = DEC)
{
#if DEBUG
	Serial.print(n, mode);
#endif // DEBUG
}

inline void Trace(u32 uN, int mode = DEC)
{
#if DEBUG
	Serial.print(uN, mode);
#endif // DEBUG
}

inline void Trace(int n, int mode = DEC)
{
#if DEBUG
	Serial.print(n, mode);
#endif // DEBUG
}

inline void Trace(float g, int cDigit = 2)
{
#if DEBUG
	Serial.print(g, cDigit);
#endif // DEBUG
}

#if DEBUG
void TraceHex(const u8 * aB, u16 cB)
{
	int iB = 0;
	for (;;)
	{
		u8 b = aB[iB];
		if (b < 0x10)
			Trace('0');
		Trace(b, HEX);

		++iB;
		if (iB >= cB)
			break;

		Trace(' ');

		if (!(iB & 0xf))
			Trace("\n");
		else if (!(iB & 0x3))
			Trace("| ");
	}
	Trace("\n");
}
#else // !DEBUG
inline void TraceHex(const u8 * aB, u16 cB)
{
}
#endif // !DEBUG



// BB (jasminp) Assigning __LINE__ to nAssertLine is necessary to get rid of warning about 'large integer implicitly
//	truncated to unsigned type'.

#define ASSERT(f)								\
	do											\
	{											\
		if (!(f))								\
		{										\
			Trace("Assert failed (");			\
			Trace(__FILE__);					\
			Trace(":");							\
			u32 nAssertLine = __LINE__;			\
			Trace(nAssertLine);					\
			Trace("): ");						\
			Trace(#f);							\
			Trace("\n");						\
		}										\
	} while(0);



// USB host configuration

USBHost g_uhost;
USBSerial g_userial(g_uhost);
bool g_fUserialActive = false;

static const u16 s_nIdTactrix = 0x0403;
static const u16 s_nIdOpenPort20 = 0xcc4c;
static const u32 s_msTimeout = 2000;



// NOTE (jasminp) J2534 protocol reverse-engineered using Wireshark, with help from this reference:
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
	u8	m_nProtocol;			// E.g. 3 for ISO9141
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



// SSM Source/Destination IDs

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
		Trace("SSM checksum error\n");
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

	int				CBReceive(u32 msTimeout);
	void			ResetReceive();
	bool			FTryReceiveMessage(const char * pChz);
	bool			FMustReceiveMessage(const char * pChz);
	int				CBMessage()
						{ return m_iBRecv - m_iBRecvPrev; }
	u8 *			PBMessage()
						{ return &m_aBRecv[m_iBRecvPrev]; }
	int				CBRemaining()
						{ return m_cBRecv - m_iBRecv; }
	u8 *			PBRemaining()
						{ return &m_aBRecv[m_iBRecv]; }

	bool			FTryReceiveMessage(MSGK msgk);
	bool			FMustReceiveMessage(MSGK msgk);

	void			FlushIncoming();

	void			ProcessParamValue(PARAM param, u32 nValue);

	TACTRIXS		m_tactrixs;						// Current state
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
		Trace("SendCommand: Can't send, not connected\n");
		m_tactrixs = TACTRIXS_Disconnected;
		return;
	}

	Trace("Sending ");
	Trace(cB);
	Trace(" bytes:\n\"");

	for (int iB = 0; iB < cB; ++iB)
	{
		u8 b = aB[iB];

		Trace(char(b));

		g_userial.write(b);
	}

	Trace("\"\n");

	TraceHex(aB, cB);
}

void CTactrix::SendCommand(const char * pChz)
{
	SendCommand((const u8 *)pChz, strlen(pChz));
}

int CTactrix::CBReceive(u32 msTimeout)
{
	ResetReceive();

	if (!g_userial)
	{
		Trace("CBReceive: Can't receive, no serial USB device connected\n");
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
			Trace("Received ");
			Trace(m_cBRecv);
			Trace(" bytes:\n\"");
			for (int iB = 0; iB < m_cBRecv; ++iB)
				Trace(char(m_aBRecv[iB]));
			Trace("\"\n");

			TraceHex(m_aBRecv, m_cBRecv);
#endif // DEBUG

			return m_cBRecv;
		}

		if (msTimeout > 0 && millis() - msStart > msTimeout)
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

bool CTactrix::FTryReceiveMessage(const char * pChz)
{
	int cCh = strlen(pChz);

	if (CBRemaining() == 0)
	{
		if (CBReceive(s_msTimeout) == 0)
		{
			Trace("FTryReceiveMessage timed out waiting for reply \"");
			for (int iCh = 0; iCh < cCh; ++iCh)
				Trace(pChz[iCh]);
			Trace("\"\n");
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

bool CTactrix::FMustReceiveMessage(const char * pChz)
{
	if (!FTryReceiveMessage(pChz))
	{
		ResetReceive();

#if DEBUG
		if (CBRemaining())
		{
			Trace("FMustReceiveMessage got unexpected reply (expected \"");
			int cCh = strlen(pChz);
			for (int iCh = 0; iCh < cCh; ++iCh)
				Trace(pChz[iCh]);
			Trace("\")\n");
		}
#endif // DEBUG

		return false;
	}

	return true;
}

bool CTactrix::FTryReceiveMessage(MSGK msgk)
{
	if (CBRemaining() == 0)
	{
		if (CBReceive(s_msTimeout) == 0)
		{
			Trace("Timed out waiting for message type ");
			Trace(s_mpMsgkPChz[msgk]);
			Trace("\n");
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
		Trace("FTryReceiveMessage called with invalid msgk\n");
		return false;
	}

	if (cBMsg > cBRem)
	{
		Trace("FTryReceiveMessage received malformed message (cBMsg > cBRem)\n");
		return false;
	}

	m_iBRecvPrev = m_iBRecv;
	m_iBRecv += cBMsg;

	return true;
}

bool CTactrix::FMustReceiveMessage(MSGK msgk)
{
	if (!FTryReceiveMessage(msgk))
	{
		ResetReceive();

		Trace("FMustReceiveMessage failed for message type ");
		Trace(s_mpMsgkPChz[msgk]);
		Trace("\n");

		return false;
	}

	return true;
}

void CTactrix::FlushIncoming()
{
	Trace("Flushing incoming data...\n");

	while (g_userial.available())
	{
		u8 b = g_userial.read();

		Trace(char(b));
	}

	Trace("FlushIncoming done.\n");
}

bool CTactrix::FTryConnect()
{
	if (m_tactrixs != TACTRIXS_Disconnected)
	{
		Disconnect();
	}

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
	// BB (jasminp) Redundant?

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

	m_tactrixs = TACTRIXS_Connected;

	return true;
}

bool CTactrix::FTryStartPolling()
{
	if (m_tactrixs != TACTRIXS_Connected)
		return false;

	// Equivalent to J2534 PassThruWriteMsgs; send SSM init sequence

	const u8 aBSsmInit[] = { "att3 6 0 2000000 11\r\n\x80\x10\xf0\x01\xbf\x40" };
	SendCommand(aBSsmInit, sizeof(aBSsmInit) - 1);

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
#endif // TEST_OFFLINE

	// Acknowledgement

	if (!FMustReceiveMessage("aro 11\r\n"))
		return false;

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
				Trace("Overflow of SSM init reply buffer\n");
				return false;
			}
			else
			{
				memcpy(&aBSsmInitReply[cBSsmInitReply], pAr->PBPayload(), cBPayload);
				cBSsmInitReply += cBPayload;
			}
		}

		// BB (jasminp) Validate SSM reply?

		Trace("SSM Init Reply: ");
		TraceHex(aBSsmInitReply, cBSsmInitReply);
		Trace("\n");
	}
#endif // TEST_OFFLINE

	{
		// Issue address read request (A8)

		m_cBParamPoll = 0;
		for (int iParamPoll = 0; iParamPoll < s_cParamPoll; ++iParamPoll)
			m_cBParamPoll += s_mpParamCB[s_aParamPoll[iParamPoll]];

		int cBSsmRead = 7; // header + checksum + PP byte (single response/respond until interrupted)
		cBSsmRead += s_cBSsmAddr * m_cBParamPoll;

		if (cBSsmRead > s_cBSsmMax)
		{
			Trace("SSM packet overflow (trying to read too many params)\n");
			return false;
		}

		// J2534 header is at most 23 bytes for this request

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
			u32 nAddr =
				s_mpParamABAddr[param][0] << 16 |
				s_mpParamABAddr[param][1] << 8 |
				s_mpParamABAddr[param][2];

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
				Trace("Loopback error\n");
				return false;
			}

			iBLoopback += cBLoopbackFrag;

			if (!FMustReceiveMessage(MSGK_LoopbackEnd))
				return false;
		}
#endif // !TEST_OFFLINE

		if (!FMustReceiveMessage("aro 12\r\n"))
			return false;
	}

	m_tactrixs = TACTRIXS_Polling;

	return true;
}

bool CTactrix::FTryUpdatePolling()
{
	if (m_tactrixs != TACTRIXS_Polling)
		return false;

#if !TEST_OFFLINE
	bool fReceivedReply = false;
	for (int i = 0; i < 10; ++i)
	{
		if (FTryReceiveMessage(MSGK_ReplyStart))
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
				Trace("Timed out while waiting to receive AddressReadResponse message\n");
				return false;
			}
		}
	}

	if (!fReceivedReply)
	{
		Trace("Failed to received AddressReadResponse message after 10 tries\n");
		return false;
	}

	const SJ2534Ar * pAr = (const SJ2534Ar *)PBMessage();
	const SSsm * pSsm = (const SSsm *)pAr->PBPayload();
	const u8 * pBData = pSsm->PBData();

	if (pSsm->CBData() != m_cBParamPoll)
	{
		Trace("Received incorrect number of bytes in AddressReadResponse\n");
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

	ASSERT(pBData - pSsm->PBData() == pSsm->CBData());

	if (!FMustReceiveMessage(MSGK_ReplyEnd))
		return false;

#else // TEST_OFFLINE
	// Dummy boost values to test gauge

	float gBoost = -10.0f + 35.0f * (-sinf(millis() / 1000.0f) * 0.5f + 0.5f);
	m_mpParamGValue[PARAM_BoostPsi] = gBoost;

	float uIam = 1.0f;
	m_mpParamGValue[PARAM_Iam] = uIam;

	float degFbkc = ((millis() & 0x1fff) < 0x100) ? -1.4f : 0.0f;
	m_mpParamGValue[PARAM_FbkcDeg] = degFbkc;

	float degFlkc = ((millis() & 0x1fff) - 0xf0 < 0x100) ? -1.4f : 0.0f;
	m_mpParamGValue[PARAM_FlkcDeg] = degFlkc;
#endif // TEST_OFFLINE

	// Update log

	SLogEntry logent;
	logent.m_msTimestamp = millis();
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
	case PARAM_LoadGPerRev:
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

	case PARAM_ThrottlePct:
		ng.m_g = float(nValue) * (100.0f / 255.0f);
		break;

	default:
		CASSERT(PARAM_ThrottlePct == PARAM_Max - 1); // Compile-time reminder to add new params to switch
		ASSERT(false);
	}

	Trace(s_mpParamPChz[param]);
	Trace(": ");
	Trace(ng.m_g);
	Trace("\n");

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



// OLED Display Configuration
// NOTE (jasminp) At text size = 1, display can fit 4 lines of 21 characters

static const int s_dXScreen = 128;
static const int s_dYScreen = 32;
static const int s_nPinOledReset = -1;	// share Arduino reset pin
static const u8 s_bI2cAddrOled = 0x3c;
static const u32 s_msDisplayWarning = 5000;	// how long to display warnings (in ms)

Adafruit_SSD1306 g_oled(s_dXScreen, s_dYScreen, &Wire, s_nPinOledReset);



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



void DisplayStatus(const char * pChz)
{
	g_oled.clearDisplay();
	g_oled.setCursor(8, 12);
	g_oled.println(pChz);
	g_oled.display();
}



// SD Card Configuration

static const int s_nPinSdChipSelect = BUILTIN_SDCARD;
static const char * s_pChzLogPrefix = "knock";
static const char * s_pChzLogSuffix = ".log";
static const int s_cLogFileMax = 1000;
static int s_nLogSuffix = -1;
static File s_fileLog;



static const u32 s_msSplashMax = 5000;
static u32 s_msSplash = 0;



void setup()
{
#if DEBUG
	while (!Serial && (millis() < 5000))
		(void) 0; // wait for Arduino Serial Monitor
#endif // DEBUG

	Serial.println("\n\nTactsy 0.1");

	// Initialize OLED

	if (!g_oled.begin(SSD1306_SWITCHCAPVCC, s_bI2cAddrOled))
		Serial.println("SSD1306 allocation failed");

	// Clear the screen

	g_oled.clearDisplay();
	g_oled.setTextSize(1);
	g_oled.setTextColor(WHITE);
	g_oled.cp437(true);
	g_oled.drawBitmap(13, 2, s_aBSti, s_dXSti, s_dYSti, WHITE);
	g_oled.drawBitmap(97, 23, s_aBGfxguy, s_dXGfxguy, s_dYGfxguy, WHITE);
	g_oled.display();

	s_msSplash = millis();

	// Initialize SD Card

	if (SD.begin(s_nPinSdChipSelect))
	{
		// Determine log file index

		int iSuffix;
		for (iSuffix = 0; iSuffix < s_cLogFileMax; ++iSuffix)
		{
			char aChzPath[32];
			snprintf(aChzPath, DIM(aChzPath), "%s%d%s", s_pChzLogPrefix, iSuffix, s_pChzLogSuffix);
			if (!SD.exists(aChzPath))
			{
				s_nLogSuffix = iSuffix;
				Trace("Will use log file name '");
				Trace(aChzPath);
				Trace("'\n");
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
}

void loop()
{
	g_uhost.Task();

	if (g_userial != g_fUserialActive)
	{
		if (g_fUserialActive)
		{
			Serial.print("*** Device - disconnected ***\n");
			g_fUserialActive = false;

			if (millis() - s_msSplash > s_msSplashMax)
				DisplayStatus("Connect to Tactrix");
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

	if (g_tactrix.Tactrixs() == TACTRIXS_Disconnected && g_fUserialActive)
	{
		if (millis() - s_msSplash > s_msSplashMax)
			DisplayStatus("Connecting...");

		if (!g_tactrix.FTryConnect())
		{
			g_tactrix.Disconnect();
			delay(s_msTimeout);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Connected)
	{
		if (millis() - s_msSplash > s_msSplashMax)
			DisplayStatus("Polling ECU...");

		if (!g_tactrix.FTryStartPolling())
		{
			g_tactrix.Disconnect();
			delay(s_msTimeout);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Polling)
	{
		if (g_tactrix.FTryUpdatePolling())
		{
			g_oled.clearDisplay();

			// Top half always shows boost gauge

			static float s_gBoostMax = -1000.0f;
			float gBoost = g_tactrix.GParam(PARAM_BoostPsi);
			s_gBoostMax = max(gBoost, s_gBoostMax);

			static const float s_gBoostDisplayMin = -10.0f;
			static const float s_gBoostDisplayMax = 22.0f;
			float uBoost = (gBoost - s_gBoostDisplayMin) / (s_gBoostDisplayMax - s_gBoostDisplayMin);
			int dXBoost = int(s_dXScreen * min(max(uBoost, 0.0f), 1.0f) + 0.5f);
			static const int s_dYBoost = 13;
			g_oled.fillRect(0, 0, dXBoost, s_dYBoost, WHITE);

			float uBoostMax = (s_gBoostMax - s_gBoostDisplayMin) / (s_gBoostDisplayMax - s_gBoostDisplayMin);
			int xBoostMax = int(s_dXScreen * min(max(uBoostMax, 0.0f), 1.0f) + 0.5f);

			g_oled.drawFastVLine(xBoostMax, 0, s_dYBoost, WHITE);

			g_oled.fillRect(6, 2, 115, 9, BLACK);
			static const int s_dXMargin = 7;
			g_oled.setCursor(s_dXMargin, 3);
			g_oled.printf("Boost:% 5.1f [% 5.1f]", gBoost, s_gBoostMax);

			// Bottom half normally shows coolant & intake temp, unless warnings need to be displayed.

			static float s_gCoolantTempFMax = -100.0f;
			float gCoolantTempF = g_tactrix.GParam(PARAM_CoolantTempF);
			s_gCoolantTempFMax = max(gCoolantTempF, s_gCoolantTempFMax);

			static float s_gIatFMax = -100.0f;
			float gIatF = g_tactrix.GParam(PARAM_IatF);
			s_gIatFMax = max(gIatF, s_gIatFMax);

			u32 msCur = millis();

			static float s_degFbkcMin = 1000.0f;
			static u32 s_msFbkcEventLast = 0;
			float degFbkc = g_tactrix.GParam(PARAM_FbkcDeg);
			s_degFbkcMin = min(degFbkc, s_degFbkcMin);
			if (degFbkc < 0.0f)
				s_msFbkcEventLast = msCur;

			static float s_degFlkcMin = 1000.0f;
			static u32 s_msFlkcEventLast = 0;
			float degFlkc = g_tactrix.GParam(PARAM_FlkcDeg);
			s_degFlkcMin = min(degFlkc, s_degFlkcMin);
			if (degFlkc < 0.0f)
				s_msFlkcEventLast = msCur;

			static float s_uIamMin = 1.0f;
			float uIam = g_tactrix.GParam(PARAM_Iam);
			s_uIamMin = min(uIam, s_uIamMin);

			g_oled.setCursor(s_dXMargin, 15);
			bool fWarned = false;
			static bool s_fWarnedLastUpdate = false;

			if (s_uIamMin < 1.0f)
			{
				g_oled.printf(
						"  IAM:% 5.2f [% 5.2f]\n",
						uIam,
						s_uIamMin);

				g_oled.setCursor(s_dXMargin, g_oled.getCursorY() + 1);

				fWarned = true;
			}

			if (s_degFbkcMin < 0.0f && msCur - s_msFbkcEventLast < s_msDisplayWarning)
			{
				g_oled.printf(
						" FBKC:% 5.2f [% 5.2f]\n",
						degFbkc,
						s_degFbkcMin);

				g_oled.setCursor(s_dXMargin, g_oled.getCursorY() + 1);

				fWarned = true;
			}

			if (s_degFlkcMin < 0.0f && msCur - s_msFlkcEventLast < s_msDisplayWarning)
			{
				g_oled.printf(
						" FLKC:% 5.2f [% 5.2f]\n",
						degFlkc,
						s_degFlkcMin);

				g_oled.setCursor(s_dXMargin, g_oled.getCursorY() + 1);

				fWarned = true;
			}

			if (fWarned)
			{
				// Invert bottom half of screen

				g_oled.fillRect(0, 14, 128, 18, INVERSE);
			}
			else
			{
				g_oled.printf("  ECT:% 5.0f [% 5.0f]\n", gCoolantTempF, s_gCoolantTempFMax);

				g_oled.setCursor(s_dXMargin, g_oled.getCursorY() + 1);

				g_oled.printf("  IAT:% 5.0f [% 5.0f]\n", gIatF, s_gIatFMax);
			}

			g_oled.display();

			// Log if we displayed a warning

			if (fWarned)
			{
				if (!s_fileLog && s_nLogSuffix >= 0)
				{
					// Open the log file

					char aChzPath[32];
					snprintf(aChzPath, DIM(aChzPath), "%s%d%s", s_pChzLogPrefix, s_nLogSuffix, s_pChzLogSuffix);
					s_fileLog = SD.open(aChzPath, FILE_WRITE);
					if (s_fileLog)
					{
						Trace("Opened file '");
						Trace(aChzPath);
						Trace("' for writing.\n");

						// Write the header

						s_fileLog.print("time,");
						for (int iParamLog = 0;;)
						{
							s_fileLog.print(s_mpParamPChz[s_aParamLog[iParamLog]]);

							if (++iParamLog >= s_cParamLog)
								break;

							s_fileLog.print(",");
						}

						s_fileLog.println();
					}
					else
					{
						// Don't try to open again

						s_nLogSuffix = -1;

						Serial.print("Failed to open file '");
						Serial.print(aChzPath);
						Serial.print("' for writing. Is SD card full?\n");
					}
				}

				if (s_fileLog)
				{
					// Write the log history

					while (!g_loghist.FIsEmpty())
					{
						const SLogEntry & logent = g_loghist.LogentRead();

						s_fileLog.print(logent.m_msTimestamp / 1000.0f, 3);

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
			}

			if (!fWarned && s_fWarnedLastUpdate && s_fileLog)
			{
				// Flush to SD card

				s_fileLog.flush();

				Trace("Flushed file.\n");
			}

			s_fWarnedLastUpdate = fWarned;
		}
		else
		{
			if (millis() - s_msSplash > s_msSplashMax)
				DisplayStatus("Polling Error");

			g_tactrix.Disconnect();
			delay(s_msTimeout);
		}
	}
	else
	{
		if (Serial.available())
		{
			while (Serial.available())
			{
				int ch = Serial.read();
				Serial.write(ch);
				g_userial.write(ch);
			}
		}

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
}
