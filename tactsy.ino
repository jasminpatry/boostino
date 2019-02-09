// Simple test of USB Host Mouse/Keyboard
//
// This example is in the public domain

#include "USBHost_t36.h"

#define DEBUG 1
#define USBBAUD 115200

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

CASSERT(sizeof(int) == sizeof(s32));



// Available ECU parameters

enum PARAM
{
	PARAM_Fbkc,
	PARAM_Flkc,
	PARAM_Boost,
	PARAM_Iam,

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
	"FBKC",						// PARAM_Fbkc
	"FLKC",						// PARAM_Flkc
	"Boost",					// PARAM_Boost
	"IAM",						// PARAM_Iam
};
CASSERT(DIM(s_mpParamPChz) == PARAM_Max);

static const u8 s_cBSsmAddr = 3;
static const u8 s_mpParamABAddr[][s_cBSsmAddr] =
{
	{ 0xff, 0x81, 0xfc },		// PARAM_Fbkc
	{ 0xff, 0x82, 0x98 },		// PARAM_Flkc
	{ 0xff, 0x62, 0x28 },		// PARAM_Boost
	{ 0xff, 0x32, 0x9C },		// PARAM_Iam
};
CASSERT(DIM(s_mpParamABAddr) == PARAM_Max);

static const u8 s_mpParamCB[] =
{
	 4,							// PARAM_Fbkc
	 4,							// PARAM_Flkc
	 4,							// PARAM_Boost
	 4,							// PARAM_Iam
};
CASSERT(DIM(s_mpParamCB) == PARAM_Max);

static const bool s_mpParamFIsFloat[] =
{
	true,						// PARAM_Fbkc
	true,						// PARAM_Flkc
	true,						// PARAM_Boost
	true,						// PARAM_Iam
};
CASSERT(DIM(s_mpParamFIsFloat) == PARAM_Max);



// ECU Parameters to Poll

static const PARAM s_aParamPoll[] =
{
	PARAM_Boost,
	PARAM_Iam,
	PARAM_Fbkc,
	PARAM_Flkc,
};
static const u8 s_cParamPoll = DIM(s_aParamPoll);



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



USBHost g_uhost;
USBSerial g_userial(g_uhost);
bool g_fUserialActive = false;

static const u16 s_nIdTactrix = 0x0403;
static const u16 s_nIdOpenPort20 = 0xcc4c;
static const u32 s_msTimeout = 2000;



// J2534 Message kinds

enum MSGK
{
	MSGK_InfoReply,
	MSGK_LoopbackStart,
	MSGK_Loopback,
	MSGK_LoopbackEnd,
	MSGK_ReplyStart,
	MSGK_Reply,
	MSGK_ReplyEnd,

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
	int cB = m_cB + 3;
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



// Bitcasting helper struct

struct SIntFloat	// tag = ng
{
	union
	{
		u32 m_n;
		float m_g;
	};
};



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
					  m_mpParamNValue()
						{ ; }

					~CTactrix()
						{ ; }

	bool			FTryConnect();
	void			Disconnect();
	bool			FTryStartPolling();
	bool			FTryUpdatePolling();
	TACTRIXS		Tactrixs() const
						{ return m_tactrixs; }

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

	TACTRIXS		m_tactrixs;
	u8				m_aBRecv[4096];
	int				m_cBRecv;
	int				m_iBRecv;
	int				m_iBRecvPrev;
	int				m_cBParamPoll;		// Total number of bytes (addresses) being polled
	u32				m_mpParamNValue[s_cParamPoll];
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
		Trace("FMustReceiveMessage got unexpected reply (expected \"");
		int cCh = strlen(pChz);
		for (int iCh = 0; iCh < cCh; ++iCh)
			Trace(pChz[iCh]);
		Trace("\")\n");
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

#define TEST_OFFLINE 0
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

		// &&& validate SSM reply?

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

		// J2534 header is at most 23 bytes

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

	return true;
}

void CTactrix::ProcessParamValue(PARAM param, u32 nValue)
{
	SIntFloat ng;
	ng.m_n = nValue;

	switch (param)
	{
	case PARAM_Fbkc:
		Trace("FBKC: ");
		Trace(ng.m_g);
		Trace("\n");
		break;

	case PARAM_Flkc:
		Trace("FLKC: ");
		Trace(ng.m_g);
		Trace("\n");
		break;

	case PARAM_Boost:
		// Convert to PSI

		ng.m_g *= 0.01933677f;

		Trace("Boost: ");
		Trace(ng.m_g);
		Trace("\n");
		break;

	case PARAM_Iam:
		Trace("IAM: ");
		Trace(ng.m_g);
		Trace("\n");
		break;

	default:
		ASSERT(false);
	}

	m_mpParamNValue[param] = ng.m_n;
}

void CTactrix::Disconnect()
{
	ResetReceive();

	if (m_tactrixs == TACTRIXS_Disconnected)
		return;

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



void setup()
{
	while (!Serial && (millis() < 5000))
		(void) 0; // wait for Arduino Serial Monitor

	Serial.println("\n\nUSB Host Testing - Serial");
	g_uhost.begin();
}

void loop()
{
	digitalWrite(13, !digitalRead(13));
	g_uhost.Task();

	// Print out information about different devices.

	if (g_userial != g_fUserialActive)
	{
		if (g_fUserialActive)
		{
			Serial.printf("*** Device - disconnected ***\n");
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

			g_userial.begin(USBBAUD);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Disconnected)
	{
		if (!g_tactrix.FTryConnect())
			delay(s_msTimeout);
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Connected)
	{
		if (!g_tactrix.FTryStartPolling())
		{
			g_tactrix.Disconnect();
			delay(s_msTimeout);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Polling)
	{
		if (!g_tactrix.FTryUpdatePolling())
		{
			g_tactrix.Disconnect();
			delay(s_msTimeout);
		}
	}

	if (Serial.available())
	{
		while (Serial.available())
		{
			int ch = Serial.read();
			Serial.write(ch);
			g_userial.write(ch);
		}
	}

	while (g_userial.available())
	{
		Serial.write(g_userial.read());
	}

	Serial.flush();
}
