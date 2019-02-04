// Simple test of USB Host Mouse/Keyboard
//
// This example is in the public domain

#include "USBHost_t36.h"

#define DEBUG 1
#define USBBAUD 115200

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

USBHost g_uhost;
USBSerial g_userial(g_uhost);
bool g_fUserialActive = false;

static const u16 g_nIdTactrix = 0x0403;
static const u16 g_nIdOpenPort20 = 0xcc4c;
static const u32 g_msTimeout = 2000;

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
					  m_iBRecvPrev(0)
						{ ; }

					~CTactrix()
						{ ; }

	bool			FTryConnect();
	void			Disconnect();
	bool			FTryStartPolling();
	TACTRIXS		Tactrixs() const
						{ return m_tactrixs; }

protected:
	void			Trace(const char * pChz);
	void			Trace(int n, int mode = DEC);
	void			Trace(u8 b, int mode = DEC);
	void			Trace(char c);

	void			SendCommand(const u8 * aB, u16 cB);
	void			SendCommand(const char * pChz);

	int				CBReceive(u32 msTimeout);
	void			ResetReceive();
	bool			FReceiveMessage(const char * pChz);
	int				CBMessage()
						{ return m_iBRecv - m_iBRecvPrev; }
	u8 *			PBMessage()
						{ return &m_aBRecv[m_iBRecvPrev]; }
	int				CBRemaining()
						{ return m_cBRecv - m_iBRecv; }
	u8 *			PBRemaining()
						{ return &m_aBRecv[m_iBRecv]; }

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

	bool			FReceiveMessage(MSGK msgk);

	void			FlushIncoming();

	TACTRIXS		m_tactrixs;
	u8				m_aBRecv[4096];
	int				m_cBRecv;
	int				m_iBRecv;
	int				m_iBRecvPrev;
};

void CTactrix::Trace(const char * pChz)
{
#if DEBUG
	Serial.print(pChz);
#endif // DEBUG
}

void CTactrix::Trace(int n, int mode)
{
#if DEBUG
	Serial.print(n, mode);
#endif // DEBUG
}

void CTactrix::Trace(u8 b, int mode)
{
#if DEBUG
	Serial.print(b, mode);
#endif // DEBUG
}

void CTactrix::Trace(char c)
{
#if DEBUG
	if (isprint(c))
	{
		Serial.print(c);
	}
	else
	{
		switch (c)
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
			Serial.print(u8(c), HEX);
			break;
		}
	}
#endif // DEBUG
}

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
	Trace(" bytes:\no \"");

	for (int iB = 0; iB < cB; ++iB)
	{
		u8 b = aB[iB];

		Trace(char(b));

		g_userial.write(b);
	}

	Trace("\"\n");
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
		Trace("CBReceive: Can't receive, not connected\n");
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
			Trace(" bytes:\ni \"");
			for (int iB = 0; iB < m_cBRecv; ++iB)
				Trace(char(m_aBRecv[iB]));
			Trace("\"\n");
#endif // DEBUG

			return m_cBRecv;
		}

		if (msTimeout > 0 && millis() - msStart > msTimeout)
			return 0;

		delayMicroseconds(10);
	}
}

void CTactrix::ResetReceive()
{
	m_cBRecv = 0;
	m_iBRecv = 0;
	m_iBRecvPrev = 0;
}

bool CTactrix::FReceiveMessage(const char * pChz)
{
	int cCh = strlen(pChz);

	if (CBRemaining() == 0)
	{
		if (CBReceive(g_msTimeout) == 0)
		{
			Trace("Timed out waiting for reply \"");
			for (int iCh = 0; iCh < cCh; ++iCh)
				Trace(pChz[iCh]);
			Trace("\"\n");
			return false;
		}
	}

	if (CBRemaining() < cCh ||
		strncasecmp((const char *)PBRemaining(), pChz, cCh) != 0)
	{
		Trace("Unexpected reply (expected \"");
		for (int iCh = 0; iCh < cCh; ++iCh)
			Trace(pChz[iCh]);
		Trace("\")\n");

		return false;
	}

	m_iBRecvPrev = m_iBRecv;
	m_iBRecv += cCh;

	return true;
}

bool CTactrix::FReceiveMessage(MSGK msgk)
{
	// MSGK_LoopbackStart: "ar3\x05\xa0"
	// MSGK_Loopback: "ar3\x07\x20"
	// MSGK_LoopbackEnd: "ar3\x05\x60"
	// MSGK_ReplyStart: "ar3\x05\x80"
	// MSGK_ReplyEnd: "ar3\x05\x40"

	// MSGK_Reply checks:
	// if (cBMsg < 6 ||
	// 	strncasecmp((const char *)PBMessage(), "ar3", 3) != 0 ||
	// 	pBMsg[4] != 0)
	// {
	// 	Trace("Malformed SSM Init reply\n");
	// 	return false;
	// }
	// else if (cBFragment + 5 > cBMsg)
	// {
	// 	Trace("Incomplete SSM init message");
	// 	return false;
	// }

	return false; // &&&
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

	if (g_userial.idVendor() != g_nIdTactrix ||
		g_userial.idProduct() != g_nIdOpenPort20)
	{
		return false;
	}

	// Only receive one bulk transfer at a time

	g_userial.setTimeout(0);

	FlushIncoming();

	SendCommand("\r\n\r\nati\r\n");
	if (!FReceiveMessage(MSGK_InfoReply))
		return false;

	SendCommand("ata 2\r\n");
	if (!FReceiveMessage("aro 2\r\n"))
		return false;

	// Equivalent to J2534 PassThruConnect:
	//	protocol = ISO9141 (3),
	//	flags = ISO9141_NO_CHECKSUM (0x200 or 512)
	//	baudrate = 4800

	SendCommand("ato3 512 4800 0 3\r\n");
	if (!FReceiveMessage("aro 3\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P1_MAX (7) -- maximum inter-byte time for ECU responses in ms (default is 20)
	//	Value = 2

	SendCommand("ats3 7 2 4\r\n");
	if (!FReceiveMessage("aro 4\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P3_MIN (0xa or 10) -- minimum time between ECU response and start of new tester request in ms (default is 55)
	//	Value = 0

	SendCommand("ats3 10 0 5\r\n");
	if (!FReceiveMessage("aro 5\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P4_MIN (0xc or 12) -- minimum inter-byte time for a tester request in ms (default is 5)
	//	Value = 0

	SendCommand("ats3 12 0 6\r\n");
	if (!FReceiveMessage("aro 6\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = LOOPBACK (3) -- echo transmitted messages in the receive queue (default is OFF (0))
	//	Value = ON (1)

	SendCommand("ats3 3 1 7\r\n");
	if (!FReceiveMessage("aro 7\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = DATA_BITS (0x20 or 32)
	//	Value = 8

	SendCommand("ats3 32 8 8\r\n");
	if (!FReceiveMessage("aro 8\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = PARITY (0x16 or 22) -- default is NO_PARITY (0)
	//	Value = NO_PARITY (0)
	// BB (jasminp) Redundant?

	SendCommand("ats3 22 0 9\r\n");
	if (!FReceiveMessage("aro 9\r\n"))
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
	if (!FReceiveMessage("arf3 0 10\r\n"))
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

	// Start of loopback message

	if (!FReceiveMessage(MSGK_LoopbackStart))
		return false;

	// Loopback message

	if (!FReceiveMessage("ar3\x07 \x80\x10\xf0\x01\xbf\x40"))
		return false;

	// End of loopback message

	if (!FReceiveMessage(MSGK_LoopbackEnd))
		return false;

	// Acknowledgement

	if (!FReceiveMessage("aro 11\r\n"))
		return false;

	// Start of normal message

	if (!FReceiveMessage(MSGK_ReplyStart))
		return false;

	u8 aBSsmInitReply[255];
	u8 cBSsmInitReply = 0;

	while (!FReceiveMessage(MSGK_ReplyEnd))
	{
		if (CBRemaining() == 0 || !FReceiveMessage(MSGK_Reply))
			return false;

		u8 * pBMsg = PBMessage();
		int cBMsg = CBMessage();

		u8 cBFragment = pBMsg[3] - 1;
		if (cBSsmInitReply + cBFragment > sizeof(aBSsmInitReply))
		{
			Trace("Overflow of SSM init reply buffer");
			return false;
		}
		else
		{
			memcpy(&aBSsmInitReply[cBSsmInitReply], &pBMsg[5], cBFragment);
			cBSsmInitReply += cBFragment;
		}
	}

	Trace("SSM Init Reply: ");
	for (int iB = 0; iB < cBSsmInitReply; ++iB)
		Trace(aBSsmInitReply[iB]);
	Trace("\n");

	return true;
}

void CTactrix::Disconnect()
{
	if (m_tactrixs == TACTRIXS_Disconnected)
		return;

	if (m_tactrixs == TACTRIXS_Connected)
	{
		// Equivalent to J2534 PassThruStopMsgFilter

		SendCommand("atk3 0 12\r\n");
		(void) FReceiveMessage("aro 12\r\n");

		// Equivalent to J2534 PassThruDisconnect

		SendCommand("atc3 13\r\n");
		(void) FReceiveMessage("aro 13\r\n");
	}

	SendCommand("atz 14\r\n");
	(void) FReceiveMessage("aro 14\r\n");

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
			delay(g_msTimeout);
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Connected)
	{
		if (!g_tactrix.FTryStartPolling())
		{
			g_tactrix.Disconnect();
			delay(g_msTimeout);
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
