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
					  m_cBRecv(0)
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
	bool			FReceive(const char * pChz);
	bool			FReceivePrefix(const char * pChzPrefix);

	void			FlushIncoming();

	TACTRIXS		m_tactrixs;
	u8				m_aBRecv[4096];
	int				m_cBRecv;
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

bool CTactrix::FReceivePrefix(const char * pChzPrefix)
{
	int cChPrefix = strlen(pChzPrefix);

	if (CBReceive(g_msTimeout) == 0)
	{
		Trace("Timed out waiting for prefix \"");
		for (int iCh = 0; iCh < cChPrefix; ++iCh)
			Trace(pChzPrefix[iCh]);
		Trace("\"\n");
		return false;
	}

	if (m_cBRecv < cChPrefix || strncasecmp((const char *)m_aBRecv, pChzPrefix, cChPrefix) != 0)
	{
		Trace("Unexpected prefix (expected \"");
		for (int iCh = 0; iCh < cChPrefix; ++iCh)
			Trace(pChzPrefix[iCh]);
		Trace("\")\n");
		return false;
	}

	return true;
}

bool CTactrix::FReceive(const char * pChz)
{
	int cCh = strlen(pChz);

	if (CBReceive(g_msTimeout) == 0)
	{
		Trace("Timed out waiting for reply \"");
		for (int iCh = 0; iCh < cCh; ++iCh)
			Trace(pChz[iCh]);
		Trace("\"\n");
		return false;
	}

	if (m_cBRecv != cCh || strncasecmp((const char *)m_aBRecv, pChz, cCh) != 0)
	{
		Trace("Unexpected reply (expected \"");
		for (int iCh = 0; iCh < cCh; ++iCh)
			Trace(pChz[iCh]);
		Trace("\")\n");
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

	if (g_userial.idVendor() != g_nIdTactrix ||
		g_userial.idProduct() != g_nIdOpenPort20)
	{
		return false;
	}

	// Only receive one bulk transfer at a time

	g_userial.setTimeout(0);

	FlushIncoming();

	SendCommand("\r\n\r\nati\r\n");
	if (!FReceivePrefix("ari"))
		return false;

	SendCommand("ata 2\r\n");
	if (!FReceive("aro 2\r\n"))
		return false;

	// Equivalent to J2534 PassThruConnect:
	//	protocol = ISO9141 (3),
	//	flags = ISO9141_NO_CHECKSUM (0x200 or 512)
	//	baudrate = 4800

	SendCommand("ato3 512 4800 0 3\r\n");
	if (!FReceive("aro 3\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P1_MAX (7) -- maximum inter-byte time for ECU responses in ms (default is 20)
	//	Value = 2

	SendCommand("ats3 7 2 4\r\n");
	if (!FReceive("aro 4\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P3_MIN (0xa or 10) -- minimum time between ECU response and start of new tester request in ms (default is 55)
	//	Value = 0

	SendCommand("ats3 10 0 5\r\n");
	if (!FReceive("aro 5\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = P4_MIN (0xc or 12) -- minimum inter-byte time for a tester request in ms (default is 5)
	//	Value = 0

	SendCommand("ats3 12 0 6\r\n");
	if (!FReceive("aro 6\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = LOOPBACK (3) -- echo transmitted messages in the receive queue (default is OFF (0))
	//	Value = ON (1)

	SendCommand("ats3 3 1 7\r\n");
	if (!FReceive("aro 7\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = DATA_BITS (0x20 or 32)
	//	Value = 8

	SendCommand("ats3 32 8 8\r\n");
	if (!FReceive("aro 8\r\n"))
		return false;

	// Equivalent to J2534 PassThruIoctl with ioctlID = SET_CONFIG:
	//	Parameter ID = PARITY (0x16 or 22) -- default is NO_PARITY (0)
	//	Value = NO_PARITY (0)
	// BB (jasminp) Redundant?

	SendCommand("ats3 22 0 9\r\n");
	if (!FReceive("aro 9\r\n"))
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
	if (!FReceive("arf3 0 10\r\n"))
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

	if (!FReceivePrefix("ar3\x05\xa0"))
		return false;

	// Loopback message

	if (!FReceive("ar3\x07 \x80\x10\xf0\x01\xbf\x40"))
		return false;

	// End of loopback message

	if (!FReceivePrefix("ar3\x05\x60"))
		return false;

	// Acknowledgement

	if (!FReceive("aro 11\r\n"))
		return false;

	// Start of normal message

	if (!FReceivePrefix("ar3\x05\x80"))
		return false;

	u8 aBSsmInitReply[255];
	u8 cBSsmInitReply = 0;

	while (!FReceivePrefix("ar3\x05\x40"))
	{
		if (m_cBRecv < 6 ||
			strncasecmp((const char *)m_aBRecv, "ar3", 3) != 0 ||
			m_aBRecv[4] != 0)
		{
			Trace("Malformed SSM Init reply\n");
			return false;
		}

		u8 cBMsg = m_aBRecv[3] - 1;
		if (cBSsmInitReply + cBMsg > sizeof(aBSsmInitReply))
		{
			Trace("Overflow of SSM init reply buffer");
			return false;
		}
		else if (cBMsg + 5 > m_cBRecv)
		{
			Trace("Incomplete SSM init message");
			return false;
		}
		else
		{
			memcpy(&aBSsmInitReply[cBSsmInitReply], &m_aBRecv[5], cBMsg);
			cBSsmInitReply += cBMsg;
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
		(void) FReceive("aro 12\r\n");

		// Equivalent to J2534 PassThruDisconnect

		SendCommand("atc3 13\r\n");
		(void) FReceive("aro 13\r\n");
	}

	SendCommand("atz 14\r\n");
	(void) FReceive("aro 14\r\n");

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
