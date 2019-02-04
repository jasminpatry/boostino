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

	bool			FTryInit();
	void			Update();
	TACTRIXS		Tactrixs() const
						{ return m_tactrixs; }

protected:
	void			Trace(const char * pChz);
	void			Trace(int n, int mode = DEC);
	void			TraceByte(u8 b);

	void			SendCommand(const u8 * aB, u16 cB);
	void			SendCommand(const char * pChz);
	int				CBReceive(u32 msTimeout);
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

void CTactrix::TraceByte(u8 b)
{
#if DEBUG
	if (isprint(b))
	{
		Serial.print((char)b);
	}
	else
	{
		switch (b)
		{
		case '\r':
			Serial.print("\\r");
			break;

		case '\n':
			Serial.print("\\n");
			break;

		default:
			Serial.print("\\x");
			Serial.print(b, HEX);
			break;
		}
	}
#endif // DEBUG
}

void CTactrix::SendCommand(const u8 * aB, u16 cB)
{
	if (!g_userial)
	{
		Trace("SendCommand: Can't send, not connected");
		m_tactrixs = TACTRIXS_Disconnected;
		return;
	}

	Trace("Sending ");
	Trace(cB);
	Trace(" bytes:\n>>");

	for (int iB = 0; iB < cB; ++iB)
	{
		u8 b = aB[iB];

		TraceByte(b);

		g_userial.write(b);
	}

	Trace("\n");
}

void CTactrix::SendCommand(const char * pChz)
{
	SendCommand((const u8 *)pChz, strlen(pChz));
}

int CTactrix::CBReceive(u32 msTimeout)
{
	u32 msStart = millis();
	for (;;)
	{
		m_cBRecv = g_userial.readBytes(m_aBRecv, sizeof(m_aBRecv));
		if (m_cBRecv > 0)
		{
#if DEBUG
			Trace("Received ");
			Trace(m_cBRecv);
			Trace(" bytes:\n<<");
			for (int iB = 0; iB < m_cBRecv; ++iB)
			{
				TraceByte(m_aBRecv[iB]);
			}
			Trace("\n");
#endif // DEBUG

			return m_cBRecv;
		}

		if (msTimeout > 0 && millis() - msStart > msTimeout)
			return 0;

		delayMicroseconds(10);
	}
}

void CTactrix::FlushIncoming()
{
	Trace("Flushing incoming data...\n");

	while (g_userial.available())
	{
		u8 b = g_userial.read();

		TraceByte(b);
	}

	Trace("FlushIncoming done.\n");
}

bool CTactrix::FTryInit()
{
	if (m_tactrixs != TACTRIXS_Disconnected)
	{
		SendCommand("atz\r\n");
		FlushIncoming();
		m_tactrixs = TACTRIXS_Disconnected;
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

	FlushIncoming();

	SendCommand("\r\n\r\nati\r\n");

	CBReceive(1000);

	m_tactrixs = TACTRIXS_Connected;

	return true;
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
			g_userial.setTimeout(0);
		}
	}

	if (g_tactrix.Tactrixs() == TACTRIXS_Disconnected)
		g_tactrix.FTryInit();

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
}
