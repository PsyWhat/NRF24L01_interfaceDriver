#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <string.h>

class NRF24L01
{
public:
	// Addr: 0x00
	struct S_CONFIG
	{
		static const uint8_t addr = 0x00;
		// RX/TX control
		uint8_t PRIM_RX : 1;
		// Power up control
		uint8_t PWR_UP : 1;
		// CRC encoding scheme, 0 - 1 byte, 1 - 2 bytes
		uint8_t CRCO : 1;
		// Enable CRC. Forced 1, if one of the bits in the EN_AA is high
		uint8_t EN_CRC : 1;
		// Mask interrupt caused by MAX_RT
		// 1: Interrupt not reflected on the IRQ pin
		// 0: Reflect MAX_RT as active low interrupt on the IRQ pin
		uint8_t MASK_MAX_RT : 1;
		// Mask interrupt caused by TX_DS
		// 1: Interrupt not reflected on the IRQ pin
		// 0: Reflect TX_DS as active low interrupt on the IRQ pin
		uint8_t MASK_TX_DS : 1;
		// Mask interrupt caused by RX_DR
		// 1: Interrupt not reflected on the IRQ pin
		// 0: Reflect RX_DR as active low interrupt on the IRQ pin
		uint8_t MASK_RX_DR : 1;
	};

	// Addr: 0x01
	struct S_EN_AA
	{
		static const uint8_t addr = 0x01;
		// Enable auto acknowledgment on data pipe 0
		uint8_t ENAA_P0 : 1;
		// Enable auto acknowledgment on data pipe 1
		uint8_t ENAA_P1 : 1;
		// Enable auto acknowledgment on data pipe 2
		uint8_t ENAA_P2 : 1;
		// Enable auto acknowledgment on data pipe 3
		uint8_t ENAA_P3 : 1;
		// Enable auto acknowledgment on data pipe 4
		uint8_t ENAA_P4 : 1;
		// Enable auto acknowledgment on data pipe 5
		uint8_t ENAA_P5 : 1;
	};

	// Addr: 0x02
	struct S_EN_RXADDR
	{
		static const uint8_t addr = 0x02;
		// Enable data pipe 0
		uint8_t ERX_P0 : 1;
		// Enable data pipe 1
		uint8_t ERX_P1 : 1;
		// Enable data pipe 2
		uint8_t ERX_P2 : 1;
		// Enable data pipe 3
		uint8_t ERX_P3 : 1;
		// Enable data pipe 4
		uint8_t ERX_P4 : 1;
		// Enable data pipe 5
		uint8_t ERX_P5 : 1;
	};

	// Addr: 0x03
	struct S_SETUP_AW
	{
		static const uint8_t addr = 0x03;
		// RX_TX Address field width
		// 00 - illegal
		// 01 - 3 bytes
		// 10 - 4 bytes
		// 11 - 5 bytes
		// LSByte is used if address width is below 5 bytes
		uint8_t AW : 2;
	};

	// Addr: 0x04
	struct S_SETUP_RETR
	{
		static const uint8_t addr = 0x04;
		// Auto retransmit count
		uint8_t ARC : 4;
		// Auto retransmit delay
		// (250 + ARD * 250) uS
		uint8_t ARD : 4;
	};

	// Addr: 0x05
	struct S_RF_CH
	{
		static const uint8_t addr = 0x05;
		// Sets the frequency channel nRF24L01 operates on
		uint8_t RF_CH : 7;
	};

	// Addr: 0x06
	struct S_RF_SETUP
	{
		static const uint8_t addr = 0x06;
		uint8_t : 1;
		// Set RF output power in TX mode
		// 00 - -18 dBm
		// 01 - -12 dBm
		// 10 - -6 dBm
		// 11 - 0 dBm
		uint8_t RF_PWR : 2;
		// Select between the high speed data rates, this bit is don't care if RF_DR_LOW is set
		// 0 - 1Mbps
		// 1 - 2Mbps
		uint8_t RF_DR_HIGH : 1;
		// Force PLL lock signal. Only used in test
		uint8_t PLL_LOCK : 1;
		// Set RF Data rate to 250kbps
		uint8_t RF_DR_LOW : 1, :1;
		// Enables continuous carrier transmit when high
		uint8_t CONT_WAVE : 1;
	};

	// Addr: 0x07
	// Status register (In parallel to the SPI command word applied on the MOSI pin
	// the STATUS register is shifted serially out on the MISO pin
	struct S_STATUS
	{
		static const uint8_t addr = 0x07;
		// TX FIFO full flag, 1 - full
		uint8_t TX_FULL : 1;
		// Data pipe number for the payload available for reading from RX_FIFO
		// 000-101: Data pipe number
		// 110: not used
		// 111: RX FIFO Empty
		uint8_t RX_P_NO : 3;
		// Maximum number of TX retransmits interrupt
		// Write 1 to clear bit. If MAX_RT is asserted, it must be cleared
		// to enable further communication
		uint8_t MAX_RT : 1;
		// Data sent TX FIFO interrupt. Asserted when packet transmitted on TX.
		// If AUTO_ACK is activated, this bit is set high only when ACK is received
		// Write 1 to clear bit
		uint8_t TX_DS : 1;
		// Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFO
		// Write 1 to clear bit
		uint8_t RX_DR : 1;
	};

	// Addr: 0x08
	// Transmit observe register
	struct S_OBSERVE_TX
	{
		static const uint8_t addr = 0x08;
		// Count retransmitted packets.
		//The counter is reset, when transmission of a new packet starts.
		uint8_t ARC_CNT : 4;
		// Count lost packets. The counter if overflow protected to 15,
		// and discontinues at max until reset. The counter is reset by writing RF_CH.
		uint8_t PLOS_CNT : 4;
	};

	// Addr: 0x09
	struct S_RDP
	{
		static const uint8_t addr = 0x09;
		// Received Power Detector.
		// Carrier Detect
		uint8_t RPD : 1, :7;
	};

	// 5 byte address struct
	struct S_ADDR
	{
		S_ADDR() {};

		S_ADDR(const uint64_t& addr)
		{
			
			*reinterpret_cast<uint32_t*>(bytes) = addr;
			bytes[4] = *(reinterpret_cast<uint8_t*>(const_cast<uint64_t*>(&addr)) + 4);
		}


		operator uint64_t() const
		{
			return (uint64_t)(*(uint32_t*)(bytes)) | (((uint64_t)(bytes[4]))<<(4*8));
		}

		operator uint32_t() const
		{
			return (*(uint32_t*)(bytes));
		}

		operator uint8_t() const
		{
			return (*(uint8_t*)(bytes));
		}

		uint8_t operator[](uint8_t i)
		{
			return bytes[i];
		}

		uint8_t bytes[5];
	};

	// Addr: 0x17
	// FIFO Status Register
	struct S_FIFO_STATUS
	{
		static const uint8_t addr = 0x17;
		// RX FIFO empty flag
		uint8_t RX_EMPTY : 1;
		// RX FIFO full flag
		uint8_t RX_FULL : 1, : 2;
		// TX FIFO empty flag
		uint8_t TX_EMPTY : 1;
		// TX FIFO full flag
		uint8_t TX_FULL : 1;
		// Used for a PTX device
		// Pulse the rfce high for at least 10us to Reuse last transmitted payload.
		// TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
		// TX_REUSE is set by the SPI command REUSE_TX_PL, and is reset by the SPI commands:
		// W_TX_PAYLOAD or FLUSH_TX
		uint8_t TX_REUSE : 1;
	};


	// Addr: 0x1C
	// Enable dynamic payload length, requires EN_DPL and ENAA_P?
	struct S_DYNPD
	{
		static const uint8_t addr = 0x1C;
		// Enable dynpd data pipe 0
		uint8_t DPL_P0 : 1;
		// Enable dynpd data pipe 1
		uint8_t DPL_P1 : 1;
		// Enable dynpd data pipe 2
		uint8_t DPL_P2 : 1;
		// Enable dynpd data pipe 3
		uint8_t DPL_P3 : 1;
		// Enable dynpd data pipe 4
		uint8_t DPL_P4 : 1;
		// Enable dynpd data pipe 5
		uint8_t DPL_P5 : 1;
	};

	// Addr: 0x1D
	// Feature register
	struct S_FEATURE
	{
		static const uint8_t addr = 0x1D;
		// Enables the W_TX_PAYLOAD_NOACK command
		uint8_t EN_DYN_ACK : 1;
		// Enables payload with ACK
		uint8_t EN_ACK_PAY : 1;
		// Enables dynamic payload length
		uint8_t EN_DPL : 1;
		uint8_t : 5;
	};


	struct R_RX_PL_RESP 
	{
		S_STATUS status;
		uint8_t len;
	};

	struct Packet
	{
		uint8_t *buffer;
		uint8_t len;
		uint8_t pipe;

		Packet(): len(0), buffer(NULL), pipe(0) {}

		inline bool IsTimeout()
		{
			return pipe == 0xff;
		}

		static Packet Timeout();

		Packet(const uint8_t &len, const uint8_t &pipe);

		Packet(const uint8_t *buff, const uint8_t &len, const uint8_t& pipe);

		Packet(const Packet &&copy);

		~Packet();

	};

	enum RFSpeeds : uint8_t
	{
		SpeedLow = 0b10,
		SpeedMed = 0b00,
		SpeedHigh = 0b01,
	};

	enum CommandTypes : uint8_t
	{
		CT_R_REGISTER = 0,
		CT_W_REGISTER = 0x20,
		CT_R_RX_PAYLOAD = 0x61,
		CT_W_TX_PAYLOAD = 0xA0,
		CT_FLUSH_TX = 0xE1,
		CT_FLUSH_RX = 0xE2,
		CT_REUSE_TX_PL = 0xE3,
		CT_R_RX_PL_WID = 0x60,
		CT_W_ACK_PAYLOAD = 0xA8,
		CT_W_TX_PAYLOAD_NO_ACK = 0xB0,
		CT_NOP = 0xFF
	};

	enum ChipStates : uint8_t
	{
		CS_Undefined,
		CS_PowerOnReset,
		CS_PowerDown,
		CS_StartUp,
		CS_Standby1,
		CS_RXSetting,
		CS_RXMode,
		CS_TXSetting,
		CS_TXMode,
		CS_Standby2,
	};

	enum RegAddrs : uint8_t
	{
		RA_CONFIG = 0x00,

		RA_EN_AA = 0x01,

		RA_EN_RXADDR = 0x02,

		RA_SETUP_AW = 0x03,

		RA_SETUP_RETR = 0x04,

		RA_RF_CH = 0x05,

		RA_RF_SETUP = 0x06,

		RA_STATUS = 0x07,

		RA_OBSERVE_TX = 0x08,

		RA_RPD = 0x09,

		// Receive address data pipe 0, 5 bytes maximum length.
		// LSByte is written first, length is defined by SETUP_AW
		RA_RX_ADDR_P0 = 0x0A,

		// Receive address data pipe 1, 5 bytes maximum length.
		// LSByte is written first, length is defined by SETUP_AW
		RA_RX_ADDR_P1 = 0x0B,

		// Receive address data pipe 2, 1 byte len, using RX_ADDR_P2 MSBytes
		RA_RX_ADDR_P2 = 0x0C,

		// Receive address data pipe 3, 1 byte len, using RX_ADDR_P2 MSBytes
		RA_RX_ADDR_P3 = 0x0D,

		// Receive address data pipe 4, 1 byte len, using RX_ADDR_P2 MSBytes
		RA_RX_ADDR_P4 = 0x0E,

		// Receive address data pipe 5, 1 byte len, using RX_ADDR_P2 MSBytes
		RA_RX_ADDR_P5 = 0x0F,

		// Transmit address. Used for a PTX device only.
		// LSByte is written first
		// Set RX_ADDR_P0 equal to this address to handle automatic acknowledge
		// if this is a PTX device with Enhanced ShockBurst enabled.
		RA_TX_ADDR = 0x10,

		RA_RX_PW_P0 = 0x11,

		RA_RX_PW_P1 = 0x12,

		RA_RX_PW_P2 = 0x13,

		RA_RX_PW_P3 = 0x14,

		RA_RX_PW_P4 = 0x15,

		RA_RX_PW_P5 = 0x16,

		RA_FIFO_STATUS = 0x17,

		RA_DYNPD = 0x1C,

		RA_FEATURE = 0x1D,

	};

	enum RFOutPowers : uint8_t
	{
		// -18dBm
		Lower = 0x0,
		// -12dBm
		Low = 0x1,
		// -6dBm
		Medium = 0x2,
		// 0dBm
		High = 0x3
	};

	enum TransmitStatus : uint8_t
	{
		// Successfully transmitted 
		TS_Success = 0,

		// Maximum retransmits reached while sending the packet
		TS_MaxRetransmit,

		// Can not send, the FIFO is not empty, and there is maximum retransmits reached
		TS_FIFO_Buisy_MaxRetransmit,
	};

	/* NRF24L01:: Recieve(int timeout = 0x400)
	 * Receives packet from radio channel
	 * Timeout is the number of tries, negative value for infinity block
	 * Packet::IsTimeout() for timeout check.
	 **/
	Packet Recieve(int timeout = 0x400);

	TransmitStatus Transmit(const uint8_t *buff, uint8_t len);
	
	// Flushes TX_FIFO and clears MAX_RT flag
	inline void FlushTXFIFO()
	{
		_status = _send_flush_tx_command();
		ClearMAX_RT();
	}

	// Flushes RX_FIFO and clears RX_DR flag
	inline void FlushRXFIFO()
	{
		_status = _send_flush_rx_command();
		ClearRX_DR();
	}

	// Clears all interrupts flags
	inline void ClearInterrupts()
	{
		_status = _send_nop_command();
		_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
	}

	// Reads FIFO status from the chip
	inline S_FIFO_STATUS ReadFIFOStatus()
	{
		S_FIFO_STATUS ffs;
		_status = _read_register(RA_FIFO_STATUS, reinterpret_cast<uint8_t*>(&ffs), 1);
		return ffs;
	}

	// Reuse last transmitted payload
	inline void ReuseTX()
	{
		_status = _send_reuse_tx_pl_command();
	}

	// Reads status register
	inline S_STATUS ReadStatus()
	{
		return _status = _send_nop_command();
	}


	// Clears Maximum Retransmit interrupt flag
	inline void ClearMAX_RT()
	{
		_status = _send_nop_command();
		_status.RX_DR = 0;
		_status.TX_DS = 0;
		_status.MAX_RT = 1;
		_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
	}

	// Clears Data Ready RX FIFO interrupt flag
	inline void ClearRX_DR()
	{
		_status = _send_nop_command();
		_status.RX_DR = 1;
		_status.TX_DS = 0;
		_status.MAX_RT = 0;
		_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
	}

	// Clears Data Sent TX interrupt flag
	inline void ClearTX_DS()
	{
		_status = _send_nop_command();
		_status.RX_DR = 0;
		_status.TX_DS = 1;
		_status.MAX_RT = 0;
		_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
	}

	inline void WriteDynamicPayload(const S_DYNPD& dynpd = { 0, 0, 0, 0, 0, 0 })
	{
		_status = _write_register(RA_DYNPD, reinterpret_cast<const uint8_t*>(&dynpd), 1);
	}

	inline S_DYNPD ReadDynamicPayload() 
	{
		S_DYNPD res;
		_status = _read_register(RA_DYNPD, reinterpret_cast<uint8_t*>(&res), 1);
		return res;
	}

	inline void WriteDynamicPayload(const uint8_t &pipe, const bool &enabled)
	{
		uint8_t dynpd;
		_status = _read_register(RA_DYNPD, &dynpd, 1);
		if (enabled)
			dynpd = dynpd | 1 << (pipe <= 5? pipe : 5);
		else
			dynpd = dynpd & ~(1 << (pipe <= 5 ? pipe : 5));
		_status = _write_register(RA_DYNPD, &dynpd, 1);
	}

	// Clears all interrupt flags
	inline void ClearInterrupts()
	{
		_status = _send_nop_command();
		_status.RX_DR = 1;
		_status.TX_DS = 1;
		_status.MAX_RT = 1;
		_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
	}

	inline void SetPipeRXWidth(const uint8_t &pipe, const uint8_t &width)
	{
		_pipePayloadWidth[pipe <= 5 ? pipe : 5] = width & 0x3F;
		uint8_t addr = RA_RX_PW_P0 + (pipe <= 5 ? pipe : 5);
		_status = _write_register(RA_RX_PW_P0 + (pipe <= 5 ? pipe : 5), &(_pipePayloadWidth[pipe <= 5 ? pipe : 5]), 1);
		uint8_t res;
		_status = _read_register(RA_RX_PW_P0 + (pipe <= 5 ? pipe : 5), &res, 1);
	}

	inline uint8_t GetPipeRXWidth(const uint8_t& pipe)
	{
		uint8_t res = 0;
		_status = _read_register(RA_RX_PW_P0 + (pipe <= 5 ? pipe : 5), &res, 1);
		return res;
	}

	inline void SetupAA(const S_EN_AA &cfg)
	{
		_status = _write_register(RA_EN_AA, reinterpret_cast<const uint8_t*>(&cfg), 1);
	}

	void SetupAutoRetransmission(const S_SETUP_RETR& conf, const S_EN_AA& enable_aa = { 1,1,1,1,1,1 })
	{
		_status = _write_register(RA_SETUP_RETR, reinterpret_cast<const uint8_t*>(&conf), sizeof(conf));
		_status = _write_register(RA_EN_AA, reinterpret_cast<const uint8_t*>(&enable_aa), sizeof(enable_aa));
	}

	// Writes the configuration register
	void WriteConfig(const S_CONFIG& config)
	{
		_status = _write_register(RA_CONFIG, reinterpret_cast<const uint8_t*>(&config), 1);
	}

	S_CONFIG ReadConfig()
	{
		S_CONFIG res;
		_status = _read_register(RA_CONFIG, reinterpret_cast<uint8_t*>(&res), 1);
		return res;
	}

	// Writes feature register
	void WriteFeature(const S_FEATURE&& feature)
	{
		_status = _write_register(RA_FEATURE, reinterpret_cast<const uint8_t*>(&feature), 1);
	}

	// Reads feature register
	S_FEATURE ReadFeature()
	{
		S_FEATURE res;
		_status = _read_register(RA_FEATURE, reinterpret_cast<uint8_t*>(&res), 1);
		return res;
	}

	// Reads config and changes PWR_UP and PRIM_RX bits
	void PowerUp(bool PrimRX = false)
	{
		S_CONFIG config;
		_read_register(RA_CONFIG, reinterpret_cast<uint8_t*>(&config), 1);
		config.PWR_UP = 1;
		config.PRIM_RX = PrimRX ? 1 : 0;
		_status = _write_register(RA_CONFIG, reinterpret_cast<const uint8_t*>(&config), 1);
	}


	// Reads config and changes PWR_UP bit to 0
	void PowerDown()
	{
		S_CONFIG config;
		_status = _read_register(RA_CONFIG, reinterpret_cast<uint8_t*>(&config), 1);
		config.PWR_UP = 0;
		_status = _write_register(RA_CONFIG, reinterpret_cast<uint8_t*>(&config), 1);
	}


	//************************************
	// FullName:  NRF24L01::SetAdressWidth
	// Description: Sets address width
	// Parameter: uint8_t width, must be from 3 to 5
	//************************************
	void SetAdressWidth(uint8_t width)
	{
		if (width >= 3 && width <= 5)
		{
			_adress_width = width;
			width -= 2;
			_write_register(RA_SETUP_AW, &(width), sizeof(width));
		}
	}


	//************************************
	// FullName:    NRF24L01::ConfigureRf
	// Description: 
	// Parameter:   uint8_t RFChannel
	// Parameter:   RFSpeeds speed
	// Parameter:   bool continuousWave
	// Parameter:   RFOutPowers power
	//************************************
	void ConfigureRf(uint8_t RFChannel, RFSpeeds speed, bool continuousWave, RFOutPowers power)
	{
		uint8_t reg[2];
		_rf_ch = reg[0] = RFChannel & 0x7F;
		S_RF_SETUP *setup = reinterpret_cast<S_RF_SETUP*>(reg + 1);
		setup->CONT_WAVE = _continuesWave = continuousWave;
		setup->PLL_LOCK = 0;
		setup->RF_PWR = _power = power;
		_speed = speed;
		setup->RF_DR_HIGH = speed & 0x1;
		setup->RF_DR_LOW = speed >> 1;
		_status = _write_register(RegAddrs::RA_RF_CH, reg, 1);
		_status = _write_register(RegAddrs::RA_RF_SETUP, reg + 1, 1);
	};

	void SetRxAddr(uint8_t pipe, S_ADDR addr, bool forceFull = true)
	{
		if (pipe >= 0 && pipe <= 5)
		{
			uint64_t mask;
			for (int i = 0; i < _adress_width; ++i)
			{
				mask |= 0xFF << i * 8;
			}
			addr = mask & static_cast<uint64_t>(addr);
			if (pipe > 1)
			{
				if (pipe == 2)
				{
					_rx_addr_p2 = addr;
				}
				else if (pipe == 3)
				{
					_rx_addr_p3 = addr;
				}
				else if (pipe == 4)
				{
					_rx_addr_p4 = addr;
				}
				else if (pipe == 5)
				{
					_rx_addr_p5 = addr;
				}
				uint8_t addri = addr;
				_status = _write_register(RegAddrs::RA_RX_ADDR_P0 + pipe, &addri, 1);

				if (forceFull)
				{
					uint64_t addrl = addr;
					addrl = addrl & (~0xFF) | (static_cast<uint64_t>(_rx_addr_p1) & 0xFF);
					_rx_addr_p1 = addrl;
					_status = _write_register(RegAddrs::RA_RX_ADDR_P1, (uint8_t*)(&_rx_addr_p1), _adress_width);
				}
			}
			else
			{
				if (pipe == 0)
				{
					_rx_addr_p0 = addr;
				}
				else
				{
					_rx_addr_p1 = addr;
				}
				_status = _write_register(RegAddrs::RA_RX_ADDR_P0 + pipe, (uint8_t*)(&addr), _adress_width);
			}

		}
	}

	void SetTxAddr(S_ADDR addr)
	{
		S_ADDR rst;
		uint64_t mask;
		for (int i = 0; i < _adress_width << 1; ++i)
		{
			mask |= 0xFF << i * 8;
		}
		_tx_addr = static_cast<uint64_t>(addr) & mask;
		_status = _write_register(RegAddrs::RA_TX_ADDR, reinterpret_cast<uint8_t*>(&_tx_addr), _adress_width);
		_status = _read_register(RegAddrs::RA_TX_ADDR, reinterpret_cast<uint8_t*>(&rst), _adress_width);
	}




	NRF24L01(
		void (*communicateToSPIFunction)(const uint8_t* txbuff, const uint8_t& txlen, uint8_t* rxbuff, const uint8_t& rxlen),
		void (*recieveSPIFunction) (uint8_t* rxbuff, const uint8_t& rxlen),
		void (*sendSPIFunction) (const uint8_t* txbuff, const uint8_t& txlen),
		void (*setCSNPinState)(bool high),
		void (*setCEPinState) (bool high)
	);


	inline void InterruptsEnabled(bool isEnabled)
	{
		_interruptEnabled = isEnabled;
	}

	inline bool InterruptsEnabled()
	{
		return _interruptEnabled;
	}

	// Call this method on IRQ PIN high to low transition
	void InterruptMethod()
	{
		S_STATUS status = _send_nop_command();
		// Maximum number of retransmits interrupt
		if (status.MAX_RT)
		{
		}
		// Data ready on the RX FIFO interrupt
		if (status.RX_DR)
		{
		}
		// Packet transmitted interrupt, if AA is enabled, this bit is one only if ACK is received
		if (status.TX_DS)
		{
		}
	}





private:


	inline S_STATUS _write_register(uint8_t addr, const uint8_t* buff, const uint8_t &len)
	{
		_SetCSNPinState(false);
		S_STATUS status;
		uint8_t command = CT_W_REGISTER | (addr & 0x1F);
		_CommunicateToSPIFunction(&command, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SendSPIFunction(buff, len % 6);
		_SetCSNPinState(true);
		return status;
	}


	inline S_STATUS _read_register(uint8_t addr, uint8_t* buff, const uint8_t &len)
	{
		_SetCSNPinState(false);
		uint8_t send = CT_R_REGISTER | (addr & 0x1F);
		S_STATUS status;
		_CommunicateToSPIFunction(&send, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_RecieveSPIFunction(buff, len % 6);
		_SetCSNPinState(true);
		return status;
	}

	inline S_STATUS _send_flush_tx_command()
	{
		_SetCSNPinState(false);
		S_STATUS status;
		uint8_t send = CommandTypes::CT_FLUSH_TX;
		_CommunicateToSPIFunction(&send, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SetCSNPinState(true);
		return status;
	}

	inline S_STATUS _send_flush_rx_command()
	{
		_SetCSNPinState(false);
		S_STATUS status;
		uint8_t send = CommandTypes::CT_FLUSH_RX;
		_CommunicateToSPIFunction(&send, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SetCSNPinState(true);
		return status;
	}

	inline S_STATUS _send_reuse_tx_pl_command()
	{
		_SetCSNPinState(false);
		S_STATUS status;
		uint8_t send = CommandTypes::CT_REUSE_TX_PL;
		_CommunicateToSPIFunction(&send, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SetCSNPinState(true);
		return status;
	}

	inline R_RX_PL_RESP _send_r_rx_pl_wid_command()
	{
		_SetCSNPinState(false);
		uint8_t command = CommandTypes::CT_R_RX_PL_WID;
		R_RX_PL_RESP res;
		_CommunicateToSPIFunction(&command, 1, reinterpret_cast<uint8_t*>(&(res.status)), 1);
		_RecieveSPIFunction(reinterpret_cast<uint8_t*>(&(res.len)), 1);
		_SetCSNPinState(true);
		return res;
	}

	inline S_STATUS _send_nop_command()
	{
		_SetCSNPinState(false);
		S_STATUS status;
		uint8_t send = CommandTypes::CT_NOP;
		_CommunicateToSPIFunction(&send, 1, (uint8_t*)(&status), 1);
		_SetCSNPinState(true);
		return status;
	}

	inline S_STATUS _read_rx_fifo(uint8_t *buffer, const uint8_t &size)
	{
		_SetCSNPinState(false);
		uint8_t command = CommandTypes::CT_R_RX_PAYLOAD;
		S_STATUS status;
		_CommunicateToSPIFunction(&command, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_RecieveSPIFunction(buffer, size % 33);
		_SetCSNPinState(true);
		return status;
	}


	inline S_STATUS _write_tx_fifo(const uint8_t* buffer, const uint8_t &size)
	{
		_SetCSNPinState(false);
		uint8_t command = CommandTypes::CT_W_TX_PAYLOAD;
		S_STATUS status;
		_CommunicateToSPIFunction(&command, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SendSPIFunction(buffer, size % 33);
		_SetCSNPinState(true);
		return status;
	}

	inline S_STATUS _write_tx_fifo_no_ack(const uint8_t* buffer, const uint8_t& size)
	{
		_SetCSNPinState(false);
		uint8_t command = CommandTypes::CT_W_TX_PAYLOAD_NO_ACK;
		S_STATUS status;
		_CommunicateToSPIFunction(&command, 1, reinterpret_cast<uint8_t*>(&status), 1);
		_SendSPIFunction(buffer, size % 33);
		_SetCSNPinState(true);
		return status;
	}


	void (*_CommunicateToSPIFunction) (const uint8_t* txbuff, const uint8_t& txlen, uint8_t* rxbuff, const uint8_t &rxlen);
	void (*_RecieveSPIFunction) (uint8_t* rxbuff, const uint8_t &rxlen);
	void (*_SendSPIFunction) (const uint8_t* txbuff, const uint8_t& txlen);

	void (*_SetCEPinState) (bool high);

	void (*_SetCSNPinState)(bool high);


	RFSpeeds _speed = RFSpeeds::SpeedHigh;

	RFOutPowers _power = RFOutPowers::High;

	ChipStates _currentState = ChipStates::CS_Undefined;

	bool _interruptEnabled = false;

	bool _continuesWave = false;


	// Last status received
	S_STATUS _status;

	uint8_t _rf_ch = 2;

	uint8_t _adress_width = 5;

	S_ADDR _rx_addr_p0 =  0xE7E7E7E7E7;
	S_ADDR _rx_addr_p1 =  0xC2C2C2C2C2;
	uint8_t _rx_addr_p2 = 0xC3;
	uint8_t _rx_addr_p3 = 0xC4;
	uint8_t _rx_addr_p4 = 0xC5;
	uint8_t _rx_addr_p5 = 0xC6;
	S_ADDR _tx_addr =     0xE7E7E7E7E7;

	S_EN_AA _enable_aa = S_EN_AA{ 1, 1, 1, 1, 1, 1 };

	uint8_t _pipePayloadWidth[6] = { 0, 0, 0, 0, 0, 0 };

};


#endif // !NRF24L01_H
