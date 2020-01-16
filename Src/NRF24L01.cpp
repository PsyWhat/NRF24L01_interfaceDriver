
#include "NRF24L01.hpp"

NRF24L01::Packet NRF24L01::Recieve(int timeout)
{
	register int i = 0;
	_SetCEPinState(true);
	while (true)
	{
		S_FIFO_STATUS fifostat;
		_status = _read_register(RA_FIFO_STATUS, reinterpret_cast<uint8_t*>(&fifostat), sizeof(fifostat));
		if (!fifostat.RX_EMPTY)
		{
			uint8_t pipe = _status.RX_P_NO;
			uint8_t bytes = 0;
			uint8_t isdynpd;
			_status.MAX_RT = 0;
			_status.TX_DS = 0;
			_status = _write_register(RA_STATUS, reinterpret_cast<uint8_t*>(&_status), 1);
			_read_register(RA_DYNPD, &isdynpd, 1);
			if (isdynpd & 1 << pipe)
			{
				R_RX_PL_RESP res = _send_r_rx_pl_wid_command();
				_status = res.status;
				bytes = res.len;
			}
			else
			{
				_read_register(RA_RX_PW_P0 + pipe, &bytes, sizeof(bytes));
			}
			Packet pack(bytes, pipe);
			_status = _read_rx_fifo(pack.buffer, pack.len);
			_read_register(RA_FIFO_STATUS, reinterpret_cast<uint8_t*>(&fifostat), sizeof(fifostat));
			return pack;
		}
		if (++i > timeout && timeout >= 0)
		{
			return Packet::Timeout();
		}
	}
}

NRF24L01::TransmitStatus NRF24L01::Transmit(const uint8_t* buff, uint8_t len)
{
	_SetCEPinState(true);
	S_FIFO_STATUS fifostat;
	do
	{
		_status = _read_register(RA_FIFO_STATUS, reinterpret_cast<uint8_t*>(&fifostat), 1);
		if (!fifostat.TX_EMPTY)
		{
			if (_status.MAX_RT)
				return TS_FIFO_Buisy_MaxRetransmit;
		}
	} while (!fifostat.TX_EMPTY);
	_write_tx_fifo(buff, len);
	while (true)
	{
		_status = _read_register(RA_FIFO_STATUS, reinterpret_cast<uint8_t*>(&fifostat), 1);
		if (fifostat.TX_EMPTY)
		{
			return TS_Success;
		}
		if (_status.MAX_RT)
		{
			return TS_MaxRetransmit;
		}
	}
}

NRF24L01::NRF24L01(void (*communicateToSPIFunction)(const uint8_t* txbuff, const uint8_t& txlen, uint8_t* rxbuff, const uint8_t& rxlen), void (*recieveSPIFunction) (uint8_t* rxbuff, const uint8_t& rxlen), void (*sendSPIFunction) (const uint8_t* txbuff, const uint8_t& txlen), void (*setCSNPinState)(bool high), void (*setCEPinState) (bool high))
{
	_CommunicateToSPIFunction = communicateToSPIFunction;
	_RecieveSPIFunction = recieveSPIFunction;
	_SendSPIFunction = sendSPIFunction;
	_SetCEPinState = setCEPinState;
	_SetCSNPinState = setCSNPinState;
}

NRF24L01::Packet NRF24L01::Packet::Timeout()
{
	return Packet(0, 0xff);
}

NRF24L01::Packet::Packet(const uint8_t& len, const uint8_t& pipe)
{
	this->pipe = pipe;
	this->len = len;
	if (len > 0)
	{
		buffer = new uint8_t[this->len];
	}
	else
	{
		buffer = NULL;
	}
}

NRF24L01::Packet::Packet(const uint8_t* buff, const uint8_t& len, const uint8_t& pipe)
{
	this->pipe = pipe;
	if ((this->len = len) > 0 && buff != NULL)
	{
		buffer = new uint8_t[len];
		memcpy(buffer, buff, len);
	}
	else
	{
		buffer = NULL;
	}
}

NRF24L01::Packet::Packet(const Packet&& copy)
{
	pipe = copy.pipe;
	if ((len = copy.len) > 0)
	{
		buffer = new uint8_t[len];
		memcpy(buffer, copy.buffer, len);
	}
	else
	{
		buffer = NULL;
	}
}

NRF24L01::Packet::~Packet()
{
	delete[] buffer;
}
