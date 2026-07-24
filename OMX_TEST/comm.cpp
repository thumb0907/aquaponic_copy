#include "comm.h"

namespace
{
constexpr uint8_t SOF = 0xAA;
constexpr uint8_t MAX_DATA_LEN = 16;

enum RxState : uint8_t
{
  RX_WAIT_SOF,
  RX_WAIT_PID,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_CHECKSUM
};

Stream *comm_port = nullptr;

RxState rx_state = RX_WAIT_SOF;

uint8_t rx_pid = 0;
uint8_t rx_len = 0;
uint8_t rx_index = 0;
uint8_t rx_checksum = 0;
uint8_t rx_data[MAX_DATA_LEN];

bool harvest_job_pending = false;
uint8_t pending_job_id = 0;

bool estop_pending = false;
bool reset_pending = false;

void resetParser()
{
  rx_state = RX_WAIT_SOF;
  rx_pid = 0;
  rx_len = 0;
  rx_index = 0;
  rx_checksum = 0;
}

void handleFrame()
{
  if (rx_pid == PID_ROBOT_JOB)
  {
    // DATA:
    // [0] job_type
    // [1] source
    // [2] destination
    // [3] job_id
    if (
      rx_len == 4 &&
      rx_data[0] == JOB_HARVEST &&
      rx_data[1] == 0x00 &&
      rx_data[2] == 0x00
    )
    {
      pending_job_id = rx_data[3];
      harvest_job_pending = true;
    }

    return;
  }

  if (rx_pid == PID_ESTOP && rx_len == 0)
  {
    estop_pending = true;
    return;
  }

  if (rx_pid == PID_RESET && rx_len == 0)
  {
    reset_pending = true;
    return;
  }
}

void feedByte(uint8_t value)
{
  switch (rx_state)
  {
    case RX_WAIT_SOF:
      if (value == SOF)
      {
        rx_state = RX_WAIT_PID;
      }
      break;

    case RX_WAIT_PID:
      rx_pid = value;
      rx_state = RX_WAIT_LEN;
      break;

    case RX_WAIT_LEN:
      rx_len = value;
      rx_index = 0;
      rx_checksum = rx_pid + rx_len;

      if (rx_len > MAX_DATA_LEN)
      {
        resetParser();
      }
      else if (rx_len == 0)
      {
        rx_state = RX_WAIT_CHECKSUM;
      }
      else
      {
        rx_state = RX_WAIT_DATA;
      }
      break;

    case RX_WAIT_DATA:
      rx_data[rx_index++] = value;
      rx_checksum += value;

      if (rx_index >= rx_len)
      {
        rx_state = RX_WAIT_CHECKSUM;
      }
      break;

    case RX_WAIT_CHECKSUM:
      if (value == rx_checksum)
      {
        handleFrame();
      }
      else
      {
        Serial.println("[COMM] CHECKSUM ERROR");
      }

      resetParser();
      break;
  }
}

void sendFrame(
  uint8_t pid,
  const uint8_t *data,
  uint8_t len
)
{
  if (comm_port == nullptr)
  {
    return;
  }

  uint8_t checksum =
    static_cast<uint8_t>(pid + len);

  comm_port->write(SOF);
  comm_port->write(pid);
  comm_port->write(len);

  for (uint8_t i = 0; i < len; ++i)
  {
    comm_port->write(data[i]);
    checksum =
      static_cast<uint8_t>(checksum + data[i]);
  }

  comm_port->write(checksum);
}
}

void commBegin(Stream &port)
{
  comm_port = &port;
  resetParser();
}

void commPoll()
{
  if (comm_port == nullptr)
  {
    return;
  }

  while (comm_port->available() > 0)
  {
    const int value = comm_port->read();

    if (value >= 0)
    {
      feedByte(static_cast<uint8_t>(value));
    }
  }
}

bool commTakeHarvestJob(uint8_t &job_id)
{
  if (!harvest_job_pending)
  {
    return false;
  }

  harvest_job_pending = false;
  job_id = pending_job_id;

  return true;
}

bool commTakeEstop()
{
  if (!estop_pending)
  {
    return false;
  }

  estop_pending = false;
  return true;
}

bool commTakeReset()
{
  if (!reset_pending)
  {
    return false;
  }

  reset_pending = false;
  return true;
}

void commSendState(
  uint8_t state,
  uint8_t job_id
)
{
  const uint8_t data[2] = {
    state,
    job_id
  };

  sendFrame(PID_STATE, data, 2);
}

void commSendDone(uint8_t job_id)
{
  const uint8_t data[2] = {
    JOB_HARVEST,
    job_id
  };

  sendFrame(PID_DONE, data, 2);
}

void commSendError(
  uint8_t job_id,
  uint8_t error_code
)
{
  const uint8_t data[3] = {
    JOB_HARVEST,
    job_id,
    error_code
  };

  sendFrame(PID_ERR, data, 3);
}