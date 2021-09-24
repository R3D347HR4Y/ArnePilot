# def create_steer_command(packer, steer, steer_req, raw_cnt):
#   """Creates a CAN message for the Seb Smith EPAS Steer Command."""

#   values = {
#     "STEER_REQUEST": steer_req,
#     "STEER_TORQUE_CMD": steer,
#     "COUNTER": raw_cnt,
#     "SET_ME_1": 1,
#   }
#   return packer.make_can_msg("STEERING_LKA", 0, values)

def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_command_ocelot(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 2, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 2, values)

def create_steer_command(packer, enabled, steer):
  values = {
    "STEER_TORQUE_CMD": (steer * 8192) if enabled else 0.
  }
  return packer.make_can_msg("STEER_COMMAND", 2, values)

#def create_gas_command(packer, gas_amount, idx):
#  # Common gas pedal msg generator
#  enable = gas_amount > 0.001
#
#  values = {
#    "ENABLE": enable,
#    "COUNTER_PEDAL": idx & 0xF,
#
#  }
#
#  if enable:
#    values["GAS_COMMAND"] = gas_amount * 255.
#    values["GAS_COMMAND2"] = gas_amount * 255.
#
#  return packer.make_can_msg("GAS_COMMAND", 2, values)

def create_ibst_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : brake * 25,
    "BRAKE_RELATIVE_COMMAND" : 0, #brake * 252,
    "BRAKE_MODE": 2 if enabled else 0,
    "COUNTER" : raw_cnt,
  }
 
  return packer.make_can_msg("BRAKE_COMMAND", 2, values)
