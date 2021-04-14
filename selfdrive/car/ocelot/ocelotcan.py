def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Seb Smith EPAS Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)

def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("GAS_COMMAND", 0, values)

def create_ibst_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : 0,
    "BRAKE_RELATIVE_COMMAND": brake * 252,
    "BRAKE_MODE": enabled,
    "COUNTER" : raw_cnt,
  }
 
  return packer.make_can_msg("BRAKE_COMMAND", 0, values)