def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Seb Smith EPAS Steer Command."""

  values = {
    "STEER_MODE": steer_req,
    "REQUESTED_STEER_TORQUE": steer,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("STEERING_COMMAND", 0, values)

def create_pedal_command(packer, gas_amount, raw_cnt):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER": raw_cnt,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("GAS_COMMAND", 0, values)

def create_ibst_command(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : brake * 40,
    "BRAKE_RELATIVE_COMMAND": 0,
    "BRAKE_MODE": enabled * 2,
    "COUNTER" : raw_cnt,
  }

  return packer.make_can_msg("BRAKE_COMMAND", 0, values)

def create_msg_command(packer, enabled, setspeed, currspeed):
  values = {
    "ENABLED" : enabled,
    "SET_SPEED" : setspeed,
    "CURRENT_SPEED" : currspeed,
  }
  return packer.make_can_msg("CURRENT_STATE", 0, values)