from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  return mqb_create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0):
  commands = []

  # ACC_05: accel/decel request to gearbox, ESP, EPB, and motor
  # ACC_01 is not used on MLB (Macan) -- the stock radar only sends ACC_05
  #
  # Stock radar behavior observed from Cabana at 77 km/h steady cruise:
  #   - ACC_Momentenanforderung: 173 Nm (engine torque request - primary accel control)
  #   - ACC_Verz_anf: 0.0 (zero during cruise/accel, negative during braking)
  #   - ACC_ax_Getriebe: 0.0 (zero at cruise, positive for accel, negative for braking)
  #   - ACC_Freigabe_Momentenanf: 1 (torque request enabled)
  #   - ACC_Freigabe_Verzanf: 0 (decel NOT requested during cruise)
  #   - ACC_Vorbefuellung_Bremsanlage: 0 (brake pre-fill OFF)
  #
  # Control architecture:
  #   Acceleration: engine torque via ACC_Momentenanforderung, ACC_Verz_anf = 0
  #   Braking: decel via ACC_Verz_anf (negative), ACC_Momentenanforderung = 0

  # Braking deadband: filter PID noise so we don't rapidly switch between torque and brake mode.
  # Small negative accel (-0.1 to 0) is handled by reducing engine torque (engine braking).
  # The ACC_ax_Getriebe >= 0 clamp in torque mode prevents the gearbox conflict that
  # previously caused brake stabs, so a tight deadband is safe now.
  braking = acc_enabled and accel < -0.1

  # Engine torque request (ACC_Momentenanforderung, 0-1021 Nm)
  #
  # Drag torque (steady cruise): combines two models for best accuracy across all speeds:
  #   Cabana-verified model: accurate at highway speed (77 km/h: 173 Nm, 119 km/h: 217 Nm)
  #   Statistical fit (214k pts): better at low speed (standstill: 63 Nm, 20 km/h: 71 Nm)
  #   Uses max() of both -- Cabana model wins above ~48 km/h, fitted wins below.
  #   Without the fitted model, standstill torque was only 30 Nm (too low to move the car).
  #
  # Accel gain (additional torque per m/s² of acceleration):
  #   Quadratic fit from stock data, floored at 63 to prevent dip at ~10 km/h
  #   gain = max(min(1.1 * v² - 6.5 * v + 63, 300), 63)
  if acc_enabled and not braking:
    cabana_drag = min(0.35 * v_ego ** 2 + 30, 0.069 * v_ego ** 2 + 141)
    fitted_drag = 0.0884 * v_ego ** 2 + 0.96 * v_ego + 63.4
    drag_torque = max(cabana_drag, fitted_drag)
    accel_gain = max(min(1.1 * v_ego ** 2 - 6.5 * v_ego + 63, 300), 63)
    accel_torque = accel * accel_gain
    acc_moment = int(max(0, min(500, drag_torque + accel_torque)))
  else:
    acc_moment = 0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    # Stock ACC_Verz_anf range during braking: -2.015 to 0
    # Allow slightly beyond stock (-2.5 vs -2.0) for more decisive braking (curves, stops)
    "ACC_Verz_anf": max(accel, -2.5) if braking else (0 if acc_enabled else 3.01),
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0,
    # Stock ACC_ax_Getriebe: 0 during cruise, positive during accel, negative only during braking
    # Torque mode: clamp to >= 0 (stock never sends negative ax while engine torque is active)
    #   When accelerating (accel > 0.1): floor at 0.3 to prevent premature upshifting
    #   (PDK shifts to 6th at 40 mph if ax is too low, starving torque)
    # Braking: allow negative, with speed-dependent lower limit (ramps in with speed)
    #   At 10 km/h: -1.4, 20 km/h: -2.2, 24+ km/h: -2.5 (slightly beyond stock -2.016)
    "ACC_ax_Getriebe": (max(min(accel, min(1.8 + 0.015 * v_ego * 3.6, 2.5)),
                             (0.3 if accel > 0.1 else 0)) if not braking else
                         max(accel, max(-2.5, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Anhalten": stopping,
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": lead_distance,
  }

  return packer.make_can_msg("ACC_02", bus, values)

def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x111: 0x10, # TSK_05
    0x30C: 0x0F, # ACC_02
    0x324: 0x27, # ACC_04
    0x10B: 0xA,  # LS_01
    0x10D: 0x0C, # ACC_05
    0x10F: 0x0E, # ACC_0x10F
    0x311: 0x12, # ACC_0x311
    0x397: 0x94, # LDW_02
    0x10C: 0x0D, # TSK_02
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return volkswagen_mqb_meb_checksum(address, sig, d)
