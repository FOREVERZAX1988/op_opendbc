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


# Braking mode state for hysteresis (prevents rapid mode switching that causes brake stabs)
_braking_prev = False


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego=0):
  global _braking_prev
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

  # Braking mode with hysteresis to prevent rapid mode switching.
  # Hysteresis ensures we only enter braking for meaningful decel requests (curves, stops)
  # and stay committed until the planner clearly wants to cruise/accelerate again.
  #   Enter braking: accel < -0.18 (responsive to brake requests; tighter now that
  #     the drag torque model is stock-calibrated and planner no longer oscillates at -0.2)
  #   Exit braking:  accel > -0.05  (planner clearly wants cruise/accel)
  # In between (-0.18 to -0.05), mild decel is handled by reducing engine torque.
  if acc_enabled:
    if _braking_prev:
      # At low speed, require positive accel to release brakes. Near standstill the
      # planner naturally eases off (e.g. -0.04) which isn't "wants to go" -- it's
      # just reducing brake pressure as the car slows. Without this, 45 Nm of drag
      # torque at standstill creeps the car through red lights.
      exit_threshold = -0.05 if v_ego > 2.0 else 0.0
      braking = accel < exit_threshold
    else:
      braking = accel < -0.18   # enter braking for curves/stops
    # Keep braking committed during stops -- planner accel can fluctuate near 0
    # and briefly cross -0.05, which would release brakes mid-stop without this
    if stopping:
      braking = True
  else:
    braking = False
  _braking_prev = braking

  # Engine torque request (ACC_Momentenanforderung, 0-1021 Nm)
  #
  # Drag torque (steady cruise): quadratic fit from 79k stock ACC samples
  #   filtered at |ACC_ax_Getriebe| < 0.1 (pure cruise, no accel/decel intent).
  #   Matches stock within ~12 Nm across 20-140 kph (RMSE 6.8 Nm).
  #   Previous model overshot by 19-55 Nm at highway speeds, causing the e2e
  #   planner to constantly request mild decel and oscillate near the brake threshold.
  #
  # Accel gain (additional torque per m/s² of acceleration):
  #   Quadratic fit from stock data, floored at 63 to prevent dip at ~10 km/h
  #   gain = max(min(1.1 * v² - 6.5 * v + 63, 300), 63)
  #
  # Torque taper: as accel approaches the braking threshold (-0.18), torque is
  # smoothly faded to 0. Taper starts at -0.1.
  # At -0.14: fade=0.5, ~65 Nm at highway (smooth engine braking).
  # At -0.18: fade=0, seamless handoff to braking mode.
  if acc_enabled and not braking:
    drag_torque = 0.0564 * v_ego ** 2 + 2.671 * v_ego + 45.54
    accel_gain = max(min(1.1 * v_ego ** 2 - 6.5 * v_ego + 63, 300), 63)
    # 1.3x torque boost on the accel component only (drag stays calibrated for cruise).
    # Planner maxes at ~1.0 m/s² while stock requests 2.5 -- this partially compensates.
    accel_torque = accel * accel_gain * 1.3
    acc_moment = int(max(0, min(500, drag_torque + accel_torque)))
    # Smooth taper: fade torque to 0 as accel approaches braking threshold (-0.18).
    # Only taper for meaningful decel requests (below -0.1), not mild ones.
    # For mild decel (0 to -0.1), the accel_torque component naturally reduces
    # torque by a few Nm, which is appropriate.
    if accel < -0.1:
      fade = max(0.0, (accel + 0.18) / 0.08)  # 1.0 at -0.1, 0.0 at -0.18
      acc_moment = int(acc_moment * fade)
  else:
    acc_moment = 0

  # Stock ACC signal behavior observed from Cabana:
  #   Cruise/Accel: ACC_Verz_anf=0, ACC_Freigabe_Verzanf=0, ACC_ax_Getriebe=positive, torque enabled
  #   Braking:      ACC_Verz_anf=negative, ACC_Freigabe_Verzanf=1, ACC_ax_Getriebe=negative, torque=0
  #   Disabled:     ACC_Verz_anf=3.01, all others=0
  acc_05_values = {
    "ACC_Status_ACC": acc_control,
    # Stock ACC_Verz_anf range during braking: -2.015 to 0
    # Panda safety allows -3.5; DBC allows -7.22. Use panda limit for max braking.
    "ACC_Verz_anf": max(accel, -3.5) if braking else (0 if acc_enabled else 3.01),
    "ACC_Freigabe_Verzanf": 1 if braking else 0,
    "ACC_Freigabe_Momentenanf": 1 if (acc_enabled and not braking) else 0,
    "ACC_Momentenanforderung": acc_moment,
    "ACC_zul_Regelabw": 0,
    # Stock ACC_ax_Getriebe: positive during accel, negative during braking, ~0 at cruise
    # Tells the PDK what acceleration to expect, influencing gear selection.
    # DBC: 9-bit unsigned, range [-2.016, +10.248]. Values below -2.016 WRAP to ~+10
    # (unsigned overflow), sending a massive accel request to the PDK during braking!
    #   Accel (positive): 2.5x multiplier to match stock ACC range (1.5-2.5 m/s²).
    #     Floor at 1.5 for any positive accel to force PDK downshifts.
    #     At 35 mph in 6th gear the PDK needs a strong signal (1.5-2.5) to drop
    #     2-3 gears. Previous 1.7x/0.5 floor left the car in tall gears with no
    #     wheel torque; 1.0 floor still wasn't enough for multi-gear downshifts.
    #   Mild decel (torque taper zone): allow gentle negative values (capped at -0.3)
    #     to signal the PDK to downshift, preparing for re-acceleration.
    #     Torque is already fading in this zone, so no conflict with engine torque.
    #   Braking: clamped to DBC min -2.016 (stock max observed: -2.015)
    "ACC_ax_Getriebe": (max(min(accel * 2.5, min(1.8 + 0.015 * v_ego * 3.6, 2.5)),
                             (1.5 if accel > 0.0 else max(accel, -0.3))) if not braking else
                         max(accel, max(-2.016, -0.6 - 0.08 * v_ego * 3.6))) if acc_enabled else 0,
    "ACC_Vorbefuellung_Bremsanlage": 1 if braking else 0,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Anhalten": stopping,
    "ACC_Betaetigung_EPB": esp_hold,  # Echo ESP hold state -- DO NOT use stopping (causes brake release when ACC off)
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, lead_object=0, zeitluecke=4):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": zeitluecke,  # Mirror stock radar's ZL from ext bus (responds to DIST button)
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": lead_distance,
    "ACC_Relevantes_Objekt": lead_object,
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
