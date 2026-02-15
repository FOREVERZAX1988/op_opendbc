from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, CanBus, VolkswagenFlags

# MLB platform uses processed ACC radar data (single lead vehicle) from ACC_02 and ACC_04 messages.
# These are sent by the stock radar ECU on the ext bus even when openpilot controls ACC.
#
# ACC_Abstandsindex: NOT a pure distance -- it's a composite radar index.
#   The index is non-monotonic w.r.t. actual distance (dips at index 175-250),
#   likely influenced by relative speed, radar cross-section, or TTC.
#   Best-effort linear calibration from 13,748 paired samples across 2 routes:
#     dist_m = 0.0984 * index + 38.37  (RMSE=18.5m, 71% overall match rate)
#   Performs well at 50-120m range (83-85% match) where radar adds most value.
#   Close-range (<30m) is unreliable -- vision is primary there anyway.
# ACC_Relevantes_Objekt: 0 = no relevant object, 1 = lead vehicle detected
# ACC_Geschw_Zielfahrzeug: lead vehicle absolute speed in km/h (accurate, radar Doppler)

# Calibrated distance model: dist = DIST_A * index + DIST_B
# Combined linear fit from 13,748 paired radar-index vs vision-distance samples (2 routes)
DIST_A = 0.0984   # meters per index unit
DIST_B = 38.37    # offset in meters
DIST_MAX = 120.0  # cap max reported distance

# Message addresses for triggering
ACC_04_ADDR = 0x324  # 804 decimal, trigger message (arrives after ACC_02)


def get_radar_can_parser_mlb(CP):
  bus = CanBus(CP)

  # Radar signals on ext bus (camera side, bus 2 for gateway network)
  ext_messages = [
    ("ACC_02", 16),   # ~16 Hz, has ACC_Abstandsindex and ACC_Relevantes_Objekt
    ("ACC_04", 16),   # ~16 Hz, has ACC_Geschw_Zielfahrzeug
  ]

  # Wheel speeds on pt bus (bus 0) for computing ego speed / vRel
  pt_messages = [
    ("ESP_03", 50),   # 50 Hz wheel speeds
  ]

  ext_parser = CANParser(DBC[CP.carFingerprint][Bus.pt], ext_messages, bus.ext)
  pt_parser = CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, bus.pt)
  return ext_parser, pt_parser


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.track_id = 0
    self.v_ego = 0.0

    self.is_mlb = bool(CP.flags & VolkswagenFlags.MLB)
    self.radar_off_can = CP.radarUnavailable

    if self.is_mlb and not self.radar_off_can:
      self.ext_parser, self.pt_parser = get_radar_can_parser_mlb(CP)
      self.trigger_msg = ACC_04_ADDR
    else:
      self.ext_parser = None
      self.pt_parser = None
      self.trigger_msg = None

    # For base class compatibility
    self.rcp = self.ext_parser

  def update(self, can_strings):
    if self.radar_off_can or self.ext_parser is None:
      return super().update(None)

    # Update both parsers with all CAN data
    vls_ext = self.ext_parser.update(can_strings)
    self.pt_parser.update(can_strings)
    self.updated_messages.update(vls_ext)

    # Update ego speed from wheel speeds
    esp03 = self.pt_parser.vl["ESP_03"]
    wheel_speeds = [
      esp03["ESP_VL_Radgeschw"],
      esp03["ESP_VR_Radgeschw"],
      esp03["ESP_HL_Radgeschw"],
      esp03["ESP_HR_Radgeschw"],
    ]
    self.v_ego = sum(wheel_speeds) / 4.0 * CV.KPH_TO_MS

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update()
    self.updated_messages.clear()
    return rr

  def _update(self):
    ret = structs.RadarData()
    if self.ext_parser is None:
      return ret

    if not self.ext_parser.can_valid:
      ret.errors.canError = True
      return ret

    acc02 = self.ext_parser.vl["ACC_02"]
    acc04 = self.ext_parser.vl["ACC_04"]

    dist_index = acc02["ACC_Abstandsindex"]
    obj_status = acc02["ACC_Relevantes_Objekt"]
    lead_speed_kph = acc04["ACC_Geschw_Zielfahrzeug"]

    has_lead = obj_status > 0 and dist_index > 0

    if has_lead:
      if 0 not in self.pts:
        self.pts[0] = structs.RadarData.RadarPoint()
        self.pts[0].trackId = self.track_id
        self.track_id += 1

      lead_speed = lead_speed_kph * CV.KPH_TO_MS
      dRel = min(max(DIST_A * dist_index + DIST_B, 1.0), DIST_MAX)  # clamp to [1m, 120m]
      vRel = lead_speed - self.v_ego

      self.pts[0].measured = True
      self.pts[0].dRel = dRel
      self.pts[0].yRel = 0.0       # no lateral offset available
      self.pts[0].vRel = vRel
      self.pts[0].aRel = float('nan')  # not available from these messages
      self.pts[0].yvRel = float('nan')

    else:
      # No lead vehicle detected
      if 0 in self.pts:
        del self.pts[0]

    ret.points = list(self.pts.values())
    return ret
