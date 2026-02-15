from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, CanBus, VolkswagenFlags

# MLB platform uses processed ACC radar data (single lead vehicle) from ACC_02 and ACC_04 messages.
# These are sent by the stock radar ECU on the ext bus even when openpilot controls ACC.
#
# ACC_Abstandsindex: NOT a pure distance -- it's a composite index that depends on
#   ACC_Gesetzte_Zeitluecke (time gap setting). Each Zeitluecke level has a different
#   index-to-distance mapping. Calibrated from 70,471 paired samples (1 route, 75 segments):
#
#   Zeitluecke=1: dist = 0.3654 * index - 9.52   (82% match, good at 0-30m)
#   Zeitluecke=2: dist = 0.3184 * index - 13.84  (82% match, 95% at 30-50m)
#   Zeitluecke=3: dist = 0.1989 * index + 1.05   (77% match, 91-93% at 30-80m)
#   Zeitluecke=4: dist = 0.1815 * index + 23.49  (87% match, 97% at 80-120m)
#
#   Overall per-Zeitluecke model: 81.7% match, RMSE=12.4m (vs 70% without Zeitluecke)
#
# ACC_Relevantes_Objekt: 0 = no relevant object, 1 = lead vehicle detected
# ACC_Geschw_Zielfahrzeug: lead vehicle absolute speed in km/h (accurate, radar Doppler)

# Per-Zeitluecke distance calibration: {ZL: (slope, intercept)}
# Calibrated from 70,471 paired samples (1 route, 75 segments):
#   Overall match rate: 81.7%, RMSE=12.4m (vs 70% without Zeitluecke)
DIST_CAL = {
    1: (0.3654, -9.52),    # 82% match, RMSE=5.8m, good at 0-30m
    2: (0.3184, -13.84),   # 82% match, RMSE=8.4m, 95% at 30-50m
    3: (0.1989, 1.05),     # 77% match, RMSE=16.5m, 91-93% at 30-80m
    4: (0.1815, 23.49),    # 87% match, RMSE=15.7m, 97% at 80-120m
}
DIST_CAL_DEFAULT = (0.1815, 23.49)  # fallback to ZL=4 calibration
DIST_MAX = 120.0   # cap max reported distance

# Message addresses for triggering
ACC_04_ADDR = 0x324  # 804 decimal, trigger message (arrives after ACC_02)


def get_radar_can_parser_mlb(CP):
  bus = CanBus(CP)

  # Radar signals on ext bus (camera side, bus 2 for gateway network)
  ext_messages = [
    ("ACC_02", 16),   # ~16 Hz, has ACC_Abstandsindex, ACC_Relevantes_Objekt, ACC_Gesetzte_Zeitluecke
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
    zeitluecke = int(acc02["ACC_Gesetzte_Zeitluecke"])
    lead_speed_kph = acc04["ACC_Geschw_Zielfahrzeug"]

    has_lead = obj_status > 0 and dist_index > 0

    if has_lead:
      if 0 not in self.pts:
        self.pts[0] = structs.RadarData.RadarPoint()
        self.pts[0].trackId = self.track_id
        self.track_id += 1

      lead_speed = lead_speed_kph * CV.KPH_TO_MS
      slope, intercept = DIST_CAL.get(zeitluecke, DIST_CAL_DEFAULT)
      dRel = min(max(slope * dist_index + intercept, 1.0), DIST_MAX)
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
