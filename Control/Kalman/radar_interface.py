#!/usr/bin/env python3
# JLL, 2022.6.20 from /OP082/selfdrive/car/toyota/radar_interface.py
from opendbc.can.parser import CANParser
from cereal import car
from selfdrive.car.toyota.values import NO_DSU_CAR, DBC, TSS2_CAR
from selfdrive.car.interfaces import RadarInterfaceBase

def _create_radar_can_parser(car_fingerprint):
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))  # [384, 385, ..., 399], see opendbc/toyota_tss2_adas.dbc
    RADAR_B_MSGS = list(range(0x190, 0x1a0))  # [400, 401, ..., 415]
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)  # = 16
  msg_b_n = len(RADAR_B_MSGS)  # = 16

  signals = list(zip(['LONG_DIST'] * msg_a_n + ['NEW_TRACK'] * msg_a_n + ['LAT_DIST'] * msg_a_n +
                ['REL_SPEED'] * msg_a_n + ['VALID'] * msg_a_n + ['SCORE'] * msg_b_n,
                RADAR_A_MSGS * 5 + RADAR_B_MSGS,
                [255] * msg_a_n + [1] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_b_n))
    # = [('LONG_DIST', 384, 255), ..., ('LONG_DIST', 399, 255),
    #    ('NEW_TRACK', 384, 1), ...,   ('NEW_TRACK', 399, 1),
    #     ('LAT_DIST', 384, 0), ...,    ('LAT_DIST', 399, 0),
    #    ('REL_SPEED', 384, 0), ...,   ('REL_SPEED', 399, 0),
    #        ('VALID', 384, 0), ...,       ('VALID', 399, 0),
    #        ('SCORE', 400, 0), ...,       ('SCORE', 415, 0)]
  checks = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20]*(msg_a_n + msg_b_n)))
    # = [(384, 20), ..., (399, 20), (400, 20), ..., (415, 20)]

  return CANParser(DBC[car_fingerprint]['radar'], signals, checks, 1)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep

    if CP.carFingerprint in TSS2_CAR:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}  # {384: 0, 385: 0, ..., 399: 0}

    self.rcp = _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]  # = 415
    self.updated_messages = set()  # Python set() function creates a set object

    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make a adas dbc file for dsu-less models
    self.no_radar = CP.carFingerprint in NO_DSU_CAR and CP.carFingerprint not in TSS2_CAR

  def update(self, can_strings):
    if self.no_radar:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)  # Python set update() method updates the set

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
