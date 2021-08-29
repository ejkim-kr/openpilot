# flake8: noqa

from cereal import car
from selfdrive.car import dbc_dict
Ecu = car.CarParams.Ecu

class CarControllerParams:

  ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
  ACCEL_MAX = 1.5
  ACCEL_MIN = -4.8
  ACCEL_SCALE = 3.9  # max(ACCEL_MAX, -ACCEL_MIN)

  STEER_MAX = 384   # 409 is the max, 255 is stock
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 5
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1

class CAR:
  # genesis
  GENESIS = "GENESIS 2015-2016"
  GENESIS_G70 = "GENESIS G70 2018"
  GENESIS_G80 = "GENESIS G80 2017"
  GENESIS_EQ900 = "GENESIS EQ900 2017"
  GENESIS_EQ900_L = "GENESIS EQ900 LIMOUSINE"
  GENESIS_G90 = "GENESIS G90 2019"
  # hyundai
  ELANTRA = "HYUNDAI ELANTRA LIMITED ULTIMATE 2017"
  ELANTRA_2021 = "HYUNDAI ELANTRA 2021"
  ELANTRA_GT_I30 = "HYUNDAI I30 N LINE 2019 & GT 2018 DCT"
  SONATA = "HYUNDAI SONATA 2020"
  SONATA_HEV = "HYUNDAI SONATA HEV 2020"
  SONATA19 = "HYUNDAI SONATA 2019"
  SONATA19_HEV = "HYUNDAI SONATA 2019 HEV"
  SONATA_LF_TURBO = "HYUNDAI SONATA LF TURBO"
  KONA = "HYUNDAI KONA 2019"
  KONA_EV = "HYUNDAI KONA EV 2019"
  KONA_HEV = "HYUNDAI KONA HEV 2019"
  IONIQ = "HYUNDAI IONIQ HYBRID PREMIUM 2017"
  IONIQ_EV_LTD = "HYUNDAI IONIQ ELECTRIC LIMITED 2019"
  SANTA_FE = "HYUNDAI SANTA FE LIMITED 2019"
  PALISADE = "HYUNDAI PALISADE 2020"
  VELOSTER = "HYUNDAI VELOSTER 2019"
  GRANDEUR_IG = "HYUNDAI GRANDEUR IG 2017"
  GRANDEUR_IG_HEV = "HYUNDAI GRANDEUR IG HEV 2019"
  GRANDEUR_IG_FL = "HYUNDAI GRANDEUR IG FL 2020"
  GRANDEUR_IG_FL_HEV = "HYUNDAI GRANDEUR IG FL HEV 2020"
  TUCSON_TL_SCC  = "HYUNDAI TUCSON TL SCC 2017"
  # kia
  FORTE = "KIA FORTE E 2018"
  K5 = "KIA K5 2019 & 2016"
  K5_HEV = "KIA K5 HYBRID 2017 & SPORTS 2019"
  SPORTAGE = "KIA SPORTAGE S 2020"  
  SORENTO = "KIA SORENTO GT LINE 2018"
  STINGER = "KIA STINGER GT2 2018"
  NIRO_EV = "KIA NIRO EV 2020 PLATINUM"
  NIRO_HEV = "KIA NIRO HEV 2018"
  CEED = "KIA CEED 2019"
  SELTOS = "KIA SELTOS 2021"
  K7 = "KIA K7 2016-2019"
  K7_HEV = "KIA K7 HEV 2016-2019"
  K9 = "KIA K9 2016-2019"

# ex) CAR_FORCE_RECOGNITION = CAR.GRANDEUR_IG
CAR_FORCE_RECOGNITION = None

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4

FINGERPRINTS = {

  CAR.NIRO_EV: [{
    127: 8, 304: 8, 320: 8, 339: 8, 352: 8, 356: 4, 516: 8, 544: 8, 593: 8, 688: 5, 832: 8, 881: 8, 882: 8, 897: 8, 902: 8, 903: 8, 905: 8, 909: 8, 916: 8, 1040: 8, 1042: 8, 1056: 8, 1057: 8, 1078: 4, 1136: 8, 1151: 6, 1156: 8, 1157: 4, 1168: 7, 1173: 8, 1183: 8, 1186: 2, 1191: 2, 1193: 8, 1225: 8, 1260: 8, 1265: 4, 1280: 1, 1287: 4, 1290: 8, 1291: 8, 1292: 8, 1294: 8, 1312: 8, 1322: 8, 1342: 6, 1345: 8, 1348: 8, 1355: 8, 1363: 8, 1369: 8, 1407: 8, 1419: 8, 1426: 8, 1427: 6, 1429: 8, 1430: 8, 1456: 4, 1470: 8, 1473: 8, 1507: 8, 1535: 8, 1990: 8, 1998: 8
    },{
    127: 8, 304: 8, 320: 8, 339: 8, 352: 8, 356: 4, 544: 8, 593: 8, 688: 5, 832: 8, 881: 8, 882: 8, 897: 8, 902: 8, 903: 8, 905: 8, 909: 8, 916: 8, 1040: 8, 1042: 8, 1056: 8, 1057: 8, 1078: 4, 1136: 8, 1151: 6, 1157: 4, 1168: 7, 1173: 8, 1186: 2, 1191: 2, 1225: 8, 1265: 4, 1280: 1, 1287: 4, 1290: 8, 1291: 8, 1292: 8, 1294: 8, 1312: 8, 1322: 8, 1342: 6, 1345: 8, 1348: 8, 1355: 8, 1363: 8, 1369: 8, 1407: 8, 1419: 8, 1426: 8, 1427: 6, 1429: 8, 1430: 8, 1456: 4, 1470: 8, 1473: 8, 1507: 8, 1535: 8, 1988: 8, 1996: 8, 2000: 8, 2004: 8, 2008: 8, 2012: 8, 2015: 8
  }],

}

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [832, 1156, 1191, 1342]
}

# Don't use these fingerprints for fingerprinting, they are still used for ECU detection
IGNORED_FINGERPRINTS = [CAR.VELOSTER, CAR.GENESIS_G70, CAR.KONA, CAR.CEED, CAR.SELTOS]

FW_VERSIONS = {

  CAR.NIRO_EV: {
    (Ecu.fwdRadar, 0x7D0, None): [
      b'\xf1\x00DEev SCC F-CUP      1.00 1.00 99110-Q4000         ',
      b'\xf1\x00DEev SCC F-CUP      1.00 1.00 99110-Q4000         \xf1\xa01.00',
      b'\xf1\x00DEev SCC F-CUP      1.00 1.03 96400-Q4100         \xf1\xa01.03',
      b'\xf1\x00OSev SCC F-CUP      1.00 1.01 99110-K4000         \xf1\xa01.01',
      b'\xf1\x8799110Q4000\xf1\x00DEev SCC F-CUP      1.00 1.00 99110-Q4000         \xf1\xa01.00',
      b'\xf1\x8799110Q4100\xf1\x00DEev SCC F-CUP      1.00 1.00 99110-Q4100         \xf1\xa01.00',
      b'\xf1\x8799110Q4500\xf1\000DEev SCC F-CUP      1.00 1.00 99110-Q4500         \xf1\xa01.00',
    ],
    (Ecu.esp, 0x7D1, None): [
      b'\xf1\x00OS IEB \r 212 \x11\x13 58520-K4000\xf1\xa02.12',
      b'\xf1\x00OS IEB \r 212 \x11\x13 58520-K4000',
    ],
    (Ecu.eps, 0x7D4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4000\x00 4DEEC105',
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4100\x00 4DEEC105',
      b'\xf1\x00OS  MDPS C 1.00 1.04 56310K4050\x00 4OEDC104',
    ],
    (Ecu.fwdCamera, 0x7C4, None): [
      b'\xf1\000DEE MFC  AT EUR LHD 1.00 1.00 99211-Q4100 200706',
      b'\xf1\x00DEE MFC  AT EUR LHD 1.00 1.00 99211-Q4000 191211',
      b'\xf1\x00DEE MFC  AT USA LHD 1.00 1.00 99211-Q4000 191211',
      b'\xf1\x00DEE MFC  AT USA LHD 1.00 1.03 95740-Q4000 180821',
      b'\xf1\x00OSE LKAS AT EUR LHD 1.00 1.00 95740-K4100 W40',
    ],
  },

}

CHECKSUM = {
  "crc8": [CAR.SANTA_FE, CAR.SONATA, CAR.PALISADE, CAR.SONATA_HEV, CAR.SELTOS, CAR.ELANTRA_2021],
  "6B": [CAR.SORENTO, CAR.GENESIS],
}

FEATURES = {
  # Use Cluster for Gear Selection, rather than Transmission
  "use_cluster_gears": {CAR.ELANTRA, CAR.KONA, CAR.ELANTRA_GT_I30, CAR.K7, CAR.NIRO_HEV, CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_FL},

  # Use TCU Message for Gear Selection
  "use_tcu_gears": {CAR.K5, CAR.SONATA19, CAR.VELOSTER, CAR.SONATA_LF_TURBO, CAR.TUCSON_TL_SCC},

  # Use E_GEAR Message for Gear Selection
  "use_elect_gears": {CAR.K5_HEV, CAR.IONIQ_EV_LTD, CAR.KONA_EV, CAR.KONA_HEV, CAR.SONATA_HEV, CAR.NIRO_EV, CAR.K7_HEV,
                      CAR.GRANDEUR_IG_HEV, CAR.GRANDEUR_IG_FL_HEV},

  # Use E_EMS11 Message for Gas and Brake for Hybrid/ELectric
  "use_elect_ems": {CAR.K5_HEV, CAR.IONIQ_EV_LTD, CAR.KONA_EV, CAR.KONA_HEV, CAR.SONATA_HEV, CAR.NIRO_EV, CAR.K7_HEV,
                    CAR.GRANDEUR_IG_HEV, CAR.GRANDEUR_IG_FL_HEV},

  # send LFA MFA message for new HKG models  CAR.NIRO_EV, 
  "send_lfa_mfa": {CAR.SONATA, CAR.PALISADE, CAR.SONATA_HEV, CAR.SANTA_FE, CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV,
                   CAR.SELTOS, CAR.KONA_EV, CAR.KONA_HEV, CAR.TUCSON_TL_SCC, CAR.ELANTRA_2021, CAR.K9, CAR.GENESIS_G90},

  # these cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
  "use_fca": {CAR.SONATA, CAR.ELANTRA, CAR.ELANTRA_GT_I30, CAR.STINGER, CAR.IONIQ, CAR.KONA, CAR.KONA_EV, CAR.FORTE,
              CAR.PALISADE, CAR.GENESIS_G70, CAR.SANTA_FE, CAR.SELTOS, CAR.ELANTRA_2021, CAR.K9, CAR.GENESIS_G90},

  "use_blinker_flash": {CAR.SONATA_LF_TURBO},

  "has_scc13": {CAR.PALISADE, CAR.NIRO_HEV, CAR.K9, CAR.GENESIS_G90},
  "has_scc14": {CAR.PALISADE, CAR.NIRO_HEV, CAR.K9, CAR.GENESIS_G90},

  "use_ldws": False,
}

DBC = {
  # genesis
  CAR.GENESIS: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G70: dbc_dict('hyundai_kia_generic', None),  
  CAR.GENESIS_G80: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900_L: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G90: dbc_dict('hyundai_kia_generic', None),
  # hyundai
  CAR.ELANTRA: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_GT_I30: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA19: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA19_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA_LF_TURBO: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_EV: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_EV_LTD: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE: dbc_dict('hyundai_kia_generic', None),
  CAR.PALISADE: dbc_dict('hyundai_kia_generic', None),
  CAR.VELOSTER: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.TUCSON_TL_SCC: dbc_dict('hyundai_kia_generic', None),
  # kia
  CAR.FORTE: dbc_dict('hyundai_kia_generic', None),
  CAR.K5: dbc_dict('hyundai_kia_generic', None),
  CAR.K5_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SPORTAGE: dbc_dict('hyundai_kia_generic', None),  
  CAR.SORENTO: dbc_dict('hyundai_kia_generic', None),
  CAR.STINGER: dbc_dict('hyundai_kia_generic', None),  
  CAR.NIRO_EV: dbc_dict('hyundai_kia_generic', None),
  CAR.NIRO_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.CEED: dbc_dict('hyundai_kia_generic', None),
  CAR.SELTOS: dbc_dict('hyundai_kia_generic', None),
  CAR.K7: dbc_dict('hyundai_kia_generic', None),
  CAR.K7_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.K9: dbc_dict('hyundai_kia_generic', None),
}

STEER_THRESHOLD = 150

def main():
  for member, value in vars(CAR).items():
    if not member.startswith("_"):
      print(value)


if __name__ == "__main__":
  main()
